/**
 * @file main.cpp
 * @brief Fatigue test unit with ESP-NOW communication
 * 
 * This is the test unit (receiver) that:
 * 1. Receives commands from UI board via ESP-NOW
 * 2. Performs bounds finding (stallguard or encoder-based)
 * 3. Runs fatigue test with sinusoidal motion
 * 4. Sends status updates back to UI board
 * 
 * Supports both StallGuard2 and encoder-based bounds detection.
 */

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"

#include "../espnow_protocol.hpp"
#include "espnow_receiver.hpp"
#include "bounds_finder.hpp"
#include <memory>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>

static const char* TAG = "FatigueTestUnit";

// Test rig selection
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;

// Forward declarations
class FatigueTestMotion;
static void espnow_command_task(void* arg);
static void motion_control_task(void* arg);
static void status_update_task(void* arg);

// Global state
static tmc51x0::TMC51x0<Esp32SPI>* g_driver = nullptr;
static FatigueTestMotion* g_motion = nullptr;
static Settings g_settings{};
static QueueHandle_t g_espnowQueue = nullptr;
static bool g_bounds_found = false;
static bool g_use_stallguard = true;

// RAII Mutex classes (same as fatigue_test_encoder.cpp)
class Esp32TmcMutex {
public:
    Esp32TmcMutex() noexcept : handle_(xSemaphoreCreateMutex()) {
        if (handle_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create mutex");
        }
    }
    ~Esp32TmcMutex() noexcept {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
            handle_ = nullptr;
        }
    }
    Esp32TmcMutex(const Esp32TmcMutex&) = delete;
    Esp32TmcMutex& operator=(const Esp32TmcMutex&) = delete;
    SemaphoreHandle_t native_handle() const noexcept { return handle_; }
    bool is_valid() const noexcept { return handle_ != nullptr; }
private:
    SemaphoreHandle_t handle_;
};

class TmcMutexGuard {
public:
    explicit TmcMutexGuard(Esp32TmcMutex& mutex) noexcept : mutex_(mutex), locked_(false) {
        if (mutex_.is_valid()) {
            SemaphoreHandle_t handle = mutex_.native_handle();
            if (xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE) {
                locked_ = true;
            }
        }
    }
    ~TmcMutexGuard() noexcept { unlock(); }
    void unlock() noexcept {
        if (locked_ && mutex_.is_valid()) {
            SemaphoreHandle_t handle = mutex_.native_handle();
            xSemaphoreGive(handle);
            locked_ = false;
        }
    }
    bool is_locked() const noexcept { return locked_; }
private:
    Esp32TmcMutex& mutex_;
    bool locked_;
};

// Simplified FatigueTestMotion class (based on fatigue_test_encoder.cpp)
// For full implementation, see that file
class FatigueTestMotion {
private:
    tmc51x0::TMC51x0<Esp32SPI>* driver_;
    int32_t global_min_bound_, global_max_bound_;
    int32_t local_min_bound_, local_max_bound_;
    int32_t home_position_;
    float amplitude_, frequency_hz_;
    bool running_;
    uint32_t start_time_us_;
    bool bounded_;
    uint16_t steps_per_rev_;
    uint32_t dwell_at_min_ms_, dwell_at_max_ms_;
    uint32_t target_cycles_, current_cycles_;
    bool cycle_complete_;
    mutable Esp32TmcMutex mutex_;

public:
    FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver) noexcept
        : driver_(driver), global_min_bound_(0), global_max_bound_(0),
          local_min_bound_(0), local_max_bound_(0), home_position_(0),
          amplitude_(1000.0F), frequency_hz_(0.5F), running_(false),
          start_time_us_(0), bounded_(false), steps_per_rev_(200),
          dwell_at_min_ms_(0), dwell_at_max_ms_(0),
          target_cycles_(0), current_cycles_(0), cycle_complete_(false) {}

    void ConfigureMotor(uint16_t steps_per_rev) noexcept {
        TmcMutexGuard guard(mutex_);
        steps_per_rev_ = steps_per_rev;
    }

    void SetGlobalBounds(int32_t min_bound, int32_t max_bound) noexcept {
        TmcMutexGuard guard(mutex_);
        global_min_bound_ = min_bound;
        global_max_bound_ = max_bound;
        bounded_ = true;
    }

    bool SetLocalBoundsFromCenterDegrees(float min_degrees, float max_degrees) noexcept {
        if (steps_per_rev_ == 0) return false;
        int32_t min_steps = tmc51x0::DegreesToSteps(min_degrees, steps_per_rev_);
        int32_t max_steps = tmc51x0::DegreesToSteps(max_degrees, steps_per_rev_);
        TmcMutexGuard guard(mutex_);
        local_min_bound_ = min_steps;
        local_max_bound_ = max_steps;
        home_position_ = (local_min_bound_ + local_max_bound_) / 2;
        amplitude_ = static_cast<float>((local_max_bound_ - local_min_bound_) / 2);
        return true;
    }

    bool SetFrequency(float frequency_hz) noexcept {
        if (frequency_hz < 0.0F || frequency_hz > 10.0F) return false;
        TmcMutexGuard guard(mutex_);
        frequency_hz_ = frequency_hz;
        return true;
    }

    bool SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms) noexcept {
        TmcMutexGuard guard(mutex_);
        dwell_at_min_ms_ = dwell_at_min_ms;
        dwell_at_max_ms_ = dwell_at_max_ms;
        return true;
    }

    bool SetTargetCycles(uint32_t cycles) noexcept {
        TmcMutexGuard guard(mutex_);
        target_cycles_ = cycles;
        return true;
    }

    uint32_t GetCurrentCycles() const noexcept {
        TmcMutexGuard guard(mutex_);
        return current_cycles_;
    }

    bool Start() noexcept {
        TmcMutexGuard guard(mutex_);
        if (local_min_bound_ == 0 && local_max_bound_ == 0) return false;
        if (cycle_complete_) return false;
        running_ = true;
        start_time_us_ = esp_timer_get_time();
        cycle_complete_ = false;
        current_cycles_ = 0;
        return true;
    }

    void Stop() noexcept {
        TmcMutexGuard guard(mutex_);
        running_ = false;
        cycle_complete_ = false;
    }

    bool IsRunning() const noexcept {
        TmcMutexGuard guard(mutex_);
        return running_ && !cycle_complete_;
    }

    void Update() noexcept {
        // Simplified update - for full implementation see fatigue_test_encoder.cpp
        // This would handle sinusoidal motion, cycle counting, etc.
        if (!IsRunning()) return;
        
        uint64_t elapsed_us = esp_timer_get_time() - start_time_us_;
        double elapsed_s = elapsed_us / 1000000.0;
        
        float freq, amp;
        int32_t home, local_min, local_max;
        uint32_t target_cycles_val;
        {
            TmcMutexGuard guard(mutex_);
            freq = frequency_hz_;
            amp = amplitude_;
            home = home_position_;
            local_min = local_min_bound_;
            local_max = local_max_bound_;
            target_cycles_val = target_cycles_;
        }
        
        double angle = 2.0 * M_PI * freq * elapsed_s;
        double sin_value = sin(angle);
        int32_t target = home + static_cast<int32_t>(amp * sin_value);
        
        // Clamp to local bounds
        if (target < local_min) target = local_min;
        if (target > local_max) target = local_max;
        
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver_->rampControl.SetTargetPosition(static_cast<float>(target), tmc51x0::Unit::Steps);
        driver_->rampControl.SetMaxSpeed(1000.0f);
        driver_->rampControl.SetAcceleration(2000.0f);
        driver_->rampControl.SetDeceleration(2000.0f);
        
        // Cycle counting (simplified - count center crossings)
        static int32_t last_target_relative = 0;
        int32_t target_relative = target - home;
        bool currently_negative = (target_relative < 0);
        bool last_was_negative = (last_target_relative < 0);
        
        int32_t abs_target = (target_relative > 0) ? target_relative : -target_relative;
        int32_t abs_last = (last_target_relative > 0) ? last_target_relative : -last_target_relative;
        if (last_was_negative != currently_negative && abs_target < 30 && abs_last < 30) {
            TmcMutexGuard guard(mutex_);
            current_cycles_++;
            if (target_cycles_val > 0 && current_cycles_ >= target_cycles_val) {
                cycle_complete_ = true;
                running_ = false;
                driver_->rampControl.Stop();
            }
        }
        last_target_relative = target_relative;
    }

    struct Status {
        bool running;
        bool bounded;
        float frequency_hz;
        float min_degrees_from_center;
        float max_degrees_from_center;
        uint32_t current_cycles;
        uint32_t target_cycles;
        uint32_t dwell_min_ms;
        uint32_t dwell_max_ms;
    };

    Status GetStatus() const noexcept {
        Status status{};
        TmcMutexGuard guard(mutex_);
        status.running = running_ && !cycle_complete_;
        status.bounded = bounded_;
        status.frequency_hz = frequency_hz_;
        status.current_cycles = current_cycles_;
        status.target_cycles = target_cycles_;
        status.dwell_min_ms = dwell_at_min_ms_;
        status.dwell_max_ms = dwell_at_max_ms_;
        if (steps_per_rev_ > 0) {
            status.min_degrees_from_center = tmc51x0::StepsToDegrees(local_min_bound_, steps_per_rev_);
            status.max_degrees_from_center = tmc51x0::StepsToDegrees(local_max_bound_, steps_per_rev_);
        }
        return status;
    }
};

// ESP-NOW command handler task
static void espnow_command_task(void* arg)
{
    ProtoEvent ev{};
    while (true) {
        if (xQueueReceive(g_espnowQueue, &ev, portMAX_DELAY) == pdTRUE) {
            switch (ev.type) {
                case ProtoEventType::CONFIG_REQUEST:
                    ESP_LOGI(TAG, "Config request received");
                    EspNowReceiver::send_config_response(g_settings);
                    break;
                    
                case ProtoEventType::CONFIG_SET:
                    ESP_LOGI(TAG, "Config set: cycles=%u, tper=%u, dwell=%u, bounds=%s",
                             ev.data.config.cycle_amount,
                             ev.data.config.time_per_cycle,
                             ev.data.config.dwell_time,
                             ev.data.config.bounds_method_stallguard ? "StallGuard" : "Encoder");
                    g_settings = ev.data.config;
                    g_use_stallguard = ev.data.config.bounds_method_stallguard;
                    
                    if (g_motion) {
                        g_motion->SetTargetCycles(g_settings.cycle_amount);
                        g_motion->SetDwellTimes(g_settings.dwell_time * 1000, g_settings.dwell_time * 1000);
                        // Convert time_per_cycle to frequency
                        float freq = 1.0f / (float)g_settings.time_per_cycle;
                        g_motion->SetFrequency(freq);
                    }
                    EspNowReceiver::send_config_ack(true, 0);
                    break;
                    
                case ProtoEventType::START:
                    ESP_LOGI(TAG, "Start command received");
                    if (!g_bounds_found) {
                        ESP_LOGW(TAG, "Bounds not found yet, cannot start");
                        EspNowReceiver::send_start_ack();
                        EspNowReceiver::send_error(1, 0); // Error: bounds not found
                        break;
                    }
                    if (g_motion && g_motion->Start()) {
                        EspNowReceiver::send_start_ack();
                        EspNowReceiver::send_status_update(0, TestState::RUNNING);
                    } else {
                        EspNowReceiver::send_start_ack();
                        EspNowReceiver::send_error(2, 0); // Error: start failed
                    }
                    break;
                    
                case ProtoEventType::PAUSE:
                    ESP_LOGI(TAG, "Pause command received");
                    if (g_motion) {
                        g_motion->Stop();
                        EspNowReceiver::send_pause_ack();
                        EspNowReceiver::send_status_update(g_motion->GetCurrentCycles(), TestState::PAUSED);
                    }
                    break;
                    
                case ProtoEventType::RESUME:
                    ESP_LOGI(TAG, "Resume command received");
                    if (g_motion && g_motion->Start()) {
                        EspNowReceiver::send_resume_ack();
                        EspNowReceiver::send_status_update(g_motion->GetCurrentCycles(), TestState::RUNNING);
                    }
                    break;
                    
                case ProtoEventType::STOP:
                    ESP_LOGI(TAG, "Stop command received");
                    if (g_motion) {
                        uint32_t cycles = g_motion->GetCurrentCycles();
                        g_motion->Stop();
                        EspNowReceiver::send_stop_ack();
                        EspNowReceiver::send_status_update(cycles, TestState::IDLE);
                    }
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unhandled event type: %d", (int)ev.type);
                    break;
            }
        }
    }
}

// Motion control task
static void motion_control_task(void* arg)
{
    while (true) {
        if (g_motion) {
            g_motion->Update();
            
            // Check if test completed
            if (!g_motion->IsRunning()) {
                auto status = g_motion->GetStatus();
                if (status.current_cycles >= status.target_cycles && status.target_cycles > 0) {
                    EspNowReceiver::send_status_update(status.current_cycles, TestState::COMPLETED);
                    EspNowReceiver::send_test_complete();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms update rate
    }
}

// Status update task (sends periodic status to UI board)
static void status_update_task(void* arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
        
        if (g_motion && g_motion->IsRunning()) {
            auto status = g_motion->GetStatus();
            TestState state = status.running ? TestState::RUNNING : TestState::IDLE;
            EspNowReceiver::send_status_update(status.current_cycles, state);
        }
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         Fatigue Test Unit with ESP-NOW Communication                          ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    // Initialize ESP-NOW receiver
    g_espnowQueue = xQueueCreate(10, sizeof(ProtoEvent));
    if (!EspNowReceiver::init(g_espnowQueue)) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW receiver");
        return;
    }

    // Get default pin configuration
    auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
    tmc51x0::PinActiveLevels active_levels;
    
    // Create SPI communication interface
    Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, 1000000, active_levels);
    if (!spi.Initialize()) {
        ESP_LOGE(TAG, "Failed to initialize SPI interface");
        return;
    }

    // Create TMC51x0 driver instance
    tmc51x0::TMC51x0<Esp32SPI> driver(spi);
    g_driver = &driver;

    // Configure driver from test rig
    tmc51x0::DriverConfig cfg{};
    tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
    
    constexpr uint16_t output_full_steps = 
        tmc51x0_test_config::GetTestRigMotorOutputFullSteps<SELECTED_TEST_RIG>();
    
    if (!driver.Initialize(cfg)) {
        ESP_LOGE(TAG, "Failed to initialize TMC51x0 driver");
        return;
    }

    ESP_LOGI(TAG, "Driver initialized successfully");

    // Configure encoder
    tmc51x0::EncoderConfig enc_cfg = 
        tmc51x0_test_config::GetTestRigEncoderConfig<SELECTED_TEST_RIG>();
    
    if (!driver.encoder.Configure(enc_cfg)) {
        ESP_LOGE(TAG, "Failed to configure encoder");
        return;
    }

    driver.encoder.SetResolution(
        tmc51x0_test_config::GetTestRigEncoderPulsesPerRev<SELECTED_TEST_RIG>(),
        tmc51x0_test_config::GetTestRigEncoderInvertDirection<SELECTED_TEST_RIG>());

    // Enable motor
    if (!driver.motorControl.Enable()) {
        ESP_LOGE(TAG, "Failed to enable motor");
        return;
    }

    ESP_LOGI(TAG, "Motor enabled");

    // Calculate steps per revolution (with microsteps)
    float steps_per_rev_with_microsteps = static_cast<float>(output_full_steps) * 256.0f;
    uint16_t full_steps_per_rev = output_full_steps; // Full steps without microsteps

    // Create motion controller
    FatigueTestMotion motion(&driver);
    motion.ConfigureMotor(static_cast<uint16_t>(steps_per_rev_with_microsteps));
    g_motion = &motion;

    // Wait for config from UI board before finding bounds
    ESP_LOGI(TAG, "Waiting for configuration from UI board...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Give UI board time to send config

    // Find bounds using abstracted bounds finder
    ESP_LOGI(TAG, "Finding bounds using %s method...", g_use_stallguard ? "StallGuard2" : "Encoder");
    
    std::unique_ptr<FatigueTest::IBoundsFinder> bounds_finder;
    if (g_use_stallguard) {
        bounds_finder = FatigueTest::CreateStallGuardBoundsFinder();
    } else {
        bounds_finder = FatigueTest::CreateEncoderBoundsFinder();
    }

    if (bounds_finder) {
        // FindBounds expects full steps per rev (without microsteps)
        auto result = bounds_finder->FindBounds(driver, full_steps_per_rev);
        if (result.success) {
            motion.SetGlobalBounds(result.min_bound, result.max_bound);
            motion.SetLocalBoundsFromCenterDegrees(-60.0f, 60.0f);
            g_bounds_found = true;
            ESP_LOGI(TAG, "Bounds found using %s: min=%d, max=%d steps (bounded=%d)", 
                     bounds_finder->GetMethodName(), result.min_bound, result.max_bound, 
                     result.bounded ? 1 : 0);
        } else {
            ESP_LOGW(TAG, "Bounds finding failed, using default bounds");
            motion.SetGlobalBounds(-10000, 10000);
            motion.SetLocalBoundsFromCenterDegrees(-60.0f, 60.0f);
            g_bounds_found = false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to create bounds finder");
        motion.SetGlobalBounds(-10000, 10000);
        motion.SetLocalBoundsFromCenterDegrees(-60.0f, 60.0f);
        g_bounds_found = false;
    }

    // Create tasks
    xTaskCreate(espnow_command_task, "espnow_cmd", 4096, nullptr, 5, nullptr);
    xTaskCreate(motion_control_task, "motion_ctrl", 8192, nullptr, 5, nullptr);
    xTaskCreate(status_update_task, "status_upd", 4096, nullptr, 3, nullptr);

    ESP_LOGI(TAG, "System ready - waiting for commands from UI board");

    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
