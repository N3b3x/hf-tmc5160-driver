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
static void espnow_command_task(void* arg);
static void motion_control_task(void* arg);
static void status_update_task(void* arg);

// Global state (forward declarations)
static tmc51x0::TMC51x0<Esp32SPI>* g_driver = nullptr;
static Settings g_settings{};
static QueueHandle_t g_espnowQueue = nullptr;
static bool g_bounds_found = false;
static bool g_use_stallguard = true;

// g_motion will be declared after FatigueTestMotion is defined

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

// Include full FatigueTestMotion implementation (after mutex classes are defined)
#include "fatigue_motion.hpp"
#include "fatigue_motion_impl.hpp"

// Now declare g_motion after FatigueTestMotion is fully defined
static FatigueTest::FatigueTestMotion* g_motion = nullptr;

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

    // Create motion controller (full implementation)
    FatigueTest::FatigueTestMotion motion(&driver);
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
