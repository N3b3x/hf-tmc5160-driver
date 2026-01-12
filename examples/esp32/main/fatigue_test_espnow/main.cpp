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

// Standard library includes
 #include <memory>
 #include <algorithm>
 #include <cmath>
 #include <cstring>
 #include <string>
 #include <vector>
 #include <cstdarg>
 #include <new>
#include <cstdint>
#include <inttypes.h>

// FreeRTOS includes (must be before tmc51x0.hpp for ESP32 platform support)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ESP-IDF includes
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"

// TMC51x0 driver includes
#include "tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc_mutex.hpp"
#include "registers/tmc51x0_registers.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"

// Application includes
#include "espnow_protocol.hpp"
#include "espnow_receiver.hpp"
#include "fatigue_motion.hpp"
static const char* TAG = "FatigueTestUnit";

// Test rig selection
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;

/**
 * @brief Centralized timing constants for FreeRTOS tasks in this application.
 *
 * @details
 * These are intentionally `static constexpr` so:
 * - all tasks share a single source of truth
 * - the compiler can still fold constants
 *
 * Change values here to tune logging cadence and control-loop update rates.
 */
namespace TaskTiming {
    // Common periodic "I'm alive" log interval for tasks
    static constexpr int64_t LOG_INTERVAL_US = 5'000'000;        // 5 seconds
    static constexpr int64_t LOG_INTERVAL_LONG_US = 30'000'000;  // 30 seconds

    // Motion update loop timing
    static constexpr uint32_t MOTION_UPDATE_PERIOD_MS = 10; // 10ms update rate (~100 Hz)

    // StallGuard monitoring (only when StallGuard method is active)
    static constexpr int64_t SG_LOG_INTERVAL_US = 200'000;          // 200ms
    static constexpr int64_t SG_LOG_INTERVAL_ALWAYS_US = 1'000'000; // 1s

    // Status update cadence to remote controller
    static constexpr uint32_t STATUS_UPDATE_PERIOD_MS = 1'000; // 1 second

    // UART polling cadence
    static constexpr uint32_t UART_POLL_PERIOD_MS = 50; // 50ms
}

// Forward declarations
static void espnow_command_task(void* arg);
static void motion_control_task(void* arg);
static void status_update_task(void* arg);
static void bounds_finding_task(void* arg);

// Global state (forward declarations)
static tmc51x0::TMC51x0<Esp32SPI>* g_driver = nullptr;
static Esp32TmcMutex* g_driver_mutex = nullptr;  // Protects ALL SPI access to g_driver
static Settings g_settings{};
static QueueHandle_t g_espnowQueue = nullptr;

// -------------------- Lifetime-safe driver storage --------------------
// These are constructed via placement-new in app_main() after we have pin/config values.
// This avoids stack-lifetime issues if initialization fails and app_main() would otherwise return.
alignas(Esp32SPI) static uint8_t g_spi_storage[sizeof(Esp32SPI)];
alignas(tmc51x0::TMC51x0<Esp32SPI>) static uint8_t g_driver_storage[sizeof(tmc51x0::TMC51x0<Esp32SPI>)];
alignas(FatigueTest::FatigueTestMotion) static uint8_t g_motion_storage[sizeof(FatigueTest::FatigueTestMotion)];

static Esp32SPI* g_spi = nullptr;

// -------------------- Runtime control/state --------------------
enum class InternalState : uint8_t {
    IDLE = 0,
    BOUNDS_FINDING,
    RUNNING,
    PAUSED,
    ERROR
};

static volatile InternalState g_state = InternalState::IDLE;
static volatile bool g_cancel_bounds = false;
static volatile bool g_bounds_task_running = false;
static TaskHandle_t g_bounds_task_handle = nullptr;

static bool g_bounds_found = false;          // true if bounds have been found at least once
static bool g_use_stallguard = true;

// -------------------- Bounds Caching System --------------------
// After bounds finding completes, motor stays energized for a configurable time.
// If START is requested within this window, bounds finding is skipped.
// After timeout, motor is de-energized to prevent heating.

namespace BoundsCache {
    /// Default time window (in minutes) during which bounds remain valid
    static constexpr uint32_t DEFAULT_VALIDITY_MINUTES = 2;
    
    /// Time (microseconds, from esp_timer_get_time) when bounds were last found
    static volatile int64_t g_bounds_timestamp_us = 0;
    
    /// Bounds validity window in microseconds
    static volatile int64_t g_bounds_validity_us = DEFAULT_VALIDITY_MINUTES * 60 * 1000000LL;
    
    /// True if motor is currently energized from bounds finding
    static volatile bool g_motor_energized_for_bounds = false;
    
    /// Timer handle for de-energize timeout
    static esp_timer_handle_t g_deenergize_timer = nullptr;
    
    /**
     * @brief Check if bounds are still valid (within time window and motor energized).
     * @return true if we can skip bounds finding
     */
    inline bool AreBoundsValid() noexcept {
        if (!g_bounds_found || !g_motor_energized_for_bounds) {
            return false;
        }
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - g_bounds_timestamp_us;
        return elapsed < g_bounds_validity_us;
    }
    
    /**
     * @brief Get remaining time until bounds expire (seconds).
     * @return Seconds remaining, or 0 if expired
     */
    inline uint32_t GetRemainingValiditySec() noexcept {
        if (!g_bounds_found || !g_motor_energized_for_bounds) {
            return 0;
        }
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - g_bounds_timestamp_us;
        int64_t remaining = g_bounds_validity_us - elapsed;
        return remaining > 0 ? static_cast<uint32_t>(remaining / 1000000LL) : 0;
    }
    
    /**
     * @brief Mark bounds as freshly found and start de-energize timer.
     */
    void MarkBoundsFound() noexcept;
    
    /**
     * @brief Cancel de-energize timer (e.g., when test starts).
     */
    void CancelDeenergizeTimer() noexcept;
    
    /**
     * @brief Invalidate bounds (e.g., on config change or explicit request).
     */
    void InvalidateBounds() noexcept;
    
    /**
     * @brief Set the validity window in minutes.
     */
    void SetValidityMinutes(uint32_t minutes) noexcept;
    
    /**
     * @brief Initialize the bounds cache system (create timer).
     */
    void Init() noexcept;
    
} // namespace BoundsCache

// Global motion instance (constructed via placement-new in app_main()).
static FatigueTest::FatigueTestMotion* g_motion = nullptr;

static inline uint8_t GetBoundsValidFlag_() noexcept
{
    return BoundsCache::AreBoundsValid() ? 1 : 0;
}

static inline void ApplyMotionConfigFromSettings_() noexcept
{
    if (!g_motion) {
        return;
    }
    g_motion->SetTargetCycles(g_settings.test_unit.cycle_amount);
    g_motion->SetMaxVelocity(g_settings.test_unit.oscillation_vmax_rpm);
    g_motion->SetAcceleration(g_settings.test_unit.oscillation_amax_rev_s2);
    g_motion->SetDwellTimes(g_settings.test_unit.dwell_time_ms, g_settings.test_unit.dwell_time_ms);
}

enum class PendingStartKind : uint8_t { NONE = 0, START, RESUME };
static volatile PendingStartKind g_pending_start = PendingStartKind::NONE;

// Cancel callback for library homing/bounds routines.
// (Signature must be a free function pointer; it reads our global cancel flag.)
static bool ShouldCancelBounds() { return g_cancel_bounds; }

/**
 * @brief Map the internal application state machine to the protocol-visible `TestState`.
 *
 * @details
 * The on-wire protocol only supports a limited set of states. Internally we track
 * additional phases (notably `BOUNDS_FINDING`). While bounds finding is active
 * the motor is energized and moving, so we currently report `TestState::Running`.
 *
 * @param s Internal state value.
 * @return Protocol state value suitable for `STATUS_UPDATE`.
 *
 * @note If you later extend the protocol to include a dedicated "BOUNDS_FINDING"
 * state, update this mapping accordingly.
 */
static inline TestState ToProtoState(InternalState s) noexcept {
    switch (s) {
        case InternalState::IDLE: return TestState::Idle;
        case InternalState::BOUNDS_FINDING: return TestState::Running; // motor is active
        case InternalState::RUNNING: return TestState::Running;
        case InternalState::PAUSED: return TestState::Paused;
        case InternalState::ERROR: return TestState::Error;
        default: return TestState::Error;
    }
}

/**
 * @brief Immediately stop motion and enter HOLD mode, but keep motor energized.
 *
 * @details
 * This is used by PAUSE to stop motion while still delivering hold current.
 * The motor remains energized and will hold position.
 *
 * @note Thread-safe: acquires g_driver_mutex internally.
 */
static void MotorStopHold() noexcept {
    if (!g_driver || !g_driver_mutex) return;
    TmcMutexGuard guard(*g_driver_mutex);
    (void)g_driver->rampControl.Stop();
    (void)g_driver->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    // Motor stays enabled - delivering hold current
}

/**
 * @brief Immediately stop motion, enter HOLD, and de-energize (disable) the motor.
 *
 * @details
 * This is the hard-stop safety primitive used by:
 * - STOP command handling (always)
 * - cancellation of bounds finding
 * - completion of a finite-cycle run
 * - bounds cache timeout (de-energize to prevent heating)
 *
 * It is safe to call multiple times.
 *
 * @warning This disables the power stage. After this, the axis may be back-drivable.
 * @note This function does *not* change `g_state`; the caller owns state transitions.
 * @note Thread-safe: acquires g_driver_mutex internally.
 */
static void MotorStopHoldDisable() noexcept {
    if (!g_driver || !g_driver_mutex) return;
    // If we're disabling the motor, cached-bounds "motor energized" must not remain true.
    BoundsCache::g_motor_energized_for_bounds = false;
    TmcMutexGuard guard(*g_driver_mutex);
    (void)g_driver->rampControl.Stop();
    (void)g_driver->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    (void)g_driver->motorControl.Disable();
}

static void FatalInitError(const char* what, uint8_t err_code) noexcept {
    ESP_LOGE(TAG, "FATAL init error: %s", what);
    g_state = InternalState::ERROR;
    // Best effort: tell remote if ESP-NOW is up.
    (void)EspNowReceiver::send_error(err_code, 0);
    // Best effort: ensure motor is de-energized.
    MotorStopHoldDisable();
    // Do not return from app_main; keep system alive for logs/remote visibility.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Enable (energize) the motor power stage.
 *
 * @details
 * Per requirements, the motor must remain disabled at boot and only be enabled
 * during active operations (bounds finding and active motion).
 *
 * @return true if the motor was enabled successfully; false otherwise.
 *
 * @note This function does *not* start motion; it only energizes the driver.
 * @note Thread-safe: acquires g_driver_mutex internally.
 */
static bool MotorEnable() noexcept {
    if (!g_driver || !g_driver_mutex) return false;
    TmcMutexGuard guard(*g_driver_mutex);
    auto res = g_driver->motorControl.Enable();
    if (!res) {
        ESP_LOGE(TAG, "Failed to enable motor (ErrorCode: %d)", static_cast<int>(res.Error()));
        return false;
    }
    return true;
}

// -------------------- BoundsCache Implementation --------------------

/**
 * @brief Timer callback to de-energize motor after bounds validity expires.
 */
static void DeenergizeTimerCallback(void* arg) {
    (void)arg;
    // Safety: never de-energize due to bounds-cache timeout while the test is active.
    // This callback runs in the ESP timer task and can fire even if application state
    // has advanced since bounds were found.
    if (g_state != InternalState::IDLE) {
        ESP_LOGW(TAG,
                 "[BoundsCache] Validity expired but state=%d; skipping auto de-energize",
                 static_cast<int>(g_state));
        BoundsCache::g_motor_energized_for_bounds = false;
        return;
    }
    ESP_LOGI(TAG, "[BoundsCache] Validity window expired - de-energizing motor to prevent heating");
    BoundsCache::g_motor_energized_for_bounds = false;
    MotorStopHoldDisable();
}

void BoundsCache::Init() noexcept {
    if (g_deenergize_timer != nullptr) {
        return; // Already initialized
    }
    
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = DeenergizeTimerCallback;
    timer_args.arg = nullptr;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "bounds_deenergize";
    
    esp_err_t err = esp_timer_create(&timer_args, &g_deenergize_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[BoundsCache] Failed to create de-energize timer: %s", esp_err_to_name(err));
    }
}

void BoundsCache::MarkBoundsFound() noexcept {
    g_bounds_timestamp_us = esp_timer_get_time();
    g_motor_energized_for_bounds = true;
    
    // Cancel any existing timer
    if (g_deenergize_timer != nullptr) {
        esp_timer_stop(g_deenergize_timer);
    }
    
    // Start new timer
    if (g_deenergize_timer != nullptr) {
        esp_err_t err = esp_timer_start_once(g_deenergize_timer, g_bounds_validity_us);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[BoundsCache] Failed to start de-energize timer: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "[BoundsCache] Bounds valid for %d seconds - motor will de-energize after timeout",
                     static_cast<int>(g_bounds_validity_us / 1000000LL));
        }
    }
}

void BoundsCache::CancelDeenergizeTimer() noexcept {
    if (g_deenergize_timer != nullptr) {
        esp_timer_stop(g_deenergize_timer);
    }
    // Bounds-cache energize is only intended for the "ready to start" window after FIND_ONLY.
    // Once we commit to starting the test, clear this so future START/RESUME won't incorrectly
    // skip bounds finding after STOP or other motor-disable events.
    g_motor_energized_for_bounds = false;
}

void BoundsCache::InvalidateBounds() noexcept {
    g_bounds_found = false;
    g_bounds_timestamp_us = 0;
    g_motor_energized_for_bounds = false;
    
    if (g_deenergize_timer != nullptr) {
        esp_timer_stop(g_deenergize_timer);
    }
    ESP_LOGI(TAG, "[BoundsCache] Bounds invalidated");
}

void BoundsCache::SetValidityMinutes(uint32_t minutes) noexcept {
    g_bounds_validity_us = static_cast<int64_t>(minutes) * 60 * 1000000LL;
    ESP_LOGI(TAG, "[BoundsCache] Validity window set to %lu minutes", (unsigned long)minutes);
}

// -------------------- End BoundsCache Implementation --------------------

/**
 * @brief Pretty-print motor-current calculated and cached register values under the FatigueTestUnit tag.
 *
 * @details
 * The driver already logs these values under the "TMC5160" tag during Initialize().
 * This function prints the same information again under the application tag so
 * field logs are easier to scan without mixing driver/component tags.
 *
 * Values are sourced from the driver’s internal calculated fields and write-only
 * register cache via `TMC51x0::GetMotorCurrentDebugInfo()` (no duplicated math).
 *
 * @param driver Initialized TMC51x0 driver instance.
 */

/**
 * @brief Begin a START/RESUME sequence by launching cancellable bounds finding.
 *
 * @details
 * This function transitions the application into `InternalState::BOUNDS_FINDING`,
 * clears any prior bounds, and starts `bounds_finding_task` in its own FreeRTOS task.
 *
 * Bounds are always re-found (even if previously found) to handle the case where
 * mechanical limits may have moved between runs.
 *
 * @param kind Whether this request originated from START or RESUME.
 *
 * @note If bounds finding is already running, this call is ignored.
 * @note Task creation failure transitions to `ERROR`, stops and disables the motor,
 * and sends an error frame.
 */
/**
 * @brief Mode for bounds finding task.
 */
enum class BoundsTaskMode : uint8_t {
    FIND_AND_START,   ///< Find bounds then start test (normal START flow)
    FIND_ONLY         ///< Find bounds only, keep motor energized with timeout
};

static volatile BoundsTaskMode g_bounds_task_mode = BoundsTaskMode::FIND_AND_START;

/**
 * @brief Request a START/RESUME with optional bounds caching.
 * 
 * @details
 * If bounds were recently found (within validity window) and motor is still
 * energized, bounds finding is skipped and motion starts immediately.
 * Otherwise, bounds finding runs first.
 * 
 * @param kind START or RESUME request type
 */
static void RequestStart(PendingStartKind kind) noexcept {
    if (g_bounds_task_running) {
        ESP_LOGW(TAG, "Start requested but bounds task already running; ignoring");
        return;
    }
    
    // Check if we can use cached bounds
    if (BoundsCache::AreBoundsValid() && g_motion) {
        ESP_LOGI(TAG, "Using cached bounds (valid for %lu more seconds)",
                 (unsigned long)BoundsCache::GetRemainingValiditySec());
        
        // Cancel the de-energize timer since we're starting the test
        BoundsCache::CancelDeenergizeTimer();
        
        // SAFETY: Ensure StallGuard stop-on-stall is disabled and StealthChop is enabled
        // These should already be set from bounds finding, but verify for safety
        if (g_driver && g_driver_mutex) {
            TmcMutexGuard guard(*g_driver_mutex);
            (void)g_driver->stallGuard.EnableStopOnStall(false);
            (void)g_driver->motorControl.SetStealthChopEnabled(true);
            (void)g_driver->thresholds.SetModeChangeSpeeds(0.0f, 0.0f, 0.0f, tmc51x0::Unit::RPM);
        }
        
        // CRITICAL: On START (new test), reset cycle count to zero.
        // On RESUME, keep existing cycle count and continue from where we left off.
        if (kind == PendingStartKind::START) {
            ESP_LOGI(TAG, "START with cached bounds: Resetting cycle count to zero");
            g_motion->ResetCycles();
        } else {
            ESP_LOGI(TAG, "RESUME with cached bounds: Continuing from cycle %lu", 
                     (unsigned long)g_motion->GetCurrentCycles());
        }

        // Ensure latest config is applied before starting motion.
        ApplyMotionConfigFromSettings_();
        
        // Start motion directly without bounds finding
        if (!g_motion->Start()) {
            ESP_LOGE(TAG, "Motion start failed with cached bounds");
            g_state = InternalState::ERROR;
            MotorStopHoldDisable();
            EspNowReceiver::send_error(2, 0);
            return;
        }
        
        g_state = InternalState::RUNNING;
        EspNowReceiver::send_status_update(g_motion->GetCurrentCycles(), TestState::Running, 0, GetBoundsValidFlag_());
        return;
    }
    
    // Need to find bounds first
    ESP_LOGI(TAG, "Bounds not cached or expired - running bounds finding");
    g_cancel_bounds = false;
    g_bounds_found = false;
    g_pending_start = kind;
    g_bounds_task_mode = BoundsTaskMode::FIND_AND_START;
    g_state = InternalState::BOUNDS_FINDING;
    g_bounds_task_running = true;
    BaseType_t ok = xTaskCreate(bounds_finding_task, "bounds_find", 8192, nullptr, 5, &g_bounds_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bounds finding task");
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        g_state = InternalState::ERROR;
        MotorStopHoldDisable();
        EspNowReceiver::send_error(2, 0);
    }
}

/**
 * @brief Run bounds finding independently (without starting test).
 * 
 * @details
 * After bounds are found, motor stays energized for the configured validity
 * period. If START is requested within this window, bounds finding is skipped.
 * 
 * @return true if bounds finding was started, false if already running or error
 */
static bool RequestBoundsOnly() noexcept {
    if (g_bounds_task_running) {
        ESP_LOGW(TAG, "Bounds finding already running");
        return false;
    }
    
    if (g_state == InternalState::RUNNING) {
        ESP_LOGW(TAG, "Cannot run bounds finding while test is running");
        return false;
    }
    
    g_cancel_bounds = false;
    g_pending_start = PendingStartKind::NONE;
    g_bounds_task_mode = BoundsTaskMode::FIND_ONLY;
    g_state = InternalState::BOUNDS_FINDING;
    g_bounds_task_running = true;
    BaseType_t ok = xTaskCreate(bounds_finding_task, "bounds_find", 8192, nullptr, 5, &g_bounds_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bounds finding task");
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        g_state = InternalState::IDLE;
        MotorStopHoldDisable();
        return false;
    }
    return true;
}

/**
 * @brief FreeRTOS task: process inbound ESP-NOW protocol events.
 *
 * @details
 * Events arrive via `g_espnowQueue` from `EspNowReceiver`. This task:
 * - applies CONFIG_SET updates to `g_settings` and `g_motion`
 * - launches bounds-finding (START/RESUME)
 * - cancels bounds finding and/or stops motion (PAUSE/STOP)
 *
 * @param arg Unused (FreeRTOS task signature).
 *
 * @note This task must never block for long periods. Bounds finding is therefore
 * executed in a separate task (`bounds_finding_task`) so PAUSE/STOP remain responsive.
 */
static void espnow_command_task(void* arg)
{
    const char* task_name = "espnow_cmd";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = TaskTiming::LOG_INTERVAL_LONG_US;
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    
    ProtoEvent ev{};
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        // Use timeout so we can check time even when no messages arrive
        if (xQueueReceive(g_espnowQueue, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (ev.type) {
                case ProtoEventType::ConfigRequest:
                    ESP_LOGI(TAG, "Config request received");
                    EspNowReceiver::send_config_response(g_settings);
                    break;
                    
                case ProtoEventType::ConfigSet: {
                    // Get test config defaults for fallback when remote sends 0.0f
                    using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<SELECTED_TEST_RIG>;
                    
                    ESP_LOGI(TAG, "Config set: cycles=%u, vmax=%.1f RPM, amax=%.1f rev/s², dwell=%lu ms, bounds=%s",
                             ev.data.config.cycle_amount,
                             ev.data.config.oscillation_vmax_rpm,
                             ev.data.config.oscillation_amax_rev_s2,
                             (unsigned long)ev.data.config.dwell_time_ms,
                             ev.data.config.bounds_method_stallguard ? "StallGuard" : "Encoder");
                    
                    // Base fields (always set) - direct velocity/acceleration control
                    g_settings.test_unit.cycle_amount = ev.data.config.cycle_amount;
                    g_settings.test_unit.oscillation_vmax_rpm = ev.data.config.oscillation_vmax_rpm;
                    g_settings.test_unit.oscillation_amax_rev_s2 = ev.data.config.oscillation_amax_rev_s2;
                    g_settings.test_unit.dwell_time_ms = ev.data.config.dwell_time_ms;
                    g_settings.test_unit.bounds_method_stallguard = ev.data.config.bounds_method_stallguard;
                    
                    // Extended fields for bounds finding: 0.0f means "use test config defaults"
                    g_settings.test_unit.bounds_search_velocity_rpm = 
                        (ev.data.config.bounds_search_velocity_rpm > 0.01f) 
                            ? ev.data.config.bounds_search_velocity_rpm 
                            : TestConfig::Motion::BOUNDS_SEARCH_SPEED_RPM;
                    g_settings.test_unit.stallguard_min_velocity_rpm = 
                        (ev.data.config.stallguard_min_velocity_rpm > 0.01f) 
                            ? ev.data.config.stallguard_min_velocity_rpm 
                            : TestConfig::StallGuard::MIN_VELOCITY_RPM;
                    g_settings.test_unit.stall_detection_current_factor = 
                        (ev.data.config.stall_detection_current_factor > 0.01f) 
                            ? ev.data.config.stall_detection_current_factor 
                            : TestConfig::StallGuard::STALL_DETECTION_CURRENT_FACTOR;
                    g_settings.test_unit.bounds_search_accel_rev_s2 = 
                        (ev.data.config.bounds_search_accel_rev_s2 > 0.01f) 
                            ? ev.data.config.bounds_search_accel_rev_s2 
                            : TestConfig::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;

                    // SGT: valid range [-64, 63]. 127 means "use default".
                    if (ev.data.config.stallguard_sgt >= -64 && ev.data.config.stallguard_sgt <= 63) {
                        g_settings.test_unit.stallguard_sgt = ev.data.config.stallguard_sgt;
                    } else {
                        g_settings.test_unit.stallguard_sgt = TestConfig::StallGuard::SGT_HOMING;
                    }
                    
                    ESP_LOGI(TAG, "Motion config: VMAX=%.1f RPM, AMAX=%.2f rev/s², dwell=%lu ms",
                             g_settings.test_unit.oscillation_vmax_rpm,
                             g_settings.test_unit.oscillation_amax_rev_s2,
                             (unsigned long)g_settings.test_unit.dwell_time_ms);
                    ESP_LOGI(TAG, "Bounds config: search_vel=%.1f, sg_min=%.1f, cur_factor=%.2f, SGT=%d",
                             g_settings.test_unit.bounds_search_velocity_rpm,
                             g_settings.test_unit.stallguard_min_velocity_rpm,
                             g_settings.test_unit.stall_detection_current_factor,
                             static_cast<int>(g_settings.test_unit.stallguard_sgt));
                    
                    g_use_stallguard = ev.data.config.bounds_method_stallguard;
                    // Any config change forces bounds re-find on next START/RESUME
                    BoundsCache::InvalidateBounds();
                    
                    if (g_motion) {
                        g_motion->SetTargetCycles(g_settings.test_unit.cycle_amount);
                        g_motion->SetMaxVelocity(g_settings.test_unit.oscillation_vmax_rpm);
                        g_motion->SetAcceleration(g_settings.test_unit.oscillation_amax_rev_s2);
                        g_motion->SetDwellTimes(g_settings.test_unit.dwell_time_ms, g_settings.test_unit.dwell_time_ms);
                    }
                    EspNowReceiver::send_config_ack(true, 0);
                    break;
                }
                    
                case ProtoEventType::CommandStart:
                    ESP_LOGI(TAG, "Start command received");
                    if (g_state == InternalState::RUNNING || g_state == InternalState::BOUNDS_FINDING) {
                        ESP_LOGW(TAG, "Start ignored: already running or finding bounds");
                        EspNowReceiver::send_start_ack();
                        break;
                    }
                    EspNowReceiver::send_start_ack(); // acknowledge receipt immediately
                    RequestStart(PendingStartKind::START);
                    EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, ToProtoState(g_state), 0, GetBoundsValidFlag_());
                    break;
                    
                case ProtoEventType::CommandPause:
                    ESP_LOGI(TAG, "Pause command received");
                    EspNowReceiver::send_pause_ack();
                    // If bounds-finding is active, cancel it and disable motor (can't pause mid-bounds-finding).
                    if (g_state == InternalState::BOUNDS_FINDING) {
                        g_cancel_bounds = true;
                        g_state = InternalState::PAUSED;
                        MotorStopHoldDisable();  // Disable during bounds-finding cancellation
                        BoundsCache::InvalidateBounds();  // Must re-find bounds on resume
                        EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, TestState::Paused, 0, GetBoundsValidFlag_());
                        break;
                    }
                    // If running, pause motion but KEEP motor energized (still delivering hold current).
                    // Resume will continue from current position without re-finding bounds.
                    if (g_motion) {
                        g_motion->Pause();  // Use Pause() instead of Stop() - keeps running_=true for resume
                    } else {
                        // If no motion object, still stop the driver
                        MotorStopHold();
                    }
                    g_state = InternalState::PAUSED;
                    ESP_LOGI(TAG, "Motor paused - holding position with hold current");
                    EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, TestState::Paused, 0, GetBoundsValidFlag_());
                    break;
                    
                case ProtoEventType::CommandResume:
                    ESP_LOGI(TAG, "Resume command received");
                    EspNowReceiver::send_resume_ack();
                    if (g_state == InternalState::RUNNING || g_state == InternalState::BOUNDS_FINDING) {
                        ESP_LOGW(TAG, "Resume ignored: already running or finding bounds");
                        break;
                    }
                    
                    // If resuming from PAUSED state, motor is still energized and bounds are still valid
                    // Just resume motion directly without re-finding bounds
                    if (g_state == InternalState::PAUSED && g_motion) {
                        ESP_LOGI(TAG, "Resuming from PAUSED - motor still energized, bounds still valid");
                        
                        // Ensure StallGuard stop-on-stall is disabled (should already be set)
                        if (g_driver && g_driver_mutex) {
                            TmcMutexGuard guard(*g_driver_mutex);
                            (void)g_driver->stallGuard.EnableStopOnStall(false);
                            (void)g_driver->motorControl.SetStealthChopEnabled(true);
                            (void)g_driver->thresholds.SetModeChangeSpeeds(0.0f, 0.0f, 0.0f, tmc51x0::Unit::RPM);
                        }
                        
                        // Cancel de-energize timer since we're resuming
                        BoundsCache::CancelDeenergizeTimer();
                        
                        // Resume motion directly
                        if (!g_motion->Start()) {
                            ESP_LOGE(TAG, "Motion resume failed");
                            g_state = InternalState::ERROR;
                            MotorStopHoldDisable();
                            EspNowReceiver::send_error(2, 0);
                            break;
                        }
                        
                        g_state = InternalState::RUNNING;
                        EspNowReceiver::send_status_update(g_motion->GetCurrentCycles(), TestState::Running, 0, GetBoundsValidFlag_());
                    } else {
                        // Resuming from IDLE or other state - may need to re-find bounds
                        // Use RequestStart which will check bounds cache
                        RequestStart(PendingStartKind::RESUME);
                        EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, ToProtoState(g_state), 0, GetBoundsValidFlag_());
                    }
                    break;
                    
                case ProtoEventType::CommandStop:
                    ESP_LOGI(TAG, "Stop command received");
                    EspNowReceiver::send_stop_ack();
                    // Stop should stop everything and de-energize the motor immediately.
                    g_cancel_bounds = true;
                    if (g_motion) {
                        g_motion->Stop();
                    }
                    g_state = InternalState::IDLE;
                    MotorStopHoldDisable();
                    EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, TestState::Idle, 0, GetBoundsValidFlag_());
                    break;

                case ProtoEventType::CommandRunBoundsFinding:
                    ESP_LOGI(TAG, "RunBoundsFinding command received");
                    // Always ACK receipt immediately; UI gates on ACK.
                    EspNowReceiver::send_start_ack();
                    if (!RequestBoundsOnly()) {
                        ESP_LOGW(TAG, "Bounds-only request rejected (busy or running)");
                        // Keep state unchanged; send a status update so UI can reflect reality.
                        EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, ToProtoState(g_state), 0, GetBoundsValidFlag_());
                        break;
                    }
                    // Bounds finding is active (InternalState::BOUNDS_FINDING maps to TestState::Running).
                    EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, ToProtoState(g_state), 0, GetBoundsValidFlag_());
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unhandled event type: %d", (int)ev.type);
                    break;
            }
        }
    }
}

/**
 * @brief FreeRTOS task: perform bounds finding, then start motion.
 *
 * @details
 * This task is launched by `RequestStart()` and exists specifically so that PAUSE/STOP
 * can cancel bounds finding while it is in progress (the bounds finders poll the
 * `g_cancel_bounds` flag).
 *
 * Sequence:
 * - snapshot settings (`g_settings.test_unit`) and method (`g_use_stallguard`)
 * - enable motor power stage
 * - run the library bounds finder (`g_driver->homing.FindBounds(..., cancel_cb)`)
 * - if successful, apply bounds to `g_motion` and start motion
 * - update state and send a STATUS_UPDATE
 *
 * Cancellation rules:
 * - If `g_cancel_bounds` becomes true at any time, the task stops motion, disables
 *   the motor, and exits. A later RESUME will re-run bounds finding from scratch.
 *
 * @param arg Unused (FreeRTOS task signature).
 *
 * @warning This task directly controls motor motion during bounds finding; ensure
 * no other code drives the axis concurrently.
 */
static void bounds_finding_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "[bounds_find] ==========================================");
    ESP_LOGI(TAG, "[bounds_find] Starting Bounds Finding...");
    ESP_LOGI(TAG, "[bounds_find] ==========================================");

    // Snapshot settings at start of bounds finding to avoid concurrent edits mid-run
    const bool use_sg = g_use_stallguard;
    const TestUnitSettings tu = g_settings.test_unit;
    const PendingStartKind start_kind = g_pending_start;
    
    // Log configuration for debugging
    ESP_LOGI(TAG, "[bounds_find] Configuration:");
    ESP_LOGI(TAG, "[bounds_find]   Search Order: MAX (+) first");
    ESP_LOGI(TAG, "[bounds_find]   Search Speed: %.1f RPM", tu.bounds_search_velocity_rpm);
    ESP_LOGI(TAG, "[bounds_find]   Search Accel: %.1f rev/s²", tu.bounds_search_accel_rev_s2);
    ESP_LOGI(TAG, "[bounds_find]   Method: %s", use_sg ? "StallGuard" : "Encoder");
    if (use_sg) {
        ESP_LOGI(TAG, "[bounds_find]   SG Min Velocity: %.1f RPM", tu.stallguard_min_velocity_rpm);
        ESP_LOGI(TAG, "[bounds_find]   Current Reduction: %.0f%%", tu.stall_detection_current_factor * 100.0f);
    }

    if (!MotorEnable()) {
        g_state = InternalState::ERROR;
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        EspNowReceiver::send_error(2, 0);
        vTaskDelete(nullptr);
        return;
    }

    if (!g_driver || !g_motion) {
        ESP_LOGE(TAG, "[bounds_find] Missing components (g_driver=%p g_motion=%p)", g_driver, g_motion);
        g_state = InternalState::ERROR;
        MotorStopHoldDisable();
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        EspNowReceiver::send_error(4, 0);
        vTaskDelete(nullptr);
        return;
    }

    // Use the library built-in homing/bounds implementation.
    using DriverT = tmc51x0::TMC51x0<Esp32SPI>;
    using Homing = DriverT::Homing;
    using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<SELECTED_TEST_RIG>;

    Homing::BoundsOptions opt{};
    opt.speed_unit = tmc51x0::Unit::RPM;
    opt.position_unit = tmc51x0::Unit::Deg;
    opt.search_speed = tu.bounds_search_velocity_rpm;      // RPM
    opt.search_span = 400.0f;                              // degrees per direction (generous cap)
    opt.backoff_distance = 0.0f;                           // NO physical backoff (library applies numerically)
    opt.timeout_ms = TestConfig::Motion::HOMING_TIMEOUT_MS;
    opt.search_positive_first = true;                      // Find MAX first, then MIN (matches bounds_finding_test)

    // Preserve the UI knobs:
    // - StallGuard min velocity/current reduction apply only to SG mode
    // - Search acceleration applies to all modes (library uses it when non-zero)
    opt.search_accel = tu.bounds_search_accel_rev_s2;
    opt.search_decel = tu.bounds_search_accel_rev_s2;
    opt.accel_unit = tmc51x0::Unit::RevPerSec; // rev/s² (library treats Unit::RevPerSec as "rev-per-sec^2" for accel fields)

    Homing::HomeConfig home{};
    home.mode = Homing::HomePlacement::AtCenter; // center becomes XACTUAL=0 when both sides found

    Homing::BoundsMethod method = use_sg ? Homing::BoundsMethod::StallGuard : Homing::BoundsMethod::Encoder;

    // StallGuard override: we want runtime control over min velocity (and we keep the homing SGT).
    // IMPORTANT: Always enable filter for bounds finding to reduce false stalls during
    // acceleration/deceleration. The filter averages SG readings over 4 samples.
    tmc51x0::StallGuardConfig sg_override{};
    if (use_sg) {
        const int8_t sgt = (tu.stallguard_sgt >= -64 && tu.stallguard_sgt <= 63)
            ? tu.stallguard_sgt
            : static_cast<int8_t>(TestConfig::StallGuard::SGT_HOMING);
        sg_override.threshold = sgt;
        sg_override.enable_filter = true;  // Always ON for bounds finding (critical for reliability)
        sg_override.min_velocity = tu.stallguard_min_velocity_rpm;
        sg_override.max_velocity = 0.0f;
        sg_override.velocity_unit = tmc51x0::Unit::RPM;
        opt.stallguard_override = &sg_override;
        opt.current_reduction_factor = tu.stall_detection_current_factor;
        
        ESP_LOGI(TAG, "[bounds_find] StallGuard config: SGT=%d, Filter=ON, MinVel=%.1f RPM, CurrentFactor=%.0f%%",
                 sg_override.threshold, sg_override.min_velocity, tu.stall_detection_current_factor * 100.0f);
    } else {
        opt.stallguard_override = nullptr;
        opt.current_reduction_factor = 0.0f;
    }

    auto lib_res = g_driver->homing.FindBounds(method, opt, home, &ShouldCancelBounds);
    if (!lib_res) {
        if (lib_res.Error() == tmc51x0::ErrorCode::CANCELLED || g_cancel_bounds) {
        ESP_LOGW(TAG, "[bounds_find] Cancelled");
        MotorStopHoldDisable();
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }
        ESP_LOGE(TAG, "[bounds_find] Failed (err=%u)", static_cast<unsigned>(lib_res.Error()));
        g_state = InternalState::ERROR;
        MotorStopHoldDisable();
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        EspNowReceiver::send_error(3, 0);
        vTaskDelete(nullptr);
        return;
    }

    Homing::BoundsResult result = lib_res.Value();
    if (result.cancelled || g_cancel_bounds) {
        ESP_LOGW(TAG, "[bounds_find] Cancelled");
        MotorStopHoldDisable();
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    // Note: library returns a successful Result<BoundsResult> even if it did not detect
    // mechanical stalls on either side. In that case, `result.success` will be false.
    // We treat this as "unbounded" and use a safe default oscillation window.

    // Apply bounds and start motion
    //
    // IMPORTANT:
    // - Bounds finders return bounds in degrees relative to the *current* established zero/home.
    // - If the finder did NOT find both mechanical stops (`bounded == false`), we must treat the
    //   system as unbounded and re-establish home at the *current* position. Otherwise the next
    //   positioning command can look like "going in circles" as the controller tries to return
    //   to an assumed center that isn't actually the current axis location.
    //
    // Snapshot current position for logging and (unbounded) home establishment.
    float pos_deg = 0.0f;
    {
        TmcMutexGuard guard(*g_driver_mutex);
        auto pos_res = g_driver->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        pos_deg = pos_res ? pos_res.Value() : 0.0f;
        
        // CRITICAL: Explicitly disable stop-on-stall after bounds finding.
        // The library's RAII guard should restore SW_MODE, but we add this explicit
        // disable as a safety measure to ensure motion won't be interrupted by
        // stall events during normal operation (especially at low velocities where
        // SG_RESULT can naturally be 0).
        auto sg_stop_result = g_driver->stallGuard.EnableStopOnStall(false);
        if (!sg_stop_result) {
            ESP_LOGW(TAG, "[bounds_find] Failed to disable stop-on-stall (err=%u)", 
                     static_cast<unsigned>(sg_stop_result.Error()));
        } else {
            ESP_LOGI(TAG, "[bounds_find] Stop-on-stall disabled for normal motion");
        }
        
        // CRITICAL: Restore smooth motion mode for fatigue testing.
        // Bounds finding uses SpreadCycle (required for StallGuard), but normal
        // motion should use StealthChop for quiet operation. Also clear mode
        // change thresholds so StealthChop is active at all velocities.
        (void)g_driver->motorControl.SetStealthChopEnabled(true);
        (void)g_driver->thresholds.SetModeChangeSpeeds(0.0f, 0.0f, 0.0f, tmc51x0::Unit::RPM);
        ESP_LOGI(TAG, "[bounds_find] Restored StealthChop and cleared mode thresholds for smooth motion");
    }
    
    // Allow motor to settle at new mode/current before starting motion
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "[bounds_find] Result: success=%d bounded=%d min=%.2f° max=%.2f° current_pos=%.2f° (method=%s)",
             result.success ? 1 : 0, result.bounded ? 1 : 0, result.min_bound, result.max_bound, pos_deg,
             use_sg ? "StallGuard" : "Encoder");

    // Edge backoff: how far inside the mechanical bounds to oscillate.
    // This prevents repeatedly hitting the endpoints and gives margin for any residual error.
    // Must match the value used in bounds_finding_test.cpp for consistent behavior.
    static constexpr float OSCILLATION_EDGE_BACKOFF_DEG = 3.5f;
    
    if (result.bounded) {
        // Use the measured mechanical range as the oscillation range.
        g_motion->SetGlobalBounds(result.min_bound, result.max_bound);
        // Apply edge backoff to stay inside mechanical bounds during oscillation
        g_motion->SetLocalBoundsFromCenterDegrees(result.min_bound, result.max_bound, OSCILLATION_EDGE_BACKOFF_DEG);
        ESP_LOGI(TAG, "[bounds_find] Oscillation range: [%.2f°, %.2f°] (with %.2f° edge backoff)",
                 result.min_bound + OSCILLATION_EDGE_BACKOFF_DEG, 
                 result.max_bound - OSCILLATION_EDGE_BACKOFF_DEG,
                 OSCILLATION_EDGE_BACKOFF_DEG);
    } else {
        // Treat as unbounded:
        // - Define home at the *current* physical position (XACTUAL=0), so subsequent motion is small and local.
        {
            TmcMutexGuard guard(*g_driver_mutex);
            (void)g_driver->rampControl.Stop();
            (void)g_driver->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
            (void)g_driver->rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
            (void)g_driver->rampControl.SetTargetPosition(0.0f, tmc51x0::Unit::Deg);
        }

        // NOTE: FatigueTestMotion::SetUnbounded expects a *total* range; pass 350° to yield ±175°.
        g_motion->SetUnbounded(0.0f, 350.0f);
        // Unbounded mode: use a safe default range with edge backoff
        g_motion->SetLocalBoundsFromCenterDegrees(-60.0f, 60.0f, OSCILLATION_EDGE_BACKOFF_DEG);
        ESP_LOGW(TAG, "[bounds_find] Unbounded mode: using default oscillation range [-60°, 60°]");
    }
    g_bounds_found = true;
    
    // Mark bounds as found in cache (starts de-energize timer)
    BoundsCache::MarkBoundsFound();

    if (g_state != InternalState::BOUNDS_FINDING) {
        // PAUSE/STOP may have arrived after bounds finished.
        ESP_LOGW(TAG, "[bounds_find] State changed before motion start; stopping and disabling motor");
        BoundsCache::InvalidateBounds();
        MotorStopHoldDisable();
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    
    // Check if this was a FIND_ONLY request (independent bounds finding)
    if (g_bounds_task_mode == BoundsTaskMode::FIND_ONLY) {
        ESP_LOGI(TAG, "[bounds_find] Bounds-only mode complete - motor staying energized for %lu seconds",
                 (unsigned long)BoundsCache::GetRemainingValiditySec());

        // Report bounds to UI (relative to center/home).
        float local_min = 0.0f;
        float local_max = 0.0f;
        float global_min = 0.0f;
        float global_max = 0.0f;
        if (g_motion) {
            g_motion->GetLocalBoundsFromCenterDegrees(local_min, local_max);
            g_motion->GetGlobalBoundsDegrees(global_min, global_max);
        }
        (void)EspNowReceiver::send_bounds_result(
            1,
            result.bounded ? 1 : 0,
            0,
            local_min,
            local_max,
            global_min,
            global_max);

        g_state = InternalState::IDLE;
        EspNowReceiver::send_status_update(g_motion ? g_motion->GetCurrentCycles() : 0, TestState::Idle, 0, GetBoundsValidFlag_());
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        // Motor stays energized; de-energize timer will fire after validity window
        vTaskDelete(nullptr);
        return;
    }

    // Cancel de-energize timer since we're starting the test
    BoundsCache::CancelDeenergizeTimer();
    
    // CRITICAL: On START (new test), reset cycle count to zero.
    // On RESUME, keep existing cycle count and continue from where we left off.
    if (start_kind == PendingStartKind::START) {
        ESP_LOGI(TAG, "[bounds_find] START: Resetting cycle count to zero");
        g_motion->ResetCycles();
    } else {
        ESP_LOGI(TAG, "[bounds_find] RESUME: Continuing from cycle %lu", 
                 (unsigned long)g_motion->GetCurrentCycles());
    }

    // Ensure latest config is applied before starting motion.
    ApplyMotionConfigFromSettings_();

    if (!g_motion->Start()) {
        ESP_LOGE(TAG, "[bounds_find] Motion start failed after bounds");
        g_state = InternalState::ERROR;
        MotorStopHoldDisable();
        g_bounds_task_running = false;
        g_bounds_task_handle = nullptr;
        EspNowReceiver::send_error(2, 0);
        vTaskDelete(nullptr);
        return;
    }

    g_state = InternalState::RUNNING;
    EspNowReceiver::send_status_update(g_motion->GetCurrentCycles(), TestState::Running, 0, GetBoundsValidFlag_());

    g_bounds_task_running = false;
    g_bounds_task_handle = nullptr;
    ESP_LOGI(TAG, "[bounds_find] Completed successfully");
    vTaskDelete(nullptr);
}

/**
 * @brief FreeRTOS task: drive the motion update loop and detect completion.
 *
 * @details
 * Runs at ~10ms period. When `g_state == RUNNING`, it calls `g_motion->Update()`.
 * It also logs StallGuard telemetry when StallGuard bounds mode is selected.
 *
 * Completion:
 * - If a finite target cycle count is reached, sends COMPLETED + TEST_COMPLETE,
 *   transitions to IDLE, and disables the motor.
 *
 * @param arg Unused (FreeRTOS task signature).
 */
static void motion_control_task(void* arg)
{
    const char* task_name = "motion_ctrl";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = TaskTiming::LOG_INTERVAL_LONG_US;

    // Driver fault polling: DRV_ERR in SPI_STATUS is only a hint/summary.
    // For actionable root-cause, poll GSTAT + DRV_STATUS at a low rate.
    static constexpr int64_t fault_poll_interval_us = 500'000; // 500ms
    int64_t last_fault_poll_time_us = start_time_us;
    int64_t last_fault_log_time_us = 0;
    
    // StallGuard monitoring (only when using StallGuard method)
    int64_t last_sg_log_time_us = start_time_us;
    const int64_t sg_log_interval_us = TaskTiming::SG_LOG_INTERVAL_US;
    const int64_t sg_log_interval_always_us = TaskTiming::SG_LOG_INTERVAL_ALWAYS_US;
    uint16_t last_sg_result = 0;
    int64_t last_sg_stealth_suppressed_log_us = 0;
    bool motion_was_running = false;
    int64_t motion_start_time_us = 0;
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    ESP_LOGI(TAG, "[%s] StallGuard monitoring: g_use_stallguard=%d, g_motion=%p, g_driver=%p", 
             task_name, g_use_stallguard ? 1 : 0, g_motion, g_driver);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();

        // ---------------------------------------------------------------------
        // Fault policy: treat SPI_STATUS.DRV_ERR as a trigger only.
        // Read GSTAT (latched flags) + DRV_STATUS (detailed bits) to decide.
        // Note: reading GSTAT clears its flags (per driver implementation).
        // ---------------------------------------------------------------------
        if (g_driver && g_driver_mutex &&
            (current_time_us - last_fault_poll_time_us) >= fault_poll_interval_us) {
            bool drv_err = false;
            bool uv_cp = false;
            bool reset = false;
            uint32_t drv_status_raw = 0;
            bool have_drv_status = false;
            bool should_hard_stop = false;

            {
                TmcMutexGuard guard(*g_driver_mutex);
                auto gstat_res = g_driver->status.GetGlobalStatus(drv_err, uv_cp);
                if (gstat_res.IsOk()) {
                    reset = gstat_res.Value();
                }

                if (drv_err || uv_cp) {
                    auto drv_status_res = g_driver->status.GetDriverStatusRegister();
                    if (drv_status_res.IsOk()) {
                        drv_status_raw = drv_status_res.Value();
                        have_drv_status = true;
                        tmc51x0::DRV_STATUS_Register ds{};
                        ds.value = drv_status_raw;

                        // "Hard stop" faults: charge pump undervoltage, shorts, or OT.
                        should_hard_stop = uv_cp || (ds.bits.ot != 0) ||
                                           (ds.bits.s2ga != 0) || (ds.bits.s2gb != 0) ||
                                           (ds.bits.s2vsa != 0) || (ds.bits.s2vsb != 0);
                    } else {
                        // If we cannot read DRV_STATUS but GSTAT reports a fault, be conservative.
                        should_hard_stop = true;
                    }
                }
            }

            // Rate-limit fault logs (avoid log storms if a fault is stuck).
            const bool should_log_fault = (drv_err || uv_cp || reset) &&
                                          ((last_fault_log_time_us == 0) ||
                                           (current_time_us - last_fault_log_time_us) >= 2'000'000);
            if (should_log_fault) {
                last_fault_log_time_us = current_time_us;
                if (drv_err || uv_cp) {
                    if (have_drv_status) {
                        tmc51x0::DRV_STATUS_Register ds{};
                        ds.value = drv_status_raw;
                        ESP_LOGW(TAG,
                                 "TMC fault (GSTAT-latched): drv_err=%d uv_cp=%d reset=%d | "
                                 "DRV_STATUS=0x%08" PRIX32 " [ot=%u otpw=%u s2ga=%u s2gb=%u s2vsa=%u s2vsb=%u stealth=%u]",
                                 drv_err ? 1 : 0, uv_cp ? 1 : 0, reset ? 1 : 0,
                                 drv_status_raw,
                                 (unsigned)ds.bits.ot, (unsigned)ds.bits.otpw,
                                 (unsigned)ds.bits.s2ga, (unsigned)ds.bits.s2gb,
                                 (unsigned)ds.bits.s2vsa, (unsigned)ds.bits.s2vsb,
                                 (unsigned)ds.bits.stealth);
                    } else {
                        ESP_LOGW(TAG,
                                 "TMC fault (GSTAT-latched): drv_err=%d uv_cp=%d reset=%d | DRV_STATUS unreadable",
                                 drv_err ? 1 : 0, uv_cp ? 1 : 0, reset ? 1 : 0);
                    }
                } else if (reset) {
                    ESP_LOGI(TAG, "TMC GSTAT indicates reset since last poll");
                }
            }

            if (should_hard_stop) {
                ESP_LOGE(TAG, "TMC fault requires stop: entering ERROR and disabling motor");
                g_state = InternalState::ERROR;
                MotorStopHoldDisable();
                (void)EspNowReceiver::send_error(2, g_motion ? g_motion->GetCurrentCycles() : 0);
            }

            last_fault_poll_time_us = current_time_us;
        }
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        if (g_motion) {
            const bool motion_is_running = (g_state == InternalState::RUNNING) && g_motion->IsRunning();
            
            // Detect when motion starts
            if (motion_is_running && !motion_was_running) {
                motion_start_time_us = current_time_us;
                ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════════════════════");
                ESP_LOGI(TAG, "Motion started - StallGuard monitoring active");
                ESP_LOGI(TAG, "StallGuard values will be logged every 200ms (or when values change)");
                ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════════════════════");
            }
            motion_was_running = motion_is_running;
            
            // Only call Update() if motion is actually running
            // This prevents UpdateSinuousMotion() from being called when motion hasn't started
            // which could cause fast oscillation if start_time_us_ is 0
            if (motion_is_running) {
                g_motion->Update();
            }
            
            // Monitor StallGuard values during motion (only when using StallGuard method)
            if (g_use_stallguard && motion_is_running && g_driver && g_driver_mutex) {
                bool always_log = (current_time_us - last_sg_log_time_us >= sg_log_interval_always_us);
                
                if (always_log || (current_time_us - last_sg_log_time_us >= sg_log_interval_us)) {
                    // StallGuard2 is only meaningful in SpreadCycle. In StealthChop, SG_RESULT is invalid/usually 0.
                    bool stealth_active = false;
                    {
                        TmcMutexGuard guard(*g_driver_mutex);
                        auto drv_status_res = g_driver->status.GetDriverStatusRegister();
                        if (drv_status_res.IsOk()) {
                            tmc51x0::DRV_STATUS_Register ds{};
                            ds.value = drv_status_res.Value();
                            stealth_active = (ds.bits.stealth != 0);
                        }
                    }

                    if (stealth_active) {
                        if (last_sg_stealth_suppressed_log_us == 0 ||
                            (current_time_us - last_sg_stealth_suppressed_log_us) >= 5'000'000) {
                            ESP_LOGI(TAG,
                                     "StallGuard monitoring suppressed (StealthChop active). "
                                     "SG_RESULT is invalid in StealthChop; disable StealthChop if you need StallGuard-based jam detection.");
                            last_sg_stealth_suppressed_log_us = current_time_us;
                        }
                        last_sg_log_time_us = current_time_us;
                        continue;
                    }

                    // Read driver telemetry with mutex protection
                    float pos_deg = 0.0f;
                    float vel_rpm = 0.0f;
                    uint16_t sg_val = 0;
                    bool sg_read_ok = false;
                    {
                        TmcMutexGuard guard(*g_driver_mutex);
                    auto pos_result = g_driver->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
                        pos_deg = pos_result.IsOk() ? pos_result.Value() : 0.0f;
                    
                    auto vel_result = g_driver->rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
                        vel_rpm = vel_result.IsOk() ? vel_result.Value() : 0.0f;
                    
                    auto sg_result = g_driver->stallGuard.GetStallGuardResult();
                        sg_read_ok = sg_result.IsOk();
                        if (sg_read_ok) {
                        sg_val = sg_result.Value();
                        }
                    }
                        
                    if (sg_read_ok) {
                        // Always log first value when motion starts, or if value changed significantly, or periodically
                        bool first_log = (motion_start_time_us > 0 && 
                                         (current_time_us - motion_start_time_us) < 500000); // First 500ms
                        bool value_changed = (sg_val != last_sg_result || (sg_val < 100 && last_sg_result >= 100));
                        
                        if (first_log || value_changed || always_log) {
                            ESP_LOGI(TAG, "SG_RESULT: %u, VACTUAL: %.2f RPM, Position: %.2f° (lower=more load, 0=stall)", 
                                     sg_val, vel_rpm, pos_deg);
                            last_sg_result = sg_val;
                            
                            // Warn if SG_RESULT is low and motor is moving (potential stall condition)
                            if (sg_val < 50 && std::abs(vel_rpm) > 10.0f) {
                                ESP_LOGW(TAG, "⚠️ High load detected: SG_RESULT=%u at %.2f RPM (consider increasing SGT if false stall)", 
                                         sg_val, vel_rpm);
                            }
                            
                            // Warn if velocity is too low for reliable StallGuard readings
                            if (std::abs(vel_rpm) < 60.0f && std::abs(vel_rpm) > 0.1f) {
                                ESP_LOGW(TAG, "⚠️ Low velocity (%.2f RPM) - StallGuard readings may be unreliable (min: ~60 RPM)", vel_rpm);
                            }
                        }
                    } else {
                        ESP_LOGW(TAG, "⚠ Failed to read StallGuard");
                    }
                    
                    last_sg_log_time_us = current_time_us;
                }
            } else if (g_use_stallguard && !motion_is_running && motion_was_running) {
                // Motion just stopped
                ESP_LOGI(TAG, "Motion stopped - StallGuard monitoring paused");
            } else if (g_use_stallguard && !motion_is_running) {
                // Debug: Log why StallGuard monitoring is not active (only once per second to avoid spam)
                static int64_t last_debug_log_us = 0;
                if (current_time_us - last_debug_log_us >= 1000000) {
                    if (!g_driver) {
                        ESP_LOGD(TAG, "StallGuard monitoring: g_driver is null");
                    } else if (!g_motion) {
                        ESP_LOGD(TAG, "StallGuard monitoring: g_motion is null");
                    } else {
                        ESP_LOGD(TAG, "StallGuard monitoring: Motion not running (waiting for START command)");
                    }
                    last_debug_log_us = current_time_us;
                }
            }
            
            // Check if test completed
            if (g_state == InternalState::RUNNING && !g_motion->IsRunning()) {
                auto status = g_motion->GetStatus();
                if (status.current_cycles >= status.target_cycles && status.target_cycles > 0) {
                    EspNowReceiver::send_status_update(status.current_cycles, TestState::Completed, 0, GetBoundsValidFlag_());
                    EspNowReceiver::send_test_complete();
                    // Completion should de-energize motor
                    g_state = InternalState::IDLE;
                    MotorStopHoldDisable();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TaskTiming::MOTION_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief FreeRTOS task: periodically send `STATUS_UPDATE` to the remote controller.
 *
 * @details
 * Runs at 1Hz and publishes cycle count and a protocol state mapping.
 * While bounds finding is active, we currently report RUNNING via `ToProtoState()`.
 *
 * @param arg Unused (FreeRTOS task signature).
 */
static void status_update_task(void* arg)
{
    const char* task_name = "status_upd";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = TaskTiming::LOG_INTERVAL_US;
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        vTaskDelay(pdMS_TO_TICKS(TaskTiming::STATUS_UPDATE_PERIOD_MS));
        
        if (g_motion) {
            InternalState s = g_state;
            if (s == InternalState::RUNNING || s == InternalState::BOUNDS_FINDING) {
                auto status = g_motion->GetStatus();
                EspNowReceiver::send_status_update(status.current_cycles, ToProtoState(s), 0, GetBoundsValidFlag_());
            }
        }
    }
}

//=============================================================================
// UART Command Parser - Redesigned Architecture
//=============================================================================

// Forward declarations for command handlers
struct ParsedCommand;
namespace FatigueTest { class FatigueTestMotion; }
static bool HandleSet(const ParsedCommand& cmd, FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleStart(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleStop(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandlePause(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleResume(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleReset(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleStatus(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleHelp(const std::string& topic) noexcept;
static bool HandlePair() noexcept;
static bool HandleBounds() noexcept;

/**
 * @brief Command types for word-based commands
 */
enum class CommandType {
    SET,      // set <options>
    START,    // start
    STOP,     // stop
    PAUSE,    // pause
    RESUME,   // resume
    BOUNDS,   // bounds - run bounds finding independently
    PAIR,     // pair - enter pairing mode for remote controller
    RESET,    // reset
    STATUS,   // status
    HELP      // help [command]
};

/**
 * @brief Option types for SET command
 */
enum class OptionType {
    VELOCITY,      // -v, --velocity (VMAX in RPM)
    ACCELERATION,  // -a, --acceleration (AMAX in rev/s²)
    DWELL,         // -d, --dwell
    BOUNDS,        // -b, --bounds
    CYCLES         // -c, --cycles
};

/**
 * @brief Parsed command structure
 */
struct ParsedCommand {
    CommandType type;
    std::vector<std::pair<OptionType, std::vector<std::string>>> options; // For SET command
    std::string help_topic; // For HELP command
};

/**
 * @brief Visual output formatting system
 */
namespace CommandOutput {
    static constexpr int BOX_WIDTH = 78;
    
    /**
     * @brief Print a success message to the ESP-IDF log.
     * @param format printf-style format string.
     * @param ... printf-style arguments.
     * @note Uses `ESP_LOGI` and includes a success glyph prefix.
     */
    void PrintSuccess(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGI(TAG, "✓ %s", buffer);
    }
    
    /**
     * @brief Print an error message to the ESP-IDF log.
     * @param format printf-style format string.
     * @param ... printf-style arguments.
     * @note Uses `ESP_LOGE` and includes an error glyph prefix.
     */
    void PrintError(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGE(TAG, "✗ Error: %s", buffer);
    }
    
    /**
     * @brief Print an informational message to the ESP-IDF log.
     * @param format printf-style format string.
     * @param ... printf-style arguments.
     * @note Uses `ESP_LOGI` and includes an info glyph prefix.
     */
    void PrintInfo(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGI(TAG, "ℹ %s", buffer);
    }
    
    /**
     * @brief Print a warning message to the ESP-IDF log.
     * @param format printf-style format string.
     * @param ... printf-style arguments.
     * @note Uses `ESP_LOGW` and includes a warning glyph prefix.
     */
    void PrintWarning(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGW(TAG, "⚠ Warning: %s", buffer);
    }
    
    /**
     * @brief Print a boxed header line for human-readable UART output.
     * @param title Title to center in the header.
     * @note This is presentation-only and does not affect system state.
     */
    void PrintHeader(const char* title) {
        ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
        char header[BOX_WIDTH + 1];
        int title_len = strlen(title);
        int padding = (BOX_WIDTH - title_len - 2) / 2;
        snprintf(header, sizeof(header), "║%*s%s%*s║", padding, "", title, BOX_WIDTH - padding - title_len - 2, "");
        ESP_LOGI(TAG, "%s", header);
        ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════════════╣");
    }
    
    /**
     * @brief Print a boxed separator line for UART output.
     */
    void PrintSeparator() {
        ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════════════╣");
    }
    
    /**
     * @brief Print a boxed footer line for UART output.
     */
    void PrintFooter() {
        ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    }
    
    /**
     * @brief Print a key/value row within the boxed UART output format.
     * @param label Left column label.
     * @param value Right column value (will be truncated to fit).
     */
    void PrintTableRow(const char* label, const char* value) {
        // Use a larger buffer to account for multi-byte Unicode characters
        char row[BOX_WIDTH * 2 + 1];
        // Truncate value if needed to prevent format truncation warning
        char value_trunc[43];
        snprintf(value_trunc, sizeof(value_trunc), "%.42s", value);
        // Format: "║  " + label (30) + " " + value (42) + "║"
        int written = snprintf(row, sizeof(row), "║  %-30s %-42s║", label, value_trunc);
        if (written < 0 || written >= static_cast<int>(sizeof(row))) {
            // Truncation occurred, ensure null termination
            row[sizeof(row) - 1] = '\0';
        }
        ESP_LOGI(TAG, "%s", row);
    }
    
    /**
     * @brief Print an empty boxed line for spacing in UART output.
     */
    void PrintEmptyLine() {
        ESP_LOGI(TAG, "║%*s║", BOX_WIDTH - 2, "");
    }
    
    /**
     * @brief Print a success/failure result line for a command.
     * @param command Command name (displayed).
     * @param success Whether the command succeeded.
     * @param details Optional additional details.
     */
    void PrintCommandResult(const char* command, bool success, const char* details = nullptr) {
        if (success) {
            if (details) {
                PrintSuccess("%s: %s", command, details);
            } else {
                PrintSuccess("%s completed successfully", command);
            }
        } else {
            if (details) {
                PrintError("%s failed: %s", command, details);
            } else {
                PrintError("%s failed", command);
            }
        }
    }
}

/**
 * @brief Option parser for SET command
 */
class OptionParser {
public:
    struct OptionDef {
        OptionType type;
        const char* short_name;
        const char* long_name;
        const char* description;
        int min_args;
        int max_args;
    };
    
    /**
     * @brief Find a supported option definition by token.
     * @param name Token such as "-f", "--frequency", etc.
     * @return Pointer to the matching definition, or nullptr if unknown.
     */
    static const OptionDef* FindOption(const std::string& name) {
        static const OptionDef options[] = {
            {OptionType::VELOCITY, "-v", "--velocity", "Max velocity in RPM", 1, 1},
            {OptionType::ACCELERATION, "-a", "--acceleration", "Acceleration in rev/s²", 1, 1},
            {OptionType::DWELL, "-d", "--dwell", "Dwell times in ms (min, max)", 2, 2},
            {OptionType::BOUNDS, "-b", "--bounds", "Angle bounds in degrees (min, max)", 2, 2},
            {OptionType::CYCLES, "-c", "--cycles", "Target cycle count (0 = infinite)", 1, 1}
        };
        
        for (const auto& opt : options) {
            if (name == opt.short_name || name == opt.long_name) {
                return &opt;
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Parse SET command options from a token sequence.
     *
     * @details Supports both flag-style (e.g. `-f 0.5`) and word-style
     * (e.g. `frequency 0.5`) options. Multiple options may be specified.
     *
     * @param tokens Tokens following the "set" verb.
     * @param options Output vector of parsed options and their argument strings.
     * @return true if parsing succeeded; false if tokens are malformed.
     */
    static bool ParseOptions(const std::vector<std::string>& tokens,
                             std::vector<std::pair<OptionType, std::vector<std::string>>>& options) {
        options.clear();
        
        for (size_t i = 0; i < tokens.size(); ++i) {
            const OptionDef* def = FindOption(tokens[i]);
            if (def) {
                // Flag-based option (e.g., "-f", "--frequency")
                // Check if we have enough arguments
                if (i + def->min_args >= tokens.size()) {
                    return false; // Not enough arguments
                }
                
                // Extract arguments
                std::vector<std::string> args;
                int arg_count = std::min(def->max_args, static_cast<int>(tokens.size() - i - 1));
                for (int j = 0; j < arg_count; ++j) {
                    args.push_back(tokens[i + j + 1]);
                }
                
                options.push_back({def->type, args});
                i += arg_count; // Skip the arguments we just consumed
            } else {
                // Check if it's a word-based option (e.g., "velocity", "dwell")
                if ((tokens[i] == "velocity" || tokens[i] == "vmax") && i + 1 < tokens.size()) {
                    options.push_back({OptionType::VELOCITY, {tokens[i + 1]}});
                    i++;
                } else if ((tokens[i] == "acceleration" || tokens[i] == "accel" || tokens[i] == "amax") && i + 1 < tokens.size()) {
                    options.push_back({OptionType::ACCELERATION, {tokens[i + 1]}});
                    i++;
                } else if (tokens[i] == "dwell" && i + 2 < tokens.size()) {
                    options.push_back({OptionType::DWELL, {tokens[i + 1], tokens[i + 2]}});
                    i += 2;
                } else if (tokens[i] == "bounds" && i + 2 < tokens.size()) {
                    options.push_back({OptionType::BOUNDS, {tokens[i + 1], tokens[i + 2]}});
                    i += 2;
                } else if (tokens[i] == "cycles" && i + 1 < tokens.size()) {
                    options.push_back({OptionType::CYCLES, {tokens[i + 1]}});
                    i++;
                } else {
                    // Unknown token - could be a value from previous option or invalid
                    // Check if previous option consumed all its args - if so, this is invalid
                    return false; // Unknown option or unexpected token
                }
            }
        }
        
        return true;
    }
    
    /**
     * @brief Get an option definition by OptionType.
     * @param type Option type enum.
     * @return Pointer to the definition, or nullptr if not found.
     */
    static const OptionDef* GetOptionDef(OptionType type) {
        static const OptionDef options[] = {
            {OptionType::VELOCITY, "-v", "--velocity", "Max velocity in RPM", 1, 1},
            {OptionType::ACCELERATION, "-a", "--acceleration", "Acceleration in rev/s²", 1, 1},
            {OptionType::DWELL, "-d", "--dwell", "Dwell times in ms (min, max)", 2, 2},
            {OptionType::BOUNDS, "-b", "--bounds", "Angle bounds in degrees (min, max)", 2, 2},
            {OptionType::CYCLES, "-c", "--cycles", "Target cycle count (0 = infinite)", 1, 1}
        };
        
        for (const auto& opt : options) {
            if (opt.type == type) {
                return &opt;
            }
        }
        return nullptr;
    }
};

/**
 * @brief Redesigned UART command parser with word-based commands
 */
class UartCommandParser {
private:
    uart_port_t uart_port_;
    char rx_buffer_[256];
    char line_buffer_[256];
    size_t line_buffer_pos_;
    static constexpr size_t RX_BUF_SIZE = 256;

    /**
     * @brief Split a command line into tokens.
     *
     * @details
     * - Splits on whitespace
     * - Preserves quoted substrings (double quotes)
     *
     * @param line Null-terminated command line string.
     * @return Token vector (may be empty).
     */
    std::vector<std::string> Tokenize(const char* line) noexcept {
        std::vector<std::string> tokens;
        std::string current;
        bool in_quotes = false;

        for (const char* p = line; *p != '\0'; ++p) {
            if (*p == '"') {
                in_quotes = !in_quotes;
            } else if (isspace(*p) && !in_quotes) {
                if (!current.empty()) {
                    tokens.push_back(current);
                    current.clear();
                }
            } else {
                current += *p;
            }
        }

        if (!current.empty()) {
            tokens.push_back(current);
        }

        return tokens;
    }
    
    /**
     * @brief Map a verb token (first token) to a `CommandType`.
     *
     * @param token Lowercase verb token (e.g. "set", "start").
     * @return Parsed command type. Unknown verbs map to HELP.
     */
    CommandType ParseCommandType(const std::string& token) noexcept {
        if (token == "set") return CommandType::SET;
        if (token == "start") return CommandType::START;
        if (token == "stop") return CommandType::STOP;
        if (token == "pause") return CommandType::PAUSE;
        if (token == "resume") return CommandType::RESUME;
        if (token == "bounds") return CommandType::BOUNDS;
        if (token == "reset") return CommandType::RESET;
        if (token == "status") return CommandType::STATUS;
        if (token == "help") return CommandType::HELP;
        if (token == "pair") return CommandType::PAIR;
        return CommandType::HELP; // Default to help for unknown
    }
    
    /**
     * @brief Convert tokens into a structured ParsedCommand.
     *
     * @param tokens Token vector as returned by Tokenize().
     * @param cmd Output structured command.
     * @return true if parsing succeeded; false otherwise.
     */
    bool ParseCommand(const std::vector<std::string>& tokens, ParsedCommand& cmd) noexcept {
        if (tokens.empty()) {
            return false;
        }
        
        cmd.type = ParseCommandType(tokens[0]);
        
        if (cmd.type == CommandType::SET) {
            // Parse options for SET command
            std::vector<std::string> option_tokens(tokens.begin() + 1, tokens.end());
            if (option_tokens.empty()) {
                // Empty SET command - will be handled by HandleSet
                return true;
            }
            if (!OptionParser::ParseOptions(option_tokens, cmd.options)) {
                CommandOutput::PrintError("Failed to parse SET command options");
                CommandOutput::PrintInfo("Use 'help set' for usage information");
                return false;
            }
        } else if (cmd.type == CommandType::HELP) {
            // Parse help topic if provided
            if (tokens.size() > 1) {
                cmd.help_topic = tokens[1];
            }
        }
        
        return true;
    }

public:
    /**
     * @brief Construct a UART command parser bound to a specific UART port.
     *
     * @details
     * Configures the UART driver for non-blocking reads and line-based parsing.
     * On ESP32-C6, UART0 is often already configured by the console; this
     * constructor tolerates that case.
     *
     * @param uart_port UART port to use (e.g. UART_NUM_0).
     */
    UartCommandParser(uart_port_t uart_port) : uart_port_(uart_port), line_buffer_pos_(0) {
        line_buffer_[0] = '\0';
        
        // On ESP32C6, UART_NUM_0 is typically used by the console
        // Try to install the driver - if it's already installed, we'll get ESP_ERR_INVALID_STATE
        ESP_LOGI(TAG, "Configuring UART driver on port %d", uart_port_);
        
        // Configure UART
        uart_config_t uart_config = {};
        uart_config.baud_rate = 115200;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_DEFAULT;

        // Try to install UART driver with RX buffer
        // If driver is already installed (by console), we'll get ESP_ERR_INVALID_STATE
        esp_err_t ret = uart_driver_install(uart_port_, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
        if (ret == ESP_ERR_INVALID_STATE) {
            // Driver already installed (likely by console) - that's fine, we can use it
            ESP_LOGI(TAG, "UART port %d already has driver installed (using existing)", uart_port_);
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
            return;
        } else {
            ESP_LOGI(TAG, "UART driver installed on port %d", uart_port_);
        }
        
        // Configure UART parameters (safe to call even if driver was already installed)
        ret = uart_param_config(uart_port_, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
            return;
        }
        
        // For USB serial (UART_NUM_0), pins are typically handled by USB driver
        ret = uart_set_pin(uart_port_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
            return;
        }
        
        // Verify we can read from the UART
        size_t test_available = 0;
        ret = uart_get_buffered_data_len(uart_port_, &test_available);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "UART port %d ready, initial buffered data: %zu bytes", 
                     uart_port_, test_available);
        } else {
            ESP_LOGE(TAG, "UART port %d not accessible: %s", uart_port_, esp_err_to_name(ret));
        }
    }

    /**
     * @brief Parse and execute a single command line.
     *
     * @param line Null-terminated command line.
     * @param motion Motion controller to operate on.
     * @return true if the command executed successfully; false otherwise.
     *
     * @note This function routes to the `Handle*` command handlers.
     */
    bool ProcessCommand(const char* line, FatigueTest::FatigueTestMotion& motion) noexcept {
        if (!line || strlen(line) == 0) {
            return false;
        }

        std::vector<std::string> tokens = Tokenize(line);
        if (tokens.empty()) {
            return false;
        }
        
        ParsedCommand cmd;
        if (!ParseCommand(tokens, cmd)) {
            CommandOutput::PrintError("Failed to parse command");
            return false;
        }
        
        // Route to appropriate handler
        switch (cmd.type) {
            case CommandType::SET:
                return HandleSet(cmd, motion);
            case CommandType::START:
                return HandleStart(motion);
            case CommandType::STOP:
                return HandleStop(motion);
            case CommandType::PAUSE:
                return HandlePause(motion);
            case CommandType::RESUME:
                return HandleResume(motion);
            case CommandType::BOUNDS:
                return HandleBounds();
            case CommandType::RESET:
                return HandleReset(motion);
            case CommandType::STATUS:
                return HandleStatus(motion);
            case CommandType::HELP:
                return HandleHelp(cmd.help_topic);
            case CommandType::PAIR:
                return HandlePair();
            default:
                CommandOutput::PrintError("Unknown command type");
                return false;
        }
    }

    /**
     * @brief Poll the UART for input and process complete lines.
     *
     * @details
     * Reads any currently buffered bytes (non-blocking) and accumulates them
     * into a line buffer. When a newline is received, the line is executed via
     * ProcessCommand().
     *
     * @param motion Motion controller to operate on.
     *
     * @warning This runs in a polling task. Keep per-call work bounded.
     */
    void ProcessUartCommands(FatigueTest::FatigueTestMotion& motion) noexcept {
        // Check available bytes first
        size_t available = 0;
        esp_err_t ret = uart_get_buffered_data_len(uart_port_, &available);
        if (ret != ESP_OK) {
            // Log error only occasionally to avoid spam
            static uint32_t error_count = 0;
            if (error_count++ % 100 == 0) {
                ESP_LOGE(TAG, "Failed to get UART buffered data length: %s (count=%lu)", 
                        esp_err_to_name(ret), error_count);
            }
            return;
        }
        
        // Log when data becomes available (for debugging)
        static size_t last_available = 0;
        if (available > 0 && available != last_available) {
            ESP_LOGI(TAG, "UART has %zu bytes available (was %zu)", available, last_available);
        }
        last_available = available;
        
        // Read available bytes from UART (non-blocking)
        // Use a small timeout to ensure we get complete data
        int len = uart_read_bytes(uart_port_, (uint8_t*)rx_buffer_, RX_BUF_SIZE - 1, 
                                  available > 0 ? pdMS_TO_TICKS(10) : 0);
        if (len > 0) {
            rx_buffer_[len] = '\0';
            ESP_LOGI(TAG, "UART read %d bytes: '%.*s'", len, len, rx_buffer_);
            
            // Process each character, accumulating into line buffer until newline
            for (int i = 0; i < len; i++) {
                char c = rx_buffer_[i];
                
                // Handle newline/carriage return - process the complete line
                if (c == '\n' || c == '\r') {
                    if (line_buffer_pos_ > 0) {
                        line_buffer_[line_buffer_pos_] = '\0';
                        ESP_LOGI(TAG, "Received UART command: '%s'", line_buffer_);
                        ProcessCommand(line_buffer_, motion);
                        line_buffer_pos_ = 0;
                        line_buffer_[0] = '\0';
                    }
                    // Skip multiple consecutive newlines
                    continue;
                }
                
                // Add character to line buffer (ignore if buffer is full)
                if (line_buffer_pos_ < (RX_BUF_SIZE - 1)) {
                    line_buffer_[line_buffer_pos_++] = c;
                } else {
                    // Buffer overflow - reset and log error
                    ESP_LOGW(TAG, "UART command line too long, resetting buffer");
                    line_buffer_pos_ = 0;
                    line_buffer_[0] = '\0';
                }
            }
        } else if (available > 0 && len == 0) {
            // Data is available but read returned 0 - this shouldn't happen
            ESP_LOGW(TAG, "UART has %zu bytes available but read returned 0", available);
        }
    }

};

//=============================================================================
// Command Handlers
//=============================================================================

/**
 * @brief Apply SET command options to the motion controller.
 *
 * @details
 * Validates arguments, applies parameters to `motion`, and prints a structured
 * success/failure summary. Some options also update global settings (e.g., cycle target).
 *
 * @param cmd Parsed command containing SET options.
 * @param motion Motion controller instance to update.
 * @return true if all options applied successfully; false otherwise.
 */
static bool HandleSet(const ParsedCommand& cmd, FatigueTest::FatigueTestMotion& motion) noexcept {
    if (cmd.options.empty()) {
        CommandOutput::PrintError("SET command requires at least one option");
        CommandOutput::PrintInfo("Use 'help set' for usage information");
        return false;
    }
    
    int success_count = 0;
    int failure_count = 0;
    
    // Process each option
    for (const auto& opt_pair : cmd.options) {
        OptionType opt_type = opt_pair.first;
        const std::vector<std::string>& args = opt_pair.second;
        bool success = false;
        
        switch (opt_type) {
            case OptionType::VELOCITY: {
                if (args.empty()) {
                    CommandOutput::PrintError("Velocity option requires a value (RPM)");
                    failure_count++;
                    break;
                }
                float vmax = std::strtof(args[0].c_str(), nullptr);
                if (vmax < 5.0f || vmax > 120.0f) {
                    CommandOutput::PrintError("Velocity must be between 5 and 120 RPM (got %.1f)", vmax);
                    failure_count++;
                    break;
                }
                success = motion.SetMaxVelocity(vmax);
                if (success) {
                    CommandOutput::PrintSuccess("Max velocity set to %.1f RPM", vmax);
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set velocity");
                    failure_count++;
                }
                break;
            }
            
            case OptionType::ACCELERATION: {
                if (args.empty()) {
                    CommandOutput::PrintError("Acceleration option requires a value (rev/s²)");
                    failure_count++;
                    break;
                }
                float amax = std::strtof(args[0].c_str(), nullptr);
                if (amax < 0.5f || amax > 30.0f) {
                    CommandOutput::PrintError("Acceleration must be between 0.5 and 30 rev/s² (got %.2f)", amax);
                    failure_count++;
                    break;
                }
                success = motion.SetAcceleration(amax);
                if (success) {
                    CommandOutput::PrintSuccess("Acceleration set to %.2f rev/s²", amax);
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set acceleration");
                    failure_count++;
                }
                break;
            }
            
            case OptionType::DWELL: {
                if (args.size() < 2) {
                    CommandOutput::PrintError("Dwell option requires 2 values (min_ms, max_ms)");
                    failure_count++;
                    break;
                }
                uint32_t min_ms = std::strtoul(args[0].c_str(), nullptr, 10);
                uint32_t max_ms = std::strtoul(args[1].c_str(), nullptr, 10);
                if (min_ms > 60000 || max_ms > 60000) {
                    CommandOutput::PrintError("Dwell times must be between 0 and 60000 ms");
                    failure_count++;
                    break;
                }
                if (min_ms > max_ms) {
                    CommandOutput::PrintError("Dwell min (%lu) must be <= max (%lu)", min_ms, max_ms);
                    failure_count++;
                    break;
                }
                success = motion.SetDwellTimes(min_ms, max_ms);
                if (success) {
                    CommandOutput::PrintSuccess("Dwell times set: %lu-%lu ms", min_ms, max_ms);
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set dwell times");
                    failure_count++;
                }
                break;
            }
            
            case OptionType::BOUNDS: {
                if (args.size() < 2) {
                    CommandOutput::PrintError("Bounds option requires 2 values (min_deg, max_deg)");
                    failure_count++;
                    break;
                }
                float min_deg = std::strtof(args[0].c_str(), nullptr);
                float max_deg = std::strtof(args[1].c_str(), nullptr);
                if (min_deg < -180.0f || max_deg > 180.0f) {
                    CommandOutput::PrintError("Bounds must be between -180° and +180°");
                    failure_count++;
                    break;
                }
                if (min_deg >= max_deg) {
                    CommandOutput::PrintError("Bounds min (%.2f°) must be < max (%.2f°)", min_deg, max_deg);
                    failure_count++;
                    break;
                }
                success = motion.SetLocalBoundsFromCenterDegrees(min_deg, max_deg);
                if (success) {
                    CommandOutput::PrintSuccess("Bounds set: %.2f° to %.2f° from center", min_deg, max_deg);
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set bounds");
                    failure_count++;
                }
                break;
            }
            
            case OptionType::CYCLES: {
                if (args.empty()) {
                    CommandOutput::PrintError("Cycles option requires a value");
                    failure_count++;
                    break;
                }
                uint32_t cycles = std::strtoul(args[0].c_str(), nullptr, 10);
                success = motion.SetTargetCycles(cycles);
                if (success) {
                    g_settings.test_unit.cycle_amount = cycles;
                    if (cycles == 0) {
                        CommandOutput::PrintSuccess("Target cycles set to infinite");
                    } else {
                        CommandOutput::PrintSuccess("Target cycles set to %lu", cycles);
                    }
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set target cycles");
                    failure_count++;
                }
                break;
            }
            
            default:
                CommandOutput::PrintError("Unknown option type");
                failure_count++;
                break;
        }
    }
    
    // Summary
    if (failure_count == 0) {
        CommandOutput::PrintSuccess("All settings applied successfully (%d option(s))", success_count);
        return true;
    } else if (success_count > 0) {
        CommandOutput::PrintWarning("Some settings were not applied (%d succeeded, %d failed)", 
                                    success_count, failure_count);
        return false;
    } else {
        CommandOutput::PrintError("No settings were applied");
        return false;
    }
}

/**
 * @brief Handle START command
 *
 * @details
 * UART START behaves like remote START: it launches cancellable bounds finding
 * and then starts motion if bounds succeed. The motor is enabled only during
 * bounds finding and active motion.
 *
 * @param motion Motion controller instance.
 * @return true if the START request was accepted; false otherwise.
 */
static bool HandleStart(FatigueTest::FatigueTestMotion& motion) noexcept {
    auto status = motion.GetStatus();
    if (status.running) {
        CommandOutput::PrintWarning("Test is already running");
        return false;
    }
    
    if (motion.IsCycleComplete()) {
        CommandOutput::PrintError("Cycle count reached. Use 'reset' to reset cycles or set new target.");
        return false;
    }
    
    // UART start should behave like remote start: (re)find bounds then start.
    if (g_state == InternalState::RUNNING || g_state == InternalState::BOUNDS_FINDING) {
        CommandOutput::PrintWarning("Already running or finding bounds");
        return false;
    }
    CommandOutput::PrintInfo("Starting: will (re)find bounds then begin motion");
    EspNowReceiver::send_start_ack();
    RequestStart(PendingStartKind::START);
    return true;
}

/**
 * @brief Handle STOP command
 *
 * @details
 * STOP is a hard stop: cancels bounds finding (if active), stops motion, sets
 * the internal state to IDLE, and de-energizes the motor.
 *
 * @param motion Motion controller instance.
 * @return true if a stop action was performed (even if already idle); false otherwise.
 */
static bool HandleStop(FatigueTest::FatigueTestMotion& motion) noexcept {
    auto status = motion.GetStatus();
    if (!status.running) {
        CommandOutput::PrintWarning("Test is not running");
        // Still ensure motor is de-energized
        g_cancel_bounds = true;
        g_state = InternalState::IDLE;
        MotorStopHoldDisable();
        EspNowReceiver::send_stop_ack();
        EspNowReceiver::send_status_update(motion.GetCurrentCycles(), TestState::Idle, 0, GetBoundsValidFlag_());
        return true;
    }
    
    motion.Stop();
    CommandOutput::PrintSuccess("Fatigue test stopped");
    g_cancel_bounds = true;
    g_state = InternalState::IDLE;
    MotorStopHoldDisable();
    EspNowReceiver::send_stop_ack();
    EspNowReceiver::send_status_update(motion.GetCurrentCycles(), TestState::Idle, 0, GetBoundsValidFlag_());
    return true;
}

/**
 * @brief Handle PAUSE command (UART path).
 *
 * @details
 * UART pause/resume are currently not implemented (remote control handles PAUSE/RESUME).
 *
 * @param motion Motion controller instance (unused).
 * @return false (not implemented).
 */
static bool HandlePause(FatigueTest::FatigueTestMotion& motion) noexcept {
    (void)motion;
    CommandOutput::PrintInfo("Pause functionality not yet implemented");
    return false;
}

/**
 * @brief Handle RESUME command (UART path).
 *
 * @details
 * UART pause/resume are currently not implemented (remote control handles PAUSE/RESUME).
 *
 * @param motion Motion controller instance (unused).
 * @return false (not implemented).
 */
static bool HandleResume(FatigueTest::FatigueTestMotion& motion) noexcept {
    (void)motion;
    CommandOutput::PrintInfo("Resume functionality not yet implemented");
    return false;
}

/**
 * @brief Handle RESET command
 *
 * @details Resets the cycle counter to 0 (does not start motion).
 *
 * @param motion Motion controller instance.
 * @return true on success.
 */
static bool HandleReset(FatigueTest::FatigueTestMotion& motion) noexcept {
    motion.ResetCycles();
    CommandOutput::PrintSuccess("Cycle counter reset to 0");
    return true;
}

/**
 * @brief Handle STATUS command
 *
 * @details Prints a human-readable status snapshot (state, bounds, frequency, cycles, dwell).
 *
 * @param motion Motion controller instance.
 * @return true always (printing only).
 */
static bool HandleStatus(FatigueTest::FatigueTestMotion& motion) noexcept {
    FatigueTest::FatigueTestMotion::Status status = motion.GetStatus();
    
    CommandOutput::PrintHeader("MOTION STATUS");
    CommandOutput::PrintEmptyLine();
    
    char state_str[32];
    snprintf(state_str, sizeof(state_str), "%s", status.running ? "RUNNING" : "IDLE");
    CommandOutput::PrintTableRow("State:", state_str);
    
    char bounded_str[32];
    snprintf(bounded_str, sizeof(bounded_str), "%s", status.bounded ? "YES" : "NO");
    CommandOutput::PrintTableRow("Bounded:", bounded_str);
    
    char vmax_str[64];
    snprintf(vmax_str, sizeof(vmax_str), "%.1f RPM", status.vmax_rpm);
    CommandOutput::PrintTableRow("VMAX:", vmax_str);
    
    char amax_str[64];
    snprintf(amax_str, sizeof(amax_str), "%.2f rev/s²", status.amax_rev_s2);
    CommandOutput::PrintTableRow("AMAX:", amax_str);
    
    char freq_str[64];
    snprintf(freq_str, sizeof(freq_str), "%.2f Hz (estimated)", status.estimated_frequency_hz);
    CommandOutput::PrintTableRow("Est. Freq:", freq_str);
    
    char local_bounds_str[64];
    snprintf(local_bounds_str, sizeof(local_bounds_str), "%.2f° to %.2f° from center",
             status.min_degrees_from_center, status.max_degrees_from_center);
    CommandOutput::PrintTableRow("Local Bounds:", local_bounds_str);
    
    char global_bounds_str[64];
    snprintf(global_bounds_str, sizeof(global_bounds_str), "%.2f° to %.2f° from center",
             status.global_min_degrees, status.global_max_degrees);
    CommandOutput::PrintTableRow("Global Bounds:", global_bounds_str);
    
    char cycles_str[64];
    if (status.target_cycles == 0) {
        snprintf(cycles_str, sizeof(cycles_str), "%lu / infinite", status.current_cycles);
    } else {
        snprintf(cycles_str, sizeof(cycles_str), "%lu / %lu", 
                 status.current_cycles, status.target_cycles);
    }
    CommandOutput::PrintTableRow("Cycles:", cycles_str);
    
    char dwell_str[64];
    snprintf(dwell_str, sizeof(dwell_str), "%lu-%lu ms", status.dwell_min_ms, status.dwell_max_ms);
    CommandOutput::PrintTableRow("Dwell Times:", dwell_str);
    
    CommandOutput::PrintEmptyLine();
    
    // Bounds cache status
    CommandOutput::PrintInfo("BOUNDS CACHE:");
    if (BoundsCache::AreBoundsValid()) {
        char cache_str[64];
        snprintf(cache_str, sizeof(cache_str), "Valid (%lu sec remaining)",
                 (unsigned long)BoundsCache::GetRemainingValiditySec());
        CommandOutput::PrintTableRow("  Status:", cache_str);
        CommandOutput::PrintTableRow("  Motor:", "Energized (ready for start)");
    } else if (g_bounds_found) {
        CommandOutput::PrintTableRow("  Status:", "Expired (motor de-energized)");
        CommandOutput::PrintTableRow("  Motor:", "Disabled");
    } else {
        CommandOutput::PrintTableRow("  Status:", "Not found");
        CommandOutput::PrintTableRow("  Motor:", "Disabled");
    }
    
    CommandOutput::PrintEmptyLine();
    CommandOutput::PrintFooter();
    
    return true;
}

/**
 * @brief Handle PAIR command
 *
 * @details Enters pairing mode for 30 seconds to allow a remote controller
 * to securely pair with this test unit. The remote controller must also
 * initiate pairing and know the shared PAIRING_SECRET.
 *
 * @return true always (pairing mode entered).
 */
/**
 * @brief Handle BOUNDS command - run bounds finding independently.
 * 
 * @details
 * Runs bounds finding without starting the test. After bounds are found,
 * the motor stays energized for the configured validity period. If START
 * is requested within this window, bounds finding is skipped.
 * 
 * @return true if bounds finding started; false on error.
 */
static bool HandleBounds() noexcept {
    // Check if we can run bounds finding
    if (g_bounds_task_running) {
        CommandOutput::PrintWarning("Bounds finding already running");
        return false;
    }
    
    if (g_state == InternalState::RUNNING) {
        CommandOutput::PrintWarning("Cannot run bounds finding while test is running");
        return false;
    }
    
    CommandOutput::PrintHeader("BOUNDS FINDING");
    CommandOutput::PrintEmptyLine();
    
    // Check if bounds are already valid
    if (BoundsCache::AreBoundsValid()) {
        CommandOutput::PrintSuccess("Bounds are already valid!");
        char remaining_str[64];
        snprintf(remaining_str, sizeof(remaining_str), 
                 "%lu seconds until motor de-energizes",
                 (unsigned long)BoundsCache::GetRemainingValiditySec());
        CommandOutput::PrintInfo(remaining_str);
        CommandOutput::PrintInfo("Use 'start' to begin test without re-finding bounds.");
        CommandOutput::PrintFooter();
        return true;
    }
    
    // Start bounds finding
    if (!RequestBoundsOnly()) {
        CommandOutput::PrintError("Failed to start bounds finding");
        CommandOutput::PrintFooter();
        return false;
    }
    
    CommandOutput::PrintInfo("Bounds finding started...");
    CommandOutput::PrintInfo("Motor will stay energized after completion.");
    
    char validity_str[64];
    uint32_t validity_min = static_cast<uint32_t>(BoundsCache::g_bounds_validity_us / (60 * 1000000LL));
    snprintf(validity_str, sizeof(validity_str), 
             "Validity window: %lu minutes", (unsigned long)validity_min);
    CommandOutput::PrintInfo(validity_str);
    
    CommandOutput::PrintEmptyLine();
    CommandOutput::PrintInfo("Run 'start' within validity window to skip bounds finding.");
    CommandOutput::PrintFooter();
    
    return true;
}

static bool HandlePair() noexcept {
    EspNowReceiver::enter_pairing_mode(PAIRING_MODE_TIMEOUT_SEC);
    
    CommandOutput::PrintHeader("PAIRING MODE");
    CommandOutput::PrintEmptyLine();
    CommandOutput::PrintInfo("Pairing mode enabled for 30 seconds.");
    CommandOutput::PrintInfo("Start pairing from your remote controller now.");
    CommandOutput::PrintEmptyLine();
    
    char peer_count_str[32];
    snprintf(peer_count_str, sizeof(peer_count_str), "%zu", 
             EspNowReceiver::get_approved_peer_count());
    CommandOutput::PrintTableRow("Current approved peers:", peer_count_str);
    
    CommandOutput::PrintFooter();
    return true;
}

/**
 * @brief Handle HELP command
 *
 * @param topic Optional help topic (e.g. "set", "start").
 * @return true if help was shown successfully; false if topic was unknown.
 */
static bool HandleHelp(const std::string& topic) noexcept {
    if (topic.empty()) {
        // General help
        CommandOutput::PrintHeader("UART COMMAND INTERFACE");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintTableRow("set [OPTIONS...]", "Configure test parameters");
        CommandOutput::PrintTableRow("start", "Start fatigue test");
        CommandOutput::PrintTableRow("stop", "Stop fatigue test");
        CommandOutput::PrintTableRow("bounds", "Run bounds finding (keeps motor ready)");
        CommandOutput::PrintTableRow("reset", "Reset cycle counter");
        CommandOutput::PrintTableRow("status", "Show current status");
        CommandOutput::PrintTableRow("pair", "Enter pairing mode for remote");
        CommandOutput::PrintTableRow("help [command]", "Show help (general or specific)");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("Use 'help <command>' for detailed help on a specific command");
        CommandOutput::PrintFooter();
        return true;
    }
    
    // Specific command help
    if (topic == "set") {
        CommandOutput::PrintHeader("COMMAND HELP: set");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Configure test parameters. Multiple options can be set in one command.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  set [OPTIONS...]");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("OPTIONS:");
        CommandOutput::PrintInfo("  -f, --frequency <Hz>        Motion frequency (0.01 - 10.0 Hz)");
        CommandOutput::PrintInfo("  -d, --dwell <min> <max>     Dwell times in ms (0 - 60000)");
        CommandOutput::PrintInfo("  -b, --bounds <min> <max>    Angle bounds in degrees (-180 to 180)");
        CommandOutput::PrintInfo("  -c, --cycles <count>        Target cycles (0 = infinite)");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  set frequency 0.5");
        CommandOutput::PrintInfo("  set -f 0.5 -d 500 1000");
        CommandOutput::PrintInfo("  set -f 0.5 -d 500 1000 -b -60 60 -c 1000");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "start") {
        CommandOutput::PrintHeader("COMMAND HELP: start");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Start the fatigue test. Runs bounds finding first unless recently done.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("BOUNDS CACHING:");
        CommandOutput::PrintInfo("  If 'bounds' was run within the validity window (default 2 min),");
        CommandOutput::PrintInfo("  bounds finding is skipped and test starts immediately.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  start");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  bounds        # Run bounds finding, keep motor ready");
        CommandOutput::PrintInfo("  start         # Starts immediately if bounds still valid");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "bounds") {
        CommandOutput::PrintHeader("COMMAND HELP: bounds");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Run bounds finding independently without starting the test.");
        CommandOutput::PrintInfo("  After bounds are found, motor stays energized for the validity");
        CommandOutput::PrintInfo("  window (default 2 minutes). If 'start' is run within this window,");
        CommandOutput::PrintInfo("  bounds finding is skipped.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("MOTOR BEHAVIOR:");
        CommandOutput::PrintInfo("  - Motor energizes during bounds finding");
        CommandOutput::PrintInfo("  - Motor stays energized after completion");
        CommandOutput::PrintInfo("  - Motor de-energizes after validity timeout to prevent heating");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  bounds");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("WORKFLOW:");
        CommandOutput::PrintInfo("  1. Run 'bounds' to find limits");
        CommandOutput::PrintInfo("  2. Adjust settings with 'set' if needed");
        CommandOutput::PrintInfo("  3. Run 'start' within 2 minutes (no re-finding needed)");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "stop") {
        CommandOutput::PrintHeader("COMMAND HELP: stop");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Stop the currently running fatigue test.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  stop");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  stop");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "reset") {
        CommandOutput::PrintHeader("COMMAND HELP: reset");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Reset the cycle counter to 0.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  reset");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  reset");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "status") {
        CommandOutput::PrintHeader("COMMAND HELP: status");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Display current test status including all parameters.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  status");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  status");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else {
        CommandOutput::PrintError("Unknown help topic: %s", topic.c_str());
        CommandOutput::PrintInfo("Available help topics: set, start, stop, reset, status");
        return false;
    }
}

/**
 * @brief FreeRTOS task: poll UART and process commands.
 *
 * @details
 * This task calls `UartCommandParser::ProcessUartCommands()` periodically.
 *
 * @param arg Pointer to a `UartCommandParser` instance.
 */
static void uart_command_task(void* arg)
{
    UartCommandParser* parser = static_cast<UartCommandParser*>(arg);
    const char* task_name = "uart_cmd";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = TaskTiming::LOG_INTERVAL_US;
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        if (g_motion && parser) {
            parser->ProcessUartCommands(*g_motion);
        } else {
            if (!g_motion) {
                ESP_LOGW(TAG, "[%s] g_motion is null, skipping UART command processing", task_name);
            }
            if (!parser) {
                ESP_LOGE(TAG, "[%s] parser is null!", task_name);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TaskTiming::UART_POLL_PERIOD_MS));
    }
}

extern "C" void app_main()
{
    /**
     * @brief ESP-IDF application entry point.
     *
     * @details
     * Performs one-time initialization:
     * - ESP-NOW receiver (event queue + WiFi)
     * - SPI bus and TMC51x0 driver configuration for the selected test rig
     * - encoder configuration
     * - UART command parser
     * - background tasks for command processing, motion control, and status updates
     *
     * Safety policy:
     * - The motor is left DISABLED at boot.
     * - The motor is enabled only during bounds finding and active motion, and
     *   disabled again on PAUSE/STOP/completion.
     *
     * @note This function never returns under normal operation.
     */
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         Fatigue Test Unit with ESP-NOW Communication                         ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    // Initialize default stall detection parameters from test config EARLY.
    // This prevents ConfigResponse from ever reporting placeholder defaults (e.g. SGT=127)
    // if a remote requests config immediately after ESP-NOW comes up.
    using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<SELECTED_TEST_RIG>;
    g_settings.test_unit.bounds_search_velocity_rpm = TestConfig::Motion::BOUNDS_SEARCH_SPEED_RPM;
    g_settings.test_unit.stallguard_min_velocity_rpm = TestConfig::StallGuard::MIN_VELOCITY_RPM;
    g_settings.test_unit.stall_detection_current_factor = TestConfig::StallGuard::STALL_DETECTION_CURRENT_FACTOR;
    g_settings.test_unit.bounds_search_accel_rev_s2 = TestConfig::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;
    g_settings.test_unit.stallguard_sgt = TestConfig::StallGuard::SGT_HOMING;
    ESP_LOGI(TAG, "Default stall detection parameters initialized from test config");

    // Initialize ESP-NOW receiver
    g_espnowQueue = xQueueCreate(10, sizeof(ProtoEvent));
    if (!EspNowReceiver::init(g_espnowQueue)) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW receiver");
        FatalInitError("ESP-NOW init failed", 10);
    }
    
    // Initialize bounds caching system (de-energize timer)
    BoundsCache::Init();

    // Get default pin configuration
    auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
    tmc51x0::PinActiveLevels active_levels;
    
    // Enable driver logs BEFORE Initialize() so config validation failures are visible.
    // (We may dial this back after init.)
    esp_log_level_set("TMC5160", ESP_LOG_DEBUG);
    esp_log_level_set("SPI", ESP_LOG_INFO);

    // Create SPI communication interface (placement-new into static storage)
    g_spi = new (g_spi_storage) Esp32SPI(tmc51x0_test_config::SPI_HOST, pin_config, 1000000, active_levels);
    auto spi_init_result = g_spi->Initialize();
    if (!spi_init_result) {
        ESP_LOGE(TAG, "Failed to initialize SPI interface (ErrorCode: %d)", static_cast<int>(spi_init_result.Error()));
        FatalInitError("SPI init failed", 11);
    }

    // Create TMC51x0 driver instance (placement-new into static storage)
    using DriverT = tmc51x0::TMC51x0<Esp32SPI>;
    DriverT* driver = new (g_driver_storage) DriverT(*g_spi);

    // Configure driver from test rig
    tmc51x0::DriverConfig cfg{};
    tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);

    ESP_LOGI(TAG,
             "DriverConfig snapshot: supply_voltage_mv=%u sense_resistor_mohm=%u rated_current_ma=%u run_current_ma=%u hold_current_ma=%u",
             cfg.motor_spec.supply_voltage_mv,
             cfg.motor_spec.sense_resistor_mohm,
             cfg.motor_spec.rated_current_ma,
             cfg.motor_spec.run_current_ma,
             cfg.motor_spec.hold_current_ma);
    
    auto driver_init_result = driver->Initialize(cfg);
    if (!driver_init_result) {
        ESP_LOGE(TAG, "Failed to initialize TMC51x0 driver (ErrorCode: %d)", static_cast<int>(driver_init_result.Error()));
        // Now that the driver propagates underlying errors, this ErrorCode should be meaningful.
        FatalInitError("TMC51x0 Initialize() failed", 12);
    }

    // From this point forward, global pointers are safe to use.
    g_driver = driver;

    ESP_LOGI(TAG, "Driver initialized successfully");

    // Configure logging levels:
    // - Homing: VERBOSE for bounds finding SG telemetry
    // - TMC5160: INFO (Debug/Verbose suppressed to avoid ramp position spam)
    esp_log_level_set("Homing", ESP_LOG_VERBOSE);
    esp_log_level_set("TMC5160", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Logging configured: Homing=VERBOSE, TMC5160=INFO (ramp spam suppressed)");

    // Configure encoder
    tmc51x0::EncoderConfig enc_cfg = 
        tmc51x0_test_config::GetTestRigEncoderConfig<SELECTED_TEST_RIG>();
    
    auto encoder_cfg_result = driver->encoder.Configure(enc_cfg);
    if (!encoder_cfg_result) {
        ESP_LOGE(TAG, "Failed to configure encoder (ErrorCode: %d)", static_cast<int>(encoder_cfg_result.Error()));
        FatalInitError("Encoder configure failed", 13);
    }

    driver->encoder.SetResolution(
        tmc51x0_test_config::GetTestRigEncoderPulsesPerRev<SELECTED_TEST_RIG>(),
        tmc51x0_test_config::GetTestRigEncoderInvertDirection<SELECTED_TEST_RIG>());

    // Motor must NOT be enabled at boot. We only energize during bounds-finding and active motion.
    (void)driver->motorControl.Disable();
    ESP_LOGI(TAG, "Motor left disabled at boot (will enable on START)");

    // Create driver mutex for thread-safe SPI access
    // This mutex protects ALL SPI transactions to the TMC5160
    static Esp32TmcMutex driver_mutex;
    g_driver_mutex = &driver_mutex;
    ESP_LOGI(TAG, "Driver mutex created for thread-safe SPI access");

    // Create motion controller (full implementation)
    // The motion controller works entirely in higher-level units (degrees, RPM, rev/s²)
    // Driver already has motor configuration, so no setup needed
    // Pass the driver mutex so FatigueTestMotion uses the same lock for SPI access
    FatigueTest::FatigueTestMotion* motion = new (g_motion_storage) FatigueTest::FatigueTestMotion(driver, driver_mutex);
    g_motion = motion;

    // Bounds finding will happen when START command is received from UI board
    ESP_LOGI(TAG, "Waiting for START command from UI board to find bounds...");

    // Initialize UART command parser
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              Initializing UART Command Interface                             ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    
    // Use static storage duration to ensure parser lives for the entire program lifetime
    // The parser is used by uart_command_task which runs indefinitely
    static UartCommandParser parser(UART_NUM_0);
    
    // Commands are now automatically routed based on word-based command names
    // Supported commands: set, start, stop, reset, status, help
    // Use 'help' command for usage information
    
    ESP_LOGI(TAG, "UART command interface ready on UART_NUM_0 (USB serial)");
    ESP_LOGI(TAG, "Type 'help' for command usage information");

    // CRITICAL: Ensure motor is stopped before creating tasks
    // Tasks will call Update() but it will return early if motion not running
    ESP_LOGI(TAG, "Ensuring motor is stopped before creating tasks...");
    {
        TmcMutexGuard guard(driver_mutex);
        driver->rampControl.Stop();
        driver->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Allow motor to fully stop

    // Create tasks
    // NOTE: motion_control_task will call Update() but Update() checks running_ flag
    // Since motion is not started, Update() will return early and cause no motion
    ESP_LOGI(TAG, "Creating background tasks...");
    xTaskCreate(espnow_command_task, "espnow_cmd", 4096, nullptr, 5, nullptr);
    xTaskCreate(motion_control_task, "motion_ctrl", 8192, nullptr, 5, nullptr);
    xTaskCreate(status_update_task, "status_upd", 4096, nullptr, 3, nullptr);
    xTaskCreate(uart_command_task, "uart_cmd", 4096, &parser, 3, nullptr);
    ESP_LOGI(TAG, "All tasks created");
    
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                    System Ready - Use UART or ESP-NOW Commands               ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    // Main loop (also tracks time elapsed)
    const char* task_name = "main_loop";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = TaskTiming::LOG_INTERVAL_US;
    
    ESP_LOGI(TAG, "[%s] Main loop started", task_name);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
