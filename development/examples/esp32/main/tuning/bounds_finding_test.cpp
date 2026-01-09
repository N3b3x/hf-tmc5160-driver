/**
 * @file bounds_finding_test.cpp
 * @brief Standalone Bounds Finding Test Tool
 *
 * This tool tests the bounds finding functionality by performing a complete
 * bounds finding sequence just like the fatigue test unit does. It:
 *
 * 1. Initializes the driver with the configured motor
 * 2. Enables the motor
 * 3. Performs bounds finding using StallGuard
 * 4. Homes to the center of the found range
 * 5. Reports detailed results
 * 6. Disables the motor
 *
 * USAGE:
 * 1. Ensure the motor is connected and has mechanical limits (hard stops)
 * 2. Flash and run this tool
 * 3. The motor will search for bounds in both directions
 * 4. Results will be logged including min/max bounds and center position
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

// FreeRTOS headers MUST come before tmc51x0.hpp (library uses FreeRTOS macros when ESP_PLATFORM defined)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "BoundsTest";

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// Test rig selection (compile-time constant) - automatically selects motor, board, and platform
// FATIGUE TEST RIG: Uses Applied Motion 5034-369 motor, TMC51x0 EVAL board, reference switches, encoder
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;

//=============================================================================
// BOUNDS FINDING PARAMETERS - Easy to adjust
//=============================================================================
// Search velocity (RPM) - how fast to search for bounds
// 60 RPM is above motor resonance for Applied Motion 5034-369
static constexpr float BOUNDS_SEARCH_SPEED_RPM = 60.0f;

// Search acceleration (rev/s²) - how fast to ramp up to search speed
static constexpr float BOUNDS_SEARCH_ACCEL_REV_S2 = 20.0f;

// Search span (degrees) - maximum distance to search in each direction
static constexpr float BOUNDS_SEARCH_SPAN_DEG = 400.0f;

// Backoff distance (degrees) - how far to back off after hitting a limit
static constexpr float BOUNDS_BACKOFF_DEG = 5.0f;

// Timeout (ms) - maximum time to search before giving up
static constexpr uint32_t BOUNDS_TIMEOUT_MS = 30000;

// StallGuard minimum velocity (RPM) - SG not valid below this
static constexpr float STALLGUARD_MIN_VELOCITY_RPM = 20.0f;

// Current reduction factor for stall detection (0.0 = no reduction, 0.3 = 30% reduction)
static constexpr float STALL_DETECTION_CURRENT_FACTOR = 0.3f;

// Use StallGuard (true) or Encoder (false) for bounds detection
static constexpr bool USE_STALLGUARD = true;

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Starting Bounds Finding Test Tool");
    ESP_LOGI(TAG, "==========================================");

    // Log configuration
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  Search Speed: %.1f RPM", BOUNDS_SEARCH_SPEED_RPM);
    ESP_LOGI(TAG, "  Search Accel: %.1f rev/s²", BOUNDS_SEARCH_ACCEL_REV_S2);
    ESP_LOGI(TAG, "  Search Span: %.1f degrees", BOUNDS_SEARCH_SPAN_DEG);
    ESP_LOGI(TAG, "  Backoff: %.1f degrees", BOUNDS_BACKOFF_DEG);
    ESP_LOGI(TAG, "  Timeout: %lu ms", static_cast<unsigned long>(BOUNDS_TIMEOUT_MS));
    ESP_LOGI(TAG, "  Method: %s", USE_STALLGUARD ? "StallGuard" : "Encoder");
    if (USE_STALLGUARD) {
        ESP_LOGI(TAG, "  SG Min Velocity: %.1f RPM", STALLGUARD_MIN_VELOCITY_RPM);
        ESP_LOGI(TAG, "  Current Reduction: %.0f%%", STALL_DETECTION_CURRENT_FACTOR * 100.0f);
    }

    // Enable verbose logging for homing subsystem
    esp_log_level_set("Homing", ESP_LOG_VERBOSE);
    esp_log_level_set("TMC5160", ESP_LOG_INFO);

    // 1. Initialize SPI Bus
    ESP_LOGI(TAG, "Initializing SPI bus...");
    auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
    Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, tmc51x0_test_config::SPI_CLOCK_SPEED_HZ);

    auto spi_init_result = spi.Initialize();
    if (!spi_init_result) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus (ErrorCode: %d)", static_cast<int>(spi_init_result.Error()));
        return;
    }
    ESP_LOGI(TAG, "SPI bus initialized");

    // 2. Initialize Driver
    ESP_LOGI(TAG, "Initializing TMC51x0 driver...");
    tmc51x0::TMC51x0<Esp32SPI> driver(spi);
    tmc51x0::DriverConfig driver_config;

    // Configure driver from unified test rig selection
    tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(driver_config);

    if (!driver.Initialize(driver_config)) {
        ESP_LOGE(TAG, "Failed to initialize driver");
        return;
    }
    ESP_LOGI(TAG, "Driver initialized");

    // 3. Verify Setup
    if (!driver.status.VerifySetup()) {
        ESP_LOGE(TAG, "Setup verification failed");
        return;
    }
    ESP_LOGI(TAG, "Setup verified");

    // 4. Enable Motor
    ESP_LOGI(TAG, "Enabling motor...");
    if (!driver.motorControl.Enable()) {
        ESP_LOGE(TAG, "Failed to enable motor");
        return;
    }
    ESP_LOGI(TAG, "Motor enabled");

    // Ensure motor is stopped before starting bounds finding
    driver.rampControl.Stop();
    vTaskDelay(pdMS_TO_TICKS(500));

    // 5. Configure SpreadCycle for StallGuard
    if (USE_STALLGUARD) {
        driver.motorControl.SetStealthChopEnabled(false);
        // Set TCOOLTHRS so StallGuard is active at search velocity
        driver.thresholds.SetModeChangeSpeeds(0.0f, STALLGUARD_MIN_VELOCITY_RPM, 0.0f, tmc51x0::Unit::RPM);
    }

    // 6. Perform Bounds Finding
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Starting Bounds Finding...");
    ESP_LOGI(TAG, "==========================================");

    using DriverT = tmc51x0::TMC51x0<Esp32SPI>;
    using Homing = DriverT::Homing;
    using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<SELECTED_TEST_RIG>;

    // Configure bounds options
    Homing::BoundsOptions opt{};
    opt.speed_unit = tmc51x0::Unit::RPM;
    opt.position_unit = tmc51x0::Unit::Deg;
    opt.search_speed = BOUNDS_SEARCH_SPEED_RPM;
    opt.search_span = BOUNDS_SEARCH_SPAN_DEG;
    opt.backoff_distance = BOUNDS_BACKOFF_DEG;
    opt.timeout_ms = BOUNDS_TIMEOUT_MS;
    opt.search_accel = BOUNDS_SEARCH_ACCEL_REV_S2;
    opt.search_decel = BOUNDS_SEARCH_ACCEL_REV_S2;
    opt.accel_unit = tmc51x0::Unit::RevPerSec;

    // Configure home placement at center
    Homing::HomeConfig home{};
    home.mode = Homing::HomePlacement::AtCenter;

    // Select bounds method
    Homing::BoundsMethod method = USE_STALLGUARD ? Homing::BoundsMethod::StallGuard : Homing::BoundsMethod::Encoder;

    // StallGuard override configuration
    tmc51x0::StallGuardConfig sg_override{};
    if (USE_STALLGUARD) {
        sg_override.threshold = TestConfig::StallGuard::SGT_HOMING;
        // IMPORTANT: Enable filter for bounds finding to reduce false stalls during
        // acceleration/deceleration. The filter averages SG readings over 4 samples.
        sg_override.enable_filter = true;
        sg_override.min_velocity = STALLGUARD_MIN_VELOCITY_RPM;
        sg_override.max_velocity = 0.0f;
        sg_override.velocity_unit = tmc51x0::Unit::RPM;
        opt.stallguard_override = &sg_override;
        opt.current_reduction_factor = STALL_DETECTION_CURRENT_FACTOR;
        
        ESP_LOGI(TAG, "StallGuard Config: SGT=%d, Filter=%s, MinVel=%.1f RPM",
                 sg_override.threshold, sg_override.enable_filter ? "ON" : "OFF",
                 sg_override.min_velocity);
    } else {
        opt.stallguard_override = nullptr;
        opt.current_reduction_factor = 0.0f;
    }

    // Record start time
    int64_t start_time_ms = esp_timer_get_time() / 1000;

    // Execute bounds finding
    auto lib_res = driver.homing.FindBounds(method, opt, home, nullptr);

    // Record end time
    int64_t end_time_ms = esp_timer_get_time() / 1000;
    int64_t elapsed_ms = end_time_ms - start_time_ms;

    // CRITICAL: Explicitly disable stop-on-stall after bounds finding.
    // The library's RAII guard should restore SW_MODE, but we add this explicit
    // disable as a safety measure.
    auto sg_stop_result = driver.stallGuard.EnableStopOnStall(false);
    if (!sg_stop_result) {
        ESP_LOGW(TAG, "Failed to disable stop-on-stall: err=%d", static_cast<int>(sg_stop_result.Error()));
    } else {
        ESP_LOGI(TAG, "Stop-on-stall disabled after bounds finding");
    }

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "BOUNDS FINDING RESULTS");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Time elapsed: %lld ms", elapsed_ms);

    if (!lib_res) {
        ESP_LOGE(TAG, "❌ BOUNDS FINDING FAILED");
        ESP_LOGE(TAG, "   ErrorCode: %d", static_cast<int>(lib_res.Error()));
        ESP_LOGE(TAG, "   Possible causes:");
        ESP_LOGE(TAG, "     - No mechanical limits found");
        ESP_LOGE(TAG, "     - StallGuard threshold (SGT) too sensitive or not sensitive enough");
        ESP_LOGE(TAG, "     - Search velocity too high or too low");
        ESP_LOGE(TAG, "     - Timeout reached before finding both bounds");
    } else {
        Homing::BoundsResult result = lib_res.Value();

        ESP_LOGI(TAG, "Success: %s", result.success ? "YES" : "NO");
        ESP_LOGI(TAG, "Bounded: %s", result.bounded ? "YES (mechanical limits found)" : "NO (open-ended)");
        ESP_LOGI(TAG, "Cancelled: %s", result.cancelled ? "YES" : "NO");

        if (result.bounded) {
            float range = result.max_bound - result.min_bound;
            float center = (result.min_bound + result.max_bound) / 2.0f;

            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Bounds Found:");
            ESP_LOGI(TAG, "  Min Bound: %.2f degrees", result.min_bound);
            ESP_LOGI(TAG, "  Max Bound: %.2f degrees", result.max_bound);
            ESP_LOGI(TAG, "  Total Range: %.2f degrees", range);
            ESP_LOGI(TAG, "  Center: %.2f degrees", center);

            // Get current position (should be at center after homing)
            auto pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
            if (pos_result) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "Current Position (after homing to center): %.2f degrees", pos_result.Value());
                float error = std::abs(pos_result.Value() - center);
                if (error < 1.0f) {
                    ESP_LOGI(TAG, "  ✓ Position is at center (error: %.2f deg)", error);
                } else {
                    ESP_LOGW(TAG, "  ⚠ Position error from center: %.2f deg", error);
                }
            }

            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "✅ BOUNDS FINDING SUCCESSFUL");
        } else {
            ESP_LOGW(TAG, "No mechanical limits detected - motor may be open-ended or limits too far");
        }
    }

    // 7. Stop and Disable Motor
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Stopping and disabling motor...");
    driver.rampControl.Stop();
    vTaskDelay(pdMS_TO_TICKS(500));

    if (!driver.motorControl.Disable()) {
        ESP_LOGW(TAG, "Warning: Failed to disable motor outputs");
    } else {
        ESP_LOGI(TAG, "Motor disabled successfully");
    }

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Bounds finding test complete.");
    ESP_LOGI(TAG, "Motor is now disabled.");
    ESP_LOGI(TAG, "Reset to run again.");
    ESP_LOGI(TAG, "==========================================");

    // Idle loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

