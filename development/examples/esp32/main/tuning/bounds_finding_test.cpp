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

// Resolved test configuration for the selected rig (compile-time).
using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<SELECTED_TEST_RIG>;

//=============================================================================
// BOUNDS FINDING PARAMETERS - Easy to adjust
//=============================================================================
// Direction search order:
// - true: find MAX bound first (search + direction first), then MIN
// - false: find MIN bound first (search - direction first), then MAX
static constexpr bool BOUNDS_SEARCH_POSITIVE_FIRST = true;

// Search velocity (RPM) - how fast to search for bounds
// Default: use test rig config so this tool matches fatigue test behavior.
static constexpr bool OVERRIDE_BOUNDS_SEARCH_SPEED = false;
static constexpr float BOUNDS_SEARCH_SPEED_RPM_OVERRIDE = 60.0f;
static constexpr float BOUNDS_SEARCH_SPEED_RPM =
    OVERRIDE_BOUNDS_SEARCH_SPEED ? BOUNDS_SEARCH_SPEED_RPM_OVERRIDE : TestConfig::Motion::BOUNDS_SEARCH_SPEED_RPM;

// Search acceleration (rev/s²) - how fast to ramp up to search speed
static constexpr bool OVERRIDE_BOUNDS_SEARCH_ACCEL = false;
static constexpr float BOUNDS_SEARCH_ACCEL_REV_S2_OVERRIDE = 15.0f;
static constexpr float BOUNDS_SEARCH_ACCEL_REV_S2 =
    OVERRIDE_BOUNDS_SEARCH_ACCEL ? BOUNDS_SEARCH_ACCEL_REV_S2_OVERRIDE : TestConfig::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;

// Search span (degrees) - maximum distance to search in each direction
static constexpr float BOUNDS_SEARCH_SPAN_DEG = 400.0f;

// Backoff distance (degrees) - how far to back off after hitting a limit
static constexpr float BOUNDS_BACKOFF_DEG = 0.0f;

// Timeout (ms) - maximum time to search before giving up
static constexpr uint32_t BOUNDS_TIMEOUT_MS = 30000;

// StallGuard parameters:
// By default these are sourced from the selected test rig config to keep this tool
// consistent with the fatigue test behavior.
// If you want to experiment locally without changing the shared test config, enable overrides below.
static constexpr bool OVERRIDE_STALLGUARD_MIN_VELOCITY = false;
static constexpr float STALLGUARD_MIN_VELOCITY_RPM_OVERRIDE = 20.0f;
static constexpr float STALLGUARD_MIN_VELOCITY_RPM =
    OVERRIDE_STALLGUARD_MIN_VELOCITY ? STALLGUARD_MIN_VELOCITY_RPM_OVERRIDE
                                     : TestConfig::StallGuard::MIN_VELOCITY_RPM;

static constexpr bool OVERRIDE_STALL_DETECTION_CURRENT_FACTOR = false;
static constexpr float STALL_DETECTION_CURRENT_FACTOR_OVERRIDE = 0.5f;
// Current reduction factor for stall detection (0.0 = no reduction, 0.3 = 30% of rated current)
static constexpr float STALL_DETECTION_CURRENT_FACTOR =
    OVERRIDE_STALL_DETECTION_CURRENT_FACTOR ? STALL_DETECTION_CURRENT_FACTOR_OVERRIDE
                                            : TestConfig::StallGuard::STALL_DETECTION_CURRENT_FACTOR;

// Use StallGuard (true) or Encoder (false) for bounds detection
static constexpr bool USE_STALLGUARD = true;

// Home placement after bounds finding:
// - AtCenter: Home to center of range (default, symmetric bounds)
// - AtMin: Home to MIN bound (0 = MIN, positive = toward MAX)
// - AtMax: Home to MAX bound (0 = MAX, negative = toward MIN)
// - None: Don't home, stay at last stall position
// Use AtMin or AtMax to test if the 3.6° offset issue is related to centering calculation.
enum class TestHomePlacement { AtCenter, AtMin, AtMax, None };
static constexpr TestHomePlacement HOME_PLACEMENT_MODE = TestHomePlacement::AtCenter;  // <-- CHANGE THIS TO TEST

//=============================================================================
// POST-BOUNDS MOTION VERIFICATION (OPTIONAL)
//=============================================================================
// After successful bounded results, optionally oscillate between min/max bounds
// to verify normal motion between endpoints.
static constexpr bool ENABLE_POST_BOUNDS_OSCILLATION = true;

// Number of oscillations to perform. One oscillation = move to max, then move to min.
static constexpr uint32_t POST_BOUNDS_OSCILLATION_COUNT = 5;

// Motion parameters for oscillation
static constexpr float POST_BOUNDS_OSC_SPEED_RPM = 60.0f;
static constexpr float POST_BOUNDS_OSC_ACCEL_REV_S2 = 5.0f;

// Edge backoff (degrees): how far inside the bounds to travel during oscillation.
// This prevents repeatedly "kissing" the endpoints (and gives margin for any residual error).
static constexpr float POST_BOUNDS_OSC_EDGE_BACKOFF_DEG = 3.5f;

// Per-leg timeout (safety): if we can't reach a bound, stop oscillation
static constexpr uint32_t POST_BOUNDS_OSC_LEG_TIMEOUT_MS = 15000;
static constexpr uint32_t POST_BOUNDS_OSC_POLL_MS = 20;
static constexpr uint32_t POST_BOUNDS_OSC_SETTLE_MS = 200;

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Starting Bounds Finding Test Tool");
    ESP_LOGI(TAG, "==========================================");

    // Log configuration
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  Search Order: %s first", BOUNDS_SEARCH_POSITIVE_FIRST ? "MAX (+)" : "MIN (-)");
    ESP_LOGI(TAG, "  Search Speed: %.1f RPM", BOUNDS_SEARCH_SPEED_RPM);
    ESP_LOGI(TAG, "  Search Accel: %.1f rev/s²", BOUNDS_SEARCH_ACCEL_REV_S2);
    ESP_LOGI(TAG, "  Search Span: %.1f degrees", BOUNDS_SEARCH_SPAN_DEG);
    ESP_LOGI(TAG, "  Backoff: %.1f degrees", BOUNDS_BACKOFF_DEG);
    ESP_LOGI(TAG, "  Timeout: %lu ms", static_cast<unsigned long>(BOUNDS_TIMEOUT_MS));
    ESP_LOGI(TAG, "  Method: %s", USE_STALLGUARD ? "StallGuard" : "Encoder");
    if (USE_STALLGUARD) {
        ESP_LOGI(TAG, "  SG Min Velocity: %.1f RPM%s", STALLGUARD_MIN_VELOCITY_RPM,
                 OVERRIDE_STALLGUARD_MIN_VELOCITY ? " (override)" : " (from test rig)");
        ESP_LOGI(TAG, "  Current Reduction: %.0f%%%s", STALL_DETECTION_CURRENT_FACTOR * 100.0f,
                 OVERRIDE_STALL_DETECTION_CURRENT_FACTOR ? " (override)" : " (from test rig)");
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

    // Configure bounds options
    Homing::BoundsOptions opt{};
    opt.speed_unit = tmc51x0::Unit::RPM;
    opt.position_unit = tmc51x0::Unit::Deg;
    opt.search_speed = BOUNDS_SEARCH_SPEED_RPM;
    opt.search_span = BOUNDS_SEARCH_SPAN_DEG;
    opt.backoff_distance = BOUNDS_BACKOFF_DEG;
    opt.timeout_ms = BOUNDS_TIMEOUT_MS;
    opt.search_positive_first = BOUNDS_SEARCH_POSITIVE_FIRST;
    opt.search_accel = BOUNDS_SEARCH_ACCEL_REV_S2;
    opt.search_decel = BOUNDS_SEARCH_ACCEL_REV_S2;
    opt.accel_unit = tmc51x0::Unit::RevPerSec;

    // Configure home placement (selectable via HOME_PLACEMENT_MODE at top of file)
    Homing::HomeConfig home{};
    switch (HOME_PLACEMENT_MODE) {
        case TestHomePlacement::AtCenter: home.mode = Homing::HomePlacement::AtCenter; break;
        case TestHomePlacement::AtMin:    home.mode = Homing::HomePlacement::AtMin; break;
        case TestHomePlacement::AtMax:    home.mode = Homing::HomePlacement::AtMax; break;
        case TestHomePlacement::None:     home.mode = Homing::HomePlacement::None; break;
    }
    
    ESP_LOGI(TAG, "Home Placement: %s", 
             HOME_PLACEMENT_MODE == TestHomePlacement::AtCenter ? "CENTER" :
             HOME_PLACEMENT_MODE == TestHomePlacement::AtMin ? "MIN" :
             HOME_PLACEMENT_MODE == TestHomePlacement::AtMax ? "MAX" : "NONE");

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

            // Optional: verify motion between bounds by oscillating between endpoints.
            if constexpr (ENABLE_POST_BOUNDS_OSCILLATION) {
                if (POST_BOUNDS_OSCILLATION_COUNT > 0) {
                    ESP_LOGI(TAG, "==========================================");
                    ESP_LOGI(TAG, "Post-bounds motion verification (oscillation)");
                    ESP_LOGI(TAG, "==========================================");
                    ESP_LOGI(TAG, "Oscillations: %lu (one oscillation = max -> min)", static_cast<unsigned long>(POST_BOUNDS_OSCILLATION_COUNT));
                    ESP_LOGI(TAG, "Speed: %.1f RPM, Accel/Decel: %.1f rev/s²", POST_BOUNDS_OSC_SPEED_RPM, POST_BOUNDS_OSC_ACCEL_REV_S2);

                    // Safety: ensure stop-on-stall is disabled so it can't interrupt verification moves.
                    (void)driver.stallGuard.EnableStopOnStall(false);

                    // For verification motion, restore quiet/smooth mode.
                    // We force StealthChop ON and clear StallGuard/CoolStep velocity thresholds so we don't
                    // accidentally remain in SpreadCycle due to earlier StallGuard setup.
                    (void)driver.motorControl.SetStealthChopEnabled(true);
                    (void)driver.thresholds.SetModeChangeSpeeds(0.0f, 0.0f, 0.0f, tmc51x0::Unit::RPM);

                    // Configure a deterministic positioning profile for verification.
                    (void)driver.rampControl.Stop();
                    vTaskDelay(pdMS_TO_TICKS(POST_BOUNDS_OSC_SETTLE_MS));
                    (void)driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
                    (void)driver.rampControl.SetMaxSpeed(POST_BOUNDS_OSC_SPEED_RPM, tmc51x0::Unit::RPM);
                    (void)driver.rampControl.SetAccelerations(POST_BOUNDS_OSC_ACCEL_REV_S2, POST_BOUNDS_OSC_ACCEL_REV_S2,
                                                             tmc51x0::Unit::RevPerSec);

                    // Compute safe oscillation targets (stay inside bounds by a margin).
                    const float osc_edge_backoff = POST_BOUNDS_OSC_EDGE_BACKOFF_DEG;
                    const float osc_min = result.min_bound + osc_edge_backoff;
                    const float osc_max = result.max_bound - osc_edge_backoff;
                    if (!(osc_min < osc_max)) {
                        ESP_LOGW(TAG,
                                 "Skipping oscillation: bounds range too small after edge backoff "
                                 "(min=%.2f max=%.2f backoff=%.2f -> osc_min=%.2f osc_max=%.2f)",
                                 result.min_bound, result.max_bound, osc_edge_backoff,
                                 osc_min, osc_max);
                    } else {
                        ESP_LOGI(TAG, "Oscillation targets (with edge backoff %.2f deg): min=%.2f max=%.2f",
                                 osc_edge_backoff, osc_min, osc_max);

                    auto wait_reached = [&](uint32_t timeout_ms) -> bool {
                        uint32_t elapsed = 0;
                        while (elapsed < timeout_ms) {
                            auto reached = driver.rampControl.IsTargetReached();
                            if (reached && reached.Value()) return true;
                            vTaskDelay(pdMS_TO_TICKS(POST_BOUNDS_OSC_POLL_MS));
                            elapsed += POST_BOUNDS_OSC_POLL_MS;
                        }
                        return false;
                    };

                    auto log_microstep_position = [&](const char* label) {
                        auto xactual = driver.rampControl.GetCurrentPositionMicrosteps();
                        auto xtarget = driver.rampControl.GetTargetPositionMicrosteps();
                        auto pos_deg = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
                        auto tgt_deg = driver.rampControl.GetTargetPosition(tmc51x0::Unit::Deg);
                        if (xactual && xtarget && pos_deg && tgt_deg) {
                            ESP_LOGI(TAG,
                                     "%s: XTARGET=%" PRId32 " µsteps (%.3f deg), XACTUAL=%" PRId32 " µsteps (%.3f deg), err=%" PRId32 " µsteps (%.3f deg)",
                                     label,
                                     xtarget.Value(), tgt_deg.Value(),
                                     xactual.Value(), pos_deg.Value(),
                                     static_cast<int32_t>(xactual.Value() - xtarget.Value()),
                                     static_cast<float>(pos_deg.Value() - tgt_deg.Value()));
                        } else {
                            ESP_LOGW(TAG, "%s: (failed to read XTARGET/XACTUAL)", label);
                        }
                    };

                    // Starting point is at home position (0). We perform oscillations: go to max, then min.
                    // Home position depends on HOME_PLACEMENT_MODE: center, min, or max.
                    for (uint32_t i = 0; i < POST_BOUNDS_OSCILLATION_COUNT; ++i) {
                        ESP_LOGI(TAG, "Oscillation %lu/%lu: moving to MAX (%.2f deg)",
                                 static_cast<unsigned long>(i + 1),
                                 static_cast<unsigned long>(POST_BOUNDS_OSCILLATION_COUNT),
                                 osc_max);
                        (void)driver.rampControl.SetTargetPosition(osc_max, tmc51x0::Unit::Deg);
                        log_microstep_position("After commanding MAX");
                        if (!wait_reached(POST_BOUNDS_OSC_LEG_TIMEOUT_MS)) {
                            ESP_LOGW(TAG, "Timeout reaching MAX bound during oscillation; stopping verification");
                            log_microstep_position("Timeout at MAX");
                            break;
                        }
                        log_microstep_position("Reached MAX");
                        vTaskDelay(pdMS_TO_TICKS(POST_BOUNDS_OSC_SETTLE_MS));

                        ESP_LOGI(TAG, "Oscillation %lu/%lu: moving to MIN (%.2f deg)",
                                 static_cast<unsigned long>(i + 1),
                                 static_cast<unsigned long>(POST_BOUNDS_OSCILLATION_COUNT),
                                 osc_min);
                        (void)driver.rampControl.SetTargetPosition(osc_min, tmc51x0::Unit::Deg);
                        log_microstep_position("After commanding MIN");
                        if (!wait_reached(POST_BOUNDS_OSC_LEG_TIMEOUT_MS)) {
                            ESP_LOGW(TAG, "Timeout reaching MIN bound during oscillation; stopping verification");
                            log_microstep_position("Timeout at MIN");
                            break;
                        }
                        log_microstep_position("Reached MIN");
                        vTaskDelay(pdMS_TO_TICKS(POST_BOUNDS_OSC_SETTLE_MS));
                    }

                    ESP_LOGI(TAG, "Post-bounds oscillation complete");
                    }
                }
            }
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

