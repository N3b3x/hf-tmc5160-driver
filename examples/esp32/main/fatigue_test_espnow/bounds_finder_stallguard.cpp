/**
 * @file bounds_finder_stallguard.cpp
 * @brief StallGuard2-based bounds finder implementation
 */

#include "bounds_finder.hpp"
#include <memory>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>

static const char* TAG = "BoundsFinderSG";

namespace FatigueTest {

/**
 * @brief StallGuard2-based bounds finder
 * 
 * Template-based implementation that automatically selects test config based on test rig type.
 */
template<tmc51x0_test_config::TestRigType test_rig>
class StallGuardBoundsFinderImpl : public IBoundsFinder {
public:
    // Get test config and motor config for this test rig
    using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<test_rig>;
    using RigConfig = tmc51x0_test_config::TestRigConfig<test_rig>;
    using MotorConfig = typename RigConfig::Motor;
    
    const char* GetMethodName() const override {
        return "StallGuard2";
    }

    BoundsResult FindBounds(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        uint16_t steps_per_rev
    ) override {
        ESP_LOGI(TAG, "Starting StallGuard2 bounds finding...");

        // Configure StallGuard2 using test config for selected test rig
        tmc51x0::StallGuardConfig sg_config{};
        sg_config.threshold = TestConfig::StallGuard::SGT_HOMING;
        sg_config.enable_filter = TestConfig::StallGuard::FILTER_ENABLED;
        
        auto sg_config_result = driver.diagnostics.ConfigureStallGuard(sg_config);
        if (!sg_config_result) {
            ESP_LOGE(TAG, "❌ Failed to configure StallGuard2 (ErrorCode: %d)", static_cast<int>(sg_config_result.Error()));
            ESP_LOGE(TAG, "   Check: SPI communication and StallGuard configuration");
            return BoundsResult(false, 0, 0, false);
        }
        ESP_LOGI(TAG, "✓ StallGuard2 configured: SGT=%d, filter=%s", 
                 sg_config.threshold, sg_config.enable_filter ? "enabled" : "disabled");

        // Disable reference switches
        tmc51x0::ReferenceSwitchConfig ref_cfg{};
        ref_cfg.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
        ref_cfg.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
        ref_cfg.left_switch_stop_enable = false;
        ref_cfg.right_switch_stop_enable = false;
        ref_cfg.latch_left = tmc51x0::ReferenceLatchMode::DISABLED;
        ref_cfg.latch_right = tmc51x0::ReferenceLatchMode::DISABLED;
        driver.rampControl.ConfigureReferenceSwitch(ref_cfg);

        // Disable automatic StallGuard2 stop - we'll handle stalls manually
        // This prevents the motor from stopping automatically and causing vibrations
        // when we clear false stalls
        driver.diagnostics.EnableStopOnStall(false);
        driver.rampControl.SetStopMode(tmc51x0::ReferenceStopMode::HARD_STOP);
        driver.diagnostics.ClearStallFlag();

        // Establish home/zero position at current location
        // IMPORTANT: After this point, we use ABSOLUTE positioning (SetTargetPosition)
        // Before establishing home, we would use RELATIVE positioning (MoveRelative)
        // This allows us to work with absolute positions even when true mechanical home is unknown
        driver.rampControl.Stop();
        driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
        vTaskDelay(pdMS_TO_TICKS(100));
        driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);

        // Configure positioning mode - use degrees for all position operations
        constexpr float TARGET_ANGLE_DEG = 360.0f; // One full revolution
        constexpr float OFFSET_ANGLE_DEG = 5.0f;   // Back off angle in degrees
        constexpr float INITIAL_BACKOFF_DEG = 30.0f; // Initial backoff to clear any existing bounds

        // Use motor config velocities and accelerations
        // Driver handles microstep conversion internally
        float search_speed_rpm = TestConfig::Motion::BOUNDS_SEARCH_SPEED_RPM;
        // Use motor config acceleration (RAMP_AMAX) for bounds finding
        float search_accel_rev_s2 = MotorConfig::RAMP_AMAX_REV_S2;

        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(search_speed_rpm, tmc51x0::Unit::RPM);
        driver.rampControl.SetAcceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetRampSpeeds(30.0f, 3.0f, 0.0f, tmc51x0::Unit::RPM); // ~1000/100 steps/s for 200 steps/rev
        vTaskDelay(pdMS_TO_TICKS(100));

        // Initial backoff to ensure we're not already at a bound
        // Use relative positioning to move away from current position
        ESP_LOGI(TAG, "Performing initial backoff (%.1f°) to clear any existing bounds...", INITIAL_BACKOFF_DEG);
        driver.rampControl.SetMaxSpeed(search_speed_rpm / 2.0f, tmc51x0::Unit::RPM); // Half speed for backoff
        auto backoff_result = driver.rampControl.MoveRelative(-INITIAL_BACKOFF_DEG, tmc51x0::Unit::Deg);
        if (backoff_result) {
            // Wait for backoff to complete (with timeout)
            uint32_t backoff_start = esp_timer_get_time() / 1000;
            while ((esp_timer_get_time() / 1000 - backoff_start) < 5000) { // 5 second timeout
                auto reached = driver.rampControl.IsTargetReached();
                if (reached && reached.Value()) break;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            // Re-establish position after backoff (we moved -30°, so new home is at -30°)
            // But we want to keep 0° as home, so we need to adjust
            auto backoff_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
            if (backoff_pos_result) {
                float backoff_pos = backoff_pos_result.Value();
                driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg); // Reset to 0° as home
                ESP_LOGI(TAG, "✓ Initial backoff completed (moved from 0° to %.2f°, reset to 0°)", backoff_pos);
            } else {
                ESP_LOGW(TAG, "⚠ Failed to read position after backoff, assuming backoff completed");
                driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
            }
        } else {
            ESP_LOGW(TAG, "⚠ Failed to start initial backoff (ErrorCode: %d), continuing anyway", 
                     static_cast<int>(backoff_result.Error()));
            }
        // Restore full search speed
        driver.rampControl.SetMaxSpeed(search_speed_rpm, tmc51x0::Unit::RPM);
        vTaskDelay(pdMS_TO_TICKS(200)); // Give motor time to settle

        // Find minimum bound FIRST (in degrees) - search backward
        ESP_LOGI(TAG, "Finding minimum bound (backward search)...");
        float min_pos_deg = FindBound(driver, -TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, false);
        auto min_reached_result = driver.rampControl.IsTargetReached();
        if (min_pos_deg == 0.0f && (!min_reached_result || !min_reached_result.Value())) {
            ESP_LOGW(TAG, "⚠ Min bound search failed or timeout");
            if (!min_reached_result) {
                ESP_LOGW(TAG, "   Error reading target status (ErrorCode: %d)", static_cast<int>(min_reached_result.Error()));
            }
        }

        // Find maximum bound SECOND (in degrees) - search forward
        // Add delay between searches to let motor settle
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "Finding maximum bound (forward search)...");
        driver.diagnostics.ClearStallFlag();
        float max_pos_deg = FindBound(driver, TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, true);
        auto max_reached_result = driver.rampControl.IsTargetReached();
        if (max_pos_deg == 0.0f && (!max_reached_result || !max_reached_result.Value())) {
            ESP_LOGW(TAG, "⚠ Max bound search failed or timeout");
            if (!max_reached_result) {
                ESP_LOGW(TAG, "   Error reading target status (ErrorCode: %d)", static_cast<int>(max_reached_result.Error()));
            }
        }

        // Disable StallGuard2 stop for normal operation
        driver.diagnostics.EnableStopOnStall(false);

        // Process results - convert degrees to steps for BoundsResult
        bool max_stall = (max_pos_deg != 0.0f);
        bool min_stall = (min_pos_deg != 0.0f);
        bool reached_360 = (!max_stall && !min_stall);

        if (reached_360) {
            // No stalls - use default bounds (in degrees)
            constexpr float bounds_deg = 175.0f;
            return BoundsResult(true, -bounds_deg, bounds_deg, false);
        } else if (max_stall && min_stall) {
            // Both stalls detected - move to center (in degrees)
            // Calculate center position relative to current zero
            float center_deg = (min_pos_deg + max_pos_deg) / 2.0f;
            MoveToPosition(driver, center_deg, search_speed_rpm);
            // Re-establish center as new home/zero for future operations
            driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
            
            // Return bounds relative to center (in degrees)
            float min_bound_deg = min_pos_deg - center_deg;
            float max_bound_deg = max_pos_deg - center_deg;
            return BoundsResult(true, min_bound_deg, max_bound_deg, true);
        } else {
            // Partial bounds - return degrees directly
            return BoundsResult(true, min_pos_deg, max_pos_deg, (max_stall && min_stall));
        }
    }

private:
    float FindBound(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        float target_angle_deg,
        float search_speed_rpm,
        float offset_angle_deg,
        bool is_max
    ) {
        constexpr float MIN_MOVEMENT_DEG = 1.0f; // Minimum movement in degrees to avoid false stalls
        uint32_t timeout_ms = TestConfig::Motion::HOMING_TIMEOUT_MS;

        auto start_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        float start_pos_deg = 0.0f;
        if (!start_pos_result) {
            ESP_LOGW(TAG, "⚠ Failed to read start position (ErrorCode: %d), using 0", static_cast<int>(start_pos_result.Error()));
            start_pos_deg = 0.0f;
        } else {
            start_pos_deg = start_pos_result.Value();
        }
        uint32_t start_time = esp_timer_get_time() / 1000;
        bool motion_started = false;

        // Use RELATIVE positioning for initial search move
        // This is more robust if we're already near a bound
        ESP_LOGI(TAG, "Starting bound search using relative positioning (%.1f°)...", target_angle_deg);
        auto move_result = driver.rampControl.MoveRelative(target_angle_deg, tmc51x0::Unit::Deg);
        if (!move_result) {
            ESP_LOGE(TAG, "❌ Failed to start relative move (ErrorCode: %d)", static_cast<int>(move_result.Error()));
            return 0.0f;
        }

        // StallGuard monitoring during bounds finding - log continuously
        uint32_t iteration_count = 0;
        
        ESP_LOGI(TAG, "Starting bound search - StallGuard monitoring active (logging every iteration)");

        while (true) {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
            if (elapsed > timeout_ms) {
                ESP_LOGW(TAG, "⚠ Bound search timeout after %u ms", timeout_ms);
                break;
            }

            auto pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
            float pos_deg = 0.0f;
            if (!pos_result) {
                ESP_LOGW(TAG, "⚠ Failed to read position (ErrorCode: %d)", static_cast<int>(pos_result.Error()));
                pos_deg = start_pos_deg; // Use start position as fallback
            } else {
                pos_deg = pos_result.Value();
            }
            float delta_deg = fabsf(pos_deg - start_pos_deg);
            
            // Log StallGuard values continuously - every iteration
            iteration_count++;
            auto vel_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
            float vel_rpm = vel_result.IsOk() ? vel_result.Value() : 0.0f;
            
            auto sg_result = driver.diagnostics.GetStallGuardResult();
            if (sg_result.IsOk()) {
                uint16_t sg_val = sg_result.Value();
                ESP_LOGI(TAG, "[Bounds] SG: %u, V: %.2f RPM, Pos: %.2f° (Δ: %.2f°, iter: %u, t: %u ms)", 
                         sg_val, vel_rpm, pos_deg, delta_deg, iteration_count, elapsed);
            } else {
                ESP_LOGW(TAG, "[Bounds] Failed to read SG (Error: %d), V: %.2f RPM, Pos: %.2f°, iter: %u", 
                         static_cast<int>(sg_result.Error()), vel_rpm, pos_deg, iteration_count);
            }

            auto reached_result = driver.rampControl.IsTargetReached();
            if (reached_result && reached_result.Value()) {
                ESP_LOGI(TAG, "✓ Target reached without stall");
                return 0.0f; // No stall
            }

            if (!motion_started && delta_deg > 0.5f) { // ~0.5 degrees movement threshold
                motion_started = true;
            }

            if (driver.diagnostics.IsStallDetected()) {
                // Check both movement AND velocity before trusting stall detection
                // Use velocity thresholds that are relative to search speed
                // Use 40% of search speed for steady-state (more lenient to avoid edge cases)
                float min_velocity_threshold = std::abs(search_speed_rpm) * 0.4f; // 40% of search speed for steady-state
                float accel_velocity_threshold = std::abs(search_speed_rpm) * 0.2f; // 20% of search speed during acceleration
                // But also enforce absolute minimums
                // Use lower thresholds to avoid edge cases and allow motor to reach full speed
                constexpr float ABSOLUTE_MIN_VELOCITY_RPM = 20.0f; // For steady-state (lowered to avoid rejecting at 30 RPM)
                constexpr float ABSOLUTE_MIN_ACCEL_VELOCITY_RPM = 10.0f; // For acceleration phase
                
                bool movement_too_small = delta_deg < MIN_MOVEMENT_DEG;
                
                // Check velocity based on movement phase
                // During acceleration (first 5°), use lower threshold
                // After acceleration, use full threshold
                bool in_acceleration_phase = delta_deg < 5.0f;
                float effective_min_velocity = in_acceleration_phase 
                    ? std::max(accel_velocity_threshold, ABSOLUTE_MIN_ACCEL_VELOCITY_RPM)
                    : std::max(min_velocity_threshold, ABSOLUTE_MIN_VELOCITY_RPM);
                
                // Use <= instead of < to avoid edge cases (30.00 vs 30.00)
                bool velocity_too_low = std::abs(vel_rpm) <= effective_min_velocity;
                
                // Additional check: If SG_RESULT=0 but velocity is reasonable, it might be a false positive
                // SG_RESULT=0 at low speeds can be unreliable
                // Only trust SG_RESULT=0 if velocity is high enough (at least 20% of search speed)
                bool sg_result_unreliable = (sg_result.IsOk() && sg_result.Value() == 0 && 
                                            std::abs(vel_rpm) < std::abs(search_speed_rpm) * 0.3f);
                
                // Reject if:
                // 1. Movement is too small (always reject)
                // 2. Velocity is too low (check applies in both phases, just with different thresholds)
                // 3. SG_RESULT=0 but velocity is too low for reliable StallGuard (unreliable reading)
                if (movement_too_small || velocity_too_low || sg_result_unreliable) {
                    if (sg_result_unreliable) {
                        ESP_LOGW(TAG, "False stall detected (SG_RESULT=0 but velocity too low for reliable reading: %.2f RPM < %.2f RPM), clearing...", 
                                 std::abs(vel_rpm), std::abs(search_speed_rpm) * 0.3f);
                    } else if (velocity_too_low && movement_too_small) {
                        ESP_LOGW(TAG, "False stall detected (velocity too low: %.2f RPM < %.2f RPM, movement too small: %.2f° < %.2f°), clearing...", 
                                 std::abs(vel_rpm), effective_min_velocity, delta_deg, MIN_MOVEMENT_DEG);
                    } else if (velocity_too_low) {
                        ESP_LOGW(TAG, "False stall detected (velocity too low: %.2f RPM < %.2f RPM at %.2f° movement [%s]), clearing...", 
                                 std::abs(vel_rpm), effective_min_velocity, delta_deg, 
                                 in_acceleration_phase ? "accel" : "steady");
                    } else {
                        ESP_LOGW(TAG, "False stall detected (movement too small: %.2f° < %.2f°), clearing...", 
                                 delta_deg, MIN_MOVEMENT_DEG);
                    }
                    
                    // Clear the stall flag (motor continues running since EnableStopOnStall is false)
                    driver.diagnostics.ClearStallFlag();
                    
                    // Small delay after clearing false stall to prevent immediate re-detection
                    // No need to restart motion since motor wasn't stopped automatically
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }

                // Real stall detected - velocity is high enough and movement is significant
                ESP_LOGI(TAG, "✓ Stall detected at %.2f° (delta: %.2f°, velocity: %.2f RPM)", 
                         pos_deg, delta_deg, vel_rpm);
                auto stop_result = driver.rampControl.Stop();
                if (!stop_result) {
                    ESP_LOGW(TAG, "⚠ Failed to stop motor (ErrorCode: %d)", static_cast<int>(stop_result.Error()));
                }
                vTaskDelay(pdMS_TO_TICKS(200));

                // Back off relative to current position (in degrees)
                // Use RELATIVE positioning here - we don't know exact position after stall,
                // and relative movement is safer and more intuitive for backoff operations
                float backoff_offset_deg = is_max ? -offset_angle_deg : offset_angle_deg;

                // Back off at half speed (in RPM)
                float backoff_speed_rpm = search_speed_rpm / 2.0f;

                // Set all parameters FIRST while in HOLD mode (motor won't move)
                // This prevents jerky motion from parameter changes during movement
                auto speed_result = driver.rampControl.SetMaxSpeed(backoff_speed_rpm, tmc51x0::Unit::RPM);
                if (!speed_result) {
                    ESP_LOGW(TAG, "⚠ Failed to set backoff speed (ErrorCode: %d)", static_cast<int>(speed_result.Error()));
                }
                // Use motor config acceleration for backoff
                float backoff_accel_rev_s2 = MotorConfig::RAMP_AMAX_REV_S2;
                driver.rampControl.SetAcceleration(backoff_accel_rev_s2, tmc51x0::Unit::RevPerSec);
                driver.rampControl.SetDeceleration(backoff_accel_rev_s2, tmc51x0::Unit::RevPerSec);

                // NOW set to POSITIONING mode and start motion with correct parameters
                auto ramp_mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
                if (!ramp_mode_result) {
                    ESP_LOGW(TAG, "⚠ Failed to set ramp mode (ErrorCode: %d)", static_cast<int>(ramp_mode_result.Error()));
                }
                auto move_result = driver.rampControl.MoveRelative(backoff_offset_deg, tmc51x0::Unit::Deg);
                if (!move_result) {
                    ESP_LOGW(TAG, "⚠ Failed to move relative (ErrorCode: %d)", static_cast<int>(move_result.Error()));
                }
                
                // Wait for backoff to complete
                int backoff_checks = 0;
                constexpr int MAX_BACKOFF_CHECKS = 50;
                while (backoff_checks < MAX_BACKOFF_CHECKS) {
                    auto backoff_reached_result = driver.rampControl.IsTargetReached();
                    if (backoff_reached_result && backoff_reached_result.Value()) {
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                    backoff_checks++;
                }
                if (backoff_checks >= MAX_BACKOFF_CHECKS) {
                    ESP_LOGW(TAG, "⚠ Backoff timeout after %d checks", MAX_BACKOFF_CHECKS);
                }

                // Add delay after backoff to let motor settle before continuing
                vTaskDelay(pdMS_TO_TICKS(300));

                auto final_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
                float final_pos_deg = 0.0f;
                if (!final_pos_result) {
                    ESP_LOGW(TAG, "⚠ Failed to read final position (ErrorCode: %d), using current", static_cast<int>(final_pos_result.Error()));
                    final_pos_deg = pos_deg; // Use position from before backoff
                } else {
                    final_pos_deg = final_pos_result.Value();
                }
                ESP_LOGI(TAG, "✓ Final bound position: %.2f° (after %.2f° backoff)", final_pos_deg, backoff_offset_deg);
                return final_pos_deg;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        return 0.0f;
    }

    void MoveToPosition(tmc51x0::TMC51x0<Esp32SPI>& driver, float target_deg, float speed_rpm) {
        // Speed is already in RPM, calculate acceleration in rev/s²
        // Driver handles microstep conversion internally
        // Use motor config acceleration (RAMP_AMAX) directly
        float accel_rev_s2 = MotorConfig::RAMP_AMAX_REV_S2;
        
        // Use ABSOLUTE positioning - home was established before calling this function
        // target_deg is relative to the established home (0.0°)
        // IMPORTANT: Set all parameters FIRST, then set target position to start motion
        // This prevents jerky motion from parameter changes during movement
        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(speed_rpm, tmc51x0::Unit::RPM);
        driver.rampControl.SetAcceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
        // Set target position LAST to start motion with correct parameters
        driver.rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
        int wait_checks = 0;
        constexpr int MAX_WAIT_CHECKS = 100;
        while (wait_checks < MAX_WAIT_CHECKS) {
            auto reached_result = driver.rampControl.IsTargetReached();
            if (reached_result && reached_result.Value()) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            wait_checks++;
        }
        if (wait_checks >= MAX_WAIT_CHECKS) {
            ESP_LOGW(TAG, "⚠ MoveToPosition timeout after %d checks", MAX_WAIT_CHECKS);
        }
    }
};

// Factory function - template-based to automatically select test config based on test rig
template<tmc51x0_test_config::TestRigType test_rig>
std::unique_ptr<IBoundsFinder> CreateStallGuardBoundsFinder() {
    return std::make_unique<StallGuardBoundsFinderImpl<test_rig>>();
}

// Explicit template instantiations for supported test rigs
template std::unique_ptr<IBoundsFinder> CreateStallGuardBoundsFinder<tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER>();
template std::unique_ptr<IBoundsFinder> CreateStallGuardBoundsFinder<tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE>();

} // namespace FatigueTest
