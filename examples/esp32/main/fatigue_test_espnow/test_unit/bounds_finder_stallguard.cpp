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
    // Get test config for this test rig's motor type
    using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<test_rig>;
    
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

        // Enable StallGuard2 stop
        driver.diagnostics.EnableStopOnStall(true);
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

        // BOUNDS_SEARCH_SPEED is now in RPM (not steps/s) - use directly
        // Driver handles microstep conversion internally
        float search_speed_rpm = TestConfig::Motion::BOUNDS_SEARCH_SPEED_RPM;
        // Acceleration: use reasonable value in rev/s² (typically 2x the velocity in rev/s)
        float search_velocity_rev_s = search_speed_rpm / 60.0f;
        float search_accel_rev_s2 = search_velocity_rev_s * 2.0f; // 2x velocity for acceleration

        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(search_speed_rpm, tmc51x0::Unit::RPM);
        driver.rampControl.SetAcceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetRampSpeeds(30.0f, 3.0f, 0.0f, tmc51x0::Unit::RPM); // ~1000/100 steps/s for 200 steps/rev
        vTaskDelay(pdMS_TO_TICKS(100));

        // Find maximum bound (in degrees)
        ESP_LOGI(TAG, "Finding maximum bound...");
        float max_pos_deg = FindBound(driver, TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, true);
        auto max_reached_result = driver.rampControl.IsTargetReached();
        if (max_pos_deg == 0.0f && (!max_reached_result || !max_reached_result.Value())) {
            ESP_LOGW(TAG, "⚠ Max bound search failed or timeout");
            if (!max_reached_result) {
                ESP_LOGW(TAG, "   Error reading target status (ErrorCode: %d)", static_cast<int>(max_reached_result.Error()));
            }
        }

        // Find minimum bound (in degrees)
        ESP_LOGI(TAG, "Finding minimum bound...");
        driver.diagnostics.ClearStallFlag();
        float min_pos_deg = FindBound(driver, -TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, false);
        auto min_reached_result = driver.rampControl.IsTargetReached();
        if (min_pos_deg == 0.0f && (!min_reached_result || !min_reached_result.Value())) {
            ESP_LOGW(TAG, "⚠ Min bound search failed or timeout");
            if (!min_reached_result) {
                ESP_LOGW(TAG, "   Error reading target status (ErrorCode: %d)", static_cast<int>(min_reached_result.Error()));
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

        // Use ABSOLUTE positioning - home was established at bounds finding start
        // target_angle_deg is relative to the established home (0.0°)
        auto target_result = driver.rampControl.SetTargetPosition(target_angle_deg, tmc51x0::Unit::Deg);
        if (!target_result) {
            ESP_LOGE(TAG, "❌ Failed to set target position (ErrorCode: %d)", static_cast<int>(target_result.Error()));
            return 0.0f;
        }

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

            auto reached_result = driver.rampControl.IsTargetReached();
            if (reached_result && reached_result.Value()) {
                ESP_LOGI(TAG, "✓ Target reached without stall");
                return 0.0f; // No stall
            }

            if (!motion_started && delta_deg > 0.5f) { // ~0.5 degrees movement threshold
                motion_started = true;
            }

            if (driver.diagnostics.IsStallDetected()) {
                if (delta_deg < MIN_MOVEMENT_DEG) {
                    ESP_LOGW(TAG, "False stall detected, clearing...");
                    driver.diagnostics.ClearStallFlag();
                    auto mode_result = driver.rampControl.GetRampMode();
                    tmc51x0::RampMode mode = tmc51x0::RampMode::HOLD;
                    if (mode_result) {
                        mode = mode_result.Value();
                    }
                    if (mode != tmc51x0::RampMode::POSITIONING) {
                        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
                    }
                    continue;
                }

                // Real stall detected
                ESP_LOGI(TAG, "✓ Stall detected at %.2f° (delta: %.2f°)", pos_deg, delta_deg);
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

                auto ramp_mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
                if (!ramp_mode_result) {
                    ESP_LOGW(TAG, "⚠ Failed to set ramp mode (ErrorCode: %d)", static_cast<int>(ramp_mode_result.Error()));
                }
                auto move_result = driver.rampControl.MoveRelative(backoff_offset_deg, tmc51x0::Unit::Deg);
                if (!move_result) {
                    ESP_LOGW(TAG, "⚠ Failed to move relative (ErrorCode: %d)", static_cast<int>(move_result.Error()));
                }
                auto speed_result = driver.rampControl.SetMaxSpeed(backoff_speed_rpm, tmc51x0::Unit::RPM);
                if (!speed_result) {
                    ESP_LOGW(TAG, "⚠ Failed to set backoff speed (ErrorCode: %d)", static_cast<int>(speed_result.Error()));
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
        float speed_rev_s = speed_rpm / 60.0f;
        float accel_rev_s2 = speed_rev_s * 2.0f; // 2x velocity for acceleration
        
        // Use ABSOLUTE positioning - home was established before calling this function
        // target_deg is relative to the established home (0.0°)
        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
        driver.rampControl.SetMaxSpeed(speed_rpm, tmc51x0::Unit::RPM);
        driver.rampControl.SetAcceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
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
