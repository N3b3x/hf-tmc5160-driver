/**
 * @file bounds_finder_encoder.cpp
 * @brief Encoder-based bounds finder implementation
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

static const char* TAG = "BoundsFinderENC";

namespace FatigueTest {

/**
 * @brief Encoder-based bounds finder
 * 
 * Template-based implementation that automatically selects test config based on test rig type.
 */
template<tmc51x0_test_config::TestRigType test_rig>
class EncoderBoundsFinderImpl : public IBoundsFinder {
public:
    // Get test config for this test rig's motor type
    using TestConfig = tmc51x0_test_config::GetTestConfigForTestRig<test_rig>;
    
    const char* GetMethodName() const override {
        return "Encoder";
    }

    BoundsResult FindBounds(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        uint16_t steps_per_rev
    ) override {
        ESP_LOGI(TAG, "Starting encoder-based bounds finding...");

        // Disable reference switches
        tmc51x0::ReferenceSwitchConfig ref_cfg{};
        ref_cfg.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
        ref_cfg.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
        ref_cfg.left_switch_stop_enable = false;
        ref_cfg.right_switch_stop_enable = false;
        ref_cfg.latch_left = tmc51x0::ReferenceLatchMode::DISABLED;
        ref_cfg.latch_right = tmc51x0::ReferenceLatchMode::DISABLED;
        driver.rampControl.ConfigureReferenceSwitch(ref_cfg);

        // Read initial encoder position
        auto enc_pos_result = driver.encoder.GetPosition();
        if (!enc_pos_result) {
            ESP_LOGE(TAG, "❌ Failed to read encoder position (ErrorCode: %d)", static_cast<int>(enc_pos_result.Error()));
            ESP_LOGE(TAG, "   Check: Encoder connection and SPI communication");
            return BoundsResult(false, 0, 0, false);
        }
        int32_t enc_baseline = enc_pos_result.Value();
        ESP_LOGI(TAG, "✓ Initial encoder position: %ld", enc_baseline);

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

        // BOUNDS_SEARCH_SPEED is in RPM - use directly, driver handles all conversions
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
        float max_pos_deg = FindBound(driver, TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, enc_baseline, true);
        if (max_pos_deg == 0.0f && !driver.rampControl.IsTargetReached()) {
            ESP_LOGW(TAG, "Max bound search failed or timeout");
        }

        // Find minimum bound (in degrees)
        ESP_LOGI(TAG, "Finding minimum bound...");
        auto min_enc_result = driver.encoder.GetPosition();
        int32_t min_enc_baseline = 0;
        if (!min_enc_result) {
            ESP_LOGW(TAG, "⚠ Failed to read encoder position (ErrorCode: %d), using previous baseline", static_cast<int>(min_enc_result.Error()));
            min_enc_baseline = enc_baseline;
        } else {
            min_enc_baseline = min_enc_result.Value();
        }
        float min_pos_deg = FindBound(driver, -TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, min_enc_baseline, false);
        auto min_reached_result = driver.rampControl.IsTargetReached();
        if (min_pos_deg == 0.0f && (!min_reached_result || !min_reached_result.Value())) {
            ESP_LOGW(TAG, "⚠ Min bound search failed or timeout");
            if (!min_reached_result) {
                ESP_LOGW(TAG, "   Error reading target status (ErrorCode: %d)", static_cast<int>(min_reached_result.Error()));
            }
        }

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
            float center_deg = (min_pos_deg + max_pos_deg) / 2.0f;
            MoveToPosition(driver, center_deg, search_speed_rpm);
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
        int32_t enc_baseline,
        bool is_max
    ) {
        constexpr float MIN_MOVEMENT_DEG = 1.0f; // Minimum movement in degrees to avoid false stalls
        constexpr uint32_t ENCODER_STALL_TIMEOUT_MS = 300;
        constexpr int32_t ENCODER_MIN_CHANGE = 5;
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
        int32_t last_enc_pos = enc_baseline;
        uint32_t last_enc_change_time = start_time;

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

            auto speed_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::Deg);
            float vactual = 0.0f;
            if (!speed_result) {
                ESP_LOGW(TAG, "⚠ Failed to read speed (ErrorCode: %d), using 0", static_cast<int>(speed_result.Error()));
                vactual = 0.0f;
            } else {
                vactual = speed_result.Value();
            }

            if (!motion_started && delta_deg > 0.5f) { // ~0.5 degrees movement threshold
                motion_started = true;
                ESP_LOGI(TAG, "✓ Motion started (delta: %.2f°)", delta_deg);
            }

            // Encoder-based stall detection
            auto enc_pos_result = driver.encoder.GetPosition();
            if (enc_pos_result) {
                int32_t enc_pos = enc_pos_result.Value();
                int32_t enc_diff = enc_pos - last_enc_pos;
                int32_t enc_change = (enc_diff > 0) ? enc_diff : -enc_diff;
                uint32_t current_time = esp_timer_get_time() / 1000;

                if (enc_change >= ENCODER_MIN_CHANGE) {
                    last_enc_change_time = current_time;
                    last_enc_pos = enc_pos;
                } else if (fabsf(vactual) > 1.0f && motion_started) { // ~1 deg/s velocity threshold
                    uint32_t time_since_change = current_time - last_enc_change_time;
                    if (time_since_change >= ENCODER_STALL_TIMEOUT_MS) {
                        if (delta_deg < MIN_MOVEMENT_DEG) {
                            ESP_LOGW(TAG, "False encoder stall, continuing...");
                            last_enc_change_time = current_time;
                            continue;
                        }

                        // Real stall detected
                        ESP_LOGI(TAG, "✓ Encoder stall detected at %.2f° (delta: %.2f°, enc_change: %ld)", 
                                 pos_deg, delta_deg, enc_pos - last_enc_pos);
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
                } else {
                    last_enc_change_time = current_time;
                }
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
        auto ramp_mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        if (!ramp_mode_result) {
            ESP_LOGW(TAG, "⚠ Failed to set ramp mode (ErrorCode: %d)", static_cast<int>(ramp_mode_result.Error()));
        }
        auto target_result = driver.rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
        if (!target_result) {
            ESP_LOGW(TAG, "⚠ Failed to set target position (ErrorCode: %d)", static_cast<int>(target_result.Error()));
        }
        auto speed_result = driver.rampControl.SetMaxSpeed(speed_rpm, tmc51x0::Unit::RPM);
        if (!speed_result) {
            ESP_LOGW(TAG, "⚠ Failed to set max speed (ErrorCode: %d)", static_cast<int>(speed_result.Error()));
        }
        auto accel_result = driver.rampControl.SetAcceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
        if (!accel_result) {
            ESP_LOGW(TAG, "⚠ Failed to set acceleration (ErrorCode: %d)", static_cast<int>(accel_result.Error()));
        }
        auto decel_result = driver.rampControl.SetDeceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
        if (!decel_result) {
            ESP_LOGW(TAG, "⚠ Failed to set deceleration (ErrorCode: %d)", static_cast<int>(decel_result.Error()));
        }
        
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
std::unique_ptr<IBoundsFinder> CreateEncoderBoundsFinder() {
    return std::make_unique<EncoderBoundsFinderImpl<test_rig>>();
}

// Explicit template instantiations for supported test rigs
template std::unique_ptr<IBoundsFinder> CreateEncoderBoundsFinder<tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER>();
template std::unique_ptr<IBoundsFinder> CreateEncoderBoundsFinder<tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE>();

} // namespace FatigueTest
