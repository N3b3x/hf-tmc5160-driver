/**
 * @file bounds_finder_encoder.cpp
 * @brief Encoder-based bounds finder implementation
 */

#include "bounds_finder.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
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
 */
class EncoderBoundsFinder : public IBoundsFinder {
public:
    const char* GetMethodName() const override {
        return "Encoder";
    }

    BoundsResult FindBounds(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        uint16_t steps_per_rev
    ) override {
        ESP_LOGI(TAG, "Starting encoder-based bounds finding...");

        namespace Test = tmc51x0_test_config::TestConfig_17HS4401S;

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
        int32_t enc_baseline = 0;
        if (!driver.encoder.GetPosition(enc_baseline)) {
            ESP_LOGE(TAG, "Failed to read encoder position");
            return BoundsResult(false, 0, 0, false);
        }

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
        float search_speed_rpm = Test::Motion::BOUNDS_SEARCH_SPEED_RPM;
        // Acceleration: use reasonable value in rev/s² (typically 2x the velocity in rev/s)
        float search_velocity_rev_s = search_speed_rpm / 60.0f;
        float search_accel_rev_s2 = search_velocity_rev_s * 2.0f; // 2x velocity for acceleration

        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(search_speed_rpm, tmc51x0::Unit::RPM);
        driver.rampControl.SetAcceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetRampSpeeds(1000.0f, 100.0f, 0.0f, tmc51x0::Unit::Steps);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Find maximum bound (in degrees)
        ESP_LOGI(TAG, "Finding maximum bound...");
        float max_pos_deg = FindBound(driver, TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, enc_baseline, true);
        if (max_pos_deg == 0.0f && !driver.rampControl.IsTargetReached()) {
            ESP_LOGW(TAG, "Max bound search failed or timeout");
        }

        // Find minimum bound (in degrees)
        ESP_LOGI(TAG, "Finding minimum bound...");
        int32_t min_enc_baseline = 0;
        driver.encoder.GetPosition(min_enc_baseline);
        float min_pos_deg = FindBound(driver, -TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG, min_enc_baseline, false);
        if (min_pos_deg == 0.0f && !driver.rampControl.IsTargetReached()) {
            ESP_LOGW(TAG, "Min bound search failed or timeout");
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
        namespace Test = tmc51x0_test_config::TestConfig_17HS4401S;
        constexpr float MIN_MOVEMENT_DEG = 1.0f; // Minimum movement in degrees to avoid false stalls
        constexpr uint32_t ENCODER_STALL_TIMEOUT_MS = 300;
        constexpr int32_t ENCODER_MIN_CHANGE = 5;
        uint32_t timeout_ms = Test::Motion::HOMING_TIMEOUT_MS;

        float start_pos_deg = 0.0f;
        driver.rampControl.GetCurrentPosition(start_pos_deg, tmc51x0::Unit::Deg);
        uint32_t start_time = esp_timer_get_time() / 1000;
        bool motion_started = false;
        int32_t last_enc_pos = enc_baseline;
        uint32_t last_enc_change_time = start_time;

        // Use ABSOLUTE positioning - home was established at bounds finding start
        // target_angle_deg is relative to the established home (0.0°)
        driver.rampControl.SetTargetPosition(target_angle_deg, tmc51x0::Unit::Deg);

        while (true) {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
            if (elapsed > timeout_ms) break;

            float pos_deg = 0.0f;
            driver.rampControl.GetCurrentPosition(pos_deg, tmc51x0::Unit::Deg);
            float delta_deg = fabsf(pos_deg - start_pos_deg);

            if (driver.rampControl.IsTargetReached()) {
                return 0.0f; // No stall
            }

            float vactual = 0.0f;
            driver.rampControl.GetCurrentSpeed(vactual, tmc51x0::Unit::Deg);

            if (!motion_started && delta_deg > 0.5f) { // ~0.5 degrees movement threshold
                motion_started = true;
            }

            // Encoder-based stall detection
            int32_t enc_pos = 0;
            if (driver.encoder.GetPosition(enc_pos)) {
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
                        driver.rampControl.Stop();
                        vTaskDelay(pdMS_TO_TICKS(200));

                        // Back off relative to current position (in degrees)
                        // Use RELATIVE positioning here - we don't know exact position after stall,
                        // and relative movement is safer and more intuitive for backoff operations
                        float backoff_offset_deg = is_max ? -offset_angle_deg : offset_angle_deg;

                        // Back off at half speed (in RPM)
                        float backoff_speed_rpm = search_speed_rpm / 2.0f;
                        
                        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
                        driver.rampControl.MoveRelative(backoff_offset_deg, tmc51x0::Unit::Deg);
                        driver.rampControl.SetMaxSpeed(backoff_speed_rpm, tmc51x0::Unit::RPM);
                        while (!driver.rampControl.IsTargetReached()) {
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }

                        float final_pos_deg = 0.0f;
                        driver.rampControl.GetCurrentPosition(final_pos_deg, tmc51x0::Unit::Deg);
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
        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
        driver.rampControl.SetMaxSpeed(speed_rpm, tmc51x0::Unit::RPM);
        driver.rampControl.SetAcceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
        while (!driver.rampControl.IsTargetReached()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
};

// Factory function
std::unique_ptr<IBoundsFinder> CreateEncoderBoundsFinder() {
    return std::make_unique<EncoderBoundsFinder>();
}

} // namespace FatigueTest
