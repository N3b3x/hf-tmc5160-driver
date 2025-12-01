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

        // Reset position
        driver.rampControl.Stop();
        driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
        vTaskDelay(pdMS_TO_TICKS(100));
        driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);

        // Configure positioning mode
        // steps_per_rev parameter is full steps, need to account for microsteps (256)
        float search_speed = Test::Motion::BOUNDS_SEARCH_SPEED;
        float steps_per_rev_with_microsteps = static_cast<float>(steps_per_rev) * 256.0f;
        int32_t steps_per_360_deg = static_cast<int32_t>(steps_per_rev_with_microsteps);
        float offset_deg = 5.0f;
        int32_t offset_steps = static_cast<int32_t>(tmc51x0::DegreesToSteps(offset_deg, steps_per_rev_with_microsteps));

        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(search_speed);
        driver.rampControl.SetAcceleration(search_speed * 2.0f);
        driver.rampControl.SetDeceleration(search_speed * 2.0f);
        driver.rampControl.SetRampSpeeds(1000.0f, 100.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Find maximum bound
        ESP_LOGI(TAG, "Finding maximum bound...");
        int32_t max_pos = FindBound(driver, steps_per_360_deg, search_speed, offset_steps, enc_baseline, true);
        if (max_pos == 0 && !driver.rampControl.IsTargetReached()) {
            ESP_LOGW(TAG, "Max bound search failed or timeout");
        }

        // Find minimum bound
        ESP_LOGI(TAG, "Finding minimum bound...");
        int32_t min_enc_baseline = 0;
        driver.encoder.GetPosition(min_enc_baseline);
        int32_t min_pos = FindBound(driver, -steps_per_360_deg, search_speed, offset_steps, min_enc_baseline, false);
        if (min_pos == 0 && !driver.rampControl.IsTargetReached()) {
            ESP_LOGW(TAG, "Min bound search failed or timeout");
        }

        // Process results
        bool max_stall = (max_pos != 0);
        bool min_stall = (min_pos != 0);
        bool reached_360 = (!max_stall && !min_stall);

        if (reached_360) {
            float bounds_deg = 175.0f;
            float steps_per_rev_with_microsteps = static_cast<float>(steps_per_rev) * 256.0f;
            int32_t bounds_steps = static_cast<int32_t>(tmc51x0::DegreesToSteps(bounds_deg, steps_per_rev_with_microsteps));
            return BoundsResult(true, -bounds_steps, bounds_steps, false);
        } else if (max_stall && min_stall) {
            int32_t center = (min_pos + max_pos) / 2;
            MoveToPosition(driver, center, search_speed);
            driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
            return BoundsResult(true, min_pos - center, max_pos - center, true);
        } else {
            return BoundsResult(true, min_pos, max_pos, (max_stall && min_stall));
        }
    }

private:
    int32_t FindBound(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        int32_t target_steps,
        float search_speed,
        int32_t offset_steps,
        int32_t enc_baseline,
        bool is_max
    ) {
        namespace Test = tmc51x0_test_config::TestConfig_17HS4401S;
        constexpr int32_t MIN_MOVEMENT = 5000;
        constexpr uint32_t ENCODER_STALL_TIMEOUT_MS = 300;
        constexpr int32_t ENCODER_MIN_CHANGE = 5;
        uint32_t timeout_ms = Test::Motion::HOMING_TIMEOUT_MS;

        float start_pos_float = 0.0f;
        driver.rampControl.GetCurrentPosition(start_pos_float, tmc51x0::Unit::Steps);
        int32_t start_pos = static_cast<int32_t>(start_pos_float);
        uint32_t start_time = esp_timer_get_time() / 1000;
        bool motion_started = false;
        int32_t last_enc_pos = enc_baseline;
        uint32_t last_enc_change_time = start_time;

        driver.rampControl.SetTargetPosition(static_cast<float>(target_steps), tmc51x0::Unit::Steps);

        while (true) {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
            if (elapsed > timeout_ms) break;

            float pos_float = 0.0f;
            driver.rampControl.GetCurrentPosition(pos_float, tmc51x0::Unit::Steps);
            int32_t pos = static_cast<int32_t>(pos_float);
            int32_t delta = (pos > start_pos) ? (pos - start_pos) : (start_pos - pos);

            if (driver.rampControl.IsTargetReached()) {
                return 0; // No stall
            }

            float vactual = 0.0f;
            driver.rampControl.GetCurrentSpeed(vactual, tmc51x0::Unit::Steps);

            if (!motion_started && delta > 100) {
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
                } else if (std::abs(vactual) > 500.0f && motion_started) {
                    uint32_t time_since_change = current_time - last_enc_change_time;
                    if (time_since_change >= ENCODER_STALL_TIMEOUT_MS) {
                        if (delta < MIN_MOVEMENT) {
                            ESP_LOGW(TAG, "False encoder stall, continuing...");
                            last_enc_change_time = current_time;
                            continue;
                        }

                        // Real stall detected
                        driver.rampControl.Stop();
                        vTaskDelay(pdMS_TO_TICKS(200));

                        // Back off
                        float stall_pos_float = 0.0f;
                        driver.rampControl.GetCurrentPosition(stall_pos_float, tmc51x0::Unit::Steps);
                        int32_t stall_pos = static_cast<int32_t>(stall_pos_float);
                        int32_t backoff_target = is_max ? (stall_pos - offset_steps) : (stall_pos + offset_steps);

                        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
                        driver.rampControl.SetTargetPosition(static_cast<float>(backoff_target), tmc51x0::Unit::Steps);
                        driver.rampControl.SetMaxSpeed(search_speed / 2.0f);
                        while (!driver.rampControl.IsTargetReached()) {
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }

                        float final_pos_float = 0.0f;
                        driver.rampControl.GetCurrentPosition(final_pos_float, tmc51x0::Unit::Steps);
                        return static_cast<int32_t>(final_pos_float);
                    }
                } else {
                    last_enc_change_time = current_time;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        return 0;
    }

    void MoveToPosition(tmc51x0::TMC51x0<Esp32SPI>& driver, int32_t target, float speed) {
        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetTargetPosition(static_cast<float>(target), tmc51x0::Unit::Steps);
        driver.rampControl.SetMaxSpeed(speed);
        driver.rampControl.SetAcceleration(speed * 2.0f);
        driver.rampControl.SetDeceleration(speed * 2.0f);
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
