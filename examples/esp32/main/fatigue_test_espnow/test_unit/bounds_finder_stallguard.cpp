/**
 * @file bounds_finder_stallguard.cpp
 * @brief StallGuard2-based bounds finder implementation
 */

#include "bounds_finder.hpp"
#include "../test_config/esp32_tmc51x0_test_config.hpp"
#include <memory>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>

static const char* TAG = "BoundsFinderSG";

namespace FatigueTest {

/**
 * @brief StallGuard2-based bounds finder
 */
class StallGuardBoundsFinder : public IBoundsFinder {
public:
    const char* GetMethodName() const override {
        return "StallGuard2";
    }

    BoundsResult FindBounds(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        uint16_t steps_per_rev
    ) override {
        ESP_LOGI(TAG, "Starting StallGuard2 bounds finding...");

        // Use test config constants
        namespace Test = tmc51x0_test_config::TestConfig_17HS4401S;
        
        // Configure StallGuard2
        tmc51x0::StallGuardConfig sg_config{};
        sg_config.threshold = Test::StallGuard::SGT_HOMING;
        sg_config.enable_filter = Test::StallGuard::FILTER_ENABLED;
        
        if (!driver.diagnostics.ConfigureStallGuard(sg_config)) {
            ESP_LOGE(TAG, "Failed to configure StallGuard2");
            return BoundsResult(false, 0, 0, false);
        }

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

        // Reset position
        driver.rampControl.Stop();
        driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
        vTaskDelay(pdMS_TO_TICKS(100));
        driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);

        // Configure positioning mode
        float search_speed = Test::Motion::BOUNDS_SEARCH_SPEED;
        int32_t steps_per_360_deg = static_cast<int32_t>(steps_per_rev * 256.0f);
        float offset_deg = 5.0f;
        int32_t offset_steps = tmc51x0::DegreesToSteps(offset_deg, steps_per_rev * 256.0f);

        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(search_speed);
        driver.rampControl.SetAcceleration(search_speed * 2.0f);
        driver.rampControl.SetDeceleration(search_speed * 2.0f);
        driver.rampControl.SetRampSpeeds(1000.0f, 100.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Find maximum bound
        ESP_LOGI(TAG, "Finding maximum bound...");
        int32_t max_pos = FindBound(driver, steps_per_360_deg, search_speed, offset_steps, true);
        if (max_pos == 0 && !driver.rampControl.IsTargetReached()) {
            ESP_LOGW(TAG, "Max bound search failed or timeout");
        }

        // Find minimum bound
        ESP_LOGI(TAG, "Finding minimum bound...");
        driver.diagnostics.ClearStallFlag();
        int32_t min_pos = FindBound(driver, -steps_per_360_deg, search_speed, offset_steps, false);
        if (min_pos == 0 && !driver.rampControl.IsTargetReached()) {
            ESP_LOGW(TAG, "Min bound search failed or timeout");
        }

        // Disable StallGuard2 stop for normal operation
        driver.diagnostics.EnableStopOnStall(false);

        // Process results
        bool max_stall = (max_pos != 0);
        bool min_stall = (min_pos != 0);
        bool reached_360 = (!max_stall && !min_stall);

        if (reached_360) {
            // No stalls - use default bounds
            float bounds_deg = 175.0f;
            int32_t bounds_steps = tmc51x0::DegreesToSteps(bounds_deg, steps_per_rev * 256.0f);
            return BoundsResult(true, -bounds_steps, bounds_steps, false);
        } else if (max_stall && min_stall) {
            // Both stalls detected - move to center
            int32_t center = (min_pos + max_pos) / 2;
            MoveToPosition(driver, center, search_speed);
            driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
            return BoundsResult(true, min_pos - center, max_pos - center, true);
        } else {
            // Partial bounds - use what we found
            return BoundsResult(true, min_pos, max_pos, (max_stall && min_stall));
        }
    }

private:
    int32_t FindBound(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        int32_t target_steps,
        float search_speed,
        int32_t offset_steps,
        bool is_max
    ) {
        namespace Test = tmc51x0_test_config::TestConfig_17HS4401S;
        constexpr int32_t MIN_MOVEMENT = 5000;
        uint32_t timeout_ms = Test::Motion::HOMING_TIMEOUT_MS;

        float start_pos_float = 0.0f;
        driver.rampControl.GetCurrentPosition(start_pos_float, tmc51x0::Unit::Steps);
        int32_t start_pos = static_cast<int32_t>(start_pos_float);
        uint32_t start_time = esp_timer_get_time() / 1000;
        bool motion_started = false;

        driver.rampControl.SetTargetPosition(static_cast<float>(target_steps), tmc51x0::Unit::Steps);

        while (true) {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
            if (elapsed > timeout_ms) break;

            float pos_float = 0.0f;
            driver.rampControl.GetCurrentPosition(pos_float, tmc51x0::Unit::Steps);
            int32_t pos = static_cast<int32_t>(pos_float);
            int32_t delta = std::abs(pos - start_pos);

            if (driver.rampControl.IsTargetReached()) {
                return 0; // No stall
            }

            if (!motion_started && delta > 100) {
                motion_started = true;
            }

            if (driver.diagnostics.IsStallDetected()) {
                if (delta < MIN_MOVEMENT) {
                    ESP_LOGW(TAG, "False stall detected, clearing...");
                    driver.diagnostics.ClearStallFlag();
                    tmc51x0::RampMode mode = tmc51x0::RampMode::HOLD;
                    driver.rampControl.GetRampMode(mode);
                    if (mode != tmc51x0::RampMode::POSITIONING) {
                        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
                    }
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
std::unique_ptr<IBoundsFinder> CreateStallGuardBoundsFinder() {
    return std::make_unique<StallGuardBoundsFinder>();
}

} // namespace FatigueTest
