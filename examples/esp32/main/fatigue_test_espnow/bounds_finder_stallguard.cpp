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
#include <inttypes.h>
#include "registers/tmc51x0_registers.hpp"

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
        tmc51x0::TMC51x0<Esp32SPI>& driver
    ) override {
        uint32_t total_start_time = esp_timer_get_time() / 1000;
        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
        ESP_LOGI(TAG, "Starting StallGuard2 bounds finding...");
        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");

        // StallGuard2 requires SpreadCycle (StealthChop disabled). We temporarily
        // force it off for this bounds-finding run and restore afterward.
        bool restore_stealthchop = false;
        auto restore_stealthchop_if_needed = [&driver, &restore_stealthchop]() {
            if (restore_stealthchop) {
                driver.motorControl.SetStealthChopEnabled(true);
            }
        };
        // We also temporarily enable sg_stop (hardware stop on stall) for bounds finding
        // so we can safely use higher acceleration without "missing" a stall.
        bool restore_stop_on_stall = false;
        bool cached_stop_on_stall = false;
        auto restore_stop_on_stall_if_needed = [&driver, &restore_stop_on_stall, &cached_stop_on_stall]() {
            if (restore_stop_on_stall) {
                driver.diagnostics.EnableStopOnStall(cached_stop_on_stall);
            }
        };
        {
            auto stealth_enabled = driver.motorControl.IsStealthChopEnabled();
            if (stealth_enabled && stealth_enabled.Value()) {
                restore_stealthchop = true;
                ESP_LOGI(TAG, "StealthChop is enabled -> disabling for StallGuard2 run");
                auto disable_res = driver.motorControl.SetStealthChopEnabled(false);
                if (!disable_res) {
                    ESP_LOGW(TAG, "⚠ Failed to disable StealthChop (ErrorCode: %d). SG may be invalid.",
                             static_cast<int>(disable_res.Error()));
                }
            }
        }

        // Configure StallGuard2
        // Configure CoolStep thresholds (COOLCONF.SEMIN/SEMAX) if desired by config.
        // Note: CoolStep is separate from StallGuard. We configure it here so that
        // the "SEMIN/SEMAX" knobs in the test config actually map to hardware.
        // For homing/bounds we typically keep SEMIN=0 (disabled) for stable current.
        {
            tmc51x0::CoolStepConfig cool_cfg{};
            if (TestConfig::StallGuard::SEMIN != 0) {
                const uint16_t semin = static_cast<uint16_t>(TestConfig::StallGuard::SEMIN);
                const uint16_t semax = static_cast<uint16_t>(TestConfig::StallGuard::SEMAX);
                cool_cfg.lower_threshold_sg = static_cast<uint16_t>(semin * 32U);
                cool_cfg.upper_threshold_sg = static_cast<uint16_t>((semin + semax + 1U) * 32U);
            } else {
                // lower_threshold_sg=0 disables CoolStep
                cool_cfg.lower_threshold_sg = 0;
                cool_cfg.upper_threshold_sg = 0;
            }
            cool_cfg.increment_step = tmc51x0::CoolStepIncrementStep::STEP_2;
            cool_cfg.decrement_speed = tmc51x0::CoolStepDecrementSpeed::EVERY_8;
            cool_cfg.min_current = tmc51x0::CoolStepMinCurrent::HALF_IRUN;
            cool_cfg.enable_filter = TestConfig::StallGuard::FILTER_ENABLED;
            // If you enable CoolStep, make sure it is only active in the same
            // velocity region where StallGuard is stable.
            cool_cfg.min_velocity = TestConfig::StallGuard::MIN_VELOCITY_RPM;
            cool_cfg.max_velocity = 0.0f; // no upper limit
            cool_cfg.velocity_unit = tmc51x0::Unit::RPM;

            auto cool_res = driver.motorControl.ConfigureCoolStep(cool_cfg);
            if (!cool_res) {
                ESP_LOGW(TAG, "⚠ Failed to configure CoolStep (ErrorCode: %d). Continuing with StallGuard-only.",
                         static_cast<int>(cool_res.Error()));
            }
        }

        tmc51x0::StallGuardConfig sg_config{};
        sg_config.threshold = TestConfig::StallGuard::SGT_HOMING;
        sg_config.enable_filter = TestConfig::StallGuard::FILTER_ENABLED;
        sg_config.min_velocity = TestConfig::StallGuard::MIN_VELOCITY_RPM;
        sg_config.velocity_unit = tmc51x0::Unit::RPM;
        
        ESP_LOGI(TAG, "StallGuard2 Config:");
        ESP_LOGI(TAG, "  SGT threshold: %d", TestConfig::StallGuard::SGT_HOMING);
        ESP_LOGI(TAG, "  Filter: %s", TestConfig::StallGuard::FILTER_ENABLED ? "enabled" : "disabled");
        ESP_LOGI(TAG, "  Min velocity: %.0f RPM", TestConfig::StallGuard::MIN_VELOCITY_RPM);
        
        if (!driver.diagnostics.ConfigureStallGuard(sg_config)) {
            ESP_LOGE(TAG, "❌ Failed to configure StallGuard2");
            restore_stealthchop_if_needed();
            return BoundsResult(false, 0, 0, false);
        }
        ESP_LOGI(TAG, "  ✓ StallGuard2 configured");

        // Print a debug snapshot of the relevant registers / thresholds so we can
        // diagnose "SG_RESULT always 0" issues.
        {
            uint32_t drv_status_raw = 0;
            uint32_t ramp_stat_raw = 0;
            auto drv_status_res = driver.diagnostics.GetDriverStatusRegister();
            auto ramp_stat_res = driver.diagnostics.GetRampStatusRegister();
            if (drv_status_res) drv_status_raw = drv_status_res.Value();
            if (ramp_stat_res) ramp_stat_raw = ramp_stat_res.Value();

            tmc51x0::DRV_STATUS_Register drv_status{};
            drv_status.value = drv_status_raw;
            tmc51x0::RAMP_STAT_Register ramp_stat{};
            ramp_stat.value = ramp_stat_raw;

            auto stealth_enabled = driver.motorControl.IsStealthChopEnabled();
            auto tpwmthrs_rpm = driver.motorControl.GetStealthChopVelocityThreshold(tmc51x0::Unit::RPM);
            auto tcoolthrs_rpm = driver.diagnostics.GetTcoolthrs(tmc51x0::Unit::RPM);
            uint32_t tpwmthrs = driver.diagnostics.GetTpwmthrsRegisterValue();
            uint32_t tcoolthrs = driver.diagnostics.GetTcoolthrsRegisterValue();

            ESP_LOGI(TAG, "StallGuard Debug Snapshot:");
            ESP_LOGI(TAG, "  GCONF.en_pwm_mode (StealthChop enabled): %s",
                     (stealth_enabled && stealth_enabled.Value()) ? "1" : "0");
            ESP_LOGI(TAG, "  DRV_STATUS: 0x%08" PRIX32 " (stealth=%u stallguard=%u sg_result=%u)",
                     drv_status_raw, drv_status.bits.stealth, drv_status.bits.stallguard, drv_status.bits.sg_result);
            ESP_LOGI(TAG, "  RAMP_STAT:  0x%08" PRIX32 " (status_sg=%u event_stop_sg=%u vzero=%u)",
                     ramp_stat_raw, ramp_stat.bits.status_sg, ramp_stat.bits.event_stop_sg, ramp_stat.bits.vzero);
            ESP_LOGI(TAG, "  TPWMTHRS(raw)=0x%05" PRIX32 " (%" PRIu32 "), TPWMTHRS(threshold)=%.2f RPM",
                     (tpwmthrs & 0xFFFFF), (tpwmthrs & 0xFFFFF),
                     tpwmthrs_rpm.IsOk() ? tpwmthrs_rpm.Value() : -1.0f);
            ESP_LOGI(TAG, "  TCOOLTHRS(raw)=0x%05" PRIX32 " (%" PRIu32 "), TCOOLTHRS(threshold)=%.2f RPM",
                     (tcoolthrs & 0xFFFFF), (tcoolthrs & 0xFFFFF),
                     tcoolthrs_rpm.IsOk() ? tcoolthrs_rpm.Value() : -1.0f);

            if (drv_status.bits.stealth != 0) {
                ESP_LOGW(TAG, "⚠ Driver reports StealthChop active (DRV_STATUS.stealth=1). "
                             "StallGuard2 readings (SG_RESULT) are likely invalid until SpreadCycle is active.");
            }
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
        ESP_LOGI(TAG, "  ✓ Reference switches disabled");

        // Configure stall handling
        {
            auto stop_on_stall = driver.diagnostics.IsStopOnStallEnabled();
            cached_stop_on_stall = stop_on_stall.IsOk() ? stop_on_stall.Value() : false;
            restore_stop_on_stall = true;
            // Enable stop-on-stall so the chip stops immediately on a stall event.
            // We'll still do our own backoff afterwards.
            driver.diagnostics.EnableStopOnStall(true);
        }
        driver.rampControl.SetStopMode(tmc51x0::ReferenceStopMode::HARD_STOP);
        driver.diagnostics.ClearStallFlag();
        ESP_LOGI(TAG, "  ✓ Stall handling configured (sg_stop enabled, hard stop)");

        // Establish home position
        driver.rampControl.Stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
        driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
        ESP_LOGI(TAG, "  ✓ Home position established at 0.0°");

        // Configure motion parameters
        constexpr float TARGET_ANGLE_DEG = 360.0f;
        constexpr float OFFSET_ANGLE_DEG = 5.0f;
        float search_speed_rpm = TestConfig::Motion::BOUNDS_SEARCH_SPEED_RPM;
        // Use the test's bounds-search acceleration (MotorConfig ramp defaults are
        // for general motion profiles and can be much slower, keeping us longer in
        // low-speed resonance bands and near the StallGuard minimum-velocity edge).
        float search_accel_rev_s2 = TestConfig::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;
        float vstart_rpm = MotorConfig::RAMP_VSTART_RPM;
        float vstop_rpm = MotorConfig::RAMP_VSTOP_RPM;
        float v1_rpm = MotorConfig::RAMP_V1_RPM;

        ESP_LOGI(TAG, "Motion Parameters:");
        ESP_LOGI(TAG, "  Search speed (VMAX): %.0f RPM", search_speed_rpm);
        ESP_LOGI(TAG, "  Acceleration: %.2f rev/s²", search_accel_rev_s2);
        ESP_LOGI(TAG, "  Ramp speeds: VSTART=%.1f, VSTOP=%.1f, V1=%.1f RPM", 
                 vstart_rpm, vstop_rpm, v1_rpm);
        ESP_LOGI(TAG, "  Target angle: ±%.0f° (backoff: %.0f°)", TARGET_ANGLE_DEG, OFFSET_ANGLE_DEG);
        
        // Clear XTARGET before setting ramp mode
        auto clear_pos = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        if (clear_pos) {
            driver.rampControl.SetTargetPosition(clear_pos.Value(), tmc51x0::Unit::Deg);
        }
        
        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(search_speed_rpm, tmc51x0::Unit::RPM);
        driver.rampControl.SetAcceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetRampSpeeds(vstart_rpm, vstop_rpm, v1_rpm, tmc51x0::Unit::RPM);
        ESP_LOGI(TAG, "  ✓ Motion parameters configured");

        // Find minimum bound
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Searching for MIN bound: +%.0f° @ %.0f RPM", TARGET_ANGLE_DEG, search_speed_rpm);
        uint32_t min_search_start = esp_timer_get_time() / 1000;
        float min_pos_deg = FindBound(driver, TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG);
        uint32_t min_search_time = (esp_timer_get_time() / 1000) - min_search_start;
        if (min_pos_deg != 0.0f) {
            ESP_LOGI(TAG, "  ✓ Min bound found: %.1f° (took %ums)", min_pos_deg, min_search_time);
        } else {
            ESP_LOGI(TAG, "  → No stall detected, reached target (took %ums)", min_search_time);
        }

        // Find maximum bound
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Searching for MAX bound: -%.0f° @ %.0f RPM", TARGET_ANGLE_DEG, search_speed_rpm);
        driver.diagnostics.ClearStallFlag();
        uint32_t max_search_start = esp_timer_get_time() / 1000;
        float max_pos_deg = FindBound(driver, -TARGET_ANGLE_DEG, search_speed_rpm, OFFSET_ANGLE_DEG);
        uint32_t max_search_time = (esp_timer_get_time() / 1000) - max_search_start;
        if (max_pos_deg != 0.0f) {
            ESP_LOGI(TAG, "  ✓ Max bound found: %.1f° (took %ums)", max_pos_deg, max_search_time);
        } else {
            ESP_LOGI(TAG, "  → No stall detected, reached target (took %ums)", max_search_time);
        }

        // Process results
        bool max_stall = (max_pos_deg != 0.0f);
        bool min_stall = (min_pos_deg != 0.0f);
        bool reached_360 = (!max_stall && !min_stall);
        
        uint32_t total_time = (esp_timer_get_time() / 1000) - total_start_time;

        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Bounds Finding Results:");
        ESP_LOGI(TAG, "  Min: %.1f° (%s)", min_pos_deg, min_stall ? "stall detected" : "no stall");
        ESP_LOGI(TAG, "  Max: %.1f° (%s)", max_pos_deg, max_stall ? "stall detected" : "no stall");
        ESP_LOGI(TAG, "  Total time: %ums", total_time);

        if (reached_360) {
            constexpr float bounds_deg = 175.0f;
            ESP_LOGI(TAG, "  → No mechanical bounds found, using default: ±%.1f°", bounds_deg);
            ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
            restore_stop_on_stall_if_needed();
            restore_stealthchop_if_needed();
            return BoundsResult(true, -bounds_deg, bounds_deg, false);
        } else if (max_stall && min_stall) {
            float center_deg = (min_pos_deg + max_pos_deg) / 2.0f;
            ESP_LOGI(TAG, "  → Both bounds found, moving to center: %.1f°", center_deg);
            MoveToPosition(driver, center_deg, search_speed_rpm);
            // IMPORTANT:
            // SetCurrentPosition() changes XACTUAL but does NOT change XTARGET.
            // In POSITIONING mode, leaving an old XTARGET will cause the ramp generator
            // to keep moving to chase that stale target. After redefining home/zero,
            // set XTARGET to the new zero (or current position) and hold.
            driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
            driver.rampControl.SetTargetPosition(0.0f, tmc51x0::Unit::Deg);
            driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
            float min_bound_deg = min_pos_deg - center_deg;
            float max_bound_deg = max_pos_deg - center_deg;
            ESP_LOGI(TAG, "  ✓ Final bounds (relative to center): %.1f° to %.1f°", min_bound_deg, max_bound_deg);
            ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
            restore_stop_on_stall_if_needed();
            restore_stealthchop_if_needed();
            return BoundsResult(true, min_bound_deg, max_bound_deg, true);
        } else {
            ESP_LOGI(TAG, "  → Partial bounds detected");
            ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
            restore_stop_on_stall_if_needed();
            restore_stealthchop_if_needed();
            return BoundsResult(true, min_pos_deg, max_pos_deg, (max_stall && min_stall));
        }
    }

private:
    float FindBound(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        float target_angle_deg,
        float search_speed_rpm,
        float offset_angle_deg
    ) {
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
        uint8_t stall_candidate_count = 0;
        struct SgSample {
            uint16_t sg{0};
            float vel_rpm{0.0f};
            float pos_deg{0.0f};
            uint8_t stealth{0};
            uint8_t stallguard{0};
            uint8_t event_stop_sg{0};
            uint32_t elapsed_ms{0};
        };
        static constexpr size_t SG_TRACE_LEN = 16;
        SgSample sg_trace[SG_TRACE_LEN]{};
        size_t sg_trace_idx = 0;

        // Ensure motor is stopped
        auto pre_move_standstill = driver.rampControl.IsStandstill();
        if (!pre_move_standstill || !pre_move_standstill.Value()) {
            driver.rampControl.Stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            for (int i = 0; i < 10; i++) {
                auto check = driver.rampControl.IsStandstill();
                if (check && check.Value()) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        
        // Clear XTARGET and restore ramp parameters
        auto current_pos_for_clear = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        if (current_pos_for_clear) {
            driver.rampControl.SetTargetPosition(current_pos_for_clear.Value(), tmc51x0::Unit::Deg);
        }
        
        driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver.rampControl.SetMaxSpeed(search_speed_rpm, tmc51x0::Unit::RPM);
        float vstart_rpm = MotorConfig::RAMP_VSTART_RPM;
        float vstop_rpm = MotorConfig::RAMP_VSTOP_RPM;
        float v1_rpm = MotorConfig::RAMP_V1_RPM;
        driver.rampControl.SetRampSpeeds(vstart_rpm, vstop_rpm, v1_rpm, tmc51x0::Unit::RPM);
        float search_accel_rev_s2 = TestConfig::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;
        driver.rampControl.SetAcceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.rampControl.SetDeceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
        driver.diagnostics.ClearStallFlag();
        auto move_result = driver.rampControl.MoveRelative(target_angle_deg, tmc51x0::Unit::Deg);
        if (!move_result) {
            ESP_LOGE(TAG, "❌ Failed to start move (ErrorCode: %d)", static_cast<int>(move_result.Error()));
            return 0.0f;
        }

        uint32_t iteration_count = 0;
        // Logging is intentionally less frequent than the polling loop.
        // Keep this reasonably small so the printed SG values don't look "stale".
        constexpr uint32_t LOG_INTERVAL = 10;

        while (true) {
            uint32_t current_time = esp_timer_get_time() / 1000;
            uint32_t elapsed = current_time - start_time;
            if (elapsed > timeout_ms) {
                ESP_LOGW(TAG, "Timeout after %ums", timeout_ms);
                break;
            }

            // Avoid a tight polling loop that can starve other tasks and flood SPI.
            vTaskDelay(pdMS_TO_TICKS(10));

            auto pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
            float pos_deg = pos_result.IsOk() ? pos_result.Value() : start_pos_deg;
            float delta_deg = fabsf(pos_deg - start_pos_deg);
            
            iteration_count++;
            auto vel_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
            float vel_rpm = vel_result.IsOk() ? vel_result.Value() : 0.0f;
            
            auto sg_result = driver.diagnostics.GetStallGuardResult();
            uint16_t sg_val = sg_result.IsOk() ? sg_result.Value() : 1023;

            // Read status every loop so we can detect sg_stop events precisely.
            uint32_t drv_status_raw = 0;
            uint32_t ramp_stat_raw = 0;
            tmc51x0::DRV_STATUS_Register drv_status{};
            tmc51x0::RAMP_STAT_Register ramp_stat{};
            auto drv_status_res = driver.diagnostics.GetDriverStatusRegister();
            auto ramp_stat_res = driver.diagnostics.GetRampStatusRegister();
            if (drv_status_res) {
                drv_status_raw = drv_status_res.Value();
                drv_status.value = drv_status_raw;
            }
            if (ramp_stat_res) {
                ramp_stat_raw = ramp_stat_res.Value();
                ramp_stat.value = ramp_stat_raw;
            }

            // Store a short trace of recent samples for post-mortem on a stall event.
            {
                SgSample& s = sg_trace[sg_trace_idx % SG_TRACE_LEN];
                s.sg = sg_val;
                s.vel_rpm = vel_rpm;
                s.pos_deg = pos_deg;
                s.stealth = static_cast<uint8_t>(drv_status.bits.stealth);
                s.stallguard = static_cast<uint8_t>(drv_status.bits.stallguard);
                s.event_stop_sg = static_cast<uint8_t>(ramp_stat.bits.event_stop_sg);
                s.elapsed_ms = elapsed;
                sg_trace_idx++;
            }

            if (iteration_count == 1 || iteration_count % LOG_INTERVAL == 0) {
                if (sg_result.IsOk()) {
                    ESP_LOGI(TAG,
                             "  [Progress] SG:%u V:%.0f Pos:%.0f° Δ:%.0f° status_sg:%u event_stop_sg:%u stealth:%u t:%ums",
                             sg_val, vel_rpm, pos_deg, delta_deg, ramp_stat.bits.status_sg, ramp_stat.bits.event_stop_sg,
                             drv_status.bits.stealth, elapsed);
                }
            }

            auto reached_result = driver.rampControl.IsTargetReached();
            if (reached_result && reached_result.Value()) {
                ESP_LOGI(TAG, "✓ Target reached without stall");
                return 0.0f; // No stall
            }

            if (!motion_started && delta_deg > 0.5f) { // ~0.5 degrees movement threshold
                motion_started = true;
            }

            // StallGuard validation:
            // - Require motion started and a minimum velocity (datasheet requirement)
            // - Require SG_RESULT to indicate high load (low SG_RESULT)
            // Per datasheet: "StallGuard needs a certain velocity to work (as set by TCOOLTHRS)"
            // SG_RESULT interpretation:
            // - 0 = definite stall (high load) - but ONLY if velocity >= MIN_VELOCITY_RPM
            // - 1-100 = high load/near stall (stall condition)
            // - 500-1023 = low load (NOT a stall - motor running freely)
            constexpr uint16_t SG_HIGH_LOAD_THRESHOLD = 100;  // <= this = high load / near-stall
            
            // CRITICAL: StallGuard requires minimum velocity (TCOOLTHRS) to operate
            // Even SG=0 is unreliable if velocity is below MIN_VELOCITY_RPM
            float min_velocity_required = TestConfig::StallGuard::MIN_VELOCITY_RPM;
            bool velocity_above_minimum = std::abs(vel_rpm) >= min_velocity_required;
            
            // Validate stall
            if (!motion_started) {
                stall_candidate_count = 0;
                continue; // Ignore early/initial stall events before motion is clearly underway
            }
            // Require corroborating stall indication and ensure we are NOT in StealthChop
            // (SG_RESULT can read 0 when StallGuard is inactive or StealthChop is active).
            if (!drv_status_res || !ramp_stat_res) {
                stall_candidate_count = 0;
                continue;
            }
            if (drv_status.bits.stealth != 0) {
                stall_candidate_count = 0;
                continue; // StallGuard not reliable in StealthChop
            }

            // Primary trigger (with sg_stop enabled): event_stop_sg.
            // This indicates the ramp generator has stopped due to a StallGuard stop event.
            const bool hw_stop_event = (ramp_stat.bits.event_stop_sg != 0);

            if (!hw_stop_event) {
                // If no hardware stop event yet, do not act on SG until we're above the
                // configured min velocity.
                if (!velocity_above_minimum) {
                    stall_candidate_count = 0;
                    continue;
                }
                if (!sg_result.IsOk()) {
                    stall_candidate_count = 0;
                    continue; // Can't reliably validate without SG_RESULT
                }
                if (sg_val > SG_HIGH_LOAD_THRESHOLD) {
                    stall_candidate_count = 0;
                    continue;
                }
                // Debounce: require DRV_STATUS.stallguard and a few consecutive low SG samples.
                if (drv_status.bits.stallguard == 0) {
                    stall_candidate_count = 0;
                    continue;
                }
                if (stall_candidate_count < 255) {
                    stall_candidate_count++;
                }
                if (stall_candidate_count < 3) {
                    continue;
                }
            }
            
            // Real stall confirmed
            ESP_LOGI(TAG,
                     "  → STALL DETECTED: SG=%u, V=%.0f RPM, Pos=%.0f° (status_sg=%u event_stop_sg=%u stallguard=%u)",
                     sg_val, vel_rpm, pos_deg, ramp_stat.bits.status_sg, ramp_stat.bits.event_stop_sg, drv_status.bits.stallguard);

            // Print the last few samples so "SG was 200 then stall" can be explained.
            ESP_LOGI(TAG, "  [SG Trace] Most recent samples (oldest->newest):");
            const size_t n = std::min(static_cast<size_t>(SG_TRACE_LEN), sg_trace_idx);
            const size_t start = (sg_trace_idx >= n) ? (sg_trace_idx - n) : 0;
            for (size_t i = 0; i < n; i++) {
                const SgSample& s = sg_trace[(start + i) % SG_TRACE_LEN];
                ESP_LOGI(TAG, "    t=%ums SG=%u V=%.0f Pos=%.1f° stealth=%u stallguard=%u event_stop_sg=%u",
                         s.elapsed_ms, s.sg, s.vel_rpm, s.pos_deg, s.stealth, s.stallguard, s.event_stop_sg);
            }

            driver.rampControl.Stop();
            vTaskDelay(pdMS_TO_TICKS(150));
            driver.diagnostics.ClearStallFlag();

            // Back off
            // Back off MUST be opposite the travel direction.
            // - If target_angle_deg > 0 (moving +), backoff should be negative.
            // - If target_angle_deg < 0 (moving -), backoff should be positive.
            float backoff_offset_deg = (target_angle_deg >= 0.0f) ? -offset_angle_deg : offset_angle_deg;
            float backoff_speed_rpm = search_speed_rpm;
            float backoff_accel_rev_s2 = TestConfig::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;
            float vstart_rpm = MotorConfig::RAMP_VSTART_RPM;
            float vstop_rpm = MotorConfig::RAMP_VSTOP_RPM;
            float v1_rpm = MotorConfig::RAMP_V1_RPM;
            
            driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
            driver.rampControl.SetMaxSpeed(backoff_speed_rpm, tmc51x0::Unit::RPM);
            driver.rampControl.SetRampSpeeds(vstart_rpm, vstop_rpm, v1_rpm, tmc51x0::Unit::RPM);
            driver.rampControl.SetAcceleration(backoff_accel_rev_s2, tmc51x0::Unit::RevPerSec);
            driver.rampControl.SetDeceleration(backoff_accel_rev_s2, tmc51x0::Unit::RevPerSec);
            driver.rampControl.MoveRelative(backoff_offset_deg, tmc51x0::Unit::Deg);
            
            // Wait for backoff
            for (int i = 0; i < 30; i++) {
                auto reached = driver.rampControl.IsTargetReached();
                if (reached && reached.Value()) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            auto final_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
            float final_pos_deg = final_pos_result.IsOk() ? final_pos_result.Value() : pos_deg;
            ESP_LOGI(TAG, "  → Backed off to: %.1f° (from stall at %.1f°)", final_pos_deg, pos_deg);
                return final_pos_deg;

            // (unreachable)
        }

        return 0.0f;
    }

    void MoveToPosition(tmc51x0::TMC51x0<Esp32SPI>& driver, float target_deg, float speed_rpm) {
        // Speed is already in RPM, calculate acceleration in rev/s²
        // Driver handles microstep conversion internally
        // Use motor config acceleration (RAMP_AMAX) directly
        float accel_rev_s2 = TestConfig::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;
        
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
