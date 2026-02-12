/**
 * @file fatigue_motion_impl.hpp
 * @brief Full implementation of FatigueTestMotion class
 * 
 * Complete implementation extracted from fatigue_test_stallguard.cpp and fatigue_test_encoder.cpp
 * Provides point-to-point motion with trajectory calculation, dwell handling, and cycle counting.
 * 
 * This file is included by fatigue_motion.hpp to provide header-only implementation.
 */

#pragma once
// When included from header, use conditional include; when compiled directly, include header
#ifdef FATIGUE_MOTION_HEADER_INCLUDED
// Already included from header - the class definition is available in the current context
// We're inside the namespace, so we can access the class
// No need to include header or open namespace
#else
// Not included from header (shouldn't happen for template implementation)
#include "fatigue_motion.hpp"
#endif

#include "esp_log.h"
#include "esp_timer.h"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include <algorithm>
#include <cmath>

static const char* TAG_MOTION = "FatigueMotion";

// When included from header, namespace is already closed in header
// When compiled standalone, we need to open the namespace
#ifdef FATIGUE_MOTION_HEADER_INCLUDED
// Included from header - namespace was closed before this include
// Use 'using namespace' to bring namespace into scope (matches tmc51x0.cpp pattern)
using namespace FatigueTest;
#else
// Standalone compilation - open namespace
namespace FatigueTest {
#endif

// Note: Esp32TmcMutex and TmcMutexGuard are included by fatigue_motion.hpp

// Implementation

FatigueTestMotion::FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver, Esp32TmcMutex& driver_mutex) noexcept
    : driver_(driver), global_min_bound_(0.0f), global_max_bound_(0.0f), local_min_bound_(0.0f), local_max_bound_(0.0f),
      home_position_(0.0f), bounded_(false), amplitude_(1000.0F),
      vmax_rpm_(60.0f), amax_rev_s2_(10.0f),  // Default motion parameters
      dwell_at_min_ms_(0), dwell_at_max_ms_(0), running_(false), start_time_us_(0),
      target_cycles_(0), current_cycles_(0), cycle_complete_(false),
      state_(MotionState::STOPPED), dwell_start_time_ms_(0),
      estimated_frequency_hz_(0.0f),
      driver_mutex_(driver_mutex) {
    // Note: Initialization order matches member declaration order in header
}

FatigueTestMotion::~FatigueTestMotion() noexcept = default;


void FatigueTestMotion::SetGlobalBounds(float min_bound_degrees, float max_bound_degrees) noexcept {
    {
        TmcMutexGuard guard(driver_mutex_);
        global_min_bound_ = min_bound_degrees;
        global_max_bound_ = max_bound_degrees;
        bounded_ = true;
    }
    ESP_LOGI(TAG_MOTION, "Global bounds set: min=%.2f°, max=%.2f°", global_min_bound_, global_max_bound_);

    // Clip local bounds to global bounds if they exist
    {
        TmcMutexGuard guard(driver_mutex_);
        if (local_min_bound_ != 0.0f || local_max_bound_ != 0.0f) {
            guard.unlock();
            ClipLocalBoundsToGlobal();
            return;
        }
    }
}

void FatigueTestMotion::GetGlobalBoundsDegrees(float& min_degrees, float& max_degrees) const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    min_degrees = global_min_bound_;
    max_degrees = global_max_bound_;
}

void FatigueTestMotion::SetUnbounded(float current_position_degrees, float default_range_degrees) noexcept {
    {
        TmcMutexGuard guard(driver_mutex_);
        bounded_ = false;
        home_position_ = current_position_degrees;
        global_min_bound_ = current_position_degrees - default_range_degrees / 2.0f;
        global_max_bound_ = current_position_degrees + default_range_degrees / 2.0f;
    }
    // Establish home/zero at current position
    // IMPORTANT: After this point, use ABSOLUTE positioning (SetTargetPosition)
    // Before this, we would use RELATIVE positioning (MoveRelative) if needed
    driver_->rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
    driver_->rampControl.SetTargetPosition(0.0f, tmc51x0::Unit::Deg);
    driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    ESP_LOGW(TAG_MOTION, "Unbounded mode: No mechanical stops found");
    ESP_LOGI(TAG_MOTION, "Using current position as home: %.2f°", current_position_degrees);
    ESP_LOGI(TAG_MOTION, "Default global range: [%.2f°, %.2f°]", global_min_bound_, global_max_bound_);
}

bool FatigueTestMotion::SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center,
                                                         float edge_backoff_deg) noexcept {
    float min_deg, max_deg;
    bool is_bounded;
    float global_min, global_max;
    {
        TmcMutexGuard guard(driver_mutex_);
        min_deg = min_degrees_from_center;
        max_deg = max_degrees_from_center;
        is_bounded = bounded_;
        global_min = global_min_bound_;
        global_max = global_max_bound_;
    }

    // Clip to global bounds (all in degrees)
    if (is_bounded) {
        min_deg = std::max(min_deg, global_min);
        max_deg = std::min(max_deg, global_max);
    }

    // Apply edge backoff to stay inside mechanical bounds during oscillation
    // This prevents repeatedly "kissing" the endpoints and accounts for any residual error
    if (edge_backoff_deg > 0.0f) {
        float backoff_min = min_deg + edge_backoff_deg;
        float backoff_max = max_deg - edge_backoff_deg;
        
        // Only apply backoff if it leaves a valid range
        if (backoff_min < backoff_max) {
            ESP_LOGI(TAG_MOTION, "Applying edge backoff: %.2f° (min: %.2f° -> %.2f°, max: %.2f° -> %.2f°)",
                     edge_backoff_deg, min_deg, backoff_min, max_deg, backoff_max);
            min_deg = backoff_min;
            max_deg = backoff_max;
        } else {
            ESP_LOGW(TAG_MOTION, "Edge backoff %.2f° too large for range [%.2f°, %.2f°] - using full range",
                     edge_backoff_deg, min_deg, max_deg);
        }
    }

    {
        TmcMutexGuard guard(driver_mutex_);
        local_min_bound_ = min_deg;
        local_max_bound_ = max_deg;
        home_position_ = (local_min_bound_ + local_max_bound_) / 2.0f;
        amplitude_ = (local_max_bound_ - local_min_bound_) / 2.0f;
    }

    ESP_LOGI(TAG_MOTION, "Local bounds set: min=%.2f°, max=%.2f° (with %.2f° edge backoff)", 
             min_deg, max_deg, edge_backoff_deg);
    
    // Recalculate estimated frequency with new bounds
    {
        TmcMutexGuard guard(driver_mutex_);
        RecalculateEstimatedFrequency();
    }
    return true;
}

void FatigueTestMotion::GetLocalBoundsFromCenterDegrees(float& min_degrees, float& max_degrees) const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    min_degrees = local_min_bound_;
    max_degrees = local_max_bound_;
}

// Velocity/Acceleration safety limits (conservative for fatigue testing)
static constexpr float VMAX_MIN_RPM = 5.0f;     // Minimum useful velocity
static constexpr float VMAX_MAX_RPM = 120.0f;   // Maximum safe velocity for fatigue testing
static constexpr float AMAX_MIN_REV_S2 = 0.5f;  // Minimum acceleration
static constexpr float AMAX_MAX_REV_S2 = 30.0f; // Maximum acceleration

bool FatigueTestMotion::SetMaxVelocity(float vmax_rpm) noexcept {
    // Clamp to safe limits
    float clamped = std::clamp(vmax_rpm, VMAX_MIN_RPM, VMAX_MAX_RPM);
    if (clamped != vmax_rpm) {
        ESP_LOGW(TAG_MOTION, "VMAX clamped: %.1f -> %.1f RPM (limits: %.1f-%.1f)", 
                 vmax_rpm, clamped, VMAX_MIN_RPM, VMAX_MAX_RPM);
    }
    {
        TmcMutexGuard guard(driver_mutex_);
        vmax_rpm_ = clamped;
        RecalculateEstimatedFrequency();
        
        // Always update the driver immediately - keeps registers in sync with settings
        // This ensures changes take effect whether motion is running, paused, or stopped
        driver_->rampControl.SetMaxSpeed(vmax_rpm_, tmc51x0::Unit::RPM);
    }
    ESP_LOGI(TAG_MOTION, "Max velocity set: %.1f RPM", clamped);
    return true;
    }

float FatigueTestMotion::GetMaxVelocity() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return vmax_rpm_;
}

bool FatigueTestMotion::SetAcceleration(float amax_rev_s2) noexcept {
    // Clamp to safe limits
    float clamped = std::clamp(amax_rev_s2, AMAX_MIN_REV_S2, AMAX_MAX_REV_S2);
    if (clamped != amax_rev_s2) {
        ESP_LOGW(TAG_MOTION, "AMAX clamped: %.2f -> %.2f rev/s² (limits: %.2f-%.2f)", 
                 amax_rev_s2, clamped, AMAX_MIN_REV_S2, AMAX_MAX_REV_S2);
    }
    {
        TmcMutexGuard guard(driver_mutex_);
        amax_rev_s2_ = clamped;
        RecalculateEstimatedFrequency();
        
        // Always update the driver immediately - keeps registers in sync with settings
        // This ensures changes take effect whether motion is running, paused, or stopped
        driver_->rampControl.SetAcceleration(amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetDeceleration(amax_rev_s2_, tmc51x0::Unit::RevPerSec);
    }
    ESP_LOGI(TAG_MOTION, "Acceleration set: %.2f rev/s²", clamped);
    return true;
}

float FatigueTestMotion::GetAcceleration() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return amax_rev_s2_;
}

float FatigueTestMotion::GetEstimatedCycleFrequency() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return estimated_frequency_hz_;
}

bool FatigueTestMotion::SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms) noexcept {
    // Accept dwell times as-is (no minimum enforcement - remote controller controls this)
    {
        TmcMutexGuard guard(driver_mutex_);
        dwell_at_min_ms_ = dwell_at_min_ms;
        dwell_at_max_ms_ = dwell_at_max_ms;
        RecalculateEstimatedFrequency();
    }
    ESP_LOGI(TAG_MOTION, "Dwell times updated: min=%lu ms, max=%lu ms", 
             dwell_at_min_ms, dwell_at_max_ms);
    return true;
}

void FatigueTestMotion::GetDwellTimes(uint32_t& dwell_at_min_ms, uint32_t& dwell_at_max_ms) const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    dwell_at_min_ms = dwell_at_min_ms_;
    dwell_at_max_ms = dwell_at_max_ms_;
}

bool FatigueTestMotion::SetTargetCycles(uint32_t cycles) noexcept {
    {
        TmcMutexGuard guard(driver_mutex_);
        target_cycles_ = cycles;
    }
    ESP_LOGI(TAG_MOTION, "Target cycles set: %lu (0 = infinite)", target_cycles_);
    return true;
}

uint32_t FatigueTestMotion::GetCurrentCycles() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return current_cycles_;
}

uint32_t FatigueTestMotion::GetTargetCycles() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return target_cycles_;
}

bool FatigueTestMotion::IsCycleComplete() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return cycle_complete_;
}

void FatigueTestMotion::ResetCycles() noexcept {
    {
        TmcMutexGuard guard(driver_mutex_);
        current_cycles_ = 0;
        cycle_complete_ = false;
    }
    ESP_LOGI(TAG_MOTION, "Cycle count reset");
}

bool FatigueTestMotion::IsBounded() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return bounded_;
}

bool FatigueTestMotion::Start() noexcept {
    uint32_t current_cycles, target_cycles;
    
    {
        TmcMutexGuard guard(driver_mutex_);
        // CRITICAL: Verify bounds are set before allowing motion to start
        if (fabsf(local_min_bound_) < 0.01f && fabsf(local_max_bound_) < 0.01f) {
            ESP_LOGE(TAG_MOTION, "Cannot start: local bounds not set!");
            return false;
        }

        if (cycle_complete_) {
            ESP_LOGW(TAG_MOTION, "Cycle count reached. Reset cycles or set new target to continue.");
            return false;
        }
    }
    
    // CRITICAL: Verify motor is stopped before starting new motion
    // This prevents parameter changes during active motion
    // Check outside mutex to avoid holding lock during driver calls
    auto standstill_result = driver_->rampControl.IsStandstill();
    if (!standstill_result || !standstill_result.Value()) {
        ESP_LOGW(TAG_MOTION, "Motor not at standstill, stopping before start...");
        driver_->rampControl.Stop();
        vTaskDelay(pdMS_TO_TICKS(200));
        // Wait for standstill
        uint32_t checks = 0;
        while (checks < 20) {
            checks++;
            auto check_result = driver_->rampControl.IsStandstill();
            if (check_result && check_result.Value()) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    {
        TmcMutexGuard guard(driver_mutex_);
        
        // Update estimated frequency before starting (informational)
        RecalculateEstimatedFrequency();

        // =====================================================================
        // RAMP SPEED CONFIGURATION
        // Use motor config defaults for VSTART/VSTOP (don't override)
        // These are tuned per motor and should be used as-is
        // =====================================================================
        // Get motor config defaults for VSTART/VSTOP
        using MotorConfig = typename tmc51x0_test_config::TestRigConfig<tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE>::Motor;
        const float vstart_rpm = MotorConfig::RAMP_VSTART_RPM;
        const float vstop_rpm = MotorConfig::RAMP_VSTOP_RPM;

        // =====================================================================
        // TMC5160 DATASHEET COMPLIANCE: Correct order for starting motion
        // Per datasheet Section 12 (Ramp Generator):
        // 1. Stay in HOLD mode while configuring
        // 2. Set VMAX, AMAX, DMAX (safe in HOLD - no motion occurs)
        // 3. Set XTARGET to desired position (safe in HOLD - no motion occurs)
        // 4. THEN switch to POSITIONING mode → motor starts moving
        //
        // WRONG order was: Set POSITIONING first, then target (causes brief wrong-direction motion)
        // =====================================================================
        
        // Ensure we're in HOLD mode before changing parameters
        // This prevents any motion until we explicitly switch to POSITIONING
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
        
        // Configure ramp parameters (safe to do in HOLD mode - no motion yet)
        driver_->rampControl.SetMaxSpeed(vmax_rpm_, tmc51x0::Unit::RPM);
        driver_->rampControl.SetAcceleration(amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetDeceleration(amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetRampSpeeds(vstart_rpm, vstop_rpm, 0.0f, tmc51x0::Unit::RPM);
        
        // CRITICAL: Ensure StallGuard stop-on-stall is disabled for normal motion
        // This prevents false stall detection during oscillation
        (void)driver_->stallGuard.EnableStopOnStall(false);
        
        // CRITICAL: Enable StealthChop for smooth, quiet motion during oscillations
        // StealthChop provides smooth motion without the vibration of SpreadCycle
        (void)driver_->motorControl.SetStealthChopEnabled(true);

        // Check if resuming from pause
        bool resuming_from_pause = (state_ == MotionState::PAUSED);
        
        if (resuming_from_pause) {
            // Resume from pause - restore motion from current position
            running_ = true;
            // Don't reset start_time_us_ - keep the original start time for cycle counting
            ESP_LOGI(TAG_MOTION, "Resuming from pause - continuing motion from current position");
        } else {
            // Fresh start - reset everything
            running_ = true;
            start_time_us_ = esp_timer_get_time();
        }
        
        // Determine target based on current position (for both fresh start and resume)
        auto current_pos_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
        float min_pos_deg = local_min_bound_;
        float max_pos_deg = local_max_bound_;
        float home_pos = home_position_;
        
        // Determine which direction to move based on current position
        float dist_to_home = fabsf(current_pos_deg - home_pos);
        float dist_to_min = fabsf(current_pos_deg - min_pos_deg);
        float dist_to_max = fabsf(current_pos_deg - max_pos_deg);
        
        // SET TARGET FIRST (while still in HOLD mode - no motion yet)
        // This ensures XTARGET is correct BEFORE we switch to POSITIONING
        float target_pos;
        constexpr float NEAR_THRESHOLD_DEG = 2.0f;
        if (dist_to_home < NEAR_THRESHOLD_DEG) {
            // At center, start cycle by moving to MAX
            state_ = MotionState::MOVING_TO_MAX;
            target_pos = max_pos_deg;
        } else if (dist_to_max < NEAR_THRESHOLD_DEG) {
            // At MAX, move to MIN
            state_ = MotionState::MOVING_TO_MIN;
            target_pos = min_pos_deg;
        } else if (dist_to_min < NEAR_THRESHOLD_DEG) {
            // At MIN, this completes a cycle - move to MAX for next cycle
            state_ = MotionState::MOVING_TO_MAX;
            target_pos = max_pos_deg;
        } else {
            // Somewhere in between - move toward nearest endpoint
            state_ = (dist_to_min <= dist_to_max) ? MotionState::MOVING_TO_MIN : MotionState::MOVING_TO_MAX;
            target_pos = (dist_to_min <= dist_to_max) ? min_pos_deg : max_pos_deg;
        }
        
        // Set target position WHILE STILL IN HOLD MODE (no motion yet)
        driver_->rampControl.SetTargetPosition(target_pos, tmc51x0::Unit::Deg);
        
        // NOW switch to POSITIONING mode - motor starts moving toward the already-set target
        // This is the correct order per TMC5160 datasheet
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);

        current_cycles = current_cycles_;
        target_cycles = target_cycles_;
    }

    ESP_LOGI(TAG_MOTION, "Starting fatigue test (cycles: %lu/%lu)", current_cycles,
             target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
    ESP_LOGI(TAG_MOTION, "  Motion: Point-to-point mode (center->MAX->MIN), VMAX=%.1f RPM, AMAX=%.2f rev/s², Est.Freq=%.2f Hz", 
             vmax_rpm_, amax_rev_s2_, estimated_frequency_hz_);
    ESP_LOGI(TAG_MOTION, "  Dwell: min=%lu ms, max=%lu ms", (unsigned long)dwell_at_min_ms_, (unsigned long)dwell_at_max_ms_);
    ESP_LOGI(TAG_MOTION, "  Cycle counting: one cycle = center -> MAX -> MIN (counted at MIN)");
    
    
    return true;
}

void FatigueTestMotion::Pause() noexcept {
    // Simple approach: Just set state to PAUSED
    // Update() will stop commanding new targets, but current move will finish naturally
    {
        TmcMutexGuard guard(driver_mutex_);
        state_ = MotionState::PAUSED;
        // running_ stays true - we can resume
        // Don't stop the motor - let it finish the current move to its target
    }
    ESP_LOGI(TAG_MOTION, "Paused fatigue test motion - current move will finish, then hold");
}

void FatigueTestMotion::Stop() noexcept {
    uint32_t cycles;
    {
        TmcMutexGuard guard(driver_mutex_);
        running_ = false;
        state_ = MotionState::STOPPED;  // Set state first to stop Update() from setting new targets
        cycles = current_cycles_;
    }
    
    // Stop the ramp generator (outside mutex)
    driver_->rampControl.Stop();
    
    // Wait for standstill
    uint32_t checks = 0;
    while (checks < 50) {  // Max 5 seconds
        checks++;
        auto standstill_result = driver_->rampControl.IsStandstill();
        if (standstill_result.IsOk() && standstill_result.Value()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Clear target and set HOLD mode
    {
        TmcMutexGuard guard(driver_mutex_);
        auto current_pos_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        if (current_pos_result.IsOk()) {
            float current_pos = current_pos_result.Value();
            driver_->rampControl.SetTargetPosition(current_pos, tmc51x0::Unit::Deg);
        }
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    }
    
    ESP_LOGI(TAG_MOTION, "Stopped fatigue test motion (cycles completed: %lu)", cycles);
}

bool FatigueTestMotion::IsRunning() const noexcept {
    TmcMutexGuard guard(driver_mutex_);
    return running_ && state_ != MotionState::STOPPED;
}

void FatigueTestMotion::RecalculateEstimatedFrequency() noexcept {
    // =========================================================================
    // ESTIMATED FREQUENCY CALCULATION
    // Calculate the estimated cycle frequency based on user-set VMAX, AMAX,
    // and travel distance. This is informational - the user directly controls
    // VMAX and AMAX, so the frequency is a derived result.
    // =========================================================================
    
    // Calculate total travel distance (one way) in degrees
    float distance_deg = fabsf(local_max_bound_ - local_min_bound_);
    if (distance_deg < 0.1f || vmax_rpm_ <= 0.0f || amax_rev_s2_ <= 0.0f) {
        estimated_frequency_hz_ = 0.0f;
        ESP_LOGW(TAG_MOTION, "Cannot estimate frequency (dist=%.2f°, vmax=%.1f RPM, amax=%.2f rev/s²)", 
                 distance_deg, vmax_rpm_, amax_rev_s2_);
        return;
    }

    // Convert VMAX to deg/s: RPM * (360 deg/rev) / (60 s/min) = deg/s
    float vmax_deg_per_s = vmax_rpm_ * 360.0f / 60.0f;
    
    // Convert AMAX to deg/s²: rev/s² * 360 deg/rev = deg/s²
    float amax_deg_per_s2 = amax_rev_s2_ * 360.0f;
    
    // Time to accelerate to VMAX: t_accel = vmax / amax
    float time_to_vmax_s = vmax_deg_per_s / amax_deg_per_s2;
    
    // Distance covered during acceleration: d_accel = 0.5 * a * t²
    float distance_during_accel_deg = 0.5f * amax_deg_per_s2 * time_to_vmax_s * time_to_vmax_s;
    
    float leg_time_s;
    
    if (2.0f * distance_during_accel_deg >= distance_deg) {
        // Triangular profile: can't reach VMAX
        // d = 0.5 * a * t² * 2 (accel + decel)
        // t_leg = sqrt(d / a)
        leg_time_s = 2.0f * sqrtf(distance_deg / amax_deg_per_s2);
    } else {
        // Trapezoidal profile: reaches VMAX
        // Cruise distance = total - 2*accel_dist
        float cruise_distance_deg = distance_deg - 2.0f * distance_during_accel_deg;
        float cruise_time_s = cruise_distance_deg / vmax_deg_per_s;
        leg_time_s = 2.0f * time_to_vmax_s + cruise_time_s;
    }
    
    // Total dwell time per cycle
    float total_dwell_s = (dwell_at_min_ms_ + dwell_at_max_ms_) / 1000.0f;
    
    // Cycle period = 2 legs + dwell
    float cycle_period_s = 2.0f * leg_time_s + total_dwell_s;
    
    estimated_frequency_hz_ = (cycle_period_s > 0.0f) ? (1.0f / cycle_period_s) : 0.0f;
    
    ESP_LOGI(TAG_MOTION, "Estimated Frequency: %.2f Hz (dist=%.1f°, leg=%.3fs, dwell=%.1fs)", 
             estimated_frequency_hz_, distance_deg, leg_time_s, total_dwell_s);
    ESP_LOGI(TAG_MOTION, "  Using VMAX=%.1f RPM (%.1f°/s), AMAX=%.2f rev/s² (%.1f°/s²)", 
             vmax_rpm_, vmax_deg_per_s, amax_rev_s2_, amax_deg_per_s2);
}

void FatigueTestMotion::ClipLocalBoundsToGlobal() noexcept {
    if (!bounded_)
        return;

    float old_min = local_min_bound_;
    float old_max = local_max_bound_;

    // Clip local bounds to global bounds (all in degrees)
    local_min_bound_ = std::max(local_min_bound_, global_min_bound_);
    local_max_bound_ = std::min(local_max_bound_, global_max_bound_);

    // Update home position
    home_position_ = (local_min_bound_ + local_max_bound_) / 2.0f;
    amplitude_ = (local_max_bound_ - local_min_bound_) / 2.0f;

    if (fabsf(old_min - local_min_bound_) > 0.01f || fabsf(old_max - local_max_bound_) > 0.01f) {
        ESP_LOGW(TAG_MOTION, "Local bounds clipped to global bounds");
        ESP_LOGI(TAG_MOTION, "Clipped local bounds: min=%.2f°, max=%.2f°", local_min_bound_, local_max_bound_);
    }
}

void FatigueTestMotion::Update() noexcept {
    // Check if running
    {
        TmcMutexGuard guard(driver_mutex_);
        // Don't update if stopped
        if (!running_ || state_ == MotionState::STOPPED) {
            return;
        }
        
        // If paused, allow current move to finish but don't command next target
        if (state_ == MotionState::PAUSED) {
            // Check if current move has finished - if so, set to HOLD mode
            auto target_reached = driver_->rampControl.IsTargetReached();
            if (target_reached.IsOk() && target_reached.Value()) {
                // Current move finished - set to HOLD mode to maintain position
                driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
            }
            return;  // Don't command next target while paused
        }

        // Check if cycle count reached
        if (target_cycles_ > 0 && current_cycles_ >= target_cycles_) {
            if (!cycle_complete_) {
                cycle_complete_ = true;
                uint32_t cycles = current_cycles_;
                state_ = MotionState::STOPPED;
                running_ = false;
                
                // Stop the ramp generator
                driver_->rampControl.Stop();
                
                // CRITICAL: Clear target by setting XTARGET = XACTUAL to prevent lingering targets
                auto current_pos = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
                if (current_pos.IsOk()) {
                    driver_->rampControl.SetTargetPosition(current_pos.Value(), tmc51x0::Unit::Deg);
                }
                
                // Set HOLD mode
                driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
                
                guard.unlock();
                ESP_LOGI(TAG_MOTION, "Target cycle count reached: %lu cycles. Stopped and holding.", cycles);
            }
            return;
        }
    }

    // Get current state and bounds
    MotionState current_state;
    float min_bound, max_bound;
    uint32_t dwell_min, dwell_max;
    uint32_t target_cycles;
    
    {
        TmcMutexGuard guard(driver_mutex_);
        current_state = state_;
        min_bound = local_min_bound_;
        max_bound = local_max_bound_;
        dwell_min = dwell_at_min_ms_;
        dwell_max = dwell_at_max_ms_;
        target_cycles = target_cycles_;
    }

    // Check if target reached (for moving states)
    bool target_reached = false;
    if (current_state == MotionState::MOVING_TO_MAX || current_state == MotionState::MOVING_TO_MIN) {
        auto reached_result = driver_->rampControl.IsTargetReached();
        target_reached = reached_result.IsOk() && reached_result.Value();
    }

    // Get current position for cycle counting
    auto current_pos_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
    float dist_to_min = fabsf(current_pos_deg - min_bound);
    float dist_to_max = fabsf(current_pos_deg - max_bound);
    constexpr float NEAR_THRESHOLD_DEG = 2.0f;

    // State machine: point-to-point motion (like bounds_finding_test.cpp)
    switch (current_state) {
    case MotionState::PAUSED:
        // Paused state - do nothing, Update() already returned early
        // This case should never be reached, but included for completeness
        return;
    case MotionState::MOVING_TO_MAX:
        if (target_reached || dist_to_max < NEAR_THRESHOLD_DEG) {
            // Reached MAX - enter dwell (which includes settle time) or move to MIN
            TmcMutexGuard guard(driver_mutex_);
            // Check if paused - if so, don't transition, just wait
            if (state_ == MotionState::PAUSED) {
                // Paused - set to HOLD and wait for resume
                driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
                return;
            }
            // Enter dwell if requested (no minimum enforced - remote controller controls this)
            if (dwell_max > 0) {
                state_ = MotionState::DWELL_AT_MAX;
                dwell_start_time_ms_ = esp_timer_get_time() / 1000;
            } else {
                // No dwell, immediately move to MIN (dwell time already provides settle)
                state_ = MotionState::MOVING_TO_MIN;
                driver_->rampControl.SetTargetPosition(min_bound, tmc51x0::Unit::Deg);
            }
        }
        break;

    case MotionState::DWELL_AT_MAX:
        {
            uint32_t current_time_ms = esp_timer_get_time() / 1000;
            uint32_t dwell_start;
            {
                TmcMutexGuard guard(driver_mutex_);
                dwell_start = dwell_start_time_ms_;
                // Check if paused - if so, don't transition
                if (state_ == MotionState::PAUSED) {
                    return;
                }
            }
            if (current_time_ms - dwell_start >= dwell_max) {
                // Dwell complete, move to MIN
                TmcMutexGuard guard(driver_mutex_);
                // Double-check not paused (could have been paused during dwell)
                if (state_ == MotionState::PAUSED) {
                    return;
                }
                state_ = MotionState::MOVING_TO_MIN;
                driver_->rampControl.SetTargetPosition(min_bound, tmc51x0::Unit::Deg);
            }
        }
        break;

    case MotionState::MOVING_TO_MIN:
        if (target_reached || dist_to_min < NEAR_THRESHOLD_DEG) {
            // Reached MIN - this completes a cycle! Count it.
            uint32_t new_cycles;
            bool should_stop = false;
            {
                TmcMutexGuard guard(driver_mutex_);
                // Check if paused - if so, don't transition, just wait
                if (state_ == MotionState::PAUSED) {
                    // Paused - set to HOLD and wait for resume
                    driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
                    return;
                }
                
                // This completes a cycle! Count it.
                current_cycles_++;
                new_cycles = current_cycles_;
                
                // Check if target reached
                if (target_cycles > 0 && new_cycles >= target_cycles) {
                    cycle_complete_ = true;
                    running_ = false;
                    state_ = MotionState::STOPPED;
                    driver_->rampControl.Stop();
                    should_stop = true;
                } else {
                    // Enter dwell or move to MAX for next cycle
                    // Dwell time is controlled by remote controller (no minimum enforced)
                    if (dwell_min > 0) {
                        state_ = MotionState::DWELL_AT_MIN;
                        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
                    } else {
                        // No dwell, immediately move to MAX for next cycle
                        state_ = MotionState::MOVING_TO_MAX;
                        driver_->rampControl.SetTargetPosition(max_bound, tmc51x0::Unit::Deg);
                    }
                }
            }
            
            if (should_stop) {
                ESP_LOGI(TAG_MOTION, "Target cycle count reached: %lu cycles. Stopping.", new_cycles);
                return;
            } else {
                ESP_LOGI(TAG_MOTION, "Cycle %lu completed at MIN position (target: %lu)", 
                         new_cycles, target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
            }
        }
        break;

    case MotionState::DWELL_AT_MIN:
        {
            uint32_t current_time_ms = esp_timer_get_time() / 1000;
            uint32_t dwell_start;
            {
                TmcMutexGuard guard(driver_mutex_);
                dwell_start = dwell_start_time_ms_;
                // Check if paused - if so, don't transition
                if (state_ == MotionState::PAUSED) {
                    return;
                }
            }
            if (current_time_ms - dwell_start >= dwell_min) {
                // Dwell complete, move to MAX for next cycle
                TmcMutexGuard guard(driver_mutex_);
                // Double-check not paused (could have been paused during dwell)
                if (state_ == MotionState::PAUSED) {
                    return;
                }
                state_ = MotionState::MOVING_TO_MAX;
                driver_->rampControl.SetTargetPosition(max_bound, tmc51x0::Unit::Deg);
            }
        }
        break;

    case MotionState::STOPPED:
        // Do nothing
        break;
    }
}

FatigueTestMotion::Status FatigueTestMotion::GetStatus() const noexcept {
    Status status{};
    {
        TmcMutexGuard guard(driver_mutex_);
        status.running = running_ && state_ != MotionState::STOPPED;
        status.bounded = bounded_;
        status.vmax_rpm = vmax_rpm_;
        status.amax_rev_s2 = amax_rev_s2_;
        status.estimated_frequency_hz = estimated_frequency_hz_;
        status.current_cycles = current_cycles_;
        status.target_cycles = target_cycles_;
        status.dwell_min_ms = dwell_at_min_ms_;
        status.dwell_max_ms = dwell_at_max_ms_;
        status.min_degrees_from_center = local_min_bound_;
        status.max_degrees_from_center = local_max_bound_;
        status.global_min_degrees = global_min_bound_;
        status.global_max_degrees = global_max_bound_;
    }
    
    return status;
}

// Close namespace only if we opened it (standalone compilation)
#ifndef FATIGUE_MOTION_HEADER_INCLUDED
} // namespace FatigueTest
#endif
