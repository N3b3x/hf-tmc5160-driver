/**
 * @file fatigue_motion_impl.hpp
 * @brief Full implementation of FatigueTestMotion class
 * 
 * Complete implementation extracted from fatigue_test_stallguard.cpp and fatigue_test_encoder.cpp
 * Provides full-featured sinusoidal motion with trajectory calculation, dwell handling, and cycle counting.
 */

#pragma once

#include "fatigue_motion.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

static const char* TAG_MOTION = "FatigueMotion";

namespace FatigueTest {

// Note: Esp32TmcMutex and TmcMutexGuard must be defined before including this file
// They are defined in main.cpp before this include

// Implementation

FatigueTestMotion::FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver) noexcept
    : driver_(driver), global_min_bound_(0.0f), global_max_bound_(0.0f), local_min_bound_(0.0f), local_max_bound_(0.0f),
      home_position_(0.0f), bounded_(false), amplitude_(1000.0F), frequency_hz_(0.5F), 
      dwell_at_min_ms_(0), dwell_at_max_ms_(0), running_(false), start_time_us_(0),
      phase_offset_(0.0F), target_cycles_(0), current_cycles_(0), cycle_complete_(false),
      last_was_negative_(false), cycle_started_(false), last_target_relative_(0.0f), 
      state_(MotionState::STOPPED), dwell_start_time_ms_(0), sinusoidal_mode_(false), 
      calculated_vmax_rpm_(30.0f), calculated_amax_rev_s2_(2.0f), estimated_frequency_hz_(0.0f) {
    // Mutex is automatically created by Esp32TmcMutex constructor
    // Note: Initialization order matches member declaration order in header
}

FatigueTestMotion::~FatigueTestMotion() noexcept = default;


void FatigueTestMotion::SetGlobalBounds(float min_bound_degrees, float max_bound_degrees) noexcept {
    {
        TmcMutexGuard guard(mutex_);
        global_min_bound_ = min_bound_degrees;
        global_max_bound_ = max_bound_degrees;
        bounded_ = true;
    }
    ESP_LOGI(TAG_MOTION, "Global bounds set: min=%.2f°, max=%.2f°", global_min_bound_, global_max_bound_);

    // Clip local bounds to global bounds if they exist
    {
        TmcMutexGuard guard(mutex_);
        if (local_min_bound_ != 0.0f || local_max_bound_ != 0.0f) {
            guard.unlock();
            ClipLocalBoundsToGlobal();
            return;
        }
    }
}

void FatigueTestMotion::GetGlobalBoundsDegrees(float& min_degrees, float& max_degrees) const noexcept {
    TmcMutexGuard guard(mutex_);
    min_degrees = global_min_bound_;
    max_degrees = global_max_bound_;
}

void FatigueTestMotion::SetUnbounded(float current_position_degrees, float default_range_degrees) noexcept {
    {
        TmcMutexGuard guard(mutex_);
        bounded_ = false;
        home_position_ = current_position_degrees;
        global_min_bound_ = current_position_degrees - default_range_degrees / 2.0f;
        global_max_bound_ = current_position_degrees + default_range_degrees / 2.0f;
    }
    // Establish home/zero at current position
    // IMPORTANT: After this point, use ABSOLUTE positioning (SetTargetPosition)
    // Before this, we would use RELATIVE positioning (MoveRelative) if needed
    driver_->rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
    ESP_LOGW(TAG_MOTION, "Unbounded mode: No mechanical stops found");
    ESP_LOGI(TAG_MOTION, "Using current position as home: %.2f°", current_position_degrees);
    ESP_LOGI(TAG_MOTION, "Default global range: [%.2f°, %.2f°]", global_min_bound_, global_max_bound_);
}

bool FatigueTestMotion::SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center) noexcept {
    float min_deg, max_deg;
    bool is_bounded;
    float global_min, global_max;
    {
        TmcMutexGuard guard(mutex_);
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

    {
        TmcMutexGuard guard(mutex_);
        local_min_bound_ = min_deg;
        local_max_bound_ = max_deg;
        home_position_ = (local_min_bound_ + local_max_bound_) / 2.0f;
        amplitude_ = (local_max_bound_ - local_min_bound_) / 2.0f;
    }

    ESP_LOGI(TAG_MOTION, "Local bounds set: min=%.2f°, max=%.2f° from center", min_deg, max_deg);
    
    // Recalculate trajectory with new bounds
    {
        TmcMutexGuard guard(mutex_);
        RecalculateTrajectory();
    }
    return true;
}

void FatigueTestMotion::GetLocalBoundsFromCenterDegrees(float& min_degrees, float& max_degrees) const noexcept {
    TmcMutexGuard guard(mutex_);
    min_degrees = local_min_bound_;
    max_degrees = local_max_bound_;
}

bool FatigueTestMotion::SetFrequency(float frequency_hz) noexcept {
    if (frequency_hz < 0.0F || frequency_hz > 10.0F) {
        ESP_LOGE(TAG_MOTION, "Invalid frequency: %.2f Hz (range: 0.0-10.0)", frequency_hz);
        return false;
    }
    {
        TmcMutexGuard guard(mutex_);
        frequency_hz_ = frequency_hz;
        RecalculateTrajectory();
    }
    ESP_LOGI(TAG_MOTION, "Frequency updated: %.2f Hz", frequency_hz);
    return true;
}

float FatigueTestMotion::GetFrequency() const noexcept {
    TmcMutexGuard guard(mutex_);
    return frequency_hz_;
}

bool FatigueTestMotion::SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms) noexcept {
    {
        TmcMutexGuard guard(mutex_);
        dwell_at_min_ms_ = dwell_at_min_ms;
        dwell_at_max_ms_ = dwell_at_max_ms;
        RecalculateTrajectory();
    }
    ESP_LOGI(TAG_MOTION, "Dwell times updated: min=%lu ms, max=%lu ms", 
             dwell_at_min_ms, dwell_at_max_ms);
    return true;
}

void FatigueTestMotion::GetDwellTimes(uint32_t& dwell_at_min_ms, uint32_t& dwell_at_max_ms) const noexcept {
    TmcMutexGuard guard(mutex_);
    dwell_at_min_ms = dwell_at_min_ms_;
    dwell_at_max_ms = dwell_at_max_ms_;
}

bool FatigueTestMotion::SetTargetCycles(uint32_t cycles) noexcept {
    {
        TmcMutexGuard guard(mutex_);
        target_cycles_ = cycles;
    }
    ESP_LOGI(TAG_MOTION, "Target cycles set: %lu (0 = infinite)", target_cycles_);
    return true;
}

uint32_t FatigueTestMotion::GetCurrentCycles() const noexcept {
    TmcMutexGuard guard(mutex_);
    return current_cycles_;
}

uint32_t FatigueTestMotion::GetTargetCycles() const noexcept {
    TmcMutexGuard guard(mutex_);
    return target_cycles_;
}

bool FatigueTestMotion::IsCycleComplete() const noexcept {
    TmcMutexGuard guard(mutex_);
    return cycle_complete_;
}

void FatigueTestMotion::ResetCycles() noexcept {
    {
        TmcMutexGuard guard(mutex_);
        current_cycles_ = 0;
        cycle_complete_ = false;
        last_was_negative_ = false;
        cycle_started_ = false;
        last_target_relative_ = 0.0f;
    }
    ESP_LOGI(TAG_MOTION, "Cycle count reset");
}

bool FatigueTestMotion::IsBounded() const noexcept {
    TmcMutexGuard guard(mutex_);
    return bounded_;
}

bool FatigueTestMotion::Start() noexcept {
    uint32_t current_cycles, target_cycles;
    float min_pos, max_pos, current_pos;
    
    {
        TmcMutexGuard guard(mutex_);
        if (fabsf(local_min_bound_) < 0.01f && fabsf(local_max_bound_) < 0.01f) {
            ESP_LOGE(TAG_MOTION, "Cannot start: local bounds not set!");
            return false;
        }

        if (cycle_complete_) {
            ESP_LOGW(TAG_MOTION, "Cycle count reached. Reset cycles or set new target to continue.");
            return false;
        }
        
        // Update trajectory before starting
        RecalculateTrajectory();

        // Configure driver for positioning mode
        // Values are already in proper units (RPM and rev/s²) - use directly
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
        driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetRampSpeeds(30.0f, 3.0f, 0.0f, tmc51x0::Unit::RPM);

        running_ = true;
        start_time_us_ = esp_timer_get_time();
        sinusoidal_mode_ = true; // Use sinusoidal mode
        
        // Determine initial state based on current position
        auto current_pos_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
        float min_pos_deg = local_min_bound_;
        float max_pos_deg = local_max_bound_;
        
        // Find closest bound or determine direction
        float dist_to_min = fabsf(current_pos_deg - min_pos_deg);
        float dist_to_max = fabsf(current_pos_deg - max_pos_deg);
        
        // Default to moving towards min unless we're already there
        if (dist_to_min < 1.0f) {  // ~1 degree threshold
            state_ = MotionState::MOVING_TO_MAX;
        } else {
            state_ = MotionState::MOVING_TO_MIN;
        }

        current_cycles = current_cycles_;
        target_cycles = target_cycles_;
    }

    ESP_LOGI(TAG_MOTION, "Starting fatigue test (cycles: %lu/%lu)", current_cycles,
             target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
    ESP_LOGI(TAG_MOTION, "  Motion: Sinusoidal mode, VMAX=%.2f RPM, AMAX=%.3f rev/s²", 
             calculated_vmax_rpm_, calculated_amax_rev_s2_);
    return true;
}

void FatigueTestMotion::Stop() noexcept {
    uint32_t cycles;
    {
        TmcMutexGuard guard(mutex_);
        running_ = false;
        state_ = MotionState::STOPPED;
        cycles = current_cycles_;
    }
    driver_->rampControl.Stop();
    ESP_LOGI(TAG_MOTION, "Stopped fatigue test motion (cycles completed: %lu)", cycles);
}

bool FatigueTestMotion::IsRunning() const noexcept {
    TmcMutexGuard guard(mutex_);
    return running_ && state_ != MotionState::STOPPED;
}

void FatigueTestMotion::RecalculateTrajectory() noexcept {
    // Calculate total travel distance (one way) in degrees
    float distance_deg = fabsf(local_max_bound_ - local_min_bound_);
    if (distance_deg < 0.1f || frequency_hz_ <= 0.0001f) {
        // Default values in proper units
        calculated_vmax_rpm_ = 30.0f;      // 30 RPM default
        calculated_amax_rev_s2_ = 2.0f;    // 2 rev/s² default
        estimated_frequency_hz_ = 0.0f;
        return;
    }

    // Target cycle period (total time for there + back + dwells)
    float target_period_s = 1.0f / frequency_hz_;
    
    // Total dwell time per cycle (min + max)
    float total_dwell_s = (dwell_at_min_ms_ + dwell_at_max_ms_) / 1000.0f;
    
    // Time available for motion (there + back)
    float total_move_time_s = target_period_s - total_dwell_s;
    
    if (total_move_time_s <= 0.0f) {
        total_move_time_s = 0.1f;
        ESP_LOGW(TAG_MOTION, "Requested frequency %.2f Hz is impossible with given dwell times!", frequency_hz_);
    }
    
    // Time for one leg (one way)
    float leg_time_s = total_move_time_s / 2.0f;
    
    // Calculate velocity in degrees per second
    // Using 1.5x factor for trapezoidal profile (allows for acceleration/deceleration phases)
    float velocity_deg_per_s = (1.5f * distance_deg) / leg_time_s;
    
    // Convert to RPM: (deg/s / 360 deg/rev) * 60 s/min = RPM
    calculated_vmax_rpm_ = (velocity_deg_per_s / 360.0f) * 60.0f;
    
    // Calculate acceleration in rev/s²
    // Acceleration = velocity_change / time_to_reach_velocity
    // Time to reach velocity is approximately leg_time / 3 (for trapezoidal profile)
    float time_to_reach_velocity = leg_time_s / 3.0f;
    float velocity_rev_per_s = calculated_vmax_rpm_ / 60.0f;
    calculated_amax_rev_s2_ = velocity_rev_per_s / time_to_reach_velocity;
    
    // Clamp to reasonable limits (300 RPM max, 10 rev/s² max)
    if (calculated_vmax_rpm_ > 300.0f) calculated_vmax_rpm_ = 300.0f;
    if (calculated_amax_rev_s2_ > 10.0f) calculated_amax_rev_s2_ = 10.0f;
    if (calculated_vmax_rpm_ < 1.0f) calculated_vmax_rpm_ = 1.0f;  // Minimum 1 RPM
    if (calculated_amax_rev_s2_ < 0.1f) calculated_amax_rev_s2_ = 0.1f;  // Minimum 0.1 rev/s²
    
    estimated_frequency_hz_ = 1.0f / (2.0f * leg_time_s + total_dwell_s);
    
    ESP_LOGI(TAG_MOTION, "Trajectory Recalculated: Dist=%.2f°, LegTime=%.3fs", distance_deg, leg_time_s);
    ESP_LOGI(TAG_MOTION, "  Target Freq=%.2fHz, Est Freq=%.2fHz", frequency_hz_, estimated_frequency_hz_);
    ESP_LOGI(TAG_MOTION, "  VMAX=%.2f RPM, AMAX=%.3f rev/s²", 
             calculated_vmax_rpm_, calculated_amax_rev_s2_);
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

void FatigueTestMotion::UpdateSinuousMotion() noexcept {
    uint64_t elapsed_us;
    float freq, amp;
    float home, local_min, local_max;
    uint32_t target_cycles;
    bool cycle_started;
    float last_target_rel;
    uint32_t dwell_min, dwell_max;
    float phase_off;
    
    {
        TmcMutexGuard guard(mutex_);
        elapsed_us = esp_timer_get_time() - start_time_us_;
        freq = frequency_hz_;
        amp = amplitude_;
        home = home_position_;
        local_min = local_min_bound_;
        local_max = local_max_bound_;
        target_cycles = target_cycles_;
        cycle_started = cycle_started_;
        last_target_rel = last_target_relative_;
        dwell_min = dwell_at_min_ms_;
        dwell_max = dwell_at_max_ms_;
        phase_off = phase_offset_;
    }

    // Calculate sinusoidal position
    double elapsed_s = elapsed_us / 1000000.0;
    double angle = 2.0 * M_PI * freq * elapsed_s + phase_off;
    double sin_value = sin(angle);

    // Calculate target position in degrees
    float target_deg = home + static_cast<float>(amp * sin_value);

    // Get current position relative to center for cycle counting
    auto current_pos_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
    float target_relative = target_deg - home;

    // Cycle counting: one cycle = center → extreme → center
    bool currently_negative = (target_relative < 0.0f);
    bool last_was_negative = (last_target_rel < 0.0f);
    bool crossing_center =
        (last_was_negative != currently_negative) && (fabsf(target_relative) < 1.0f) && (fabsf(last_target_rel) < 1.0f);

    // If we've started a cycle (left center) and now crossing back through center
    if (cycle_started && crossing_center) {
        // Completed a cycle
        uint32_t new_cycles;
        {
            TmcMutexGuard guard(mutex_);
            current_cycles_++;
            cycle_started_ = false;
            new_cycles = current_cycles_;
            
            // Check if target reached
            if (target_cycles > 0 && new_cycles >= target_cycles) {
                cycle_complete_ = true;
                running_ = false;
                state_ = MotionState::STOPPED;
                guard.unlock();
                driver_->rampControl.Stop();
                ESP_LOGI(TAG_MOTION, "Target cycle count reached: %lu cycles. Stopping.", new_cycles);
                return;
            }
        }
        ESP_LOGI(TAG_MOTION, "Cycle %lu completed at center (target: %lu)", new_cycles, 
                 target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
    } else if (!cycle_started && fabsf(target_relative) > 1.0f) {
        // We've left center, cycle has started
        TmcMutexGuard guard(mutex_);
        cycle_started_ = true;
        last_was_negative_ = currently_negative;
    }

    // Update tracking
    {
        TmcMutexGuard guard(mutex_);
        last_target_relative_ = target_relative;
        if (fabsf(target_relative) > 0.5f) {
            last_was_negative_ = currently_negative;
        }
    }

    // Clamp to local bounds and handle dwell states
    // Use ABSOLUTE positioning - home is established via bounds finding or SetUnbounded()
    // target_deg is calculated as home + amplitude * sin(), so it's already absolute
    if (target_deg <= local_min) {
        target_deg = local_min;
        if (dwell_min > 0) {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::DWELL_AT_MIN;
            dwell_start_time_ms_ = esp_timer_get_time() / 1000;
            driver_->rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
            return;
        } else {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::MOVING_TO_MAX;
        }
    } else if (target_deg >= local_max) {
        target_deg = local_max;
        if (dwell_max > 0) {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::DWELL_AT_MAX;
            dwell_start_time_ms_ = esp_timer_get_time() / 1000;
            driver_->rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
            return;
        } else {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::MOVING_TO_MIN;
        }
    } else {
        TmcMutexGuard guard(mutex_);
        if (target_relative > 0.0f) {
            state_ = MotionState::MOVING_TO_MAX;
        } else {
            state_ = MotionState::MOVING_TO_MIN;
        }
    }

    // Update target position if it changed significantly
    // Use ABSOLUTE positioning - home is established, target_deg is absolute
    if (fabsf(target_deg - current_pos_deg) > 0.5f) {  // ~0.5 degree threshold
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        // Use slow speed for position correction (5 RPM, 1 rev/s²)
        constexpr float SLOW_VMAX_RPM = 5.0f;
        constexpr float SLOW_AMAX_REV_S2 = 1.0f;
        
        driver_->rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
        driver_->rampControl.SetMaxSpeed(SLOW_VMAX_RPM, tmc51x0::Unit::RPM);
        driver_->rampControl.SetAcceleration(SLOW_AMAX_REV_S2, tmc51x0::Unit::RevPerSec);
    }
}

void FatigueTestMotion::Update() noexcept {
    {
        TmcMutexGuard guard(mutex_);
        if (!running_ || state_ == MotionState::STOPPED) {
            return;
        }

        // Check if cycle count reached
        if (target_cycles_ > 0 && current_cycles_ >= target_cycles_) {
            if (!cycle_complete_) {
                cycle_complete_ = true;
                uint32_t cycles = current_cycles_;
                state_ = MotionState::STOPPED;
                running_ = false;
                guard.unlock();
                driver_->rampControl.Stop();
                ESP_LOGI(TAG_MOTION, "Target cycle count reached: %lu cycles. Stopping.", cycles);
            }
            return;
        }
    }

    // Check if we're in sinusoidal mode
    bool use_sinusoidal;
    MotionState current_state;
    {
        TmcMutexGuard guard(mutex_);
        use_sinusoidal = sinusoidal_mode_;
        current_state = state_;
    }
    
    // If sinusoidal mode and not in a dwell state, use sinusoidal update
    if (use_sinusoidal && current_state != MotionState::DWELL_AT_MIN && 
        current_state != MotionState::DWELL_AT_MAX) {
        UpdateSinuousMotion();
        return;
    }
    
    // Handle dwell states
    uint32_t current_time_ms = esp_timer_get_time() / 1000;
    uint32_t dwell_min, dwell_max, dwell_start;
    float min_bound, max_bound;
    
    {
        TmcMutexGuard guard(mutex_);
        current_state = state_;
        dwell_min = dwell_at_min_ms_;
        dwell_max = dwell_at_max_ms_;
        dwell_start = dwell_start_time_ms_;
        min_bound = local_min_bound_;  // Already in degrees
        max_bound = local_max_bound_;  // Already in degrees
    }

    switch (current_state) {
    case MotionState::DWELL_AT_MIN:
        if (current_time_ms - dwell_start >= dwell_min) {
            TmcMutexGuard guard(mutex_);
            if (!sinusoidal_mode_) {
                // Values are already in proper units (RPM and rev/s²) - use directly
                state_ = MotionState::MOVING_TO_MAX;
                driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
                driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
                driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
                driver_->rampControl.SetTargetPosition(max_bound, tmc51x0::Unit::Deg);
            } else {
                state_ = MotionState::MOVING_TO_MAX;
            }
        }
        break;

    case MotionState::DWELL_AT_MAX:
        if (current_time_ms - dwell_start >= dwell_max) {
            TmcMutexGuard guard(mutex_);
            if (!sinusoidal_mode_) {
                // Values are already in proper units (RPM and rev/s²) - use directly
                state_ = MotionState::MOVING_TO_MIN;
                driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
                driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
                driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
                driver_->rampControl.SetTargetPosition(min_bound, tmc51x0::Unit::Deg);
            } else {
                state_ = MotionState::MOVING_TO_MIN;
            }
        }
        break;

    case MotionState::MOVING_TO_MIN:
    case MotionState::MOVING_TO_MAX:
    case MotionState::STOPPED:
        // These are handled by UpdateSinuousMotion or ramp control
        break;
    }
}

FatigueTestMotion::Status FatigueTestMotion::GetStatus() const noexcept {
    Status status{};
    {
        TmcMutexGuard guard(mutex_);
        status.running = running_ && state_ != MotionState::STOPPED;
        status.bounded = bounded_;
        status.frequency_hz = frequency_hz_;
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

float FatigueTestMotion::GetEstimatedFrequency() const noexcept {
    TmcMutexGuard guard(mutex_);
    return estimated_frequency_hz_;
}

} // namespace FatigueTest
