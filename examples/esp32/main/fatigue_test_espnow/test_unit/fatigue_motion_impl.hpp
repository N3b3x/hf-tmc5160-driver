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
    : driver_(driver), global_min_bound_(0), global_max_bound_(0), local_min_bound_(0), local_max_bound_(0),
      home_position_(0), bounded_(false), amplitude_(1000.0F), frequency_hz_(0.5F), 
      dwell_at_min_ms_(0), dwell_at_max_ms_(0), running_(false), start_time_us_(0),
      phase_offset_(0.0F), target_cycles_(0), current_cycles_(0), cycle_complete_(false),
      last_was_negative_(false), cycle_started_(false), last_target_relative_(0), 
      state_(MotionState::STOPPED), dwell_start_time_ms_(0), sinusoidal_mode_(false), 
      calculated_vmax_(10000.0f), calculated_amax_(5000.0f), estimated_frequency_hz_(0.0f),
      steps_per_rev_(200), angle_unit_(AngleUnit::DEGREES) {
    // Mutex is automatically created by Esp32TmcMutex constructor
    // Note: Initialization order matches member declaration order in header
}

FatigueTestMotion::~FatigueTestMotion() noexcept = default;

void FatigueTestMotion::ConfigureMotor(uint16_t steps_per_rev, AngleUnit unit) noexcept {
    TmcMutexGuard guard(mutex_);
    steps_per_rev_ = steps_per_rev;
    angle_unit_ = unit;
    ESP_LOGI(TAG_MOTION, "Motor configured: %d steps/rev, angle unit: %s", steps_per_rev_,
             unit == AngleUnit::DEGREES ? "degrees" : "radians");
}

void FatigueTestMotion::SetGlobalBounds(int32_t min_bound, int32_t max_bound) noexcept {
    {
        TmcMutexGuard guard(mutex_);
        global_min_bound_ = min_bound;
        global_max_bound_ = max_bound;
        bounded_ = true;
    }
    ESP_LOGI(TAG_MOTION, "Global bounds set: min=%d, max=%d steps", global_min_bound_, global_max_bound_);
    if (steps_per_rev_ > 0) {
        float min_deg = tmc51x0::StepsToDegrees(global_min_bound_, steps_per_rev_);
        float max_deg = tmc51x0::StepsToDegrees(global_max_bound_, steps_per_rev_);
        ESP_LOGI(TAG_MOTION, "Global bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
    }

    // Clip local bounds to global bounds if they exist
    {
        TmcMutexGuard guard(mutex_);
        if (local_min_bound_ != 0 || local_max_bound_ != 0) {
            guard.unlock();
            ClipLocalBoundsToGlobal();
            return;
        }
    }
}

void FatigueTestMotion::GetGlobalBoundsDegrees(float& min_degrees, float& max_degrees) const noexcept {
    int32_t min_bound, max_bound;
    uint16_t steps;
    {
        TmcMutexGuard guard(mutex_);
        if (steps_per_rev_ == 0) {
            min_degrees = 0.0F;
            max_degrees = 0.0F;
            return;
        }
        min_bound = global_min_bound_;
        max_bound = global_max_bound_;
        steps = steps_per_rev_;
    }
    min_degrees = tmc51x0::StepsToDegrees(min_bound, steps);
    max_degrees = tmc51x0::StepsToDegrees(max_bound, steps);
}

void FatigueTestMotion::SetUnbounded(int32_t current_position, int32_t default_range_steps) noexcept {
    int32_t min_bound, max_bound;
    uint16_t steps;
    {
        TmcMutexGuard guard(mutex_);
        bounded_ = false;
        home_position_ = current_position;
        global_min_bound_ = current_position - default_range_steps / 2;
        global_max_bound_ = current_position + default_range_steps / 2;
        min_bound = global_min_bound_;
        max_bound = global_max_bound_;
        steps = steps_per_rev_;
    }
    driver_->rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
    ESP_LOGW(TAG_MOTION, "Unbounded mode: No mechanical stops found");
    ESP_LOGI(TAG_MOTION, "Using current position as home: %d steps", current_position);
    ESP_LOGI(TAG_MOTION, "Default global range: [%d, %d] steps", min_bound, max_bound);
    if (steps > 0) {
        float range_deg = tmc51x0::StepsToDegrees(default_range_steps, steps);
        ESP_LOGI(TAG_MOTION, "Default global range: %.2f°", range_deg);
    }
}

bool FatigueTestMotion::SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center) noexcept {
    if (steps_per_rev_ == 0) {
        ESP_LOGE(TAG_MOTION, "Cannot set bounds: steps_per_rev not configured");
        return false;
    }

    float min_deg, max_deg;
    uint16_t steps;
    bool is_bounded;
    int32_t global_min, global_max;
    {
        TmcMutexGuard guard(mutex_);
        min_deg = min_degrees_from_center;
        max_deg = max_degrees_from_center;
        steps = steps_per_rev_;
        is_bounded = bounded_;
        global_min = global_min_bound_;
        global_max = global_max_bound_;
    }

    // Convert to steps
    int32_t min_steps = tmc51x0::DegreesToSteps(min_deg, steps);
    int32_t max_steps = tmc51x0::DegreesToSteps(max_deg, steps);

    // Clip to global bounds
    if (is_bounded) {
        min_steps = std::max(min_steps, global_min);
        max_steps = std::min(max_steps, global_max);
    }

    {
        TmcMutexGuard guard(mutex_);
        local_min_bound_ = min_steps;
        local_max_bound_ = max_steps;
        home_position_ = (local_min_bound_ + local_max_bound_) / 2;
        amplitude_ = static_cast<float>((local_max_bound_ - local_min_bound_) / 2);
    }

    float actual_min = tmc51x0::StepsToDegrees(min_steps, steps);
    float actual_max = tmc51x0::StepsToDegrees(max_steps, steps);
    ESP_LOGI(TAG_MOTION, "Local bounds set: min=%.2f°, max=%.2f° from center", actual_min, actual_max);
    
    // Recalculate trajectory with new bounds
    {
        TmcMutexGuard guard(mutex_);
        RecalculateTrajectory();
    }
    return true;
}

void FatigueTestMotion::GetLocalBoundsFromCenterDegrees(float& min_degrees, float& max_degrees) const noexcept {
    int32_t min_bound, max_bound;
    uint16_t steps;
    {
        TmcMutexGuard guard(mutex_);
        if (steps_per_rev_ == 0) {
            min_degrees = 0.0F;
            max_degrees = 0.0F;
            return;
        }
        min_bound = local_min_bound_;
        max_bound = local_max_bound_;
        steps = steps_per_rev_;
    }
    min_degrees = tmc51x0::StepsToDegrees(min_bound, steps);
    max_degrees = tmc51x0::StepsToDegrees(max_bound, steps);
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
        last_target_relative_ = 0;
    }
    ESP_LOGI(TAG_MOTION, "Cycle count reset");
}

bool FatigueTestMotion::IsBounded() const noexcept {
    TmcMutexGuard guard(mutex_);
    return bounded_;
}

bool FatigueTestMotion::Start() noexcept {
    uint32_t current_cycles, target_cycles;
    int32_t min_pos, max_pos, current_pos;
    
    {
        TmcMutexGuard guard(mutex_);
        if (local_min_bound_ == 0 && local_max_bound_ == 0) {
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
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver_->rampControl.SetMaxSpeed(calculated_vmax_);
        driver_->rampControl.SetAcceleration(calculated_amax_);
        driver_->rampControl.SetDeceleration(calculated_amax_);
        driver_->rampControl.SetRampSpeeds(1000.0f, 100.0f, 0.0f);

        running_ = true;
        start_time_us_ = esp_timer_get_time();
        sinusoidal_mode_ = true; // Use sinusoidal mode
        
        // Determine initial state based on current position
        float current_pos_float = 0.0f;
        if (!driver_->rampControl.GetCurrentPosition(current_pos_float, tmc51x0::Unit::Steps)) {
            current_pos_float = 0.0f;
        }
        current_pos = static_cast<int32_t>(current_pos_float);
        min_pos = local_min_bound_;
        max_pos = local_max_bound_;
        
        // Find closest bound or determine direction
        int32_t dist_to_min = abs(current_pos - min_pos);
        int32_t dist_to_max = abs(current_pos - max_pos);
        
        // Default to moving towards min unless we're already there
        if (dist_to_min < 100) {
            state_ = MotionState::MOVING_TO_MAX;
        } else {
            state_ = MotionState::MOVING_TO_MIN;
        }

        current_cycles = current_cycles_;
        target_cycles = target_cycles_;
    }

    ESP_LOGI(TAG_MOTION, "Starting fatigue test (cycles: %lu/%lu)", current_cycles,
             target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
    ESP_LOGI(TAG_MOTION, "  Motion: Sinusoidal mode, VMAX=%.1f, AMAX=%.1f", 
             calculated_vmax_, calculated_amax_);
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
    // Calculate total travel distance (one way)
    int32_t distance = abs(local_max_bound_ - local_min_bound_);
    if (distance == 0 || frequency_hz_ <= 0.0001f) {
        calculated_vmax_ = 1000.0f;
        calculated_amax_ = 5000.0f;
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
    
    // Calculate VMAX and AMAX for Trapezoidal Profile
    calculated_vmax_ = (1.5f * distance) / leg_time_s;
    calculated_amax_ = calculated_vmax_ / (leg_time_s / 3.0f);
    
    // Clamp to driver limits
    if (calculated_vmax_ > 5000000.0f) calculated_vmax_ = 5000000.0f;
    if (calculated_amax_ > 5000000.0f) calculated_amax_ = 5000000.0f;
    
    estimated_frequency_hz_ = 1.0f / (2.0f * leg_time_s + total_dwell_s);
    
    ESP_LOGI(TAG_MOTION, "Trajectory Recalculated: Dist=%ld steps, LegTime=%.3fs", distance, leg_time_s);
    ESP_LOGI(TAG_MOTION, "  Target Freq=%.2fHz, Est Freq=%.2fHz", frequency_hz_, estimated_frequency_hz_);
    ESP_LOGI(TAG_MOTION, "  VMAX=%.1f, AMAX=%.1f", calculated_vmax_, calculated_amax_);
}

void FatigueTestMotion::ClipLocalBoundsToGlobal() noexcept {
    if (!bounded_)
        return;

    int32_t old_min = local_min_bound_;
    int32_t old_max = local_max_bound_;

    // Clip local bounds to global bounds
    local_min_bound_ = std::max(local_min_bound_, global_min_bound_);
    local_max_bound_ = std::min(local_max_bound_, global_max_bound_);

    // Update home position
    home_position_ = (local_min_bound_ + local_max_bound_) / 2;
    amplitude_ = static_cast<float>((local_max_bound_ - local_min_bound_) / 2);

    if (old_min != local_min_bound_ || old_max != local_max_bound_) {
        ESP_LOGW(TAG_MOTION, "Local bounds clipped to global bounds");
        ESP_LOGI(TAG_MOTION, "Clipped local bounds: min=%d, max=%d steps", local_min_bound_, local_max_bound_);
        if (steps_per_rev_ > 0) {
            float min_deg = tmc51x0::StepsToDegrees(local_min_bound_, steps_per_rev_);
            float max_deg = tmc51x0::StepsToDegrees(local_max_bound_, steps_per_rev_);
            ESP_LOGI(TAG_MOTION, "Clipped local bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
        }
    }
}

void FatigueTestMotion::UpdateSinuousMotion() noexcept {
    uint64_t elapsed_us;
    float freq, amp;
    int32_t home, local_min, local_max;
    uint32_t target_cycles;
    bool cycle_started;
    int32_t last_target_rel;
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

    // Calculate target position
    int32_t target = home + static_cast<int32_t>(amp * sin_value);

    // Get current position relative to center for cycle counting
    float current_pos_float = 0.0f;
    if (!driver_->rampControl.GetCurrentPosition(current_pos_float, tmc51x0::Unit::Steps)) {
        current_pos_float = 0.0f;
    }
    int32_t current_pos = static_cast<int32_t>(current_pos_float);
    int32_t target_relative = target - home;

    // Cycle counting: one cycle = center → extreme → center
    bool currently_negative = (target_relative < 0);
    bool last_was_negative = (last_target_rel < 0);
    bool crossing_center =
        (last_was_negative != currently_negative) && (abs(target_relative) < 30) && (abs(last_target_rel) < 30);

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
    } else if (!cycle_started && abs(target_relative) > 30) {
        // We've left center, cycle has started
        TmcMutexGuard guard(mutex_);
        cycle_started_ = true;
        last_was_negative_ = currently_negative;
    }

    // Update tracking
    {
        TmcMutexGuard guard(mutex_);
        last_target_relative_ = target_relative;
        if (abs(target_relative) > 10) {
            last_was_negative_ = currently_negative;
        }
    }

    // Clamp to local bounds and handle dwell states
    if (target <= local_min) {
        target = local_min;
        if (dwell_min > 0) {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::DWELL_AT_MIN;
            dwell_start_time_ms_ = esp_timer_get_time() / 1000;
            driver_->rampControl.SetTargetPosition(static_cast<float>(target), tmc51x0::Unit::Steps);
            return;
        } else {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::MOVING_TO_MAX;
        }
    } else if (target >= local_max) {
        target = local_max;
        if (dwell_max > 0) {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::DWELL_AT_MAX;
            dwell_start_time_ms_ = esp_timer_get_time() / 1000;
            driver_->rampControl.SetTargetPosition(static_cast<float>(target), tmc51x0::Unit::Steps);
            return;
        } else {
            TmcMutexGuard guard(mutex_);
            state_ = MotionState::MOVING_TO_MIN;
        }
    } else {
        TmcMutexGuard guard(mutex_);
        if (target_relative > 0) {
            state_ = MotionState::MOVING_TO_MAX;
        } else {
            state_ = MotionState::MOVING_TO_MIN;
        }
    }

    // Update target position if it changed significantly
    if (abs(target - current_pos) > 10) {
        driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver_->rampControl.SetTargetPosition(static_cast<float>(target), tmc51x0::Unit::Steps);
        driver_->rampControl.SetMaxSpeed(1000.0F);
        driver_->rampControl.SetAcceleration(2000.0F);
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
    int32_t min_bound, max_bound;
    
    {
        TmcMutexGuard guard(mutex_);
        current_state = state_;
        dwell_min = dwell_at_min_ms_;
        dwell_max = dwell_at_max_ms_;
        dwell_start = dwell_start_time_ms_;
        min_bound = local_min_bound_;
        max_bound = local_max_bound_;
    }

    switch (current_state) {
    case MotionState::DWELL_AT_MIN:
        if (current_time_ms - dwell_start >= dwell_min) {
            TmcMutexGuard guard(mutex_);
            if (!sinusoidal_mode_) {
                state_ = MotionState::MOVING_TO_MAX;
                driver_->rampControl.SetMaxSpeed(calculated_vmax_);
                driver_->rampControl.SetAcceleration(calculated_amax_);
                driver_->rampControl.SetDeceleration(calculated_amax_);
                driver_->rampControl.SetTargetPosition(static_cast<float>(max_bound), tmc51x0::Unit::Steps);
            } else {
                state_ = MotionState::MOVING_TO_MAX;
            }
        }
        break;

    case MotionState::DWELL_AT_MAX:
        if (current_time_ms - dwell_start >= dwell_max) {
            TmcMutexGuard guard(mutex_);
            if (!sinusoidal_mode_) {
                state_ = MotionState::MOVING_TO_MIN;
                driver_->rampControl.SetMaxSpeed(calculated_vmax_);
                driver_->rampControl.SetAcceleration(calculated_amax_);
                driver_->rampControl.SetDeceleration(calculated_amax_);
                driver_->rampControl.SetTargetPosition(static_cast<float>(min_bound), tmc51x0::Unit::Steps);
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
    int32_t min_bound, max_bound, global_min, global_max;
    uint16_t steps;
    {
        TmcMutexGuard guard(mutex_);
        status.running = running_ && state_ != MotionState::STOPPED;
        status.bounded = bounded_;
        status.frequency_hz = frequency_hz_;
        status.current_cycles = current_cycles_;
        status.target_cycles = target_cycles_;
        status.dwell_min_ms = dwell_at_min_ms_;
        status.dwell_max_ms = dwell_at_max_ms_;
        min_bound = local_min_bound_;
        max_bound = local_max_bound_;
        global_min = global_min_bound_;
        global_max = global_max_bound_;
        steps = steps_per_rev_;
    }
    
    if (steps > 0) {
        status.min_degrees_from_center = tmc51x0::StepsToDegrees(min_bound, steps);
        status.max_degrees_from_center = tmc51x0::StepsToDegrees(max_bound, steps);
        status.global_min_degrees = tmc51x0::StepsToDegrees(global_min, steps);
        status.global_max_degrees = tmc51x0::StepsToDegrees(global_max, steps);
    }
    
    return status;
}

float FatigueTestMotion::GetEstimatedFrequency() const noexcept {
    TmcMutexGuard guard(mutex_);
    return estimated_frequency_hz_;
}

} // namespace FatigueTest
