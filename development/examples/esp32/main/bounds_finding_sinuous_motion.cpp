/**
 * @file bounds_finding_sinuous_motion.cpp
 * @brief Fatigue testing example: Sensorless bounds finding and sinusoidal motion
 *
 * This example is designed for cable/strain relief fatigue testing:
 * 1. Finding motor bounds using sensorless homing (both directions)
 * 2. Setting global bounds (hardware limits) and local bounds (oscillation range)
 * 3. Performing pure sinusoidal back-and-forth motion between local bounds with:
 *    - Configurable angle amplitude and frequency
 *    - Target cycle count (cycles counted at center crossing)
 *    - Dwell times at bounds and optionally at center
 *    - Automatic clipping of local bounds to global bounds
 *    - Automatic stop at center when cycle count reached
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 * - Mechanical stops at both ends for bounds finding (optional - handles unbounded)
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5
 * - Control: EN=2, DIR=4, STEP=15
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <algorithm>

static const char *TAG = "FatigueTest";

/**
 * @brief Angle unit enumeration
 */
enum class AngleUnit {
  DEGREES,
  RADIANS
};

/**
 * @brief Fatigue test motion controller
 * 
 * Provides pure sinusoidal back-and-forth motion between bounds for fatigue testing.
 * Supports global bounds (hardware limits) and local bounds (oscillation range).
 */
class FatigueTestMotion {
private:
  tmc5160::TMC5160<Esp32SPI> *driver_;
  
  // Global bounds (hardware limits found during initialization)
  int32_t global_min_bound_;  // Global minimum position in steps
  int32_t global_max_bound_;  // Global maximum position in steps
  
  // Local bounds (oscillation range, clipped to global bounds)
  int32_t local_min_bound_;   // Local minimum for oscillation in steps
  int32_t local_max_bound_;   // Local maximum for oscillation in steps
  
  int32_t home_position_;      // Home position (center) in steps
  float amplitude_;            // Amplitude in steps
  float frequency_hz_;        // Frequency in Hz
  bool running_;
  uint32_t start_time_us_;
  float phase_offset_;
  bool bounded_;              // Whether global bounds were found

  // Motor configuration for unit conversions
  uint16_t steps_per_rev_;   // Steps per revolution
  AngleUnit angle_unit_;      // Preferred angle unit

  // Dwell times (can be set to 0 to disable)
  uint32_t dwell_at_min_ms_;   // Dwell time at minimum bound
  uint32_t dwell_at_max_ms_;   // Dwell time at maximum bound
  uint32_t dwell_at_center_ms_; // Dwell time at center/home (optional)

  // Cycle tracking
  uint32_t target_cycles_;     // Target number of cycles (0 = infinite)
  uint32_t current_cycles_;    // Current cycle count
  bool cycle_complete_;         // Whether target cycles reached
  bool last_was_negative_;     // Last position relative to center (for cycle counting)
  bool cycle_started_;          // Whether a cycle has started (left center)
  int32_t last_target_relative_; // Last target position relative to center (for cycle counting)

  // State machine
  enum class MotionState {
    SINUOUS_MOTION,
    DWELL_AT_MIN,
    DWELL_AT_MAX,
    DWELL_AT_CENTER,
    STOPPED
  };
  MotionState state_;
  uint32_t dwell_start_time_ms_;

public:
  FatigueTestMotion(tmc5160::TMC5160<Esp32SPI> *driver)
      : driver_(driver), global_min_bound_(0), global_max_bound_(0),
        local_min_bound_(0), local_max_bound_(0), home_position_(0),
        amplitude_(1000.0f), frequency_hz_(0.5f), running_(false),
        start_time_us_(0), phase_offset_(0.0f), bounded_(false),
        steps_per_rev_(200), angle_unit_(AngleUnit::DEGREES),
        dwell_at_min_ms_(0), dwell_at_max_ms_(0), dwell_at_center_ms_(0),
        target_cycles_(0), current_cycles_(0), cycle_complete_(false),
        last_was_negative_(false), cycle_started_(false), last_target_relative_(0),
        state_(MotionState::STOPPED), dwell_start_time_ms_(0) {}

  /**
   * @brief Configure motor parameters for unit conversions
   * @param steps_per_rev Steps per revolution (e.g., 200 for 1.8° motor)
   * @param unit Preferred angle unit (degrees or radians)
   */
  void ConfigureMotor(uint16_t steps_per_rev, AngleUnit unit = AngleUnit::DEGREES) {
    steps_per_rev_ = steps_per_rev;
    angle_unit_ = unit;
    ESP_LOGI(TAG, "Motor configured: %d steps/rev, angle unit: %s",
             steps_per_rev_, unit == AngleUnit::DEGREES ? "degrees" : "radians");
  }

  /**
   * @brief Set global bounds (hardware limits found during initialization)
   * @param min_bound Global minimum position in steps
   * @param max_bound Global maximum position in steps
   */
  void SetGlobalBounds(int32_t min_bound, int32_t max_bound) {
    global_min_bound_ = min_bound;
    global_max_bound_ = max_bound;
    bounded_ = true;
    ESP_LOGI(TAG, "Global bounds set: min=%d, max=%d steps", global_min_bound_, global_max_bound_);
    if (steps_per_rev_ > 0) {
      float min_deg = tmc5160::StepsToDegrees(global_min_bound_, steps_per_rev_);
      float max_deg = tmc5160::StepsToDegrees(global_max_bound_, steps_per_rev_);
      ESP_LOGI(TAG, "Global bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
    }
    
    // Clip local bounds to global bounds if they exist
    if (local_min_bound_ != 0 || local_max_bound_ != 0) {
      ClipLocalBoundsToGlobal();
    }
  }

  /**
   * @brief Set global bounds in degrees
   */
  void SetGlobalBoundsDegrees(float min_degrees, float max_degrees) {
    if (steps_per_rev_ == 0) {
      ESP_LOGE(TAG, "Cannot set global bounds in degrees: steps_per_rev not configured");
      return;
    }
    int32_t min_steps = tmc5160::DegreesToSteps(min_degrees, steps_per_rev_);
    int32_t max_steps = tmc5160::DegreesToSteps(max_degrees, steps_per_rev_);
    SetGlobalBounds(min_steps, max_steps);
  }

  /**
   * @brief Set global bounds in radians
   */
  void SetGlobalBoundsRadians(float min_radians, float max_radians) {
    float min_degrees = min_radians * 180.0f / M_PI;
    float max_degrees = max_radians * 180.0f / M_PI;
    SetGlobalBoundsDegrees(min_degrees, max_degrees);
  }

  /**
   * @brief Set local bounds (oscillation range, will be clipped to global bounds)
   * @param min_bound Local minimum position in steps
   * @param max_bound Local maximum position in steps
   */
  void SetLocalBounds(int32_t min_bound, int32_t max_bound) {
    local_min_bound_ = min_bound;
    local_max_bound_ = max_bound;
    home_position_ = (local_min_bound_ + local_max_bound_) / 2;
    
    // Clip to global bounds if they exist
    if (bounded_) {
      ClipLocalBoundsToGlobal();
    }
    
    ESP_LOGI(TAG, "Local bounds set: min=%d, max=%d steps", local_min_bound_, local_max_bound_);
    if (steps_per_rev_ > 0) {
      float min_deg = tmc5160::StepsToDegrees(local_min_bound_, steps_per_rev_);
      float max_deg = tmc5160::StepsToDegrees(local_max_bound_, steps_per_rev_);
      ESP_LOGI(TAG, "Local bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
    }
  }

  /**
   * @brief Set local bounds in degrees
   */
  void SetLocalBoundsDegrees(float min_degrees, float max_degrees) {
    if (steps_per_rev_ == 0) {
      ESP_LOGE(TAG, "Cannot set local bounds in degrees: steps_per_rev not configured");
      return;
    }
    int32_t min_steps = tmc5160::DegreesToSteps(min_degrees, steps_per_rev_);
    int32_t max_steps = tmc5160::DegreesToSteps(max_degrees, steps_per_rev_);
    SetLocalBounds(min_steps, max_steps);
  }

  /**
   * @brief Set local bounds in radians
   */
  void SetLocalBoundsRadians(float min_radians, float max_radians) {
    float min_degrees = min_radians * 180.0f / M_PI;
    float max_degrees = max_radians * 180.0f / M_PI;
    SetLocalBoundsDegrees(min_degrees, max_degrees);
  }

  /**
   * @brief Set unbounded mode (no mechanical stops found)
   * Uses current position as home and sets reasonable default global bounds
   */
  void SetUnbounded(int32_t current_position, int32_t default_range_steps = 10000) {
    bounded_ = false;
    home_position_ = current_position;
    global_min_bound_ = current_position - default_range_steps / 2;
    global_max_bound_ = current_position + default_range_steps / 2;
    driver_->rampControl.SetCurrentPosition(0);
    ESP_LOGW(TAG, "Unbounded mode: No mechanical stops found");
    ESP_LOGI(TAG, "Using current position as home: %d steps", current_position);
    ESP_LOGI(TAG, "Default global range: [%d, %d] steps", global_min_bound_, global_max_bound_);
    if (steps_per_rev_ > 0) {
      float range_deg = tmc5160::StepsToDegrees(default_range_steps, steps_per_rev_);
      ESP_LOGI(TAG, "Default global range: %.2f°", range_deg);
    }
  }

  /**
   * @brief Reset home position by relative angle
   * @param relative_angle Angle offset from current home (in configured unit)
   */
  void ResetHomeByAngle(float relative_angle) {
    int32_t offset_steps = 0;
    if (angle_unit_ == AngleUnit::DEGREES) {
      offset_steps = tmc5160::DegreesToSteps(relative_angle, steps_per_rev_);
    } else {
      float degrees = relative_angle * 180.0f / M_PI;
      offset_steps = tmc5160::DegreesToSteps(degrees, steps_per_rev_);
    }

    int32_t current_pos = driver_->rampControl.GetCurrentPosition();
    int32_t new_home_pos = current_pos + offset_steps;

    // Move to new home position
    driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver_->rampControl.SetTargetPosition(new_home_pos);
    driver_->rampControl.SetMaxSpeed(1000.0f);
    driver_->rampControl.SetAcceleration(2000.0f);

    while (!driver_->rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Set new home position
    driver_->rampControl.SetCurrentPosition(0);
    home_position_ = 0;

    // Recalculate bounds relative to new home
    global_min_bound_ -= new_home_pos;
    global_max_bound_ -= new_home_pos;
    local_min_bound_ -= new_home_pos;
    local_max_bound_ -= new_home_pos;

    // Clip local bounds to global bounds
    if (bounded_) {
      ClipLocalBoundsToGlobal();
    }

    ESP_LOGI(TAG, "Home reset by %.2f %s", relative_angle,
             angle_unit_ == AngleUnit::DEGREES ? "degrees" : "radians");
  }

  /**
   * @brief Reset home position by relative degrees
   */
  void ResetHomeByDegrees(float relative_degrees) {
    ResetHomeByAngle(relative_degrees);
  }

  /**
   * @brief Reset home position by relative radians
   */
  void ResetHomeByRadians(float relative_radians) {
    float degrees = relative_radians * 180.0f / M_PI;
    ResetHomeByAngle(degrees);
  }

  /**
   * @brief Set sinuous motion parameters (can be changed in real-time)
   * @param amplitude_steps Amplitude in steps
   * @param frequency_hz Frequency in Hz
   */
  void SetSinuousParams(float amplitude_steps, float frequency_hz) {
    amplitude_ = amplitude_steps;
    frequency_hz_ = frequency_hz;
    ESP_LOGI(TAG, "Sinuous params updated: amplitude=%.1f steps, frequency=%.2f Hz",
             amplitude_, frequency_hz_);
    if (steps_per_rev_ > 0) {
      float amp_deg = tmc5160::StepsToDegrees(static_cast<int32_t>(amplitude_), steps_per_rev_);
      ESP_LOGI(TAG, "  Amplitude: %.2f°", amp_deg);
    }
  }

  /**
   * @brief Set sinuous motion amplitude in degrees (can be changed in real-time)
   */
  void SetSinuousAmplitudeDegrees(float amplitude_degrees) {
    if (steps_per_rev_ == 0) {
      ESP_LOGE(TAG, "Cannot set amplitude in degrees: steps_per_rev not configured");
      return;
    }
    amplitude_ = static_cast<float>(tmc5160::DegreesToSteps(amplitude_degrees, steps_per_rev_));
    ESP_LOGI(TAG, "Sinuous amplitude updated: %.2f° (%.1f steps)", amplitude_degrees, amplitude_);
  }

  /**
   * @brief Set sinuous motion amplitude in radians (can be changed in real-time)
   */
  void SetSinuousAmplitudeRadians(float amplitude_radians) {
    float degrees = amplitude_radians * 180.0f / M_PI;
    SetSinuousAmplitudeDegrees(degrees);
  }

  /**
   * @brief Set frequency (can be changed in real-time)
   */
  void SetFrequency(float frequency_hz) {
    frequency_hz_ = frequency_hz;
    ESP_LOGI(TAG, "Frequency updated: %.2f Hz", frequency_hz_);
  }

  /**
   * @brief Set dwell times (can be changed in real-time, 0 to disable)
   * @param dwell_at_min_ms Dwell time at minimum bound (ms)
   * @param dwell_at_max_ms Dwell time at maximum bound (ms)
   * @param dwell_at_center_ms Dwell time at center (ms, optional, 0 to disable)
   */
  void SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms,
                      uint32_t dwell_at_center_ms = 0) {
    dwell_at_min_ms_ = dwell_at_min_ms;
    dwell_at_max_ms_ = dwell_at_max_ms;
    dwell_at_center_ms_ = dwell_at_center_ms;
    ESP_LOGI(TAG, "Dwell times updated: min=%lu ms, max=%lu ms, center=%lu ms",
             dwell_at_min_ms_, dwell_at_max_ms_, dwell_at_center_ms_);
  }

  /**
   * @brief Set target cycle count (0 = infinite)
   * @param cycles Target number of cycles (0 for infinite)
   */
  void SetTargetCycles(uint32_t cycles) {
    target_cycles_ = cycles;
    ESP_LOGI(TAG, "Target cycles set: %lu (0 = infinite)", target_cycles_);
  }

  /**
   * @brief Get current cycle count
   */
  uint32_t GetCurrentCycles() const { return current_cycles_; }

  /**
   * @brief Get target cycle count
   */
  uint32_t GetTargetCycles() const { return target_cycles_; }

  /**
   * @brief Check if cycle count reached
   */
  bool IsCycleComplete() const { return cycle_complete_; }

  /**
   * @brief Reset cycle count
   */
  void ResetCycles() {
    current_cycles_ = 0;
    cycle_complete_ = false;
    last_was_negative_ = false;
    cycle_started_ = false;
    last_target_relative_ = 0;
    ESP_LOGI(TAG, "Cycle count reset");
  }

  /**
   * @brief Get local bounds in degrees
   */
  void GetLocalBoundsDegrees(float &min_degrees, float &max_degrees) const {
    if (steps_per_rev_ == 0) {
      min_degrees = 0.0f;
      max_degrees = 0.0f;
      return;
    }
    min_degrees = tmc5160::StepsToDegrees(local_min_bound_, steps_per_rev_);
    max_degrees = tmc5160::StepsToDegrees(local_max_bound_, steps_per_rev_);
  }

  /**
   * @brief Get local bounds in radians
   */
  void GetLocalBoundsRadians(float &min_radians, float &max_radians) const {
    float min_deg, max_deg;
    GetLocalBoundsDegrees(min_deg, max_deg);
    min_radians = min_deg * M_PI / 180.0f;
    max_radians = max_deg * M_PI / 180.0f;
  }

  /**
   * @brief Get global bounds in degrees
   */
  void GetGlobalBoundsDegrees(float &min_degrees, float &max_degrees) const {
    if (steps_per_rev_ == 0) {
      min_degrees = 0.0f;
      max_degrees = 0.0f;
      return;
    }
    min_degrees = tmc5160::StepsToDegrees(global_min_bound_, steps_per_rev_);
    max_degrees = tmc5160::StepsToDegrees(global_max_bound_, steps_per_rev_);
  }

  /**
   * @brief Check if system is bounded
   */
  bool IsBounded() const { return bounded_; }

  /**
   * @brief Start sinuous motion (can be called at any time)
   */
  void Start() {
    if (local_min_bound_ == 0 && local_max_bound_ == 0) {
      ESP_LOGE(TAG, "Cannot start: local bounds not set!");
      return;
    }

    if (cycle_complete_) {
      ESP_LOGW(TAG, "Cycle count reached. Reset cycles or set new target to continue.");
      return;
    }

    running_ = true;
    state_ = MotionState::SINUOUS_MOTION;
    start_time_us_ = esp_timer_get_time();
    
    // If resuming from stop, calculate phase offset from current position
    int32_t current_pos = driver_->rampControl.GetCurrentPosition();
    int32_t pos_relative = current_pos - home_position_;
    if (amplitude_ > 0) {
      double normalized_pos = static_cast<double>(pos_relative) / amplitude_;
      if (normalized_pos > 1.0) normalized_pos = 1.0;
      if (normalized_pos < -1.0) normalized_pos = -1.0;
      phase_offset_ = asin(normalized_pos);
      // Initialize cycle tracking based on current position
      last_was_negative_ = (pos_relative < 0);
      cycle_started_ = (abs(pos_relative) > 10); // Started if away from center
      last_target_relative_ = pos_relative;
    } else {
      phase_offset_ = 0.0f;
      last_was_negative_ = false;
      cycle_started_ = false;
    }

    ESP_LOGI(TAG, "Starting fatigue test motion (cycles: %lu/%lu)", 
             current_cycles_, target_cycles_ == 0 ? 0xFFFFFFFF : target_cycles_);
  }

  /**
   * @brief Stop sinuous motion (can be called at any time)
   */
  void Stop() {
    running_ = false;
    state_ = MotionState::STOPPED;
    driver_->rampControl.Stop();
    ESP_LOGI(TAG, "Stopped fatigue test motion (cycles completed: %lu)", current_cycles_);
  }

  /**
   * @brief Check if motion is running
   */
  bool IsRunning() const { return running_ && state_ != MotionState::STOPPED; }

  /**
   * @brief Update motion (call this in main loop)
   */
  void Update() {
    if (!running_ || state_ == MotionState::STOPPED) {
      return;
    }

    // Check if cycle count reached - if so, move to center and stop
    if (target_cycles_ > 0 && current_cycles_ >= target_cycles_) {
      if (!cycle_complete_) {
        cycle_complete_ = true;
        ESP_LOGI(TAG, "Target cycle count reached: %lu cycles", current_cycles_);
        // Move to center before stopping
        driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
        driver_->rampControl.SetTargetPosition(home_position_);
        driver_->rampControl.SetMaxSpeed(1000.0f);
        driver_->rampControl.SetAcceleration(2000.0f);
        state_ = MotionState::STOPPED;
        // Wait for center to be reached
        while (!driver_->rampControl.IsTargetReached()) {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        running_ = false;
        ESP_LOGI(TAG, "Motion stopped at center position");
      }
      return;
    }

    uint32_t current_time_ms = esp_timer_get_time() / 1000;

    switch (state_) {
    case MotionState::DWELL_AT_MIN:
      if (current_time_ms - dwell_start_time_ms_ >= dwell_at_min_ms_) {
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        phase_offset_ = -M_PI / 2.0; // Start from minimum position
      }
      break;

    case MotionState::DWELL_AT_MAX:
      if (current_time_ms - dwell_start_time_ms_ >= dwell_at_max_ms_) {
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        phase_offset_ = M_PI / 2.0; // Start from maximum position
      }
      break;

    case MotionState::DWELL_AT_CENTER:
      if (current_time_ms - dwell_start_time_ms_ >= dwell_at_center_ms_) {
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        // Continue from center (phase = 0 or π)
        int32_t current_pos = driver_->rampControl.GetCurrentPosition();
        int32_t pos_relative = current_pos - home_position_;
        if (abs(pos_relative) < 10) {
          // At center, determine direction from last position
          phase_offset_ = last_was_negative_ ? M_PI : 0.0f;
        } else {
          // Near center, calculate phase
          double normalized_pos = static_cast<double>(pos_relative) / amplitude_;
          if (normalized_pos > 1.0) normalized_pos = 1.0;
          if (normalized_pos < -1.0) normalized_pos = -1.0;
          phase_offset_ = asin(normalized_pos);
        }
      }
      break;

    case MotionState::SINUOUS_MOTION:
      UpdateSinuousMotion();
      break;

    case MotionState::STOPPED:
      break;
    }
  }

private:
  /**
   * @brief Clip local bounds to global bounds
   */
  void ClipLocalBoundsToGlobal() {
    if (!bounded_) return;

    int32_t old_min = local_min_bound_;
    int32_t old_max = local_max_bound_;

    // Clip local bounds to global bounds
    local_min_bound_ = std::max(local_min_bound_, global_min_bound_);
    local_max_bound_ = std::min(local_max_bound_, global_max_bound_);

    // Update home position
    home_position_ = (local_min_bound_ + local_max_bound_) / 2;

    if (old_min != local_min_bound_ || old_max != local_max_bound_) {
      ESP_LOGW(TAG, "Local bounds clipped to global bounds");
      ESP_LOGI(TAG, "Clipped local bounds: min=%d, max=%d steps", local_min_bound_, local_max_bound_);
      if (steps_per_rev_ > 0) {
        float min_deg = tmc5160::StepsToDegrees(local_min_bound_, steps_per_rev_);
        float max_deg = tmc5160::StepsToDegrees(local_max_bound_, steps_per_rev_);
        ESP_LOGI(TAG, "Clipped local bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
      }
    }
  }

  /**
   * @brief Update sinuous motion target position
   */
  void UpdateSinuousMotion() {
    uint64_t elapsed_us = esp_timer_get_time() - start_time_us_;
    double elapsed_s = elapsed_us / 1000000.0;

    // Calculate sinusoidal position
    // Position oscillates around home_position_ with amplitude_
    double angle = 2.0 * M_PI * frequency_hz_ * elapsed_s + phase_offset_;
    double sin_value = sin(angle);

    // Calculate target position
    int32_t target = home_position_ + static_cast<int32_t>(amplitude_ * sin_value);

    // Get current position relative to center for cycle counting
    int32_t current_pos = driver_->rampControl.GetCurrentPosition();
    int32_t target_relative = target - home_position_;
    
    // Cycle counting: one cycle = center → min → max → center (or center → max → min → center)
    // Count cycles when crossing through center (0 crossing point)
    if (state_ == MotionState::SINUOUS_MOTION) {
      // Check if we're crossing through center (sign change of target position)
      bool currently_negative = (target_relative < 0);
      bool last_was_negative = (last_target_relative_ < 0);
      bool crossing_center = (last_was_negative != currently_negative) && 
                              (abs(target_relative) < 30) && 
                              (abs(last_target_relative_) < 30);
      
      // If we've started a cycle (left center) and now crossing back through center
      if (cycle_started_ && crossing_center) {
        // Completed a cycle: center → extreme → center
        current_cycles_++;
        cycle_started_ = false; // Reset for next cycle
        ESP_LOGI(TAG, "Cycle %lu completed at center (target: %lu)", 
                 current_cycles_, target_cycles_);
        
        // Check if target cycles reached
        if (target_cycles_ > 0 && current_cycles_ >= target_cycles_) {
          cycle_complete_ = true;
          // Will be handled in Update() to stop at center
        }
      } else if (!cycle_started_ && abs(target_relative) > 30) {
        // We've left center, cycle has started
        cycle_started_ = true;
        last_was_negative_ = currently_negative;
      }
      
      // Update tracking
      last_target_relative_ = target_relative;
      if (abs(target_relative) > 10) { // Only update if significantly away from center
        last_was_negative_ = currently_negative;
      }
    }

    // Clamp to local bounds and handle dwell states
    if (target <= local_min_bound_) {
      target = local_min_bound_;
      if (state_ == MotionState::SINUOUS_MOTION && dwell_at_min_ms_ > 0) {
        state_ = MotionState::DWELL_AT_MIN;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(target);
        return;
      }
    } else if (target >= local_max_bound_) {
      target = local_max_bound_;
      if (state_ == MotionState::SINUOUS_MOTION && dwell_at_max_ms_ > 0) {
        state_ = MotionState::DWELL_AT_MAX;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(target);
        return;
      }
    }

    // Check if we're passing through center and need to dwell
    if (dwell_at_center_ms_ > 0 && state_ == MotionState::SINUOUS_MOTION) {
      if (abs(target_relative) < 20) { // Within 20 steps of center
        state_ = MotionState::DWELL_AT_CENTER;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(home_position_);
        return;
      }
    }

    // Update target position if it changed significantly
    // Reuse current_pos from above (line 607)
    if (abs(target - current_pos) > 10) { // Update threshold: 10 steps
      driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(target);
      driver_->rampControl.SetMaxSpeed(1000.0f);
      driver_->rampControl.SetAcceleration(2000.0f);
    }
  }
};

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Fatigue Test: Bounds Finding and Sinuous Motion");

  // Create SPI communication interface
  Esp32SPI spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
               GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15, 4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC5160 driver instance
  tmc5160::TMC5160 driver(spi);

  // Configure driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;
  cfg.motor.global_scaler = 32;
  cfg.chopper.mres = 5; // 32 microsteps for smooth motion
  cfg.chopper.intpol = true;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Configure StallGuard2 for bounds finding
  tmc5160::StallGuardConfig sg_config{};
  sg_config.sgt = -10;    // Stall threshold (tune for your motor)
  sg_config.sfilt = true; // Enable filter for stability
  sg_config.semin = 2;
  sg_config.semax = 5;

  ESP_LOGI(TAG, "Configuring StallGuard2: sgt=%d", sg_config.sgt);
  if (!driver.diagnostics.ConfigureStallGuard(sg_config)) {
    ESP_LOGE(TAG, "Failed to configure StallGuard2");
    return;
  }

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled");

  // Configure motor parameters for unit conversions
  // IMPORTANT: Set this based on your motor specifications
  uint16_t steps_per_rev = 200; // Example: 200 steps/rev for 1.8° motor
  // Adjust based on microsteps: if using 32 microsteps, steps_per_rev = 200 * 32 = 6400
  steps_per_rev = 200 * 32; // 6400 steps per revolution with 32 microsteps

  // ============================================================
  // STEP 1: Find global bounds using sensorless homing
  // ============================================================
  ESP_LOGI(TAG, "=== Step 1: Finding global bounds ===");

  // Find minimum bound (negative direction)
  ESP_LOGI(TAG, "Finding minimum bound (negative direction)...");
  int32_t min_position = 0;
  float search_speed = 500.0f; // steps/s

  driver.rampControl.SetCurrentPosition(0);

  if (!driver.diagnostics.PerformSensorlessHoming(false, -10, search_speed, min_position)) {
    ESP_LOGW(TAG, "Failed to find minimum bound, using default");
  }

  uint32_t timeout_ms = 30000;
  uint32_t start_time = esp_timer_get_time() / 1000;
  bool stall_detected_min = false;
  int32_t initial_pos = driver.rampControl.GetCurrentPosition();

  while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
    uint16_t sg_value = driver.diagnostics.GetStallGuard();
    int32_t current_pos = driver.rampControl.GetCurrentPosition();

    if (driver.rampControl.IsTargetReached() || sg_value < 100 ||
        abs(current_pos - initial_pos) > 1000) {
      min_position = current_pos;
      stall_detected_min = true;
      ESP_LOGI(TAG, "Stall detected! Minimum position: %d steps", min_position);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!stall_detected_min) {
    ESP_LOGW(TAG, "No stall detected in negative direction - may be unbounded");
    min_position = driver.rampControl.GetCurrentPosition();
  } else {
    driver.rampControl.Stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Moving 100 steps away from minimum stop...");
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(min_position + 100);
    driver.rampControl.SetMaxSpeed(500.0f);
    driver.rampControl.SetAcceleration(1000.0f);
    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    min_position = driver.rampControl.GetCurrentPosition();
    driver.rampControl.SetCurrentPosition(0);
  }

  // Find maximum bound (positive direction)
  ESP_LOGI(TAG, "Finding maximum bound (positive direction)...");
  int32_t max_position = 0;
  if (!driver.diagnostics.PerformSensorlessHoming(true, -10, search_speed, max_position)) {
    ESP_LOGW(TAG, "Failed to find maximum bound, using default");
  }

  start_time = esp_timer_get_time() / 1000;
  bool stall_detected_max = false;
  initial_pos = driver.rampControl.GetCurrentPosition();

  while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
    uint16_t sg_value = driver.diagnostics.GetStallGuard();
    int32_t current_pos = driver.rampControl.GetCurrentPosition();

    if (driver.rampControl.IsTargetReached() || sg_value < 100 ||
        abs(current_pos - initial_pos) > 1000) {
      max_position = current_pos;
      stall_detected_max = true;
      ESP_LOGI(TAG, "Stall detected! Maximum position: %d steps", max_position);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!stall_detected_max) {
    ESP_LOGW(TAG, "No stall detected in positive direction - may be unbounded");
    max_position = driver.rampControl.GetCurrentPosition();
  } else {
    driver.rampControl.Stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Moving 100 steps away from maximum stop...");
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(max_position - 100);
    driver.rampControl.SetMaxSpeed(500.0f);
    driver.rampControl.SetAcceleration(1000.0f);
    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    max_position = driver.rampControl.GetCurrentPosition();
  }

  // ============================================================
  // STEP 2: Set up global bounds and home
  // ============================================================
  ESP_LOGI(TAG, "=== Step 2: Setting global bounds and home ===");

  bool bounded = stall_detected_min && stall_detected_max;
  int32_t current_pos = driver.rampControl.GetCurrentPosition();

  FatigueTestMotion motion(&driver);
  motion.ConfigureMotor(steps_per_rev, AngleUnit::DEGREES);

  if (!bounded) {
    ESP_LOGW(TAG, "=== UNBOUNDED MODE ===");
    motion.SetUnbounded(current_pos, 10000);
  } else {
    ESP_LOGI(TAG, "=== BOUNDED MODE ===");
    
    // Set middle as home
    int32_t middle_position = (min_position + max_position) / 2;
    ESP_LOGI(TAG, "Moving to middle position: %d steps", middle_position);

    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(middle_position);
    driver.rampControl.SetMaxSpeed(1000.0f);
    driver.rampControl.SetAcceleration(2000.0f);

    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    driver.rampControl.SetCurrentPosition(0);
    ESP_LOGI(TAG, "Home position set to 0 (middle of bounds)");

    // Set global bounds relative to new home
    int32_t global_min = min_position - middle_position;
    int32_t global_max = max_position - middle_position;
    motion.SetGlobalBounds(global_min, global_max);

    float min_deg, max_deg;
    motion.GetGlobalBoundsDegrees(min_deg, max_deg);
    ESP_LOGI(TAG, "Global bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
  }

  // ============================================================
  // STEP 3: Set local bounds (oscillation range) and start motion
  // ============================================================
  ESP_LOGI(TAG, "=== Step 3: Configuring fatigue test ===");

  // Set local bounds for oscillation (will be clipped to global bounds if needed)
  // Example: ±60 degrees for fatigue testing
  motion.SetLocalBoundsDegrees(-60.0f, 60.0f);

  // Configure sinuous motion
  motion.SetSinuousAmplitudeDegrees(60.0f);  // 60° amplitude
  motion.SetSinuousParams(0, 0.5f);  // 0.5 Hz frequency

  // Set dwell times (can be 0 to disable)
  // For fatigue testing: dwell at extremes simulates holding tool in awkward positions
  motion.SetDwellTimes(2000,  // 2 seconds at minimum bound
                        2000,  // 2 seconds at maximum bound
                        0);    // No dwell at center (set to >0 to enable)

  // Set target cycle count (0 = infinite)
  motion.SetTargetCycles(1000);  // Run for 1000 cycles, then stop

  ESP_LOGI(TAG, "Fatigue test configured:");
  float min_deg = 0.0f;
  float max_deg = 0.0f;
  motion.GetLocalBoundsDegrees(min_deg, max_deg);
  ESP_LOGI(TAG, "  Local bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
  ESP_LOGI(TAG, "  Frequency: 0.5 Hz");
  ESP_LOGI(TAG, "  Target cycles: %lu", motion.GetTargetCycles());
  ESP_LOGI(TAG, "  Bounded: %s", motion.IsBounded() ? "Yes" : "No");

  // Start motion
  motion.Start();

  // Example: Settings can be changed in real-time
  // motion.SetFrequency(1.0f);  // Change frequency to 1.0 Hz
  // motion.SetSinuousAmplitudeDegrees(45.0f);  // Change amplitude to 45°
  // motion.SetDwellTimes(1000, 1000, 0);  // Change dwell times
  // motion.SetTargetCycles(2000);  // Change target cycles

  // Main loop - update motion controller
  ESP_LOGI(TAG, "Running fatigue test (press reset to stop)...");
  uint32_t last_log_time = 0;

  while (true) {
    motion.Update();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Log cycle count periodically
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_log_time > 10000) { // Log every 10 seconds
      int32_t pos = driver.rampControl.GetCurrentPosition();
      float speed = driver.rampControl.GetCurrentSpeed();
      float pos_deg = tmc5160::StepsToDegrees(pos, steps_per_rev);
      uint32_t cycles = motion.GetCurrentCycles();
      uint32_t target = motion.GetTargetCycles();
      ESP_LOGI(TAG, "Position: %d steps (%.2f°), Speed: %.1f steps/s, Cycles: %lu/%lu %s",
               pos, pos_deg, speed, cycles, target == 0 ? 0xFFFFFFFF : target,
               motion.IsRunning() ? "(running)" : "(stopped)");
      last_log_time = current_time;
    }

    // Example: Stop and restart motion
    // uint32_t current_cycles = motion.GetCurrentCycles();
    // if (current_cycles >= 500 && current_cycles < 501) {
    //   motion.Stop();
    //   vTaskDelay(pdMS_TO_TICKS(2000));
    //   motion.Start();  // Resume motion
    // }

    // Example: Change settings in real-time
    // uint32_t current_cycles = motion.GetCurrentCycles();
    // if (current_cycles == 100) {
    //   motion.SetFrequency(1.0f);  // Increase frequency
    //   motion.SetSinuousAmplitudeDegrees(45.0f);  // Reduce amplitude
    // }
  }
}
