/**
 * @file fatigue_test_stallguard.cpp
 * @brief Fatigue testing platform: Back-and-forth motion between bounds using StallGuard2 sensorless detection
 *
 * This example is designed for cable/strain relief fatigue testing platforms:
 * 1. Finding motor bounds using sensorless homing (both directions)
 * 2. Setting global bounds (hardware limits) and local bounds (oscillation range)
 * 3. Performing pure sinusoidal back-and-forth motion between local bounds with:
 *    - Configurable angle amplitude and frequency
 *    - Target cycle count (cycles counted at center crossing)
 *    - Dwell times at bounds
 *    - Automatic clipping of local bounds to global bounds
 *    - Automatic stop at center when cycle count reached
 * 4. UART command interface for real-time parameter adjustment
 *
 * MOTOR SELECTION:
 * Motor selection is done via a static constexpr variable at the top of this file.
 * See esp32_tmc51x0_test_config.hpp for detailed motor specifications and selection guide.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC51x0 stepper motor driver
 * - Stepper motor connected to TMC51x0 (see motor selection above)
 * - SPI connection between ESP32 and TMC51x0
 * - Mechanical stops at both ends for bounds finding (optional - handles unbounded)
 * - UART debug port for command interface (typically UART_NUM_0 for USB serial)
 * - Power supply: 12-36V DC (ensure adequate current capacity for selected motor)
 *
 * Pin Configuration (uses default dev board pins from esp32_tmc51x0_test_config.hpp):
 * - SPI: MOSI=6, MISO=2, SCLK=5, CS=18
 * - Control: EN=11, CLK=10, DIAG0=23, DIAG1=15
 * - UART: Uses default UART_NUM_0 (USB serial port)
 *
 * UART Command Interface:
 * Commands follow Linux-like argument structure:
 *   -f <value> or --freq <value>     : Set frequency in Hz
 *   -d <min> <max>                   : Set dwell times in ms (min, max)
 *   -b <min> <max> or --bounds <min> <max> : Set angle bounds from center in degrees
 *   -c <count> or --cycles <count>   : Set target cycle count (0 = infinite)
 *   -a start|stop|reset              : Action commands (start motion, stop, reset cycles)
 *   -s or --status                   : Show current status
 *   -h or --help                     : Show help message
 *
 * Examples:
 *   -f 0.5                           : Set frequency to 0.5 Hz
 *   -d 2000 2000                     : Set dwell times: 2s at min, 2s at max
 *   -b -60 60                        : Set bounds to ±60 degrees from center
 *   -c 1000                          : Set target to 1000 cycles
 *   -a start                         : Start motion
 *   -a stop                          : Stop motion
 *   -a reset                         : Reset cycle count
 *   -s                                : Show status
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"

#include "test_config/esp32_tmc51x0_test_config.hpp"

//=============================================================================
// CONFIGURATION SELECTION - Unified Test Rig Selection
//=============================================================================
// See esp32_tmc51x0_test_config.hpp for detailed test rig specifications.
// 
// TEST RIG CONFIGURATION:
// This example is configured for the FATIGUE TEST RIG which automatically selects:
// - Motor: Applied Motion 5034-369 (direct drive, NEMA 34)
// - Board: TMC51x0 Evaluation Kit
// - Platform: Fatigue Test Rig (reference switches, encoder)
//
// For the CORE DRIVER TEST RIG, change to TEST_RIG_CORE_DRIVER

// Test rig selection (compile-time constant) - automatically selects motor, board, and platform
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
    
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

static const char* TAG = "FatigueTest";

//=============================================================================
// RAII Mutex Classes
//=============================================================================

/**
 * @brief ESP32 TMC mutex wrapper using FreeRTOS semaphore
 * 
 * RAII wrapper for FreeRTOS mutex that automatically creates and destroys
 * the mutex handle. Non-copyable but movable.
 */
class Esp32TmcMutex {
public:
  /**
   * @brief Create a new mutex
   */
  Esp32TmcMutex() noexcept : handle_(xSemaphoreCreateMutex()) {
    if (handle_ == nullptr) {
      ESP_LOGE(TAG, "Failed to create mutex");
    }
  }

  /**
   * @brief Destroy mutex (automatically deletes semaphore)
   */
  ~Esp32TmcMutex() noexcept {
    if (handle_ != nullptr) {
      vSemaphoreDelete(handle_);
      handle_ = nullptr;
    }
  }

  // Non-copyable
  Esp32TmcMutex(const Esp32TmcMutex&) = delete;
  Esp32TmcMutex& operator=(const Esp32TmcMutex&) = delete;

  // Movable
  Esp32TmcMutex(Esp32TmcMutex&& other) noexcept : handle_(other.handle_) {
    other.handle_ = nullptr;
  }

  Esp32TmcMutex& operator=(Esp32TmcMutex&& other) noexcept {
    if (this != &other) {
      if (handle_ != nullptr) {
        vSemaphoreDelete(handle_);
      }
      handle_ = other.handle_;
      other.handle_ = nullptr;
    }
    return *this;
  }

  /**
   * @brief Get native mutex handle
   */
  SemaphoreHandle_t native_handle() const noexcept {
    return handle_;
  }

  /**
   * @brief Check if mutex is valid
   */
  bool is_valid() const noexcept {
    return handle_ != nullptr;
  }

private:
  SemaphoreHandle_t handle_;
};

/**
 * @brief RAII mutex guard for automatic lock/unlock
 * 
 * Automatically locks mutex on construction and unlocks on destruction.
 * Provides exception-safe mutex locking following RAII principles.
 */
class TmcMutexGuard {
public:
  /**
   * @brief Lock mutex (blocks until acquired)
   * @param mutex Reference to mutex to lock
   */
  explicit TmcMutexGuard(Esp32TmcMutex& mutex) noexcept : mutex_(mutex), locked_(false) {
    if (mutex_.is_valid()) {
      SemaphoreHandle_t handle = mutex_.native_handle();
      if (xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE) {
        locked_ = true;
      }
    }
  }

  /**
   * @brief Unlock mutex (automatically called on destruction)
   */
  ~TmcMutexGuard() noexcept {
    unlock();
  }

  // Non-copyable
  TmcMutexGuard(const TmcMutexGuard&) = delete;
  TmcMutexGuard& operator=(const TmcMutexGuard&) = delete;

  // Non-movable (to prevent accidental moves that would break RAII)
  TmcMutexGuard(TmcMutexGuard&&) = delete;
  TmcMutexGuard& operator=(TmcMutexGuard&&) = delete;

  /**
   * @brief Manually unlock mutex (called automatically in destructor)
   */
  void unlock() noexcept {
    if (locked_ && mutex_.is_valid()) {
      SemaphoreHandle_t handle = mutex_.native_handle();
      xSemaphoreGive(handle);
      locked_ = false;
    }
  }

  /**
   * @brief Check if mutex is currently locked by this guard
   */
  bool is_locked() const noexcept {
    return locked_;
  }

private:
  Esp32TmcMutex& mutex_;
  bool locked_;
};

/**
 * @brief Angle unit enumeration
 */
enum class AngleUnit { DEGREES, RADIANS };

/**
 * @brief Fatigue test motion controller
 *
 * Provides pure sinusoidal back-and-forth motion between bounds for fatigue testing.
 * Supports global bounds (hardware limits) and local bounds (oscillation range).
 */
class FatigueTestMotion {
private:
  tmc51x0::TMC51x0<Esp32SPI>* driver_;

  // Global bounds (hardware limits found during initialization)
  int32_t global_min_bound_; // Global minimum position in steps
  int32_t global_max_bound_; // Global maximum position in steps

  // Local bounds (oscillation range, clipped to global bounds)
  int32_t local_min_bound_; // Local minimum for oscillation in steps
  int32_t local_max_bound_; // Local maximum for oscillation in steps

  int32_t home_position_; // Home position (center) in steps
  float amplitude_;       // Amplitude in steps
  float frequency_hz_;    // Frequency in Hz
  bool running_;
  uint32_t start_time_us_;
  float phase_offset_;
  bool bounded_; // Whether global bounds were found

  // Motor configuration for unit conversions
  uint16_t steps_per_rev_; // Steps per revolution
  AngleUnit angle_unit_;   // Preferred angle unit

  // Dwell times (can be set to 0 to disable)
  uint32_t dwell_at_min_ms_;    // Dwell time at minimum bound
  uint32_t dwell_at_max_ms_;    // Dwell time at maximum bound

  // Cycle tracking
  uint32_t target_cycles_;       // Target number of cycles (0 = infinite)
  uint32_t current_cycles_;      // Current cycle count
  bool cycle_complete_;          // Whether target cycles reached
  bool last_was_negative_;       // Last position relative to center (for cycle counting)
  bool cycle_started_;           // Whether a cycle has started (left center)
  int32_t last_target_relative_; // Last target position relative to center (for cycle counting)

  // State machine
  // Note: Sinusoidal motion uses the same states - it's a motion mode, not a separate state
  enum class MotionState { MOVING_TO_MIN, MOVING_TO_MAX, DWELL_AT_MIN, DWELL_AT_MAX, STOPPED };
  MotionState state_;
  uint32_t dwell_start_time_ms_;
  
  // Motion mode flag (true = sinusoidal, false = ramp-based)
  bool sinusoidal_mode_;

  // Computed trajectory parameters
  float calculated_vmax_;
  float calculated_amax_;
  float estimated_frequency_hz_;

  // Thread safety
  mutable Esp32TmcMutex mutex_;

  /**
   * @brief Recalculate trajectory parameters based on frequency, bounds, and dwell
   */
  void RecalculateTrajectory() noexcept {
    // NOTE: Must be called with mutex locked
    
    // Calculate total travel distance (one way)
    int32_t distance = abs(local_max_bound_ - local_min_bound_);
    if (distance == 0 || frequency_hz_ <= 0.0001f) {
      calculated_vmax_ = 1000.0f; // Default safe fallback
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
      // Frequency is too high for the requested dwell times
      total_move_time_s = 0.1f; // Minimal fallback
      ESP_LOGW(TAG, "Requested frequency %.2f Hz is impossible with given dwell times!", frequency_hz_);
    }
    
    // Time for one leg (one way)
    float leg_time_s = total_move_time_s / 2.0f;
    
    // Calculate VMAX and AMAX for Trapezoidal Profile (no center dwell)
    // We want to reach the target distance D in time T (leg_time_s).
    // Profile: Accel -> Constant Vel -> Decel.
    // Let's assume 1/3 Accel, 1/3 Const, 1/3 Decel for smooth motion.
    // Vmax = 1.5 * Distance / Time.
    // Accel = Vmax / (Time/3) = 4.5 * Distance / Time^2.
    
    calculated_vmax_ = (1.5f * distance) / leg_time_s;
    calculated_amax_ = calculated_vmax_ / (leg_time_s / 3.0f);
    
    // Clamp to driver limits (approximate)
    if (calculated_vmax_ > 5000000.0f) calculated_vmax_ = 5000000.0f; // Cap velocity (higher limit)
    if (calculated_amax_ > 5000000.0f) calculated_amax_ = 5000000.0f; // Cap acceleration
    
    // Recalculate actual expected frequency based on calculated parameters
    // T_leg_actual = Distance/Vavg. For 1/3-1/3-1/3 profile, Vavg = 2/3 Vmax.
    // So T_leg = D / (2/3 * 1.5 * D/T) = T. It should match.
    estimated_frequency_hz_ = 1.0f / (2.0f * leg_time_s + total_dwell_s);
    
    ESP_LOGI(TAG, "Trajectory Recalculated: Dist=%ld steps, LegTime=%.3fs", distance, leg_time_s);
    ESP_LOGI(TAG, "  Target Freq=%.2fHz, Est Freq=%.2fHz", frequency_hz_, estimated_frequency_hz_);
    ESP_LOGI(TAG, "  VMAX=%.1f, AMAX=%.1f", calculated_vmax_, calculated_amax_);
  }

public:
  FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver) noexcept
      : driver_(driver), global_min_bound_(0), global_max_bound_(0), local_min_bound_(0), local_max_bound_(0),
        home_position_(0), amplitude_(1000.0F), frequency_hz_(0.5F), running_(false), start_time_us_(0),
        phase_offset_(0.0F), bounded_(false), steps_per_rev_(200), angle_unit_(AngleUnit::DEGREES), dwell_at_min_ms_(0),
        dwell_at_max_ms_(0), target_cycles_(0), current_cycles_(0), cycle_complete_(false),
        last_was_negative_(false), cycle_started_(false), last_target_relative_(0), state_(MotionState::STOPPED),
        dwell_start_time_ms_(0), sinusoidal_mode_(false), calculated_vmax_(10000.0f), calculated_amax_(5000.0f), estimated_frequency_hz_(0.0f) {
    // Mutex is automatically created by Esp32TmcMutex constructor
  }

  ~FatigueTestMotion() noexcept = default; // Mutex automatically destroyed by Esp32TmcMutex destructor

  /**
   * @brief Configure motor parameters for unit conversions
   * @param steps_per_rev Steps per revolution (e.g., 200 for 1.8° motor)
   * @param unit Preferred angle unit (degrees or radians)
   */
  void ConfigureMotor(uint16_t steps_per_rev, AngleUnit unit = AngleUnit::DEGREES) noexcept {
    TmcMutexGuard guard(mutex_);
    steps_per_rev_ = steps_per_rev;
    angle_unit_ = unit;
    ESP_LOGI(TAG, "Motor configured: %d steps/rev, angle unit: %s", steps_per_rev_,
             unit == AngleUnit::DEGREES ? "degrees" : "radians");
  }

  /**
   * @brief Set global bounds (hardware limits found during initialization)
   * @param min_bound Global minimum position in steps
   * @param max_bound Global maximum position in steps
   */
  void SetGlobalBounds(int32_t min_bound, int32_t max_bound) noexcept {
    {
      TmcMutexGuard guard(mutex_);
      global_min_bound_ = min_bound;
      global_max_bound_ = max_bound;
      bounded_ = true;
    }
    ESP_LOGI(TAG, "Global bounds set: min=%d, max=%d steps", global_min_bound_, global_max_bound_);
    if (steps_per_rev_ > 0) {
      float min_deg = tmc51x0::StepsToDegrees(global_min_bound_, steps_per_rev_);
      float max_deg = tmc51x0::StepsToDegrees(global_max_bound_, steps_per_rev_);
      ESP_LOGI(TAG, "Global bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
    }

    // Clip local bounds to global bounds if they exist
    {
      TmcMutexGuard guard(mutex_);
      if (local_min_bound_ != 0 || local_max_bound_ != 0) {
        guard.unlock(); // Unlock before calling ClipLocalBoundsToGlobal which will lock again
        ClipLocalBoundsToGlobal();
        return;
      }
    }
  }

  /**
   * @brief Get global bounds in degrees (for command interface)
   */
  void GetGlobalBoundsDegrees(float& min_degrees, float& max_degrees) const noexcept {
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

  /**
   * @brief Set unbounded mode (no mechanical stops found)
   * Uses current position as home and sets reasonable default global bounds
   */
  void SetUnbounded(int32_t current_position, int32_t default_range_steps = 10000) noexcept {
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
    ESP_LOGW(TAG, "Unbounded mode: No mechanical stops found");
    ESP_LOGI(TAG, "Using current position as home: %d steps", current_position);
    ESP_LOGI(TAG, "Default global range: [%d, %d] steps", min_bound, max_bound);
    if (steps > 0) {
      float range_deg = tmc51x0::StepsToDegrees(default_range_steps, steps);
      ESP_LOGI(TAG, "Default global range: %.2f°", range_deg);
    }
  }

  /**
   * @brief Set local bounds in degrees from center (thread-safe)
   * @param min_degrees_from_center Minimum angle from center (negative)
   * @param max_degrees_from_center Maximum angle from center (positive)
   */
  bool SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center) noexcept {
    if (steps_per_rev_ == 0) {
      ESP_LOGE(TAG, "Cannot set bounds: steps_per_rev not configured");
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
    ESP_LOGI(TAG, "Local bounds set: min=%.2f°, max=%.2f° from center", actual_min, actual_max);
    
    // Recalculate trajectory with new bounds
    {
      TmcMutexGuard guard(mutex_);
      RecalculateTrajectory();
    }
    return true;
  }

  /**
   * @brief Set frequency (thread-safe, can be changed in real-time)
   */
  bool SetFrequency(float frequency_hz) noexcept {
    if (frequency_hz < 0.0F || frequency_hz > 10.0F) {
      ESP_LOGE(TAG, "Invalid frequency: %.2f Hz (range: 0.0-10.0)", frequency_hz);
      return false;
    }
    {
      TmcMutexGuard guard(mutex_);
      frequency_hz_ = frequency_hz;
      RecalculateTrajectory();
    }
    ESP_LOGI(TAG, "Frequency updated: %.2f Hz", frequency_hz);
    return true;
  }

  /**
   * @brief Get frequency (thread-safe)
   */
  float GetFrequency() const noexcept {
    TmcMutexGuard guard(mutex_);
    return frequency_hz_;
  }

  /**
   * @brief Set dwell times (thread-safe, can be changed in real-time)
   */
  bool SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms) noexcept {
    {
      TmcMutexGuard guard(mutex_);
      dwell_at_min_ms_ = dwell_at_min_ms;
      dwell_at_max_ms_ = dwell_at_max_ms;
      RecalculateTrajectory();
    }
    ESP_LOGI(TAG, "Dwell times updated: min=%lu ms, max=%lu ms", 
             dwell_at_min_ms, dwell_at_max_ms);
    return true;
  }

  /**
   * @brief Get dwell times (thread-safe)
   */
  void GetDwellTimes(uint32_t& dwell_at_min_ms, uint32_t& dwell_at_max_ms) const noexcept {
    TmcMutexGuard guard(mutex_);
    dwell_at_min_ms = dwell_at_min_ms_;
    dwell_at_max_ms = dwell_at_max_ms_;
  }

  /**
   * @brief Set target cycle count (thread-safe)
   */
  bool SetTargetCycles(uint32_t cycles) noexcept {
    {
      TmcMutexGuard guard(mutex_);
      target_cycles_ = cycles;
    }
    ESP_LOGI(TAG, "Target cycles set: %lu (0 = infinite)", target_cycles_);
    return true;
  }

  /**
   * @brief Get current cycle count (thread-safe)
   */
  uint32_t GetCurrentCycles() const noexcept {
    TmcMutexGuard guard(mutex_);
    return current_cycles_;
  }

  /**
   * @brief Get target cycle count (thread-safe)
   */
  uint32_t GetTargetCycles() const noexcept {
    TmcMutexGuard guard(mutex_);
    return target_cycles_;
  }

  /**
   * @brief Check if cycle count reached (thread-safe)
   */
  bool IsCycleComplete() const noexcept {
    TmcMutexGuard guard(mutex_);
    return cycle_complete_;
  }

  /**
   * @brief Reset cycle count (thread-safe)
   */
  void ResetCycles() noexcept {
    {
      TmcMutexGuard guard(mutex_);
      current_cycles_ = 0;
      cycle_complete_ = false;
      last_was_negative_ = false;
      cycle_started_ = false;
      last_target_relative_ = 0;
    }
    ESP_LOGI(TAG, "Cycle count reset");
  }

  /**
   * @brief Get local bounds in degrees from center (thread-safe)
   */
  void GetLocalBoundsFromCenterDegrees(float& min_degrees, float& max_degrees) const noexcept {
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

  /**
   * @brief Check if system is bounded (thread-safe)
   */
  bool IsBounded() const noexcept {
    TmcMutexGuard guard(mutex_);
    return bounded_;
  }

  /**
   * @brief Start sinuous motion (thread-safe)
   */
  bool Start() noexcept {
    uint32_t current_cycles, target_cycles;
    int32_t min_pos, max_pos, current_pos;
    
    {
      TmcMutexGuard guard(mutex_);
      if (local_min_bound_ == 0 && local_max_bound_ == 0) {
        ESP_LOGE(TAG, "Cannot start: local bounds not set!");
        return false;
      }

      if (cycle_complete_) {
        ESP_LOGW(TAG, "Cycle count reached. Reset cycles or set new target to continue.");
        return false;
      }
      
      // Update trajectory before starting
      RecalculateTrajectory();

      // Configure driver for positioning mode
      driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
      driver_->rampControl.SetMaxSpeed(calculated_vmax_);
      driver_->rampControl.SetAcceleration(calculated_amax_);
      driver_->rampControl.SetDeceleration(calculated_amax_); // Symmetric acceleration/deceleration
      // Ensure VSTART/VSTOP/VZERO are reasonable
      driver_->rampControl.SetRampSpeeds(1000.0f, 100.0f, 0.0f);

      running_ = true;
      start_time_us_ = esp_timer_get_time();
      
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
        driver_->rampControl.SetTargetPosition(static_cast<float>(max_pos), tmc51x0::Unit::Steps);
      } else {
        state_ = MotionState::MOVING_TO_MIN;
        driver_->rampControl.SetTargetPosition(static_cast<float>(min_pos), tmc51x0::Unit::Steps);
      }

      current_cycles = current_cycles_;
      target_cycles = target_cycles_;
    }

    ESP_LOGI(TAG, "Starting fatigue test (cycles: %lu/%lu)", current_cycles,
             target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
    ESP_LOGI(TAG, "  Motion: Positioning mode, VMAX=%.1f, AMAX=%.1f", 
             calculated_vmax_, calculated_amax_);
    return true;
  }

  /**
   * @brief Stop sinuous motion (thread-safe)
   */
  void Stop() noexcept {
    uint32_t cycles;
    {
      TmcMutexGuard guard(mutex_);
      running_ = false;
      state_ = MotionState::STOPPED;
      cycles = current_cycles_;
    }
    driver_->rampControl.Stop();
    ESP_LOGI(TAG, "Stopped fatigue test motion (cycles completed: %lu)", cycles);
  }

  /**
   * @brief Check if motion is running (thread-safe)
   */
  bool IsRunning() const noexcept {
    TmcMutexGuard guard(mutex_);
    return running_ && state_ != MotionState::STOPPED;
  }

  /**
   * @brief Update motion (call this in main loop, thread-safe)
   */
  void Update() noexcept {
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
          
          // Move to center before stopping
          // We can just set target to home, and switch to a special "GOING_HOME" state if needed,
          // but essentially we just want to stop safely.
          // For simplicity, let's just stop update logic and let Start() handle restart.
          // Or better, trigger a move to home.
          
          state_ = MotionState::STOPPED;
          running_ = false;
          guard.unlock();
          
          ESP_LOGI(TAG, "Target cycle count reached: %lu cycles. Stopping.", cycles);
          driver_->rampControl.Stop();
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
      // Sinusoidal motion mode - update continuously based on sine wave
      UpdateSinuousMotion();
      return;
    }
    
    // Handle dwell states or ramp-based motion mode
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
    case MotionState::MOVING_TO_MAX:
      if (driver_->rampControl.IsTargetReached()) {
        // Reached Max
        TmcMutexGuard guard(mutex_);
        if (dwell_max > 0) {
          state_ = MotionState::DWELL_AT_MAX;
          dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        } else {
          state_ = MotionState::MOVING_TO_MIN;
          // Apply potentially new speed/accel for next leg
          driver_->rampControl.SetMaxSpeed(calculated_vmax_);
          driver_->rampControl.SetAcceleration(calculated_amax_);
          driver_->rampControl.SetDeceleration(calculated_amax_);
          driver_->rampControl.SetTargetPosition(static_cast<float>(min_bound), tmc51x0::Unit::Steps);
        }
      }
      break;

    case MotionState::MOVING_TO_MIN:
      if (driver_->rampControl.IsTargetReached()) {
        // Reached Min - Cycle Complete!
        {
          TmcMutexGuard guard(mutex_);
          current_cycles_++;
          if (dwell_min > 0) {
            state_ = MotionState::DWELL_AT_MIN;
            dwell_start_time_ms_ = esp_timer_get_time() / 1000;
          } else {
            state_ = MotionState::MOVING_TO_MAX;
            // Apply potentially new speed/accel for next leg
            driver_->rampControl.SetMaxSpeed(calculated_vmax_);
            driver_->rampControl.SetAcceleration(calculated_amax_);
            driver_->rampControl.SetDeceleration(calculated_amax_);
            driver_->rampControl.SetTargetPosition(static_cast<float>(max_bound), tmc51x0::Unit::Steps);
          }
        }
      }
      break;

    case MotionState::DWELL_AT_MIN:
      if (current_time_ms - dwell_start >= dwell_min) {
        TmcMutexGuard guard(mutex_);
        // Resume motion - if sinusoidal mode, UpdateSinuousMotion() will handle it
        // Otherwise, move to max bound
        if (!sinusoidal_mode_) {
        state_ = MotionState::MOVING_TO_MAX;
        driver_->rampControl.SetMaxSpeed(calculated_vmax_);
        driver_->rampControl.SetAcceleration(calculated_amax_);
        driver_->rampControl.SetDeceleration(calculated_amax_);
        driver_->rampControl.SetTargetPosition(static_cast<float>(max_bound), tmc51x0::Unit::Steps);
        } else {
          // Sinusoidal mode - will be handled by UpdateSinuousMotion()
          state_ = MotionState::MOVING_TO_MAX;
        }
      }
      break;

    case MotionState::DWELL_AT_MAX:
      if (current_time_ms - dwell_start >= dwell_max) {
        TmcMutexGuard guard(mutex_);
        // Resume motion - if sinusoidal mode, UpdateSinuousMotion() will handle it
        // Otherwise, move to min bound
        if (!sinusoidal_mode_) {
        state_ = MotionState::MOVING_TO_MIN;
        driver_->rampControl.SetMaxSpeed(calculated_vmax_);
        driver_->rampControl.SetAcceleration(calculated_amax_);
        driver_->rampControl.SetDeceleration(calculated_amax_);
        driver_->rampControl.SetTargetPosition(static_cast<float>(min_bound), tmc51x0::Unit::Steps);
        } else {
          // Sinusoidal mode - will be handled by UpdateSinuousMotion()
          state_ = MotionState::MOVING_TO_MIN;
        }
      }
      break;

    case MotionState::STOPPED:
      break;
    }
  }

  /**
   * @brief Get status information (thread-safe)
   */
  struct Status {
    bool running;
    bool bounded;
    float frequency_hz;
    float min_degrees_from_center;
    float max_degrees_from_center;
    uint32_t current_cycles;
    uint32_t target_cycles;
    uint32_t dwell_min_ms;
    uint32_t dwell_max_ms;
    float global_min_degrees;
    float global_max_degrees;
  };

  Status GetStatus() const noexcept {
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

  /**
   * @brief Get estimated actual frequency (thread-safe)
   */
  float GetEstimatedFrequency() const noexcept {
    TmcMutexGuard guard(mutex_);
    return estimated_frequency_hz_;
  }

private:
  /**
   * @brief Clip local bounds to global bounds
   */
  void ClipLocalBoundsToGlobal() noexcept {
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
      ESP_LOGW(TAG, "Local bounds clipped to global bounds");
      ESP_LOGI(TAG, "Clipped local bounds: min=%d, max=%d steps", local_min_bound_, local_max_bound_);
      if (steps_per_rev_ > 0) {
        float min_deg = tmc51x0::StepsToDegrees(local_min_bound_, steps_per_rev_);
        float max_deg = tmc51x0::StepsToDegrees(local_max_bound_, steps_per_rev_);
        ESP_LOGI(TAG, "Clipped local bounds: min=%.2f°, max=%.2f°", min_deg, max_deg);
      }
    }
  }

  /**
   * @brief Update sinuous motion target position
   */
  void UpdateSinuousMotion() noexcept {
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

    // Cycle counting: one cycle = center → min → max → center (or center → max → min → center)
    // Count cycles when crossing through center (0 crossing point)
      // Check if we're crossing through center (sign change of target position)
      bool currently_negative = (target_relative < 0);
      bool last_was_negative = (last_target_rel < 0);
      bool crossing_center =
          (last_was_negative != currently_negative) && (abs(target_relative) < 30) && (abs(last_target_rel) < 30);

      // If we've started a cycle (left center) and now crossing back through center
      if (cycle_started && crossing_center) {
        // Completed a cycle: center → extreme → center
        uint32_t new_cycles;
        {
          TmcMutexGuard guard(mutex_);
          current_cycles_++;
          cycle_started_ = false; // Reset for next cycle
          new_cycles = current_cycles_;
        }
        ESP_LOGI(TAG, "Cycle %lu completed at center (target: %lu)", new_cycles, 
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
        if (abs(target_relative) > 10) { // Only update if significantly away from center
          last_was_negative_ = currently_negative;
      }
    }

    // Clamp to local bounds and handle dwell states
    // During sinusoidal motion, we transition to proper motion states when hitting bounds
    if (target <= local_min) {
      target = local_min;
      if (dwell_min > 0) {
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::DWELL_AT_MIN;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(static_cast<float>(target), tmc51x0::Unit::Steps);
        return;
      } else {
        // No dwell - continue sinusoidal motion (will reverse direction naturally)
        // Update state to indicate we're moving away from min
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
        // No dwell - continue sinusoidal motion (will reverse direction naturally)
        // Update state to indicate we're moving away from max
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::MOVING_TO_MIN;
      }
    } else {
      // Between bounds - update state based on direction
      TmcMutexGuard guard(mutex_);
      if (target_relative > 0) {
        state_ = MotionState::MOVING_TO_MAX;
      } else {
        state_ = MotionState::MOVING_TO_MIN;
      }
    }

    // Update target position if it changed significantly
    if (abs(target - current_pos) > 10) { // Update threshold: 10 steps
      driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(static_cast<float>(target), tmc51x0::Unit::Steps);
      driver_->rampControl.SetMaxSpeed(1000.0F);
      driver_->rampControl.SetAcceleration(2000.0F);
    }
  }
};

//=============================================================================
// UART Command Parser
//=============================================================================

/**
 * @brief Command argument structure for modular command parsing
 */
struct CommandArg {
  const char* short_name;    // e.g., "-f"
  const char* long_name;     // e.g., "--freq"
  const char* description;   // Help text
  int min_args;              // Minimum number of arguments required
  int max_args;              // Maximum number of arguments (0 = unlimited)
};

/**
 * @brief Command handler function type
 */
typedef bool (*CommandHandler)(const std::vector<std::string>& args, FatigueTestMotion& motion) noexcept;

/**
 * @brief Command registry entry
 */
struct CommandEntry {
  CommandArg arg;
  CommandHandler handler;
};

/**
 * @brief Modular UART command parser with Linux-like argument structure
 */
class UartCommandParser {
private:
  uart_port_t uart_port_;
  std::vector<CommandEntry> commands_;
  char rx_buffer_[256];
  static constexpr size_t RX_BUF_SIZE = 256;

  /**
   * @brief Parse command line into tokens
   */
  std::vector<std::string> Tokenize(const char* line) noexcept {
    std::vector<std::string> tokens;
    std::string current;
    bool in_quotes = false;

    for (const char* p = line; *p != '\0'; ++p) {
      if (*p == '"') {
        in_quotes = !in_quotes;
      } else if (isspace(*p) && !in_quotes) {
        if (!current.empty()) {
          tokens.push_back(current);
          current.clear();
        }
      } else {
        current += *p;
      }
    }

    if (!current.empty()) {
      tokens.push_back(current);
    }

    return tokens;
  }

  /**
   * @brief Find command handler for given argument
   */
  CommandEntry* FindCommand(const std::string& arg) noexcept {
    for (auto& entry : commands_) {
      if (arg == entry.arg.short_name || arg == entry.arg.long_name) {
        return &entry;
      }
    }
    return nullptr;
  }

public:
  UartCommandParser(uart_port_t uart_port) : uart_port_(uart_port) {
    // Configure UART
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(uart_port_, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart_port_, &uart_config);
    uart_set_pin(uart_port_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  }

  /**
   * @brief Register a command handler
   */
  void RegisterCommand(const CommandArg& arg, CommandHandler handler) noexcept {
    commands_.push_back({arg, handler});
  }

  /**
   * @brief Process a command line
   */
  bool ProcessCommand(const char* line, FatigueTestMotion& motion) noexcept {
    if (!line || strlen(line) == 0) {
      return false;
    }

    std::vector<std::string> tokens = Tokenize(line);
    if (tokens.empty()) {
      return false;
    }

    // Special handling for help command
    if (tokens[0] == "-h" || tokens[0] == "--help") {
      PrintHelp();
      return true;
    }

    // Find command
    CommandEntry* entry = FindCommand(tokens[0]);
    if (!entry) {
      ESP_LOGW(TAG, "Unknown command: %s", tokens[0].c_str());
      return false;
    }

    // Check argument count
    int arg_count = tokens.size() - 1;
    if (arg_count < entry->arg.min_args) {
      ESP_LOGE(TAG, "Command %s requires at least %d arguments, got %d", 
               tokens[0].c_str(), entry->arg.min_args, arg_count);
      return false;
    }
    if (entry->arg.max_args > 0 && arg_count > entry->arg.max_args) {
      ESP_LOGE(TAG, "Command %s accepts at most %d arguments, got %d", 
               tokens[0].c_str(), entry->arg.max_args, arg_count);
      return false;
    }

    // Extract arguments
    std::vector<std::string> args(tokens.begin() + 1, tokens.end());

    // Call handler
    return entry->handler(args, motion);
  }

  /**
   * @brief Read and process commands from UART
   */
  void ProcessUartCommands(FatigueTestMotion& motion) noexcept {
    int len = uart_read_bytes(uart_port_, (uint8_t*)rx_buffer_, RX_BUF_SIZE - 1, pdMS_TO_TICKS(100));
    if (len > 0) {
      rx_buffer_[len] = '\0';
      
      // Remove trailing newline/carriage return
      while (len > 0 && (rx_buffer_[len - 1] == '\n' || rx_buffer_[len - 1] == '\r')) {
        rx_buffer_[len - 1] = '\0';
        len--;
      }

      if (len > 0) {
        ProcessCommand(rx_buffer_, motion);
      }
    }
  }

  /**
   * @brief Print help message
   */
  void PrintHelp() noexcept {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                         UART COMMAND INTERFACE                             ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "Commands:");
    for (const auto& entry : commands_) {
      ESP_LOGI(TAG, "  %s, %s : %s", entry.arg.short_name, entry.arg.long_name, entry.arg.description);
    }
    ESP_LOGI(TAG, "");
  }
};

// Command handlers
static bool HandleFrequency(const std::vector<std::string>& args, FatigueTestMotion& motion) noexcept {
  if (args.empty()) {
    ESP_LOGE(TAG, "Frequency command requires a value");
    return false;
  }
  float freq = std::strtof(args[0].c_str(), nullptr);
  return motion.SetFrequency(freq);
}

  static bool HandleDwell(const std::vector<std::string>& args, FatigueTestMotion& motion) noexcept {
    if (args.size() < 2) {
      ESP_LOGE(TAG, "Dwell command requires at least 2 arguments (min_ms, max_ms)");
      return false;
    }
    if (args.size() > 2) {
      ESP_LOGW(TAG, "Extra arguments ignored. Dwell command takes exactly 2 arguments (min_ms, max_ms)");
    }
    uint32_t min_ms = std::strtoul(args[0].c_str(), nullptr, 10);
    uint32_t max_ms = std::strtoul(args[1].c_str(), nullptr, 10);
    return motion.SetDwellTimes(min_ms, max_ms);
  }

static bool HandleBounds(const std::vector<std::string>& args, FatigueTestMotion& motion) noexcept {
  if (args.size() < 2) {
    ESP_LOGE(TAG, "Bounds command requires 2 arguments (min_degrees, max_degrees)");
    return false;
  }
  float min_deg = std::strtof(args[0].c_str(), nullptr);
  float max_deg = std::strtof(args[1].c_str(), nullptr);
  return motion.SetLocalBoundsFromCenterDegrees(min_deg, max_deg);
}

static bool HandleCycles(const std::vector<std::string>& args, FatigueTestMotion& motion) noexcept {
  if (args.empty()) {
    ESP_LOGE(TAG, "Cycles command requires a value");
    return false;
  }
  uint32_t cycles = std::strtoul(args[0].c_str(), nullptr, 10);
  return motion.SetTargetCycles(cycles);
}

static bool HandleAction(const std::vector<std::string>& args, FatigueTestMotion& motion) noexcept {
  if (args.empty()) {
    ESP_LOGE(TAG, "Action command requires an action (start/stop/reset)");
    return false;
  }
  
  const std::string& action = args[0];
  if (action == "start") {
    return motion.Start();
  } else if (action == "stop") {
    motion.Stop();
    return true;
  } else if (action == "reset") {
    motion.ResetCycles();
    return true;
  } else {
    ESP_LOGE(TAG, "Unknown action: %s (use: start, stop, or reset)", action.c_str());
    return false;
  }
}

static bool HandleStatus(const std::vector<std::string>& args, FatigueTestMotion& motion) noexcept {
  (void)args; // Unused
  FatigueTestMotion::Status status = motion.GetStatus();
  
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                            MOTION STATUS                                      ║");
  ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════════════╣");
  ESP_LOGI(TAG, "  Running: %s", status.running ? "YES" : "NO");
  ESP_LOGI(TAG, "  Bounded: %s", status.bounded ? "YES" : "NO");
  ESP_LOGI(TAG, "  Frequency: %.2f Hz (Estimated: %.2f Hz)", status.frequency_hz, motion.GetEstimatedFrequency());
  ESP_LOGI(TAG, "  Local Bounds: %.2f° to %.2f° from center", 
           status.min_degrees_from_center, status.max_degrees_from_center);
  ESP_LOGI(TAG, "  Global Bounds: %.2f° to %.2f° from center", 
           status.global_min_degrees, status.global_max_degrees);
  ESP_LOGI(TAG, "  Cycles: %lu / %lu %s", status.current_cycles, 
           status.target_cycles == 0 ? 0xFFFFFFFF : status.target_cycles,
           status.target_cycles == 0 ? "(infinite)" : "");
  ESP_LOGI(TAG, "  Dwell Times: min=%lu ms, max=%lu ms",
           status.dwell_min_ms, status.dwell_max_ms);
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  return true;
}


//=============================================================================
// Motion Control Task
//=============================================================================

/**
 * @brief Motion control task - handles bounds finding and sinusoidal motion
 */
void motion_control_task(void* param) noexcept {
  FatigueTestMotion* motion = static_cast<FatigueTestMotion*>(param);
  
  ESP_LOGI(TAG, "Motion control task started");
  
  while (true) {
    motion->Update();
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms update rate
  }
}

//=============================================================================
// UART Command Task
//=============================================================================

/**
 * @brief UART command processing task
 */
void uart_command_task(void* param) noexcept {
  struct TaskParams {
    UartCommandParser* parser;
    FatigueTestMotion* motion;
  };
  
  TaskParams* params = static_cast<TaskParams*>(param);
  UartCommandParser* parser = params->parser;
  FatigueTestMotion* motion = params->motion;
  
  ESP_LOGI(TAG, "UART command task started");
  ESP_LOGI(TAG, "Type '--help' or '-h' for command list");
  
  while (true) {
    parser->ProcessUartCommands(*motion);
    vTaskDelay(pdMS_TO_TICKS(50)); // Check for commands every 50ms
  }
}

//=============================================================================
// Main Application
//=============================================================================

extern "C" void app_main() {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║         TMC51x0 Fatigue Test Platform: Bounds Finding & Sinuous Motion      ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Get default pin configuration (matches dev board setup)
  auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
  
  // Default: EN is active LOW (LOW = enable, HIGH = disable) per TMC51x0 datasheet
  // If your board has an inverter, set en = true in PinActiveLevels
  tmc51x0::PinActiveLevels active_levels; // Uses defaults: en=false (LOW=enable)
  
  // Uncomment the line below if your board has an inverter on the EN pin:
  // active_levels.en = true; // EN pin has inverter, so ACTIVE = HIGH to enable
  
  // Create SPI communication interface using default pin configuration
  Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, 1000000, active_levels); // 1 MHz SPI clock (reduced for stability)

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC51x0 driver instance
  tmc51x0::TMC51x0<Esp32SPI> driver(spi);

  // Configure driver from unified test rig selection
  // This automatically selects motor, board, and platform based on SELECTED_TEST_RIG
  tmc51x0::DriverConfig cfg{};
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
  
  // Get motor output steps for unit conversions
  constexpr uint16_t output_full_steps = 
      tmc51x0_test_config::GetTestRigMotorOutputFullSteps<SELECTED_TEST_RIG>();
  
  // Log test rig info
  if constexpr (SELECTED_TEST_RIG == tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER) {
    ESP_LOGI(TAG, "Test Rig: Core Driver (17HS4401S)");
  } else if constexpr (SELECTED_TEST_RIG == tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE) {
    ESP_LOGI(TAG, "Test Rig: Fatigue (Applied Motion 5034-369 NEMA 34)");
  }
  
  // Test configuration (currently shared across motors, can be motor-specific if needed)
  namespace Test = tmc51x0_test_config::TestConfig_17HS4401S;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC51x0 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // CRITICAL: StallGuard2 ONLY works in SpreadCycle mode (en_stealthchop_mode=0)!
  // Explicitly enable SpreadCycle mode for sensorless homing
  tmc51x0::GlobalConfig gconf_read;
  if (!driver.motorControl.GetGlobalConfig(gconf_read)) {
    ESP_LOGE(TAG, "Failed to read GCONF register");
    return;
  }
  
  // Always ensure SpreadCycle is enabled (en_stealthchop_mode = 0)
  if (gconf_read.en_stealthchop_mode) {
    ESP_LOGW(TAG, "StealthChop is enabled - switching to SpreadCycle mode for StallGuard2");
    if (!driver.motorControl.SetStealthChopEnabled(false)) {
      ESP_LOGE(TAG, "Failed to enable SpreadCycle mode");
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay for mode switch
  } else {
    ESP_LOGI(TAG, "Already in SpreadCycle mode - verifying for StallGuard2");
  }
  
  // Verify SpreadCycle is enabled
  if (driver.motorControl.GetGlobalConfig(gconf_read)) {
    if (!gconf_read.en_stealthchop_mode) {
      ESP_LOGI(TAG, "✓ SpreadCycle mode confirmed enabled (en_stealthchop_mode=0) - StallGuard2 ready");
    } else {
      ESP_LOGE(TAG, "✗ CRITICAL: SpreadCycle mode NOT enabled!");
      return;
    }
  } else {
    ESP_LOGW(TAG, "Could not verify SpreadCycle mode - proceeding anyway");
  }

  // CRITICAL: Set TCOOLTHRS and THIGH BEFORE configuring StallGuard2
  // TCOOLTHRS = velocity threshold below which StallGuard2 is disabled
  // For homing, we want StallGuard2 active at search speed, so set appropriately
  // Setting to a low value ensures StallGuard2 is active at search speeds
  if (driver.motorControl.SetCoolStepThreshold(1000.0f, tmc51x0::Unit::Steps)) {
    ESP_LOGI(TAG, "✓ TCOOLTHRS set to 1000 - StallGuard2 active at search speeds");
  } else {
    ESP_LOGE(TAG, "✗ Failed to set TCOOLTHRS");
  }
  
  // THIGH = velocity threshold for chopper mode switching (set high to avoid interference)
  if (driver.motorControl.SetHighSpeedThreshold(1048575.0f, tmc51x0::Unit::Steps)) { // 0xFFFFF
    ESP_LOGI(TAG, "✓ THIGH set to maximum (0xFFFFF)");
  } else {
    ESP_LOGW(TAG, "Failed to set THIGH (may not be critical)");
  }

  // Configure StallGuard2 for bounds finding using test defaults
  // NOTE: SGT threshold tuning guide:
  //   - Lower values (-64 to 0) = More sensitive (stops easier, more false positives)
  //   - Higher values (0 to +63) = Less sensitive (needs more force to stop, fewer false positives)
  //   - For free-rotating motors with no mechanical stops, use higher values (0 to +20)
  //   - For motors with mechanical stops, use lower values (-20 to 0)
  //   - If getting false stalls (SG_RESULT=0 during normal motion), INCREASE SGT value
  tmc51x0::StallGuardConfig sg_config{};
  sg_config.threshold = Test::StallGuard::SGT_HOMING; 
  sg_config.enable_filter = Test::StallGuard::FILTER_ENABLED;
  // Note: semin/semax are CoolStep parameters, configure separately if needed

  ESP_LOGI(TAG, "Configuring StallGuard2: threshold=%d (lower=more sensitive, higher=less sensitive)", 
           static_cast<int>(sg_config.threshold));
  if (sg_config.threshold < 0) {
    ESP_LOGW(TAG, "  Note: Threshold is negative (sensitive) - may cause false stalls on free-rotating motors");
    ESP_LOGW(TAG, "  Consider increasing threshold to 0 or +10 if experiencing false stalls");
  }
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
  
  // Ensure clean state before homing - clear any existing stall flags
  driver.diagnostics.ClearStallFlag();
  ESP_LOGI(TAG, "Cleared any existing stall flags");

  // Configure motor parameters for unit conversions
  // Steps per output revolution = Motor Full Steps * Gear Ratio * Microsteps
  // 200 * 5.18 * 256 = ~265,216 steps/rev (for geared motor)
  float steps_per_rev = static_cast<float>(output_full_steps) * 256.0f; 
  
  // ============================================================
  // STEP 1: Find global bounds using positioning mode with StallGuard2 stop
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                    STEP 1: Finding Global Bounds                            ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Calculate 360 degrees in steps (one full output revolution)
  // steps_per_rev already accounts for microsteps (OUTPUT_FULL_STEPS * 256)
  int32_t steps_per_360_deg = static_cast<int32_t>(steps_per_rev);
  float search_speed = Test::Motion::BOUNDS_SEARCH_SPEED; // steps/s
  
  // Calculate 5° offset in steps (for backing off from detected stalls)
  // Using utility function to ensure proper microstep handling
  float offset_deg = 5.0f;
  int32_t offset_steps = tmc51x0::DegreesToSteps(offset_deg, steps_per_rev);
  
  // CRITICAL: Reset position to 0 and ensure clean state before starting
  float initial_pos_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(initial_pos_float, tmc51x0::Unit::Steps)) {
    initial_pos_float = 0.0f;
  }
  int32_t initial_position = static_cast<int32_t>(initial_pos_float);
  ESP_LOGI(TAG, "Initial position before reset: %ld steps", initial_position);
  
  driver.rampControl.Stop();
  driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Reset position to 0
  driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
  float pos_after_reset_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(pos_after_reset_float, tmc51x0::Unit::Steps)) {
    pos_after_reset_float = 0.0f;
  }
  int32_t position_after_reset = static_cast<int32_t>(pos_after_reset_float);
  ESP_LOGI(TAG, "Position reset to: %ld steps (should be 0)", position_after_reset);
  
  // Ensure motor is enabled
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor driver");
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "✓ Motor driver enabled");
  
  // CRITICAL: Disable reference switches to prevent motion blocking
  // Use ConfigureReferenceSwitch() like internal_ramp_sinusoidal.cpp does for proper configuration
  tmc51x0::ReferenceSwitchConfig ref_cfg{};
  // Configure switches but disable motor stop (allows reading switch state without stopping)
  ref_cfg.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
  ref_cfg.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
  ref_cfg.left_switch_stop_enable = false;   // Don't stop motor
  ref_cfg.right_switch_stop_enable = false;  // Don't stop motor
  ref_cfg.latch_left = tmc51x0::ReferenceLatchMode::DISABLED;   // No latching
  ref_cfg.latch_right = tmc51x0::ReferenceLatchMode::DISABLED;  // No latching
  if (!driver.rampControl.ConfigureReferenceSwitch(ref_cfg)) {
    ESP_LOGW(TAG, "Failed to configure reference switches (may not be critical)");
  } else {
    ESP_LOGI(TAG, "✓ Reference switches disabled (not using endstops)");
    
    // Verify SW_MODE register was written correctly
    tmc51x0::ReferenceSwitchConfig verify_ref_cfg{};
    if (driver.rampControl.GetReferenceSwitchConfig(verify_ref_cfg)) {
      ESP_LOGI(TAG, "SW_MODE verification: stop_l_enable=%d, stop_r_enable=%d, stop_mode=%s",
               verify_ref_cfg.left_switch_stop_enable ? 1 : 0,
               verify_ref_cfg.right_switch_stop_enable ? 1 : 0,
               (verify_ref_cfg.stop_mode == tmc51x0::ReferenceStopMode::SOFT_STOP) ? "SOFT_STOP" : "HARD_STOP");
      
      if (verify_ref_cfg.left_switch_stop_enable || verify_ref_cfg.right_switch_stop_enable) {
        ESP_LOGE(TAG, "ERROR: Reference switches still enabled in SW_MODE!");
        ESP_LOGE(TAG, "Motion will be blocked. Re-configuring...");
        // Try again
        driver.rampControl.ConfigureReferenceSwitch(ref_cfg);
        vTaskDelay(pdMS_TO_TICKS(50));
      } else {
        ESP_LOGI(TAG, "✓ Reference switches confirmed disabled in SW_MODE");
      }
    }
  }
  
  // Check physical pin states (informational only - switches are disabled)
  uint32_t ramp_stat_precheck = 0;
  if (driver.diagnostics.GetRampStatusRegister(ramp_stat_precheck)) {
    tmc51x0::RAMP_STAT_Register stat{};
    stat.value = ramp_stat_precheck;
    if (stat.bits.status_stop_l || stat.bits.status_stop_r) {
      ESP_LOGW(TAG, "Reference switch pins are active (stop_l=%d, stop_r=%d) but stops are DISABLED",
               stat.bits.status_stop_l ? 1 : 0, stat.bits.status_stop_r ? 1 : 0);
      ESP_LOGI(TAG, "  Motion should work since stop_l_enable=0 and stop_r_enable=0 in SW_MODE");
      ESP_LOGI(TAG, "  The status bits just reflect pin state - they don't block motion when disabled");
    }
  }
  
  // Enable StallGuard2 stop for automatic stall detection in positioning mode
  // IMPORTANT: Must preserve reference switch disable settings from above
  // Per datasheet SW_MODE register (0x34):
  //   Bit 10 (sg_stop): 1 = Enable stop by StallGuard2
  //   Bit 11 (en_softstop): 0 = Hard stop (REQUIRED for StallGuard2)
  //   Datasheet: "Attention: Do not use soft stop in combination with StallGuard2"
  // Note: Reference switches are already disabled via ConfigureReferenceSwitch above
  if (!driver.diagnostics.EnableStopOnStall(true)) {
    ESP_LOGE(TAG, "Failed to enable StallGuard2 stop");
    return;
  }
  
  // Verify SW_MODE was written correctly per datasheet requirements
  // Ensure hard stop mode (required for StallGuard2)
  if (!driver.rampControl.SetStopMode(tmc51x0::ReferenceStopMode::HARD_STOP)) {
    ESP_LOGE(TAG, "Failed to set hard stop mode");
  }
  
  // Verify configuration
  tmc51x0::ReferenceSwitchConfig verify_ref_cfg{};
  if (driver.rampControl.GetReferenceSwitchConfig(verify_ref_cfg)) {
    ESP_LOGI(TAG, "SW_MODE register verification:");
    ESP_LOGI(TAG, "  stop_l_enable: %d, stop_r_enable: %d", 
             verify_ref_cfg.left_switch_stop_enable ? 1 : 0, 
             verify_ref_cfg.right_switch_stop_enable ? 1 : 0);
    ESP_LOGI(TAG, "  stop_mode: %s (HARD_STOP required for StallGuard2)", 
             (verify_ref_cfg.stop_mode == tmc51x0::ReferenceStopMode::SOFT_STOP) ? "SOFT_STOP" : "HARD_STOP");
    
    if (verify_ref_cfg.stop_mode == tmc51x0::ReferenceStopMode::SOFT_STOP) {
      ESP_LOGE(TAG, "✗ CRITICAL: Soft stop is enabled! Datasheet says 'Do not use soft stop in combination with StallGuard2'!");
      ESP_LOGE(TAG, "  This will cause incorrect behavior. Fixing...");
      driver.rampControl.SetStopMode(tmc51x0::ReferenceStopMode::HARD_STOP);
    } else {
      ESP_LOGI(TAG, "✓ Hard stop enabled (correct for StallGuard2)");
    }
    
    if (verify_ref_cfg.left_switch_stop_enable || verify_ref_cfg.right_switch_stop_enable) {
      ESP_LOGE(TAG, "✗ Reference switches are still enabled! This will block motion!");
    } else {
      ESP_LOGI(TAG, "✓ Reference switches disabled (stop_l_enable=0, stop_r_enable=0)");
    }
  } else {
    ESP_LOGW(TAG, "Failed to verify SW_MODE register");
  }
  
  // Verify sg_stop is enabled (via EnableStopOnStall)
  ESP_LOGI(TAG, "✓ StallGuard2 stop enabled (sg_stop=1)");
  
  // Clear any existing stall flags and verify they're cleared
  if (!driver.diagnostics.ClearStallFlag()) {
    ESP_LOGE(TAG, "Failed to clear stall flags");
    return;
  }
  
  // Verify stall flag is cleared
  vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for register write
  if (driver.diagnostics.IsStallDetected()) {
    ESP_LOGW(TAG, "Warning: Stall flag still set after clear attempt");
  }
  
  // Read initial SG_RESULT for baseline diagnostics
  uint16_t sg_result_baseline = 0;
  if (driver.diagnostics.GetStallGuardResult(sg_result_baseline)) {
    ESP_LOGI(TAG, "Initial SG_RESULT baseline: %d (at standstill)", sg_result_baseline);
  } else {
    if (!driver.diagnostics.GetStallGuard(sg_result_baseline)) {
      sg_result_baseline = 0; // Fallback if read fails
    }
    ESP_LOGI(TAG, "Initial SG_RESULT baseline: %d (at standstill)", sg_result_baseline);
  }
  if (sg_result_baseline == 0) {
    ESP_LOGE(TAG, "⚠️ CRITICAL: SG_RESULT=0 at standstill indicates:");
    ESP_LOGE(TAG, "  1. Motor wiring may be incorrect (phases swapped)");
    ESP_LOGE(TAG, "  2. Motor current too high (try reducing IRUN)");
    ESP_LOGE(TAG, "  3. StallGuard2 threshold needs adjustment (SGT too sensitive)");
    ESP_LOGE(TAG, "  Current SGT=%d - consider increasing to 0 or +10", Test::StallGuard::SGT_HOMING);
  }
  
  // Configure positioning mode parameters
  // CRITICAL: Ensure motor is stopped before changing RAMPMODE
  driver.rampControl.Stop();
  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for motor to fully stop
  
  // Read current RAMPMODE
  tmc51x0::RampMode rampmode_before = tmc51x0::RampMode::HOLD;
  if (driver.rampControl.GetRampMode(rampmode_before)) {
    const char* mode_str = (rampmode_before == tmc51x0::RampMode::HOLD) ? "HOLD" :
                          (rampmode_before == tmc51x0::RampMode::POSITIONING) ? "POSITIONING" :
                          (rampmode_before == tmc51x0::RampMode::VELOCITY_POS) ? "VELOCITY_POS" :
                          (rampmode_before == tmc51x0::RampMode::VELOCITY_NEG) ? "VELOCITY_NEG" : "UNKNOWN";
    ESP_LOGI(TAG, "Current RAMPMODE before setting: %s", mode_str);
  }
  
  // CRITICAL: Set RAMPMODE to POSITIONING and verify it sticks
  if (!driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING)) {
    ESP_LOGE(TAG, "Failed to set RAMPMODE to POSITIONING!");
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay for register write
  
  // Verify RAMPMODE was set correctly
  tmc51x0::RampMode rampmode_verify = tmc51x0::RampMode::HOLD;
  if (driver.rampControl.GetRampMode(rampmode_verify)) {
    if (rampmode_verify != tmc51x0::RampMode::POSITIONING) {
      ESP_LOGE(TAG, "CRITICAL: RAMPMODE not set to POSITIONING! Current mode: %d", static_cast<int>(rampmode_verify));
      return;
    } else {
      ESP_LOGI(TAG, "✓ RAMPMODE confirmed POSITIONING");
    }
  } else {
    ESP_LOGE(TAG, "Failed to read RAMPMODE register!");
    return;
  }
  
  driver.rampControl.SetMaxSpeed(search_speed);
  driver.rampControl.SetAcceleration(search_speed * 2.0f); // Reasonable acceleration
  driver.rampControl.SetDeceleration(search_speed * 2.0f);
  driver.rampControl.SetRampSpeeds(1000.0f, 100.0f, 0.0f); // VSTART, VSTOP, V1
  
  // Small delay to ensure all registers are written before starting motion
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // ============================================================
  // Find maximum bound: Command to +360° and detect stall
  // ============================================================
  ESP_LOGI(TAG, "Finding maximum bound: Commanding to +360° (%ld steps)...", steps_per_360_deg);
  
  // Verify current position before setting target
  float pos_before_target_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(pos_before_target_float, tmc51x0::Unit::Steps)) {
    pos_before_target_float = 0.0f;
  }
  int32_t pos_before_target = static_cast<int32_t>(pos_before_target_float);
  ESP_LOGI(TAG, "Current position before setting target: %ld steps", pos_before_target);
  
  // Set target position
  if (!driver.rampControl.SetTargetPosition(static_cast<float>(steps_per_360_deg), tmc51x0::Unit::Steps)) {
    ESP_LOGE(TAG, "Failed to set target position!");
    return;
  }
  
  // Target position set via SetTargetPosition() - no need to verify register directly
  
  // Small delay to allow motion to start
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Check if motion started
  float initial_speed = 0.0f;
  if (!driver.rampControl.GetCurrentSpeed(initial_speed, tmc51x0::Unit::Steps)) {
    initial_speed = 0.0f;
  }
  float initial_pos_check_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(initial_pos_check_float, tmc51x0::Unit::Steps)) {
    initial_pos_check_float = 0.0f;
  }
  int32_t initial_pos_check = static_cast<int32_t>(initial_pos_check_float);
  ESP_LOGI(TAG, "After setting target: position=%ld, speed=%.1f steps/s", initial_pos_check, initial_speed);
  
  if (std::abs(initial_speed) < 10.0f && std::abs(initial_pos_check - pos_before_target) < 10) {
    ESP_LOGW(TAG, "⚠️ Motor not moving after setting target! Checking status...");
    
    // Read RAMP_STAT to see why motion isn't starting
    uint32_t ramp_stat_no_motion = 0;
    if (driver.diagnostics.GetRampStatusRegister(ramp_stat_no_motion)) {
      tmc51x0::RAMP_STAT_Register stat{};
      stat.value = ramp_stat_no_motion;
      ESP_LOGW(TAG, "RAMP_STAT: vzero=%d, velocity_reached=%d, position_reached=%d, stop_l=%d, stop_r=%d, event_stop_sg=%d",
               stat.bits.vzero ? 1 : 0,
               stat.bits.velocity_reached ? 1 : 0,
               stat.bits.position_reached ? 1 : 0,
               stat.bits.status_stop_l ? 1 : 0,
               stat.bits.status_stop_r ? 1 : 0,
               stat.bits.event_stop_sg ? 1 : 0);
      
      if (stat.bits.position_reached) {
        ESP_LOGW(TAG, "  Position already reached - motor may be at target already");
      }
      // Check if reference switches are blocking (only if enabled)
      if (stat.bits.status_stop_l || stat.bits.status_stop_r) {
        // Check if switches are actually enabled
        tmc51x0::ReferenceSwitchConfig sw_mode_check{};
        if (driver.rampControl.GetReferenceSwitchConfig(sw_mode_check)) {
          if (sw_mode_check.left_switch_stop_enable || sw_mode_check.right_switch_stop_enable) {
            ESP_LOGE(TAG, "  CRITICAL: Reference switches are ENABLED and ACTIVE - blocking motion!");
            ESP_LOGE(TAG, "  stop_l_enable=%d, stop_r_enable=%d", 
                     sw_mode_check.left_switch_stop_enable ? 1 : 0, 
                     sw_mode_check.right_switch_stop_enable ? 1 : 0);
            ESP_LOGE(TAG, "  Disabling reference switches...");
            tmc51x0::ReferenceSwitchConfig ref_cfg_disable{};
            // Disable motor stop (keep polarity configured for reading switch state)
            ref_cfg_disable.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
            ref_cfg_disable.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
            ref_cfg_disable.left_switch_stop_enable = false;   // Don't stop motor
            ref_cfg_disable.right_switch_stop_enable = false;  // Don't stop motor
            ref_cfg_disable.stop_mode = tmc51x0::ReferenceStopMode::HARD_STOP;
            ref_cfg_disable.latch_left = tmc51x0::ReferenceLatchMode::DISABLED;   // No latching
            ref_cfg_disable.latch_right = tmc51x0::ReferenceLatchMode::DISABLED;  // No latching
            driver.rampControl.ConfigureReferenceSwitch(ref_cfg_disable);
            vTaskDelay(pdMS_TO_TICKS(50));
          } else {
            ESP_LOGI(TAG, "  Reference switch pins active but DISABLED - motion should work");
          }
        }
      }
    }
    
    // CRITICAL: Ensure we're in POSITIONING mode (not HOLD)
    // Stall events or other conditions might force RAMPMODE back to HOLD
    tmc51x0::RampMode rampmode_check = tmc51x0::RampMode::HOLD;
    if (driver.rampControl.GetRampMode(rampmode_check)) {
      const char* mode_str = (rampmode_check == tmc51x0::RampMode::HOLD) ? "HOLD" :
                            (rampmode_check == tmc51x0::RampMode::POSITIONING) ? "POSITIONING" :
                            (rampmode_check == tmc51x0::RampMode::VELOCITY_POS) ? "VELOCITY_POS" :
                            (rampmode_check == tmc51x0::RampMode::VELOCITY_NEG) ? "VELOCITY_NEG" : "UNKNOWN";
      ESP_LOGI(TAG, "RAMPMODE: %s", mode_str);
      if (rampmode_check != tmc51x0::RampMode::POSITIONING) {
        ESP_LOGW(TAG, "  WARNING: Not in POSITIONING mode! Current=%s, setting to POSITIONING...", mode_str);
        
        // Clear any stall flags first (they might be forcing HOLD mode)
        driver.diagnostics.ClearStallFlag();
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Now set to POSITIONING
        if (!driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING)) {
          ESP_LOGE(TAG, "  ✗ SetRampMode() call failed!");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay for register write
        
        // Verify it was set
        tmc51x0::RampMode rampmode_verify = tmc51x0::RampMode::HOLD;
        if (driver.rampControl.GetRampMode(rampmode_verify)) {
          if (rampmode_verify == tmc51x0::RampMode::POSITIONING) {
            ESP_LOGI(TAG, "  ✓ RAMPMODE confirmed set to POSITIONING");
          } else {
            ESP_LOGE(TAG, "  ✗ Failed to set RAMPMODE to POSITIONING! Still %s", 
                     (rampmode_verify == tmc51x0::RampMode::HOLD) ? "HOLD" : "UNKNOWN");
            ESP_LOGE(TAG, "  This is a critical error - motor cannot move in HOLD mode!");
            ESP_LOGE(TAG, "  Possible causes: SPI communication issue, chip reset, or register write failure");
          }
        }
      } else {
        ESP_LOGI(TAG, "  ✓ RAMPMODE is POSITIONING (correct)");
      }
    }
  }
  
  // Wait for motion to complete (either target reached or stall detected)
  bool max_stall_detected = false;
  bool max_reached_360 = false;
  uint32_t max_start_time = esp_timer_get_time() / 1000;
  uint32_t timeout_ms = Test::Motion::HOMING_TIMEOUT_MS;
  
  float start_pos_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(start_pos_float, tmc51x0::Unit::Steps)) {
    start_pos_float = 0.0f;
  }
  int32_t start_position = static_cast<int32_t>(start_pos_float);
  int32_t last_position = start_position;
  uint32_t last_position_check_time = max_start_time;
  constexpr int32_t MIN_MOVEMENT_FOR_VALID_STALL = 5000; // Must move at least 5000 steps (~7°) before stall is valid
  constexpr int32_t MIN_MOVEMENT_FOR_STALL_CHECK = 2000; // Don't even check for stalls until motor moves this much
  uint32_t last_sg_result_time = max_start_time;
  uint16_t last_sg_result = sg_result_baseline;
  bool motion_started = false;
  
  while (true) {
    // Check timeout
    uint32_t elapsed = (esp_timer_get_time() / 1000) - max_start_time;
    if (elapsed > timeout_ms) {
      ESP_LOGW(TAG, "Maximum bound search timeout");
      break;
    }
    
    // Check current position to verify motor is actually moving
    float current_pos_float = 0.0f;
    if (!driver.rampControl.GetCurrentPosition(current_pos_float, tmc51x0::Unit::Steps)) {
      current_pos_float = 0.0f;
    }
    int32_t current_pos = static_cast<int32_t>(current_pos_float);
    int32_t position_delta = current_pos - start_position;
    
    // CRITICAL SAFETY CHECK: Never rotate more than 360° from start position
    // This prevents excessive rotation that could damage cables or mechanical systems
    if (position_delta > steps_per_360_deg) {
      ESP_LOGE(TAG, "⚠️ SAFETY LIMIT: Motor rotated %.2f° (exceeds 360° limit) - STOPPING IMMEDIATELY!",
               tmc51x0::StepsToDegrees(position_delta, steps_per_rev));
      ESP_LOGE(TAG, "  Position delta: %ld steps (limit: %ld steps)", position_delta, steps_per_360_deg);
      driver.rampControl.Stop();
      vTaskDelay(pdMS_TO_TICKS(200));
      max_reached_360 = true; // Treat as reached 360° to use default bounds
      ESP_LOGI(TAG, "Using -175° to +175° bounds (safety limit reached)");
      break;
    }
    
    // Check if target reached (no stall, reached 360°)
    if (driver.rampControl.IsTargetReached()) {
      max_reached_360 = true;
      ESP_LOGI(TAG, "Reached +360° target - no stall detected");
      break;
    }
    uint32_t current_time = esp_timer_get_time() / 1000;
    float vactual = 0.0f;
    if (!driver.rampControl.GetCurrentSpeed(vactual, tmc51x0::Unit::Steps)) {
      vactual = 0.0f;
    }
    
    // Detect if motion has started
    if (!motion_started && std::abs(position_delta) > 100) {
      motion_started = true;
      ESP_LOGI(TAG, "Motion started: position=%ld, speed=%.1f steps/s", current_pos, vactual);
    }
    
    // Check for stall stop event unconditionally
    // We must check this even if movement is small, because a stall might have stopped us immediately
    bool stall_event = driver.diagnostics.IsStallDetected();
    
    // ALSO check SG_RESULT directly as fallback (sg_stop might not always set event_stop_sg flag)
    // If SG_RESULT=0 and motor is moving, it's a stall condition
    uint16_t sg_result_check = 0;
    bool sg_result_stall = false;
    if (driver.diagnostics.GetStallGuardResult(sg_result_check)) {
      // SG_RESULT=0 means maximum load/stall, and motor should be moving for it to be valid
      if (sg_result_check == 0 && std::abs(vactual) > 1000.0f) {
        sg_result_stall = true;
        ESP_LOGW(TAG, "⚠️ SG_RESULT=0 detected (stall condition) at V=%.1f steps/s, but event_stop_sg flag not set!", vactual);
      }
    }
    
    // If stall event detected OR SG_RESULT=0, verify it's a real stall
    if (stall_event || sg_result_stall) {
      // Read DRV_STATUS to get SG_RESULT for diagnostics
      uint32_t drv_status_val = 0;
      uint16_t sg_result = 0;
      bool motor_moving = false;
      if (driver.diagnostics.GetStallGuardResult(sg_result)) {
        // Check if motor is moving by checking velocity
        motor_moving = (std::abs(vactual) > 100.0f);
      }
      
      // If detected via SG_RESULT=0 (not flag), manually stop the motor
      if (sg_result_stall && !stall_event) {
        ESP_LOGW(TAG, "⚠️ Stall detected via SG_RESULT=0 (event_stop_sg flag not set) - sg_stop may not be working!");
        ESP_LOGW(TAG, "  Position=%ld, VACTUAL=%.1f steps/s, SG_RESULT=%d", current_pos, vactual, sg_result);
        ESP_LOGW(TAG, "  Manually stopping motor due to SG_RESULT=0...");
        // Manually stop the motor since sg_stop didn't work
        driver.rampControl.Stop();
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait for stop to take effect
        // Re-read velocity to confirm stop
        if (!driver.rampControl.GetCurrentSpeed(vactual, tmc51x0::Unit::Steps)) {
          vactual = 0.0f;
        }
        ESP_LOGI(TAG, "  After manual stop: VACTUAL=%.1f steps/s", vactual);
      }
      
      // Check if motor is actually moving (not already stopped)
      if (std::abs(vactual) < 10.0f && motor_moving) {
        ESP_LOGW(TAG, "Stall event but motor appears stopped (VACTUAL=%.1f) - may be false stall", vactual);
      }
      
      // Check if motor has moved enough to consider stall valid
      if (position_delta < MIN_MOVEMENT_FOR_VALID_STALL) {
        ESP_LOGW(TAG, "⚠️ Stall event detected but motor hasn't moved enough (%ld steps < %d) - IGNORING FALSE STALL", 
                 position_delta, MIN_MOVEMENT_FOR_VALID_STALL);
        ESP_LOGW(TAG, "  Diagnostics: SG_RESULT=%d (baseline=%d), VACTUAL=%.1f steps/s, position=%ld", 
                 sg_result, sg_result_baseline, vactual, current_pos);
        ESP_LOGW(TAG, "  SGT threshold=%d (lower=more sensitive) - consider increasing if false stalls persist", 
                 Test::StallGuard::SGT_HOMING);
        ESP_LOGW(TAG, "  Clearing stall flag and continuing search...");
        
        // Clear the stall event flag and continue
        if (!driver.diagnostics.ClearStallFlag()) {
          ESP_LOGE(TAG, "Failed to clear stall flag!");
        }
        
        // CRITICAL: Stall events can force RAMPMODE to HOLD - ensure it's back to POSITIONING
        tmc51x0::RampMode rampmode_check = tmc51x0::RampMode::HOLD;
        if (driver.rampControl.GetRampMode(rampmode_check)) {
          if (rampmode_check != tmc51x0::RampMode::POSITIONING) {
            ESP_LOGW(TAG, "  Stall event forced RAMPMODE to HOLD - resetting to POSITIONING...");
            driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
            vTaskDelay(pdMS_TO_TICKS(50));
          }
        }
        
        // Verify stall flag cleared
        vTaskDelay(pdMS_TO_TICKS(50));
        uint32_t ramp_stat_after = 0;
        if (driver.diagnostics.GetRampStatusRegister(ramp_stat_after)) {
          tmc51x0::RAMP_STAT_Register after{};
          after.value = ramp_stat_after;
          if (after.bits.event_stop_sg) {
            ESP_LOGE(TAG, "CRITICAL: Stall flag still set after clear - possible race condition!");
          } else {
            ESP_LOGI(TAG, "✓ Stall flag cleared successfully");
          }
        }
        
        continue; // Continue searching, don't break
      }
      
      // Motor has moved enough - this is likely a real stall
      max_stall_detected = true;
      ESP_LOGI(TAG, "✓ Stall detected during maximum bound search!");
      ESP_LOGI(TAG, "  Position moved: %ld steps from start (threshold: %d)", 
               position_delta, MIN_MOVEMENT_FOR_VALID_STALL);
      ESP_LOGI(TAG, "  SG_RESULT=%d (baseline=%d, lower=more load, 0=highest load)", 
               sg_result, sg_result_baseline);
      ESP_LOGI(TAG, "  VACTUAL=%.1f steps/s, elapsed=%lu ms", vactual, elapsed);
      break;
    }
    
    // Monitor SG_RESULT periodically for diagnostics
    if (current_time - last_sg_result_time >= 200) {
      uint16_t current_sg = 0;
      if (driver.diagnostics.GetStallGuardResult(current_sg)) {
        if (current_sg != last_sg_result) {
          ESP_LOGI(TAG, "  SG_RESULT changed: %d -> %d (position=%ld, speed=%.1f)", 
                   last_sg_result, current_sg, current_pos, vactual);
          last_sg_result = current_sg;
        }
      }
      last_sg_result_time = current_time;
    }
    
    // Log position progress periodically (every 500ms)
    if (current_time - last_position_check_time >= 500) {
      int32_t position_change = current_pos - last_position;
      ESP_LOGI(TAG, "  Progress: position=%ld (+%ld from start, +%ld since last), speed=%.1f steps/s, elapsed=%lu ms",
               current_pos, position_delta, position_change, vactual, elapsed);
      
      // Check if motor is stuck (position not changing but should be moving)
      if (motion_started && std::abs(position_change) < 50 && std::abs(vactual) < 100.0f && elapsed > 1000) {
        ESP_LOGW(TAG, "  ⚠️ Motor appears stuck: position change=%ld steps, speed=%.1f steps/s", 
                 position_change, vactual);
      }
      
      last_position_check_time = current_time;
      last_position = current_pos;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms
  }
  
  float max_pos_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(max_pos_float, tmc51x0::Unit::Steps)) {
    max_pos_float = 0.0f;
  }
  int32_t max_position = static_cast<int32_t>(max_pos_float);
  
  if (max_stall_detected) {
    ESP_LOGI(TAG, "Maximum bound found at stall: %ld steps", max_position);
    
    // Back off with 5° offset
    driver.rampControl.Stop();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Backing off 5° (%ld steps) from maximum stall...", offset_steps);
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(static_cast<float>(max_position - offset_steps), tmc51x0::Unit::Steps);
    driver.rampControl.SetMaxSpeed(search_speed / 2.0f);
    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    float max_pos_float = 0.0f;
    if (!driver.rampControl.GetCurrentPosition(max_pos_float, tmc51x0::Unit::Steps)) {
      max_pos_float = 0.0f;
    }
    max_position = static_cast<int32_t>(max_pos_float);
  } else if (max_reached_360) {
    ESP_LOGI(TAG, "No stall at +360° - will use -175° to +175° bounds");
    // Will handle this after checking minimum bound
  }
  
  // ============================================================
  // Find minimum bound: Command to -360° and detect stall
  // ============================================================
  ESP_LOGI(TAG, "Finding minimum bound: Commanding to -360° (%ld steps)...", -steps_per_360_deg);
  
  // Clear stall flag before starting
  // Clear any existing stall flags before starting
  driver.diagnostics.ClearStallFlag();
  
  driver.rampControl.SetTargetPosition(static_cast<float>(-steps_per_360_deg), tmc51x0::Unit::Steps);
  
  // Wait for motion to complete (either target reached or stall detected)
  bool min_stall_detected = false;
  bool min_reached_360 = false;
  uint32_t min_start_time = esp_timer_get_time() / 1000;
  float min_start_pos_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(min_start_pos_float, tmc51x0::Unit::Steps)) {
    min_start_pos_float = 0.0f;
  }
  int32_t min_start_position = static_cast<int32_t>(min_start_pos_float);
  int32_t min_last_position = min_start_position;
  uint32_t min_last_position_check_time = min_start_time;
  uint32_t min_last_sg_result_time = min_start_time;
  uint16_t min_last_sg_result = sg_result_baseline;
  bool min_motion_started = false;
  
  while (true) {
    // Check timeout
    uint32_t elapsed = (esp_timer_get_time() / 1000) - min_start_time;
    if (elapsed > timeout_ms) {
      ESP_LOGW(TAG, "Minimum bound search timeout");
      break;
    }
    
    // Check current position to verify motor is actually moving
    float current_pos_float = 0.0f;
    if (!driver.rampControl.GetCurrentPosition(current_pos_float, tmc51x0::Unit::Steps)) {
      current_pos_float = 0.0f;
    }
    int32_t current_pos = static_cast<int32_t>(current_pos_float);
    int32_t position_delta = std::abs(current_pos - min_start_position);
    
    // CRITICAL SAFETY CHECK: Never rotate more than 360° from start position
    // This prevents excessive rotation that could damage cables or mechanical systems
    if (position_delta > steps_per_360_deg) {
      ESP_LOGE(TAG, "⚠️ SAFETY LIMIT: Motor rotated %.2f° (exceeds 360° limit) - STOPPING IMMEDIATELY!",
               tmc51x0::StepsToDegrees(position_delta, steps_per_rev));
      ESP_LOGE(TAG, "  Position delta: %ld steps (limit: %ld steps)", position_delta, steps_per_360_deg);
      driver.rampControl.Stop();
      vTaskDelay(pdMS_TO_TICKS(200));
      min_reached_360 = true; // Treat as reached 360° to use default bounds
      ESP_LOGI(TAG, "Using -175° to +175° bounds (safety limit reached)");
      break;
    }
    
    // Check if target reached (no stall, reached -360°)
    if (driver.rampControl.IsTargetReached()) {
      min_reached_360 = true;
      ESP_LOGI(TAG, "Reached -360° target - no stall detected");
      break;
    }
    uint32_t current_time = esp_timer_get_time() / 1000;
    float vactual = 0.0f;
    if (!driver.rampControl.GetCurrentSpeed(vactual, tmc51x0::Unit::Steps)) {
      vactual = 0.0f;
    }
    
    // Detect if motion has started
    if (!min_motion_started && position_delta > 100) {
      min_motion_started = true;
      ESP_LOGI(TAG, "Motion started: position=%ld, speed=%.1f steps/s", current_pos, vactual);
    }
    
    // Check for stall stop event unconditionally
    bool stall_event = driver.diagnostics.IsStallDetected();
    
    // If stall event detected, verify it's a real stall
    if (stall_event) {
      // Read SG_RESULT for diagnostics
      uint16_t sg_result = 0;
      bool motor_moving = false;
      if (driver.diagnostics.GetStallGuardResult(sg_result)) {
        // Check if motor is moving by checking velocity
        motor_moving = (std::abs(vactual) > 100.0f);
      }
      
      // Check if motor is actually moving
      if (std::abs(vactual) < 10.0f && motor_moving) {
        ESP_LOGW(TAG, "Stall event but motor appears stopped (VACTUAL=%.1f) - may be false stall", vactual);
      }
      
      // Check if motor has moved enough to consider stall valid
      if (position_delta < MIN_MOVEMENT_FOR_VALID_STALL) {
        ESP_LOGW(TAG, "⚠️ Stall event detected but motor hasn't moved enough (%ld steps < %d) - IGNORING FALSE STALL", 
                 position_delta, MIN_MOVEMENT_FOR_VALID_STALL);
        ESP_LOGW(TAG, "  Diagnostics: SG_RESULT=%d (baseline=%d), VACTUAL=%.1f steps/s, position=%ld", 
                 sg_result, sg_result_baseline, vactual, current_pos);
        ESP_LOGW(TAG, "  SGT threshold=%d (lower=more sensitive) - consider increasing if false stalls persist", 
                 Test::StallGuard::SGT_HOMING);
        ESP_LOGW(TAG, "  Clearing stall flag and continuing search...");
        
        // Clear the stall event flag and continue
        constexpr uint32_t CLEAR_STALL_BIT = 0x01; // event_stop_sg bit
        if (!driver.diagnostics.ClearRampStatus(CLEAR_STALL_BIT)) {
          ESP_LOGE(TAG, "Failed to clear stall flag!");
        }
        
        // CRITICAL: Stall events can force RAMPMODE to HOLD - ensure it's back to POSITIONING
        tmc51x0::RampMode rampmode_check = tmc51x0::RampMode::HOLD;
        if (driver.rampControl.GetRampMode(rampmode_check)) {
          if (rampmode_check != tmc51x0::RampMode::POSITIONING) {
            ESP_LOGW(TAG, "  Stall event forced RAMPMODE to HOLD - resetting to POSITIONING...");
            driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
            vTaskDelay(pdMS_TO_TICKS(50));
          }
        }
        
        // Verify stall flag cleared
        vTaskDelay(pdMS_TO_TICKS(50));
        uint32_t ramp_stat_after = 0;
        if (driver.diagnostics.GetRampStatusRegister(ramp_stat_after)) {
          tmc51x0::RAMP_STAT_Register after{};
          after.value = ramp_stat_after;
          if (after.bits.event_stop_sg) {
            ESP_LOGE(TAG, "CRITICAL: Stall flag still set after clear - possible race condition!");
          } else {
            ESP_LOGI(TAG, "✓ Stall flag cleared successfully");
          }
        }
        
        continue;
      }
      
      // Motor has moved enough - this is likely a real stall
      min_stall_detected = true;
      ESP_LOGI(TAG, "✓ Stall detected during minimum bound search!");
      ESP_LOGI(TAG, "  Position moved: %ld steps from start (threshold: %d)", 
               position_delta, MIN_MOVEMENT_FOR_VALID_STALL);
      ESP_LOGI(TAG, "  SG_RESULT=%d (baseline=%d, lower=more load, 0=highest load)", 
               sg_result, sg_result_baseline);
      ESP_LOGI(TAG, "  VACTUAL=%.1f steps/s, elapsed=%lu ms", vactual, elapsed);
      break;
    }
    
    // Monitor SG_RESULT periodically
    if (current_time - min_last_sg_result_time >= 200) {
      uint16_t current_sg = 0;
      if (driver.diagnostics.GetStallGuardResult(current_sg)) {
        if (current_sg != min_last_sg_result) {
          ESP_LOGI(TAG, "  SG_RESULT changed: %d -> %d (position=%ld, speed=%.1f)", 
                   min_last_sg_result, current_sg, current_pos, vactual);
          min_last_sg_result = current_sg;
        }
      }
      min_last_sg_result_time = current_time;
    }
    
    // Log position progress periodically (every 500ms)
    if (current_time - min_last_position_check_time >= 500) {
      int32_t position_change = current_pos - min_last_position;
      ESP_LOGI(TAG, "  Progress: position=%ld (%ld from start, %ld since last), speed=%.1f steps/s, elapsed=%lu ms",
               current_pos, current_pos - min_start_position, position_change, vactual, elapsed);
      
      // Check if motor is stuck
      if (min_motion_started && std::abs(position_change) < 50 && std::abs(vactual) < 100.0f && elapsed > 1000) {
        ESP_LOGW(TAG, "  ⚠️ Motor appears stuck: position change=%ld steps, speed=%.1f steps/s", 
                 position_change, vactual);
      }
      
      min_last_position_check_time = current_time;
      min_last_position = current_pos;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms
  }
  
  float min_pos_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(min_pos_float, tmc51x0::Unit::Steps)) {
    min_pos_float = 0.0f;
  }
  int32_t min_position = static_cast<int32_t>(min_pos_float);
  
  if (min_stall_detected) {
    ESP_LOGI(TAG, "Minimum bound found at stall: %ld steps", min_position);
    
    // Back off with 5° offset
    driver.rampControl.Stop();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Backing off 5° (%ld steps) from minimum stall...", offset_steps);
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(static_cast<float>(min_position + offset_steps), tmc51x0::Unit::Steps);
    driver.rampControl.SetMaxSpeed(search_speed / 2.0f);
    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    float min_pos_float = 0.0f;
    if (!driver.rampControl.GetCurrentPosition(min_pos_float, tmc51x0::Unit::Steps)) {
      min_pos_float = 0.0f;
    }
    min_position = static_cast<int32_t>(min_pos_float);
  } else if (min_reached_360) {
    ESP_LOGI(TAG, "No stall at -360° - will use -175° to +175° bounds");
  }
  
  // ============================================================
  // Handle results and set up bounds
  // ============================================================
  
  // Disable StallGuard2 stop for normal operation
  if (driver.diagnostics.EnableStopOnStall(false)) {
    ESP_LOGI(TAG, "✓ StallGuard2 stop disabled for normal operation");
  }
  
  bool stall_detected_min = min_stall_detected;
  bool stall_detected_max = max_stall_detected;
  bool reached_360 = (min_reached_360 && max_reached_360); // Both reached 360° = no stalls
  
  if (reached_360) {
    // No stalls detected - mark current position as 0 and use -175° to +175° bounds
    ESP_LOGI(TAG, "No stalls detected - marking current position as 0, using -175° to +175° bounds");
    
    driver.rampControl.Stop();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    
    // Wait for motor to stop
    uint32_t stop_wait_start = esp_timer_get_time() / 1000;
    while (true) {
      float vactual = 0.0f;
      if (!driver.rampControl.GetCurrentSpeed(vactual, tmc51x0::Unit::Steps)) {
        vactual = 0.0f;
      }
      if (std::abs(vactual) < 10.0f) break;
      if ((esp_timer_get_time() / 1000) - stop_wait_start > 2000) break;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Reset position to 0
    driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
    
    // Use -175° to +175° bounds
    // Using utility function to ensure proper microstep handling
    float bounds_deg = 175.0f;
    int32_t bounds_steps = tmc51x0::DegreesToSteps(bounds_deg, steps_per_rev);
    max_position = bounds_steps;
    min_position = -bounds_steps;
    
    ESP_LOGI(TAG, "Position reset to 0, bounds set to ±%.1f° (%ld steps)", bounds_deg, bounds_steps);
    
    // Move to center (0) if not already there
    float current_pos_float = 0.0f;
    if (!driver.rampControl.GetCurrentPosition(current_pos_float, tmc51x0::Unit::Steps)) {
      current_pos_float = 0.0f;
    }
    int32_t current_pos = static_cast<int32_t>(current_pos_float);
    if (std::abs(current_pos) > 100) {
      ESP_LOGI(TAG, "Moving to center position (0)...");
      driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
      driver.rampControl.SetTargetPosition(0.0f, tmc51x0::Unit::Steps);
      driver.rampControl.SetMaxSpeed(1000.0f);
      driver.rampControl.SetAcceleration(2000.0f);
      driver.rampControl.SetDeceleration(2000.0f);
      while (!driver.rampControl.IsTargetReached()) {
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
      ESP_LOGI(TAG, "Arrived at center position (0)");
    }
  } else {
    // At least one stall detected - move to center between bounds
    int32_t center_position = (min_position + max_position) / 2;
    ESP_LOGI(TAG, "Moving to center position: %ld steps (between %ld and %ld)", 
             center_position, min_position, max_position);
    
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(static_cast<float>(center_position), tmc51x0::Unit::Steps);
    driver.rampControl.SetMaxSpeed(1000.0f);
    driver.rampControl.SetAcceleration(2000.0f);
    driver.rampControl.SetDeceleration(2000.0f);
    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Reset position to 0 at center
    driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
    
    // Adjust bounds relative to new center
    min_position = min_position - center_position;
    max_position = max_position - center_position;
    
    ESP_LOGI(TAG, "Home position set to 0 (center of bounds)");
    ESP_LOGI(TAG, "Adjusted bounds: min=%ld, max=%ld steps", min_position, max_position);
  }


  // ============================================================
  // STEP 2: Set up global bounds and home
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║              STEP 2: Setting Global Bounds and Home                        ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Determine if we're bounded (either by stall detection or 360° limit)
  bool bounded = (stall_detected_min && stall_detected_max) || reached_360;
  float current_pos_float = 0.0f;
  if (!driver.rampControl.GetCurrentPosition(current_pos_float, tmc51x0::Unit::Steps)) {
    current_pos_float = 0.0f;
  }
  int32_t current_pos = static_cast<int32_t>(current_pos_float);

  FatigueTestMotion motion(&driver);
  motion.ConfigureMotor(steps_per_rev, AngleUnit::DEGREES);

  if (!bounded) {
    ESP_LOGW(TAG, "=== UNBOUNDED MODE ===");
    motion.SetUnbounded(current_pos, 10000);
  } else {
    ESP_LOGI(TAG, "=== BOUNDED MODE ===");

    if (reached_360) {
      // Special case: No stall detected, using -175° to +175° bounds
      // Position is already at 0 (we reset it at 360° point and moved to center)
      ESP_LOGI(TAG, "Using -175° to +175° bounds (no stall detected at 360°)");
      
      // Set global bounds directly (already relative to 0)
      motion.SetGlobalBounds(min_position, max_position);
      
      float min_deg, max_deg;
      motion.GetGlobalBoundsDegrees(min_deg, max_deg);
      ESP_LOGI(TAG, "Global bounds: min=%.2f°, max=%.2f° from center", min_deg, max_deg);
    } else {
      // Normal case: Stall detected on both ends
    // Set middle as home
    int32_t middle_position = (min_position + max_position) / 2;
    ESP_LOGI(TAG, "Moving to middle position: %d steps", middle_position);

    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(static_cast<float>(middle_position), tmc51x0::Unit::Steps);
    driver.rampControl.SetMaxSpeed(1000.0F);
    driver.rampControl.SetAcceleration(2000.0F);

    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);
    ESP_LOGI(TAG, "Home position set to 0 (middle of bounds)");

    // Set global bounds relative to new home
    int32_t global_min = min_position - middle_position;
    int32_t global_max = max_position - middle_position;
    motion.SetGlobalBounds(global_min, global_max);

    float min_deg, max_deg;
    motion.GetGlobalBoundsDegrees(min_deg, max_deg);
    ESP_LOGI(TAG, "Global bounds: min=%.2f°, max=%.2f° from center", min_deg, max_deg);
    }
  }

  // ============================================================
  // STEP 3: Set default local bounds and motion parameters
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║         STEP 3: Configuring Default Motion Parameters                        ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Set default local bounds for oscillation (will be clipped to global bounds if needed)
  float amplitude = Test::Motion::FATIGUE_AMPLITUDE_DEG;
  motion.SetLocalBoundsFromCenterDegrees(-amplitude, amplitude);

  // Configure default sinuous motion parameters
  motion.SetFrequency(Test::Motion::FATIGUE_FREQ_HZ);

  // Set default dwell times
  uint32_t dwell = Test::Motion::DWELL_MS;
  motion.SetDwellTimes(dwell, dwell);

  // Set default target cycle count (0 = infinite)
  motion.SetTargetCycles(0); // Infinite by default, can be changed via UART

  ESP_LOGI(TAG, "Default configuration:");
  FatigueTestMotion::Status status = motion.GetStatus();
  ESP_LOGI(TAG, "  Local bounds: %.2f° to %.2f° from center", 
           status.min_degrees_from_center, status.max_degrees_from_center);
  ESP_LOGI(TAG, "  Frequency: %.2f Hz", status.frequency_hz);
  ESP_LOGI(TAG, "  Target cycles: %lu %s", status.target_cycles, 
           status.target_cycles == 0 ? "(infinite)" : "");
  ESP_LOGI(TAG, "  Bounded: %s", status.bounded ? "Yes" : "No");

  // ============================================================
  // STEP 4: Initialize UART command parser
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║              STEP 4: Initializing UART Command Interface                     ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  UartCommandParser parser(UART_NUM_0);

  // Register commands (modular structure for easy extension)
  parser.RegisterCommand({"-f", "--freq", "Set frequency in Hz (0.0-10.0)", 1, 1}, HandleFrequency);
  parser.RegisterCommand({"-d", "--dwell", "Set dwell times in ms (min max)", 2, 2}, HandleDwell);
  parser.RegisterCommand({"-b", "--bounds", "Set angle bounds from center in degrees (min max)", 2, 2}, HandleBounds);
  parser.RegisterCommand({"-c", "--cycles", "Set target cycle count (0 = infinite)", 1, 1}, HandleCycles);
  parser.RegisterCommand({"-a", "--action", "Action: start, stop, or reset", 1, 1}, HandleAction);
  parser.RegisterCommand({"-s", "--status", "Show current status", 0, 0}, HandleStatus);
  // Note: Help command (-h, --help) is handled specially in ProcessCommand()

  ESP_LOGI(TAG, "UART command interface ready on UART_NUM_0 (USB serial)");
  ESP_LOGI(TAG, "Type '--help' or '-h' for command list");

  // ============================================================
  // STEP 5: Create tasks
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                    STEP 5: Starting Control Tasks                           ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Create motion control task
  xTaskCreate(motion_control_task, "motion_control", 8192, &motion, 5, nullptr);
  ESP_LOGI(TAG, "Motion control task created");

  // Create UART command task
  struct TaskParams {
    UartCommandParser* parser;
    FatigueTestMotion* motion;
  } task_params = {&parser, &motion};
  
  xTaskCreate(uart_command_task, "uart_command", 4096, &task_params, 3, nullptr);
  ESP_LOGI(TAG, "UART command task created");

  // ============================================================
  // STEP 6: Main loop - status updates
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                    System Ready - Use UART Commands to Control              ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  ESP_LOGI(TAG, "Example commands:");
  ESP_LOGI(TAG, "  -a start          : Start motion");
  ESP_LOGI(TAG, "  -f 0.5            : Set frequency to 0.5 Hz");
  ESP_LOGI(TAG, "  -b -60 60         : Set bounds to ±60 degrees");
  ESP_LOGI(TAG, "  -s                : Show status");
  ESP_LOGI(TAG, "");

  uint32_t last_log_time = 0;

  while (true) {
    // Periodic status logging (UART commands are handled by uart_command_task)
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_log_time > 10000) { // Log every 10 seconds
      float pos_float = 0.0f;
      if (!driver.rampControl.GetCurrentPosition(pos_float, tmc51x0::Unit::Steps)) {
        pos_float = 0.0f;
      }
      int32_t pos = static_cast<int32_t>(pos_float);
      float speed = 0.0f;
      if (!driver.rampControl.GetCurrentSpeed(speed, tmc51x0::Unit::Steps)) {
        speed = 0.0f;
      }
      float pos_deg = tmc51x0::StepsToDegrees(pos, steps_per_rev);
      FatigueTestMotion::Status status = motion.GetStatus();
      ESP_LOGI(TAG, "Position: %d steps (%.2f°), Speed: %.1f steps/s, Cycles: %lu/%lu %s", 
               pos, pos_deg, speed, status.current_cycles,
               status.target_cycles == 0 ? 0xFFFFFFFF : status.target_cycles, 
               status.running ? "(running)" : "(stopped)");
      last_log_time = current_time;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
