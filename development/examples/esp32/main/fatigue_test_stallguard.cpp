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

// Include bounds finder from ESP-NOW test unit (shared implementation)
#include "fatigue_test_espnow/test_unit/bounds_finder.hpp"
#include <memory>

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

  // Global bounds (hardware limits found during initialization) - all in degrees
  float global_min_bound_; // Global minimum position in degrees
  float global_max_bound_; // Global maximum position in degrees

  // Local bounds (oscillation range, clipped to global bounds) - all in degrees
  float local_min_bound_; // Local minimum for oscillation in degrees
  float local_max_bound_; // Local maximum for oscillation in degrees

  float home_position_; // Home position (center) in degrees
  float amplitude_;       // Amplitude in degrees
  float frequency_hz_;    // Frequency in Hz
  bool running_;
  uint32_t start_time_us_;
  float phase_offset_;
  bool bounded_; // Whether global bounds were found

  // Dwell times (can be set to 0 to disable)
  uint32_t dwell_at_min_ms_;    // Dwell time at minimum bound
  uint32_t dwell_at_max_ms_;    // Dwell time at maximum bound

  // Cycle tracking
  uint32_t target_cycles_;       // Target number of cycles (0 = infinite)
  uint32_t current_cycles_;      // Current cycle count
  bool cycle_complete_;          // Whether target cycles reached
  bool last_was_negative_;       // Last position relative to center (for cycle counting)
  bool cycle_started_;           // Whether a cycle has started (left center)
  float last_target_relative_; // Last target position relative to center (in degrees)

  // State machine
  // Note: Sinusoidal motion uses the same states - it's a motion mode, not a separate state
  enum class MotionState { MOVING_TO_MIN, MOVING_TO_MAX, DWELL_AT_MIN, DWELL_AT_MAX, STOPPED };
  MotionState state_;
  uint32_t dwell_start_time_ms_;
  
  // Motion mode flag (true = sinusoidal, false = ramp-based)
  bool sinusoidal_mode_;

  // Computed trajectory parameters (in physical units)
  float calculated_vmax_rpm_;      // Maximum velocity in RPM
  float calculated_amax_rev_s2_;    // Maximum acceleration in rev/s²
  float estimated_frequency_hz_;

  // Thread safety
  mutable Esp32TmcMutex mutex_;

  /**
   * @brief Recalculate trajectory parameters based on frequency, bounds, and dwell
   * All calculations in degrees, output in RPM and rev/s²
   */
  void RecalculateTrajectory() noexcept {
    // NOTE: Must be called with mutex locked
    
    // Calculate total travel distance (one way) in degrees
    float distance_deg = std::abs(local_max_bound_ - local_min_bound_);
    if (distance_deg < 0.1f || frequency_hz_ <= 0.0001f) {
      calculated_vmax_rpm_ = 30.0f; // Default safe fallback in RPM
      calculated_amax_rev_s2_ = 10.0f; // Default in rev/s²
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
    
    // Calculate velocity and acceleration in physical units
    // Distance in degrees, convert to revolutions: distance_deg / 360.0f
    // Velocity in rev/s: (distance_deg / 360.0f) / leg_time_s
    // Velocity in RPM: ((distance_deg / 360.0f) / leg_time_s) * 60.0f
    // For trapezoidal profile (1/3 accel, 1/3 const, 1/3 decel):
    // Average velocity = 2/3 * max velocity
    // So: distance_rev = (2/3 * vmax_rev_s) * leg_time_s
    // vmax_rev_s = (3/2) * distance_rev / leg_time_s
    float distance_rev = distance_deg / 360.0f;
    float vmax_rev_s = (1.5f * distance_rev) / leg_time_s;
    calculated_vmax_rpm_ = vmax_rev_s * 60.0f;
    
    // Acceleration: reach max velocity in leg_time_s / 3
    // amax_rev_s2 = vmax_rev_s / (leg_time_s / 3.0f)
    calculated_amax_rev_s2_ = vmax_rev_s / (leg_time_s / 3.0f);
    
    // Clamp to reasonable limits
    if (calculated_vmax_rpm_ > 1000.0f) calculated_vmax_rpm_ = 1000.0f; // Cap velocity
    if (calculated_amax_rev_s2_ > 100.0f) calculated_amax_rev_s2_ = 100.0f; // Cap acceleration
    
    // Recalculate actual expected frequency
    estimated_frequency_hz_ = 1.0f / (2.0f * leg_time_s + total_dwell_s);
    
    ESP_LOGI(TAG, "Trajectory Recalculated: Dist=%.2f degrees, LegTime=%.3fs", distance_deg, leg_time_s);
    ESP_LOGI(TAG, "  Target Freq=%.2fHz, Est Freq=%.2fHz", frequency_hz_, estimated_frequency_hz_);
    ESP_LOGI(TAG, "  VMAX=%.1f RPM, AMAX=%.2f rev/s²", calculated_vmax_rpm_, calculated_amax_rev_s2_);
  }

public:
  FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver) noexcept
      : driver_(driver), global_min_bound_(0.0f), global_max_bound_(0.0f), local_min_bound_(0.0f), local_max_bound_(0.0f),
        home_position_(0.0f), amplitude_(180.0f), frequency_hz_(0.5f), running_(false), start_time_us_(0),
        phase_offset_(0.0f), bounded_(false), dwell_at_min_ms_(0),
        dwell_at_max_ms_(0), target_cycles_(0), current_cycles_(0), cycle_complete_(false),
        last_was_negative_(false), cycle_started_(false), last_target_relative_(0.0f), state_(MotionState::STOPPED),
        dwell_start_time_ms_(0), sinusoidal_mode_(false), calculated_vmax_rpm_(30.0f), calculated_amax_rev_s2_(10.0f), estimated_frequency_hz_(0.0f) {
    // Mutex is automatically created by Esp32TmcMutex constructor
    // Driver handles all unit conversions internally - no need for steps_per_rev
  }

  ~FatigueTestMotion() noexcept = default; // Mutex automatically destroyed by Esp32TmcMutex destructor

  /**
   * @brief Set global bounds (hardware limits found during initialization) - in degrees
   * @param min_bound_deg Global minimum position in degrees
   * @param max_bound_deg Global maximum position in degrees
   */
  void SetGlobalBounds(float min_bound_deg, float max_bound_deg) noexcept {
    {
      TmcMutexGuard guard(mutex_);
      global_min_bound_ = min_bound_deg;
      global_max_bound_ = max_bound_deg;
      bounded_ = true;
    }
    ESP_LOGI(TAG, "Global bounds set: min=%.2f°, max=%.2f°", global_min_bound_, global_max_bound_);

    // Clip local bounds to global bounds if they exist
    {
      TmcMutexGuard guard(mutex_);
      if (std::abs(local_min_bound_) > 0.01f || std::abs(local_max_bound_) > 0.01f) {
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
    TmcMutexGuard guard(mutex_);
    min_degrees = global_min_bound_;
    max_degrees = global_max_bound_;
  }

  /**
   * @brief Set unbounded mode (no mechanical stops found)
   * Uses current position as home and sets reasonable default global bounds
   * @param current_position_deg Current position in degrees
   * @param default_range_deg Default range in degrees (default: 350°)
   */
  void SetUnbounded(float current_position_deg, float default_range_deg = 350.0f) noexcept {
    {
      TmcMutexGuard guard(mutex_);
      bounded_ = false;
      home_position_ = current_position_deg;
      global_min_bound_ = current_position_deg - default_range_deg / 2.0f;
      global_max_bound_ = current_position_deg + default_range_deg / 2.0f;
    }
    driver_->rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
    ESP_LOGW(TAG, "Unbounded mode: No mechanical stops found");
    ESP_LOGI(TAG, "Using current position as home: %.2f degrees", current_position_deg);
    ESP_LOGI(TAG, "Default global range: [%.2f°, %.2f°] (%.2f° total)", 
             global_min_bound_, global_max_bound_, default_range_deg);
  }

  /**
   * @brief Set local bounds in degrees from center (thread-safe)
   * @param min_degrees_from_center Minimum angle from center (negative)
   * @param max_degrees_from_center Maximum angle from center (positive)
   */
  bool SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center) noexcept {
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

    ESP_LOGI(TAG, "Local bounds set: min=%.2f°, max=%.2f° from center", min_deg, max_deg);
    
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
    TmcMutexGuard guard(mutex_);
    min_degrees = local_min_bound_;
    max_degrees = local_max_bound_;
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
    float min_pos_deg, max_pos_deg;
    
    {
      TmcMutexGuard guard(mutex_);
      if (std::abs(local_min_bound_) < 0.01f && std::abs(local_max_bound_) < 0.01f) {
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
      // Use calculated values directly (already in RPM and rev/s²)
      driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
      driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
      driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec); // Symmetric acceleration/deceleration
      // Ensure VSTART/VSTOP/VZERO are reasonable (in RPM)
      float vstart_rpm = 30.0f;  // ~1000 steps/s for 200 steps/rev
      float vstop_rpm = 3.0f;   // ~100 steps/s for 200 steps/rev
      driver_->rampControl.SetRampSpeeds(vstart_rpm, vstop_rpm, 0.0f, tmc51x0::Unit::RPM);

      running_ = true;
      start_time_us_ = esp_timer_get_time();
      
      // Determine initial state based on current position
      auto current_pos_deg_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
      float current_pos_deg = current_pos_deg_result.IsOk() ? current_pos_deg_result.Value() : 0.0f;
      min_pos_deg = local_min_bound_;
      max_pos_deg = local_max_bound_;
      
      // Find closest bound or determine direction
      float dist_to_min = fabsf(current_pos_deg - min_pos_deg);
      float dist_to_max = fabsf(current_pos_deg - max_pos_deg);
      
      // Default to moving towards min unless we're already there
      if (dist_to_min < 1.0f) { // 1 degree threshold
        state_ = MotionState::MOVING_TO_MAX;
        driver_->rampControl.SetTargetPosition(max_pos_deg, tmc51x0::Unit::Deg);
      } else {
        state_ = MotionState::MOVING_TO_MIN;
        driver_->rampControl.SetTargetPosition(min_pos_deg, tmc51x0::Unit::Deg);
      }

      current_cycles = current_cycles_;
      target_cycles = target_cycles_;
    }

    ESP_LOGI(TAG, "Starting fatigue test (cycles: %lu/%lu)", current_cycles,
             target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
    ESP_LOGI(TAG, "  Motion: Positioning mode, VMAX=%.1f RPM, AMAX=%.2f rev/s²", 
             calculated_vmax_rpm_, calculated_amax_rev_s2_);
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
    float min_bound_deg, max_bound_deg;
    
    {
      TmcMutexGuard guard(mutex_);
      current_state = state_;
      dwell_min = dwell_at_min_ms_;
      dwell_max = dwell_at_max_ms_;
      dwell_start = dwell_start_time_ms_;
      min_bound_deg = local_min_bound_;
      max_bound_deg = local_max_bound_;
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
          // Apply potentially new speed/accel for next leg (already in physical units)
          driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
          driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
          driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
          driver_->rampControl.SetTargetPosition(min_bound_deg, tmc51x0::Unit::Deg);
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
            // Apply potentially new speed/accel for next leg (already in physical units)
            driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
            driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
            driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
            driver_->rampControl.SetTargetPosition(max_bound_deg, tmc51x0::Unit::Deg);
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
        driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
        driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetTargetPosition(max_bound_deg, tmc51x0::Unit::Deg);
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
        driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
        driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetDeceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
        driver_->rampControl.SetTargetPosition(min_bound_deg, tmc51x0::Unit::Deg);
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

    float old_min = local_min_bound_;
    float old_max = local_max_bound_;

    // Clip local bounds to global bounds (all in degrees)
    local_min_bound_ = std::max(local_min_bound_, global_min_bound_);
    local_max_bound_ = std::min(local_max_bound_, global_max_bound_);

    // Update home position
    home_position_ = (local_min_bound_ + local_max_bound_) / 2.0f;
    amplitude_ = (local_max_bound_ - local_min_bound_) / 2.0f;

    if (fabsf(old_min - local_min_bound_) > 0.01f || fabsf(old_max - local_max_bound_) > 0.01f) {
      ESP_LOGW(TAG, "Local bounds clipped to global bounds");
      ESP_LOGI(TAG, "Clipped local bounds: min=%.2f°, max=%.2f°", local_min_bound_, local_max_bound_);
    }
  }

  /**
   * @brief Update sinuous motion target position
   */
  void UpdateSinuousMotion() noexcept {
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
    float target_deg = home + (amp * sin_value);

    // Get current position relative to center for cycle counting
    auto current_pos_deg_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    float current_pos_deg = current_pos_deg_result.IsOk() ? current_pos_deg_result.Value() : 0.0f;
    float target_relative_deg = target_deg - home;

    // Cycle counting: one cycle = center → min → max → center (or center → max → min → center)
    // Count cycles when crossing through center (0 crossing point)
      // Check if we're crossing through center (sign change of target position)
      bool currently_negative = (target_relative_deg < 0);
      bool last_was_neg = (last_target_rel < 0);
      bool crossing_center =
          (last_was_neg != currently_negative) && (fabsf(target_relative_deg) < 1.0f) && (fabsf(last_target_rel) < 1.0f);

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
      } else if (!cycle_started && fabsf(target_relative_deg) > 1.0f) {
        // We've left center, cycle has started
        TmcMutexGuard guard(mutex_);
        cycle_started_ = true;
        last_was_negative_ = currently_negative;
      }

      // Update tracking
      {
        TmcMutexGuard guard(mutex_);
        last_target_relative_ = target_relative_deg; // Store in degrees
        if (fabsf(target_relative_deg) > 0.5f) { // Only update if significantly away from center (0.5 degrees)
          last_was_negative_ = currently_negative;
        }
    }

    // Clamp to local bounds and handle dwell states (all in degrees)
    if (target_deg <= local_min) {
      target_deg = local_min;
      if (dwell_min > 0) {
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::DWELL_AT_MIN;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
        return;
      } else {
        // No dwell - continue sinusoidal motion (will reverse direction naturally)
        // Update state to indicate we're moving away from min
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
        // No dwell - continue sinusoidal motion (will reverse direction naturally)
        // Update state to indicate we're moving away from max
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::MOVING_TO_MIN;
      }
    } else {
      // Between bounds - update state based on direction
      TmcMutexGuard guard(mutex_);
      if (target_relative_deg > 0) {
        state_ = MotionState::MOVING_TO_MAX;
      } else {
        state_ = MotionState::MOVING_TO_MIN;
      }
    }

    // Update target position if it changed significantly
    if (fabsf(target_deg - current_pos_deg) > 0.5f) { // Update threshold: 0.5 degrees
      driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
      // Use calculated values (already in physical units)
      driver_->rampControl.SetMaxSpeed(calculated_vmax_rpm_, tmc51x0::Unit::RPM);
      driver_->rampControl.SetAcceleration(calculated_amax_rev_s2_, tmc51x0::Unit::RevPerSec);
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

  auto spi_init_result = spi.Initialize();
  if (!spi_init_result) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface (ErrorCode: %d)", static_cast<int>(spi_init_result.Error()));
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
  using Test = tmc51x0_test_config::TestConfig_17HS4401S;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC51x0 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // CRITICAL: StallGuard2 ONLY works in SpreadCycle mode (en_stealthchop_mode=0)!
  // Explicitly enable SpreadCycle mode for sensorless homing
  auto gconf_result = driver.motorControl.GetGlobalConfig();
  if (gconf_result.IsErr()) {
    ESP_LOGE(TAG, "Failed to read GCONF register (ErrorCode: %d)", static_cast<int>(gconf_result.Error()));
    return;
  }
  tmc51x0::GlobalConfig gconf_read = gconf_result.Value();
  
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
  auto gconf_verify_result = driver.motorControl.GetGlobalConfig();
  if (gconf_verify_result.IsOk()) {
    tmc51x0::GlobalConfig gconf_read = gconf_verify_result.Value();
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
  // Use RPM - driver handles conversion internally
  float tcoolthrs_rpm = 30.0f; // Keep StallGuard2 active at low speeds (~1000 steps/s for 200 steps/rev)
  if (driver.motorControl.SetCoolStepThreshold(tcoolthrs_rpm, tmc51x0::Unit::RPM)) {
    ESP_LOGI(TAG, "✓ TCOOLTHRS set to %.0f RPM - StallGuard2 active at search speeds", tcoolthrs_rpm);
  } else {
    ESP_LOGE(TAG, "✗ Failed to set TCOOLTHRS");
  }
  
  // THIGH = velocity threshold for chopper mode switching (set high to avoid interference)
  // Use RPM - driver handles conversion internally
  float thigh_rpm = 10000.0f; // Very high RPM threshold (driver converts to max register value)
  if (driver.motorControl.SetHighSpeedThreshold(thigh_rpm, tmc51x0::Unit::RPM)) {
    ESP_LOGI(TAG, "✓ THIGH set to maximum (%.0f RPM)", thigh_rpm);
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
  // Driver handles all unit conversions internally - no need to calculate steps_per_rev 
  
  // ============================================================
  // STEP 1: Find global bounds using positioning mode with StallGuard2 stop
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                    STEP 1: Finding Global Bounds                            ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Search speed is already in RPM
  float search_speed_rpm = Test::Motion::BOUNDS_SEARCH_SPEED_RPM;
  
  // Calculate 5° offset (for backing off from detected stalls)
  float offset_deg = 5.0f;
  
  // Declare position variables at function scope (will be set during bounds finding)
  float min_pos_deg_val = 0.0f;
  float max_pos_deg_val = 0.0f;
  
  // CRITICAL: Reset position to 0 and ensure clean state before starting
  auto initial_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float initial_pos_deg = initial_pos_result.IsOk() ? initial_pos_result.Value() : 0.0f;
  ESP_LOGI(TAG, "Initial position before reset: %.2f degrees", initial_pos_deg);
  
  driver.rampControl.Stop();
  driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Reset position to 0
  driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
  auto pos_after_reset_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float pos_after_reset_deg = pos_after_reset_result.IsOk() ? pos_after_reset_result.Value() : 0.0f;
  ESP_LOGI(TAG, "Position reset to: %.2f degrees (should be 0)", pos_after_reset_deg);
  
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
    auto verify_ref_result = driver.rampControl.GetReferenceSwitchConfig();
    if (verify_ref_result.IsOk()) {
      tmc51x0::ReferenceSwitchConfig verify_ref_cfg = verify_ref_result.Value();
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
  auto ramp_stat_result = driver.diagnostics.GetRampStatusRegister();
  if (ramp_stat_result.IsOk()) {
    uint32_t ramp_stat_precheck = ramp_stat_result.Value();
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
  auto verify_ref_result = driver.rampControl.GetReferenceSwitchConfig();
  if (verify_ref_result.IsOk()) {
    tmc51x0::ReferenceSwitchConfig verify_ref_cfg = verify_ref_result.Value();
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
  auto sg_result_baseline_result = driver.diagnostics.GetStallGuardResult();
  if (sg_result_baseline_result) {
    sg_result_baseline = sg_result_baseline_result.Value();
    ESP_LOGI(TAG, "✓ Initial SG_RESULT baseline: %d (at standstill)", sg_result_baseline);
  } else {
    auto sg_fallback_result = driver.diagnostics.GetStallGuard();
    if (sg_fallback_result) {
      sg_result_baseline = sg_fallback_result.Value();
    } else {
      ESP_LOGW(TAG, "⚠ Failed to read StallGuard (ErrorCode: %d), using 0", static_cast<int>(sg_fallback_result.Error()));
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
  auto rampmode_before_result = driver.rampControl.GetRampMode();
  tmc51x0::RampMode rampmode_before = tmc51x0::RampMode::HOLD;
  if (rampmode_before_result) {
    rampmode_before = rampmode_before_result.Value();
    const char* mode_str = (rampmode_before == tmc51x0::RampMode::HOLD) ? "HOLD" :
                          (rampmode_before == tmc51x0::RampMode::POSITIONING) ? "POSITIONING" :
                          (rampmode_before == tmc51x0::RampMode::VELOCITY_POS) ? "VELOCITY_POS" :
                          (rampmode_before == tmc51x0::RampMode::VELOCITY_NEG) ? "VELOCITY_NEG" : "UNKNOWN";
    ESP_LOGI(TAG, "✓ Current RAMPMODE before setting: %s", mode_str);
  } else {
    ESP_LOGW(TAG, "⚠ Failed to read RAMPMODE (ErrorCode: %d), assuming HOLD", static_cast<int>(rampmode_before_result.Error()));
  }
  
  // CRITICAL: Set RAMPMODE to POSITIONING and verify it sticks
  auto set_rampmode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
  if (!set_rampmode_result) {
    ESP_LOGE(TAG, "❌ Failed to set RAMPMODE to POSITIONING (ErrorCode: %d)!", static_cast<int>(set_rampmode_result.Error()));
    ESP_LOGE(TAG, "   Check: SPI communication and motor state");
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay for register write
  
  // Verify RAMPMODE was set correctly
  auto rampmode_verify_result = driver.rampControl.GetRampMode();
  if (!rampmode_verify_result) {
    ESP_LOGE(TAG, "❌ Failed to read RAMPMODE register (ErrorCode: %d)!", static_cast<int>(rampmode_verify_result.Error()));
    ESP_LOGE(TAG, "   Check: SPI communication");
    return;
  }
  tmc51x0::RampMode rampmode_verify = rampmode_verify_result.Value();
  if (rampmode_verify != tmc51x0::RampMode::POSITIONING) {
    ESP_LOGE(TAG, "❌ CRITICAL: RAMPMODE not set to POSITIONING! Current mode: %d", static_cast<int>(rampmode_verify));
    ESP_LOGE(TAG, "   Expected: POSITIONING (%d), Got: %d", 
             static_cast<int>(tmc51x0::RampMode::POSITIONING), static_cast<int>(rampmode_verify));
    return;
  } else {
    ESP_LOGI(TAG, "✓ RAMPMODE confirmed POSITIONING");
  }
  
  // Convert search speed from RPM to rev/s for acceleration calculation
  float search_velocity_rev_s = search_speed_rpm / 60.0f;
  float search_accel_rev_s2 = search_velocity_rev_s * 2.0f; // Reasonable acceleration: reach speed in 0.5s
  // Use physical units directly - driver handles conversions
  float vstart_rpm = 30.0f;  // ~1000 steps/s for 200 steps/rev motor
  float vstop_rpm = 3.0f;   // ~100 steps/s for 200 steps/rev motor
  
  driver.rampControl.SetMaxSpeed(search_speed_rpm, tmc51x0::Unit::RPM);
  driver.rampControl.SetAcceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
  driver.rampControl.SetDeceleration(search_accel_rev_s2, tmc51x0::Unit::RevPerSec);
  driver.rampControl.SetRampSpeeds(vstart_rpm, vstop_rpm, 0.0f, tmc51x0::Unit::RPM); // VSTART, VSTOP, V1
  
  // Small delay to ensure all registers are written before starting motion
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // ============================================================
  // Find maximum bound: Command to +360° and detect stall
  // ============================================================
  float target_deg = 360.0f;
  ESP_LOGI(TAG, "Finding maximum bound: Commanding to +360°...");
  
  // Verify current position before setting target
  auto pos_before_target_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float pos_before_target_deg = pos_before_target_result.IsOk() ? pos_before_target_result.Value() : 0.0f;
  ESP_LOGI(TAG, "Current position before setting target: %.2f degrees", pos_before_target_deg);
  
  // Set target position
  if (!driver.rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg)) {
    ESP_LOGE(TAG, "Failed to set target position!");
    return;
  }
  
  // Target position set via SetTargetPosition() - no need to verify register directly
  
  // Small delay to allow motion to start
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Check if motion started
  auto initial_speed_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
  float initial_speed_rpm = initial_speed_result.IsOk() ? initial_speed_result.Value() : 0.0f;
  auto initial_pos_check_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float initial_pos_check_deg = initial_pos_check_result.IsOk() ? initial_pos_check_result.Value() : 0.0f;
  ESP_LOGI(TAG, "After setting target: position=%.2f degrees, speed=%.1f RPM", initial_pos_check_deg, initial_speed_rpm);
  
  // Use physical units directly
  float speed_threshold_rpm = 0.3f;  // ~10 steps/s for 200 steps/rev motor
  if (std::abs(initial_speed_rpm) < speed_threshold_rpm && std::abs(initial_pos_check_deg - pos_before_target_deg) < 1.0f) {
    ESP_LOGW(TAG, "⚠️ Motor not moving after setting target! Checking status...");
    
    // Read RAMP_STAT to see why motion isn't starting
    auto ramp_stat_no_motion_result = driver.diagnostics.GetRampStatusRegister();
    if (ramp_stat_no_motion_result.IsOk()) {
      uint32_t ramp_stat_no_motion = ramp_stat_no_motion_result.Value();
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
        auto sw_mode_check_result = driver.rampControl.GetReferenceSwitchConfig();
        if (sw_mode_check_result.IsOk()) {
          tmc51x0::ReferenceSwitchConfig sw_mode_check = sw_mode_check_result.Value();
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
    auto rampmode_check_result = driver.rampControl.GetRampMode();
        if (rampmode_check_result.IsOk()) {
          tmc51x0::RampMode rampmode_check = rampmode_check_result.Value();
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
        auto rampmode_verify_result = driver.rampControl.GetRampMode();
        if (rampmode_verify_result.IsOk()) {
          tmc51x0::RampMode rampmode_verify = rampmode_verify_result.Value();
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
  
  auto start_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float start_pos_deg = start_pos_result.IsOk() ? start_pos_result.Value() : 0.0f;
  float last_pos_deg = start_pos_deg;
  uint32_t last_position_check_time = max_start_time;
  constexpr float MIN_MOVEMENT_FOR_VALID_STALL_DEG = 7.0f; // Must move at least 7° before stall is valid
  constexpr float MIN_MOVEMENT_FOR_STALL_CHECK_DEG = 2.0f; // Don't even check for stalls until motor moves this much
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
    auto current_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
    float position_delta_deg = current_pos_deg - start_pos_deg;
    
    // CRITICAL SAFETY CHECK: Never rotate more than 360° from start position
    // This prevents excessive rotation that could damage cables or mechanical systems
    if (position_delta_deg > 360.0f) {
      ESP_LOGE(TAG, "⚠️ SAFETY LIMIT: Motor rotated %.2f° (exceeds 360° limit) - STOPPING IMMEDIATELY!",
               position_delta_deg);
      driver.rampControl.Stop();
      vTaskDelay(pdMS_TO_TICKS(200));
      max_reached_360 = true; // Treat as reached 360° to use default bounds
      ESP_LOGI(TAG, "Using -175° to +175° bounds (safety limit reached)");
      break;
    }
    
    // Check if target reached (no stall, reached 360°)
    auto reached_result = driver.rampControl.IsTargetReached();
    if (reached_result && reached_result.Value()) {
      max_reached_360 = true;
      ESP_LOGI(TAG, "Reached +360° target - no stall detected");
      break;
    }
    uint32_t current_time = esp_timer_get_time() / 1000;
    auto speed_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
    float vactual_rpm = 0.0f;
    if (!speed_result) {
      ESP_LOGW(TAG, "⚠ Failed to read speed (ErrorCode: %d), using 0", static_cast<int>(speed_result.Error()));
      vactual_rpm = 0.0f;
    } else {
      vactual_rpm = speed_result.Value();
    }
    
    // Detect if motion has started (use physical units)
    float motion_start_threshold_deg = 0.5f;  // ~100 steps for 200 steps/rev motor
    if (!motion_started && std::abs(position_delta_deg) > motion_start_threshold_deg) {
      motion_started = true;
      ESP_LOGI(TAG, "Motion started: position=%.2f degrees, speed=%.1f RPM", current_pos_deg, vactual_rpm);
    }
    
    // Check for stall stop event unconditionally
    // We must check this even if movement is small, because a stall might have stopped us immediately
    auto stall_event_result = driver.diagnostics.IsStallDetected();
    bool stall_event = stall_event_result.IsOk() && stall_event_result.Value();
    
    // ALSO check SG_RESULT directly as fallback (sg_stop might not always set event_stop_sg flag)
    // If SG_RESULT=0 and motor is moving, it's a stall condition
    auto sg_result_check_result = driver.diagnostics.GetStallGuardResult();
    uint16_t sg_result_check = 0;
    bool sg_result_stall = false;
    if (sg_result_check_result) {
      sg_result_check = sg_result_check_result.Value();
      // SG_RESULT=0 means maximum load/stall, and motor should be moving for it to be valid
      // Use physical units directly
      float speed_threshold_rpm = 30.0f;  // ~1000 steps/s for 200 steps/rev motor
      if (sg_result_check == 0 && std::abs(vactual_rpm) > speed_threshold_rpm) {
        sg_result_stall = true;
        ESP_LOGW(TAG, "⚠️ SG_RESULT=0 detected (stall condition) at V=%.1f RPM, but event_stop_sg flag not set!", vactual_rpm);
      }
    } else {
      ESP_LOGW(TAG, "⚠ Failed to read SG_RESULT (ErrorCode: %d)", static_cast<int>(sg_result_check_result.Error()));
    }
    
    // If stall event detected OR SG_RESULT=0, verify it's a real stall
    if (stall_event || sg_result_stall) {
      // Read DRV_STATUS to get SG_RESULT for diagnostics
      uint32_t drv_status_val = 0;
      auto sg_result_result = driver.diagnostics.GetStallGuardResult();
      uint16_t sg_result = 0;
      bool motor_moving = false;
      if (sg_result_result) {
        sg_result = sg_result_result.Value();
        // Check if motor is moving by checking velocity (use physical units)
        float moving_threshold_rpm = 3.0f;  // ~100 steps/s for 200 steps/rev motor
        motor_moving = (std::abs(vactual_rpm) > moving_threshold_rpm);
      } else {
        ESP_LOGW(TAG, "⚠ Failed to read SG_RESULT for diagnostics (ErrorCode: %d)", static_cast<int>(sg_result_result.Error()));
      }
      
      // If detected via SG_RESULT=0 (not flag), manually stop the motor
      if (sg_result_stall && !stall_event) {
        ESP_LOGW(TAG, "⚠️ Stall detected via SG_RESULT=0 (event_stop_sg flag not set) - sg_stop may not be working!");
        ESP_LOGW(TAG, "  Position=%.2f degrees, VACTUAL=%.1f RPM, SG_RESULT=%d", current_pos_deg, vactual_rpm, sg_result);
        ESP_LOGW(TAG, "  Manually stopping motor due to SG_RESULT=0...");
        // Manually stop the motor since sg_stop didn't work
        driver.rampControl.Stop();
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait for stop to take effect
        // Re-read velocity to confirm stop
        auto vactual_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
        vactual_rpm = vactual_result.IsOk() ? vactual_result.Value() : 0.0f;
        ESP_LOGI(TAG, "  After manual stop: VACTUAL=%.1f RPM", vactual_rpm);
      }
      
      // Check if motor is actually moving (not already stopped)
      // Use physical units directly
      float stop_threshold_rpm = 0.3f;  // ~10 steps/s for 200 steps/rev motor
      if (std::abs(vactual_rpm) < stop_threshold_rpm && motor_moving) {
        ESP_LOGW(TAG, "Stall event but motor appears stopped (VACTUAL=%.1f RPM) - may be false stall", vactual_rpm);
      }
      
      // Check if motor has moved enough to consider stall valid
      if (position_delta_deg < MIN_MOVEMENT_FOR_VALID_STALL_DEG) {
        ESP_LOGW(TAG, "⚠️ Stall event detected but motor hasn't moved enough (%.2f° < %.2f°) - IGNORING FALSE STALL", 
                 position_delta_deg, MIN_MOVEMENT_FOR_VALID_STALL_DEG);
        ESP_LOGW(TAG, "  Diagnostics: SG_RESULT=%d (baseline=%d), VACTUAL=%.1f RPM, position=%.2f degrees", 
                 sg_result, sg_result_baseline, vactual_rpm, current_pos_deg);
        ESP_LOGW(TAG, "  SGT threshold=%d (lower=more sensitive) - consider increasing if false stalls persist", 
                 Test::StallGuard::SGT_HOMING);
        ESP_LOGW(TAG, "  Clearing stall flag and continuing search...");
        
        // Clear the stall event flag and continue
        if (!driver.diagnostics.ClearStallFlag()) {
          ESP_LOGE(TAG, "Failed to clear stall flag!");
        }
        
        // CRITICAL: Stall events can force RAMPMODE to HOLD - ensure it's back to POSITIONING
        auto rampmode_check_result = driver.rampControl.GetRampMode();
        if (rampmode_check_result.IsOk()) {
          tmc51x0::RampMode rampmode_check = rampmode_check_result.Value();
          if (rampmode_check != tmc51x0::RampMode::POSITIONING) {
            ESP_LOGW(TAG, "  Stall event forced RAMPMODE to HOLD - resetting to POSITIONING...");
            driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
            vTaskDelay(pdMS_TO_TICKS(50));
          }
        }
        
        // Verify stall flag cleared
        vTaskDelay(pdMS_TO_TICKS(50));
        auto ramp_stat_after_result = driver.diagnostics.GetRampStatusRegister();
  if (ramp_stat_after_result.IsOk()) {
    uint32_t ramp_stat_after = ramp_stat_after_result.Value();
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
      ESP_LOGI(TAG, "  Position moved: %.2f degrees from start (threshold: %.2f°)", 
               position_delta_deg, MIN_MOVEMENT_FOR_VALID_STALL_DEG);
      ESP_LOGI(TAG, "  SG_RESULT=%d (baseline=%d, lower=more load, 0=highest load)", 
               sg_result, sg_result_baseline);
      ESP_LOGI(TAG, "  VACTUAL=%.1f RPM, elapsed=%lu ms", vactual_rpm, elapsed);
      break;
    }
    
    // Monitor SG_RESULT periodically for diagnostics
    if (current_time - last_sg_result_time >= 200) {
      auto current_sg_result = driver.diagnostics.GetStallGuardResult();
    uint16_t current_sg = 0;
    if (current_sg_result.IsOk()) {
      current_sg = current_sg_result.Value();
        if (current_sg != last_sg_result) {
          ESP_LOGI(TAG, "  SG_RESULT changed: %d -> %d (position=%.2f degrees, speed=%.1f RPM)", 
                   last_sg_result, current_sg, current_pos_deg, vactual_rpm);
          last_sg_result = current_sg;
        }
      }
      last_sg_result_time = current_time;
    }
    
    // Log position progress periodically (every 500ms)
    if (current_time - last_position_check_time >= 500) {
      float position_change_deg = current_pos_deg - last_pos_deg;
      ESP_LOGI(TAG, "  Progress: position=%.2f° (+%.2f° from start, +%.2f° since last), speed=%.1f RPM, elapsed=%lu ms",
               current_pos_deg, position_delta_deg, position_change_deg, vactual_rpm, elapsed);
      
      // Check if motor is stuck (position not changing but should be moving)
      // Use physical units directly
      float stuck_pos_threshold_deg = 0.25f;  // ~50 steps for 200 steps/rev motor
      float stuck_speed_threshold_rpm = 3.0f;  // ~100 steps/s for 200 steps/rev motor
      if (motion_started && std::abs(position_change_deg) < stuck_pos_threshold_deg && std::abs(vactual_rpm) < stuck_speed_threshold_rpm && elapsed > 1000) {
        ESP_LOGW(TAG, "  ⚠️ Motor appears stuck: position change=%.2f degrees, speed=%.1f RPM", 
                 position_change_deg, vactual_rpm);
      }
      
      last_position_check_time = current_time;
      last_pos_deg = current_pos_deg;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms
  }
  
  auto max_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float max_pos_deg_local = max_pos_result.IsOk() ? max_pos_result.Value() : 0.0f;
  
  if (max_stall_detected) {
    ESP_LOGI(TAG, "Maximum bound found at stall: %.2f degrees", max_pos_deg_local);
    
    // Back off with 5° offset
    driver.rampControl.Stop();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    float backoff_target_deg = max_pos_deg_local - offset_deg;
    ESP_LOGI(TAG, "Backing off 5° from maximum stall (%.2f° -> %.2f°)...", max_pos_deg_local, backoff_target_deg);
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(backoff_target_deg, tmc51x0::Unit::Deg);
    driver.rampControl.SetMaxSpeed(search_speed_rpm / 2.0f, tmc51x0::Unit::RPM);
    int backoff_wait_checks = 0;
    constexpr int MAX_BACKOFF_WAIT_CHECKS = 50;
    while (backoff_wait_checks < MAX_BACKOFF_WAIT_CHECKS) {
      auto backoff_reached_result = driver.rampControl.IsTargetReached();
      if (backoff_reached_result && backoff_reached_result.Value()) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      backoff_wait_checks++;
    }
    if (backoff_wait_checks >= MAX_BACKOFF_WAIT_CHECKS) {
      ESP_LOGW(TAG, "⚠ Backoff wait timeout after %d checks", MAX_BACKOFF_WAIT_CHECKS);
    }
    auto max_pos_after_backoff_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    max_pos_deg_local = max_pos_after_backoff_result.IsOk() ? max_pos_after_backoff_result.Value() : max_pos_deg_local;
    max_pos_deg_val = max_pos_deg_local;
  } else if (max_reached_360) {
    ESP_LOGI(TAG, "No stall at +360° - will use -175° to +175° bounds");
    // Will handle this after checking minimum bound
    max_pos_deg_val = max_pos_deg_local; // Store for later use
  }
  
  // ============================================================
  // Find minimum bound: Command to -360° and detect stall
  // ============================================================
  float min_target_deg = -360.0f;
  ESP_LOGI(TAG, "Finding minimum bound: Commanding to -360°...");
  
  // Clear stall flag before starting
  // Clear any existing stall flags before starting
  driver.diagnostics.ClearStallFlag();
  
  driver.rampControl.SetTargetPosition(min_target_deg, tmc51x0::Unit::Deg);
  
  // Wait for motion to complete (either target reached or stall detected)
  bool min_stall_detected = false;
  bool min_reached_360 = false;
  uint32_t min_start_time = esp_timer_get_time() / 1000;
  auto min_start_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float min_start_pos_deg = min_start_pos_result.IsOk() ? min_start_pos_result.Value() : 0.0f;
  float min_last_pos_deg = min_start_pos_deg;
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
    auto current_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
    float position_delta_deg = std::abs(current_pos_deg - min_start_pos_deg);
    
    // CRITICAL SAFETY CHECK: Never rotate more than 360° from start position
    // This prevents excessive rotation that could damage cables or mechanical systems
    if (position_delta_deg > 360.0f) {
      ESP_LOGE(TAG, "⚠️ SAFETY LIMIT: Motor rotated %.2f° (exceeds 360° limit) - STOPPING IMMEDIATELY!",
               position_delta_deg);
      driver.rampControl.Stop();
      vTaskDelay(pdMS_TO_TICKS(200));
      min_reached_360 = true; // Treat as reached 360° to use default bounds
      ESP_LOGI(TAG, "Using -175° to +175° bounds (safety limit reached)");
      break;
    }
    
    // Check if target reached (no stall, reached -360°)
    auto reached_result = driver.rampControl.IsTargetReached();
    if (reached_result && reached_result.Value()) {
      min_reached_360 = true;
      ESP_LOGI(TAG, "Reached -360° target - no stall detected");
      break;
    }
    uint32_t current_time = esp_timer_get_time() / 1000;
    auto speed_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
    float vactual_rpm = 0.0f;
    if (!speed_result) {
      ESP_LOGW(TAG, "⚠ Failed to read speed (ErrorCode: %d), using 0", static_cast<int>(speed_result.Error()));
      vactual_rpm = 0.0f;
    } else {
      vactual_rpm = speed_result.Value();
    }
    
    // Detect if motion has started (use physical units)
    float motion_start_threshold_deg = 0.5f;  // ~100 steps for 200 steps/rev motor
    if (!min_motion_started && position_delta_deg > motion_start_threshold_deg) {
      min_motion_started = true;
      ESP_LOGI(TAG, "Motion started: position=%.2f degrees, speed=%.1f RPM", current_pos_deg, vactual_rpm);
    }
    
    // Check for stall stop event unconditionally
    auto stall_event_result = driver.diagnostics.IsStallDetected();
    bool stall_event = stall_event_result.IsOk() && stall_event_result.Value();
    
    // If stall event detected, verify it's a real stall
    if (stall_event) {
      // Read SG_RESULT for diagnostics
      auto sg_result_result = driver.diagnostics.GetStallGuardResult();
      uint16_t sg_result = 0;
      bool motor_moving = false;
      if (sg_result_result.IsOk()) {
        sg_result = sg_result_result.Value();
        // Check if motor is moving by checking velocity (use physical units)
        float moving_threshold_rpm = 3.0f;  // ~100 steps/s for 200 steps/rev motor
        motor_moving = (std::abs(vactual_rpm) > moving_threshold_rpm);
      }
      
      // Check if motor is actually moving
      // Use physical units directly
      float stop_threshold_rpm = 0.3f;  // ~10 steps/s for 200 steps/rev motor
      if (std::abs(vactual_rpm) < stop_threshold_rpm && motor_moving) {
        ESP_LOGW(TAG, "Stall event but motor appears stopped (VACTUAL=%.1f RPM) - may be false stall", vactual_rpm);
      }
      
      // Check if motor has moved enough to consider stall valid
      if (position_delta_deg < MIN_MOVEMENT_FOR_VALID_STALL_DEG) {
        ESP_LOGW(TAG, "⚠️ Stall event detected but motor hasn't moved enough (%.2f° < %.2f°) - IGNORING FALSE STALL", 
                 position_delta_deg, MIN_MOVEMENT_FOR_VALID_STALL_DEG);
        ESP_LOGW(TAG, "  Diagnostics: SG_RESULT=%d (baseline=%d), VACTUAL=%.1f RPM, position=%.2f degrees", 
                 sg_result, sg_result_baseline, vactual_rpm, current_pos_deg);
        ESP_LOGW(TAG, "  SGT threshold=%d (lower=more sensitive) - consider increasing if false stalls persist", 
                 Test::StallGuard::SGT_HOMING);
        ESP_LOGW(TAG, "  Clearing stall flag and continuing search...");
        
        // Clear the stall event flag and continue
        constexpr uint32_t CLEAR_STALL_BIT = 0x01; // event_stop_sg bit
        if (!driver.diagnostics.ClearRampStatus(CLEAR_STALL_BIT)) {
          ESP_LOGE(TAG, "Failed to clear stall flag!");
        }
        
        // CRITICAL: Stall events can force RAMPMODE to HOLD - ensure it's back to POSITIONING
        auto rampmode_check_result = driver.rampControl.GetRampMode();
        if (rampmode_check_result.IsOk()) {
          tmc51x0::RampMode rampmode_check = rampmode_check_result.Value();
          if (rampmode_check != tmc51x0::RampMode::POSITIONING) {
            ESP_LOGW(TAG, "  Stall event forced RAMPMODE to HOLD - resetting to POSITIONING...");
            driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
            vTaskDelay(pdMS_TO_TICKS(50));
          }
        }
        
        // Verify stall flag cleared
        vTaskDelay(pdMS_TO_TICKS(50));
        auto ramp_stat_after_result = driver.diagnostics.GetRampStatusRegister();
  if (ramp_stat_after_result.IsOk()) {
    uint32_t ramp_stat_after = ramp_stat_after_result.Value();
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
      ESP_LOGI(TAG, "  Position moved: %.2f degrees from start (threshold: %.2f°)", 
               position_delta_deg, MIN_MOVEMENT_FOR_VALID_STALL_DEG);
      ESP_LOGI(TAG, "  SG_RESULT=%d (baseline=%d, lower=more load, 0=highest load)", 
               sg_result, sg_result_baseline);
      ESP_LOGI(TAG, "  VACTUAL=%.1f RPM, elapsed=%lu ms", vactual_rpm, elapsed);
      break;
    }
    
    // Monitor SG_RESULT periodically
    if (current_time - min_last_sg_result_time >= 200) {
      auto current_sg_result = driver.diagnostics.GetStallGuardResult();
    uint16_t current_sg = 0;
    if (current_sg_result.IsOk()) {
      current_sg = current_sg_result.Value();
        if (current_sg != min_last_sg_result) {
          ESP_LOGI(TAG, "  SG_RESULT changed: %d -> %d (position=%.2f degrees, speed=%.1f RPM)", 
                   min_last_sg_result, current_sg, current_pos_deg, vactual_rpm);
          min_last_sg_result = current_sg;
        }
      }
      min_last_sg_result_time = current_time;
    }
    
    // Log position progress periodically (every 500ms)
    if (current_time - min_last_position_check_time >= 500) {
      float position_change_deg = current_pos_deg - min_last_pos_deg;
      ESP_LOGI(TAG, "  Progress: position=%.2f° (%.2f° from start, %.2f° since last), speed=%.1f RPM, elapsed=%lu ms",
               current_pos_deg, current_pos_deg - min_start_pos_deg, position_change_deg, vactual_rpm, elapsed);
      
      // Check if motor is stuck (use physical units)
      float stuck_pos_threshold_deg = 0.25f;  // ~50 steps for 200 steps/rev motor
      float stuck_speed_threshold_rpm = 3.0f;  // ~100 steps/s for 200 steps/rev motor
      if (min_motion_started && std::abs(position_change_deg) < stuck_pos_threshold_deg && std::abs(vactual_rpm) < stuck_speed_threshold_rpm && elapsed > 1000) {
        ESP_LOGW(TAG, "  ⚠️ Motor appears stuck: position change=%.2f degrees, speed=%.1f RPM", 
                 position_change_deg, vactual_rpm);
      }
      
      min_last_position_check_time = current_time;
      min_last_pos_deg = current_pos_deg;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms
  }
  
  auto min_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float min_pos_deg_local = min_pos_result.IsOk() ? min_pos_result.Value() : 0.0f;
  
  if (min_stall_detected) {
    ESP_LOGI(TAG, "Minimum bound found at stall: %.2f degrees", min_pos_deg_local);
    
    // Back off with 5° offset
    driver.rampControl.Stop();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    float backoff_target_deg = min_pos_deg_local + offset_deg;
    ESP_LOGI(TAG, "Backing off 5° from minimum stall (%.2f° -> %.2f°)...", min_pos_deg_local, backoff_target_deg);
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(backoff_target_deg, tmc51x0::Unit::Deg);
    driver.rampControl.SetMaxSpeed(search_speed_rpm / 2.0f, tmc51x0::Unit::RPM);
    int backoff_wait_checks = 0;
    constexpr int MAX_BACKOFF_WAIT_CHECKS = 50;
    while (backoff_wait_checks < MAX_BACKOFF_WAIT_CHECKS) {
      auto backoff_reached_result = driver.rampControl.IsTargetReached();
      if (backoff_reached_result && backoff_reached_result.Value()) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      backoff_wait_checks++;
    }
    if (backoff_wait_checks >= MAX_BACKOFF_WAIT_CHECKS) {
      ESP_LOGW(TAG, "⚠ Backoff wait timeout after %d checks", MAX_BACKOFF_WAIT_CHECKS);
    }
    auto min_pos_after_backoff_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    min_pos_deg_local = min_pos_after_backoff_result.IsOk() ? min_pos_after_backoff_result.Value() : min_pos_deg_local;
  }
  // Store min position for later use (regardless of whether stall was detected)
  min_pos_deg_val = min_pos_deg_local;
  
  if (min_reached_360) {
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
  
  // Declare position variables at function scope (in degrees)
  float min_position_deg = 0.0f;
  float max_position_deg = 0.0f;
  
  if (reached_360) {
    // No stalls detected - mark current position as 0 and use -175° to +175° bounds
    ESP_LOGI(TAG, "No stalls detected - marking current position as 0, using -175° to +175° bounds");
    
    driver.rampControl.Stop();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    
    // Wait for motor to stop
    uint32_t stop_wait_start = esp_timer_get_time() / 1000;
    while (true) {
      auto vactual_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
      float vactual_rpm = vactual_result.IsOk() ? vactual_result.Value() : 0.0f;
      // Use physical units directly
      float speed_threshold_rpm = 0.3f;  // ~10 steps/s for 200 steps/rev motor
      if (std::abs(vactual_rpm) < speed_threshold_rpm) break;
      if ((esp_timer_get_time() / 1000) - stop_wait_start > 2000) break;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Reset position to 0
    driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
    
    // Use -175° to +175° bounds
    float bounds_deg = 175.0f;
    max_pos_deg_val = bounds_deg;
    min_pos_deg_val = -bounds_deg;
    
    ESP_LOGI(TAG, "Position reset to 0, bounds set to ±%.1f°", bounds_deg);
    
    // Move to center (0) if not already there
    auto current_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
    if (std::abs(current_pos_deg) > 1.0f) { // 1 degree threshold
      ESP_LOGI(TAG, "Moving to center position (0)...");
      driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
      driver.rampControl.SetTargetPosition(0.0f, tmc51x0::Unit::Deg);
      // Use physical units directly
      float vmax_rpm = 30.0f;  // ~1000 steps/s for 200 steps/rev motor
      float amax_rev_s2 = 10.0f;  // Reasonable acceleration
      driver.rampControl.SetMaxSpeed(vmax_rpm, tmc51x0::Unit::RPM);
      driver.rampControl.SetAcceleration(amax_rev_s2, tmc51x0::Unit::RevPerSec);
      driver.rampControl.SetDeceleration(amax_rev_s2, tmc51x0::Unit::RevPerSec);
      while (!driver.rampControl.IsTargetReached()) {
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
      ESP_LOGI(TAG, "Arrived at center position (0)");
    }
    
    // Store bounds in degrees (SetGlobalBounds now accepts degrees)
    max_position_deg = max_pos_deg_val;
    min_position_deg = min_pos_deg_val;
  } else {
    // At least one stall detected - move to center between bounds
    float center_pos_deg = (min_pos_deg_val + max_pos_deg_val) / 2.0f;
    ESP_LOGI(TAG, "Moving to center position: %.2f degrees (between %.2f° and %.2f°)", 
             center_pos_deg, min_pos_deg_val, max_pos_deg_val);
    
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(center_pos_deg, tmc51x0::Unit::Deg);
    // Use physical units directly
    float vmax_rpm = 30.0f;  // ~1000 steps/s for 200 steps/rev motor
    float amax_rev_s2 = 10.0f;  // Reasonable acceleration
    driver.rampControl.SetMaxSpeed(vmax_rpm, tmc51x0::Unit::RPM);
    driver.rampControl.SetAcceleration(amax_rev_s2, tmc51x0::Unit::RevPerSec);
    driver.rampControl.SetDeceleration(amax_rev_s2, tmc51x0::Unit::RevPerSec);
    int backoff_wait_checks = 0;
    constexpr int MAX_BACKOFF_WAIT_CHECKS = 50;
    while (backoff_wait_checks < MAX_BACKOFF_WAIT_CHECKS) {
      auto backoff_reached_result = driver.rampControl.IsTargetReached();
      if (backoff_reached_result && backoff_reached_result.Value()) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      backoff_wait_checks++;
    }
    if (backoff_wait_checks >= MAX_BACKOFF_WAIT_CHECKS) {
      ESP_LOGW(TAG, "⚠ Backoff wait timeout after %d checks", MAX_BACKOFF_WAIT_CHECKS);
    }
    
    // Reset position to 0 at center
    driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
    
    // Adjust bounds relative to new center (in degrees)
    float adjusted_min_deg = min_pos_deg_val - center_pos_deg;
    float adjusted_max_deg = max_pos_deg_val - center_pos_deg;
    min_position_deg = adjusted_min_deg;
    max_position_deg = adjusted_max_deg;
    
    ESP_LOGI(TAG, "Home position set to 0 (center of bounds)");
    ESP_LOGI(TAG, "Adjusted bounds: min=%.2f°, max=%.2f° from center", adjusted_min_deg, adjusted_max_deg);
  }


  // ============================================================
  // STEP 2: Set up global bounds and home
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║              STEP 2: Setting Global Bounds and Home                        ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Determine if we're bounded (either by stall detection or 360° limit)
  bool bounded = (stall_detected_min && stall_detected_max) || reached_360;
  auto current_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float current_pos_deg = current_pos_result.IsOk() ? current_pos_result.Value() : 0.0f;
  auto current_pos_deg_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
  float current_pos_deg = current_pos_deg_result.IsOk() ? current_pos_deg_result.Value() : 0.0f;

  FatigueTestMotion motion(&driver);
  // Driver handles all unit conversions - no ConfigureMotor needed

  if (!bounded) {
    ESP_LOGW(TAG, "=== UNBOUNDED MODE ===");
    motion.SetUnbounded(current_pos_deg, 350.0f); // 350 degrees default range
  } else {
    ESP_LOGI(TAG, "=== BOUNDED MODE ===");

    if (reached_360) {
      // Special case: No stall detected, using -175° to +175° bounds
      // Position is already at 0 (we reset it at 360° point and moved to center)
      ESP_LOGI(TAG, "Using -175° to +175° bounds (no stall detected at 360°)");
      
      // Set global bounds directly in degrees (already relative to 0)
      motion.SetGlobalBounds(-175.0f, 175.0f);
      
      float min_deg, max_deg;
      motion.GetGlobalBoundsDegrees(min_deg, max_deg);
      ESP_LOGI(TAG, "Global bounds: min=%.2f°, max=%.2f° from center", min_deg, max_deg);
    } else {
      // Normal case: Stall detected on both ends
    // Set middle as home
    float middle_pos_deg = (min_pos_deg_val + max_pos_deg_val) / 2.0f;
    ESP_LOGI(TAG, "Moving to middle position: %.2f degrees", middle_pos_deg);

    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(middle_pos_deg, tmc51x0::Unit::Deg);
    // Use physical units directly
    float vmax_rpm = 30.0f;  // ~1000 steps/s for 200 steps/rev
    float amax_rev_s2 = 10.0f;  // Reasonable acceleration
    driver.rampControl.SetMaxSpeed(vmax_rpm, tmc51x0::Unit::RPM);
    driver.rampControl.SetAcceleration(amax_rev_s2, tmc51x0::Unit::RevPerSec);

    int backoff_wait_checks = 0;
    constexpr int MAX_BACKOFF_WAIT_CHECKS = 50;
    while (backoff_wait_checks < MAX_BACKOFF_WAIT_CHECKS) {
      auto backoff_reached_result = driver.rampControl.IsTargetReached();
      if (backoff_reached_result && backoff_reached_result.Value()) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      backoff_wait_checks++;
    }
    if (backoff_wait_checks >= MAX_BACKOFF_WAIT_CHECKS) {
      ESP_LOGW(TAG, "⚠ Backoff wait timeout after %d checks", MAX_BACKOFF_WAIT_CHECKS);
    }

    driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Deg);
    ESP_LOGI(TAG, "Home position set to 0 (middle of bounds)");

    // Set global bounds relative to new home (in degrees)
    float global_min_deg = min_pos_deg_val - middle_pos_deg;
    float global_max_deg = max_pos_deg_val - middle_pos_deg;
    motion.SetGlobalBounds(global_min_deg, global_max_deg);

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
      auto pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
      float pos_deg = pos_result.IsOk() ? pos_result.Value() : 0.0f;
      auto speed_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
      float speed_rpm = speed_result.IsOk() ? speed_result.Value() : 0.0f;
      FatigueTestMotion::Status status = motion.GetStatus();
      ESP_LOGI(TAG, "Position: %.2f°, Speed: %.1f RPM, Cycles: %lu/%lu %s", 
               pos_deg, speed_rpm, status.current_cycles,
               status.target_cycles == 0 ? 0xFFFFFFFF : status.target_cycles, 
               status.running ? "(running)" : "(stopped)");
      last_log_time = current_time;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
