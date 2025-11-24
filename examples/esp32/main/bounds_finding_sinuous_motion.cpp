/**
 * @file bounds_finding_sinuous_motion.cpp
 * @brief Fatigue testing platform: Sensorless bounds finding and sinusoidal motion with UART command interface
 *
 * This example is designed for cable/strain relief fatigue testing platforms:
 * 1. Finding motor bounds using sensorless homing (both directions)
 * 2. Setting global bounds (hardware limits) and local bounds (oscillation range)
 * 3. Performing pure sinusoidal back-and-forth motion between local bounds with:
 *    - Configurable angle amplitude and frequency
 *    - Target cycle count (cycles counted at center crossing)
 *    - Dwell times at bounds and optionally at center
 *    - Automatic clipping of local bounds to global bounds
 *    - Automatic stop at center when cycle count reached
 * 4. UART command interface for real-time parameter adjustment
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 * - Mechanical stops at both ends for bounds finding (optional - handles unbounded)
 * - UART debug port for command interface (typically UART_NUM_0 for USB serial)
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5
 * - Control: EN=2, DIR=4, STEP=15
 * - UART: Uses default UART_NUM_0 (USB serial port)
 *
 * UART Command Interface:
 * Commands follow Linux-like argument structure:
 *   -f <value> or --freq <value>     : Set frequency in Hz
 *   -d <min> <max> [center]          : Set dwell times in ms (min, max, optional center)
 *   -b <min> <max> or --bounds <min> <max> : Set angle bounds from center in degrees
 *   -c <count> or --cycles <count>   : Set target cycle count (0 = infinite)
 *   -a start|stop|reset              : Action commands (start motion, stop, reset cycles)
 *   -s or --status                   : Show current status
 *   -h or --help                     : Show help message
 *
 * Examples:
 *   -f 0.5                           : Set frequency to 0.5 Hz
 *   -d 2000 2000 500                 : Set dwell times: 2s at min, 2s at max, 0.5s at center
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

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
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
  tmc5160::TMC5160<Esp32SPI>* driver_;

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
  uint32_t dwell_at_center_ms_; // Dwell time at center/home (optional)

  // Cycle tracking
  uint32_t target_cycles_;       // Target number of cycles (0 = infinite)
  uint32_t current_cycles_;      // Current cycle count
  bool cycle_complete_;          // Whether target cycles reached
  bool last_was_negative_;       // Last position relative to center (for cycle counting)
  bool cycle_started_;           // Whether a cycle has started (left center)
  int32_t last_target_relative_; // Last target position relative to center (for cycle counting)

  // State machine
  enum class MotionState { SINUOUS_MOTION, DWELL_AT_MIN, DWELL_AT_MAX, DWELL_AT_CENTER, STOPPED };
  MotionState state_;
  uint32_t dwell_start_time_ms_;

  // Thread safety
  mutable Esp32TmcMutex mutex_;

public:
  FatigueTestMotion(tmc5160::TMC5160<Esp32SPI>* driver) noexcept
      : driver_(driver), global_min_bound_(0), global_max_bound_(0), local_min_bound_(0), local_max_bound_(0),
        home_position_(0), amplitude_(1000.0F), frequency_hz_(0.5F), running_(false), start_time_us_(0),
        phase_offset_(0.0F), bounded_(false), steps_per_rev_(200), angle_unit_(AngleUnit::DEGREES), dwell_at_min_ms_(0),
        dwell_at_max_ms_(0), dwell_at_center_ms_(0), target_cycles_(0), current_cycles_(0), cycle_complete_(false),
        last_was_negative_(false), cycle_started_(false), last_target_relative_(0), state_(MotionState::STOPPED),
        dwell_start_time_ms_(0) {
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
      float min_deg = tmc5160::StepsToDegrees(global_min_bound_, steps_per_rev_);
      float max_deg = tmc5160::StepsToDegrees(global_max_bound_, steps_per_rev_);
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
    min_degrees = tmc5160::StepsToDegrees(min_bound, steps);
    max_degrees = tmc5160::StepsToDegrees(max_bound, steps);
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
    driver_->rampControl.SetCurrentPosition(0);
    ESP_LOGW(TAG, "Unbounded mode: No mechanical stops found");
    ESP_LOGI(TAG, "Using current position as home: %d steps", current_position);
    ESP_LOGI(TAG, "Default global range: [%d, %d] steps", min_bound, max_bound);
    if (steps > 0) {
      float range_deg = tmc5160::StepsToDegrees(default_range_steps, steps);
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
    int32_t min_steps = tmc5160::DegreesToSteps(min_deg, steps);
    int32_t max_steps = tmc5160::DegreesToSteps(max_deg, steps);

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

    float actual_min = tmc5160::StepsToDegrees(min_steps, steps);
    float actual_max = tmc5160::StepsToDegrees(max_steps, steps);
    ESP_LOGI(TAG, "Local bounds set: min=%.2f°, max=%.2f° from center", actual_min, actual_max);
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
    }
    ESP_LOGI(TAG, "Frequency updated: %.2f Hz", frequency_hz_);
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
  bool SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms, uint32_t dwell_at_center_ms = 0) noexcept {
    {
      TmcMutexGuard guard(mutex_);
      dwell_at_min_ms_ = dwell_at_min_ms;
      dwell_at_max_ms_ = dwell_at_max_ms;
      dwell_at_center_ms_ = dwell_at_center_ms;
    }
    ESP_LOGI(TAG, "Dwell times updated: min=%lu ms, max=%lu ms, center=%lu ms", 
             dwell_at_min_ms_, dwell_at_max_ms_, dwell_at_center_ms_);
    return true;
  }

  /**
   * @brief Get dwell times (thread-safe)
   */
  void GetDwellTimes(uint32_t& dwell_at_min_ms, uint32_t& dwell_at_max_ms, uint32_t& dwell_at_center_ms) const noexcept {
    TmcMutexGuard guard(mutex_);
    dwell_at_min_ms = dwell_at_min_ms_;
    dwell_at_max_ms = dwell_at_max_ms_;
    dwell_at_center_ms = dwell_at_center_ms_;
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
    min_degrees = tmc5160::StepsToDegrees(min_bound, steps);
    max_degrees = tmc5160::StepsToDegrees(max_bound, steps);
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

      running_ = true;
      state_ = MotionState::SINUOUS_MOTION;
      start_time_us_ = esp_timer_get_time();

      // If resuming from stop, calculate phase offset from current position
      int32_t current_pos = driver_->rampControl.GetCurrentPosition();
      int32_t pos_relative = current_pos - home_position_;
      if (amplitude_ > 0) {
        double normalized_pos = static_cast<double>(pos_relative) / amplitude_;
        if (normalized_pos > 1.0)
          normalized_pos = 1.0;
        if (normalized_pos < -1.0)
          normalized_pos = -1.0;
        phase_offset_ = asin(normalized_pos);
        // Initialize cycle tracking based on current position
        last_was_negative_ = (pos_relative < 0);
        cycle_started_ = (abs(pos_relative) > 10); // Started if away from center
        last_target_relative_ = pos_relative;
      } else {
        phase_offset_ = 0.0F;
        last_was_negative_ = false;
        cycle_started_ = false;
      }
      current_cycles = current_cycles_;
      target_cycles = target_cycles_;
    }

    ESP_LOGI(TAG, "Starting fatigue test motion (cycles: %lu/%lu)", current_cycles,
             target_cycles == 0 ? 0xFFFFFFFF : target_cycles);
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

      // Check if cycle count reached - if so, move to center and stop
      if (target_cycles_ > 0 && current_cycles_ >= target_cycles_) {
        if (!cycle_complete_) {
          cycle_complete_ = true;
          uint32_t cycles = current_cycles_;
          int32_t home = home_position_;
          guard.unlock();
          ESP_LOGI(TAG, "Target cycle count reached: %lu cycles", cycles);
          // Move to center before stopping
          driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
          driver_->rampControl.SetTargetPosition(home);
          driver_->rampControl.SetMaxSpeed(1000.0F);
          driver_->rampControl.SetAcceleration(2000.0F);
          // Wait for center to be reached
          while (!driver_->rampControl.IsTargetReached()) {
            vTaskDelay(pdMS_TO_TICKS(10));
          }
          {
            TmcMutexGuard guard2(mutex_);
            state_ = MotionState::STOPPED;
            running_ = false;
          }
          ESP_LOGI(TAG, "Motion stopped at center position");
        }
        return;
      }
    }

    uint32_t current_time_ms = esp_timer_get_time() / 1000;
    MotionState current_state;
    uint32_t dwell_min, dwell_max, dwell_center, dwell_start;
    {
      TmcMutexGuard guard(mutex_);
      current_state = state_;
      dwell_min = dwell_at_min_ms_;
      dwell_max = dwell_at_max_ms_;
      dwell_center = dwell_at_center_ms_;
      dwell_start = dwell_start_time_ms_;
    }

    switch (current_state) {
    case MotionState::DWELL_AT_MIN:
      if (current_time_ms - dwell_start >= dwell_min) {
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        phase_offset_ = -M_PI / 2.0; // Start from minimum position
      }
      break;

    case MotionState::DWELL_AT_MAX:
      if (current_time_ms - dwell_start >= dwell_max) {
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        phase_offset_ = M_PI / 2.0; // Start from maximum position
      }
      break;

    case MotionState::DWELL_AT_CENTER:
      if (current_time_ms - dwell_start >= dwell_center) {
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        // Continue from center (phase = 0 or π)
        int32_t current_pos = driver_->rampControl.GetCurrentPosition();
        int32_t pos_relative = current_pos - home_position_;
        if (abs(pos_relative) < 10) {
          // At center, determine direction from last position
          phase_offset_ = last_was_negative_ ? M_PI : 0.0F;
        } else {
          // Near center, calculate phase
          double normalized_pos = static_cast<double>(pos_relative) / amplitude_;
          if (normalized_pos > 1.0)
            normalized_pos = 1.0;
          if (normalized_pos < -1.0)
            normalized_pos = -1.0;
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
    uint32_t dwell_center_ms;
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
      status.dwell_center_ms = dwell_at_center_ms_;
      min_bound = local_min_bound_;
      max_bound = local_max_bound_;
      global_min = global_min_bound_;
      global_max = global_max_bound_;
      steps = steps_per_rev_;
    }
    
    if (steps > 0) {
      status.min_degrees_from_center = tmc5160::StepsToDegrees(min_bound, steps);
      status.max_degrees_from_center = tmc5160::StepsToDegrees(max_bound, steps);
      status.global_min_degrees = tmc5160::StepsToDegrees(global_min, steps);
      status.global_max_degrees = tmc5160::StepsToDegrees(global_max, steps);
    }
    
    return status;
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
        float min_deg = tmc5160::StepsToDegrees(local_min_bound_, steps_per_rev_);
        float max_deg = tmc5160::StepsToDegrees(local_max_bound_, steps_per_rev_);
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
    uint32_t target_cycles, current_cycles;
    bool cycle_started;
    int32_t last_target_rel;
    uint32_t dwell_min, dwell_max, dwell_center;
    MotionState current_state;
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
      current_cycles = current_cycles_;
      cycle_started = cycle_started_;
      last_target_rel = last_target_relative_;
      dwell_min = dwell_at_min_ms_;
      dwell_max = dwell_at_max_ms_;
      dwell_center = dwell_at_center_ms_;
      current_state = state_;
      phase_off = phase_offset_;
    }

    // Calculate sinusoidal position
    double elapsed_s = elapsed_us / 1000000.0;
    double angle = 2.0 * M_PI * freq * elapsed_s + phase_off;
    double sin_value = sin(angle);

    // Calculate target position
    int32_t target = home + static_cast<int32_t>(amp * sin_value);

    // Get current position relative to center for cycle counting
    int32_t current_pos = driver_->rampControl.GetCurrentPosition();
    int32_t target_relative = target - home;

    // Cycle counting: one cycle = center → min → max → center (or center → max → min → center)
    // Count cycles when crossing through center (0 crossing point)
    if (current_state == MotionState::SINUOUS_MOTION) {
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
    }

    // Clamp to local bounds and handle dwell states
    if (target <= local_min) {
      target = local_min;
      if (current_state == MotionState::SINUOUS_MOTION && dwell_min > 0) {
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::DWELL_AT_MIN;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(target);
        return;
      }
    } else if (target >= local_max) {
      target = local_max;
      if (current_state == MotionState::SINUOUS_MOTION && dwell_max > 0) {
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::DWELL_AT_MAX;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(target);
        return;
      }
    }

    // Check if we're passing through center and need to dwell
    if (dwell_center > 0 && current_state == MotionState::SINUOUS_MOTION) {
      if (abs(target_relative) < 20) { // Within 20 steps of center
        TmcMutexGuard guard(mutex_);
        state_ = MotionState::DWELL_AT_CENTER;
        dwell_start_time_ms_ = esp_timer_get_time() / 1000;
        driver_->rampControl.SetTargetPosition(home);
        return;
      }
    }

    // Update target position if it changed significantly
    if (abs(target - current_pos) > 10) { // Update threshold: 10 steps
      driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(target);
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
  uint32_t min_ms = std::strtoul(args[0].c_str(), nullptr, 10);
  uint32_t max_ms = std::strtoul(args[1].c_str(), nullptr, 10);
  uint32_t center_ms = 0;
  if (args.size() >= 3) {
    center_ms = std::strtoul(args[2].c_str(), nullptr, 10);
  }
  return motion.SetDwellTimes(min_ms, max_ms, center_ms);
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
  ESP_LOGI(TAG, "  Frequency: %.2f Hz", status.frequency_hz);
  ESP_LOGI(TAG, "  Local Bounds: %.2f° to %.2f° from center", 
           status.min_degrees_from_center, status.max_degrees_from_center);
  ESP_LOGI(TAG, "  Global Bounds: %.2f° to %.2f° from center", 
           status.global_min_degrees, status.global_max_degrees);
  ESP_LOGI(TAG, "  Cycles: %lu / %lu %s", status.current_cycles, 
           status.target_cycles == 0 ? 0xFFFFFFFF : status.target_cycles,
           status.target_cycles == 0 ? "(infinite)" : "");
  ESP_LOGI(TAG, "  Dwell Times: min=%lu ms, max=%lu ms, center=%lu ms",
           status.dwell_min_ms, status.dwell_max_ms, status.dwell_center_ms);
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
  ESP_LOGI(TAG, "║         TMC5160 Fatigue Test Platform: Bounds Finding & Sinuous Motion      ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Create SPI communication interface
  Esp32SPI spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15,
               4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC5160 driver instance
  tmc5160::TMC5160<Esp32SPI> driver(spi);

  // Use centralized configuration
  using Motor = tmc5160_test_config::MotorConfig_17HS4401S;
  using Test = tmc5160_test_config::TestConfig_17HS4401S;

  // Configure driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = Motor::IRUN;
  cfg.motor.ihold = Motor::IHOLD;
  cfg.motor.global_scaler = Motor::GLOBAL_SCALER;
  cfg.chopper.mres = Motor::MRES; // 256 microsteps (MRES=0)
  cfg.chopper.toff = Motor::TOFF;
  cfg.chopper.hend = Motor::HEND;
  cfg.chopper.hstrt = Motor::HSTRT;
  cfg.chopper.intpol = Motor::INTERPOLATION;
  cfg.chopper.tbl = Motor::TBL;
  
  // StealthChop settings
  cfg.stealthchop.pwm_autoscale = Motor::STEALTH_AUTOSCALE;
  cfg.stealthchop.pwm_autograd = Motor::STEALTH_AUTOGRAD;
  cfg.stealthchop.pwm_freq = Motor::STEALTH_FREQ;
  cfg.stealthchop.pwm_ofs = Motor::STEALTH_OFS;
  
  // Protection settings
  cfg.short_protection.s2vs_level = 6;
  cfg.short_protection.s2g_level = 4;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Configure StallGuard2 for bounds finding using test defaults
  tmc5160::StallGuardConfig sg_config{};
  sg_config.sgt = Test::StallGuard::SGT_HOMING; 
  sg_config.sfilt = Test::StallGuard::FILTER_ENABLED;
  sg_config.semin = Test::StallGuard::SEMIN;
  sg_config.semax = Test::StallGuard::SEMAX;

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
  // Steps per output revolution = Motor Full Steps * Gear Ratio * Microsteps
  // 200 * 5.18 * 256 = ~265,216 steps/rev
  float steps_per_rev = static_cast<float>(Motor::OUTPUT_FULL_STEPS) * 256.0f; 
  
  // ============================================================
  // STEP 1: Find global bounds using sensorless homing
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                    STEP 1: Finding Global Bounds                            ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

  // Find minimum bound (negative direction)
  ESP_LOGI(TAG, "Finding minimum bound (negative direction)...");
  int32_t min_position = 0;
  float search_speed = Test::Motion::BOUNDS_SEARCH_SPEED; // steps/s (needs to be fast enough for BEMF)

  driver.rampControl.SetCurrentPosition(0);

  // Use updated PerformSensorlessHoming which includes timeout and polling
  // Pass updated timeout from config
  if (!driver.diagnostics.PerformSensorlessHoming(false, Test::StallGuard::SGT_HOMING, search_speed, 
                                                  min_position, Test::Motion::HOMING_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "Failed to find minimum bound (timeout or no stall), using default");
    min_position = driver.rampControl.GetCurrentPosition();
    // If no stall, assume unbounded or problem
  } else {
    ESP_LOGI(TAG, "Stall detected! Minimum position: %ld steps", min_position);
    
    // Back off
    driver.rampControl.Stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Moving away from minimum stop...");
    
    // Move away relative (e.g. 5000 steps = ~7 degrees at output)
    int32_t backoff = 5000;
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(min_position + backoff);
    driver.rampControl.SetMaxSpeed(search_speed / 2.0f);
    driver.rampControl.SetAcceleration(search_speed);
    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Reset position to 0 after backoff for cleaner coordinates? 
    // Or keep absolute. Let's keep absolute but update min_position reference.
    min_position = driver.rampControl.GetCurrentPosition();
    driver.rampControl.SetCurrentPosition(0); // Reset 0 to this new "safe" minimum
    min_position = 0; 
  }
  
  // Find maximum bound (positive direction)
  ESP_LOGI(TAG, "Finding maximum bound (positive direction)...");
  int32_t max_position = 0;
  
  if (!driver.diagnostics.PerformSensorlessHoming(true, Test::StallGuard::SGT_HOMING, search_speed, 
                                                  max_position, Test::Motion::HOMING_TIMEOUT_MS)) {
     ESP_LOGW(TAG, "Failed to find maximum bound, using default");
     max_position = driver.rampControl.GetCurrentPosition();
  } else {
    ESP_LOGI(TAG, "Stall detected! Maximum position: %ld steps", max_position);
    
    driver.rampControl.Stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Moving away from maximum stop...");
    
    int32_t backoff = 5000;
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(max_position - backoff);
    driver.rampControl.SetMaxSpeed(search_speed / 2.0f);
    driver.rampControl.SetAcceleration(search_speed);
    while (!driver.rampControl.IsTargetReached()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    max_position = driver.rampControl.GetCurrentPosition();
  }
  
  // Note: We replaced the manual polling loops with the robust PerformSensorlessHoming call
  // which handles the timeout and status checks internally now.
  
  bool stall_detected_min = (min_position == 0); // Since we reset to 0 at min
  bool stall_detected_max = (max_position > 10000); // Assume valid if moved significantly


  // ============================================================
  // STEP 2: Set up global bounds and home
  // ============================================================
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║              STEP 2: Setting Global Bounds and Home                        ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

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
    driver.rampControl.SetMaxSpeed(1000.0F);
    driver.rampControl.SetAcceleration(2000.0F);

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
    ESP_LOGI(TAG, "Global bounds: min=%.2f°, max=%.2f° from center", min_deg, max_deg);
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
  motion.SetDwellTimes(dwell, dwell, 0);   // No dwell at center

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
  parser.RegisterCommand({"-d", "--dwell", "Set dwell times in ms (min max [center])", 2, 3}, HandleDwell);
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
      int32_t pos = driver.rampControl.GetCurrentPosition();
      float speed = driver.rampControl.GetCurrentSpeed();
      float pos_deg = tmc5160::StepsToDegrees(pos, steps_per_rev);
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
