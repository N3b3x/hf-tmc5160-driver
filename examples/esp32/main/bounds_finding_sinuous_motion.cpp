/**
 * @file bounds_finding_sinuous_motion.cpp
 * @brief Comprehensive example: Sensorless bounds finding and sinuous motion
 *
 * This example demonstrates:
 * 1. Finding motor bounds using sensorless homing (both directions)
 * 2. Setting the middle position as home
 * 3. Performing sinuous motion between bounds with:
 *    - Configurable angle amplitude and frequency
 *    - Customizable wait times at ends and middle
 *    - User-defined waypoints with wait times
 *    - Ability to add/remove waypoints
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 * - Mechanical stops at both ends for bounds finding
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
#include <vector>

static const char *TAG = "BoundsSinuous";

/**
 * @brief Structure for waypoint with wait time
 */
struct Waypoint {
  int32_t position;  // Position in steps
  uint32_t wait_ms;  // Wait time in milliseconds

  Waypoint(int32_t pos, uint32_t wait) : position(pos), wait_ms(wait) {}
};

/**
 * @brief Sinuous motion controller with bounds and waypoints
 */
class BoundedSinuousMotion {
private:
  tmc5160::TMC5160<Esp32SPI> *driver_;
  int32_t min_bound_;      // Minimum position (negative end)
  int32_t max_bound_;      // Maximum position (positive end)
  int32_t home_position_;  // Home position (middle)
  float amplitude_;        // Amplitude in steps
  float frequency_hz_;     // Frequency in Hz
  bool running_;
  uint32_t start_time_us_;
  int32_t current_target_;
  float phase_offset_;

  // Default wait points (always exist, can be set to 0)
  uint32_t wait_at_min_ms_;  // Wait time at minimum bound
  uint32_t wait_at_max_ms_;  // Wait time at maximum bound
  uint32_t wait_at_home_ms_; // Wait time at home/middle

  // User-defined waypoints
  std::vector<Waypoint> waypoints_;

  // State machine
  enum class MotionState {
    MOVING_TO_MIN,
    WAITING_AT_MIN,
    MOVING_TO_MAX,
    WAITING_AT_MAX,
    MOVING_TO_HOME,
    WAITING_AT_HOME,
    MOVING_TO_WAYPOINT,
    WAITING_AT_WAYPOINT,
    SINUOUS_MOTION,
    STOPPED
  };
  MotionState state_;
  size_t current_waypoint_idx_;
  uint32_t wait_start_time_ms_;

public:
  BoundedSinuousMotion(tmc5160::TMC5160<Esp32SPI> *driver)
      : driver_(driver), min_bound_(0), max_bound_(0), home_position_(0),
        amplitude_(1000.0f), frequency_hz_(0.5f), running_(false),
        start_time_us_(0), current_target_(0), phase_offset_(0.0f),
        wait_at_min_ms_(500), wait_at_max_ms_(500), wait_at_home_ms_(300),
        state_(MotionState::STOPPED), current_waypoint_idx_(0),
        wait_start_time_ms_(0) {}

  /**
   * @brief Set bounds (must be called before starting motion)
   */
  void SetBounds(int32_t min_bound, int32_t max_bound) {
    min_bound_ = min_bound;
    max_bound_ = max_bound;
    home_position_ = (min_bound + max_bound) / 2;
    ESP_LOGI(TAG, "Bounds set: min=%d, max=%d, home=%d", min_bound_, max_bound_,
             home_position_);
  }

  /**
   * @brief Set sinuous motion parameters
   */
  void SetSinuousParams(float amplitude_steps, float frequency_hz) {
    amplitude_ = amplitude_steps;
    frequency_hz_ = frequency_hz;
    ESP_LOGI(TAG, "Sinuous params: amplitude=%.1f steps, frequency=%.2f Hz",
             amplitude_, frequency_hz_);
  }

  /**
   * @brief Set default wait times (can be 0 to disable)
   */
  void SetDefaultWaits(uint32_t wait_at_min_ms, uint32_t wait_at_max_ms,
                       uint32_t wait_at_home_ms) {
    wait_at_min_ms_ = wait_at_min_ms;
    wait_at_max_ms_ = wait_at_max_ms;
    wait_at_home_ms_ = wait_at_home_ms;
    ESP_LOGI(TAG, "Default waits: min=%lu ms, max=%lu ms, home=%lu ms",
             wait_at_min_ms_, wait_at_max_ms_, wait_at_home_ms_);
  }

  /**
   * @brief Add a waypoint with wait time
   * @return true if added successfully
   */
  bool AddWaypoint(int32_t position, uint32_t wait_ms) {
    // Validate position is within bounds
    if (position < min_bound_ || position > max_bound_) {
      ESP_LOGW(TAG, "Waypoint position %d is outside bounds [%d, %d]",
               position, min_bound_, max_bound_);
      return false;
    }

    waypoints_.emplace_back(position, wait_ms);
    ESP_LOGI(TAG, "Added waypoint: position=%d, wait=%lu ms", position,
             wait_ms);
    return true;
  }

  /**
   * @brief Remove waypoint at index
   * @return true if removed successfully
   */
  bool RemoveWaypoint(size_t index) {
    if (index >= waypoints_.size()) {
      return false;
    }
    waypoints_.erase(waypoints_.begin() + index);
    ESP_LOGI(TAG, "Removed waypoint at index %zu", index);
    return true;
  }

  /**
   * @brief Clear all waypoints
   */
  void ClearWaypoints() {
    waypoints_.clear();
    ESP_LOGI(TAG, "Cleared all waypoints");
  }

  /**
   * @brief Get number of waypoints
   */
  size_t GetWaypointCount() const { return waypoints_.size(); }

  /**
   * @brief Get current bounds
   */
  void GetBounds(int32_t &min_bound, int32_t &max_bound) const {
    min_bound = min_bound_;
    max_bound = max_bound_;
  }

  /**
   * @brief Get home position
   */
  int32_t GetHomePosition() const { return home_position_; }

  /**
   * @brief Check if motion is running
   */
  bool IsRunning() const { return running_; }

  /**
   * @brief Start sinuous motion
   */
  void Start() {
    if (min_bound_ == 0 && max_bound_ == 0) {
      ESP_LOGE(TAG, "Cannot start: bounds not set!");
      return;
    }

    running_ = true;
    current_waypoint_idx_ = 0;
    start_time_us_ = esp_timer_get_time();
    phase_offset_ = 0.0f;

    // If waypoints exist, visit them first, otherwise go to home
    if (!waypoints_.empty()) {
      state_ = MotionState::MOVING_TO_WAYPOINT;
      int32_t target = waypoints_[0].position;
      driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(target);
      driver_->rampControl.SetMaxSpeed(1000.0f);
      driver_->rampControl.SetAcceleration(2000.0f);
      ESP_LOGI(TAG, "Starting waypoint sequence, then sinuous motion");
    } else {
      // No waypoints, go directly to home then start sinuous motion
      state_ = MotionState::MOVING_TO_HOME;
      driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(home_position_);
      driver_->rampControl.SetMaxSpeed(1000.0f);
      driver_->rampControl.SetAcceleration(2000.0f);
      ESP_LOGI(TAG, "Starting sinuous motion from home position");
    }
  }

  /**
   * @brief Stop sinuous motion
   */
  void Stop() {
    running_ = false;
    state_ = MotionState::STOPPED;
    driver_->rampControl.Stop();
    ESP_LOGI(TAG, "Stopped sinuous motion");
  }

  /**
   * @brief Update motion (call this in main loop)
   */
  void Update() {
    if (!running_ || state_ == MotionState::STOPPED) {
      return;
    }

    uint32_t current_time_ms = esp_timer_get_time() / 1000;

    switch (state_) {
    case MotionState::MOVING_TO_MIN:
      if (driver_->rampControl.IsTargetReached()) {
        ESP_LOGI(TAG, "Reached minimum bound");
        if (wait_at_min_ms_ > 0) {
          state_ = MotionState::WAITING_AT_MIN;
          wait_start_time_ms_ = current_time_ms;
        } else {
          // Resume sinuous motion immediately
          state_ = MotionState::SINUOUS_MOTION;
        }
      }
      break;

    case MotionState::WAITING_AT_MIN:
      if (current_time_ms - wait_start_time_ms_ >= wait_at_min_ms_) {
        // Resume sinuous motion after waiting at minimum bound
        // Reset timing and set phase to start from minimum (sin = -1, angle = -π/2)
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        phase_offset_ = -M_PI / 2.0; // Start from minimum position
      }
      break;

    case MotionState::MOVING_TO_MAX:
      if (driver_->rampControl.IsTargetReached()) {
        ESP_LOGI(TAG, "Reached maximum bound");
        if (wait_at_max_ms_ > 0) {
          state_ = MotionState::WAITING_AT_MAX;
          wait_start_time_ms_ = current_time_ms;
        } else {
          // Resume sinuous motion immediately
          state_ = MotionState::SINUOUS_MOTION;
        }
      }
      break;

    case MotionState::WAITING_AT_MAX:
      if (current_time_ms - wait_start_time_ms_ >= wait_at_max_ms_) {
        // Resume sinuous motion after waiting at maximum bound
        // Reset timing and set phase to start from maximum (sin = +1, angle = π/2)
        state_ = MotionState::SINUOUS_MOTION;
        start_time_us_ = esp_timer_get_time();
        phase_offset_ = M_PI / 2.0; // Start from maximum position
      }
      break;

    case MotionState::MOVING_TO_HOME:
      if (driver_->rampControl.IsTargetReached()) {
        ESP_LOGI(TAG, "Reached home position");
        if (wait_at_home_ms_ > 0) {
          state_ = MotionState::WAITING_AT_HOME;
          wait_start_time_ms_ = current_time_ms;
        } else {
          StartSinuousMotion();
        }
      }
      break;

    case MotionState::WAITING_AT_HOME:
      if (current_time_ms - wait_start_time_ms_ >= wait_at_home_ms_) {
        StartSinuousMotion();
      }
      break;

    case MotionState::MOVING_TO_WAYPOINT:
      if (driver_->rampControl.IsTargetReached()) {
        HandleWaypointReached();
      }
      break;

    case MotionState::WAITING_AT_WAYPOINT:
      if (current_time_ms - wait_start_time_ms_ >=
          waypoints_[current_waypoint_idx_].wait_ms) {
        current_waypoint_idx_++;
        // After visiting all waypoints, start sinuous motion
        // (waypoints don't loop - they're visited once before sinuous motion)
        StartSinuousMotion();
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
   * @brief Start sinuous motion pattern
   */
  void StartSinuousMotion() {
    // Check if we should visit more waypoints
    if (!waypoints_.empty() && current_waypoint_idx_ < waypoints_.size()) {
      // Move to next waypoint
      int32_t target = waypoints_[current_waypoint_idx_].position;
      driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(target);
      driver_->rampControl.SetMaxSpeed(1000.0f);
      driver_->rampControl.SetAcceleration(2000.0f);
      state_ = MotionState::MOVING_TO_WAYPOINT;
      ESP_LOGI(TAG, "Moving to waypoint %zu at position %d",
               current_waypoint_idx_, target);
      return;
    }

    // All waypoints visited (or none exist), start sinuous motion
    state_ = MotionState::SINUOUS_MOTION;
    start_time_us_ = esp_timer_get_time();
    phase_offset_ = 0.0f;
    ESP_LOGI(TAG, "Starting sinuous motion between bounds [%d, %d]",
             min_bound_, max_bound_);
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

    // Calculate target position (clamped to bounds)
    int32_t target = home_position_ +
                     static_cast<int32_t>(amplitude_ * sin_value);

    // Clamp to bounds and handle wait states
    if (target < min_bound_) {
      target = min_bound_;
      // Hit minimum bound, wait if configured
      if (wait_at_min_ms_ > 0 && state_ == MotionState::SINUOUS_MOTION) {
        state_ = MotionState::WAITING_AT_MIN;
        wait_start_time_ms_ = esp_timer_get_time() / 1000;
        // Stop motion while waiting
        driver_->rampControl.SetTargetPosition(target);
        return;
      }
    } else if (target > max_bound_) {
      target = max_bound_;
      // Hit maximum bound, wait if configured
      if (wait_at_max_ms_ > 0 && state_ == MotionState::SINUOUS_MOTION) {
        state_ = MotionState::WAITING_AT_MAX;
        wait_start_time_ms_ = esp_timer_get_time() / 1000;
        // Stop motion while waiting
        driver_->rampControl.SetTargetPosition(target);
        return;
      }
    }

    // Update target position if it changed significantly
    int32_t current_pos = driver_->rampControl.GetCurrentPosition();
    if (abs(target - current_pos) > 10) { // Update threshold: 10 steps
      driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
      driver_->rampControl.SetTargetPosition(target);
      driver_->rampControl.SetMaxSpeed(1000.0f);
      driver_->rampControl.SetAcceleration(2000.0f);
    }
  }

  /**
   * @brief Handle waypoint reached
   */
  void HandleWaypointReached() {
    if (current_waypoint_idx_ < waypoints_.size()) {
      uint32_t wait_time = waypoints_[current_waypoint_idx_].wait_ms;
      ESP_LOGI(TAG, "Reached waypoint %zu at position %d (wait: %lu ms)",
               current_waypoint_idx_, waypoints_[current_waypoint_idx_].position,
               wait_time);
      if (wait_time > 0) {
        state_ = MotionState::WAITING_AT_WAYPOINT;
        wait_start_time_ms_ = esp_timer_get_time() / 1000;
      } else {
        // No wait time, move to next waypoint immediately
        current_waypoint_idx_++;
        StartSinuousMotion();
      }
    } else {
      // No more waypoints, start sinuous motion
      StartSinuousMotion();
    }
  }
};

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Bounds Finding and Sinuous Motion Example");

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

  // ============================================================
  // STEP 1: Find bounds using sensorless homing
  // ============================================================
  ESP_LOGI(TAG, "=== Step 1: Finding bounds ===");

  // Find minimum bound (negative direction)
  ESP_LOGI(TAG, "Finding minimum bound (negative direction)...");
  int32_t min_position = 0;
  float search_speed = 500.0f; // steps/s

  // Set initial position to a safe starting point
  driver.rampControl.SetCurrentPosition(0);

  // Perform sensorless homing in negative direction
  if (!driver.diagnostics.PerformSensorlessHoming(false, // negative direction
                                                    -10,   // stall threshold
                                                    search_speed, min_position)) {
    ESP_LOGE(TAG, "Failed to find minimum bound");
    return;
  }

  // Wait for movement to complete and stall detection
  ESP_LOGI(TAG, "Waiting for stall detection...");
  uint32_t timeout_ms = 30000; // 30 second timeout
  uint32_t start_time = esp_timer_get_time() / 1000;
  bool stall_detected = false;

  while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
    uint16_t sg_value = driver.diagnostics.GetStallGuard();
    int32_t current_pos = driver.rampControl.GetCurrentPosition();

    // Check if motor has stopped (stall detected)
    if (driver.rampControl.IsTargetReached() || sg_value < 100) {
      min_position = current_pos;
      stall_detected = true;
      ESP_LOGI(TAG, "Stall detected! Minimum position: %d steps", min_position);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!stall_detected) {
    ESP_LOGW(TAG, "Timeout waiting for stall, using current position");
    min_position = driver.rampControl.GetCurrentPosition();
  }

  driver.rampControl.Stop();
  vTaskDelay(pdMS_TO_TICKS(500));

  // Move away from stop slightly
  ESP_LOGI(TAG, "Moving 100 steps away from minimum stop...");
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(min_position + 100);
  driver.rampControl.SetMaxSpeed(500.0f);
  driver.rampControl.SetAcceleration(1000.0f);

  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  min_position = driver.rampControl.GetCurrentPosition();
  driver.rampControl.SetCurrentPosition(0); // Reset position counter
  ESP_LOGI(TAG, "Minimum bound found: %d steps (offset from new zero)",
           min_position);

  // Find maximum bound (positive direction)
  ESP_LOGI(TAG, "Finding maximum bound (positive direction)...");

  int32_t max_position = 0;
  if (!driver.diagnostics.PerformSensorlessHoming(true, // positive direction
                                                   -10,  // stall threshold
                                                   search_speed, max_position)) {
    ESP_LOGE(TAG, "Failed to find maximum bound");
    return;
  }

  // Wait for movement to complete and stall detection
  ESP_LOGI(TAG, "Waiting for stall detection...");
  start_time = esp_timer_get_time() / 1000;
  stall_detected = false;

  while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
    uint16_t sg_value = driver.diagnostics.GetStallGuard();
    int32_t current_pos = driver.rampControl.GetCurrentPosition();

    if (driver.rampControl.IsTargetReached() || sg_value < 100) {
      max_position = current_pos;
      stall_detected = true;
      ESP_LOGI(TAG, "Stall detected! Maximum position: %d steps",
               max_position);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!stall_detected) {
    ESP_LOGW(TAG, "Timeout waiting for stall, using current position");
    max_position = driver.rampControl.GetCurrentPosition();
  }

  driver.rampControl.Stop();
  vTaskDelay(pdMS_TO_TICKS(500));

  // Move away from stop slightly
  ESP_LOGI(TAG, "Moving 100 steps away from maximum stop...");
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(max_position - 100);
  driver.rampControl.SetMaxSpeed(500.0f);
  driver.rampControl.SetAcceleration(1000.0f);

  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  max_position = driver.rampControl.GetCurrentPosition();
  ESP_LOGI(TAG, "Maximum bound found: %d steps", max_position);

  // ============================================================
  // STEP 2: Set middle as home
  // ============================================================
  ESP_LOGI(TAG, "=== Step 2: Setting middle as home ===");

  int32_t middle_position = (min_position + max_position) / 2;
  ESP_LOGI(TAG, "Moving to middle position: %d steps", middle_position);

  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(middle_position);
  driver.rampControl.SetMaxSpeed(1000.0f);
  driver.rampControl.SetAcceleration(2000.0f);

  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Set current position as home (0)
  driver.rampControl.SetCurrentPosition(0);
  ESP_LOGI(TAG, "Home position set to 0 (middle of bounds)");

  // Recalculate bounds relative to new home
  int32_t min_bound = min_position - middle_position;
  int32_t max_bound = max_position - middle_position;
  ESP_LOGI(TAG, "Bounds relative to home: min=%d, max=%d", min_bound,
           max_bound);

  // ============================================================
  // STEP 3: Configure and start sinuous motion
  // ============================================================
  ESP_LOGI(TAG, "=== Step 3: Starting sinuous motion ===");

  // Create sinuous motion controller
  BoundedSinuousMotion motion(&driver);

  // Set bounds
  motion.SetBounds(min_bound, max_bound);

  // Configure sinuous motion parameters
  float amplitude = (max_bound - min_bound) / 2.0f * 0.8f; // 80% of half range
  float frequency = 0.5f; // 0.5 Hz (2 second period)
  motion.SetSinuousParams(amplitude, frequency);

  // Configure default wait times (user can set these to 0 to disable)
  motion.SetDefaultWaits(500,  // Wait 500ms at minimum bound
                         500,  // Wait 500ms at maximum bound
                         300); // Wait 300ms at home/middle

  // Add some example waypoints (optional)
  int32_t waypoint1 = min_bound / 3;
  int32_t waypoint2 = max_bound / 3;
  motion.AddWaypoint(waypoint1, 200); // Wait 200ms at waypoint 1
  motion.AddWaypoint(waypoint2, 200); // Wait 200ms at waypoint 2

  ESP_LOGI(TAG, "Sinuous motion configured:");
  ESP_LOGI(TAG, "  Amplitude: %.1f steps", amplitude);
  ESP_LOGI(TAG, "  Frequency: %.2f Hz", frequency);
  ESP_LOGI(TAG, "  Waypoints: %zu", motion.GetWaypointCount());

  // Start sinuous motion
  motion.Start();

  // Main loop - update motion controller
  ESP_LOGI(TAG, "Running sinuous motion (press reset to stop)...");
  while (true) {
    motion.Update();
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms update rate

    // Optional: Log position periodically
    static uint32_t last_log_time = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_log_time > 1000) { // Log every second
      int32_t pos = driver.rampControl.GetCurrentPosition();
      float speed = driver.rampControl.GetCurrentSpeed();
      ESP_LOGI(TAG, "Position: %d steps, Speed: %.1f steps/s", pos, speed);
      last_log_time = current_time;
    }
  }
}
