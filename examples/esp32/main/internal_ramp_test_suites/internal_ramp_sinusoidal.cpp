/**
 * @file internal_ramp_sinusoidal.cpp
 * @brief Sinusoidal motion pattern example for TMC51x0 stepper motor driver
 *
 * This example demonstrates sinusoidal motion control using the TMC51x0's
 * internal ramp generator. The motor velocity varies in a sinusoidal pattern
 * using velocity mode control.
 *
 * MOTOR SELECTION:
 * Motor selection is done via a static constexpr variable at the top of this file.
 * See esp32_tmc51x0_bus_config.hpp for detailed motor specifications and selection guide.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC51x0 stepper motor driver
 * - Stepper motor connected to TMC51x0 (see motor selection above)
 * - SPI connection between ESP32 and TMC51x0
 * - Chip must be in SPI_INTERNAL_RAMP mode (SPI_MODE=HIGH, SD_MODE=LOW)
 * - Power supply: 12-36V DC (ensure adequate current capacity for selected motor)
 *
 * Pin Configuration (using standard test config):
 * - SPI: MOSI=6, MISO=2, SCLK=5, CS=18
 * - Control: EN=11
 * - Clock: CLK=10
 * - Diagnostics: DIAG0=23, DIAG1=15
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// See esp32_tmc51x0_test_config.hpp for detailed motor, board, and platform specifications.
// Change the values below to select different configurations:
//
// TEST RIG CONFIGURATION:
// This example is configured for the FATIGUE TEST RIG which uses:
// - Applied Motion 5034-369 motor (direct drive, NEMA 34)
// - Used for sinusoidal motion testing
//
// For the CORE DRIVER TEST RIG (gearbox motor), change to MOTOR_17HS4401S_GEARBOX

// Test rig selection (compile-time constant) - automatically selects motor, board, and platform
// FATIGUE TEST RIG: Uses Applied Motion 5034-369 motor, TMC51x0 EVAL board, reference switches, encoder
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;

static const char* TAG = "Sinusoidal";

/**
 * @brief Back-and-forth motion controller using positioning mode
 *
 * Simple back-and-forth motion using TMC51x0's positioning mode.
 * Sets target position to one end, waits until reached, then sets target to other end.
 * Repeats continuously.
 */
class BackAndForthMotion {
private:
  tmc51x0::TMC51x0<Esp32SPI>* driver_;
  float max_velocity_rpm_;        // Maximum velocity in RPM
  float acceleration_rev_s2_;        // Acceleration in rev/s²
  float travel_distance_deg_;  // Distance to travel in each direction (in degrees)
  float center_position_deg_;   // Center position (starting point) in degrees
  float target_position_deg_;   // Current target position in degrees
  bool moving_forward_;        // Direction flag (true = forward, false = backward)
  bool initialized_;
  uint32_t cycles_completed_; // Number of complete back-and-forth cycles
  int max_cycles_;            // Maximum cycles (-1 for infinite)

public:
  BackAndForthMotion(tmc51x0::TMC51x0<Esp32SPI>* driver)
      : driver_(driver), max_velocity_rpm_(30.0f), acceleration_rev_s2_(1.0f),
        travel_distance_deg_(180.0f), center_position_deg_(0.0f), target_position_deg_(0.0f),
        moving_forward_(true), initialized_(false), 
        cycles_completed_(0), max_cycles_(-1) {}

  /**
   * @brief Configure back-and-forth motion parameters
   * @param max_vel_rpm Maximum velocity in RPM
   * @param accel_rev_s2 Acceleration in rev/s²
   * @param travel_dist_deg Distance to travel in each direction (in degrees)
   * @param max_cycles Maximum number of back-and-forth cycles (-1 for infinite)
   */
  void Config(float max_vel_rpm, float accel_rev_s2, float travel_dist_deg, int max_cycles = -1) {
    max_velocity_rpm_ = max_vel_rpm;
    acceleration_rev_s2_ = accel_rev_s2;
    travel_distance_deg_ = travel_dist_deg;
    max_cycles_ = max_cycles;
    initialized_ = false;
    cycles_completed_ = 0;
  }

  /**
   * @brief Initialize and start back-and-forth motion
   */
  void Start() {
    if (initialized_) {
      return;
    }

    // Set acceleration and deceleration
    driver_->rampControl.SetAcceleration(acceleration_rev_s2_, tmc51x0::Unit::RevPerSec);
    driver_->rampControl.SetDeceleration(acceleration_rev_s2_, tmc51x0::Unit::RevPerSec);
    
    // Set start/stop velocities in RPM
    // VSTART: 1000 steps/s ≈ 30 RPM for 200 steps/rev motor (driver handles conversion)
    // VSTOP: 100 steps/s ≈ 3 RPM for 200 steps/rev motor
    float vstart_rpm = 30.0f;  // Start velocity in RPM
    float vstop_rpm = 3.0f;    // Stop velocity in RPM
    driver_->rampControl.SetRampSpeeds(vstart_rpm, vstop_rpm, 0.0f, tmc51x0::Unit::RPM);
    
    // Set maximum velocity
    driver_->rampControl.SetMaxSpeed(max_velocity_rpm_, tmc51x0::Unit::RPM);
    
    // Set to positioning mode
    driver_->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    
    // Get current position as center
    auto center_pos_result = driver_->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
    float center_pos_deg = 0.0f;
    if (!center_pos_result) {
      ESP_LOGW(TAG, "⚠ Failed to read center position (ErrorCode: %d), using 0", static_cast<int>(center_pos_result.Error()));
      center_pos_deg = 0.0f; // Default to 0 if read fails
    } else {
      center_pos_deg = center_pos_result.Value();
    }
    center_position_deg_ = center_pos_deg;
    
    // Start by moving forward (positive direction)
    moving_forward_ = true;
    target_position_deg_ = center_position_deg_ + travel_distance_deg_;
    driver_->rampControl.SetTargetPosition(target_position_deg_, tmc51x0::Unit::Deg);
    
    initialized_ = true;
    cycles_completed_ = 0;
    
    ESP_LOGI(TAG, "Back-and-forth motion started:");
    ESP_LOGI(TAG, "  Max velocity: %.1f RPM", max_velocity_rpm_);
    ESP_LOGI(TAG, "  Acceleration: %.2f rev/s²", acceleration_rev_s2_);
    ESP_LOGI(TAG, "  Travel distance: %.2f degrees per direction", travel_distance_deg_);
    ESP_LOGI(TAG, "  Center position: %.2f degrees", center_position_deg_);
    ESP_LOGI(TAG, "  Target position: %.2f degrees", target_position_deg_);
  }

  /**
   * @brief Update back-and-forth motion (call periodically)
   * @return true if motion is active, false if completed
   */
  bool Update() {
    if (!initialized_) {
      Start();
      return true;
    }

    // Check if target position has been reached
    if (driver_->rampControl.IsTargetReached()) {
      // Target reached - switch direction
      if (moving_forward_) {
        // Just finished moving forward, now move backward
        moving_forward_ = false;
        target_position_deg_ = center_position_deg_ - travel_distance_deg_;
        driver_->rampControl.SetTargetPosition(target_position_deg_, tmc51x0::Unit::Deg);
        ESP_LOGI(TAG, "Reached forward end, reversing to position %.2f degrees", target_position_deg_);
      } else {
        // Just finished moving backward, now move forward
        moving_forward_ = true;
        target_position_deg_ = center_position_deg_ + travel_distance_deg_;
        driver_->rampControl.SetTargetPosition(target_position_deg_, tmc51x0::Unit::Deg);
        cycles_completed_++;
        ESP_LOGI(TAG, "Reached backward end, reversing to position %.2f degrees (cycle %lu complete)", 
                 target_position_deg_, cycles_completed_);
        
        // Check if we've completed the requested number of cycles
        if (max_cycles_ > 0 && cycles_completed_ >= static_cast<uint32_t>(max_cycles_)) {
          ESP_LOGI(TAG, "Back-and-forth motion completed: %lu cycles", cycles_completed_);
          Stop();
          return false;
        }
      }
    }
    
    return true;
  }

  /**
   * @brief Stop back-and-forth motion
   */
  void Stop() {
    driver_->rampControl.SetRampMode(tmc51x0::RampMode::HOLD);
    driver_->rampControl.SetMaxSpeed(0.0f, tmc51x0::Unit::RPM);
    initialized_ = false;
    ESP_LOGI(TAG, "Back-and-forth motion stopped after %lu cycles", cycles_completed_);
  }

  /**
   * @brief Get number of completed cycles
   */
  uint32_t GetCyclesCompleted() const {
    return cycles_completed_;
  }
};

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC51x0 Back-and-Forth Motion Example for NEMA 44mm Motors");
  ESP_LOGI(TAG, "Using internal ramp generator with positioning control");

  // Get standard pin configuration
  auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();

  // Create SPI communication interface with pin configuration
  // Check if EN pin needs to be inverted (some boards have inverters on EN pin)
  // Default: EN is active LOW (LOW = enable, HIGH = disable) per TMC51x0 datasheet
  // If your board has an inverter, set en = true in PinActiveLevels
  tmc51x0::PinActiveLevels active_levels; // Uses defaults: en=false (LOW=enable)
  
  // Uncomment the line below if your board has an inverter on the EN pin:
  // active_levels.en = true; // EN pin has inverter, so ACTIVE = HIGH to enable
  
  Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, 1000000, active_levels); // 1 MHz SPI clock (reduced for stability)

  // Initialize SPI interface
  auto spi_init_result = spi.Initialize();
  if (!spi_init_result) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface (ErrorCode: %d)", static_cast<int>(spi_init_result.Error()));
    return;
  }

  // Create TMC51x0 driver instance
  tmc51x0::TMC51x0<Esp32SPI> driver(spi);

  // Configure driver from unified test rig selection
  tmc51x0::DriverConfig cfg{};
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
  
  // Get motor configuration for later use (output steps calculation)
  uint16_t output_full_steps = 0;
  float gear_ratio = 1.0f;
  constexpr auto motor_type = tmc51x0_test_config::GetTestRigMotorType<SELECTED_TEST_RIG>();
  if constexpr (motor_type == tmc51x0_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    using Motor = tmc51x0_test_config::MotorConfig_17HS4401S;
    output_full_steps = Motor::OUTPUT_FULL_STEPS;
    gear_ratio = Motor::GEAR_RATIO;
    ESP_LOGI(TAG, "Test Rig: Core Driver (17HS4401S with 5.18:1 gearbox)");
  } else if constexpr (motor_type == tmc51x0_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    using Motor = tmc51x0_test_config::MotorConfig_17HS4401S_Direct;
    output_full_steps = Motor::OUTPUT_FULL_STEPS;
    gear_ratio = Motor::GEAR_RATIO;
    ESP_LOGI(TAG, "Test Rig: Core Driver (17HS4401S direct drive)");
  } else if constexpr (motor_type == tmc51x0_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    using Motor = tmc51x0_test_config::MotorConfig_AppliedMotion_5034_369;
    output_full_steps = Motor::OUTPUT_FULL_STEPS;
    gear_ratio = Motor::GEAR_RATIO;
    ESP_LOGI(TAG, "Test Rig: Fatigue (Applied Motion 5034-369 NEMA 34)");
  }
  
  // Enable StealthChop
  cfg.global_config.en_stealthchop_mode = true; // Enable StealthChop

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC51x0 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");
  
  // Run verification immediately after initialization
  ESP_LOGI(TAG, "Running startup verification...");
  if (!driver.status.VerifySetup()) {
      ESP_LOGW(TAG, "Startup verification flagged issues - check logs above");
  } else {
      ESP_LOGI(TAG, "Startup verification passed");
  }

  ESP_LOGI(TAG, "Motor specifications: rated_current=%u mA, sense_resistor=%u mOhm, supply_voltage=%u mV",
           cfg.motor_spec.rated_current_ma, cfg.motor_spec.sense_resistor_mohm, cfg.motor_spec.supply_voltage_mv);
  ESP_LOGI(TAG, "Note: IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated from motor specifications");
  
  // CLK16 (CLK pin) configuration note:
  // The CLK pin (pin 12) should be TIED TO GND in hardware when using internal clock
  // It is NOT a GPIO output - it's a hardware connection
  // If you're using internal clock (default), ensure CLK pin is connected to GND
  // If you're using external clock, connect your clock source to CLK pin
  ESP_LOGI(TAG, "Note: CLK16 (CLK pin) should be tied to GND for internal clock operation");
  
  // COMPLETELY DISABLE StallGuard2 stop for sinusoidal motion
  // StallGuard2 should NOT stop the motor - it's only for diagnostics/homing
  // We need to disable it in multiple places to ensure it can't interfere
  
  // 1. Disable StallGuard2 stop and soft stop (not needed for continuous motion)
  bool changed = false;
  if (driver.stallGuard.IsStopOnStallEnabled()) {
    ESP_LOGW(TAG, "StallGuard2 stop is ENABLED - disabling for sinusoidal motion");
    if (driver.stallGuard.EnableStopOnStall(false)) {
      changed = true;
    }
  }
  if (driver.stallGuard.IsSoftStopEnabled()) {
    ESP_LOGW(TAG, "Soft stop is ENABLED - disabling for continuous sinusoidal motion");
    if (driver.stallGuard.SetSoftStop(false)) {
      changed = true;
    }
  }
  if (changed) {
    // Verify it was written
    if (!driver.stallGuard.IsStopOnStallEnabled() && !driver.stallGuard.IsSoftStopEnabled()) {
      ESP_LOGI(TAG, "✓ StallGuard2 stop and soft stop confirmed DISABLED");
    } else {
      ESP_LOGE(TAG, "✗ Failed to disable StallGuard2 stop or soft stop!");
    }
  } else {
    ESP_LOGI(TAG, "✓ StallGuard2 stop and soft stop already disabled (correct for sinusoidal motion)");
  }
  
  // 2. Set TCOOLTHRS to 0 to disable StallGuard2 at all speeds
  // TCOOLTHRS = velocity threshold below which StallGuard2 is disabled
  // Enabled if TSTEP < TCOOLTHRS (Velocity > Threshold)
  // To disable, we want Threshold to be infinite (TCOOLTHRS = 0)
  if (driver.thresholds.SetTcoolthrs(0.0f, tmc51x0::Unit::RPM)) {
    ESP_LOGI(TAG, "✓ TCOOLTHRS set to 0 RPM - StallGuard2 disabled at all speeds");
  } else {
    ESP_LOGE(TAG, "✗ Failed to set TCOOLTHRS");
  }
  
  // 3. Set THIGH to maximum to ensure StallGuard2 doesn't interfere
  // THIGH = velocity threshold for chopper mode switching
  // Setting high ensures StallGuard2 doesn't affect operation
  // Maximum value corresponds to very high RPM (driver handles conversion)
  constexpr float MAX_THIGH_RPM = 10000.0f; // Very high RPM threshold
  if (driver.thresholds.SetHighSpeedThreshold(MAX_THIGH_RPM, tmc51x0::Unit::RPM)) {
    ESP_LOGI(TAG, "✓ THIGH set to maximum (%.0f RPM) - ensures StallGuard2 doesn't interfere", MAX_THIGH_RPM);
  } else {
    ESP_LOGW(TAG, "Failed to set THIGH (may not be critical)");
  }
  
  // 4. Configure StallGuard2 threshold to be least sensitive (even though it's disabled)
  // This is just for diagnostics - it won't stop the motor
  // NOTE: StallGuard2 ONLY works in SpreadCycle mode! In StealthChop, SG_RESULT is invalid/zero.
  tmc51x0::StallGuardConfig sg_cfg{};
  sg_cfg.threshold = 63;        // Maximum threshold (least sensitive) - won't trigger
  sg_cfg.enable_filter = false; // No filter (faster response)
  // Note: semin/semax are CoolStep parameters, configure separately if needed
  if (driver.stallGuard.ConfigureStallGuard(sg_cfg)) {
    ESP_LOGI(TAG, "✓ StallGuard2 configured for diagnostics only (sgt=63, least sensitive)");
    ESP_LOGI(TAG, "  Note: StallGuard2 is DISABLED and will NOT stop the motor");
    if (cfg.global_config.en_stealthchop_mode) {
      ESP_LOGW(TAG, "  Note: StealthChop is enabled, so StallGuard2 values will be invalid (usually 0)");
    }
  } else {
    ESP_LOGW(TAG, "Failed to configure StallGuard2 (may not be critical)");
  }
  
  // Disable reference switches if not using them (prevents motion blocking)
  // If you have reference switches connected, configure them instead
  tmc51x0::ReferenceSwitchConfig ref_cfg{};
  // Configure switches but disable motor stop (allows reading switch state without stopping)
  ref_cfg.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
  ref_cfg.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
  ref_cfg.left_switch_stop_enable = false;   // Don't stop motor
  ref_cfg.right_switch_stop_enable = false;  // Don't stop motor
  ref_cfg.latch_left = tmc51x0::ReferenceLatchMode::DISABLED;   // No latching
  ref_cfg.latch_right = tmc51x0::ReferenceLatchMode::DISABLED;  // No latching
  if (!driver.switches.ConfigureReferenceSwitch(ref_cfg)) {
    ESP_LOGW(TAG, "Failed to configure reference switches (may not be critical)");
  } else {
    ESP_LOGI(TAG, "Reference switches disabled (not using endstops)");
    
    // Verify SW_MODE register was written correctly
    // Verify reference switch configuration
    bool right_active = false;
    bool left_enabled = false, right_enabled = false;
    auto ref_switch_result = driver.switches.GetReferenceSwitchStatus(right_active, left_enabled, right_enabled);
    if (ref_switch_result.IsOk()) {
      ESP_LOGI(TAG, "SW_MODE verification: stop_l_enable=%d, stop_r_enable=%d, en_softstop=%d",
               left_enabled ? 1 : 0,
               right_enabled ? 1 : 0,
               driver.stallGuard.IsSoftStopEnabled() ? 1 : 0);
      
      if (left_enabled || right_enabled) {
        ESP_LOGE(TAG, "ERROR: Reference switches still enabled in SW_MODE!");
        ESP_LOGE(TAG, "Motion will be blocked. Re-configuring...");
        // Try again
        driver.switches.ConfigureReferenceSwitch(ref_cfg);
      } else {
        ESP_LOGI(TAG, "✓ Reference switches confirmed disabled in SW_MODE");
      }
    }
  }
  
  // Check physical pin states (if pins are mapped)
  auto ref_left_result = driver.GetComm().GpioRead(tmc51x0::TMC51x0CtrlPin::REFL_STEP);
  bool ref_left_read = ref_left_result.IsOk();
  tmc51x0::GpioSignal ref_left_signal = ref_left_read ? ref_left_result.Value() : tmc51x0::GpioSignal::INACTIVE;
  if (ref_left_read) {
    ESP_LOGI(TAG, "REFL_STEP (left ref) pin state: %s",
             ref_left_signal == tmc51x0::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  auto ref_right_result = driver.GetComm().GpioRead(tmc51x0::TMC51x0CtrlPin::REFR_DIR);
  bool ref_right_read = ref_right_result.IsOk();
  tmc51x0::GpioSignal ref_right_signal = ref_right_read ? ref_right_result.Value() : tmc51x0::GpioSignal::INACTIVE;
  if (ref_right_read) {
    ESP_LOGI(TAG, "REFR_DIR (right ref) pin state: %s",
             ref_right_signal == tmc51x0::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  
  if (ref_left_read && ref_left_signal == tmc51x0::GpioSignal::ACTIVE) {
    ESP_LOGW(TAG, "WARNING: REFL_STEP pin is HIGH - if this is a switch, it may be active");
    ESP_LOGW(TAG, "  If not using switches, ensure pin is pulled LOW or left floating");
  }
  if (ref_right_read && ref_right_signal == tmc51x0::GpioSignal::ACTIVE) {
    ESP_LOGW(TAG, "WARNING: REFR_DIR pin is HIGH - if this is a switch, it may be active");
    ESP_LOGW(TAG, "  If not using switches, ensure pin is pulled LOW or left floating");
  }

  // Verify chip is in internal ramp mode (SPI_MODE=HIGH, SD_MODE=LOW)
  // If mode pins are configured, verify they're set correctly
  if (pin_config.tmc51x0_pins.spi_mode_pin != -1 && pin_config.tmc51x0_pins.sd_mode_pin != -1) {
    auto mode_result = driver.io.GetOperatingMode();
    if (mode_result.IsOk()) {
      tmc51x0::ChipCommMode current_mode = mode_result.Value();
      if (current_mode != tmc51x0::ChipCommMode::SPI_INTERNAL_RAMP) {
        ESP_LOGW(TAG, "Chip is not in SPI_INTERNAL_RAMP mode (current: %d)", static_cast<int>(current_mode));
        ESP_LOGW(TAG, "Setting to SPI_INTERNAL_RAMP mode...");
        if (driver.io.SetOperatingMode(tmc51x0::ChipCommMode::SPI_INTERNAL_RAMP)) {
          ESP_LOGW(TAG, "Mode changed - chip reset required! Power cycle the TMC51x0 now.");
          ESP_LOGW(TAG, "After reset, restart this program.");
          return;
        }
      } else {
        ESP_LOGI(TAG, "Chip verified in SPI_INTERNAL_RAMP mode (SPI_MODE=HIGH, SD_MODE=LOW)");
      }
    }
  } else {
    ESP_LOGW(TAG, "Mode pins not configured - ensure SPI_MODE (pin 22)=HIGH, SD_MODE (pin 21)=LOW");
    ESP_LOGW(TAG, "Chip must be in SPI_INTERNAL_RAMP mode for this example to work");
  }

  // Enable motor driver
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor driver");
    return;
  }
  ESP_LOGI(TAG, "Motor enabled");
  
  // Diagnostic: Check EN pin state to verify enable logic
  auto en_result = spi.GpioRead(tmc51x0::TMC51x0CtrlPin::EN);
  if (en_result.IsOk()) {
    tmc51x0::GpioSignal en_signal = en_result.Value();
    ESP_LOGI(TAG, "EN pin state after Enable(): %s", 
             en_signal == tmc51x0::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
    ESP_LOGI(TAG, "  Note: TMC51x0 DRV_ENN is active LOW (LOW=enable, HIGH=disable)");
    ESP_LOGI(TAG, "  If motor doesn't move, check if your board has an inverter on EN pin");
    ESP_LOGI(TAG, "  If so, configure: active_levels.en = true in PinActiveLevels");
  }
  
  // Check for Charge Pump Undervoltage immediately after enabling
  // Check for critical hardware errors
  bool drv_err = false, uv_cp = false;
  auto global_status_result = driver.status.GetGlobalStatus(drv_err, uv_cp);
  if (global_status_result.IsOk()) {
    bool reset = global_status_result.Value();
    if (uv_cp) {
      ESP_LOGE(TAG, "CRITICAL HARDWARE ERROR: Charge Pump Undervoltage (uv_cp=1) detected immediately!");
      ESP_LOGE(TAG, "  This usually means VSA/VS voltage is too low or the charge pump capacitor is missing/bad.");
      ESP_LOGE(TAG, "  The motor DRIVER STAGE IS DISABLED by the chip protection.");
      ESP_LOGE(TAG, "  Please check your power supply (12-36V) and wiring.");
    }
  }
  
  // Verify motor is enabled
  if (!driver.motorControl.IsEnabled()) {
    ESP_LOGE(TAG, "Motor driver not enabled! Check EN pin connection and Enable() call");
    return;
  } else {
    ESP_LOGI(TAG, "Motor driver verified enabled");
  }
  
  // CRITICAL: Check StealthChop status - if enabled but not calibrated, motor won't move!
  // This was already checked above, but we can verify again if needed
  // (StealthChop check is done in the earlier section)
  
  // Check motor current settings
  // NOTE: IHOLD_IRUN and GLOBAL_SCALER are WRITE-ONLY registers per datasheet!
  // We cannot read them back, so we display the configured values instead.
  ESP_LOGI(TAG, "=== Motor Current Diagnostic ===");
  ESP_LOGI(TAG, "Motor specifications:");
  ESP_LOGI(TAG, "  Rated current: %u mA", cfg.motor_spec.rated_current_ma);
  ESP_LOGI(TAG, "  Sense resistor: %u mOhm", cfg.motor_spec.sense_resistor_mohm);
  ESP_LOGI(TAG, "  Supply voltage: %u mV", cfg.motor_spec.supply_voltage_mv);
  ESP_LOGI(TAG, "Note: IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated during initialization");
  ESP_LOGI(TAG, "Note: IHOLD_IRUN and GLOBAL_SCALER are write-only registers - cannot verify by reading");
  
  // Check if motor specs are adequate
  if (cfg.motor_spec.rated_current_ma < 1000) {
    ESP_LOGW(TAG, "⚠️ Motor rated current (%u mA) may be low - ensure adequate torque");
  } else {
    ESP_LOGI(TAG, "✓ Motor rated current appears adequate (%u mA)", cfg.motor_spec.rated_current_ma);
  }
  
  // Comprehensive DIAG pin diagnostic function
  auto diagnose_diag_pins = [&driver]() {
    ESP_LOGI(TAG, "=== DIAG Pin Diagnostic ===");
    
    // Read DIAG pin states
    auto diag0_result = driver.GetComm().GpioRead(tmc51x0::TMC51x0CtrlPin::DIAG0);
    bool diag0_read = diag0_result.IsOk();
    tmc51x0::GpioSignal diag0_signal = diag0_read ? diag0_result.Value() : tmc51x0::GpioSignal::INACTIVE;
    if (diag0_read) {
      ESP_LOGI(TAG, "DIAG0 pin state: %s", 
               diag0_signal == tmc51x0::GpioSignal::ACTIVE ? "HIGH" : "LOW");
    } else {
      ESP_LOGW(TAG, "DIAG0 pin not configured or read failed");
    }
    
    auto diag1_result = driver.GetComm().GpioRead(tmc51x0::TMC51x0CtrlPin::DIAG1);
    bool diag1_read = diag1_result.IsOk();
    tmc51x0::GpioSignal diag1_signal = diag1_read ? diag1_result.Value() : tmc51x0::GpioSignal::INACTIVE;
    if (diag1_read) {
      ESP_LOGI(TAG, "DIAG1 pin state: %s", 
               diag1_signal == tmc51x0::GpioSignal::ACTIVE ? "HIGH" : "LOW");
    } else {
      ESP_LOGW(TAG, "DIAG1 pin not configured or read failed");
    }
    
    // Read GCONF to see which diagnostic features are enabled
      // Get diagnostic configuration
      auto diag0_config_result = driver.motorControl.GetDiag0Config();
      auto diag1_config_result = driver.motorControl.GetDiag1Config();
      bool has_diag0 = diag0_config_result.IsOk();
      bool has_diag1 = diag1_config_result.IsOk();
      tmc51x0::Diag0Config diag0_config = has_diag0 ? diag0_config_result.Value() : tmc51x0::Diag0Config{};
      tmc51x0::Diag1Config diag1_config = has_diag1 ? diag1_config_result.Value() : tmc51x0::Diag1Config{};
      
      if (has_diag0 || has_diag1) {
        ESP_LOGI(TAG, "Diagnostic settings:");
        if (has_diag0) {
          ESP_LOGI(TAG, "  diag0_error=%d, diag0_otpw=%d, diag0_stall_step=%d",
                   diag0_config.error ? 1 : 0,
                   diag0_config.otpw ? 1 : 0,
                   diag0_config.stall_step ? 1 : 0);
          ESP_LOGI(TAG, "  diag0_pushpull=%d", diag0_config.pushpull ? 1 : 0);
        }
        if (has_diag1) {
          ESP_LOGI(TAG, "  diag1_stall_dir=%d, diag1_index=%d, diag1_onstate=%d, diag1_steps_skipped=%d",
                   diag1_config.stall_dir ? 1 : 0,
                   diag1_config.index ? 1 : 0,
                   diag1_config.onstate ? 1 : 0,
                   diag1_config.steps_skipped ? 1 : 0);
          ESP_LOGI(TAG, "  diag1_pushpull=%d", diag1_config.pushpull ? 1 : 0);
        }
        
        // Read GCONF to check diagnostic pin configuration
        auto gconf_result = driver.GetComm().ReadRegister(tmc51x0::Registers::GCONF, driver.communication.GetDaisyChainPosition());
        bool gconf_read = gconf_result.IsOk();
        uint32_t gconf_value = gconf_read ? gconf_result.Value() : 0;
        tmc51x0::GCONF_Register gconf{};
        if (gconf_read) {
          gconf.value = gconf_value;
        } else {
          // If GCONF read fails, initialize to safe defaults
          gconf.value = 0;
        }
        
        // Read GSTAT for reset and driver errors
        bool drv_err = false, uv_cp = false;
        auto gstat_result = driver.status.GetGlobalStatus(drv_err, uv_cp);
        bool gstat_read = gstat_result.IsOk();
        bool reset = gstat_read ? gstat_result.Value() : false;
        tmc51x0::GSTAT_Register gstat{};
        uint32_t gstat_value = 0;
        auto gstat_read_result = driver.GetComm().ReadRegister(tmc51x0::Registers::GSTAT, driver.communication.GetDaisyChainPosition());
        if (gstat_read_result.IsOk()) {
          gstat_value = gstat_read_result.Value();
          gstat.value = gstat_value;
        }
        
        if (gstat_read) {
          ESP_LOGI(TAG, "GSTAT: reset=%d, drv_err=%d, uv_cp=%d",
                   reset ? 1 : 0,
                   drv_err ? 1 : 0,
                   uv_cp ? 1 : 0);
        
        // DIAG0 always shows reset status (active low during reset)
        if (gstat.bits.reset) {
          ESP_LOGW(TAG, "  → DIAG0 should be LOW (reset condition)");
        }
        if (gstat.bits.drv_err) {
          ESP_LOGE(TAG, "  → Driver error detected! DIAG0 may be LOW if diag0_error enabled");
        }
        if (gstat.bits.uv_cp) {
          ESP_LOGE(TAG, "  → Charge pump undervoltage! DIAG0 may be LOW if diag0_error enabled");
          ESP_LOGE(TAG, "  CRITICAL: Charge pump not working - motor cannot drive properly!");
          ESP_LOGE(TAG, "  Check: VCC power supply (12-36V), charge pump capacitor, power stability");
        }
      }
      
      // Read DRV_STATUS for detailed error information
      tmc51x0::DRV_STATUS_Register drv_status{};
      drv_status.value = 0;
      bool drv_status_read = false;
      auto drv_status_result = driver.status.GetDriverStatusRegister();
      if (drv_status_result) {
        uint32_t drv_status_value = drv_status_result.Value();
        drv_status.value = drv_status_value;
        drv_status_read = true;
        
        ESP_LOGI(TAG, "DRV_STATUS errors:");
        ESP_LOGI(TAG, "  ot=%d (overtemperature), otpw=%d (overtemp prewarning)",
                 drv_status.bits.ot ? 1 : 0,
                 drv_status.bits.otpw ? 1 : 0);
        ESP_LOGI(TAG, "  s2ga=%d, s2gb=%d (short to GND), s2vsa=%d, s2vsb=%d (short to VS)",
                 drv_status.bits.s2ga ? 1 : 0,
                 drv_status.bits.s2gb ? 1 : 0,
                 drv_status.bits.s2vsa ? 1 : 0,
                 drv_status.bits.s2vsb ? 1 : 0);
        ESP_LOGI(TAG, "  stallguard=%d (SG result, 0=highest load, higher=less load)",
                 drv_status.bits.sg_result);
        
        // Check for wiring issues / stall
        // Note: SG_RESULT is generally invalid in StealthChop mode on TMC51x0!
        // Only report wiring issues if in SpreadCycle (en_stealthchop_mode=0)
        if (drv_status.bits.sg_result == 0) {
          if (!gconf.bits.en_pwm_mode) {
            ESP_LOGE(TAG, "  ⚠️ WIRING ISSUE: SG=0 with no load suggests:");
            ESP_LOGE(TAG, "     - Motor phases may be swapped or incorrectly wired");
            ESP_LOGE(TAG, "     - Check: A+/A- and B+/B- connections");
            ESP_LOGE(TAG, "     - Try swapping one phase pair (A+/A- or B+/B-)");
          } else {
            ESP_LOGI(TAG, "  (Note: SG=0 is expected in StealthChop mode - measurement invalid)");
          }
        }
        
        // Correlate with DIAG0
        if (gconf.bits.diag0_error && (drv_status.bits.ot || drv_status.bits.s2ga || 
                                       drv_status.bits.s2gb || drv_status.bits.s2vsa || 
                                       drv_status.bits.s2vsb)) {
          ESP_LOGE(TAG, "  → DIAG0 should be LOW (driver error: OT or short circuit)");
        }
        if (gconf.bits.diag0_otpw && drv_status.bits.otpw) {
          ESP_LOGW(TAG, "  → DIAG0 should be LOW (overtemperature prewarning)");
        }
        if (gconf.bits.diag0_stall_step && drv_status.bits.sg_result < 100) {
          ESP_LOGW(TAG, "  → DIAG0 may be LOW (stall detected, SG=%d)", drv_status.bits.sg_result);
        }
      }
      
      // Read RAMP_STAT for stall and position information
      tmc51x0::RAMP_STAT_Register ramp_stat{};
      ramp_stat.value = 0;
      auto ramp_stat_result = driver.status.GetRampStatusRegister();
      if (ramp_stat_result.IsOk()) {
        ramp_stat.value = ramp_stat_result.Value();
        
        ESP_LOGI(TAG, "RAMP_STAT: status_sg=%d (stall guard active)",
                 ramp_stat.bits.status_sg ? 1 : 0);
        
        if (gconf.bits.diag0_stall_step && ramp_stat.bits.status_sg) {
          ESP_LOGW(TAG, "  → DIAG0 may be LOW (stall detected via RAMP_STAT)");
        }
        if (gconf.bits.diag1_stall_dir && ramp_stat.bits.status_sg) {
          ESP_LOGW(TAG, "  → DIAG1 may be LOW (stall detected via RAMP_STAT)");
        }
      }
      
      // Summary
      ESP_LOGI(TAG, "=== DIAG Pin Summary ===");
      if (diag0_read) {
        bool diag0_active = (diag0_signal == tmc51x0::GpioSignal::ACTIVE);
        bool diag0_expected_low = false;
        
        if (gstat_read) {
          diag0_expected_low = gstat.bits.reset || 
                              (gconf.bits.diag0_error && gstat.bits.drv_err);
        }
        if (drv_status_read) {
          diag0_expected_low = diag0_expected_low ||
                              (gconf.bits.diag0_otpw && drv_status.bits.otpw) ||
                              (gconf.bits.diag0_error && (drv_status.bits.ot || drv_status.bits.s2ga || 
                                                          drv_status.bits.s2gb || drv_status.bits.s2vsa || 
                                                          drv_status.bits.s2vsb));
        }
        
        if (diag0_expected_low && diag0_active) {
          ESP_LOGW(TAG, "DIAG0: HIGH (expected LOW - check pullup or pushpull mode)");
        } else if (!diag0_expected_low && !diag0_active) {
          ESP_LOGW(TAG, "DIAG0: LOW (unexpected - check for errors)");
        } else {
          ESP_LOGI(TAG, "DIAG0: %s (expected)", diag0_active ? "HIGH" : "LOW");
        }
      }
      
      if (diag1_read) {
        bool diag1_active = (diag1_signal == tmc51x0::GpioSignal::ACTIVE);
        ESP_LOGI(TAG, "DIAG1: %s", diag1_active ? "HIGH" : "LOW");
        if (!diag1_active && (gconf.bits.diag1_stall_dir || gconf.bits.diag1_index || 
                              gconf.bits.diag1_onstate)) {
          ESP_LOGI(TAG, "  (DIAG1 diagnostic features enabled - check RAMP_STAT/DRV_STATUS)");
        }
      }
    }
  };
  
  // Run diagnostic
  diagnose_diag_pins();
  
  // Read RAMP_STAT to check for any flags preventing motion
  uint32_t ramp_stat = 0;
  auto ramp_stat_result = driver.status.GetRampStatusRegister();
  if (ramp_stat_result.IsOk()) {
    ramp_stat = ramp_stat_result.Value();
    tmc51x0::RAMP_STAT_Register status{};
    status.value = ramp_stat;
    ESP_LOGI(TAG, "RAMP_STAT: vzero=%d, velocity_reached=%d, position_reached=%d, stop_l=%d, stop_r=%d",
             status.bits.vzero ? 1 : 0,
             status.bits.velocity_reached ? 1 : 0,
             status.bits.position_reached ? 1 : 0,
             status.bits.status_stop_l ? 1 : 0,
             status.bits.status_stop_r ? 1 : 0);
    
    // Check if stops are actually enabled
    bool right_active = false;
    bool left_enabled = false, right_enabled = false;
    bool stops_enabled = false;
    bool left_active = false;
    auto ref_switch_result = driver.switches.GetReferenceSwitchStatus(right_active, left_enabled, right_enabled);
    if (ref_switch_result.IsOk()) {
      left_active = ref_switch_result.Value();
      stops_enabled = left_enabled || right_enabled;
    }
    
    if (left_active || right_active) {
      if (stops_enabled) {
        ESP_LOGE(TAG, "ERROR: Reference switch active AND enabled! stop_l=%d, stop_r=%d", 
                 left_active ? 1 : 0, right_active ? 1 : 0);
        ESP_LOGE(TAG, "This WILL prevent motion in internal ramp mode!");
        ESP_LOGE(TAG, "Solution: Disable reference switches via ConfigureReferenceSwitch()");
      } else {
        ESP_LOGW(TAG, "Reference switch pins are active (stop_l=%d, stop_r=%d) but stops are DISABLED",
                 left_active ? 1 : 0, right_active ? 1 : 0);
        ESP_LOGI(TAG, "Motion should still work since stop_l_enable=0 and stop_r_enable=0 in SW_MODE");
        ESP_LOGI(TAG, "The status bits just reflect pin state - they don't block motion when disabled");
      }
    }
  }
  
  // Read VACTUAL to see if motor is trying to move
  auto velocity_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
  float actual_velocity_rpm = 0.0f;
  if (!velocity_result) {
    ESP_LOGW(TAG, "⚠ Failed to read current speed (ErrorCode: %d), using 0", static_cast<int>(velocity_result.Error()));
    ESP_LOGW(TAG, "Failed to get current speed");
  } else {
    actual_velocity_rpm = velocity_result.Value();
  }
  ESP_LOGI(TAG, "VACTUAL (actual velocity): %.1f RPM", actual_velocity_rpm);
  if (actual_velocity_rpm != 0.0f) {
    ESP_LOGI(TAG, "  ✓ Motor IS moving at %.1f RPM", actual_velocity_rpm);
  } else {
    ESP_LOGW(TAG, "  ✗ Motor is NOT moving (VACTUAL=0)");
  }

  // Create back-and-forth motion controller
  // Driver handles all unit conversions internally based on motor configuration
  BackAndForthMotion motion(&driver);

  // Configure back-and-forth motion for NEMA 44mm motor with gearbox
  // Calculate motion parameters in physical units
  // Travel distance: 1 full output revolution = 360 degrees
  float travel_distance_deg = 360.0f;
  
  // Max velocity: 0.5 RPS = 30 RPM
  float max_velocity_rpm = 30.0f;
  
  // Acceleration: reach max velocity in ~0.2 seconds
  // max_velocity_rpm = 30 RPM = 0.5 rev/s
  // acceleration = 0.5 rev/s / 0.2s = 2.5 rev/s²
  float acceleration_rev_s2 = max_velocity_rpm / 60.0f / 0.2f; // Convert RPM to rev/s, then divide by time
  if (acceleration_rev_s2 < 1.0f) acceleration_rev_s2 = 1.0f; // Minimum acceleration
  
  int max_cycles = -1; // Infinite cycles
  
  motion.Config(max_velocity_rpm, acceleration_rev_s2, travel_distance_deg, max_cycles);

  ESP_LOGI(TAG, "Starting back-and-forth motion for NEMA 44mm motor:");
  ESP_LOGI(TAG, "  Max velocity: %.1f RPM (%.2f rev/s)", max_velocity_rpm, max_velocity_rpm / 60.0f);
  ESP_LOGI(TAG, "  Acceleration: %.2f rev/s²", acceleration_rev_s2);
  ESP_LOGI(TAG, "  Travel distance: %.2f degrees (%.2f output revolutions per direction)", 
           travel_distance_deg, travel_distance_deg / 360.0f);
  ESP_LOGI(TAG, "  Max cycles: %d", max_cycles > 0 ? max_cycles : -1);
  ESP_LOGI(TAG, "  Using internal ramp generator with positioning mode");

  // Run back-and-forth motion
  // Check position periodically and switch direction when target is reached
  uint32_t last_diag_time = 0;
  while (true) {
    if (!motion.Update()) {
      // Motion completed
      ESP_LOGI(TAG, "Motion sequence completed. Restarting in 5 seconds...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      motion.Start(); // Restart
    }
    
    // Periodic diagnostics (every 1 second)
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_diag_time >= 1000) {
      last_diag_time = current_time;
      
      // Read actual velocity
      auto speed_result = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
      float actual_velocity_rpm = 0.0f;
      if (!speed_result) {
        ESP_LOGW(TAG, "⚠ Failed to get current speed (ErrorCode: %d)", static_cast<int>(speed_result.Error()));
      } else {
        actual_velocity_rpm = speed_result.Value();
      }
      
      auto pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
      float actual_pos_deg = 0.0f;
      if (!pos_result) {
        ESP_LOGW(TAG, "⚠ Failed to get current position (ErrorCode: %d)", static_cast<int>(pos_result.Error()));
        actual_pos_deg = 0.0f;
      } else {
        actual_pos_deg = pos_result.Value();
      }
      
      // Read ramp status
      auto ramp_stat_result = driver.status.GetRampStatusRegister();
      bool has_ramp_stat = ramp_stat_result.IsOk();
      uint32_t ramp_stat = has_ramp_stat ? ramp_stat_result.Value() : 0;
      
      // Read DRV_STATUS for stall detection
      uint32_t drv_status_value = 0;
      auto drv_status_result = driver.GetComm().ReadRegister(tmc51x0::Registers::DRV_STATUS, driver.communication.GetDaisyChainPosition());
      bool has_drv_status = drv_status_result.IsOk();
      drv_status_value = has_drv_status ? drv_status_result.Value() : 0;
      tmc51x0::DRV_STATUS_Register drv_status{};
      if (has_drv_status) {
        drv_status.value = drv_status_value;
      }
      
      // Read GSTAT for charge pump status
      uint32_t gstat_value = 0;
      auto gstat_result = driver.GetComm().ReadRegister(tmc51x0::Registers::GSTAT, driver.communication.GetDaisyChainPosition());
      bool has_gstat = gstat_result.IsOk();
      gstat_value = has_gstat ? gstat_result.Value() : 0;
      tmc51x0::GSTAT_Register gstat{};
      if (has_gstat) {
        gstat.value = gstat_value;
      }
      
      // Read GCONF to check StealthChop mode (needed for StallGuard2 interpretation)
      uint32_t gconf_value = 0;
      auto gconf_result = driver.GetComm().ReadRegister(tmc51x0::Registers::GCONF, driver.communication.GetDaisyChainPosition());
      bool has_gconf = gconf_result.IsOk();
      gconf_value = has_gconf ? gconf_result.Value() : 0;
      tmc51x0::GCONF_Register gconf{};
      if (has_gconf) {
        gconf.value = gconf_value;
      }
      
      // Note: VSTART, VSTOP, AMAX are WRITE-ONLY registers - they always read as 0
      // We can't verify them by reading, but we know they were set in BackAndForthMotion::Start()
      // VSTART=1000, VSTOP=100, AMAX=acceleration (from SetRampSpeeds, SetAcceleration, and SetDeceleration)
      
      // Read RAMPMODE to verify we're in positioning mode
      uint32_t rampmode_value = 0;
      bool in_positioning_mode = false;
      auto rampmode_result = driver.rampControl.GetRampMode();
      if (rampmode_result.IsOk()) {
        tmc51x0::RampMode rampmode_enum = rampmode_result.Value();
        uint32_t rampmode_value = static_cast<uint32_t>(rampmode_enum);
        in_positioning_mode = (rampmode_value == 0); // POSITIONING mode
        const char* mode_str = (rampmode_value == 0) ? "POSITIONING" :
                              (rampmode_value == 1) ? "VELOCITY_POS" :
                              (rampmode_value == 2) ? "VELOCITY_NEG" : "HOLD";
        ESP_LOGI(TAG, "  RAMPMODE=%lu (%s)", rampmode_value, mode_str);
        
        if (!in_positioning_mode) {
          ESP_LOGW(TAG, "  ⚠️ WARNING: Not in POSITIONING mode! Expected RAMPMODE=0");
        }
      }
      
      // Read target position
      auto target_pos_result = driver.rampControl.GetTargetPosition(tmc51x0::Unit::Deg);
      if (target_pos_result.IsOk()) {
        float target_pos_deg = target_pos_result.Value();
        if (target_pos_deg != 0.0f) {
          ESP_LOGI(TAG, "  XTARGET (target position): %.2f degrees", target_pos_deg);
          ESP_LOGI(TAG, "  XACTUAL (current position): %.2f degrees", actual_pos_deg);
          float distance_to_target_deg = target_pos_deg - actual_pos_deg;
          ESP_LOGI(TAG, "  Distance to target: %.2f degrees", distance_to_target_deg);
        }
      }
      
      // Calculate position change since last diagnostic
      static float last_position_deg = 0.0f;
      static uint32_t last_diag_time_pos = 0;
      float position_delta_deg = actual_pos_deg - last_position_deg;
      uint32_t time_delta = current_time - last_diag_time_pos;
      float position_change_rate_deg_per_s = (time_delta > 0) ? (position_delta_deg * 1000.0f / time_delta) : 0.0f;
      last_position_deg = actual_pos_deg;
      last_diag_time_pos = current_time;
      
      // Calculate output shaft movement (assuming gearbox ratio)
      // Position delta is in degrees, so convert to revolutions
      float output_revolutions = position_delta_deg / 360.0f;
      
      // NOTE: Adjust this ratio based on your actual gearbox!
      // Example: If gearbox is 5.18:1, then motor_revolutions = output_revolutions * 5.18
      // For the 17HS4401S-PG518 motor, the gearbox ratio is 5.18:1
      // Set to 1.0 for direct drive (no gearbox) or to match your actual gearbox ratio
      float motor_revolutions = output_revolutions * gear_ratio;
      
      ESP_LOGI(TAG, "Diagnostics: VACTUAL=%.1f RPM, XACTUAL=%.2f degrees", actual_velocity_rpm, actual_pos_deg);
      ESP_LOGI(TAG, "  Position change: %.2f degrees in %d ms = %.2f deg/s (calculated)", 
               position_delta_deg, time_delta, position_change_rate_deg_per_s);
      ESP_LOGI(TAG, "  Output: %.3f rev (%.1f deg) in %.1f seconds [gearbox=%.2f:1]", 
               output_revolutions, position_delta_deg, time_delta / 1000.0f, gear_ratio);
      ESP_LOGI(TAG, "  Motor: %.3f rev (%.1f deg) [estimated from gearbox ratio]", 
               motor_revolutions, motor_revolutions * 360.0f);
      
      // Calculate output speed
      float output_rpm = (output_revolutions * 60.0f) / (time_delta / 1000.0f);
      float output_deg_per_sec = position_delta_deg / (time_delta / 1000.0f);
      
      // Check if motion is visible
      if (std::abs(output_revolutions) < 0.01f && time_delta > 500 && std::abs(position_delta_deg) > 1.0f) {
        ESP_LOGW(TAG, "  ⚠️ Output shaft movement is very small (%.4f rev = %.2f deg) despite large motor movement", 
                 output_revolutions, position_delta_deg);
        ESP_LOGW(TAG, "  Output speed: %.2f RPM, %.2f deg/s", output_rpm, output_deg_per_sec);
        ESP_LOGW(TAG, "  This suggests a HIGH gearbox ratio - motor moves many steps but output moves little");
        ESP_LOGW(TAG, "  Update gear_ratio in code to match your actual gearbox");
        ESP_LOGW(TAG, "  Tip: Mark the output shaft and observe movement, then adjust ratio accordingly");
      } else if (std::abs(output_revolutions) > 0.001f) {
        ESP_LOGI(TAG, "  ✓ Output shaft IS moving: %.3f rev (%.1f deg) at %.2f RPM (%.2f deg/s)", 
                 output_revolutions, output_revolutions * 360.0f, output_rpm, output_deg_per_sec);
      } else {
        ESP_LOGW(TAG, "  ⚠️ Output movement too small to measure (%.6f rev) - check gear_ratio setting", output_revolutions);
      }
      
      // Check if motor is actually moving
      if (std::abs(actual_velocity_rpm) > 0.1f && std::abs(position_delta_deg) < 1.0f && time_delta > 500) {
        ESP_LOGW(TAG, "  ⚠️ WARNING: VACTUAL shows motion (%.1f RPM) but position barely changed (%.2f degrees)",
                 actual_velocity_rpm, position_delta_deg);
        ESP_LOGW(TAG, "  This suggests motor is slipping or gearbox ratio is very high");
      } else if (std::abs(position_delta_deg) > 1.0f) {
        ESP_LOGI(TAG, "  ✓ Motor IS moving: position changed by %.2f degrees in %d ms",
                 position_delta_deg, time_delta);
      }
      
      if (has_ramp_stat) {
        tmc51x0::RAMP_STAT_Register status{};
        status.value = ramp_stat;
        ESP_LOGI(TAG, "  Ramp status: vzero=%d, velocity_reached=%d, position_reached=%d, stop_l=%d, stop_r=%d, status_sg=%d",
                 status.bits.vzero ? 1 : 0,
                 status.bits.velocity_reached ? 1 : 0,
                 status.bits.position_reached ? 1 : 0,
                 status.bits.status_stop_l ? 1 : 0,
                 status.bits.status_stop_r ? 1 : 0,
                 status.bits.status_sg ? 1 : 0);
        
        // Check for stall condition (informational only - won't stop motor)
        if (status.bits.status_sg) {
          ESP_LOGW(TAG, "  ⚠️ StallGuard2 active (status_sg=1) - motor may be stalling");
          ESP_LOGI(TAG, "  Note: StallGuard2 stop is DISABLED - motor will continue running");
          ESP_LOGI(TAG, "  If motor is actually stalling, consider: increase current (irun), reduce speed");
        }
        
        if (status.bits.vzero && std::abs(actual_velocity_rpm) < 0.1f) {
          ESP_LOGW(TAG, "  WARNING: VACTUAL is zero - motor may not be starting");
          ESP_LOGW(TAG, "  Check: VSTART may be too low, or motor current too low");
        }
      }
      
      if (has_drv_status) {
        ESP_LOGI(TAG, "  DRV_STATUS: stallguard=%d (SG result, 0=highest load), stallguard_flag=%d",
                 drv_status.bits.sg_result,
                 drv_status.bits.stallguard ? 1 : 0);
        
        // StallGuard2 interpretation: Lower value = more load
        // SG result of 0 with no mechanical load suggests:
        // 1. Wrong motor wiring (phases swapped or incorrect)
        // 2. Motor current too high
        // 3. StallGuard2 threshold needs adjustment
        // 4. StealthChop calibration issue
        if (drv_status.bits.sg_result == 0) {
          if (has_gconf && !gconf.bits.en_pwm_mode) {
            // In SpreadCycle mode, SG=0 indicates a real problem
          ESP_LOGE(TAG, "  ⚠️ CRITICAL: SG result = 0 (highest load) but motor has NO mechanical load!");
          ESP_LOGE(TAG, "  This indicates a PROBLEM:");
          ESP_LOGE(TAG, "    1. Check motor wiring (phases may be swapped or wrong)");
          ESP_LOGE(TAG, "    2. Motor current may be too high (try reducing irun)");
            ESP_LOGE(TAG, "    3. StallGuard2 threshold needs adjustment");
          ESP_LOGE(TAG, "    4. Motor may be electrically stalling");
          } else {
            // In StealthChop mode, SG_RESULT=0 is expected and normal (measurement invalid)
            ESP_LOGI(TAG, "  StallGuard2 result: 0 (Expected in StealthChop mode - measurement invalid)");
          }
        } else if (drv_status.bits.sg_result < 100) {
          ESP_LOGW(TAG, "  ⚠️ High mechanical load detected (SG=%d < 100)", drv_status.bits.sg_result);
          ESP_LOGW(TAG, "  This is normal for geared motors, but may indicate stall if too low");
        } else {
          ESP_LOGI(TAG, "  ✓ StallGuard2 reading normal (SG=%d, higher=less load)", drv_status.bits.sg_result);
        }
        
        if (drv_status.bits.stallguard) {
          ESP_LOGW(TAG, "  ⚠️ StallGuard2 flag is set (stall detected), but motor will NOT stop");
          ESP_LOGI(TAG, "  StallGuard2 stop is disabled - this is informational only");
        }
      }
      
      if (has_gstat && gstat.bits.uv_cp) {
        ESP_LOGE(TAG, "  ⚠️ CHARGE PUMP UNDERVOLTAGE (uv_cp=1) - Motor will stop!");
        ESP_LOGE(TAG, "  This is why the motor stops - fix power supply/capacitor");
      }
      
      // Check if motor stopped unexpectedly
      if (std::abs(actual_velocity_rpm) < 0.1f && std::abs(position_delta_deg) < 1.0f) {
        ESP_LOGW(TAG, "  ⚠️ Motor appears stopped (VACTUAL=%.1f RPM, position_delta=%.2f degrees)", actual_velocity_rpm, position_delta_deg);
        if (has_gstat && gstat.bits.uv_cp) {
          ESP_LOGE(TAG, "  Root cause: Charge pump undervoltage!");
        } else if (has_ramp_stat) {
          tmc51x0::RAMP_STAT_Register status{};
          status.value = ramp_stat;
          if (status.bits.status_sg) {
            ESP_LOGW(TAG, "  StallGuard2 active (status_sg=1), but motor will NOT stop (SG2 stop disabled)");
          }
        }
        ESP_LOGW(TAG, "  Try: Increase VSTART (should be 100), increase motor current, or check wiring");
      }
      
      // Check if motor is still enabled
      auto chopper_result = driver.motorControl.GetChopperConfig();
      if (chopper_result.IsOk()) {
        tmc51x0::ChopperConfig chopconf_check = chopper_result.Value();
        if (chopconf_check.toff == 0) {
          ESP_LOGW(TAG, "WARNING: Motor driver disabled! Re-enabling...");
          driver.motorControl.Enable();
        }
      }
    }
    
    // Delay for update period (check position every 50ms)
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
