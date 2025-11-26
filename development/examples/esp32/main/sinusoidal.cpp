/**
 * @file sinusoidal.cpp
 * @brief Sinusoidal motion pattern example for TMC5160 stepper motor driver
 *
 * This example demonstrates sinusoidal motion control using the TMC5160's
 * internal ramp generator. The motor velocity varies in a sinusoidal pattern
 * using velocity mode control.
 *
 * MOTOR SELECTION:
 * Motor selection is done via a static constexpr variable at the top of this file.
 * See esp32_tmc5160_bus_config.hpp for detailed motor specifications and selection guide.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160 (see motor selection above)
 * - SPI connection between ESP32 and TMC5160
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

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp32_tmc5160_test_config.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// See esp32_tmc5160_bus_config.hpp for detailed motor, board, and platform specifications.
// Change the values below to select different configurations:

// Motor selection (compile-time constant)
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;

// Board selection (compile-time constant)
static constexpr tmc5160_test_config::BoardType SELECTED_BOARD = 
    tmc5160_test_config::BoardType::BOARD_TMC5160_EVAL;

// Platform selection (compile-time constant)
static constexpr tmc5160_test_config::PlatformType SELECTED_PLATFORM = 
    tmc5160_test_config::PlatformType::PLATFORM_TEST_RIG;

static const char* TAG = "Sinusoidal";

/**
 * @brief Back-and-forth motion controller using positioning mode
 *
 * Simple back-and-forth motion using TMC5160's positioning mode.
 * Sets target position to one end, waits until reached, then sets target to other end.
 * Repeats continuously.
 */
class BackAndForthMotion {
private:
  tmc5160::TMC5160<Esp32SPI>* driver_;
  float max_velocity_;        // Maximum velocity in steps/s
  float acceleration_;        // Acceleration in steps/s²
  int32_t travel_distance_;  // Distance to travel in each direction (in microsteps)
  int32_t center_position_;   // Center position (starting point)
  int32_t target_position_;   // Current target position
  bool moving_forward_;        // Direction flag (true = forward, false = backward)
  bool initialized_;
  uint32_t cycles_completed_; // Number of complete back-and-forth cycles
  int max_cycles_;            // Maximum cycles (-1 for infinite)

public:
  BackAndForthMotion(tmc5160::TMC5160<Esp32SPI>* driver)
      : driver_(driver), max_velocity_(10000.0f), acceleration_(50000.0f),
        travel_distance_(100000), center_position_(0), target_position_(0),
        moving_forward_(true), initialized_(false), cycles_completed_(0), max_cycles_(-1) {}

  /**
   * @brief Configure back-and-forth motion parameters
   * @param max_vel Maximum velocity in steps/s
   * @param accel Acceleration in steps/s²
   * @param travel_dist Distance to travel in each direction (in microsteps)
   * @param max_cycles Maximum number of back-and-forth cycles (-1 for infinite)
   */
  void Config(float max_vel, float accel, int32_t travel_dist, int max_cycles = -1) {
    max_velocity_ = max_vel;
    acceleration_ = accel;
    travel_distance_ = travel_dist;
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
    driver_->rampControl.SetAcceleration(acceleration_);
    driver_->rampControl.SetDeceleration(acceleration_);
    
    // Set start/stop velocities
    driver_->rampControl.SetRampSpeeds(1000.0f, 100.0f, 0.0f);
    
    // Set maximum velocity
    driver_->rampControl.SetMaxSpeed(max_velocity_);
    
    // Set to positioning mode
    driver_->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    
    // Get current position as center
    center_position_ = driver_->rampControl.GetCurrentPosition();
    
    // Start by moving forward (positive direction)
    moving_forward_ = true;
    target_position_ = center_position_ + travel_distance_;
    driver_->rampControl.SetTargetPosition(static_cast<float>(target_position_), tmc5160::Unit::Steps);
    
    initialized_ = true;
    cycles_completed_ = 0;
    
    ESP_LOGI(TAG, "Back-and-forth motion started:");
    ESP_LOGI(TAG, "  Max velocity: %.1f steps/s", max_velocity_);
    ESP_LOGI(TAG, "  Acceleration: %.1f steps/s²", acceleration_);
    ESP_LOGI(TAG, "  Travel distance: %ld microsteps (%.2f mm per direction)", 
             travel_distance_, travel_distance_ / 51200.0f); // Assuming 200 steps/rev, 256 microsteps
    ESP_LOGI(TAG, "  Center position: %ld", center_position_);
    ESP_LOGI(TAG, "  Target position: %ld", target_position_);
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
        target_position_ = center_position_ - travel_distance_;
        driver_->rampControl.SetTargetPosition(static_cast<float>(target_position_), tmc5160::Unit::Steps);
        ESP_LOGI(TAG, "Reached forward end, reversing to position %ld", target_position_);
      } else {
        // Just finished moving backward, now move forward
        moving_forward_ = true;
        target_position_ = center_position_ + travel_distance_;
        driver_->rampControl.SetTargetPosition(static_cast<float>(target_position_), tmc5160::Unit::Steps);
        cycles_completed_++;
        ESP_LOGI(TAG, "Reached backward end, reversing to position %ld (cycle %lu complete)", 
                 target_position_, cycles_completed_);
        
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
    driver_->rampControl.SetRampMode(tmc5160::RampMode::HOLD);
    driver_->rampControl.SetMaxSpeed(0.0);
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
  ESP_LOGI(TAG, "TMC5160 Back-and-Forth Motion Example for NEMA 44mm Motors");
  ESP_LOGI(TAG, "Using internal ramp generator with positioning control");

  // Get standard pin configuration
  auto pin_config = tmc5160_test_config::GetDefaultPinConfig();

  // Create SPI communication interface with pin configuration
  // Check if EN pin needs to be inverted (some boards have inverters on EN pin)
  // Default: EN is active LOW (LOW = enable, HIGH = disable) per TMC5160 datasheet
  // If your board has an inverter, set en = true in PinActiveLevels
  tmc5160::PinActiveLevels active_levels; // Uses defaults: en=false (LOW=enable)
  
  // Uncomment the line below if your board has an inverter on the EN pin:
  // active_levels.en = true; // EN pin has inverter, so ACTIVE = HIGH to enable
  
  Esp32SPI spi(tmc5160_test_config::SPI_HOST, pin_config, 1000000, active_levels); // 1 MHz SPI clock (reduced for stability)

  // Initialize SPI interface
  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC5160 driver instance
  tmc5160::TMC5160<Esp32SPI> driver(spi);

  // Select motor configuration based on compile-time selection
  // Note: Namespace aliases must be declared at namespace scope, so we use conditional compilation
  if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    ESP_LOGI(TAG, "Selected Motor: 17HS4401S with 5.18:1 gearbox");
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    ESP_LOGI(TAG, "Selected Motor: 17HS4401S direct drive (no gearbox)");
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    ESP_LOGI(TAG, "Selected Motor: Applied Motion 5034-369 NEMA 34 (high torque, 4.17A)");
  }
  
  // Configure driver using helper functions
  tmc5160::DriverConfig cfg{};
  
  // Motor configuration constants (extracted for use later in code)
  uint16_t output_full_steps = 0;
  float gear_ratio = 1.0f;
  
  // Configure motor
  if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
    output_full_steps = Motor::OUTPUT_FULL_STEPS;
    gear_ratio = Motor::GEAR_RATIO;
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S_Direct;
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Direct(cfg);
    output_full_steps = Motor::OUTPUT_FULL_STEPS;
    gear_ratio = Motor::GEAR_RATIO;
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    namespace Motor = tmc5160_test_config::MotorConfig_AppliedMotion_5034_369;
    tmc5160_test_config::ConfigureDriverFromMotor_AppliedMotion_5034(cfg);
    output_full_steps = Motor::OUTPUT_FULL_STEPS;
    gear_ratio = Motor::GEAR_RATIO;
  }
  
  // Apply board configuration
  tmc5160_test_config::ApplyBoardConfig<SELECTED_BOARD>(cfg);
  
  // Apply platform configuration
  tmc5160_test_config::ApplyPlatformConfig<SELECTED_PLATFORM>(cfg);
  
  // Enable StealthChop
  cfg.global_config.en_pwm_mode = true; // Enable StealthChop

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");
  
  // Run verification immediately after initialization
  ESP_LOGI(TAG, "Running startup verification...");
  if (!driver.diagnostics.VerifySetup()) {
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
  
  // 1. Disable StallGuard2 stop in SW_MODE (this is the main stop mechanism)
  uint32_t sw_mode_value = 0;
  if (driver.GetComm().ReadRegister(tmc5160::Registers::SW_MODE, sw_mode_value)) {
    tmc5160::SW_MODE_Register sw_mode{};
    sw_mode.value = sw_mode_value;
    bool changed = false;
    if (sw_mode.bits.sg_stop) {
      ESP_LOGW(TAG, "StallGuard2 stop is ENABLED - disabling for sinusoidal motion");
      sw_mode.bits.sg_stop = 0;
      changed = true;
    }
    // Also ensure soft stop is disabled (not needed for continuous motion)
    if (sw_mode.bits.en_softstop) {
      ESP_LOGW(TAG, "Soft stop is ENABLED - disabling for continuous sinusoidal motion");
      sw_mode.bits.en_softstop = 0;
      changed = true;
    }
    if (changed) {
      driver.GetComm().WriteRegister(tmc5160::Registers::SW_MODE, sw_mode.value);
      // Verify it was written
      uint32_t verify_value = 0;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::SW_MODE, verify_value)) {
        tmc5160::SW_MODE_Register verify{};
        verify.value = verify_value;
        if (verify.bits.sg_stop == 0 && verify.bits.en_softstop == 0) {
          ESP_LOGI(TAG, "✓ StallGuard2 stop and soft stop confirmed DISABLED in SW_MODE");
        } else {
          ESP_LOGE(TAG, "✗ Failed to disable StallGuard2 stop! sg_stop=%d, en_softstop=%d",
                   verify.bits.sg_stop ? 1 : 0, verify.bits.en_softstop ? 1 : 0);
        }
      }
    } else {
      ESP_LOGI(TAG, "✓ StallGuard2 stop already disabled (correct for sinusoidal motion)");
    }
  }
  
  // 2. Set TCOOLTHRS to 0 to disable StallGuard2 at all speeds
  // TCOOLTHRS = velocity threshold below which StallGuard2 is disabled
  // Enabled if TSTEP < TCOOLTHRS (Velocity > Threshold)
  // To disable, we want Threshold to be infinite (TCOOLTHRS = 0)
  uint32_t tcoolthrs = 0; // 0 = infinite velocity threshold = disabled everywhere
  if (driver.GetComm().WriteRegister(tmc5160::Registers::TCOOLTHRS, tcoolthrs)) {
    ESP_LOGI(TAG, "✓ TCOOLTHRS set to 0 - StallGuard2 disabled at all speeds");
  } else {
    ESP_LOGE(TAG, "✗ Failed to set TCOOLTHRS");
  }
  
  // 3. Set THIGH to maximum to ensure StallGuard2 doesn't interfere
  // THIGH = velocity threshold for chopper mode switching
  // Setting high ensures StallGuard2 doesn't affect operation
  uint32_t thigh = 0xFFFFF; // Maximum value
  if (driver.GetComm().WriteRegister(tmc5160::Registers::THIGH, thigh)) {
    ESP_LOGI(TAG, "✓ THIGH set to maximum (0x%05X) - ensures StallGuard2 doesn't interfere", thigh);
  } else {
    ESP_LOGW(TAG, "Failed to set THIGH (may not be critical)");
  }
  
  // 4. Configure StallGuard2 threshold to be least sensitive (even though it's disabled)
  // This is just for diagnostics - it won't stop the motor
  // NOTE: StallGuard2 ONLY works in SpreadCycle mode! In StealthChop, SG_RESULT is invalid/zero.
  tmc5160::StallGuardConfig sg_cfg{};
  sg_cfg.threshold = 63;        // Maximum threshold (least sensitive) - won't trigger
  sg_cfg.enable_filter = false; // No filter (faster response)
  // Note: semin/semax are CoolStep parameters, configure separately if needed
  if (driver.diagnostics.ConfigureStallGuard(sg_cfg)) {
    ESP_LOGI(TAG, "✓ StallGuard2 configured for diagnostics only (sgt=63, least sensitive)");
    ESP_LOGI(TAG, "  Note: StallGuard2 is DISABLED and will NOT stop the motor");
    if (cfg.global_config.en_pwm_mode) {
      ESP_LOGW(TAG, "  Note: StealthChop is enabled, so StallGuard2 values will be invalid (usually 0)");
    }
  } else {
    ESP_LOGW(TAG, "Failed to configure StallGuard2 (may not be critical)");
  }
  
  // Disable reference switches if not using them (prevents motion blocking)
  // If you have reference switches connected, configure them instead
  tmc5160::ReferenceSwitchConfig ref_cfg{};
  // Configure switches but disable motor stop (allows reading switch state without stopping)
  ref_cfg.left_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;
  ref_cfg.right_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;
  ref_cfg.left_switch_stop_enable = false;   // Don't stop motor
  ref_cfg.right_switch_stop_enable = false;  // Don't stop motor
  ref_cfg.latch_left = tmc5160::ReferenceLatchMode::DISABLED;   // No latching
  ref_cfg.latch_right = tmc5160::ReferenceLatchMode::DISABLED;  // No latching
  if (!driver.rampControl.ConfigureReferenceSwitch(ref_cfg)) {
    ESP_LOGW(TAG, "Failed to configure reference switches (may not be critical)");
  } else {
    ESP_LOGI(TAG, "Reference switches disabled (not using endstops)");
    
    // Verify SW_MODE register was written correctly
    uint32_t sw_mode_value = 0;
    if (driver.GetComm().ReadRegister(tmc5160::Registers::SW_MODE, sw_mode_value)) {
      tmc5160::SW_MODE_Register sw_mode{};
      sw_mode.value = sw_mode_value;
      ESP_LOGI(TAG, "SW_MODE verification: stop_l_enable=%d, stop_r_enable=%d, en_softstop=%d",
               sw_mode.bits.stop_l_enable ? 1 : 0,
               sw_mode.bits.stop_r_enable ? 1 : 0,
               sw_mode.bits.en_softstop ? 1 : 0);
      
      if (sw_mode.bits.stop_l_enable || sw_mode.bits.stop_r_enable) {
        ESP_LOGE(TAG, "ERROR: Reference switches still enabled in SW_MODE!");
        ESP_LOGE(TAG, "Motion will be blocked. Re-configuring...");
        // Try again
        driver.rampControl.ConfigureReferenceSwitch(ref_cfg);
      } else {
        ESP_LOGI(TAG, "✓ Reference switches confirmed disabled in SW_MODE");
      }
    }
  }
  
  // Check physical pin states (if pins are mapped)
  tmc5160::GpioSignal ref_left_signal, ref_right_signal;
  bool ref_left_read = driver.GetComm().GpioRead(tmc5160::TMC5160CtrlPin::REFL_STEP, ref_left_signal);
  bool ref_right_read = driver.GetComm().GpioRead(tmc5160::TMC5160CtrlPin::REFR_DIR, ref_right_signal);
  
  if (ref_left_read) {
    ESP_LOGI(TAG, "REFL_STEP (left ref) pin state: %s",
             ref_left_signal == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  if (ref_right_read) {
    ESP_LOGI(TAG, "REFR_DIR (right ref) pin state: %s",
             ref_right_signal == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  
  if (ref_left_read && ref_left_signal == tmc5160::GpioSignal::ACTIVE) {
    ESP_LOGW(TAG, "WARNING: REFL_STEP pin is HIGH - if this is a switch, it may be active");
    ESP_LOGW(TAG, "  If not using switches, ensure pin is pulled LOW or left floating");
  }
  if (ref_right_read && ref_right_signal == tmc5160::GpioSignal::ACTIVE) {
    ESP_LOGW(TAG, "WARNING: REFR_DIR pin is HIGH - if this is a switch, it may be active");
    ESP_LOGW(TAG, "  If not using switches, ensure pin is pulled LOW or left floating");
  }

  // Verify chip is in internal ramp mode (SPI_MODE=HIGH, SD_MODE=LOW)
  // If mode pins are configured, verify they're set correctly
  if (pin_config.tmc5160_pins.spi_mode_pin != -1 && pin_config.tmc5160_pins.sd_mode_pin != -1) {
    tmc5160::ChipCommMode current_mode;
    if (driver.GetChipCommMode(current_mode)) {
      if (current_mode != tmc5160::ChipCommMode::SPI_INTERNAL_RAMP) {
        ESP_LOGW(TAG, "Chip is not in SPI_INTERNAL_RAMP mode (current: %d)", static_cast<int>(current_mode));
        ESP_LOGW(TAG, "Setting to SPI_INTERNAL_RAMP mode...");
        if (driver.SetChipCommMode(tmc5160::ChipCommMode::SPI_INTERNAL_RAMP)) {
          ESP_LOGW(TAG, "Mode changed - chip reset required! Power cycle the TMC5160 now.");
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
  tmc5160::GpioSignal en_signal;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::EN, en_signal)) {
    ESP_LOGI(TAG, "EN pin state after Enable(): %s", 
             en_signal == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
    ESP_LOGI(TAG, "  Note: TMC5160 DRV_ENN is active LOW (LOW=enable, HIGH=disable)");
    ESP_LOGI(TAG, "  If motor doesn't move, check if your board has an inverter on EN pin");
    ESP_LOGI(TAG, "  If so, configure: active_levels.en = true in PinActiveLevels");
  }
  
  // Check for Charge Pump Undervoltage immediately after enabling
  uint32_t gstat_val = 0;
  if (driver.GetComm().ReadRegister(tmc5160::Registers::GSTAT, gstat_val)) {
    tmc5160::GSTAT_Register gstat{};
    gstat.value = gstat_val;
    if (gstat.bits.uv_cp) {
      ESP_LOGE(TAG, "CRITICAL HARDWARE ERROR: Charge Pump Undervoltage (uv_cp=1) detected immediately!");
      ESP_LOGE(TAG, "  This usually means VSA/VS voltage is too low or the charge pump capacitor is missing/bad.");
      ESP_LOGE(TAG, "  The motor DRIVER STAGE IS DISABLED by the chip protection.");
      ESP_LOGE(TAG, "  Please check your power supply (12-36V) and wiring.");
    }
  }
  
  // Verify motor is enabled by checking CHOPCONF register
  uint32_t chopconf_value = 0;
  if (driver.GetComm().ReadRegister(tmc5160::Registers::CHOPCONF, chopconf_value)) {
    tmc5160::CHOPCONF_Register chopconf{};
    chopconf.value = chopconf_value;
    if (chopconf.bits.toff == 0) {
      ESP_LOGE(TAG, "Motor driver not enabled! CHOPCONF.toff=0 (driver disabled)");
      ESP_LOGE(TAG, "Check EN pin connection and Enable() call");
      return;
    } else {
      ESP_LOGI(TAG, "Motor driver verified enabled (CHOPCONF.toff=%u)", chopconf.bits.toff);
    }
  }
  
  // CRITICAL: Check StealthChop status - if enabled but not calibrated, motor won't move!
  uint32_t gconf_value = 0;
  if (driver.GetComm().ReadRegister(tmc5160::Registers::GCONF, gconf_value)) {
    tmc5160::GCONF_Register gconf{};
    gconf.value = gconf_value;
    
    ESP_LOGI(TAG, "=== StealthChop Diagnostic ===");
    ESP_LOGI(TAG, "GCONF.en_pwm_mode = %d (1=enabled, 0=disabled/SpreadCycle)", gconf.bits.en_pwm_mode ? 1 : 0);
    
    if (gconf.bits.en_pwm_mode) {
      ESP_LOGW(TAG, "⚠️ StealthChop is ENABLED - checking calibration...");
      
      // Read PWM_SCALE to check if StealthChop is actually working
      uint32_t pwm_scale_value = 0;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::PWM_SCALE, pwm_scale_value)) {
        tmc5160::PWM_SCALE_Register pwm_scale{};
        pwm_scale.value = pwm_scale_value;
        
        ESP_LOGI(TAG, "PWM_SCALE: pwm_scale_sum=%d, pwm_scale_auto=%d", 
                 pwm_scale.bits.pwm_scale_sum, pwm_scale.bits.pwm_scale_auto);
        
        // Also check PWM_AUTO for calibration status
        uint32_t pwm_auto_value = 0;
        bool has_pwm_auto = driver.GetComm().ReadRegister(tmc5160::Registers::PWM_AUTO, pwm_auto_value);
        if (has_pwm_auto) {
          tmc5160::PWM_AUTO_Register pwm_auto{};
          pwm_auto.value = pwm_auto_value;
          ESP_LOGI(TAG, "PWM_AUTO: pwm_ofs_auto=%d, pwm_grad_auto=%d", 
                   pwm_auto.bits.pwm_ofs_auto, pwm_auto.bits.pwm_grad_auto);
        }
        
        // If pwm_scale_auto is 0, StealthChop is not calibrated!
        // pwm_scale_auto is a 9-bit signed value, so 0 or very small values indicate no calibration
        int16_t pwm_scale_auto_signed = static_cast<int16_t>(pwm_scale.bits.pwm_scale_auto);
        if (pwm_scale_auto_signed & 0x100) { // Sign extend 9-bit to 16-bit
          pwm_scale_auto_signed |= 0xFE00;
        }
        
        if (pwm_scale_auto_signed == 0 || (pwm_scale_auto_signed > -10 && pwm_scale_auto_signed < 10)) {
          ESP_LOGW(TAG, "⚠️ StealthChop is enabled but NOT YET CALIBRATED (pwm_scale_auto=%d)", pwm_scale_auto_signed);
          ESP_LOGI(TAG, "   Calibration will occur automatically:");
          ESP_LOGI(TAG, "   AT#1: Wait 130ms+ at standstill with CS=IRUN for PWM_OFS_AUTO");
          ESP_LOGI(TAG, "         (Motor rated current: %u mA - IRUN/IHOLD calculated automatically)", 
                   cfg.motor_spec.rated_current_ma);
          ESP_LOGI(TAG, "   AT#2: Move motor at 60-300 RPM for PWM_GRAD_AUTO (~400 fullsteps)");
          ESP_LOGI(TAG, "   Motor may not move until calibration completes - this is normal");
          ESP_LOGI(TAG, "   Keeping StealthChop enabled - calibration will happen during operation");
        } else {
          ESP_LOGI(TAG, "✓ StealthChop appears calibrated (pwm_scale_auto=%d)", pwm_scale_auto_signed);
        }
      }
    } else {
      ESP_LOGI(TAG, "✓ StealthChop is DISABLED - using SpreadCycle mode");
    }
  }
  
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
    tmc5160::GpioSignal diag0_signal, diag1_signal;
    bool diag0_read = driver.GetComm().GpioRead(tmc5160::TMC5160CtrlPin::DIAG0, diag0_signal);
    bool diag1_read = driver.GetComm().GpioRead(tmc5160::TMC5160CtrlPin::DIAG1, diag1_signal);
    
    if (diag0_read) {
      ESP_LOGI(TAG, "DIAG0 pin state: %s", 
               diag0_signal == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
    } else {
      ESP_LOGW(TAG, "DIAG0 pin not configured or read failed");
    }
    
    if (diag1_read) {
      ESP_LOGI(TAG, "DIAG1 pin state: %s", 
               diag1_signal == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
    } else {
      ESP_LOGW(TAG, "DIAG1 pin not configured or read failed");
    }
    
    // Read GCONF to see which diagnostic features are enabled
    uint32_t gconf_value = 0;
    if (driver.GetComm().ReadRegister(tmc5160::Registers::GCONF, gconf_value)) {
      tmc5160::GCONF_Register gconf{};
      gconf.value = gconf_value;
      
      ESP_LOGI(TAG, "GCONF diagnostic settings:");
      ESP_LOGI(TAG, "  diag0_error=%d, diag0_otpw=%d, diag0_stall_step=%d",
               gconf.bits.diag0_error ? 1 : 0,
               gconf.bits.diag0_otpw ? 1 : 0,
               gconf.bits.diag0_stall_step ? 1 : 0);
      ESP_LOGI(TAG, "  diag1_stall_dir=%d, diag1_index=%d, diag1_onstate=%d, diag1_steps_skipped=%d",
               gconf.bits.diag1_stall_dir ? 1 : 0,
               gconf.bits.diag1_index ? 1 : 0,
               gconf.bits.diag1_onstate ? 1 : 0,
               gconf.bits.diag1_steps_skipped ? 1 : 0);
      ESP_LOGI(TAG, "  diag0_pushpull=%d, diag1_pushpull=%d",
               gconf.bits.diag0_int_pushpull ? 1 : 0,
               gconf.bits.diag1_poscomp_pushpull ? 1 : 0);
      
      // Read GSTAT for reset and driver errors
      tmc5160::GSTAT_Register gstat{};
      gstat.value = 0;
      bool gstat_read = false;
      uint32_t gstat_value = 0;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::GSTAT, gstat_value)) {
        gstat.value = gstat_value;
        gstat_read = true;
        
        ESP_LOGI(TAG, "GSTAT: reset=%d, drv_err=%d, uv_cp=%d",
                 gstat.bits.reset ? 1 : 0,
                 gstat.bits.drv_err ? 1 : 0,
                 gstat.bits.uv_cp ? 1 : 0);
        
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
      tmc5160::DRV_STATUS_Register drv_status{};
      drv_status.value = 0;
      bool drv_status_read = false;
      uint32_t drv_status_value = 0;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::DRV_STATUS, drv_status_value)) {
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
        // Note: SG_RESULT is generally invalid in StealthChop mode on TMC5160!
        // Only report wiring issues if in SpreadCycle (en_pwm_mode=0)
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
      tmc5160::RAMP_STAT_Register ramp_stat{};
      ramp_stat.value = 0;
      uint32_t ramp_stat_value = 0;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::RAMP_STAT, ramp_stat_value)) {
        ramp_stat.value = ramp_stat_value;
        
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
        bool diag0_active = (diag0_signal == tmc5160::GpioSignal::ACTIVE);
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
        bool diag1_active = (diag1_signal == tmc5160::GpioSignal::ACTIVE);
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
  if (driver.GetComm().ReadRegister(tmc5160::Registers::RAMP_STAT, ramp_stat)) {
    tmc5160::RAMP_STAT_Register status{};
    status.value = ramp_stat;
    ESP_LOGI(TAG, "RAMP_STAT: vzero=%d, velocity_reached=%d, position_reached=%d, stop_l=%d, stop_r=%d",
             status.bits.vzero ? 1 : 0,
             status.bits.velocity_reached ? 1 : 0,
             status.bits.position_reached ? 1 : 0,
             status.bits.status_stop_l ? 1 : 0,
             status.bits.status_stop_r ? 1 : 0);
    
    // Check SW_MODE to see if stops are actually enabled
    uint32_t sw_mode_check = 0;
    bool stops_enabled = false;
    if (driver.GetComm().ReadRegister(tmc5160::Registers::SW_MODE, sw_mode_check)) {
      tmc5160::SW_MODE_Register sw_mode{};
      sw_mode.value = sw_mode_check;
      stops_enabled = sw_mode.bits.stop_l_enable || sw_mode.bits.stop_r_enable;
    }
    
    if (status.bits.status_stop_l || status.bits.status_stop_r) {
      if (stops_enabled) {
        ESP_LOGE(TAG, "ERROR: Reference switch active AND enabled! stop_l=%d, stop_r=%d", 
                 status.bits.status_stop_l ? 1 : 0, status.bits.status_stop_r ? 1 : 0);
        ESP_LOGE(TAG, "This WILL prevent motion in internal ramp mode!");
        ESP_LOGE(TAG, "Solution: Disable reference switches via ConfigureReferenceSwitch()");
      } else {
        ESP_LOGW(TAG, "Reference switch pins are active (stop_l=%d, stop_r=%d) but stops are DISABLED",
                 status.bits.status_stop_l ? 1 : 0, status.bits.status_stop_r ? 1 : 0);
        ESP_LOGI(TAG, "Motion should still work since stop_l_enable=0 and stop_r_enable=0 in SW_MODE");
        ESP_LOGI(TAG, "The status bits just reflect pin state - they don't block motion when disabled");
      }
    }
  }
  
  // Read VACTUAL to see if motor is trying to move
  // Use GetCurrentSpeed() which properly converts from internal units to steps/s
  float actual_velocity = driver.rampControl.GetCurrentSpeed();
  ESP_LOGI(TAG, "VACTUAL (actual velocity): %.1f steps/s", actual_velocity);
  if (actual_velocity != 0.0f) {
    ESP_LOGI(TAG, "  ✓ Motor IS moving at %.1f steps/s", actual_velocity);
  } else {
    ESP_LOGW(TAG, "  ✗ Motor is NOT moving (VACTUAL=0)");
  }

  // Create back-and-forth motion controller
  BackAndForthMotion motion(&driver);

  // Configure back-and-forth motion for NEMA 44mm motor with gearbox
  // For 200 steps/rev with 256 microsteps = 51,200 microsteps per motor revolution
  // With 5.18 gearbox = ~265,216 microsteps per output revolution
  
  // Calculate motion parameters
  // Travel distance: ~2 full output revolutions = 2 * 265,216 = ~530,432 microsteps
  // Or use a more reasonable distance like 1 output revolution = ~265,216 microsteps
  float output_steps_per_rev = static_cast<float>(output_full_steps) * 256.0f;
  int32_t travel_distance = static_cast<int32_t>(output_steps_per_rev * 1.0f); // 1 full output revolution each direction
  
  // Max velocity: ~0.5 RPS output = ~132,608 steps/s
  float max_velocity = output_steps_per_rev * 0.5f;
  
  // Acceleration: reach max velocity in ~0.2 seconds
  float acceleration = max_velocity * 5.0f; // 5x max_velocity for 0.2s ramp time
  if (acceleration < 50000.0f) acceleration = 50000.0f; // Minimum acceleration
  
  int max_cycles = -1; // Infinite cycles
  
  motion.Config(max_velocity, acceleration, travel_distance, max_cycles);

  ESP_LOGI(TAG, "Starting back-and-forth motion for NEMA 44mm motor:");
  ESP_LOGI(TAG, "  Max velocity: %.1f steps/s (%.2f RPS output)", max_velocity, 0.5f);
  ESP_LOGI(TAG, "  Acceleration: %.1f steps/s²", acceleration);
  ESP_LOGI(TAG, "  Travel distance: %ld microsteps (%.2f output revolutions per direction)", 
           travel_distance, travel_distance / output_steps_per_rev);
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
      float actual_velocity = driver.rampControl.GetCurrentSpeed();
      int32_t actual_position = driver.rampControl.GetCurrentPosition();
      
      // Read ramp status
      uint32_t ramp_stat = 0;
      bool has_ramp_stat = driver.GetComm().ReadRegister(tmc5160::Registers::RAMP_STAT, ramp_stat);
      
      // Read DRV_STATUS for stall detection
      uint32_t drv_status_value = 0;
      bool has_drv_status = driver.GetComm().ReadRegister(tmc5160::Registers::DRV_STATUS, drv_status_value);
      tmc5160::DRV_STATUS_Register drv_status{};
      if (has_drv_status) {
        drv_status.value = drv_status_value;
      }
      
      // Read GSTAT for charge pump status
      uint32_t gstat_value = 0;
      bool has_gstat = driver.GetComm().ReadRegister(tmc5160::Registers::GSTAT, gstat_value);
      tmc5160::GSTAT_Register gstat{};
      if (has_gstat) {
        gstat.value = gstat_value;
      }
      
      // Read GCONF to check StealthChop mode (needed for StallGuard2 interpretation)
      uint32_t gconf_value = 0;
      bool has_gconf = driver.GetComm().ReadRegister(tmc5160::Registers::GCONF, gconf_value);
      tmc5160::GCONF_Register gconf{};
      if (has_gconf) {
        gconf.value = gconf_value;
      }
      
      // Note: VSTART, VSTOP, AMAX are WRITE-ONLY registers - they always read as 0
      // We can't verify them by reading, but we know they were set in BackAndForthMotion::Start()
      // VSTART=1000, VSTOP=100, AMAX=acceleration (from SetRampSpeeds, SetAcceleration, and SetDeceleration)
      
      // Read RAMPMODE to verify we're in positioning mode
      uint32_t rampmode_value = 0;
      bool in_positioning_mode = false;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::RAMPMODE, rampmode_value)) {
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
      uint32_t xtarget_value = 0;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::XTARGET, xtarget_value)) {
        int32_t target_pos = static_cast<int32_t>(xtarget_value);
        ESP_LOGI(TAG, "  XTARGET (target position): %ld", target_pos);
        ESP_LOGI(TAG, "  XACTUAL (current position): %d", actual_position);
        int32_t distance_to_target = target_pos - actual_position;
        ESP_LOGI(TAG, "  Distance to target: %ld microsteps", distance_to_target);
      }
      
      // Calculate position change since last diagnostic
      static int32_t last_position = 0;
      static uint32_t last_diag_time_pos = 0;
      int32_t position_delta = actual_position - last_position;
      uint32_t time_delta = current_time - last_diag_time_pos;
      float position_change_rate = (time_delta > 0) ? (position_delta * 1000.0f / time_delta) : 0.0f;
      last_position = actual_position;
      last_diag_time_pos = current_time;
      
      // Calculate output shaft movement (assuming gearbox ratio)
      // Motor is configured with 256 microsteps (MRES=0) and 200 steps/rev
      // So: 200 steps/rev * 256 microsteps = 51,200 microsteps per motor revolution
      // Position delta is in microsteps, so divide by microsteps per rev to get motor revolutions
      constexpr float MICROSTEPS_PER_MOTOR_REV = 200.0f * 256.0f; // 51,200 microsteps/rev
      float motor_revolutions = position_delta / MICROSTEPS_PER_MOTOR_REV;
      
      // NOTE: Adjust this ratio based on your actual gearbox!
      // Example: If gearbox is 5.18:1, then output_revolutions = motor_revolutions / 5.18
      // For the 17HS4401S-PG518 motor, the gearbox ratio is 5.18:1
      // Set to 1.0 for direct drive (no gearbox) or to match your actual gearbox ratio
      float output_revolutions = motor_revolutions / gear_ratio;
      
      ESP_LOGI(TAG, "Diagnostics: VACTUAL=%.1f steps/s, XACTUAL=%d", actual_velocity, actual_position);
      ESP_LOGI(TAG, "  Position change: %d microsteps in %d ms = %.1f steps/s (calculated)", 
               position_delta, time_delta, position_change_rate);
      ESP_LOGI(TAG, "  Motor: %.3f rev (%.1f deg) in %.1f seconds", 
               motor_revolutions, motor_revolutions * 360.0f, time_delta / 1000.0f);
      ESP_LOGI(TAG, "  Output: %.3f rev (%.1f deg) [gearbox=%.2f:1]", 
               output_revolutions, output_revolutions * 360.0f, gear_ratio);
      
      // Calculate output speed
      float output_rpm = (output_revolutions * 60.0f) / (time_delta / 1000.0f);
      float output_deg_per_sec = output_revolutions * 360.0f / (time_delta / 1000.0f);
      
      // Check if motion is visible
      if (std::abs(output_revolutions) < 0.01f && time_delta > 500 && std::abs(position_delta) > 1000) {
        ESP_LOGW(TAG, "  ⚠️ Output shaft movement is very small (%.4f rev = %.2f deg) despite large motor movement", 
                 output_revolutions, output_revolutions * 360.0f);
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
      if (std::abs(actual_velocity) > 1.0f && std::abs(position_delta) < 10 && time_delta > 500) {
        ESP_LOGW(TAG, "  ⚠️ WARNING: VACTUAL shows motion (%.1f steps/s) but position barely changed (%d steps)",
                 actual_velocity, position_delta);
        ESP_LOGW(TAG, "  This suggests motor is slipping or gearbox ratio is very high");
      } else if (std::abs(position_delta) > 100) {
        ESP_LOGI(TAG, "  ✓ Motor IS moving: position changed by %d steps in %d ms",
                 position_delta, time_delta);
      }
      
      if (has_ramp_stat) {
        tmc5160::RAMP_STAT_Register status{};
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
        
        if (status.bits.vzero && std::abs(actual_velocity) < 1.0f) {
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
      if (std::abs(actual_velocity) < 1.0f && std::abs(position_delta) < 5) {
        ESP_LOGW(TAG, "  ⚠️ Motor appears stopped (VACTUAL=%.1f, position_delta=%d)", actual_velocity, position_delta);
        if (has_gstat && gstat.bits.uv_cp) {
          ESP_LOGE(TAG, "  Root cause: Charge pump undervoltage!");
        } else if (has_ramp_stat) {
          tmc5160::RAMP_STAT_Register status{};
          status.value = ramp_stat;
          if (status.bits.status_sg) {
            ESP_LOGW(TAG, "  StallGuard2 active (status_sg=1), but motor will NOT stop (SG2 stop disabled)");
          }
        }
        ESP_LOGW(TAG, "  Try: Increase VSTART (should be 100), increase motor current, or check wiring");
      }
      
      // Check if motor is still enabled
      uint32_t chopconf_check = 0;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::CHOPCONF, chopconf_check)) {
        tmc5160::CHOPCONF_Register chopconf{};
        chopconf.value = chopconf_check;
        if (chopconf.bits.toff == 0) {
          ESP_LOGW(TAG, "WARNING: Motor driver disabled! Re-enabling...");
          driver.motorControl.Enable();
        }
      }
    }
    
    // Delay for update period (check position every 50ms)
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
