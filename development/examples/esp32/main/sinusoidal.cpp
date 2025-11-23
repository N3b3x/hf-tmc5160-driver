/**
 * @file sinusoidal.cpp
 * @brief Sinusoidal motion pattern example for TMC5160 stepper motor driver
 *
 * This example demonstrates sinusoidal motion control using the TMC5160's
 * internal ramp generator. The motor velocity varies in a sinusoidal pattern
 * using velocity mode control.
 *
 * Configured for NEMA 44mm x 44mm body stepper motors (regular small steppers):
 * - Typical specs: 1.8° step angle (200 steps/rev), 0.5-1.5A per phase, 24V
 * - Uses 16 microsteps for smooth motion
 * - Current set to ~1.0A run, ~0.3A hold (adjust based on your motor)
 * - Optimized for 24V operation (better performance than 12V)
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - NEMA 44mm stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 * - Chip must be in SPI_INTERNAL_RAMP mode (SPI_MODE=HIGH, SD_MODE=LOW)
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
#include "esp32_tmc5160_bus_config.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char* TAG = "Sinusoidal";

/**
 * @brief Sinusoidal motion controller class using internal ramp generator
 *
 * Generates sinusoidal velocity profile using TMC5160's velocity mode.
 * The velocity varies sinusoidally, creating smooth back-and-forth motion.
 */
class SinusoidalMotion {
private:
  tmc5160::TMC5160<Esp32SPI>* driver_;
  double frequency_;           // Frequency in Hz
  float max_velocity_;        // Maximum velocity in steps/s
  float base_velocity_;      // Base velocity offset (for unidirectional motion)
  uint32_t update_period_ms_; // Update period in milliseconds
  uint32_t init_time_;        // Initial time reference
  bool initialized_;
  int rounds_;                // Number of rounds to execute (-1 for infinite)
  int current_round_;

public:
  SinusoidalMotion(tmc5160::TMC5160<Esp32SPI>* driver)
      : driver_(driver), frequency_(1.0), max_velocity_(500.0), base_velocity_(0.0),
        update_period_ms_(50), init_time_(0), initialized_(false), rounds_(-1), current_round_(0) {}

  /**
   * @brief Configure sinusoidal motion parameters
   * @param freq Frequency in Hz (how fast the sinusoidal cycle repeats)
   * @param max_vel Maximum velocity in steps/s (amplitude of velocity variation)
   * @param base_vel Base velocity offset in steps/s (0 for bidirectional, >0 for unidirectional)
   * @param update_period_ms Update period in milliseconds (how often to update velocity)
   * @param rounds Number of rounds (-1 for infinite)
   */
  void Config(double freq, float max_vel, float base_vel, uint32_t update_period_ms, int rounds = -1) {
    frequency_ = freq;
    max_velocity_ = max_vel;
    base_velocity_ = base_vel;
    update_period_ms_ = update_period_ms;
    rounds_ = rounds;
    initialized_ = false;
    current_round_ = 0;
  }

  /**
   * @brief Initialize and start sinusoidal motion
   */
  void Start() {
    if (initialized_) {
      return;
    }

    // Set acceleration for smooth velocity changes
    // For geared motors: Lower acceleration to avoid gearbox backlash and stress
    // But not too low - need enough to overcome static friction
    driver_->rampControl.SetAccelerations(1500.0, 1500.0); // 1500 steps/s² (increased from 1000)
    
    // Set start/stop velocities - VSTART must be > 0 for motion to start
    // For geared motors: Higher VSTART to overcome gearbox friction and static load
    // The "jerk" suggests VSTART might be too low - increase it significantly
    // VSTOP should be low for smooth stopping, VSTART should be much higher for geared motors
    driver_->rampControl.SetRampSpeeds(100.0, 30.0, 0.0); // VSTART=100 (increased from 50), VSTOP=30, V1=0
    
    // Set to velocity mode (will be updated in Update() based on velocity sign)
    driver_->rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
    
    // Set initial velocity to start motion
    driver_->rampControl.SetMaxSpeed(100.0); // Start with small velocity
    
    initialized_ = true;
    init_time_ = esp_timer_get_time() / 1000; // Convert to milliseconds
    current_round_ = 0;
    
    ESP_LOGI(TAG, "Sinusoidal motion started: freq=%.2f Hz, max_vel=%.1f steps/s, base_vel=%.1f steps/s",
             frequency_, max_velocity_, base_velocity_);
  }

  /**
   * @brief Update sinusoidal motion (call periodically)
   * @return true if motion is active, false if completed
   */
  bool Update() {
    if (!initialized_) {
      Start();
    }

    // Calculate elapsed time in seconds
    uint32_t current_time = esp_timer_get_time() / 1000;
    float elapsed_seconds = (current_time - init_time_) / 1000.0f;
    
    // Calculate sinusoidal velocity: base + max * sin(2*PI*frequency*time)
    float sin_value = sin(2.0 * M_PI * frequency_ * elapsed_seconds);
    float current_velocity = base_velocity_ + max_velocity_ * sin_value;
    
    // Determine direction based on velocity sign
    // Use a minimum velocity threshold to ensure motion starts
    const float min_velocity = 10.0f; // Minimum velocity to ensure motion
    
    if (current_velocity > min_velocity) {
      driver_->rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
      driver_->rampControl.SetMaxSpeed(current_velocity);
    } else if (current_velocity < -min_velocity) {
      driver_->rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_NEG);
      driver_->rampControl.SetMaxSpeed(-current_velocity);
    } else {
      // Near zero velocity - use small velocity to keep motor active
      if (current_velocity >= 0.0f) {
        driver_->rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
        driver_->rampControl.SetMaxSpeed(min_velocity);
      } else {
        driver_->rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_NEG);
        driver_->rampControl.SetMaxSpeed(min_velocity);
      }
    }
    
    // Check if we've completed the requested number of rounds
    if (rounds_ > 0) {
      float cycles_completed = elapsed_seconds * frequency_;
      if (cycles_completed >= rounds_) {
        // Stop motion
        driver_->rampControl.SetRampMode(tmc5160::RampMode::HOLD);
        driver_->rampControl.SetMaxSpeed(0.0);
        initialized_ = false;
        ESP_LOGI(TAG, "Sinusoidal motion completed: %d rounds", rounds_);
        return false;
      }
    }
    
    return true;
  }

  /**
   * @brief Stop sinusoidal motion
   */
  void Stop() {
    driver_->rampControl.SetRampMode(tmc5160::RampMode::HOLD);
    driver_->rampControl.SetMaxSpeed(0.0);
    initialized_ = false;
    ESP_LOGI(TAG, "Sinusoidal motion stopped");
  }
};

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Sinusoidal Motion Example for NEMA 44mm Motors");
  ESP_LOGI(TAG, "Using internal ramp generator with velocity control");

  // Get standard pin configuration
  auto pin_config = tmc5160_test_config::GetDefaultPinConfig();

  // Create SPI communication interface with pin configuration
  Esp32SPI spi(tmc5160_test_config::SPI_HOST, pin_config, 4000000); // 4 MHz SPI clock

  // Initialize SPI interface
  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC5160 driver instance
  tmc5160::TMC5160<Esp32SPI> driver(spi);

  // Configure driver for NEMA 44mm stepper motor with gearbox at 24V
  // Geared motors need: Higher current (more torque), Lower speed, Different acceleration
  // Typical specs: 200 steps/rev, 0.5-1.5A per phase, 24V operation
  // 24V provides better performance: faster acceleration, higher speeds, better torque
  tmc5160::DriverConfig cfg{};
  
  // Motor current settings for NEMA 44mm with gearbox at 24V
  // Geared motors need more current due to gearbox load and friction
  // Increase current for better torque to overcome gearbox resistance
  cfg.motor.global_scaler = 32;  // Standard scaling for small motors
  cfg.motor.irun = 24;            // Run current (~1.2A - increased for geared motor torque)
  cfg.motor.ihold = 10;           // Hold current (~0.5A, 40% of run - higher for gearbox)
  
  // Chopper settings for smooth motion at 24V
  cfg.chopper.toff = 5;           // Chopper off time (5 is good for small motors at 24V)
  cfg.chopper.mres = 4;           // 16 microsteps (256/16=16 microsteps per full step)
  cfg.chopper.intpol = true;      // Enable interpolation for smoother motion
  cfg.chopper.hend = 3;          // Hysteresis end (3 is good for small motors)
  cfg.chopper.hstrt = 0;         // Hysteresis start
  
  // StealthChop settings optimized for 24V operation
  // At 24V, PWM can be more aggressive for better performance
  cfg.stealthchop.pwm_ofs = 30;   // PWM offset for smooth start (good for 24V)
  cfg.stealthchop.pwm_grad = 0;   // PWM gradient (auto-gradient handles this)
  cfg.stealthchop.pwm_autoscale = true; // Auto-scale PWM (important for 24V)
  cfg.stealthchop.pwm_autograd = true;  // Auto-gradient PWM (optimizes for 24V)
  cfg.stealthchop.pwm_freq = 1;   // PWM frequency (1 = 23.4kHz, good for 24V operation)
  
  // Short protection (important for small motors)
  cfg.short_protection.s2vs_level = 6;  // Short to VS level (6 is conservative)
  cfg.short_protection.s2g_level = 4;  // Short to GND level (4 is conservative)

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully for NEMA 44mm motor");
  ESP_LOGI(TAG, "Motor settings: irun=%u, ihold=%u, microsteps=16, global_scaler=%u",
           cfg.motor.irun, cfg.motor.ihold, 16, cfg.motor.global_scaler);
  
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
  
  // 2. Set TCOOLTHRS to maximum to disable StallGuard2 at all speeds
  // TCOOLTHRS = velocity threshold below which StallGuard2 is disabled
  // Setting to maximum (0xFFFFF) ensures StallGuard2 is disabled at all velocities
  uint32_t tcoolthrs = 0xFFFFF; // Maximum value (20 bits) - disables SG2 at all speeds
  if (driver.GetComm().WriteRegister(tmc5160::Registers::TCOOLTHRS, tcoolthrs)) {
    ESP_LOGI(TAG, "✓ TCOOLTHRS set to maximum (0x%05X) - StallGuard2 disabled at all speeds", tcoolthrs);
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
  tmc5160::StallGuardConfig sg_cfg{};
  sg_cfg.sgt = 63;        // Maximum threshold (least sensitive) - won't trigger
  sg_cfg.semin = 0;       // Disable CoolStep (semin=0 means CoolStep off)
  sg_cfg.semax = 0;       // No hysteresis
  sg_cfg.sfilt = false;   // No filter
  if (driver.diagnostics.ConfigureStallGuard(sg_cfg)) {
    ESP_LOGI(TAG, "✓ StallGuard2 configured for diagnostics only (sgt=63, least sensitive)");
    ESP_LOGI(TAG, "  Note: StallGuard2 is DISABLED and will NOT stop the motor");
  } else {
    ESP_LOGW(TAG, "Failed to configure StallGuard2 (may not be critical)");
  }
  
  // Disable reference switches if not using them (prevents motion blocking)
  // If you have reference switches connected, configure them instead
  tmc5160::ReferenceSwitchConfig ref_cfg{};
  ref_cfg.stop_left_enable = false;   // Disable left reference switch
  ref_cfg.stop_right_enable = false;  // Disable right reference switch
  ref_cfg.en_softstop = false;        // Disable soft stop
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
        
        // Check for wiring issues
        if (drv_status.bits.sg_result == 0) {
          ESP_LOGE(TAG, "  ⚠️ WIRING ISSUE: SG=0 with no load suggests:");
          ESP_LOGE(TAG, "     - Motor phases may be swapped or incorrectly wired");
          ESP_LOGE(TAG, "     - Check: A+/A- and B+/B- connections");
          ESP_LOGE(TAG, "     - Try swapping one phase pair (A+/A- or B+/B-)");
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

  // Create sinusoidal motion controller
  SinusoidalMotion motion(&driver);

  // Configure sinusoidal motion for NEMA 44mm motor with gearbox
  // For 200 steps/rev with 16 microsteps = 3200 microsteps per revolution
  // Geared motors: Lower max velocity, higher base velocity to overcome gearbox friction
  // If motor "jerks" then stops, increase base_velocity and max_velocity
  double frequency = 0.5;        // Frequency in Hz (0.5 Hz = one complete cycle every 2 seconds)
  float max_velocity = 400.0;    // Maximum velocity in steps/s (increased from 300 for better motion)
  float base_velocity = 200.0;   // Base velocity offset (increased from 150 to overcome friction better)
  uint32_t update_period_ms = 50; // Update velocity every 50ms (20 Hz update rate)
  int rounds = -1;                // Number of complete cycles (-1 for infinite)
  
  motion.Config(frequency, max_velocity, base_velocity, update_period_ms, rounds);

  ESP_LOGI(TAG, "Starting sinusoidal motion for NEMA 44mm motor:");
  ESP_LOGI(TAG, "  Frequency: %.2f Hz (one cycle every %.1f seconds)",
           frequency, 1.0 / frequency);
  ESP_LOGI(TAG, "  Max velocity: %.1f steps/s", max_velocity);
  ESP_LOGI(TAG, "  Base velocity: %.1f steps/s", base_velocity);
  ESP_LOGI(TAG, "  Update period: %lu ms", update_period_ms);
  ESP_LOGI(TAG, "  Rounds: %d", rounds > 0 ? rounds : -1);
  ESP_LOGI(TAG, "  Using internal ramp generator - TMC5160 handles step generation");

  // Run sinusoidal motion
  // Update velocity periodically based on sinusoidal function
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
      
      // Note: VSTART, VSTOP, AMAX are WRITE-ONLY registers - they always read as 0
      // We can't verify them by reading, but we know they were set in SinusoidalMotion::Start()
      // VSTART=100, VSTOP=30, AMAX=1500 (from SetRampSpeeds and SetAccelerations)
      
      // Read RAMPMODE to verify we're in velocity mode
      uint32_t rampmode_value = 0;
      bool in_velocity_mode = false;
      if (driver.GetComm().ReadRegister(tmc5160::Registers::RAMPMODE, rampmode_value)) {
        in_velocity_mode = (rampmode_value == 1 || rampmode_value == 2); // VELOCITY_POS or VELOCITY_NEG
        ESP_LOGI(TAG, "  RAMPMODE=%lu (%s)", rampmode_value, 
                 in_velocity_mode ? "VELOCITY" : (rampmode_value == 0 ? "POSITIONING" : "HOLD"));
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
      // For typical NEMA 44mm with gearbox: 200 steps/rev motor, 16 microsteps = 3200 microsteps/rev
      // Position delta is in microsteps, so divide by microsteps per rev to get motor revolutions
      float motor_revolutions = position_delta / 3200.0f;
      
      // NOTE: Adjust this ratio based on your actual gearbox!
      // Example: If gearbox is 100:1, then output_revolutions = motor_revolutions / 100
      // For now, assume 1:1 to show motor movement (change this to your actual ratio)
      float gearbox_ratio = 1.0f; // CHANGE THIS to your actual gearbox ratio!
      float output_revolutions = motor_revolutions / gearbox_ratio;
      
      ESP_LOGI(TAG, "Diagnostics: VACTUAL=%.1f steps/s, XACTUAL=%d", actual_velocity, actual_position);
      ESP_LOGI(TAG, "  Position change: %d microsteps in %d ms = %.1f steps/s (calculated)", 
               position_delta, time_delta, position_change_rate);
      ESP_LOGI(TAG, "  Motor: %.3f rev (%.1f deg) in %.1f seconds", 
               motor_revolutions, motor_revolutions * 360.0f, time_delta / 1000.0f);
      ESP_LOGI(TAG, "  Output: %.3f rev (%.1f deg) [gearbox=%.0f:1]", 
               output_revolutions, output_revolutions * 360.0f, gearbox_ratio);
      
      // Calculate output speed
      float output_rpm = (output_revolutions * 60.0f) / (time_delta / 1000.0f);
      float output_deg_per_sec = output_revolutions * 360.0f / (time_delta / 1000.0f);
      
      // Check if motion is visible
      if (std::abs(output_revolutions) < 0.01f && time_delta > 500 && std::abs(position_delta) > 1000) {
        ESP_LOGW(TAG, "  ⚠️ Output shaft movement is very small (%.4f rev = %.2f deg) despite large motor movement", 
                 output_revolutions, output_revolutions * 360.0f);
        ESP_LOGW(TAG, "  Output speed: %.2f RPM, %.2f deg/s", output_rpm, output_deg_per_sec);
        ESP_LOGW(TAG, "  This suggests a HIGH gearbox ratio - motor moves many steps but output moves little");
        ESP_LOGW(TAG, "  Update gearbox_ratio in code (line ~725) to match your actual gearbox");
        ESP_LOGW(TAG, "  Tip: Mark the output shaft and observe movement, then adjust ratio accordingly");
      } else if (std::abs(output_revolutions) > 0.001f) {
        ESP_LOGI(TAG, "  ✓ Output shaft IS moving: %.3f rev (%.1f deg) at %.2f RPM (%.2f deg/s)", 
                 output_revolutions, output_revolutions * 360.0f, output_rpm, output_deg_per_sec);
      } else {
        ESP_LOGW(TAG, "  ⚠️ Output movement too small to measure (%.6f rev) - check gearbox_ratio setting", output_revolutions);
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
          ESP_LOGE(TAG, "  ⚠️ CRITICAL: SG result = 0 (highest load) but motor has NO mechanical load!");
          ESP_LOGE(TAG, "  This indicates a PROBLEM:");
          ESP_LOGE(TAG, "    1. Check motor wiring (phases may be swapped or wrong)");
          ESP_LOGE(TAG, "    2. Motor current may be too high (try reducing irun)");
          ESP_LOGE(TAG, "    3. StealthChop may need recalibration");
          ESP_LOGE(TAG, "    4. Motor may be electrically stalling");
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
    
    // Delay for update period
    vTaskDelay(pdMS_TO_TICKS(update_period_ms));
  }
}
