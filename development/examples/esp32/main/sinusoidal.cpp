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

    // Set acceleration for smooth velocity changes (higher for faster response)
    driver_->rampControl.SetAccelerations(2000.0, 2000.0); // 2000 steps/s²
    
    // Set start/stop velocities - VSTART must be > 0 for motion to start
    // VSTOP should be low for smooth stopping, VSTART should be low for smooth starting
    driver_->rampControl.SetRampSpeeds(50.0, 50.0, 0.0); // VSTART=50, VSTOP=50, V1=0
    
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

  // Configure driver for NEMA 44mm stepper motor at 24V
  // Typical specs: 200 steps/rev, 0.5-1.5A per phase, 24V operation
  // 24V provides better performance: faster acceleration, higher speeds, better torque
  tmc5160::DriverConfig cfg{};
  
  // Motor current settings for NEMA 44mm at 24V (adjust based on your motor specs)
  // For 1.0A motor: global_scaler=32, irun=20 (80% of max), ihold=7 (30% of max)
  // For 0.8A motor: global_scaler=32, irun=16, ihold=5
  // For 1.2A motor: global_scaler=32, irun=24, ihold=8
  // Note: At 24V, you can run higher currents safely due to better heat dissipation
  cfg.motor.global_scaler = 32;  // Standard scaling for small motors
  cfg.motor.irun = 20;            // Run current (~1.0A for typical NEMA 44mm)
  cfg.motor.ihold = 7;            // Hold current (~0.35A, 30% of run)
  
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

  // Create sinusoidal motion controller
  SinusoidalMotion motion(&driver);

  // Configure sinusoidal motion for NEMA 44mm motor
  // For 200 steps/rev with 16 microsteps = 3200 microsteps per revolution
  double frequency = 0.5;        // Frequency in Hz (0.5 Hz = one complete cycle every 2 seconds)
  float max_velocity = 500.0;    // Maximum velocity in steps/s (reduced for smoother motion)
  float base_velocity = 100.0;   // Base velocity offset (100 steps/s ensures motion never stops)
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
      
      ESP_LOGI(TAG, "Diagnostics: VACTUAL=%.1f steps/s, XACTUAL=%d, RAMP_STAT=0x%08X",
               actual_velocity, actual_position, has_ramp_stat ? ramp_stat : 0);
      
      if (has_ramp_stat) {
        tmc5160::RAMP_STAT_Register status{};
        status.value = ramp_stat;
        ESP_LOGI(TAG, "  Ramp status: vzero=%d, velocity_reached=%d, position_reached=%d",
                 status.bits.vzero ? 1 : 0,
                 status.bits.velocity_reached ? 1 : 0,
                 status.bits.position_reached ? 1 : 0);
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
