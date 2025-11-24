/**
 * @file ramp_control_comprehensive_test.cpp
 * @brief Comprehensive Ramp Control testing suite for TMC5160 (single motor)
 *
 * This file contains comprehensive testing for TMC5160 ramp control features:
 * - All ramp modes (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD)
 * - Position control (target, current, relative moves)
 * - Speed control (max speed, acceleration, deceleration)
 * - Ramp speeds (start, stop, transition)
 * - Power-down delay and zero wait time
 * - First acceleration phase
 * - Reference switch/endstop configuration
 * - Unit conversions (mm, RPM)
 *
 * Configured for NEMA 44mm x 44mm body stepper motors (2A rated):
 * - Typical specs: 1.8° step angle (200 steps/rev), 2.0A per phase, 24V
 * - Uses 16 microsteps for smooth motion
 * - Current set to ~1.6A run, ~0.6A hold (80% and 30% of 2A rated)
 * - Optimized for 24V operation
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - NEMA 44mm stepper motor (2A) connected to TMC5160
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
#include "../../../inc/tmc5160_units.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp32_tmc5160_bus_config.hpp"
#include "TestFramework.h"
#include <memory>

static const char* TAG = "RampControl_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
// Enable/disable specific test categories by setting to true or false

static constexpr bool ENABLE_RAMP_MODE_TESTS = true;
static constexpr bool ENABLE_POSITION_CONTROL_TESTS = true;
static constexpr bool ENABLE_SPEED_CONTROL_TESTS = true;
static constexpr bool ENABLE_RAMP_PARAMETER_TESTS = true;
static constexpr bool ENABLE_REFERENCE_SWITCH_TESTS = true;
static constexpr bool ENABLE_UNIT_CONVERSION_TESTS = true;

// Test configuration constants for NEMA 44mm 2A motor at 24V
// Current calculation: For 2A motor, use ~80% for run (1.6A), ~30% for hold (0.6A)
// With global_scaler=160 (recommended >128): 
// Scaled Max = 160/256 * 3A(typ) = ~1.875A
// irun=27 (27/32) * 1.875A = ~1.58A
static constexpr uint8_t TEST_IRUN = 27;          // Run current (~1.6A for 2A motor)
static constexpr uint8_t TEST_IHOLD = 10;         // Hold current (~0.6A for 2A motor)
static constexpr uint8_t TEST_GLOBAL_SCALER = 160; // Global scaler (>128 recommended by datasheet)
static constexpr uint8_t TEST_TOFF = 5;           // Chopper off time (5 is good for 2A motors)
static constexpr uint8_t TEST_MRES = 4;          // 16 microsteps (256/16=16 microsteps per full step)
static constexpr uint16_t STEPS_PER_REV = 200;   // Steps per revolution (1.8° per step)
static constexpr float LEAD_SCREW_PITCH_MM = 2.0F; // Lead screw pitch (adjust for your setup)

// Forward declarations
bool test_ramp_modes() noexcept;
bool test_position_control() noexcept;
bool test_speed_control() noexcept;
bool test_ramp_parameters() noexcept;
bool test_reference_switch_configuration() noexcept;
bool test_unit_conversions() noexcept;

// Helper functions
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;
  std::unique_ptr<tmc5160::TMC5160<Esp32SPI>> driver;
};

std::unique_ptr<TestDriverHandle> create_test_driver() noexcept {
  auto handle = std::make_unique<TestDriverHandle>();
  
  // Get complete pin configuration from test config
  tmc5160::Esp32SpiPinConfig pin_config = tmc5160_test_config::GetDefaultPinConfig();
  
  handle->spi = std::make_unique<Esp32SPI>(
    tmc5160_test_config::SPI_HOST,
    pin_config,
    tmc5160_test_config::SPI_CLOCK_SPEED_HZ);
  
  if (!handle->spi->Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return nullptr;
  }
  
  handle->driver = std::make_unique<tmc5160::TMC5160<Esp32SPI>>(*handle->spi);
  
  // Verify mode pins match expected communication mode (if pins are configured)
  gpio_num_t spi_mode_gpio = handle->spi->GetPinMapping(tmc5160::TMC5160CtrlPin::SPI_MODE);
  gpio_num_t sd_mode_gpio = handle->spi->GetPinMapping(tmc5160::TMC5160CtrlPin::SD_MODE);
  constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
  if (spi_mode_gpio != UNMAPPED_PIN && sd_mode_gpio != UNMAPPED_PIN) {
    tmc5160::ChipCommMode actual_mode;
    if (handle->driver->GetChipCommMode(actual_mode)) {
      if (actual_mode == tmc5160::ChipCommMode::SPI_INTERNAL_RAMP ||
          actual_mode == tmc5160::ChipCommMode::SPI_EXTERNAL_STEPDIR) {
        ESP_LOGI(TAG, "✓ Mode pin verification passed (SPI mode)");
      } else {
        ESP_LOGE(TAG, "✗ Mode pin verification FAILED: Mode pins indicate non-SPI mode");
      }
    }
  }
  
  // Configure driver for NEMA 44mm 2A stepper motor at 24V
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = TEST_IRUN;
  cfg.motor.ihold = TEST_IHOLD;
  cfg.motor.global_scaler = TEST_GLOBAL_SCALER;
  
  // Chopper settings for 2A motor at 24V
  cfg.chopper.toff = TEST_TOFF;
  cfg.chopper.mres = TEST_MRES;
  cfg.chopper.intpol = true;
  cfg.chopper.hend = 3;          // Hysteresis end (3 is good for 2A motors)
  cfg.chopper.hstrt = 0;         // Hysteresis start
  
  // StealthChop settings for quiet operation at 24V
  cfg.stealthchop.pwm_ofs = 30;   // PWM offset for smooth start
  cfg.stealthchop.pwm_autoscale = true; // Auto-scale PWM (important for 24V)
  cfg.stealthchop.pwm_autograd = true;  // Auto-gradient PWM
  cfg.stealthchop.pwm_freq = 1;   // PWM frequency (1 = 23.4kHz)
  
  // Power stage settings for 2A motor
  cfg.power_stage.drv_strength = 2;  // Driver strength (2 is good for 2A motors)
  cfg.power_stage.bbm_time = 24;     // Break-before-make time
  cfg.power_stage.bbm_clks = 4;      // Break-before-make clocks
  
  // Short protection for 2A motor
  cfg.short_protection.s2vs_level = 6;  // Short to VS level
  cfg.short_protection.s2g_level = 4;   // Short to GND level
  
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return nullptr;
  }
  
  return handle;
}

bool test_ramp_modes() noexcept {
  ESP_LOGI(TAG, "Testing ramp modes...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  bool success = true;
  
  // Test POSITIONING mode
  if (!handle->driver->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING)) {
    ESP_LOGE(TAG, "Failed to set POSITIONING mode");
    success = false;
  }
  
  // Test VELOCITY_POS mode
  if (!handle->driver->rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS)) {
    ESP_LOGE(TAG, "Failed to set VELOCITY_POS mode");
    success = false;
  }
  
  // Test VELOCITY_NEG mode
  if (!handle->driver->rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_NEG)) {
    ESP_LOGE(TAG, "Failed to set VELOCITY_NEG mode");
    success = false;
  }
  
  // Test HOLD mode
  if (!handle->driver->rampControl.SetRampMode(tmc5160::RampMode::HOLD)) {
    ESP_LOGE(TAG, "Failed to set HOLD mode");
    success = false;
  }
  
  // Return to POSITIONING mode
  handle->driver->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  
  return success;
}

bool test_position_control() noexcept {
  ESP_LOGI(TAG, "Testing position control...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  handle->driver->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  
  // Test setting target position
  if (!handle->driver->rampControl.SetTargetPosition(1000)) {
    ESP_LOGE(TAG, "Failed to set target position");
    return false;
  }
  
  // Test getting current position
  int32_t current = handle->driver->rampControl.GetCurrentPosition();
  ESP_LOGI(TAG, "Current position: %ld", current);
  
  // Test setting current position
  handle->driver->rampControl.SetCurrentPosition(0);
  current = handle->driver->rampControl.GetCurrentPosition();
  if (current != 0) {
    ESP_LOGW(TAG, "SetCurrentPosition may not have taken effect immediately");
  }
  
  return true;
}

bool test_speed_control() noexcept {
  ESP_LOGI(TAG, "Testing speed control...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  bool success = true;
  
  // Test setting max speed (appropriate for NEMA 44mm 2A motor)
  // 2000 steps/s = 10 rev/s = 600 RPM (good for small motors)
  if (!handle->driver->rampControl.SetMaxSpeed(2000.0F)) {
    ESP_LOGE(TAG, "Failed to set max speed");
    success = false;
  }
  
  // Test setting acceleration (appropriate for NEMA 44mm 2A motor)
  // 2000 steps/s² provides good acceleration for small motors
  if (!handle->driver->rampControl.SetAcceleration(2000.0F)) {
    ESP_LOGE(TAG, "Failed to set acceleration");
    success = false;
  }
  
  // Test setting accelerations (both) - higher decel for faster stopping
  if (!handle->driver->rampControl.SetAccelerations(2000.0F, 2500.0F)) {
    ESP_LOGE(TAG, "Failed to set accelerations");
    success = false;
  }
  
  // Test getting current speed
  float speed = handle->driver->rampControl.GetCurrentSpeed();
  ESP_LOGI(TAG, "Current speed: %.2f steps/s", speed);
  
  return success;
}

bool test_ramp_parameters() noexcept {
  ESP_LOGI(TAG, "Testing ramp parameters...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  bool success = true;
  
  // Test ramp speeds
  if (!handle->driver->rampControl.SetRampSpeeds(10.0F, 5.0F, 2.0F)) {
    ESP_LOGE(TAG, "Failed to set ramp speeds");
    success = false;
  }
  
  // Test power-down delay
  if (!handle->driver->rampControl.SetPowerDownDelay(100)) {
    ESP_LOGE(TAG, "Failed to set power-down delay");
    success = false;
  }
  
  // Test zero wait time
  if (!handle->driver->rampControl.SetZeroWaitTime(50)) {
    ESP_LOGE(TAG, "Failed to set zero wait time");
    success = false;
  }
  
  // Test first acceleration
  if (!handle->driver->rampControl.SetFirstAcceleration(300.0F)) {
    ESP_LOGE(TAG, "Failed to set first acceleration");
    success = false;
  }
  
  return success;
}

bool test_reference_switch_configuration() noexcept {
  ESP_LOGI(TAG, "Testing reference switch configuration and homing...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Test configuration
  tmc5160::ReferenceSwitchConfig ref_cfg{};
  ref_cfg.swap_left_right = false;
  ref_cfg.pol_stop_left = false;
  ref_cfg.pol_stop_right = false;
  ref_cfg.latch_left_active = true;
  ref_cfg.latch_right_active = true;
  
  if (!handle->driver->rampControl.ConfigureReferenceSwitch(ref_cfg)) {
    ESP_LOGE(TAG, "Failed to configure reference switch");
    return false;
  }

  // Test Switch Homing (Simulation / API check)
  // Since we don't have physical switches in this test, we just verify the API call works
  // and timeouts (as expected without a switch press).
  ESP_LOGI(TAG, "Testing PerformSwitchHoming API (expect timeout)...");
  int32_t final_pos = 0;
  // Use short timeout for test
  bool result = handle->driver->diagnostics.PerformSwitchHoming(true, 1000.0f, 100.0f, final_pos, true, 100);
  
  if (!result) {
    ESP_LOGI(TAG, "Homing timed out as expected (no physical switch)");
  } else {
    ESP_LOGW(TAG, "Homing reported success unexpectedly (switch noise?)");
  }
  
  return true;
}

bool test_unit_conversions() noexcept {
  ESP_LOGI(TAG, "Testing unit conversions...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  handle->driver->rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  
  bool success = true;
  
  // Test setting target position in millimeters
  if (!handle->driver->rampControl.SetTargetPositionMm(10.0F, STEPS_PER_REV, LEAD_SCREW_PITCH_MM)) {
    ESP_LOGE(TAG, "Failed to set target position in mm");
    success = false;
  }
  
  // Test setting max speed in RPM
  if (!handle->driver->rampControl.SetMaxSpeedRpm(60.0F, STEPS_PER_REV)) {
    ESP_LOGE(TAG, "Failed to set max speed in RPM");
    success = false;
  }
  
  // Test unit conversion functions
  float target_mm = 10.0F;
  int32_t steps = tmc5160::MmToSteps(target_mm, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
  ESP_LOGI(TAG, "%.2f mm = %ld steps", target_mm, steps);
  
  float target_rpm = 100.0F;
  float steps_per_sec = tmc5160::RpmToStepsPerSec(target_rpm, STEPS_PER_REV);
  ESP_LOGI(TAG, "%.2f RPM = %.2f steps/s", target_rpm, steps_per_sec);
  
  return success;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║              ESP32 TMC5160 RAMP CONTROL COMPREHENSIVE TEST SUITE            ║");
  ESP_LOGI(TAG, "║                         HardFOC TMC5160 Driver Tests                         ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  print_test_section_status(TAG, "Ramp Control");
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_RAMP_MODE_TESTS, "RAMP MODE TESTS", 5,
    ESP_LOGI(TAG, "Running ramp mode tests...");
    RUN_TEST_IN_TASK("ramp_modes", test_ramp_modes, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_POSITION_CONTROL_TESTS, "POSITION CONTROL TESTS", 5,
    ESP_LOGI(TAG, "Running position control tests...");
    RUN_TEST_IN_TASK("position_control", test_position_control, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_SPEED_CONTROL_TESTS, "SPEED CONTROL TESTS", 5,
    ESP_LOGI(TAG, "Running speed control tests...");
    RUN_TEST_IN_TASK("speed_control", test_speed_control, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_RAMP_PARAMETER_TESTS, "RAMP PARAMETER TESTS", 5,
    ESP_LOGI(TAG, "Running ramp parameter tests...");
    RUN_TEST_IN_TASK("ramp_parameters", test_ramp_parameters, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_REFERENCE_SWITCH_TESTS, "REFERENCE SWITCH TESTS", 5,
    ESP_LOGI(TAG, "Running reference switch tests...");
    RUN_TEST_IN_TASK("reference_switch", test_reference_switch_configuration, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_UNIT_CONVERSION_TESTS, "UNIT CONVERSION TESTS", 5,
    ESP_LOGI(TAG, "Running unit conversion tests...");
    RUN_TEST_IN_TASK("unit_conversions", test_unit_conversions, 8192, 1);
    flip_test_progress_indicator();
  );
  
  print_test_summary(g_test_results, "Ramp Control", TAG);
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

