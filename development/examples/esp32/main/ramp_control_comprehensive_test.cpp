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

// Motor configuration using 17HS4401S-PG518 profile from header
namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
namespace Test = tmc5160_test_config::TestConfig_17HS4401S;

static constexpr uint8_t TEST_IRUN = Motor::IRUN;
static constexpr uint8_t TEST_IHOLD = Motor::IHOLD;
static constexpr uint8_t TEST_GLOBAL_SCALER = Motor::GLOBAL_SCALER;
static constexpr uint8_t TEST_TOFF = Motor::TOFF;
static constexpr uint8_t TEST_MRES = Motor::MRES; // 0 (256 microsteps)
static constexpr float MICROSTEPS = 256.0f;
// Steps per revolution for unit conversions (Output Shaft full steps * Microsteps)
// 17HS4401S-PG518: 200 steps * 5.18 ratio * 256 microsteps = ~265,216 steps/rev
static constexpr float STEPS_PER_REV = static_cast<float>(Motor::OUTPUT_FULL_STEPS) * MICROSTEPS;
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
  
  // Configure driver for NEMA 17 (17HS4401S-PG518)
  tmc5160::DriverConfig cfg{};
  cfg.motor_spec.irun = TEST_IRUN;
  cfg.motor_spec.ihold = TEST_IHOLD;
  cfg.motor_spec.global_scaler = TEST_GLOBAL_SCALER;
  
  // Chopper settings
  cfg.chopper.toff = TEST_TOFF;
  cfg.chopper.mres = TEST_MRES;
  cfg.chopper.intpol = Motor::INTERPOLATION;
  cfg.chopper.hend = Motor::HEND;
  cfg.chopper.hstrt = Motor::HSTRT;
  cfg.chopper.tbl = Motor::TBL;
  
  // StealthChop settings
  cfg.stealthchop.pwm_ofs = Motor::STEALTH_OFS;
  cfg.stealthchop.pwm_autoscale = Motor::STEALTH_AUTOSCALE;
  cfg.stealthchop.pwm_autograd = Motor::STEALTH_AUTOGRAD;
  cfg.stealthchop.pwm_freq = Motor::STEALTH_FREQ;
  
  // Power stage settings for 2A motor
  // Power stage: typical MOSFET with ~30nC Miller charge, 200ns BBM time
  cfg.power_stage.mosfet_miller_charge_nc = 30.0f;
  cfg.power_stage.bbm_time_ns = 200;
  
  // Short protection for 2A motor (user-friendly voltage thresholds)
  cfg.power_stage.s2vs_voltage_mv = 625;  // 625mV = S2VS_LEVEL=6 (recommended)
  cfg.power_stage.s2g_voltage_mv = 500;  // ~500mV = S2G_LEVEL=4 (higher sensitivity)
  
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
  // Use 2 revolutions per second (output shaft)
  float max_speed = STEPS_PER_REV * 2.0f;
  if (!handle->driver->rampControl.SetMaxSpeed(max_speed)) {
    ESP_LOGE(TAG, "Failed to set max speed");
    success = false;
  }
  
  // Test setting acceleration (appropriate for NEMA 44mm 2A motor)
  // Reach max speed in 0.5s
  float accel = max_speed * 2.0f;
  if (!handle->driver->rampControl.SetAcceleration(accel)) {
    ESP_LOGE(TAG, "Failed to set acceleration");
    success = false;
  }
  
  // Test setting accelerations (both) - higher decel for faster stopping
  if (!handle->driver->rampControl.SetAcceleration(accel)) {
    ESP_LOGE(TAG, "Failed to set acceleration");
    success = false;
  }
  if (!handle->driver->rampControl.SetDeceleration(accel * 1.5f)) {
    ESP_LOGE(TAG, "Failed to set deceleration");
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
  
  float vstart = STEPS_PER_REV * 0.01f; // 0.01 RPS start
  float vstop = STEPS_PER_REV * 0.005f; // 0.005 RPS stop
  
  // Test ramp speeds
  if (!handle->driver->rampControl.SetRampSpeeds(vstart, vstop, 0.0f)) {
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
  if (!handle->driver->rampControl.SetFirstAcceleration(vstart * 5.0f)) {
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
  
  // Test configuration using defaults from header
  tmc5160::ReferenceSwitchConfig ref_cfg{};
  ref_cfg.swap_left_right = Test::Switches::SWAP_INPUTS;
  ref_cfg.pol_stop_left = Test::Switches::POLARITY_LEFT;
  ref_cfg.pol_stop_right = Test::Switches::POLARITY_RIGHT;
  ref_cfg.latch_left_active = Test::Switches::LATCH_LEFT;
  ref_cfg.latch_right_active = Test::Switches::LATCH_RIGHT;
  
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
  bool result = handle->driver->diagnostics.PerformSwitchHoming(true, 
                                                                Test::Motion::HOMING_SEARCH_SPEED, 
                                                                Test::Motion::HOMING_SWITCH_SPEED, 
                                                                final_pos, true, 100);
  
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

