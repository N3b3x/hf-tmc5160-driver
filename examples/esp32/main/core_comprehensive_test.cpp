/**
 * @file core_comprehensive_test.cpp
 * @brief Comprehensive Core initialization and basic setup testing suite for TMC5160 (single motor)
 *
 * This file contains comprehensive testing for TMC5160 core initialization and basic setup:
 * - Driver initialization and configuration
 * - Register read/write operations
 * - Setting and verifying motor parameters (current, microsteps, chopper)
 * - Setting and verifying ramp parameters (speed, acceleration)
 * - Global configuration settings
 * - Register persistence verification
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Single stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
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
#include "esp32_tmc5160_bus_config.hpp"
#include "TestFramework.h"
#include "driver/gpio.h"
#include <memory>

static const char* TAG = "Core_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
// Enable/disable specific test categories by setting to true or false

static constexpr bool ENABLE_INITIALIZATION_TESTS = true;
static constexpr bool ENABLE_REGISTER_ACCESS_TESTS = true;
static constexpr bool ENABLE_MOTOR_PARAMETER_TESTS = true;
static constexpr bool ENABLE_RAMP_PARAMETER_TESTS = true;
static constexpr bool ENABLE_GLOBAL_CONFIG_TESTS = true;

// Test configuration constants
namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
namespace Test = tmc5160_test_config::TestConfig_17HS4401S;

static constexpr uint8_t TEST_IRUN = Motor::IRUN;
static constexpr uint8_t TEST_IHOLD = Motor::IHOLD;
static constexpr uint8_t TEST_GLOBAL_SCALER = Motor::GLOBAL_SCALER;
static constexpr uint8_t TEST_TOFF = Motor::TOFF;
static constexpr uint8_t TEST_MRES = Motor::MRES; // 256 microsteps
static constexpr float MICROSTEPS = 256.0f;
// Steps per revolution for unit conversions (Output Shaft full steps * Microsteps)
// 17HS4401S-PG518: 200 steps * 5.18 ratio * 256 microsteps = ~265,216 steps/rev
static constexpr float STEPS_PER_REV = static_cast<float>(Motor::OUTPUT_FULL_STEPS) * MICROSTEPS;

static constexpr float TEST_MAX_SPEED = STEPS_PER_REV * 1.0f; // 1 rev/s
static constexpr float TEST_ACCELERATION = TEST_MAX_SPEED * 2.0f; // 0.5s to full speed
static constexpr float TEST_DECELERATION = TEST_MAX_SPEED * 2.0f;

// Forward declarations
bool test_driver_initialization() noexcept;
bool test_register_read_write() noexcept;
bool test_motor_parameter_settings() noexcept;
bool test_ramp_parameter_settings() noexcept;
bool test_global_configuration() noexcept;

// Helper functions
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;
  std::unique_ptr<tmc5160::TMC5160<Esp32SPI>> driver;
};

/**
 * @brief Verify mode pins match expected communication mode
 * @param spi SPI communication interface
 * @param driver TMC5160 driver instance
 * @param expected_comm_mode Expected communication mode (SPI or UART)
 * @return true if verification passed or pins not configured, false on mismatch
 */
bool verify_mode_pins(const Esp32SPI& spi, const tmc5160::TMC5160<Esp32SPI>& driver, tmc5160::CommMode expected_comm_mode) noexcept {
  // Check if mode pins are configured
  gpio_num_t spi_mode_gpio = spi.GetPinMapping(tmc5160::TMC5160CtrlPin::SPI_MODE);
  gpio_num_t sd_mode_gpio = spi.GetPinMapping(tmc5160::TMC5160CtrlPin::SD_MODE);
  
  constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
  
  // If pins are not configured, skip verification
  if (spi_mode_gpio == UNMAPPED_PIN || sd_mode_gpio == UNMAPPED_PIN) {
    ESP_LOGI(TAG, "Mode pins not configured, skipping verification");
    return true;
  }
  
  // Read mode pins
  tmc5160::ChipCommMode actual_mode;
  if (!driver.GetChipCommMode(actual_mode)) {
    ESP_LOGW(TAG, "Failed to read mode pins for verification");
    return false;
  }
  
  // Verify mode matches expected communication interface
  bool mode_valid = false;
  const char* mode_name = nullptr;
  
  if (expected_comm_mode == tmc5160::CommMode::SPI) {
    // For SPI, we expect SPI_MODE=HIGH
    // SD_MODE can be LOW (internal ramp) or HIGH (external step/dir)
    if (actual_mode == tmc5160::ChipCommMode::SPI_INTERNAL_RAMP ||
        actual_mode == tmc5160::ChipCommMode::SPI_EXTERNAL_STEPDIR) {
      mode_valid = true;
      mode_name = (actual_mode == tmc5160::ChipCommMode::SPI_INTERNAL_RAMP) 
                  ? "SPI + Internal Ramp" 
                  : "SPI + External Step/Dir";
    }
  } else if (expected_comm_mode == tmc5160::CommMode::UART) {
    // For UART, we expect SPI_MODE=LOW, SD_MODE=LOW
    if (actual_mode == tmc5160::ChipCommMode::UART_INTERNAL_RAMP) {
      mode_valid = true;
      mode_name = "UART + Internal Ramp";
    }
  }
  
  if (mode_valid) {
    ESP_LOGI(TAG, "✓ Mode pin verification passed: %s (matches %s interface)", 
             mode_name,
             (expected_comm_mode == tmc5160::CommMode::SPI) ? "SPI" : "UART");
    return true;
  } else {
    ESP_LOGE(TAG, "✗ Mode pin verification FAILED: Mode pins indicate %d, but using %s interface",
             static_cast<int>(actual_mode),
             (expected_comm_mode == tmc5160::CommMode::SPI) ? "SPI" : "UART");
    ESP_LOGE(TAG, "  Expected: SPI_MODE=%s, SD_MODE=%s for %s",
             (expected_comm_mode == tmc5160::CommMode::SPI) ? "HIGH" : "LOW",
             (expected_comm_mode == tmc5160::CommMode::SPI) ? "LOW/HIGH" : "LOW",
             (expected_comm_mode == tmc5160::CommMode::SPI) ? "SPI" : "UART");
    return false;
  }
}

std::unique_ptr<TestDriverHandle> create_test_driver() noexcept {
  auto handle = std::make_unique<TestDriverHandle>();
  
  // Get complete pin configuration from test config
  // This includes both SPI pins and TMC5160 control pins in one structure
  tmc5160::Esp32SpiPinConfig pin_config = tmc5160_test_config::GetDefaultPinConfig();
  
  // Create SPI communication interface using complete pin configuration
  handle->spi = std::make_unique<Esp32SPI>(
    tmc5160_test_config::SPI_HOST,
    pin_config,
    tmc5160_test_config::SPI_CLOCK_SPEED_HZ);
  
  // Initialize SPI interface
  if (!handle->spi->Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return nullptr;
  }
  
  // Explicitly set chain length to 1 for single-chip mode
  // This avoids auto-detection and ensures reliable communication
  handle->spi->SetDaisyChainLength(1);
  ESP_LOGI(TAG, "Configured for single-chip mode (chain length = 1)");
  
  // Create TMC5160 driver instance
  handle->driver = std::make_unique<tmc5160::TMC5160<Esp32SPI>>(*handle->spi);
  
  // Verify mode pins match expected communication mode (if pins are configured)
  if (!verify_mode_pins(*handle->spi, *handle->driver, tmc5160::CommMode::SPI)) {
    ESP_LOGE(TAG, "Mode pin verification failed - driver may not work correctly");
    // Continue anyway, but log the error
  }
  
  return handle;
}

bool test_driver_initialization() noexcept {
  ESP_LOGI(TAG, "Testing driver initialization...");
  
  auto handle = create_test_driver();
  if (!handle) {
    ESP_LOGE(TAG, "Failed to create test driver");
    return false;
  }
  
  // Configure driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = TEST_IRUN;
  cfg.motor.ihold = TEST_IHOLD;
  cfg.motor.global_scaler = TEST_GLOBAL_SCALER;
  cfg.chopper.toff = TEST_TOFF;
  cfg.chopper.mres = TEST_MRES;
  cfg.chopper.intpol = Motor::INTERPOLATION;
  cfg.chopper.hend = Motor::HEND;
  cfg.chopper.hstrt = Motor::HSTRT;
  cfg.chopper.tbl = Motor::TBL;
  
  cfg.stealthchop.pwm_ofs = Motor::STEALTH_OFS;
  cfg.stealthchop.pwm_autoscale = Motor::STEALTH_AUTOSCALE;
  cfg.stealthchop.pwm_autograd = Motor::STEALTH_AUTOGRAD;
  cfg.stealthchop.pwm_freq = Motor::STEALTH_FREQ;

  cfg.power_stage.drv_strength = 2;
  cfg.power_stage.bbm_time = 24;
  cfg.power_stage.bbm_clks = 4;
  
  // Initialize driver
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return false;
  }
  
  ESP_LOGI(TAG, "Driver initialized successfully");
  
  // Verify driver status
  tmc5160::DriverStatus status = handle->driver->diagnostics.GetStatus();
  ESP_LOGI(TAG, "Driver Status: %d", static_cast<int>(status));
  
  return true;
}

bool test_register_read_write() noexcept {
  ESP_LOGI(TAG, "Testing register read/write operations...");
  
  auto handle = create_test_driver();
  if (!handle) {
    ESP_LOGE(TAG, "Failed to create test driver");
    return false;
  }
  
  // Initialize driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = TEST_IRUN;
  cfg.motor.ihold = TEST_IHOLD;
  cfg.motor.global_scaler = TEST_GLOBAL_SCALER;
  cfg.chopper.toff = TEST_TOFF;
  cfg.chopper.mres = TEST_MRES;
  
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return false;
  }
  
  // Test reading GSTAT register (RWC - read-write-clear, reading is OK)
  uint32_t gstat_value = 0;
  if (!handle->driver->GetComm().ReadRegister(tmc5160::Registers::GSTAT, gstat_value)) {
    ESP_LOGE(TAG, "Failed to read GSTAT register");
    return false;
  }
  ESP_LOGI(TAG, "GSTAT register value: 0x%08lX", gstat_value);
  
  // Note: GLOBAL_SCALER (0x0B) is write-only per datasheet
  // Write verification is done via response data in WriteRegister()
  // We trust that WriteRegister() succeeded if it returned true
  
  // Test writing X_COMPARE register (write-only per datasheet)
  // Note: X_COMPARE (0x05) is write-only, cannot be read back
  // Write verification is done via response data in WriteRegister()
  constexpr uint32_t TEST_X_COMPARE = 12345;
  if (!handle->driver->GetComm().WriteRegister(tmc5160::Registers::X_COMPARE, TEST_X_COMPARE)) {
    ESP_LOGE(TAG, "Failed to write X_COMPARE register");
    return false;
  }
  ESP_LOGI(TAG, "X_COMPARE register written: 0x%08lX (write-only register, verified via write response)", TEST_X_COMPARE);
  
  ESP_LOGI(TAG, "Register read/write test passed");
  return true;
}

bool test_motor_parameter_settings() noexcept {
  ESP_LOGI(TAG, "Testing motor parameter settings...");
  
  auto handle = create_test_driver();
  if (!handle) {
    ESP_LOGE(TAG, "Failed to create test driver");
    return false;
  }
  
  // Initialize driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = TEST_IRUN;
  cfg.motor.ihold = TEST_IHOLD;
  cfg.motor.global_scaler = TEST_GLOBAL_SCALER;
  cfg.chopper.toff = TEST_TOFF;
  cfg.chopper.mres = TEST_MRES;
  cfg.chopper.intpol = Motor::INTERPOLATION;
  cfg.chopper.hend = Motor::HEND;
  cfg.chopper.hstrt = Motor::HSTRT;
  cfg.chopper.tbl = Motor::TBL;
  
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return false;
  }
  
  // Note: IHOLD_IRUN (0x10) is write-only per datasheet
  // Write verification is done via response data in WriteRegister()
  // We trust that Initialize() and SetCurrent() succeeded if they returned true
  ESP_LOGI(TAG, "IHOLD_IRUN configured: irun=%u, ihold=%u (write-only register, verified via write response)", 
           TEST_IRUN, TEST_IHOLD);
  
  // Verify chopper settings
  uint32_t chopconf_value = 0;
  if (!handle->driver->GetComm().ReadRegister(tmc5160::Registers::CHOPCONF, chopconf_value)) {
    ESP_LOGE(TAG, "Failed to read CHOPCONF register");
    return false;
  }
  
  tmc5160::CHOPCONF_Register chopconf{};
  chopconf.value = chopconf_value;
  ESP_LOGI(TAG, "CHOPCONF: toff=%u, mres=%u, intpol=%u", chopconf.bits.toff, chopconf.bits.mres, chopconf.bits.intpol);
  
  if (chopconf.bits.toff != TEST_TOFF) {
    ESP_LOGE(TAG, "TOFF mismatch: expected %u, got %u", TEST_TOFF, chopconf.bits.toff);
    return false;
  }
  if (chopconf.bits.mres != TEST_MRES) {
    ESP_LOGE(TAG, "MRES mismatch: expected %u, got %u", TEST_MRES, chopconf.bits.mres);
    return false;
  }
  if (chopconf.bits.intpol != Motor::INTERPOLATION) {
    ESP_LOGE(TAG, "INTPOL mismatch: expected %d, got %u", Motor::INTERPOLATION, chopconf.bits.intpol);
    return false;
  }
  
  // Test setting new motor current values
  // Note: IHOLD_IRUN (0x10) is write-only per datasheet
  constexpr uint8_t NEW_IRUN = 25;
  constexpr uint8_t NEW_IHOLD = 15;
  if (!handle->driver->motorControl.SetCurrent(NEW_IRUN, NEW_IHOLD)) {
    ESP_LOGE(TAG, "Failed to set motor current");
    return false;
  }
  ESP_LOGI(TAG, "Updated IHOLD_IRUN: irun=%u, ihold=%u (write-only register, verified via write response)", 
           NEW_IRUN, NEW_IHOLD);
  
  ESP_LOGI(TAG, "Motor parameter settings test passed");
  return true;
}

bool test_ramp_parameter_settings() noexcept {
  ESP_LOGI(TAG, "Testing ramp parameter settings...");
  
  auto handle = create_test_driver();
  if (!handle) {
    ESP_LOGE(TAG, "Failed to create test driver");
    return false;
  }
  
  // Initialize driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = TEST_IRUN;
  cfg.motor.ihold = TEST_IHOLD;
  cfg.motor.global_scaler = TEST_GLOBAL_SCALER;
  cfg.chopper.toff = TEST_TOFF;
  cfg.chopper.mres = TEST_MRES;
  
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return false;
  }
  
  // Set ramp parameters
  if (!handle->driver->rampControl.SetMaxSpeed(TEST_MAX_SPEED)) {
    ESP_LOGE(TAG, "Failed to set max speed");
    return false;
  }
  if (!handle->driver->rampControl.SetAccelerations(TEST_ACCELERATION, TEST_DECELERATION)) {
    ESP_LOGE(TAG, "Failed to set accelerations");
    return false;
  }
  
  // Note: VMAX (0x27), AMAX (0x26), and DMAX (0x28) are write-only per datasheet
  // Write verification is done via response data in WriteRegister()
  // We trust that SetMaxSpeed() and SetAccelerations() succeeded if they returned true
  ESP_LOGI(TAG, "Ramp parameters set: VMAX=%.1f steps/s, AMAX=%.1f steps/s², DMAX=%.1f steps/s² (write-only registers, verified via write response)",
           TEST_MAX_SPEED, TEST_ACCELERATION, TEST_DECELERATION);
  
  // Test setting target position
  constexpr int32_t TEST_TARGET = 10000;
  handle->driver->rampControl.SetTargetPosition(TEST_TARGET);
  
  int32_t current_pos = handle->driver->rampControl.GetCurrentPosition();
  ESP_LOGI(TAG, "Target position set to %ld, current position: %ld", TEST_TARGET, current_pos);
  
  ESP_LOGI(TAG, "Ramp parameter settings test passed");
  return true;
}

bool test_global_configuration() noexcept {
  ESP_LOGI(TAG, "Testing global configuration settings...");
  
  auto handle = create_test_driver();
  if (!handle) {
    ESP_LOGE(TAG, "Failed to create test driver");
    return false;
  }
  
  // Initialize driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = TEST_IRUN;
  cfg.motor.ihold = TEST_IHOLD;
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return false;
  }
  
  // Configure global settings
  tmc5160::GlobalConfig gconf{};
  gconf.faststandstill = true;
  gconf.diag0_error = true;
  gconf.diag0_otpw = true;
  gconf.multistep_filt = true;
  handle->driver->motorControl.ConfigureGlobalConfig(gconf);
  
  // Read back and verify
  uint32_t gconf_value = 0;
  if (!handle->driver->GetComm().ReadRegister(tmc5160::Registers::GCONF, gconf_value)) {
    ESP_LOGE(TAG, "Failed to read GCONF register");
    return false;
  }
  
  tmc5160::GCONF_Register gconf_reg{};
  gconf_reg.value = gconf_value;
  ESP_LOGI(TAG, "GCONF: faststandstill=%u, diag0_error=%u, diag0_otpw=%u, multistep_filt=%u",
           gconf_reg.bits.faststandstill, gconf_reg.bits.diag0_error, gconf_reg.bits.diag0_otpw,
           gconf_reg.bits.multistep_filt);
  
  if (gconf_reg.bits.faststandstill != 1) {
    ESP_LOGE(TAG, "faststandstill not set correctly");
    return false;
  }
  if (gconf_reg.bits.diag0_error != 1) {
    ESP_LOGE(TAG, "diag0_error not set correctly");
    return false;
  }
  if (gconf_reg.bits.diag0_otpw != 1) {
    ESP_LOGE(TAG, "diag0_otpw not set correctly");
    return false;
  }
  if (gconf_reg.bits.multistep_filt != 1) {
    ESP_LOGE(TAG, "multistep_filt not set correctly");
    return false;
  }
  
  ESP_LOGI(TAG, "Global configuration test passed");
  return true;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                   ESP32 TMC5160 CORE COMPREHENSIVE TEST SUITE              ║");
  ESP_LOGI(TAG, "║                         HardFOC TMC5160 Driver Tests                         ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  // Report test section configuration
  print_test_section_status(TAG, "Core");
  
  // Run all Core tests based on configuration
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_INITIALIZATION_TESTS, "CORE INITIALIZATION TESTS", 5,
    ESP_LOGI(TAG, "Running core initialization tests...");
    RUN_TEST_IN_TASK("driver_initialization", test_driver_initialization, 8192, 1);
    flip_test_progress_indicator();
  );

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_REGISTER_ACCESS_TESTS, "REGISTER ACCESS TESTS", 5,
    ESP_LOGI(TAG, "Running register access tests...");
    RUN_TEST_IN_TASK("register_read_write", test_register_read_write, 8192, 1);
    flip_test_progress_indicator();
  );

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_MOTOR_PARAMETER_TESTS, "MOTOR PARAMETER TESTS", 5,
    ESP_LOGI(TAG, "Running motor parameter tests...");
    RUN_TEST_IN_TASK("motor_parameter_settings", test_motor_parameter_settings, 8192, 1);
    flip_test_progress_indicator();
  );

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_RAMP_PARAMETER_TESTS, "RAMP PARAMETER TESTS", 5,
    ESP_LOGI(TAG, "Running ramp parameter tests...");
    RUN_TEST_IN_TASK("ramp_parameter_settings", test_ramp_parameter_settings, 8192, 1);
    flip_test_progress_indicator();
  );

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_GLOBAL_CONFIG_TESTS, "GLOBAL CONFIGURATION TESTS", 5,
    ESP_LOGI(TAG, "Running global configuration tests...");
    RUN_TEST_IN_TASK("global_configuration", test_global_configuration, 8192, 1);
    flip_test_progress_indicator();
  );
  
  print_test_summary(g_test_results, "Core", TAG);
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

