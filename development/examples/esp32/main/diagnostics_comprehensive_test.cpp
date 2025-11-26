/**
 * @file diagnostics_comprehensive_test.cpp
 * @brief Comprehensive Diagnostics testing suite for TMC5160 (single motor)
 *
 * This file contains comprehensive testing for TMC5160 diagnostics features:
 * - Driver status
 * - StallGuard2 configuration and reading
 * - Lost steps detection
 * - Phase currents
 * - PWM scale
 * - Microstep counter and current
 * - Time between microsteps
 * - GPIO pin reading
 * - Factory and OTP configuration reading
 * - UART transmission count
 * - Offset calibration reading
 * - Sensorless homing
 * - Open load detection
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
#include <memory>

static const char* TAG = "Diagnostics_Test";
static TestResults g_test_results;

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// Motor selection (compile-time constant)
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;

// Board selection (compile-time constant)
static constexpr tmc5160_test_config::BoardType SELECTED_BOARD = 
    tmc5160_test_config::BoardType::BOARD_TMC5160_EVAL;

// Platform selection (compile-time constant)
static constexpr tmc5160_test_config::PlatformType SELECTED_PLATFORM = 
    tmc5160_test_config::PlatformType::PLATFORM_TEST_RIG;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_DRIVER_STATUS_TESTS = true;
static constexpr bool ENABLE_STALLGUARD_TESTS = true;
static constexpr bool ENABLE_LOST_STEPS_TESTS = true;
static constexpr bool ENABLE_PHASE_CURRENT_TESTS = true;
static constexpr bool ENABLE_PWM_SCALE_TESTS = true;
static constexpr bool ENABLE_MICROSTEP_DIAGNOSTICS_TESTS = true;
static constexpr bool ENABLE_GPIO_TESTS = true;
static constexpr bool ENABLE_FACTORY_OTP_TESTS = true;
static constexpr bool ENABLE_UART_COUNT_TESTS = true;
static constexpr bool ENABLE_OFFSET_CALIBRATION_TESTS = true;
static constexpr bool ENABLE_SENSORLESS_HOMING_TESTS = true;
static constexpr bool ENABLE_OPEN_LOAD_TESTS = true;

// Test configuration constants
namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
namespace Test = tmc5160_test_config::TestConfig_17HS4401S;

static constexpr uint8_t TEST_IRUN = Motor::IRUN;
static constexpr uint8_t TEST_IHOLD = Motor::IHOLD;
static constexpr uint8_t TEST_GLOBAL_SCALER = Motor::GLOBAL_SCALER;
static constexpr uint8_t TEST_TOFF = Motor::TOFF;
static constexpr uint8_t TEST_MRES = Motor::MRES; // 256 microsteps

// Forward declarations
bool test_driver_status() noexcept;
bool test_stallguard() noexcept;
bool test_lost_steps() noexcept;
bool test_phase_currents() noexcept;
bool test_pwm_scale() noexcept;
bool test_microstep_diagnostics() noexcept;
bool test_gpio_pins() noexcept;
bool test_factory_otp_config() noexcept;
bool test_uart_transmission_count() noexcept;
bool test_offset_calibration() noexcept;
bool test_sensorless_homing() noexcept;
bool test_open_load() noexcept;

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
  
  tmc5160::DriverConfig cfg{};
  
  // Use helper function to configure from motor/platform specs
  // Configure motor
  if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Direct(cfg);
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    tmc5160_test_config::ConfigureDriverFromMotor_AppliedMotion_5034(cfg);
  }
  
  // Apply board configuration
  tmc5160_test_config::ApplyBoardConfig<SELECTED_BOARD>(cfg);
  
  // Apply platform configuration
  tmc5160_test_config::ApplyPlatformConfig<SELECTED_PLATFORM>(cfg);
  
  // Override with test-specific values if needed
  cfg.chopper.mres = TEST_MRES;
  
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return nullptr;
  }
  
  return handle;
}

bool test_driver_status() noexcept {
  ESP_LOGI(TAG, "Testing driver status...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  tmc5160::DriverStatus status = handle->driver->diagnostics.GetStatus();
  ESP_LOGI(TAG, "Driver Status: %d", static_cast<int>(status));
  
  return true;
}

bool test_stallguard() noexcept {
  ESP_LOGI(TAG, "Testing StallGuard2...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Use default test configuration for StallGuard
  tmc5160::StallGuardConfig sg_cfg{};
  sg_cfg.threshold = Test::StallGuard::SGT_HOMING;
  sg_cfg.enable_filter = Test::StallGuard::FILTER_ENABLED;
  // Note: semin/semax are CoolStep parameters, not StallGuard2 parameters
  
  if (!handle->driver->diagnostics.ConfigureStallGuard(sg_cfg)) {
    ESP_LOGE(TAG, "Failed to configure StallGuard");
    return false;
  }
  
  uint16_t sg_value = handle->driver->diagnostics.GetStallGuard();
  ESP_LOGI(TAG, "StallGuard Value: %u (threshold=%d)", sg_value, sg_cfg.threshold);
  
  return true;
}

bool test_lost_steps() noexcept {
  ESP_LOGI(TAG, "Testing lost steps detection...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  uint32_t lost_steps = handle->driver->diagnostics.GetLostSteps();
  ESP_LOGI(TAG, "Lost Steps: %lu", lost_steps);
  
  return true;
}

bool test_phase_currents() noexcept {
  ESP_LOGI(TAG, "Testing phase currents...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  int16_t phase_a = 0, phase_b = 0;
  if (!handle->driver->diagnostics.GetMicrostepCurrent(phase_a, phase_b)) {
    ESP_LOGE(TAG, "Failed to get microstep currents");
    return false;
  }
  
  ESP_LOGI(TAG, "Phase A: %d, Phase B: %d", phase_a, phase_b);
  
  return true;
}

bool test_pwm_scale() noexcept {
  ESP_LOGI(TAG, "Testing PWM scale...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  uint8_t pwm_scale_sum = 0;
  int16_t pwm_scale_auto = 0;
  if (!handle->driver->diagnostics.GetPwmScale(pwm_scale_sum, pwm_scale_auto)) {
    ESP_LOGE(TAG, "Failed to get PWM scale");
    return false;
  }
  
  ESP_LOGI(TAG, "PWM Scale Sum: %u, Auto: %d", pwm_scale_sum, pwm_scale_auto);
  
  return true;
}

bool test_microstep_diagnostics() noexcept {
  ESP_LOGI(TAG, "Testing microstep diagnostics...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  uint32_t time_between = handle->driver->diagnostics.GetTimeBetweenMicrosteps();
  ESP_LOGI(TAG, "Time Between Microsteps: %lu", time_between);
  
  uint16_t mscnt = handle->driver->diagnostics.GetMicrostepCounter();
  ESP_LOGI(TAG, "Microstep Counter: %u", mscnt);
  
  int16_t ms_current_a = 0, ms_current_b = 0;
  handle->driver->diagnostics.GetMicrostepCurrent(ms_current_a, ms_current_b);
  ESP_LOGI(TAG, "Microstep Current A: %d, B: %d", ms_current_a, ms_current_b);
  
  return true;
}

bool test_gpio_pins() noexcept {
  ESP_LOGI(TAG, "Testing GPIO pins...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  uint32_t gpio_pins = 0;
  if (!handle->driver->diagnostics.ReadGpioPins(gpio_pins)) {
    ESP_LOGW(TAG, "Failed to read GPIO pins (may not be mapped)");
    return true; // Not a failure if pins aren't mapped
  }
  
  ESP_LOGI(TAG, "GPIO Pins: 0x%08lX", gpio_pins);
  
  return true;
}

bool test_factory_otp_config() noexcept {
  ESP_LOGI(TAG, "Testing factory and OTP configuration...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  uint8_t factory_cfg = 0;
  if (!handle->driver->diagnostics.ReadFactoryConfig(factory_cfg)) {
    ESP_LOGW(TAG, "Failed to read factory config");
  }
  
  uint8_t otp_fclktrim = 0;
  bool otp_s2_level = false, otp_bbm = false, otp_tbl = false;
  if (!handle->driver->diagnostics.ReadOtpConfig(otp_fclktrim, otp_s2_level, otp_bbm, otp_tbl)) {
    ESP_LOGW(TAG, "Failed to read OTP config");
  }
  
  ESP_LOGI(TAG, "OTP: FCLKTRIM=%u, S2=%s, BBM=%s, TBL=%s", 
           otp_fclktrim, otp_s2_level ? "true" : "false",
           otp_bbm ? "true" : "false", otp_tbl ? "true" : "false");
  
  return true;
}

bool test_uart_transmission_count() noexcept {
  ESP_LOGI(TAG, "Testing UART transmission count...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  uint8_t uart_count = handle->driver->diagnostics.GetUartTransmissionCount();
  ESP_LOGI(TAG, "UART Transmission Count: %u", uart_count);
  
  return true;
}

bool test_offset_calibration() noexcept {
  ESP_LOGI(TAG, "Testing offset calibration...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  uint8_t offset_a = 0, offset_b = 0;
  if (!handle->driver->diagnostics.ReadOffsetCalibration(offset_a, offset_b)) {
    ESP_LOGW(TAG, "Failed to read offset calibration");
  }
  
  ESP_LOGI(TAG, "Offset A: %u, Offset B: %u", offset_a, offset_b);
  
  return true;
}

bool test_sensorless_homing() noexcept {
  ESP_LOGI(TAG, "Testing sensorless homing...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Configure StallGuard2 for homing using default test config
  tmc5160::StallGuardConfig sg_config{};
  sg_config.threshold = Test::StallGuard::SGT_HOMING;
  sg_config.enable_filter = Test::StallGuard::FILTER_ENABLED;
  // Note: semin/semax are CoolStep parameters, not StallGuard2 parameters
  
  if (!handle->driver->diagnostics.ConfigureStallGuard(sg_config)) {
    ESP_LOGE(TAG, "Failed to configure StallGuard2 for homing");
    return false;
  }
  
  // Note: PerformSensorlessHoming requires motor movement and may not complete
  // in a test environment, so we just verify the configuration
  ESP_LOGI(TAG, "StallGuard2 configured for sensorless homing (threshold=%d)", sg_config.threshold);
  
  return true;
}

bool test_open_load() noexcept {
  ESP_LOGI(TAG, "Testing open load detection...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Ensure SpreadCycle mode (StealthChop disabled) for open load detection
  tmc5160::GlobalConfig gconf{};
  if (!handle->driver->motorControl.GetGlobalConfig(gconf)) {
    ESP_LOGE(TAG, "Failed to get global config");
    return false;
  }
  
  if (gconf.en_pwm_mode) {
    ESP_LOGI(TAG, "Disabling StealthChop for open load detection test");
    gconf.en_pwm_mode = false;
    if (!handle->driver->motorControl.ConfigureGlobalConfig(gconf)) {
      ESP_LOGE(TAG, "Failed to disable StealthChop");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  // Move motor at low velocity (minimum 4× microstep resolution = 1024 steps)
  ESP_LOGI(TAG, "Moving motor for open load detection test");
  handle->driver->rampControl.SetMaxSpeed(500.0f, tmc5160::Unit::Steps);
  handle->driver->rampControl.SetTargetPosition(1024.0f, tmc5160::Unit::Steps);  // 4× microstep resolution
  
  // Check for open load during motion
  bool open_load_detected = false;
  uint32_t check_count = 0;
  while (!handle->driver->rampControl.IsTargetReached() && check_count < 50) {
    bool phase_a = handle->driver->diagnostics.IsOpenLoadA();
    bool phase_b = handle->driver->diagnostics.IsOpenLoadB();
    
    if (phase_a || phase_b) {
      ESP_LOGW(TAG, "Open load detected: Phase A=%d, Phase B=%d", phase_a, phase_b);
      open_load_detected = true;
    }
    
    check_count++;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  
  // Wait for motion to complete
  while (!handle->driver->rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  // Check both phases at once
  bool phase_a_final, phase_b_final;
  if (handle->driver->diagnostics.CheckOpenLoad(phase_a_final, phase_b_final)) {
    ESP_LOGI(TAG, "Final open load check: Phase A=%d, Phase B=%d", 
             phase_a_final, phase_b_final);
    if (phase_a_final || phase_b_final) {
      ESP_LOGW(TAG, "⚠️ Open load flags are set (check wiring if unexpected)");
      ESP_LOGW(TAG, "Note: Flags may also indicate undervoltage, high velocity, or other conditions");
    } else {
      ESP_LOGI(TAG, "✓ No open load detected");
    }
  } else {
    ESP_LOGE(TAG, "Failed to check open load status");
    return false;
  }
  
  if (!open_load_detected && !phase_a_final && !phase_b_final) {
    ESP_LOGI(TAG, "✓ Open load detection test passed (no open load detected)");
  }
  
  return true;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║            ESP32 TMC5160 DIAGNOSTICS COMPREHENSIVE TEST SUITE                 ║");
  ESP_LOGI(TAG, "║                         HardFOC TMC5160 Driver Tests                         ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  print_test_section_status(TAG, "Diagnostics");
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_DRIVER_STATUS_TESTS, "DRIVER STATUS TESTS", 5,
    ESP_LOGI(TAG, "Running driver status tests...");
    RUN_TEST_IN_TASK("driver_status", test_driver_status, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_STALLGUARD_TESTS, "STALLGUARD TESTS", 5,
    ESP_LOGI(TAG, "Running StallGuard tests...");
    RUN_TEST_IN_TASK("stallguard", test_stallguard, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_LOST_STEPS_TESTS, "LOST STEPS TESTS", 5,
    ESP_LOGI(TAG, "Running lost steps tests...");
    RUN_TEST_IN_TASK("lost_steps", test_lost_steps, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_PHASE_CURRENT_TESTS, "PHASE CURRENT TESTS", 5,
    ESP_LOGI(TAG, "Running phase current tests...");
    RUN_TEST_IN_TASK("phase_currents", test_phase_currents, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_PWM_SCALE_TESTS, "PWM SCALE TESTS", 5,
    ESP_LOGI(TAG, "Running PWM scale tests...");
    RUN_TEST_IN_TASK("pwm_scale", test_pwm_scale, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_MICROSTEP_DIAGNOSTICS_TESTS, "MICROSTEP DIAGNOSTICS TESTS", 5,
    ESP_LOGI(TAG, "Running microstep diagnostics tests...");
    RUN_TEST_IN_TASK("microstep_diagnostics", test_microstep_diagnostics, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_GPIO_TESTS, "GPIO TESTS", 5,
    ESP_LOGI(TAG, "Running GPIO tests...");
    RUN_TEST_IN_TASK("gpio_pins", test_gpio_pins, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_FACTORY_OTP_TESTS, "FACTORY/OTP TESTS", 5,
    ESP_LOGI(TAG, "Running factory/OTP tests...");
    RUN_TEST_IN_TASK("factory_otp_config", test_factory_otp_config, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_UART_COUNT_TESTS, "UART COUNT TESTS", 5,
    ESP_LOGI(TAG, "Running UART count tests...");
    RUN_TEST_IN_TASK("uart_transmission_count", test_uart_transmission_count, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_OFFSET_CALIBRATION_TESTS, "OFFSET CALIBRATION TESTS", 5,
    ESP_LOGI(TAG, "Running offset calibration tests...");
    RUN_TEST_IN_TASK("offset_calibration", test_offset_calibration, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_SENSORLESS_HOMING_TESTS, "SENSORLESS HOMING TESTS", 5,
    ESP_LOGI(TAG, "Running sensorless homing tests...");
    RUN_TEST_IN_TASK("sensorless_homing", test_sensorless_homing, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_OPEN_LOAD_TESTS, "OPEN LOAD DETECTION TESTS", 5,
    ESP_LOGI(TAG, "Running open load detection tests...");
    RUN_TEST_IN_TASK("open_load", test_open_load, 8192, 1);
    flip_test_progress_indicator();
  );
  
  print_test_summary(g_test_results, "Diagnostics", TAG);
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

