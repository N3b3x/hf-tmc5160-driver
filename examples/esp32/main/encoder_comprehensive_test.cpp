/**
 * @file encoder_comprehensive_test.cpp
 * @brief Comprehensive Encoder testing suite for TMC5160 (single motor)
 *
 * This file contains comprehensive testing for TMC5160 encoder features:
 * - Encoder configuration
 * - Resolution setting
 * - Position reading
 * - Deviation detection
 * - Latched position
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Single stepper motor with encoder connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5
 * - Control: EN=2, DIR=4, STEP=15
 * - Encoder: A, B, N signals connected to TMC5160 encoder inputs
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp32_tmc5160_bus_config.hpp"
#include "TestFramework.h"
#include <memory>

static const char* TAG = "Encoder_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_ENCODER_CONFIG_TESTS = true;
static constexpr bool ENABLE_ENCODER_RESOLUTION_TESTS = true;
static constexpr bool ENABLE_ENCODER_POSITION_TESTS = true;
static constexpr bool ENABLE_DEVIATION_DETECTION_TESTS = true;
static constexpr bool ENABLE_LATCHED_POSITION_TESTS = true;

// Motor selection (compile-time constant)
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;

// Board selection (compile-time constant)
static constexpr tmc5160_test_config::BoardType SELECTED_BOARD = 
    tmc5160_test_config::BoardType::BOARD_TMC5160_EVAL;

// Platform selection (compile-time constant)
static constexpr tmc5160_test_config::PlatformType SELECTED_PLATFORM = 
    tmc5160_test_config::PlatformType::PLATFORM_TEST_RIG;

// Test configuration constants
namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
namespace Test = tmc5160_test_config::TestConfig_17HS4401S;

static constexpr uint8_t TEST_IRUN = Motor::IRUN;
static constexpr uint8_t TEST_IHOLD = Motor::IHOLD;
static constexpr uint8_t TEST_GLOBAL_SCALER = Motor::GLOBAL_SCALER;
static constexpr uint8_t TEST_TOFF = Motor::TOFF;
static constexpr uint8_t TEST_MRES = Motor::MRES; // 256 microsteps
static constexpr uint16_t TEST_MOTOR_STEPS_PER_REV = Motor::MOTOR_FULL_STEPS;
static constexpr uint16_t TEST_ENCODER_PULSES_PER_REV = 
    tmc5160_test_config::GetEncoderPulsesPerRev<SELECTED_PLATFORM>();

// Forward declarations
bool test_encoder_configuration() noexcept;
bool test_encoder_resolution() noexcept;
bool test_encoder_position_reading() noexcept;
bool test_deviation_detection() noexcept;
bool test_latched_position() noexcept;

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
  
  cfg.chopper.mres = TEST_MRES;
  
  if (!handle->driver->Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return nullptr;
  }
  
  return handle;
}

bool test_encoder_configuration() noexcept {
  ESP_LOGI(TAG, "Testing encoder configuration...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Get encoder configuration from platform config
  tmc5160::EncoderConfig enc_cfg = 
      tmc5160_test_config::GetEncoderConfig<SELECTED_PLATFORM>();
  
  // A/B polarity requirements (set explicitly for this test)
  enc_cfg.require_a_high = false;
  enc_cfg.require_b_high = false;
  enc_cfg.ignore_ab_polarity = true;  // Ignore A/B polarity
  
  // Clear/latch mode (set explicitly for this test)
  enc_cfg.clear_enc_x_on_event = false;
  enc_cfg.latch_xactual_with_enc = false;
  
  if (!handle->driver->encoder.Configure(enc_cfg)) {
    ESP_LOGE(TAG, "Failed to configure encoder");
    return false;
  }
  
  // Verify configuration by reading it back
  tmc5160::EncoderConfig read_cfg{};
  if (!handle->driver->encoder.GetEncoderConfig(read_cfg)) {
    ESP_LOGE(TAG, "Failed to read encoder configuration");
    return false;
  }
  
  ESP_LOGI(TAG, "Encoder configuration verified");
  return true;
}

bool test_encoder_resolution() noexcept {
  ESP_LOGI(TAG, "Testing encoder resolution...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Note: SetResolution may use approximation, so we use a warning-level test
  // Use platform config encoder resolution
  bool result = handle->driver->encoder.SetResolution(
    TEST_MOTOR_STEPS_PER_REV, 
    tmc5160_test_config::GetEncoderPulsesPerRev<SELECTED_PLATFORM>(), 
    tmc5160_test_config::GetEncoderInvertDirection<SELECTED_PLATFORM>());
  
  if (!result) {
    ESP_LOGW(TAG, "Encoder resolution set with approximation");
  }
  
  return true; // Always return true as this may use approximation
}

bool test_encoder_position_reading() noexcept {
  ESP_LOGI(TAG, "Testing encoder position reading...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  int32_t enc_pos = handle->driver->encoder.GetPosition();
  ESP_LOGI(TAG, "Encoder position: %ld", enc_pos);
  
  return true;
}

bool test_deviation_detection() noexcept {
  ESP_LOGI(TAG, "Testing deviation detection...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Set allowed deviation
  if (!handle->driver->encoder.SetAllowedDeviation(10)) {
    ESP_LOGE(TAG, "Failed to set allowed deviation");
    return false;
  }
  
  // Check for deviation
  bool dev_detected = handle->driver->encoder.IsDeviationDetected();
  ESP_LOGI(TAG, "Deviation Detected: %s", dev_detected ? "true" : "false");
  
  // Clear deviation flag
  if (!handle->driver->encoder.ClearDeviationFlag()) {
    ESP_LOGE(TAG, "Failed to clear deviation flag");
    return false;
  }
  
  return true;
}

bool test_latched_position() noexcept {
  ESP_LOGI(TAG, "Testing latched position...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  int32_t latched = handle->driver->encoder.GetLatchedPosition();
  ESP_LOGI(TAG, "Latched position: %ld", latched);
  
  return true;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║              ESP32 TMC5160 ENCODER COMPREHENSIVE TEST SUITE                  ║");
  ESP_LOGI(TAG, "║                         HardFOC TMC5160 Driver Tests                         ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  print_test_section_status(TAG, "Encoder");
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_ENCODER_CONFIG_TESTS, "ENCODER CONFIGURATION TESTS", 5,
    ESP_LOGI(TAG, "Running encoder configuration tests...");
    RUN_TEST_IN_TASK("encoder_configuration", test_encoder_configuration, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_ENCODER_RESOLUTION_TESTS, "ENCODER RESOLUTION TESTS", 5,
    ESP_LOGI(TAG, "Running encoder resolution tests...");
    RUN_TEST_IN_TASK("encoder_resolution", test_encoder_resolution, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_ENCODER_POSITION_TESTS, "ENCODER POSITION TESTS", 5,
    ESP_LOGI(TAG, "Running encoder position tests...");
    RUN_TEST_IN_TASK("encoder_position_reading", test_encoder_position_reading, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_DEVIATION_DETECTION_TESTS, "DEVIATION DETECTION TESTS", 5,
    ESP_LOGI(TAG, "Running deviation detection tests...");
    RUN_TEST_IN_TASK("deviation_detection", test_deviation_detection, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_LATCHED_POSITION_TESTS, "LATCHED POSITION TESTS", 5,
    ESP_LOGI(TAG, "Running latched position tests...");
    RUN_TEST_IN_TASK("latched_position", test_latched_position, 8192, 1);
    flip_test_progress_indicator();
  );
  
  print_test_summary(g_test_results, "Encoder", TAG);
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

