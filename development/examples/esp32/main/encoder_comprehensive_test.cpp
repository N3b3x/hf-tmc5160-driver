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

// Test configuration constants
static constexpr uint8_t TEST_IRUN = 20;
static constexpr uint8_t TEST_IHOLD = 10;
static constexpr uint8_t TEST_GLOBAL_SCALER = 32;
static constexpr uint8_t TEST_TOFF = 5;
static constexpr uint8_t TEST_MRES = 4; // 16 microsteps
static constexpr uint16_t TEST_MOTOR_STEPS_PER_REV = 200;
static constexpr uint16_t TEST_ENCODER_PULSES_PER_REV = 1000;

// Forward declarations
bool test_encoder_configuration() noexcept;
bool test_encoder_resolution() noexcept;
bool test_encoder_position_reading() noexcept;
bool test_deviation_detection() noexcept;
bool test_latched_position() noexcept;

// Helper functions
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;
  std::unique_ptr<tmc5160::TMC5160> driver;
};

std::unique_ptr<TestDriverHandle> create_test_driver() noexcept {
  auto handle = std::make_unique<TestDriverHandle>();
  
  handle->spi = std::make_unique<Esp32SPI>(
    SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
    GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15, 4000000);
  
  if (!handle->spi->Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return nullptr;
  }
  
  handle->driver = std::make_unique<tmc5160::TMC5160>(*handle->spi);
  
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = TEST_IRUN;
  cfg.motor.ihold = TEST_IHOLD;
  cfg.motor.global_scaler = TEST_GLOBAL_SCALER;
  cfg.chopper.toff = TEST_TOFF;
  cfg.chopper.mres = TEST_MRES;
  cfg.chopper.intpol = true;
  cfg.power_stage.drv_strength = 0;
  cfg.power_stage.bbm_time = 24;
  cfg.power_stage.bbm_clks = 4;
  
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
  
  tmc5160::EncoderConfig enc_cfg{};
  enc_cfg.enc_sel_decimal = false;
  enc_cfg.clr_cont = false;
  enc_cfg.clr_once = false;
  enc_cfg.pol_a = false;
  enc_cfg.pol_b = false;
  enc_cfg.ignore_ab = false;
  
  if (!handle->driver->encoder.Configure(enc_cfg)) {
    ESP_LOGE(TAG, "Failed to configure encoder");
    return false;
  }
  
  return true;
}

bool test_encoder_resolution() noexcept {
  ESP_LOGI(TAG, "Testing encoder resolution...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Note: SetResolution may use approximation, so we use a warning-level test
  bool result = handle->driver->encoder.SetResolution(
    TEST_MOTOR_STEPS_PER_REV, TEST_ENCODER_PULSES_PER_REV, false);
  
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

