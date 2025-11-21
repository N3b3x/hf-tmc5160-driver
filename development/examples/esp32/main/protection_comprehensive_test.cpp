/**
 * @file protection_comprehensive_test.cpp
 * @brief Comprehensive Protection testing suite for TMC5160 (single motor)
 *
 * This file contains comprehensive testing for TMC5160 protection features:
 * - Short circuit protection
 * - Overtemperature protection
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
#include "TestFramework.h"
#include <memory>

static const char* TAG = "Protection_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_SHORT_CIRCUIT_TESTS = true;
static constexpr bool ENABLE_OVERTEMPERATURE_TESTS = true;

// Test configuration constants
static constexpr uint8_t TEST_IRUN = 20;
static constexpr uint8_t TEST_IHOLD = 10;
static constexpr uint8_t TEST_GLOBAL_SCALER = 32;
static constexpr uint8_t TEST_TOFF = 5;
static constexpr uint8_t TEST_MRES = 4; // 16 microsteps

// Forward declarations
bool test_short_circuit_protection() noexcept;
bool test_overtemperature_protection() noexcept;

// Helper functions
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;
  std::unique_ptr<tmc5160::TMC5160<Esp32SPI>> driver;
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
  
  handle->driver = std::make_unique<tmc5160::TMC5160<Esp32SPI>>(*handle->spi);
  
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

bool test_short_circuit_protection() noexcept {
  ESP_LOGI(TAG, "Testing short circuit protection...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  tmc5160::ShortProtectionConfig short_cfg{};
  short_cfg.s2vs_level = 6;
  short_cfg.s2g_level = 6;
  short_cfg.shortfilter = 1;
  short_cfg.shortdelay = false;
  
  if (!handle->driver->protection.ConfigureShortProtection(short_cfg)) {
    ESP_LOGE(TAG, "Failed to configure short protection");
    return false;
  }
  
  return true;
}

bool test_overtemperature_protection() noexcept {
  ESP_LOGI(TAG, "Testing overtemperature protection...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Overtemperature status is read via diagnostics.GetStatus()
  tmc5160::DriverStatus prot_status = handle->driver->diagnostics.GetStatus();
  bool has_otpw = (prot_status == tmc5160::DriverStatus::OTPW);
  bool has_ot = (prot_status == tmc5160::DriverStatus::OT);
  ESP_LOGI(TAG, "OTPW: %s, OT: %s", has_otpw ? "true" : "false", has_ot ? "true" : "false");
  
  return true;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║            ESP32 TMC5160 PROTECTION COMPREHENSIVE TEST SUITE                    ║");
  ESP_LOGI(TAG, "║                         HardFOC TMC5160 Driver Tests                          ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  print_test_section_status(TAG, "Protection");
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_SHORT_CIRCUIT_TESTS, "SHORT CIRCUIT PROTECTION TESTS", 5,
    ESP_LOGI(TAG, "Running short circuit protection tests...");
    RUN_TEST_IN_TASK("short_circuit_protection", test_short_circuit_protection, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_OVERTEMPERATURE_TESTS, "OVERTEMPERATURE PROTECTION TESTS", 5,
    ESP_LOGI(TAG, "Running overtemperature protection tests...");
    RUN_TEST_IN_TASK("overtemperature_protection", test_overtemperature_protection, 8192, 1);
    flip_test_progress_indicator();
  );
  
  print_test_summary(g_test_results, "Protection", TAG);
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

