/**
 * @file core_comprehensive_test.cpp
 * @brief Comprehensive Core initialization and basic setup testing suite for TMC5160 (single motor)
 *
 * This file contains comprehensive testing for TMC5160 core initialization and basic setup:
 * - Driver initialization and configuration
 * - Basic driver instance creation
 * - Configuration validation
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

static const char* TAG = "Core_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
// Enable/disable specific test categories by setting to true or false

// Core initialization tests
static constexpr bool ENABLE_INITIALIZATION_TESTS = true;

// Test configuration constants
static constexpr uint8_t TEST_IRUN = 20;
static constexpr uint8_t TEST_IHOLD = 10;
static constexpr uint8_t TEST_GLOBAL_SCALER = 32;
static constexpr uint8_t TEST_TOFF = 5;
static constexpr uint8_t TEST_MRES = 4; // 16 microsteps

// Forward declarations
bool test_driver_initialization() noexcept;

// Helper functions
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;
  std::unique_ptr<tmc5160::TMC5160<Esp32SPI>> driver;
};

std::unique_ptr<TestDriverHandle> create_test_driver() noexcept {
  auto handle = std::make_unique<TestDriverHandle>();
  
  // Create SPI communication interface
  handle->spi = std::make_unique<Esp32SPI>(
    SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
    GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15, 4000000);
  
  // Initialize SPI interface
  if (!handle->spi->Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return nullptr;
  }
  
  // Create TMC5160 driver instance
  handle->driver = std::make_unique<tmc5160::TMC5160<Esp32SPI>>(*handle->spi);
  
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
  cfg.chopper.mres = TEST_MRES; // 16 microsteps
  cfg.chopper.intpol = true;
  cfg.power_stage.drv_strength = 0;
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
  
  print_test_summary(g_test_results, "Core", TAG);
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

