/**
 * @file spi_daisy_chain_comprehensive_test.cpp
 * @brief Comprehensive SPI Daisy Chain testing suite for TMC51x0 (MULTI-MOTOR)
 *
 * ⚠️ MULTI-MOTOR HARDWARE REQUIRED ⚠️
 * This test suite requires multiple TMC51x0 drivers connected in a SPI daisy chain.
 * DO NOT run these tests on a single-motor setup.
 *
 * This file contains comprehensive testing for TMC51x0 SPI daisy chain features:
 * - SPI daisy chain setup and configuration
 * - Daisy-chain position management
 * - Multi-motor coordination
 * - Chain length configuration
 * - Sequential positioning
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - 2+ TMC51x0 stepper motor drivers (daisy-chained via SPI)
 * - Stepper motors connected to each TMC51x0
 * - SPI connection: All chips share CSN, SCK, MOSI; MISO daisy-chained
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5 (shared by all chips)
 * - Control: EN=2, DIR=4, STEP=15 (shared or separate per chip)
 *
 * Daisy-Chain Wiring:
 * - MCU MISO ──> Chip 1 SDO ──> Chip 2 SDI
 *                Chip 1 SDI <── MCU MOSI
 *                Chip 2 SDO ──> MCU MISO (if 2 chips) or Chip 3 SDI (if 3 chips)
 * - All chips: CSN, SCK, MOSI (SDI) tied together
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include "test_config/TestFramework.h"
#include <memory>
#include <vector>

static const char* TAG = "SPI_DaisyChain_Test";
static TestResults g_test_results;

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// CONFIGURATION SELECTION - Unified Test Rig Selection
//=============================================================================
// Test rig selection (compile-time constant) - automatically selects motor, board, and platform
// CORE DRIVER TEST RIG: Uses 17HS4401S gearbox motor, TMC51x0 EVAL board, reference switches, encoder
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_DAISY_CHAIN_SETUP_TESTS = true;
static constexpr bool ENABLE_POSITION_MANAGEMENT_TESTS = true;
static constexpr bool ENABLE_MULTI_MOTOR_COORDINATION_TESTS = true;

// Test configuration constants
static constexpr uint8_t TEST_CHAIN_LENGTH = 2; // Number of devices in chain
static constexpr uint8_t TEST_IRUN = 20;
static constexpr uint8_t TEST_IHOLD = 10;
static constexpr uint8_t TEST_GLOBAL_SCALER = 32;
static constexpr uint8_t TEST_TOFF = 5;
static constexpr tmc51x0::MicrostepResolution TEST_MRES = tmc51x0::MicrostepResolution::MRES_256; // 256 microsteps

// Forward declarations
bool test_daisy_chain_setup() noexcept;
bool test_daisy_chain_position_management() noexcept;
bool test_multi_motor_coordination() noexcept;

// Helper functions
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;
  std::vector<std::unique_ptr<tmc51x0::TMC51x0<Esp32SPI>>> drivers;
};

std::unique_ptr<TestDriverHandle> create_daisy_chain_drivers() noexcept {
  auto handle = std::make_unique<TestDriverHandle>();
  
  // Create shared SPI communication interface
  // Get complete pin configuration from test config
  tmc51x0::Esp32SpiPinConfig pin_config = tmc51x0_test_config::GetDefaultPinConfig();
  
  handle->spi = std::make_unique<Esp32SPI>(
    tmc51x0_test_config::SPI_HOST,
    pin_config,
    tmc51x0_test_config::SPI_CLOCK_SPEED_HZ);
  
  if (!handle->spi->Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return nullptr;
  }
  
  // Set chain length
  handle->spi->SetDaisyChainLength(TEST_CHAIN_LENGTH);
  
  // Create driver instances for each position in chain
  for (uint8_t pos = 0; pos < TEST_CHAIN_LENGTH; ++pos) {
    handle->drivers.push_back(
      std::make_unique<tmc51x0::TMC51x0<Esp32SPI>>(*handle->spi, 12'000'000, pos));
  }
  
  // Verify mode pins match expected communication mode (if pins are configured)
  // Only check the first driver (all drivers share the same SPI interface)
  if (!handle->drivers.empty()) {
    tmc51x0::ChipCommMode actual_mode;
    if (handle->drivers[0]->communication.GetOperatingMode(actual_mode)) {
      gpio_num_t spi_mode_gpio = handle->spi->GetPinMapping(tmc51x0::TMC51x0CtrlPin::SPI_MODE);
      gpio_num_t sd_mode_gpio = handle->spi->GetPinMapping(tmc51x0::TMC51x0CtrlPin::SD_MODE);
      constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
      
      if (spi_mode_gpio != UNMAPPED_PIN && sd_mode_gpio != UNMAPPED_PIN) {
        if (actual_mode == tmc51x0::ChipCommMode::SPI_INTERNAL_RAMP ||
            actual_mode == tmc51x0::ChipCommMode::SPI_EXTERNAL_STEPDIR) {
          ESP_LOGI(TAG, "✓ Mode pin verification passed for daisy chain (SPI mode)");
        } else {
          ESP_LOGE(TAG, "✗ Mode pin verification FAILED for daisy chain: Mode pins indicate non-SPI mode");
        }
      }
    }
  }
  
  return handle;
}

bool test_daisy_chain_setup() noexcept {
  ESP_LOGI(TAG, "Testing SPI daisy chain setup...");
  
  auto handle = create_daisy_chain_drivers();
  if (!handle) {
    ESP_LOGE(TAG, "Failed to create daisy chain drivers");
    return false;
  }
  
  // Configure and initialize each driver using helper functions
  tmc51x0::DriverConfig cfg{};
  
  // Configure driver from unified test rig selection
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
  
  cfg.chopper.mres = TEST_MRES;
  
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    if (!handle->drivers[i]->Initialize(cfg)) {
      ESP_LOGE(TAG, "Failed to initialize driver at position %zu", i);
      return false;
    }
    
    uint8_t pos = handle->drivers[i]->communication.GetDaisyChainPosition();
    if (pos != i) {
      ESP_LOGE(TAG, "Position mismatch: expected %zu, got %u", i, pos);
      return false;
    }
    
    ESP_LOGI(TAG, "Driver at position %zu initialized successfully", i);
  }
  
  return true;
}

bool test_daisy_chain_position_management() noexcept {
  ESP_LOGI(TAG, "Testing daisy chain position management...");
  
  auto handle = create_daisy_chain_drivers();
  if (!handle) {
    return false;
  }
  
  // Test getting and setting daisy chain positions
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    uint8_t pos = handle->drivers[i]->communication.GetDaisyChainPosition();
    if (pos != i) {
      ESP_LOGE(TAG, "Position mismatch: expected %zu, got %u", i, pos);
      return false;
    }
    
    // Test setting position (should remain the same)
    handle->drivers[i]->communication.SetDaisyChainPosition(i);
    pos = handle->drivers[i]->communication.GetDaisyChainPosition();
    if (pos != i) {
      ESP_LOGE(TAG, "Position set failed: expected %zu, got %u", i, pos);
      return false;
    }
    
    ESP_LOGI(TAG, "Driver %zu position: %u", i, pos);
  }
  
  return true;
}

bool test_multi_motor_coordination() noexcept {
  ESP_LOGI(TAG, "Testing multi-motor coordination...");
  
  auto handle = create_daisy_chain_drivers();
  if (!handle) {
    return false;
  }
  
  // Initialize all drivers
  tmc51x0::DriverConfig cfg{};
  // Configure driver from unified test rig selection
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
  
  cfg.chopper.mres = TEST_MRES;
  
  for (auto& driver : handle->drivers) {
    if (!driver->Initialize(cfg)) {
      ESP_LOGE(TAG, "Failed to initialize driver");
      return false;
    }
  }
  
  // Configure ramp control for each motor
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    handle->drivers[i]->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    handle->drivers[i]->rampControl.SetTargetPosition(static_cast<float>(1000 * (i + 1)), tmc51x0::Unit::Steps);
    handle->drivers[i]->rampControl.SetMaxSpeed(1000.0F);
    handle->drivers[i]->rampControl.SetAcceleration(500.0F);
  }
  
  // Enable all motors
  for (auto& driver : handle->drivers) {
    if (!driver->motorControl.Enable()) {
      ESP_LOGE(TAG, "Failed to enable motor");
      return false;
    }
  }
  
  // Monitor all motors
  bool all_reached = false;
  for (int check = 0; check < 50 && !all_reached; ++check) {
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_reached = true;
    for (size_t i = 0; i < handle->drivers.size(); ++i) {
      float pos_float = 0.0f;
      if (!handle->drivers[i]->rampControl.GetCurrentPosition(pos_float, tmc51x0::Unit::Steps)) {
        pos_float = 0.0f;
      }
      int32_t pos = static_cast<int32_t>(pos_float);
      bool reached = handle->drivers[i]->rampControl.IsTargetReached();
      
      if (!reached) {
        all_reached = false;
      }
      
      if (check % 10 == 0) {
        ESP_LOGI(TAG, "Driver %zu: Position=%ld, Target Reached=%s", i, pos, reached ? "YES" : "NO");
      }
    }
  }
  
  // Disable all motors
  for (auto& driver : handle->drivers) {
    driver->motorControl.Disable();
  }
  
  return all_reached;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║         ESP32 TMC51x0 SPI DAISY CHAIN COMPREHENSIVE TEST SUITE               ║");
  ESP_LOGI(TAG, "║                         HardFOC TMC51x0 Driver Tests                         ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  ESP_LOGW(TAG, "⚠️  MULTI-MOTOR HARDWARE REQUIRED - DO NOT RUN ON SINGLE-MOTOR SETUP ⚠️");
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  print_test_section_status(TAG, "SPI Daisy Chain");
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_DAISY_CHAIN_SETUP_TESTS, "DAISY CHAIN SETUP TESTS", 5,
    ESP_LOGI(TAG, "Running daisy chain setup tests...");
    RUN_TEST_IN_TASK("daisy_chain_setup", test_daisy_chain_setup, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_POSITION_MANAGEMENT_TESTS, "POSITION MANAGEMENT TESTS", 5,
    ESP_LOGI(TAG, "Running position management tests...");
    RUN_TEST_IN_TASK("daisy_chain_position_management", test_daisy_chain_position_management, 8192, 1);
    flip_test_progress_indicator();
  );
  
  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_MULTI_MOTOR_COORDINATION_TESTS, "MULTI-MOTOR COORDINATION TESTS", 5,
    ESP_LOGI(TAG, "Running multi-motor coordination tests...");
    RUN_TEST_IN_TASK("multi_motor_coordination", test_multi_motor_coordination, 8192, 1);
    flip_test_progress_indicator();
  );
  
  print_test_summary(g_test_results, "SPI Daisy Chain", TAG);
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

