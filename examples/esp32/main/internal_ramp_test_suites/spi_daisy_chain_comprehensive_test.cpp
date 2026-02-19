/**
 * @file spi_daisy_chain_comprehensive_test.cpp
 * @brief Comprehensive SPI Daisy Chain testing suite for TMC51x0
 *
 * Supports 1 or more TMC51x0 drivers. Set TEST_CHAIN_LENGTH to match your
 * hardware (default: 2). With TEST_CHAIN_LENGTH=1, standard single-driver
 * SPI operation is tested (position 0, chain length 1).
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
/**
 * @brief Test SPI daisy chain setup and initialization.
 * @return true if test passed, false otherwise
 */
bool test_daisy_chain_setup() noexcept;

/**
 * @brief Test daisy chain position management and addressing.
 * @return true if test passed, false otherwise
 */
bool test_daisy_chain_position_management() noexcept;

/**
 * @brief Test multi-motor coordination in daisy chain configuration.
 * @return true if test passed, false otherwise
 */
bool test_multi_motor_coordination() noexcept;

// Helper functions
/**
 * @brief Test driver handle for daisy chain tests.
 * 
 * @details
 * Contains the shared SPI interface and a vector of driver instances,
 * one for each position in the daisy chain.
 */
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;  ///< Shared SPI communication interface
  std::vector<std::unique_ptr<tmc51x0::TMC51x0<Esp32SPI>>> drivers;  ///< Driver instances for each chain position
};

/**
 * @brief Create and initialize daisy chain driver instances.
 * 
 * @details
 * Creates a shared SPI interface and multiple driver instances configured
 * for daisy chain operation. Each driver is assigned a unique chain position.
 * 
 * @return Unique pointer to TestDriverHandle, or nullptr on failure
 */
std::unique_ptr<TestDriverHandle> create_daisy_chain_drivers() noexcept {
  auto handle = std::make_unique<TestDriverHandle>();
  
  // Create shared SPI communication interface
  // Get complete pin configuration from test config
  Esp32SpiPinConfig pin_config = tmc51x0_test_config::GetDefaultPinConfig();
  
  handle->spi = std::make_unique<Esp32SPI>(
    tmc51x0_test_config::SPI_HOST,
    pin_config,
    tmc51x0_test_config::SPI_CLOCK_SPEED_HZ);
  
  auto spi_init_result = handle->spi->Initialize();
  if (!spi_init_result) {
    ESP_LOGE(TAG, "❌ Failed to initialize SPI interface (ErrorCode: %d)", static_cast<int>(spi_init_result.Error()));
    ESP_LOGE(TAG, "   Check: SPI pin configuration, clock speed, and hardware connections");
    return nullptr;
  }
  ESP_LOGI(TAG, "✓ SPI interface initialized successfully");
  
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
    auto mode_result = handle->drivers[0]->io.GetOperatingMode();
    if (mode_result.IsOk()) {
      tmc51x0::ChipCommMode actual_mode = mode_result.Value();
      gpio_num_t spi_mode_gpio = handle->spi->GetPinMapping(tmc51x0::TMC51x0CtrlPin::SPI_MODE);
      gpio_num_t sd_mode_gpio = handle->spi->GetPinMapping(tmc51x0::TMC51x0CtrlPin::SD_MODE);
      constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
      
      if (spi_mode_gpio != UNMAPPED_PIN && sd_mode_gpio != UNMAPPED_PIN) {
        if (actual_mode == tmc51x0::ChipCommMode::SPI_INTERNAL_RAMP ||
            actual_mode == tmc51x0::ChipCommMode::SPI_EXTERNAL_STEPDIR) {
          ESP_LOGI(TAG, "✓ Mode pin verification passed for daisy chain (SPI mode: %s)", 
                   (actual_mode == tmc51x0::ChipCommMode::SPI_INTERNAL_RAMP) ? "INTERNAL_RAMP" : "EXTERNAL_STEPDIR");
        } else {
          ESP_LOGE(TAG, "✗ Mode pin verification FAILED for daisy chain: Mode pins indicate non-SPI mode");
          ESP_LOGE(TAG, "   Expected: SPI_INTERNAL_RAMP or SPI_EXTERNAL_STEPDIR");
          ESP_LOGE(TAG, "   Actual: %d", static_cast<int>(actual_mode));
        }
      } else {
        ESP_LOGI(TAG, "ℹ Mode pins not configured, skipping mode verification");
      }
    } else {
      ESP_LOGW(TAG, "⚠ Could not read operating mode (ErrorCode: %d)", static_cast<int>(mode_result.Error()));
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
    ESP_LOGI(TAG, "Initializing driver at position %zu...", i);
    auto init_result = handle->drivers[i]->Initialize(cfg);
    if (init_result.IsErr()) {
      ESP_LOGE(TAG, "❌ Failed to initialize driver at position %zu", i);
      ESP_LOGE(TAG, "   ErrorCode: %d", static_cast<int>(init_result.Error()));
      ESP_LOGE(TAG, "   Check: Motor configuration, SPI connection, and power supply");
      return false;
    }
    
    uint8_t pos = handle->drivers[i]->communication.GetDaisyChainPosition();
    if (pos != i) {
      ESP_LOGE(TAG, "❌ Position mismatch for driver %zu: expected %zu, got %u", i, i, pos);
      ESP_LOGE(TAG, "   Check: Daisy chain wiring and position configuration");
      return false;
    }
    
    ESP_LOGI(TAG, "✓ Driver at position %zu initialized successfully (position verified: %u)", i, pos);
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
  
  ESP_LOGI(TAG, "Initializing %zu drivers for coordination test...", handle->drivers.size());
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    auto init_result = handle->drivers[i]->Initialize(cfg);
    if (init_result.IsErr()) {
      ESP_LOGE(TAG, "❌ Failed to initialize driver %zu", i);
      ESP_LOGE(TAG, "   ErrorCode: %d", static_cast<int>(init_result.Error()));
      return false;
    }
    ESP_LOGI(TAG, "✓ Driver %zu initialized", i);
  }
  
  // Configure ramp control for each motor
  ESP_LOGI(TAG, "Configuring ramp control for %zu motors...", handle->drivers.size());
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    float target_pos = static_cast<float>(1000 * (i + 1));
    auto ramp_mode_result = handle->drivers[i]->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    if (ramp_mode_result.IsErr()) {
      ESP_LOGE(TAG, "❌ Failed to set ramp mode for driver %zu (ErrorCode: %d)", i, static_cast<int>(ramp_mode_result.Error()));
      return false;
    }
    
    // Convert target position from steps to degrees (assuming 200 steps/rev motor)
    float target_pos_deg = target_pos / 200.0f * 360.0f;
    auto target_result = handle->drivers[i]->rampControl.SetTargetPosition(target_pos_deg, tmc51x0::Unit::Deg);
    if (target_result.IsErr()) {
      ESP_LOGE(TAG, "❌ Failed to set target position for driver %zu (ErrorCode: %d)", i, static_cast<int>(target_result.Error()));
      return false;
    }
    
    // Use physical units for speed and acceleration
    float speed_rpm = 30.0f; // ~1000 steps/s for 200 steps/rev
    auto speed_result = handle->drivers[i]->rampControl.SetMaxSpeed(speed_rpm, tmc51x0::Unit::RPM);
    if (speed_result.IsErr()) {
      ESP_LOGE(TAG, "❌ Failed to set max speed for driver %zu (ErrorCode: %d)", i, static_cast<int>(speed_result.Error()));
      return false;
    }
    
    float accel_rev_s2 = 15.0f; // ~500 steps/s² for 200 steps/rev
    auto accel_result = handle->drivers[i]->rampControl.SetAcceleration(accel_rev_s2, tmc51x0::Unit::RevPerSec);
    if (accel_result.IsErr()) {
      ESP_LOGE(TAG, "❌ Failed to set acceleration for driver %zu (ErrorCode: %d)", i, static_cast<int>(accel_result.Error()));
      return false;
    }
    
    ESP_LOGI(TAG, "✓ Driver %zu configured: target=%.2f degrees, max_speed=%.1f RPM, accel=%.1f rev/s²", 
             i, target_pos_deg, speed_rpm, accel_rev_s2);
  }
  
  // Enable all motors
  ESP_LOGI(TAG, "Enabling %zu motors...", handle->drivers.size());
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    auto enable_result = handle->drivers[i]->motorControl.Enable();
    if (enable_result.IsErr()) {
      ESP_LOGE(TAG, "❌ Failed to enable motor %zu", i);
      ESP_LOGE(TAG, "   ErrorCode: %d", static_cast<int>(enable_result.Error()));
      ESP_LOGE(TAG, "   Check: Enable pin connection and power supply");
      return false;
    }
    ESP_LOGI(TAG, "✓ Motor %zu enabled", i);
  }
  
  // Monitor all motors
  ESP_LOGI(TAG, "Monitoring %zu motors for target position...", handle->drivers.size());
  bool all_reached = false;
  int check_count = 0;
  constexpr int MAX_CHECKS = 50;
  constexpr int LOG_INTERVAL = 10;
  
  for (int check = 0; check < MAX_CHECKS && !all_reached; ++check) {
    vTaskDelay(pdMS_TO_TICKS(100));
    check_count = check + 1;
    
    all_reached = true;
    for (size_t i = 0; i < handle->drivers.size(); ++i) {
      // Get current position in degrees
      auto pos_result = handle->drivers[i]->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
      float pos_deg = 0.0f;
      if (pos_result.IsErr()) {
        ESP_LOGW(TAG, "⚠ Driver %zu: Failed to read position (ErrorCode: %d)", i, static_cast<int>(pos_result.Error()));
        all_reached = false;
        continue;
      }
      pos_deg = pos_result.Value();
      // Note: pos_deg is already in degrees, no conversion needed
      
      // Check if target reached
      auto reached_result = handle->drivers[i]->rampControl.IsTargetReached();
      bool reached = false;
      if (reached_result.IsErr()) {
        ESP_LOGW(TAG, "⚠ Driver %zu: Failed to check target status (ErrorCode: %d)", i, static_cast<int>(reached_result.Error()));
        all_reached = false;
        continue;
      }
      reached = reached_result.Value();
      
      if (!reached) {
        all_reached = false;
      }
      
      if (check % LOG_INTERVAL == 0) {
        ESP_LOGI(TAG, "  Driver %zu: Position=%.2f deg, Target Reached=%s", i, pos_deg, reached ? "YES" : "NO");
      }
    }
    
    if (all_reached) {
      ESP_LOGI(TAG, "✓ All %zu motors reached target position after %d checks (%.1f seconds)", 
               handle->drivers.size(), check_count, check_count * 0.1f);
      break;
    }
  }
  
  if (!all_reached) {
    ESP_LOGW(TAG, "⚠ Not all motors reached target within %d checks (%.1f seconds)", 
             MAX_CHECKS, MAX_CHECKS * 0.1f);
    ESP_LOGW(TAG, "   This may indicate: motor stall, incorrect target, or insufficient time");
  }
  
  // Disable all motors
  ESP_LOGI(TAG, "Disabling %zu motors...", handle->drivers.size());
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    auto disable_result = handle->drivers[i]->motorControl.Disable();
    if (disable_result.IsErr()) {
      ESP_LOGW(TAG, "⚠ Failed to disable motor %zu (ErrorCode: %d)", i, static_cast<int>(disable_result.Error()));
    } else {
      ESP_LOGI(TAG, "✓ Motor %zu disabled", i);
    }
  }
  
  return all_reached;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║         ESP32 TMC51x0 SPI DAISY CHAIN COMPREHENSIVE TEST SUITE               ║");
  ESP_LOGI(TAG, "║                         HardFOC TMC51x0 Driver Tests                         ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  ESP_LOGI(TAG, "Driver version: %s", tmc51x0::GetDriverVersion());
  ESP_LOGI(TAG, "Chain length: %u driver(s) -- change TEST_CHAIN_LENGTH to match your hardware", TEST_CHAIN_LENGTH);
  
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

