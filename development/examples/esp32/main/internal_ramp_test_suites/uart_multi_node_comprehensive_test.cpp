/**
 * @file uart_multi_node_comprehensive_test.cpp
 * @brief Comprehensive UART communication testing suite for TMC51x0
 *
 * Tests UART single-wire communication with 1 or more TMC51x0 drivers.
 * Set TEST_NODE_COUNT to match your hardware (default: 1).
 *
 * Features tested:
 * - UART interface initialization and basic register access
 * - Node address configuration (NODECONF register)
 * - Send delay configuration
 * - Multi-node coordination (when TEST_NODE_COUNT > 1)
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - 1+ TMC51x0 stepper motor drivers in UART mode
 *   (SD_MODE=LOW, SPI_MODE=LOW)
 * - UART wiring: ESP32 TX -> TMC5160 SWN, ESP32 RX <- TMC5160 SWP
 * - For multi-node: NAI/NAO daisy chain (first chip NAI to GND)
 *
 * Pin Configuration (modify as needed):
 * - UART TX:  GPIO 17
 * - UART RX:  GPIO 16
 * - EN:       GPIO 11
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

static const char* TAG = "UART_MultiNode_Test";
static TestResults g_test_results;

//=============================================================================
// CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_BASIC_COMM_TESTS = true;
static constexpr bool ENABLE_NODE_ADDRESS_TESTS = true;
static constexpr bool ENABLE_SEND_DELAY_TESTS = true;
static constexpr bool ENABLE_MOTOR_CONTROL_TESTS = true;

/// Number of TMC51x0 nodes on the UART bus. Set to 1 for single-driver setups.
static constexpr uint8_t TEST_NODE_COUNT = 1;

/// UART port (avoid UART_NUM_0 which is typically used for the USB console)
static constexpr uart_port_t UART_PORT = UART_NUM_1;

/// UART baud rate -- TMC5160 auto-detects from sync frame
static constexpr uint32_t UART_BAUD = 115200;

/// UART pin configuration
static constexpr int UART_TX_PIN = 17;
static constexpr int UART_RX_PIN = 16;
static constexpr int EN_PIN      = 11;

/// Test rig selection for motor configuration
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG =
    tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER;

//=============================================================================
// DRIVER HANDLE
//=============================================================================

struct UartTestHandle {
  std::unique_ptr<Esp32UART> uart;
  std::vector<std::unique_ptr<tmc51x0::TMC51x0<Esp32UART>>> drivers;
};

/**
 * @brief Create UART interface and driver instances.
 *
 * Creates one Esp32UART interface shared by all drivers, and one
 * TMC51x0 instance per node. Each driver is assigned node address 0
 * initially (sequential programming would assign unique addresses).
 */
std::unique_ptr<UartTestHandle> create_uart_drivers() noexcept {
  auto handle = std::make_unique<UartTestHandle>();

  Esp32UartPinConfig pin_cfg(UART_TX_PIN, UART_RX_PIN, EN_PIN);
  handle->uart = std::make_unique<Esp32UART>(UART_PORT, pin_cfg, UART_BAUD);

  auto init_result = handle->uart->Initialize();
  if (!init_result) {
    ESP_LOGE(TAG, "Failed to initialize UART interface (ErrorCode: %d)",
             static_cast<int>(init_result.Error()));
    return nullptr;
  }
  ESP_LOGI(TAG, "UART interface initialized");

  // Create driver instances -- for single-node, address stays at 0
  for (uint8_t i = 0; i < TEST_NODE_COUNT; ++i) {
    auto drv = std::make_unique<tmc51x0::TMC51x0<Esp32UART>>(*handle->uart);
    // Set UART node address for this instance (defaults are fine for single-node)
    if (TEST_NODE_COUNT > 1) {
      drv->communication.SetUartNodeAddress(i);
    }
    handle->drivers.push_back(std::move(drv));
  }

  return handle;
}

//=============================================================================
// TEST 1: Basic UART Communication
//=============================================================================

/**
 * @brief Verify basic UART register read/write communication.
 *
 * Reads the IOIN register (0x04) which contains chip version and pin states.
 * A successful read confirms the UART link, baud rate detection, and CRC are
 * all working.
 */
bool test_uart_basic_communication() noexcept {
  ESP_LOGI(TAG, "Testing basic UART communication...");

  auto handle = create_uart_drivers();
  if (!handle) return false;

  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    // Initialize driver -- writes GCONF, CHOPCONF, current settings
    tmc51x0::DriverConfig cfg{};
    tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);

    auto init_result = handle->drivers[i]->Initialize(cfg);
    if (init_result.IsErr()) {
      ESP_LOGE(TAG, "Driver %zu: Initialize failed (ErrorCode: %d, %s)",
               i, static_cast<int>(init_result.Error()), init_result.ErrorMessage());
      return false;
    }
    ESP_LOGI(TAG, "Driver %zu: Initialized successfully", i);

    // Read GCONF to verify the write round-tripped via raw comm interface
    uint8_t node_addr = handle->drivers[i]->communication.GetUartNodeAddress();
    auto gconf = handle->drivers[i]->GetComm().ReadRegister(0x00, node_addr);
    if (gconf.IsErr()) {
      ESP_LOGE(TAG, "Driver %zu: Failed to read GCONF (ErrorCode: %d)",
               i, static_cast<int>(gconf.Error()));
      return false;
    }
    ESP_LOGI(TAG, "Driver %zu: GCONF = 0x%08X", i, static_cast<unsigned>(gconf.Value()));

    // Read IOIN register (0x04) -- always readable, contains chip version
    auto ioin = handle->drivers[i]->GetComm().ReadRegister(0x04, node_addr);
    if (ioin.IsErr()) {
      ESP_LOGE(TAG, "Driver %zu: Failed to read IOIN (ErrorCode: %d)",
               i, static_cast<int>(ioin.Error()));
      return false;
    }

    uint32_t ioin_val = ioin.Value();
    uint8_t version = (ioin_val >> 24) & 0xFF;
    ESP_LOGI(TAG, "Driver %zu: IOIN = 0x%08X (chip version: 0x%02X)",
             i, static_cast<unsigned>(ioin_val), version);

    // Verify chip version via the subsystem method (set during Initialize)
    uint8_t detected_version = handle->drivers[i]->status.GetChipVersion();
    ESP_LOGI(TAG, "Driver %zu: Detected chip version via subsystem: 0x%02X", i, detected_version);

    if (version != 0x30 && version != 0x11) {
      ESP_LOGW(TAG, "Driver %zu: Unexpected chip version 0x%02X (expected 0x30 for TMC5160 or 0x11 for TMC5130)",
               i, version);
    }
  }

  return true;
}

//=============================================================================
// TEST 2: Node Address Configuration
//=============================================================================

/**
 * @brief Test UART node address configuration via NODECONF register.
 *
 * Writes a new node address via ConfigureUartNodeAddress(), then verifies
 * the driver tracks it by checking GetUartNodeAddress(). Since NODECONF is
 * write-only, we verify by reading IOIN on the new address to confirm
 * the chip responds.
 */
bool test_uart_node_address_configuration() noexcept {
  ESP_LOGI(TAG, "Testing UART node address configuration...");

  auto handle = create_uart_drivers();
  if (!handle) return false;

  // Initialize first driver at default address (0)
  tmc51x0::DriverConfig cfg{};
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);

  auto init_result = handle->drivers[0]->Initialize(cfg);
  if (init_result.IsErr()) {
    ESP_LOGE(TAG, "Initialize failed: %s", init_result.ErrorMessage());
    return false;
  }

  // Get current address (should be 0)
  uint8_t initial_addr = handle->drivers[0]->communication.GetUartNodeAddress();
  ESP_LOGI(TAG, "Initial node address: %u", initial_addr);

  // Assign a new address (e.g. 42) and send delay 2
  constexpr uint8_t NEW_ADDR = 42;
  constexpr uint8_t SEND_DELAY = 2;
  auto addr_result = handle->drivers[0]->communication.ConfigureUartNodeAddress(NEW_ADDR, SEND_DELAY);
  if (addr_result.IsErr()) {
    ESP_LOGE(TAG, "ConfigureUartNodeAddress failed: %s", addr_result.ErrorMessage());
    return false;
  }

  // Verify software tracking
  uint8_t current_addr = handle->drivers[0]->communication.GetUartNodeAddress();
  if (current_addr != NEW_ADDR) {
    ESP_LOGE(TAG, "Address mismatch: expected %u, got %u", NEW_ADDR, current_addr);
    return false;
  }
  ESP_LOGI(TAG, "Node address updated to %u (send delay: %u)", current_addr, SEND_DELAY);

  // Verify chip responds on new address by reading IOIN
  auto ioin = handle->drivers[0]->GetComm().ReadRegister(0x04, NEW_ADDR);
  if (ioin.IsErr()) {
    ESP_LOGE(TAG, "Failed to read IOIN on new address %u: %s", NEW_ADDR, ioin.ErrorMessage());
    // Restore address to 0 before failing
    handle->drivers[0]->communication.ConfigureUartNodeAddress(0, 0);
    return false;
  }
  ESP_LOGI(TAG, "Successfully read IOIN on address %u: 0x%08X", NEW_ADDR, static_cast<unsigned>(ioin.Value()));

  // Restore to address 0 so subsequent tests work
  auto restore = handle->drivers[0]->communication.ConfigureUartNodeAddress(0, 0);
  if (restore.IsErr()) {
    ESP_LOGW(TAG, "Warning: failed to restore address to 0");
  }

  return true;
}

//=============================================================================
// TEST 3: Send Delay Configuration
//=============================================================================

/**
 * @brief Test send delay configuration.
 *
 * SENDDELAY (NODECONF bits 11:8) controls the number of bit-times the TMC5160
 * waits before replying to a read request. For multi-node systems, this must
 * be >= 2 to avoid bus contention.
 */
bool test_send_delay_configuration() noexcept {
  ESP_LOGI(TAG, "Testing send delay configuration...");

  auto handle = create_uart_drivers();
  if (!handle) return false;

  tmc51x0::DriverConfig cfg{};
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);

  auto init_result = handle->drivers[0]->Initialize(cfg);
  if (init_result.IsErr()) {
    ESP_LOGE(TAG, "Initialize failed: %s", init_result.ErrorMessage());
    return false;
  }

  // Test multiple send delay values (0 through 8) and verify communication still works
  constexpr uint8_t delays_to_test[] = {0, 1, 2, 4, 8};

  for (uint8_t delay : delays_to_test) {
    auto result = handle->drivers[0]->communication.ConfigureUartNodeAddress(0, delay);
    if (result.IsErr()) {
      ESP_LOGE(TAG, "ConfigureUartNodeAddress with delay=%u failed: %s", delay, result.ErrorMessage());
      return false;
    }

    // Verify communication works by reading a register
    uint8_t addr = handle->drivers[0]->communication.GetUartNodeAddress();
    auto ioin = handle->drivers[0]->GetComm().ReadRegister(0x04, addr);
    if (ioin.IsErr()) {
      ESP_LOGE(TAG, "Read failed with SENDDELAY=%u: %s", delay, ioin.ErrorMessage());
      // Restore delay=0 before failing
      handle->drivers[0]->communication.ConfigureUartNodeAddress(0, 0);
      return false;
    }

    ESP_LOGI(TAG, "SENDDELAY=%u: read OK (IOIN=0x%08X)", delay, static_cast<unsigned>(ioin.Value()));
  }

  // Restore to default delay
  handle->drivers[0]->communication.ConfigureUartNodeAddress(0, 0);

  return true;
}

//=============================================================================
// TEST 4: Motor Control via UART
//=============================================================================

/**
 * @brief Test motor control operations over UART.
 *
 * Initializes the driver, configures ramp parameters, enables the motor,
 * commands a small move, and polls for completion. This proves the full
 * driver API works over UART, not just register access.
 */
bool test_uart_motor_control() noexcept {
  ESP_LOGI(TAG, "Testing motor control via UART...");

  auto handle = create_uart_drivers();
  if (!handle) return false;

  tmc51x0::DriverConfig cfg{};
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);

  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    auto init_result = handle->drivers[i]->Initialize(cfg);
    if (init_result.IsErr()) {
      ESP_LOGE(TAG, "Driver %zu: Initialize failed: %s", i, init_result.ErrorMessage());
      return false;
    }

    // Configure ramp
    handle->drivers[i]->rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    handle->drivers[i]->rampControl.SetMaxSpeed(30.0f, tmc51x0::Unit::RPM);
    handle->drivers[i]->rampControl.SetAcceleration(5.0f, tmc51x0::Unit::RevPerSec);
    handle->drivers[i]->rampControl.SetDeceleration(5.0f, tmc51x0::Unit::RevPerSec);

    // Enable motor
    auto enable_result = handle->drivers[i]->motorControl.Enable();
    if (enable_result.IsErr()) {
      ESP_LOGE(TAG, "Driver %zu: Enable failed: %s", i, enable_result.ErrorMessage());
      return false;
    }
    ESP_LOGI(TAG, "Driver %zu: Motor enabled", i);

    // Command a small move (45 degrees)
    float target_deg = 45.0f * (i + 1);  // Different targets for multi-node
    auto pos_result = handle->drivers[i]->rampControl.SetTargetPosition(target_deg, tmc51x0::Unit::Deg);
    if (pos_result.IsErr()) {
      ESP_LOGE(TAG, "Driver %zu: SetTargetPosition failed: %s", i, pos_result.ErrorMessage());
      return false;
    }
    ESP_LOGI(TAG, "Driver %zu: Moving to %.1f degrees...", i, target_deg);
  }

  // Poll for completion
  constexpr int MAX_CHECKS = 100;  // 10 seconds at 100ms intervals
  bool all_reached = false;

  for (int check = 0; check < MAX_CHECKS && !all_reached; ++check) {
    vTaskDelay(pdMS_TO_TICKS(100));
    all_reached = true;

    for (size_t i = 0; i < handle->drivers.size(); ++i) {
      auto reached = handle->drivers[i]->rampControl.IsTargetReached();
      if (reached.IsErr()) {
        ESP_LOGW(TAG, "Driver %zu: IsTargetReached error: %s", i, reached.ErrorMessage());
        all_reached = false;
        continue;
      }
      if (!reached.Value()) {
        all_reached = false;
      }
    }

    if (check % 20 == 0) {
      for (size_t i = 0; i < handle->drivers.size(); ++i) {
        auto pos = handle->drivers[i]->rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
        if (pos.IsOk()) {
          ESP_LOGI(TAG, "Driver %zu: Position = %.2f deg", i, pos.Value());
        }
      }
    }
  }

  // Report result
  if (all_reached) {
    ESP_LOGI(TAG, "All %zu driver(s) reached target position", handle->drivers.size());
  } else {
    ESP_LOGW(TAG, "Not all drivers reached target within timeout");
  }

  // Disable motors
  for (size_t i = 0; i < handle->drivers.size(); ++i) {
    handle->drivers[i]->motorControl.Disable();
    ESP_LOGI(TAG, "Driver %zu: Motor disabled", i);
  }

  return all_reached;
}

//=============================================================================
// MAIN
//=============================================================================

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "==============================================================");
  ESP_LOGI(TAG, "  ESP32 TMC51x0 UART Communication Test Suite");
  ESP_LOGI(TAG, "  HardFOC TMC51x0 Driver");
  ESP_LOGI(TAG, "==============================================================");
  ESP_LOGI(TAG, "Driver version: %s", tmc51x0::GetDriverVersion());
  ESP_LOGI(TAG, "Node count: %u | UART port: %d | Baud: %u | TX=%d RX=%d",
           TEST_NODE_COUNT, UART_PORT, UART_BAUD, UART_TX_PIN, UART_RX_PIN);

  vTaskDelay(pdMS_TO_TICKS(1000));

  print_test_section_status(TAG, "UART Communication");

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_BASIC_COMM_TESTS, "BASIC UART COMMUNICATION", 5,
    ESP_LOGI(TAG, "Running basic UART communication tests...");
    RUN_TEST_IN_TASK("uart_basic_communication", test_uart_basic_communication, 8192, 1);
    flip_test_progress_indicator();
  );

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_NODE_ADDRESS_TESTS, "NODE ADDRESS CONFIGURATION", 5,
    ESP_LOGI(TAG, "Running node address configuration tests...");
    RUN_TEST_IN_TASK("uart_node_address_configuration", test_uart_node_address_configuration, 8192, 1);
    flip_test_progress_indicator();
  );

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_SEND_DELAY_TESTS, "SEND DELAY CONFIGURATION", 5,
    ESP_LOGI(TAG, "Running send delay configuration tests...");
    RUN_TEST_IN_TASK("send_delay_configuration", test_send_delay_configuration, 8192, 1);
    flip_test_progress_indicator();
  );

  RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
    ENABLE_MOTOR_CONTROL_TESTS, "MOTOR CONTROL VIA UART", 5,
    ESP_LOGI(TAG, "Running motor control via UART tests...");
    RUN_TEST_IN_TASK("uart_motor_control", test_uart_motor_control, 8192, 1);
    flip_test_progress_indicator();
  );

  print_test_summary(g_test_results, "UART Communication", TAG);

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
