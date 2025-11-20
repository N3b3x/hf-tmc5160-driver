/**
 * @file uart_daisy_chain_example.cpp
 * @brief Example demonstrating daisy-chained TMC5160 drivers in UART mode
 *
 * This example shows how to use multiple TMC5160 drivers on a single UART bus
 * using daisy chaining with NAI/NAO pins. All chips share the same UART bus,
 * while NAI/NAO pins are used for sequential addressing.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - 2-3 TMC5160 stepper motor drivers (daisy-chained via UART)
 * - Stepper motors connected to each TMC5160
 * - UART connection: All chips share TXD/RXD
 * - NAI/NAO pins for addressing: First chip NAI to GND, chain NAO→NAI
 *
 * Pin Configuration (modify as needed):
 * - UART: TX=17, RX=16, TXEN=4 (optional, for RS485 transceiver)
 * - NAI/NAO: GPIO pins for controlling addressing (e.g., NAI=25, NAO=26)
 * - Control: EN=2, DIR=4, STEP=15 (can be shared or separate per chip)
 * - Mode pins: SD_MODE=0 (GND), SPI_MODE=0 (GND) for UART mode
 *
 * UART Daisy-Chain Wiring:
 * - MCU TXD ──> All chips UART_RXD (shared bus)
 * - MCU RXD <── All chips UART_TXD (shared bus)
 * - First chip: NAI tied to GND (address 0)
 * - Chip 1 NAO ──> Chip 2 NAI
 * - Chip 2 NAO ──> Chip 3 NAI (if 3 chips)
 *
 * Mode Configuration:
 * - SD_MODE (pin 21): Must be LOW (0) - tie to GND
 * - SPI_MODE (pin 22): Must be LOW (0) - tie to GND
 * - When both are LOW, UART operation is enabled
 *
 * Pin Functions in UART Mode:
 * - SDI_CFG1 (pin 15) → NAI (Next Address Input)
 * - SDO_CFG0 (pin 16) → NAO (Next Address Output)
 * - DIAG0_SWN (pin 26) → SWION (Single Wire I/O Negative)
 * - DIAG1_SWP (pin 27) → SWIOP (Single Wire I/O Positive)
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdarg>
#include <cstdint>

static const char *TAG = "UartDaisyChain";

/**
 * @brief ESP32 UART implementation with NAI/NAO pin control for daisy chaining
 */
class Esp32UART : public tmc5160::UartCommInterface<Esp32UART> {
public:
  Esp32UART(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
            gpio_num_t txen_pin, gpio_num_t nai_pin, gpio_num_t nao_pin,
            uint8_t slave_address)
      : UartCommInterface(true, true, true, slave_address),
        uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin),
        txen_pin_(txen_pin), nai_pin_(nai_pin), nao_pin_(nao_pin) {
    // Configure UART
    uart_config_t uart_config = {};
    uart_config.baud_rate = 500000; // 500 kbps (adjust as needed)
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    uart_param_config(uart_num_, &uart_config);
    uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num_, 1024, 1024, 0, NULL, 0);

    // Configure TXEN pin (for RS485 transceiver, if used)
    if (txen_pin_ != GPIO_NUM_NC) {
      gpio_set_direction(txen_pin_, GPIO_MODE_OUTPUT);
      gpio_set_level(txen_pin_, 0); // Receive mode by default
    }

    // Configure NAI pin (input for reading, but can be set for first chip)
    if (nai_pin_ != GPIO_NUM_NC) {
      gpio_set_direction(nai_pin_, GPIO_MODE_OUTPUT);
      gpio_set_level(nai_pin_, 0); // Default to LOW
    }

    // Configure NAO pin (output for reading from chip)
    if (nao_pin_ != GPIO_NUM_NC) {
      gpio_set_direction(nao_pin_, GPIO_MODE_INPUT);
      gpio_set_pull_mode(nao_pin_, GPIO_PULLDOWN_ONLY);
    }
  }

  tmc5160::CommMode GetMode() const noexcept {
    return tmc5160::CommMode::UART;
  }

  bool UartSend(const uint8_t *data, size_t length) noexcept {
    if (txen_pin_ != GPIO_NUM_NC) {
      gpio_set_level(txen_pin_, 1); // Enable transmitter
    }

    int bytes_written = uart_write_bytes(uart_num_, data, length);
    uart_wait_tx_done(uart_num_, portMAX_DELAY);

    if (txen_pin_ != GPIO_NUM_NC) {
      gpio_set_level(txen_pin_, 0); // Disable transmitter (receive mode)
    }

    return bytes_written == static_cast<int>(length);
  }

  bool UartReceive(uint8_t *data, size_t length) noexcept {
    int bytes_read = uart_read_bytes(uart_num_, data, length, pdMS_TO_TICKS(100));
    return bytes_read == static_cast<int>(length);
  }

  bool GpioSet(tmc5160::TMC5160CtrlPin pin,
               tmc5160::GpioSignal signal) noexcept {
    // UART mode doesn't use standard GPIO pins for control
    // (EN, DIR, STEP are handled differently in UART mode)
    return true;
  }

  bool GpioRead(tmc5160::TMC5160CtrlPin pin,
                tmc5160::GpioSignal &signal) noexcept {
    return false;
  }

  /**
   * @brief Set NAI (Next Address Input) pin state
   * @param active true for high, false for low
   */
  bool SetNaiPin(bool active) noexcept {
    if (nai_pin_ == GPIO_NUM_NC) {
      return false;
    }
    gpio_set_level(nai_pin_, active ? 1 : 0);
    return true;
  }

  /**
   * @brief Read NAO (Next Address Output) pin state from chip
   * @param active Reference to store pin state
   */
  bool GetNaoPin(bool &active) noexcept {
    if (nao_pin_ == GPIO_NUM_NC) {
      return false;
    }
    int level = gpio_get_level(nao_pin_);
    active = (level == 1);
    return true;
  }

  void DebugLog(int level, const char *tag, const char *format,
                va_list args) noexcept {
    esp_log_level_t esp_level;
    switch (level) {
    case 0:
      esp_level = ESP_LOG_ERROR;
      break;
    case 1:
      esp_level = ESP_LOG_WARN;
      break;
    case 2:
      esp_level = ESP_LOG_INFO;
      break;
    case 3:
      esp_level = ESP_LOG_DEBUG;
      break;
    default:
      esp_level = ESP_LOG_VERBOSE;
      break;
    }
    esp_log_writev(esp_level, tag, format, args);
  }

  void DelayMs(uint32_t ms) noexcept { vTaskDelay(pdMS_TO_TICKS(ms)); }

  void DelayUs(uint32_t us) noexcept { esp_rom_delay_us(us); }

private:
  uart_port_t uart_num_;
  gpio_num_t tx_pin_;
  gpio_num_t rx_pin_;
  gpio_num_t txen_pin_;
  gpio_num_t nai_pin_;  // NAI pin (SDI_CFG1, pin 15 in UART mode)
  gpio_num_t nao_pin_;  // NAO pin (SDO_CFG0, pin 16 in UART mode)
};

/**
 * @brief Helper function to program a chip and verify its address
 */
template <typename CommType>
bool programChipAndVerify(tmc5160::TMC5160<CommType> &driver, uint8_t target_address,
                         CommType &uart_interface) {
  ESP_LOGI(TAG, "Programming chip to address %d...", target_address);

  // Configure slave address (send_delay >= 2 for multi-node systems)
  if (!driver.communication.ConfigureSlaveAddress(target_address, 2)) {
    ESP_LOGE(TAG, "Failed to configure slave address %d", target_address);
    return false;
  }

  // Update the UART interface's slave address for subsequent operations
  uart_interface.SetSlaveAddress(target_address);

  // Small delay to ensure register write completes
  uart_interface.DelayMs(10);

  // Verify the address was set correctly
  uint8_t read_addr = driver.communication.GetSlaveAddress();
  if (read_addr != target_address) {
    ESP_LOGE(TAG, "Address verification failed: expected %d, got %d",
             target_address, read_addr);
    return false;
  }

  ESP_LOGI(TAG, "Chip successfully programmed to address %d", target_address);

  // Check NAO pin state (should be LOW after programming to enable next chip)
  bool nao_state;
  if (uart_interface.GetNaoPin(nao_state)) {
    ESP_LOGI(TAG, "NAO pin state: %s", nao_state ? "HIGH" : "LOW");
    if (nao_state) {
      ESP_LOGW(TAG, "NAO is HIGH - next chip may not be accessible yet");
    }
  }

  return true;
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 UART Daisy-Chain Example");
  ESP_LOGI(TAG, "This example demonstrates multiple TMC5160 drivers on a single UART bus");

  // Pin configuration - modify as needed for your hardware
  const gpio_num_t UART_TX = GPIO_NUM_17;
  const gpio_num_t UART_RX = GPIO_NUM_16;
  const gpio_num_t UART_TXEN = GPIO_NUM_4; // Optional, for RS485 transceiver
  const gpio_num_t NAI_PIN = GPIO_NUM_25;  // NAI pin (SDI_CFG1, pin 15)
  const gpio_num_t NAO_PIN = GPIO_NUM_26;  // NAO pin (SDO_CFG0, pin 16)

  // Create UART interface for first chip (starts at address 0)
  // First chip's NAI is tied to GND in hardware
  Esp32UART uart1(UART_NUM_1, UART_TX, UART_RX, UART_TXEN, NAI_PIN, NAO_PIN, 0);
  uart1.SetNaiPin(false); // Ensure NAI is LOW (tied to GND for first chip)

  ESP_LOGI(TAG, "UART interface initialized");
  ESP_LOGI(TAG, "First chip: NAI tied to GND, responds to address 0");

  // Create driver instance for first chip
  tmc5160::TMC5160 driver1(uart1, 12'000'000); // 12 MHz clock

  // Program first chip to address 1
  if (!programChipAndVerify(driver1, 1, uart1)) {
    ESP_LOGE(TAG, "Failed to program first chip");
    return;
  }

  // After programming first chip, its NAO should be LOW to enable second chip
  // In hardware: chip1.NAO -> chip2.NAI
  // For this example, we'll create a new UART interface for the second chip
  // In a real system, you might reuse the same interface and just change the address

  // Create UART interface for second chip (now responds to address 1)
  // Note: In hardware, chip2's NAI is connected to chip1's NAO
  Esp32UART uart2(UART_NUM_1, UART_TX, UART_RX, UART_TXEN, GPIO_NUM_NC,
                  GPIO_NUM_NC, 1);
  uart2.SetSlaveAddress(1);

  ESP_LOGI(TAG, "Second chip: NAI connected to chip1 NAO, responds to address 1");

  // Create driver instance for second chip
  tmc5160::TMC5160 driver2(uart2, 12'000'000);

  // Program second chip to address 2
  if (!programChipAndVerify(driver2, 2, uart2)) {
    ESP_LOGE(TAG, "Failed to program second chip");
    return;
  }

  // Uncomment for third chip:
  /*
  Esp32UART uart3(UART_NUM_1, UART_TX, UART_RX, UART_TXEN, GPIO_NUM_NC,
                  GPIO_NUM_NC, 2);
  uart3.SetSlaveAddress(2);
  tmc5160::TMC5160 driver3(uart3, 12'000'000);
  if (!programChipAndVerify(driver3, 3, uart3)) {
    ESP_LOGE(TAG, "Failed to program third chip");
    return;
  }
  */

  ESP_LOGI(TAG, "All chips programmed successfully!");

  // Configure driver settings
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;          // Run current (0-31)
  cfg.motor.ihold = 10;         // Hold current (0-31)
  cfg.motor.global_scaler = 32; // Global current scaler (32-256)
  cfg.chopper.toff = 5;         // Chopper off time
  cfg.chopper.mres = 4;         // 16 microsteps
  cfg.chopper.intpol = true;    // Enable interpolation

  // Initialize each driver
  if (!driver1.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver 1 (address 1)");
    return;
  }
  ESP_LOGI(TAG, "Driver 1 initialized (address 1)");

  if (!driver2.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver 2 (address 2)");
    return;
  }
  ESP_LOGI(TAG, "Driver 2 initialized (address 2)");

  // Configure ramp control for each driver
  driver1.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver1.rampControl.SetTargetPosition(1000);  // Move 1000 steps
  driver1.rampControl.SetMaxSpeed(1000.0f);     // 1000 steps/s
  driver1.rampControl.SetAcceleration(500.0f);   // 500 steps/s²

  driver2.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver2.rampControl.SetTargetPosition(2000);  // Move 2000 steps
  driver2.rampControl.SetMaxSpeed(1000.0f);       // 1000 steps/s
  driver2.rampControl.SetAcceleration(500.0f);   // 500 steps/s²

  ESP_LOGI(TAG, "Ramp control configured for both drivers");

  // Enable motors
  driver1.motorControl.Enable();
  driver2.motorControl.Enable();
  ESP_LOGI(TAG, "Motors enabled");

  // Monitor motion
  ESP_LOGI(TAG, "Monitoring motion...");
  for (int i = 0; i < 50; ++i) {
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay

    int32_t pos1 = driver1.rampControl.GetCurrentPosition();
    int32_t pos2 = driver2.rampControl.GetCurrentPosition();

    bool reached1 = driver1.rampControl.IsTargetReached();
    bool reached2 = driver2.rampControl.IsTargetReached();

    ESP_LOGI(TAG, "Driver 1 (addr 1): Position=%ld, Target Reached=%s", pos1,
             reached1 ? "YES" : "NO");
    ESP_LOGI(TAG, "Driver 2 (addr 2): Position=%ld, Target Reached=%s", pos2,
             reached2 ? "YES" : "NO");

    if (reached1 && reached2) {
      ESP_LOGI(TAG, "Both drivers reached their targets!");
      break;
    }
  }

  // Read status from each chip
  ESP_LOGI(TAG, "Reading status from each chip...");

  uint32_t gstat1, gstat2;
  if (driver1.GetComm().ReadRegister(0x00, gstat1)) {
    ESP_LOGI(TAG, "Driver 1 (addr 1) GSTAT: 0x%08lX", gstat1);
  }

  if (driver2.GetComm().ReadRegister(0x00, gstat2)) {
    ESP_LOGI(TAG, "Driver 2 (addr 2) GSTAT: 0x%08lX", gstat2);
  }

  // Disable motors
  driver1.motorControl.Disable();
  driver2.motorControl.Disable();
  ESP_LOGI(TAG, "Motors disabled");

  ESP_LOGI(TAG, "Example completed successfully");
}

