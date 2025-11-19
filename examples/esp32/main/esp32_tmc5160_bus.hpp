/**
 * @file esp32_tmc5160_bus.hpp
 * @brief ESP32-specific communication interfaces for TMC5160 using SPI and UART
 *
 * This file provides ESP32-specific implementations of the TMC5160
 * communication interfaces using ESP-IDF SPI and UART drivers.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include "../../../inc/tmc5160_comm_interface.hpp"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdarg>
#include <cstdint>
#include <cstring>

static const char *BUS_TAG = "TMC5160_Bus";

/**
 * @brief Pin configuration structure for TMC5160 control pins
 *
 * All pins are optional - use GPIO_NUM_NC for pins that are not connected.
 * This allows flexible hardware configurations where not all pins are used.
 */
struct TMC5160PinConfig {
  // SPI pins (required)
  gpio_num_t mosi_pin;
  gpio_num_t miso_pin;
  gpio_num_t sclk_pin;
  gpio_num_t cs_pin;

  // Basic control pins (commonly used)
  gpio_num_t en_pin = GPIO_NUM_NC;      ///< EN/DRV_ENN pin (Pin 28)
  gpio_num_t dir_pin = GPIO_NUM_NC;     ///< DIR/REFR_DIR pin (Pin 18)
  gpio_num_t step_pin = GPIO_NUM_NC;    ///< STEP/REFL_STEP pin (Pin 17)

  // Reference switch pins (when SD_MODE=0)
  gpio_num_t refl_step_pin = GPIO_NUM_NC;  ///< REFL_STEP pin (Pin 17, when SD_MODE=0)
  gpio_num_t refr_dir_pin = GPIO_NUM_NC;   ///< REFR_DIR pin (Pin 18, when SD_MODE=0)

  // Encoder interface pins
  gpio_num_t enca_pin = GPIO_NUM_NC;    ///< ENCA_DCIN_CFG5 pin (Pin 24)
  gpio_num_t encb_pin = GPIO_NUM_NC;    ///< ENCB_DCEN_CFG4 pin (Pin 23)
  gpio_num_t encn_pin = GPIO_NUM_NC;    ///< ENCN_DCO_CFG6 pin (Pin 25)

  // Diagnostic/interrupt output pins (open-drain, need pull-ups)
  gpio_num_t diag0_pin = GPIO_NUM_NC;   ///< DIAG0_SWN pin (Pin 26)
  gpio_num_t diag1_pin = GPIO_NUM_NC;   ///< DIAG1_SWP pin (Pin 27)

  // Mode selection pins
  gpio_num_t sd_mode_pin = GPIO_NUM_NC;  ///< SD_MODE pin (Pin 21)
  gpio_num_t spi_mode_pin = GPIO_NUM_NC; ///< SPI_MODE pin (Pin 22)

  // Clock input
  gpio_num_t clk_pin = GPIO_NUM_NC;     ///< CLK pin (Pin 12)

  // Driver enable (hardware kill signal)
  gpio_num_t drv_enn_pin = GPIO_NUM_NC; ///< DRV_ENN pin (Pin 28, same as EN)
};

/**
 * @brief ESP32 SPI implementation of TMC5160 communication interface
 *
 * This class provides SPI communication for the TMC5160 using ESP-IDF SPI
 * driver. Supports all TMC5160 control pins with optional configuration.
 */
class Esp32SPI : public tmc5160::SpiCommInterface<Esp32SPI> {
public:
  /**
   * @brief Construct ESP32 SPI communication interface (simplified constructor)
   * @param host SPI host device (e.g., SPI2_HOST)
   * @param mosi_pin MOSI GPIO pin
   * @param miso_pin MISO GPIO pin
   * @param sclk_pin SCLK GPIO pin
   * @param cs_pin CS GPIO pin
   * @param en_pin EN control pin
   * @param dir_pin DIR control pin
   * @param step_pin STEP control pin
   * @param clock_speed_hz SPI clock speed in Hz (max 4 MHz recommended)
   */
  Esp32SPI(spi_host_device_t host, gpio_num_t mosi_pin, gpio_num_t miso_pin,
           gpio_num_t sclk_pin, gpio_num_t cs_pin, gpio_num_t en_pin,
           gpio_num_t dir_pin, gpio_num_t step_pin,
           uint32_t clock_speed_hz = 4000000) noexcept
      : SpiCommInterface(true, true, true), // EN, DIR, STEP active high
        host_(host), clock_speed_hz_(clock_speed_hz),
        device_handle_(nullptr), initialized_(false) {
    // Initialize pin config with provided pins
    pin_config_.mosi_pin = mosi_pin;
    pin_config_.miso_pin = miso_pin;
    pin_config_.sclk_pin = sclk_pin;
    pin_config_.cs_pin = cs_pin;
    pin_config_.en_pin = en_pin;
    pin_config_.dir_pin = dir_pin;
    pin_config_.step_pin = step_pin;
    // All other pins default to GPIO_NUM_NC (not connected)
  }

  /**
   * @brief Construct ESP32 SPI communication interface (full pin configuration)
   * @param host SPI host device (e.g., SPI2_HOST)
   * @param pin_config Pin configuration structure with all optional pins
   * @param clock_speed_hz SPI clock speed in Hz (max 4 MHz recommended)
   */
  Esp32SPI(spi_host_device_t host, const TMC5160PinConfig &pin_config,
           uint32_t clock_speed_hz = 4000000) noexcept
      : SpiCommInterface(true, true, true), // EN, DIR, STEP active high
        host_(host), pin_config_(pin_config), clock_speed_hz_(clock_speed_hz),
        device_handle_(nullptr), initialized_(false) {
    // Configure active levels for pins that are typically active-low
    if (pin_config_.drv_enn_pin != GPIO_NUM_NC) {
      SetPinActiveLevel(tmc5160::TMC5160CtrlPin::DRV_ENN, false); // Active low
    }
  }

  /**
   * @brief Destructor - cleans up SPI resources
   */
  ~Esp32SPI() noexcept { Deinitialize(); }

  /**
   * @brief Initialize the SPI interface
   * @return true if successful, false otherwise
   */
  bool Initialize() noexcept {
    if (initialized_) {
      return true;
    }

    // Configure GPIO pins (only configure pins that are connected)
    configureGpioPins();

    // Configure SPI bus
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = pin_config_.mosi_pin;
    bus_config.miso_io_num = pin_config_.miso_pin;
    bus_config.sclk_io_num = pin_config_.sclk_pin;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 8; // TMC5160 uses 8-byte transfers
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t ret = spi_bus_initialize(host_, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
      ESP_LOGE(BUS_TAG, "Failed to initialize SPI bus: %s",
               esp_err_to_name(ret));
      return false;
    }

    // Configure SPI device (Mode 3: CPOL=1, CPHA=1)
    spi_device_interface_config_t dev_config = {};
    dev_config.clock_speed_hz = clock_speed_hz_;
    dev_config.mode = 3; // SPI Mode 3 for TMC5160
    dev_config.spics_io_num = pin_config_.cs_pin;
    dev_config.queue_size = 1;
    dev_config.cs_ena_pretrans = 2;
    dev_config.cs_ena_posttrans = 2;

    ret = spi_bus_add_device(host_, &dev_config, &device_handle_);
    if (ret != ESP_OK) {
      ESP_LOGE(BUS_TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
      spi_bus_free(host_);
      return false;
    }

    initialized_ = true;
    ESP_LOGI(BUS_TAG, "SPI interface initialized successfully");
    return true;
  }

  /**
   * @brief Get communication mode (always SPI for this interface)
   * @return CommMode::SPI
   */
  tmc5160::CommMode GetMode() const noexcept { return tmc5160::CommMode::SPI; }

  /**
   * @brief Deinitialize the SPI interface
   */
  bool Deinitialize() noexcept {
    if (!initialized_) {
      return true;
    }

    if (device_handle_) {
      spi_bus_remove_device(device_handle_);
      device_handle_ = nullptr;
    }

    spi_bus_free(host_);
    initialized_ = false;
    ESP_LOGI(BUS_TAG, "SPI interface deinitialized");
    return true;
  }

  /**
   * @brief Perform SPI transfer
   * @param tx Transmit buffer
   * @param rx Receive buffer
   * @param length Number of bytes to transfer
   * @return true if successful, false otherwise
   */
  bool SpiTransfer(const uint8_t *tx, uint8_t *rx, size_t length) noexcept {
    if (!initialized_ || !device_handle_) {
      ESP_LOGE(BUS_TAG, "SPI interface not initialized");
      return false;
    }

    spi_transaction_t trans = {};
    trans.length = length * 8;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;

    esp_err_t ret = spi_device_transmit(device_handle_, &trans);
    if (ret != ESP_OK) {
      ESP_LOGE(BUS_TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
      return false;
    }

    return true;
  }

  /**
   * @brief Set GPIO pin state
   *
   * Supports all TMC5160 control pins. Returns false if pin is not configured
   * (GPIO_NUM_NC) or not connected.
   */
  bool GpioSet(tmc5160::TMC5160CtrlPin pin,
               tmc5160::GpioSignal signal) noexcept {
    gpio_num_t gpio_pin = GPIO_NUM_NC;

    // Map TMC5160CtrlPin enum to actual GPIO pin
    switch (pin) {
    case tmc5160::TMC5160CtrlPin::EN:
      gpio_pin = pin_config_.en_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DIR:
      gpio_pin = pin_config_.dir_pin;
      break;
    case tmc5160::TMC5160CtrlPin::STEP:
      gpio_pin = pin_config_.step_pin;
      break;
    case tmc5160::TMC5160CtrlPin::REFL_STEP:
      gpio_pin = pin_config_.refl_step_pin;
      break;
    case tmc5160::TMC5160CtrlPin::REFR_DIR:
      gpio_pin = pin_config_.refr_dir_pin;
      break;
    case tmc5160::TMC5160CtrlPin::ENCA_DCIN_CFG5:
      gpio_pin = pin_config_.enca_pin;
      break;
    case tmc5160::TMC5160CtrlPin::ENCB_DCEN_CFG4:
      gpio_pin = pin_config_.encb_pin;
      break;
    case tmc5160::TMC5160CtrlPin::ENCN_DCO_CFG6:
      gpio_pin = pin_config_.encn_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DIAG0_SWN:
      gpio_pin = pin_config_.diag0_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DIAG1_SWP:
      gpio_pin = pin_config_.diag1_pin;
      break;
    case tmc5160::TMC5160CtrlPin::SD_MODE:
      gpio_pin = pin_config_.sd_mode_pin;
      break;
    case tmc5160::TMC5160CtrlPin::SPI_MODE:
      gpio_pin = pin_config_.spi_mode_pin;
      break;
    case tmc5160::TMC5160CtrlPin::CLK:
      gpio_pin = pin_config_.clk_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DRV_ENN:
      gpio_pin = pin_config_.drv_enn_pin;
      // If DRV_ENN not configured, fall back to EN pin
      if (gpio_pin == GPIO_NUM_NC) {
        gpio_pin = pin_config_.en_pin;
      }
      break;
    default:
      return false;
    }

    // Check if pin is configured
    if (gpio_pin == GPIO_NUM_NC) {
      ESP_LOGW(BUS_TAG, "Pin %d not configured (GPIO_NUM_NC)", static_cast<int>(pin));
      return false;
    }

    bool level = SignalToGpioLevel(pin, signal);
    gpio_set_level(gpio_pin, static_cast<uint32_t>(level ? 1U : 0U));
    return true;
  }

  /**
   * @brief Read GPIO pin state
   *
   * Supports reading from input pins (reference switches, encoder inputs,
   * diagnostic outputs). Returns false if pin is not configured or is output-only.
   */
  bool GpioRead(tmc5160::TMC5160CtrlPin pin,
                tmc5160::GpioSignal &signal) noexcept {
    gpio_num_t gpio_pin = GPIO_NUM_NC;

    // Map TMC5160CtrlPin enum to actual GPIO pin
    switch (pin) {
    case tmc5160::TMC5160CtrlPin::REFL_STEP:
      gpio_pin = pin_config_.refl_step_pin;
      break;
    case tmc5160::TMC5160CtrlPin::REFR_DIR:
      gpio_pin = pin_config_.refr_dir_pin;
      break;
    case tmc5160::TMC5160CtrlPin::ENCA_DCIN_CFG5:
      gpio_pin = pin_config_.enca_pin;
      break;
    case tmc5160::TMC5160CtrlPin::ENCB_DCEN_CFG4:
      gpio_pin = pin_config_.encb_pin;
      break;
    case tmc5160::TMC5160CtrlPin::ENCN_DCO_CFG6:
      gpio_pin = pin_config_.encn_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DIAG0_SWN:
      gpio_pin = pin_config_.diag0_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DIAG1_SWP:
      gpio_pin = pin_config_.diag1_pin;
      break;
    case tmc5160::TMC5160CtrlPin::SD_MODE:
      gpio_pin = pin_config_.sd_mode_pin;
      break;
    case tmc5160::TMC5160CtrlPin::SPI_MODE:
      gpio_pin = pin_config_.spi_mode_pin;
      break;
    case tmc5160::TMC5160CtrlPin::EN:
      gpio_pin = pin_config_.en_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DIR:
      gpio_pin = pin_config_.dir_pin;
      break;
    case tmc5160::TMC5160CtrlPin::STEP:
      gpio_pin = pin_config_.step_pin;
      break;
    case tmc5160::TMC5160CtrlPin::DRV_ENN:
      gpio_pin = pin_config_.drv_enn_pin;
      if (gpio_pin == GPIO_NUM_NC) {
        gpio_pin = pin_config_.en_pin;
      }
      break;
    default:
      return false;
    }

    // Check if pin is configured
    if (gpio_pin == GPIO_NUM_NC) {
      return false;
    }

    int level = gpio_get_level(gpio_pin);
    signal = GpioLevelToSignal(pin, static_cast<bool>(level != 0));
    return true;
  }

  /**
   * @brief Debug logging
   */
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

  /**
   * @brief Delay milliseconds
   */
  void DelayMs(uint32_t ms) noexcept { vTaskDelay(pdMS_TO_TICKS(ms)); }

  /**
   * @brief Delay microseconds
   */
  void DelayUs(uint32_t us) noexcept { esp_rom_delay_us(us); }

  /**
   * @brief Set chip select (CSN) pin state for daisy chaining
   * @param csn_pin_index Index of the CSN pin (0-based) for multi-chip setups
   * @param active true to assert CSN (active low), false to deassert
   * @return true if CSN was set successfully
   *
   * For single-chip setups (index 0), CSN is handled by hardware SPI driver.
   * For multi-chip setups, override this method to control additional CSN pins.
   */
  bool SetChipSelect(uint8_t csn_pin_index, bool active) noexcept {
    // Single-chip operation: CSN handled by ESP-IDF SPI driver
    if (csn_pin_index == 0) {
      return true;
    }
    // Multi-chip: would need additional CSN pins - not implemented by default
    ESP_LOGW(BUS_TAG, "Multi-chip CSN control not implemented for index %u",
             csn_pin_index);
    return false;
  }


private:
  spi_host_device_t host_;
  TMC5160PinConfig pin_config_;
  uint32_t clock_speed_hz_;
  spi_device_handle_t device_handle_;
  bool initialized_;

  /**
   * @brief Configure all GPIO pins that are connected
   *
   * Sets up output pins for control signals and input pins for reading
   * diagnostic/status signals.
   */
  bool configureGpioPins() noexcept {
    // Configure output pins (control signals)
    if (pin_config_.en_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.en_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.en_pin, static_cast<uint32_t>(0U)); // Disable by default
    }
    if (pin_config_.dir_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.dir_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.dir_pin, static_cast<uint32_t>(0U));
    }
    if (pin_config_.step_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.step_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.step_pin, static_cast<uint32_t>(0U));
    }
    if (pin_config_.refl_step_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.refl_step_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.refl_step_pin, static_cast<uint32_t>(0U));
    }
    if (pin_config_.refr_dir_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.refr_dir_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.refr_dir_pin, static_cast<uint32_t>(0U));
    }
    if (pin_config_.enca_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.enca_pin, GPIO_MODE_OUTPUT);
    }
    if (pin_config_.encb_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.encb_pin, GPIO_MODE_OUTPUT);
    }
    if (pin_config_.encn_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.encn_pin, GPIO_MODE_INPUT); // Usually input
    }
    if (pin_config_.sd_mode_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.sd_mode_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.sd_mode_pin, static_cast<uint32_t>(0U)); // Default: internal motion controller
    }
    if (pin_config_.spi_mode_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.spi_mode_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.spi_mode_pin, static_cast<uint32_t>(1U)); // Default: SPI mode
    }
    if (pin_config_.clk_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.clk_pin, GPIO_MODE_OUTPUT);
      // Clock output would be configured separately if needed
    }
    if (pin_config_.drv_enn_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.drv_enn_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(pin_config_.drv_enn_pin, static_cast<uint32_t>(1U)); // Active low, so set high to disable initially
    }

    // Configure input pins (diagnostic/status signals)
    if (pin_config_.diag0_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.diag0_pin, GPIO_MODE_INPUT);
      gpio_set_pull_mode(pin_config_.diag0_pin, GPIO_PULLUP_ONLY); // Open-drain needs pull-up
    }
    if (pin_config_.diag1_pin != GPIO_NUM_NC) {
      gpio_set_direction(pin_config_.diag1_pin, GPIO_MODE_INPUT);
      gpio_set_pull_mode(pin_config_.diag1_pin, GPIO_PULLUP_ONLY); // Open-drain needs pull-up
    }

    return true;
  }
};
