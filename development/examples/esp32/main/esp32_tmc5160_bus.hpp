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
 * @brief ESP32 SPI implementation of TMC5160 communication interface
 *
 * This class provides SPI communication for the TMC5160 using ESP-IDF SPI
 * driver.
 */
class Esp32SPI : public tmc5160::SpiCommInterface<Esp32SPI> {
public:
  /**
   * @brief Construct ESP32 SPI communication interface
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
        host_(host), mosi_pin_(mosi_pin), miso_pin_(miso_pin),
        sclk_pin_(sclk_pin), cs_pin_(cs_pin), en_pin_(en_pin),
        dir_pin_(dir_pin), step_pin_(step_pin), clock_speed_hz_(clock_speed_hz),
        device_handle_(nullptr), initialized_(false) {}

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

    // Configure GPIO pins
    gpio_set_direction(en_pin_, GPIO_MODE_OUTPUT);
    gpio_set_direction(dir_pin_, GPIO_MODE_OUTPUT);
    gpio_set_direction(step_pin_, GPIO_MODE_OUTPUT);
    gpio_set_level(en_pin_, 0); // Disable by default
    gpio_set_level(dir_pin_, 0);
    gpio_set_level(step_pin_, 0);

    // Configure SPI bus
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = mosi_pin_;
    bus_config.miso_io_num = miso_pin_;
    bus_config.sclk_io_num = sclk_pin_;
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
    dev_config.spics_io_num = cs_pin_;
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
   */
  bool GpioSet(tmc5160::TMC5160CtrlPin pin,
               tmc5160::GpioSignal signal) noexcept {
    gpio_num_t gpio_pin;
    switch (pin) {
    case tmc5160::TMC5160CtrlPin::EN:
      gpio_pin = en_pin_;
      break;
    case tmc5160::TMC5160CtrlPin::DIR:
      gpio_pin = dir_pin_;
      break;
    case tmc5160::TMC5160CtrlPin::STEP:
      gpio_pin = step_pin_;
      break;
    default:
      return false;
    }

    bool level = SignalToGpioLevel(pin, signal);
    gpio_set_level(gpio_pin, level ? 1 : 0);
    return true;
  }

  /**
   * @brief Read GPIO pin state
   */
  bool GpioRead(tmc5160::TMC5160CtrlPin pin,
                tmc5160::GpioSignal &signal) noexcept {
    // TMC5160 doesn't have input pins for reading, but implement for
    // completeness
    return false;
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

private:
  spi_host_device_t host_;
  gpio_num_t mosi_pin_;
  gpio_num_t miso_pin_;
  gpio_num_t sclk_pin_;
  gpio_num_t cs_pin_;
  gpio_num_t en_pin_;
  gpio_num_t dir_pin_;
  gpio_num_t step_pin_;
  uint32_t clock_speed_hz_;
  spi_device_handle_t device_handle_;
  bool initialized_;

  bool configureGpioPins() noexcept {
    // GPIO pins are configured in initialize()
    return true;
  }
};
