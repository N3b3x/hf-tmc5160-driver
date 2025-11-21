/**
 * @file esp32_tmc5160_bus_config.hpp
 * @brief ESP32 GPIO pin configuration for TMC5160 driver tests
 *
 * This file defines the standard GPIO pin assignments used across all
 * TMC5160 comprehensive test suites. All tests should use these pin
 * definitions for consistency.
 *
 * This file provides both individual pin constants and a complete
 * Esp32SpiPinConfig structure that can be used directly with the
 * Esp32SPI constructor.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#ifndef ESP32_TMC5160_BUS_CONFIG_HPP
#define ESP32_TMC5160_BUS_CONFIG_HPP

#include "driver/gpio.h"
#include "../../../inc/tmc5160_comm_interface.hpp"

namespace tmc5160_test_config {

// SPI bus pins
constexpr gpio_num_t SPI_SCK = GPIO_NUM_5;   ///< SPI clock pin
constexpr gpio_num_t SPI_MOSI = GPIO_NUM_6;  ///< SPI MOSI (master out, slave in)
constexpr gpio_num_t SPI_MISO = GPIO_NUM_2;  ///< SPI MISO (master in, slave out)
constexpr gpio_num_t SPI_CS = GPIO_NUM_18;   ///< SPI chip select pin

// TMC5160 control pins
constexpr gpio_num_t DRV_EN = GPIO_NUM_11;   ///< Driver enable pin (DRV_ENN)
constexpr gpio_num_t CLK = GPIO_NUM_10;      ///< Clock input pin (CLK, pin 12)

// Mode configuration pins (if available as control pins)
constexpr gpio_num_t SPI_MODE_PIN = GPIO_NUM_0;  ///< SPI_MODE pin (pin 22) - GPIO0 if available as control pin
constexpr gpio_num_t SD_MODE_PIN = GPIO_NUM_1;   ///< SD_MODE pin (pin 21) - GPIO1 if available as control pin

// Diagnostic pins
constexpr gpio_num_t DIAG0 = GPIO_NUM_23;    ///< Diagnostic output 0 (DIAG0_SWN, pin 26)
constexpr gpio_num_t DIAG1 = GPIO_NUM_15;    ///< Diagnostic output 1 (DIAG1_SWP, pin 27)

// Optional pins (not used in all tests, set to -1 if not connected)
constexpr gpio_num_t DIR = static_cast<gpio_num_t>(-1);      ///< Direction pin (optional)
constexpr gpio_num_t STEP = static_cast<gpio_num_t>(-1);     ///< Step pin (optional)

// SPI configuration
constexpr uint32_t SPI_CLOCK_SPEED_HZ = 500000;  ///< SPI clock speed 
constexpr spi_host_device_t SPI_HOST = SPI2_HOST; ///< SPI host device

/**
 * @brief Complete pin configuration for all tests
 *
 * This structure includes both SPI pins and TMC5160 control pins,
 * allowing all GPIO assignments to be managed in one place.
 * Use this with the Esp32SPI constructor that takes Esp32SpiPinConfig.
 */
inline tmc5160::Esp32SpiPinConfig GetDefaultPinConfig() noexcept {
  tmc5160::Esp32SpiPinConfig config{};
  
  // SPI pins
  config.spi_mosi = static_cast<int>(SPI_MOSI);
  config.spi_miso = static_cast<int>(SPI_MISO);
  config.spi_sclk = static_cast<int>(SPI_SCK);
  config.spi_cs = static_cast<int>(SPI_CS);
  
  // TMC5160 control pins
  config.tmc5160_pins.en_pin = static_cast<int>(DRV_EN);
  config.tmc5160_pins.clk_pin = static_cast<int>(CLK);
  config.tmc5160_pins.diag0_pin = static_cast<int>(DIAG0);
  config.tmc5160_pins.diag1_pin = static_cast<int>(DIAG1);
  config.tmc5160_pins.dir_pin = static_cast<int>(DIR);
  config.tmc5160_pins.step_pin = static_cast<int>(STEP);
  
  // Mode configuration pins (if available as control pins)
  config.tmc5160_pins.spi_mode_pin = static_cast<int>(SPI_MODE_PIN);
  config.tmc5160_pins.sd_mode_pin = static_cast<int>(SD_MODE_PIN);
  
  return config;
}

} // namespace tmc5160_test_config

#endif // ESP32_TMC5160_BUS_CONFIG_HPP

