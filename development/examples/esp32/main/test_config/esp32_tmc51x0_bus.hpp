/**
 * @file esp32_tmc51x0_bus.hpp
 * @brief ESP32-specific communication interfaces for TMC51x0 using SPI and UART (TMC5130 & TMC51x0)
 *
 * This file provides ESP32-specific implementations of the TMC51x0
 * communication interfaces using ESP-IDF SPI and UART drivers.
 * Supports both TMC5130 and TMC51x0 chips.
 *
 * Pin Configuration Support:
 * - Supports all TMC51x0 control pins (EN, DIR, STEP, REFL_STEP, REFR_DIR, DIAG0, DIAG1, ENCA, ENCB, ENCN, CLK)
 * - Supports user-defined GPIO pins for custom board configurations
 * - Diagnostic pins (DIAG0, DIAG1) can be read via GpioRead()
 * - Reference switch pins (REFL_STEP, REFR_DIR) can be read/written depending on mode
 * - Encoder pins (ENCA, ENCB, ENCN) can be read depending on mode
 * - CLK pin can be configured for external clock input (PWM support via user implementation)
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include "tmc51x0_comm_interface.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

static const char* BUS_TAG = "TMC51x0_Bus";

/**
 * @brief Complete ESP32 SPI bus and TMC51x0 pin configuration structure
 *
 * This structure extends TMC51x0PinConfig to include SPI bus pins, providing
 * a single configuration structure for all GPIO pins used by the ESP32 SPI
 * communication interface.
 *
 * This allows users to define all pin assignments in one place, making it
 * easier to manage and configure the hardware setup.
 *
 * @note SPI pins are required for SPI communication.
 * @note TMC51x0 control pins are optional and depend on the operating mode.
 * @note This is ESP32-specific and should not be in the core driver interface.
 */
struct Esp32SpiPinConfig {
  // SPI bus pins (required for SPI communication)
  int spi_mosi{-1}; ///< SPI MOSI pin (Master Out, Slave In)
  int spi_miso{-1}; ///< SPI MISO pin (Master In, Slave Out)
  int spi_sclk{-1}; ///< SPI clock pin (SCLK)
  int spi_cs{-1};   ///< SPI chip select pin (CS)

  // TMC51x0 control pins (from TMC51x0PinConfig)
  tmc51x0::TMC51x0PinConfig tmc51x0_pins; ///< TMC51x0 control pin configuration

  /**
   * @brief Default constructor - all pins unmapped (-1)
   */
  Esp32SpiPinConfig() = default;

  /**
   * @brief Constructor with SPI pins and basic TMC51x0 pins
   * @param mosi SPI MOSI pin
   * @param miso SPI MISO pin
   * @param sclk SPI clock pin
   * @param cs SPI chip select pin
   * @param en TMC5160 EN pin (required)
   * @param dir TMC5160 DIR pin (optional, -1 if not used)
   * @param step TMC5160 STEP pin (optional, -1 if not used)
   */
  Esp32SpiPinConfig(int mosi, int miso, int sclk, int cs, int en, int dir = -1, int step = -1) noexcept
      : spi_mosi(mosi), spi_miso(miso), spi_sclk(sclk), spi_cs(cs), tmc51x0_pins(en, dir, step) {}

  /**
   * @brief Constructor with SPI pins and full TMC5160 pin config
   * @param mosi SPI MOSI pin
   * @param miso SPI MISO pin
   * @param sclk SPI clock pin
   * @param cs SPI chip select pin
   * @param tmc_pins TMC5160 pin configuration structure
   */
  Esp32SpiPinConfig(int mosi, int miso, int sclk, int cs, const tmc51x0::TMC51x0PinConfig& tmc_pins) noexcept
      : spi_mosi(mosi), spi_miso(miso), spi_sclk(sclk), spi_cs(cs), tmc51x0_pins(tmc_pins) {}
};

/**
 * @brief ESP32 SPI implementation of TMC51x0 communication interface
 *
 * This class provides SPI communication for the TMC51x0 using ESP-IDF SPI
 * driver with full GPIO pin support.
 *
 * Pin Configuration Example (ESP32-C6):
 * ```cpp
 * Esp32SPI spi(SPI2_HOST,
 *              GPIO_NUM_6,   // MOSI
 *              GPIO_NUM_12,  // MISO
 *              GPIO_NUM_5,   // SCLK
 *              GPIO_NUM_18,  // CS
 *              GPIO_NUM_11,  // EN (DRV_EN)
 *              -1,            // DIR (not used in SPI mode with internal ramp generator)
 *              -1,            // STEP (not used in SPI mode with internal ramp generator)
 *              4000000);      // 4 MHz SPI clock
 *
 * // Configure additional pins
 * spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG0, GPIO_NUM_XX); // DIAG0 pin
 * spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG1, GPIO_NUM_XX); // DIAG1 pin
 * spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::CLK, GPIO_NUM_10);   // CLK pin (GPIO10)
 * ```
 */
class Esp32SPI : public tmc51x0::SpiCommInterface<Esp32SPI> {
public:
  /**
   * @brief Construct ESP32 SPI communication interface with complete pin configuration
   * @param host SPI host device (e.g., SPI2_HOST)
   * @param pin_config Complete pin configuration including SPI pins and TMC51x0 control pins
   * @param clock_speed_hz SPI clock speed in Hz (max 4 MHz recommended)
   * @param active_levels Pin active level configuration (optional, uses datasheet defaults if not provided)
   *
   * This is the recommended constructor as it allows all GPIO pins (SPI + TMC51x0 control)
   * to be configured in a single structure, making it easier to manage pin assignments.
   *
   * @note Compound pins are automatically handled - if you specify dir_pin, ref_right_pin
   *       is automatically mapped to the same GPIO (and vice versa). Same for step_pin/ref_left_pin,
   *       enc_a_pin/dc_in_pin, enc_b_pin/dc_en_pin, and enc_n_pin/dc_out_pin.
   * @note EN pin (DRV_ENN) is active LOW to enable the power stage by default.
   * @note Users can override active levels by creating a PinActiveLevels struct, modifying
   *       specific pins, and passing it to this constructor.
   *
   * Example:
   * @code
   * // Use defaults
   * Esp32SPI spi(SPI2_HOST, pin_config);
   *
   * // Override for custom board
   * tmc51x0::PinActiveLevels levels;
   * levels.en = true; // Board has inverter on EN pin
   * Esp32SPI spi(SPI2_HOST, pin_config, 4000000, levels);
   * @endcode
   */
  Esp32SPI(spi_host_device_t host, const Esp32SpiPinConfig& pin_config,
           uint32_t clock_speed_hz = 4000000,
           const tmc51x0::PinActiveLevels& active_levels = tmc51x0::PinActiveLevels{}) noexcept
      : SpiCommInterface(), // Active level management handled in this derived class
        active_levels_(active_levels), // Store the struct directly
        host_(host), mosi_pin_(static_cast<gpio_num_t>(pin_config.spi_mosi)),
        miso_pin_(static_cast<gpio_num_t>(pin_config.spi_miso)),
        sclk_pin_(static_cast<gpio_num_t>(pin_config.spi_sclk)),
        cs_pin_(static_cast<gpio_num_t>(pin_config.spi_cs)),
        en_pin_(static_cast<gpio_num_t>(pin_config.tmc51x0_pins.en_pin)),
        dir_pin_(static_cast<gpio_num_t>(pin_config.tmc51x0_pins.dir_pin != -1
                                             ? pin_config.tmc51x0_pins.dir_pin
                                             : pin_config.tmc51x0_pins.ref_right_pin)),
        step_pin_(static_cast<gpio_num_t>(pin_config.tmc51x0_pins.step_pin != -1
                                              ? pin_config.tmc51x0_pins.step_pin
                                              : pin_config.tmc51x0_pins.ref_left_pin)),
        clock_speed_hz_(clock_speed_hz), device_handle_(nullptr), initialized_(false) {
    // Initialize pin mapping array (all pins unmapped by default, use -1)
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    for (size_t i = 0; i < sizeof(pin_mapping_) / sizeof(pin_mapping_[0]); ++i) {
      pin_mapping_[i] = UNMAPPED_PIN;
    }

    // Apply pin configuration (handles compound pins automatically)
    ApplyPinConfig(pin_config.tmc51x0_pins);
  }

private:
  /**
   * @brief Pin active level configuration storage
   *
   * Stores the PinActiveLevels struct which defines the physical GPIO level
   * (HIGH or LOW) that corresponds to the ACTIVE state for each TMC51x0 control pin.
   * This struct is initialized from the constructor parameter (or defaults).
   */
  tmc51x0::PinActiveLevels active_levels_;

  /**
   * @brief Convert signal state to physical GPIO level
   * @param pin The TMC51x0 control pin
   * @param signal The signal state (ACTIVE or INACTIVE)
   * @return Physical GPIO level (true=HIGH, false=LOW)
   */
  [[nodiscard]] bool SignalToGpioLevel(tmc51x0::TMC51x0CtrlPin pin, tmc51x0::GpioSignal signal) const noexcept {
    bool active_level = active_levels_.GetActiveLevel(pin);
    return (signal == tmc51x0::GpioSignal::ACTIVE) ? active_level : !active_level;
  }

  /**
   * @brief Convert physical GPIO level to signal state
   * @param pin The TMC51x0 control pin
   * @param gpio_level Physical GPIO level (true=HIGH, false=LOW)
   * @return Signal state (ACTIVE or INACTIVE)
   */
  [[nodiscard]] tmc51x0::GpioSignal GpioLevelToSignal(tmc51x0::TMC51x0CtrlPin pin, bool gpio_level) const noexcept {
    bool active_level = active_levels_.GetActiveLevel(pin);
    return (gpio_level == active_level) ? tmc51x0::GpioSignal::ACTIVE : tmc51x0::GpioSignal::INACTIVE;
  }

public:
  /**
   * @brief Configure pin active levels from PinActiveLevels struct
   * @param active_levels Pin active level configuration structure
   * 
   * Allows users to update active levels after construction, useful for
   * runtime configuration or if board setup changes.
   * 
   * Example:
   * @code
   * tmc51x0::PinActiveLevels levels;
   * levels.en = true; // Override EN pin active level
   * spi.ConfigureActiveLevels(levels);
   * @endcode
   */
  void ConfigureActiveLevels(const tmc51x0::PinActiveLevels& active_levels) noexcept {
    active_levels_ = active_levels; // Simply copy the struct
  }

  /**
   * @brief Get current active level configuration
   * @return PinActiveLevels struct with current active level settings
   */
  [[nodiscard]] const tmc51x0::PinActiveLevels& GetActiveLevels() const noexcept {
    return active_levels_; // Return reference to stored struct
  }

  /**
   * @brief Construct ESP32 SPI communication interface with separate SPI pins and TMC51x0 pin config
   * @param host SPI host device (e.g., SPI2_HOST)
   * @param mosi_pin MOSI GPIO pin
   * @param miso_pin MISO GPIO pin
   * @param sclk_pin SCLK GPIO pin
   * @param cs_pin CS GPIO pin
   * @param pin_config TMC51x0 pin configuration structure (handles compound pins automatically)
   * @param clock_speed_hz SPI clock speed in Hz (max 4 MHz recommended)
   * @param active_levels Pin active level configuration (optional, uses datasheet defaults if not provided)
   *
   * @note Compound pins are automatically handled - if you specify dir_pin, ref_right_pin
   *       is automatically mapped to the same GPIO (and vice versa). Same for step_pin/ref_left_pin,
   *       enc_a_pin/dc_in_pin, enc_b_pin/dc_en_pin, and enc_n_pin/dc_out_pin.
   * @note EN pin (DRV_ENN) is active LOW to enable the power stage by default.
   * @note This constructor is provided for backward compatibility. Consider using the
   *       constructor with Esp32SpiPinConfig for a more unified configuration.
   */
  Esp32SPI(spi_host_device_t host, gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin,
           const tmc51x0::TMC51x0PinConfig& pin_config, uint32_t clock_speed_hz = 4000000,
           const tmc51x0::PinActiveLevels& active_levels = tmc51x0::PinActiveLevels{}) noexcept
      : SpiCommInterface(), // Active level management handled in this derived class
        active_levels_(active_levels), // Store the struct directly
        host_(host), mosi_pin_(mosi_pin), miso_pin_(miso_pin), sclk_pin_(sclk_pin), cs_pin_(cs_pin),
        en_pin_(static_cast<gpio_num_t>(pin_config.en_pin)),
        dir_pin_(static_cast<gpio_num_t>(pin_config.dir_pin != -1 ? pin_config.dir_pin : pin_config.ref_right_pin)),
        step_pin_(static_cast<gpio_num_t>(pin_config.step_pin != -1 ? pin_config.step_pin : pin_config.ref_left_pin)),
        clock_speed_hz_(clock_speed_hz), device_handle_(nullptr), initialized_(false) {
    // Initialize pin mapping array (all pins unmapped by default, use -1)
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    for (size_t i = 0; i < sizeof(pin_mapping_) / sizeof(pin_mapping_[0]); ++i) {
      pin_mapping_[i] = UNMAPPED_PIN;
    }

    // Apply pin configuration (handles compound pins automatically)
    ApplyPinConfig(pin_config);
  }

  /**
   * @brief Construct ESP32 SPI communication interface (legacy constructor for backward compatibility)
   * @param host SPI host device (e.g., SPI2_HOST)
   * @param mosi_pin MOSI GPIO pin
   * @param miso_pin MISO GPIO pin
   * @param sclk_pin SCLK GPIO pin
   * @param cs_pin CS GPIO pin
   * @param en_pin EN control pin (DRV_ENN, active LOW enables power stage)
   * @param dir_pin DIR control pin (REFR_DIR, optional - used in external step/dir mode, use -1 if not connected)
   * @param step_pin STEP control pin (REFL_STEP, optional - used in external step/dir mode, use -1 if not connected)
   * @param clock_speed_hz SPI clock speed in Hz (max 4 MHz recommended)
   *
   * @param active_levels Pin active level configuration (optional, uses datasheet defaults if not provided)
   *
   * @note For SPI mode with internal ramp generator (SD_MODE=0), DIR and STEP pins
   *       are not used. Pass -1 if not connected.
   * @note EN pin (DRV_ENN) is active LOW to enable the power stage by default.
   * @deprecated Use constructor with TMC51x0PinConfig struct for better pin management
   */
  Esp32SPI(spi_host_device_t host, gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin,
           gpio_num_t en_pin, gpio_num_t dir_pin = static_cast<gpio_num_t>(-1),
           gpio_num_t step_pin = static_cast<gpio_num_t>(-1), uint32_t clock_speed_hz = 4000000,
           const tmc51x0::PinActiveLevels& active_levels = tmc51x0::PinActiveLevels{}) noexcept
      : SpiCommInterface(), // Active level management handled in this derived class
        active_levels_(active_levels), // Store the struct directly
        host_(host), mosi_pin_(mosi_pin), miso_pin_(miso_pin), sclk_pin_(sclk_pin), cs_pin_(cs_pin), en_pin_(en_pin),
        dir_pin_(dir_pin), step_pin_(step_pin), clock_speed_hz_(clock_speed_hz), device_handle_(nullptr),
        initialized_(false) {
    // Initialize pin mapping array (all pins unmapped by default, use -1)
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    for (size_t i = 0; i < sizeof(pin_mapping_) / sizeof(pin_mapping_[0]); ++i) {
      pin_mapping_[i] = UNMAPPED_PIN;
    }

    // Set default pin mappings
    pin_mapping_[static_cast<size_t>(tmc51x0::TMC51x0CtrlPin::EN)] = en_pin;
    if (dir_pin != UNMAPPED_PIN) {
      pin_mapping_[static_cast<size_t>(tmc51x0::TMC51x0CtrlPin::DIR)] = dir_pin;
      pin_mapping_[static_cast<size_t>(tmc51x0::TMC51x0CtrlPin::REFR_DIR)] = dir_pin; // Same physical pin
    }
    if (step_pin != UNMAPPED_PIN) {
      pin_mapping_[static_cast<size_t>(tmc51x0::TMC51x0CtrlPin::STEP)] = step_pin;
      pin_mapping_[static_cast<size_t>(tmc51x0::TMC51x0CtrlPin::REFL_STEP)] = step_pin; // Same physical pin
    }
  }

  /**
   * @brief Destructor - cleans up SPI resources
   */
  ~Esp32SPI() noexcept {
    Deinitialize();
  }

  /**
   * @brief Apply pin configuration structure (handles compound pins automatically)
   * @param pin_config Pin configuration structure
   * @return true if configuration was applied successfully
   *
   * This method automatically handles compound pins (pins that share the same physical GPIO):
   * - If dir_pin is set, ref_right_pin is automatically mapped to the same GPIO
   * - If step_pin is set, ref_left_pin is automatically mapped to the same GPIO
   * - If enc_a_pin is set, dc_in_pin is automatically mapped to the same GPIO
   * - If enc_b_pin is set, dc_en_pin is automatically mapped to the same GPIO
   * - If enc_n_pin is set, dc_out_pin is automatically mapped to the same GPIO
   *
   * You can override individual mappings using SetPinMapping() if needed.
   */
  bool ApplyPinConfig(const tmc51x0::TMC51x0PinConfig& pin_config) noexcept {
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);

    // Basic control pins
    if (pin_config.en_pin != -1) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::EN, static_cast<gpio_num_t>(pin_config.en_pin));
    }

    // DIR/REFR_DIR compound pin (pin 18)
    gpio_num_t dir_gpio = UNMAPPED_PIN;
    if (pin_config.dir_pin != -1) {
      dir_gpio = static_cast<gpio_num_t>(pin_config.dir_pin);
    } else if (pin_config.ref_right_pin != -1) {
      dir_gpio = static_cast<gpio_num_t>(pin_config.ref_right_pin);
    }
    if (dir_gpio != UNMAPPED_PIN) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIR, dir_gpio);
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::REFR_DIR, dir_gpio); // Same physical pin
    }

    // STEP/REFL_STEP compound pin (pin 17)
    gpio_num_t step_gpio = UNMAPPED_PIN;
    if (pin_config.step_pin != -1) {
      step_gpio = static_cast<gpio_num_t>(pin_config.step_pin);
    } else if (pin_config.ref_left_pin != -1) {
      step_gpio = static_cast<gpio_num_t>(pin_config.ref_left_pin);
    }
    if (step_gpio != UNMAPPED_PIN) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::STEP, step_gpio);
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::REFL_STEP, step_gpio); // Same physical pin
    }

    // Diagnostic pins
    if (pin_config.diag0_pin != -1) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG0, static_cast<gpio_num_t>(pin_config.diag0_pin));
    }
    if (pin_config.diag1_pin != -1) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG1, static_cast<gpio_num_t>(pin_config.diag1_pin));
    }

    // ENCA/DCIN compound pin (pin 24)
    gpio_num_t enca_gpio = UNMAPPED_PIN;
    if (pin_config.enc_a_pin != -1) {
      enca_gpio = static_cast<gpio_num_t>(pin_config.enc_a_pin);
    } else if (pin_config.dc_in_pin != -1) {
      enca_gpio = static_cast<gpio_num_t>(pin_config.dc_in_pin);
    }
    if (enca_gpio != UNMAPPED_PIN) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::ENCA, enca_gpio);
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::DCIN, enca_gpio); // Same physical pin
    }

    // ENCB/DCEN compound pin (pin 23)
    gpio_num_t encb_gpio = UNMAPPED_PIN;
    if (pin_config.enc_b_pin != -1) {
      encb_gpio = static_cast<gpio_num_t>(pin_config.enc_b_pin);
    } else if (pin_config.dc_en_pin != -1) {
      encb_gpio = static_cast<gpio_num_t>(pin_config.dc_en_pin);
    }
    if (encb_gpio != UNMAPPED_PIN) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::ENCB, encb_gpio);
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::DCEN, encb_gpio); // Same physical pin
    }

    // ENCN/DCO compound pin (pin 25)
    gpio_num_t encn_gpio = UNMAPPED_PIN;
    if (pin_config.enc_n_pin != -1) {
      encn_gpio = static_cast<gpio_num_t>(pin_config.enc_n_pin);
    } else if (pin_config.dc_out_pin != -1) {
      encn_gpio = static_cast<gpio_num_t>(pin_config.dc_out_pin);
    }
    if (encn_gpio != UNMAPPED_PIN) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::ENCN, encn_gpio);
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::DCO, encn_gpio); // Same physical pin
    }

    // Clock pin
    if (pin_config.clk_pin != -1) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::CLK, static_cast<gpio_num_t>(pin_config.clk_pin));
    }

    // Mode configuration pins (if available as control pins)
    // ⚠️ WARNING: These are typically hardwired. Only configure if connected to GPIO.
    if (pin_config.spi_mode_pin != -1) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::SPI_MODE, static_cast<gpio_num_t>(pin_config.spi_mode_pin));
    }
    if (pin_config.sd_mode_pin != -1) {
      SetPinMapping(tmc51x0::TMC51x0CtrlPin::SD_MODE, static_cast<gpio_num_t>(pin_config.sd_mode_pin));
    }

    return true;
  }

  /**
   * @brief Set pin mapping for a TMC51x0 control pin
   * @param pin TMC51x0 control pin identifier
   * @param gpio_pin ESP32 GPIO pin number (or -1 to disable/unmap)
   * @return true if pin mapping was set successfully
   *
   * Use this method to configure additional pins like DIAG0, DIAG1, CLK, encoder pins, etc.
   *
   * Example:
   * ```cpp
   * spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG0, GPIO_NUM_XX);
   * spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG1, GPIO_NUM_XX);
   * spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::CLK, GPIO_NUM_10);
   * ```
   */
  bool SetPinMapping(tmc51x0::TMC51x0CtrlPin pin, gpio_num_t gpio_pin) noexcept {
    size_t index = static_cast<size_t>(pin);
    if (index >= sizeof(pin_mapping_) / sizeof(pin_mapping_[0])) {
      return false;
    }
    pin_mapping_[index] = gpio_pin;

    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    // Configure GPIO direction based on pin type
    if (gpio_pin != UNMAPPED_PIN) {
      // Diagnostic pins are outputs (read-only from MCU perspective)
      if (pin == tmc51x0::TMC51x0CtrlPin::DIAG0 || pin == tmc51x0::TMC51x0CtrlPin::DIAG1) {
        gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);  // MCU reads TMC51x0 output
        gpio_set_pull_mode(gpio_pin, GPIO_PULLUP_ONLY); // Diagnostic pins have pullups
      }
      // Encoder pins are inputs (when SD_MODE=0)
      else if (pin == tmc51x0::TMC51x0CtrlPin::ENCA || pin == tmc51x0::TMC51x0CtrlPin::ENCB ||
               pin == tmc51x0::TMC51x0CtrlPin::ENCN) {
        gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(gpio_pin, GPIO_PULLUP_ONLY);
      }
      // DC Step pins (when SD_MODE=1, SPI_MODE=1)
      else if (pin == tmc51x0::TMC51x0CtrlPin::DCEN || pin == tmc51x0::TMC51x0CtrlPin::DCIN) {
        gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT); // DCEN and DCIN are inputs to TMC51x0
      } else if (pin == tmc51x0::TMC51x0CtrlPin::DCO) {
        gpio_set_direction(gpio_pin, GPIO_MODE_INPUT); // DCO is output from TMC51x0
        gpio_set_pull_mode(gpio_pin, GPIO_PULLUP_ONLY);
      }
      // Reference switch pins can be inputs (when used as reference switches, SD_MODE=0)
      else if (pin == tmc51x0::TMC51x0CtrlPin::REFL_STEP || pin == tmc51x0::TMC51x0CtrlPin::REFR_DIR) {
        gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(gpio_pin, GPIO_PULLUP_ONLY); // Typically active LOW
        active_levels_.SetActiveLevel(pin, false);      // Active LOW for reference switches
      }
      // CLK pin can be output (for external clock) or input (for reading clock state)
      else if (pin == tmc51x0::TMC51x0CtrlPin::CLK) {
        // Default to input, user can configure as output if needed for PWM clock
        gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
      }
      // Mode configuration pins (SPI_MODE, SD_MODE) - INPUT by default (read-only)
      // These pins are typically hardwired on dev boards. Only set as OUTPUT if user
      // explicitly calls communication.SetOperatingMode() or GpioSet() to control them.
      else if (pin == tmc51x0::TMC51x0CtrlPin::SPI_MODE || pin == tmc51x0::TMC51x0CtrlPin::SD_MODE) {
        gpio_set_direction(gpio_pin, GPIO_MODE_INPUT); // Input to read chip mode (hardwired on dev boards)
        gpio_set_pull_mode(gpio_pin, GPIO_PULLUP_ONLY); // Pullup for stable reading
      }
      // Other pins default to output
      else {
        gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
      }
    }
    return true;
  }

  /**
   * @brief Get pin mapping for a TMC51x0 control pin
   * @param pin TMC51x0 control pin identifier
   * @return ESP32 GPIO pin number, or -1 if not mapped
   */
  [[nodiscard]] gpio_num_t GetPinMapping(tmc51x0::TMC51x0CtrlPin pin) const noexcept {
    size_t index = static_cast<size_t>(pin);
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    if (index >= sizeof(pin_mapping_) / sizeof(pin_mapping_[0])) {
      return UNMAPPED_PIN;
    }
    return pin_mapping_[index];
  }

  /**
   * @brief Initialize the SPI interface
   * @return Result<void> indicating success or error code
   */
  tmc51x0::Result<void> Initialize() noexcept {
    if (initialized_) {
      return tmc51x0::Result<void>();
    }

    if (spi_mutex_ == nullptr) {
      spi_mutex_ = xSemaphoreCreateMutex();
      if (spi_mutex_ == nullptr) {
        ESP_LOGE(BUS_TAG, "Failed to create SPI mutex");
        return tmc51x0::Result<void>(tmc51x0::ErrorCode::COMM_ERROR);
      }
    }

    // Configure GPIO pins that are mapped
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    if (en_pin_ != UNMAPPED_PIN) {
      gpio_set_direction(en_pin_, GPIO_MODE_OUTPUT);
      // Disable by default using signal abstraction (EN is active LOW, so INACTIVE = HIGH = disabled)
      auto en_result = GpioSet(tmc51x0::TMC51x0CtrlPin::EN, tmc51x0::GpioSignal::INACTIVE);
      if (!en_result.IsOk()) {
        ESP_LOGW(BUS_TAG, "Failed to set EN pin during initialization (non-critical)");
        // Continue anyway - this is not critical for initialization
      }
    }
    if (dir_pin_ != UNMAPPED_PIN) {
      gpio_set_direction(dir_pin_, GPIO_MODE_OUTPUT);
      // Set DIR to inactive by default using signal abstraction
      auto dir_result = GpioSet(tmc51x0::TMC51x0CtrlPin::DIR, tmc51x0::GpioSignal::INACTIVE);
      if (!dir_result.IsOk()) {
        ESP_LOGW(BUS_TAG, "Failed to set DIR pin during initialization (non-critical)");
        // Continue anyway - this is not critical for initialization
      }
    }
    if (step_pin_ != UNMAPPED_PIN) {
      gpio_set_direction(step_pin_, GPIO_MODE_OUTPUT);
      // Set STEP to inactive by default using signal abstraction
      auto step_result = GpioSet(tmc51x0::TMC51x0CtrlPin::STEP, tmc51x0::GpioSignal::INACTIVE);
      if (!step_result.IsOk()) {
        ESP_LOGW(BUS_TAG, "Failed to set STEP pin during initialization (non-critical)");
        // Continue anyway - this is not critical for initialization
      }
    }

    // Configure SPI bus
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = mosi_pin_;
    bus_config.miso_io_num = miso_pin_;
    bus_config.sclk_io_num = sclk_pin_;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    // Size large enough for the core driver's scratch requirement:
    // (TMC51X0_SPI_MAX_CHAIN_DEVICES + 2) * 5 bytes (40 bits per device)
    const int max_transfer_sz_bytes =
        static_cast<int>((TMC51X0_SPI_MAX_CHAIN_DEVICES + 2U) * 5U);
    bus_config.max_transfer_sz = max_transfer_sz_bytes;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t ret = spi_bus_initialize(host_, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
      ESP_LOGE(BUS_TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::COMM_ERROR);
    }

    // Configure SPI device (Mode 3: CPOL=1, CPHA=1)
    spi_device_interface_config_t dev_config = {};
    dev_config.clock_speed_hz = clock_speed_hz_;
    dev_config.mode = 3; // SPI Mode 3 for TMC51x0
    dev_config.spics_io_num = cs_pin_;
    dev_config.queue_size = 1;
    dev_config.cs_ena_pretrans = 2;
    dev_config.cs_ena_posttrans = 2;

    ret = spi_bus_add_device(host_, &dev_config, &device_handle_);
    if (ret != ESP_OK) {
      ESP_LOGE(BUS_TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
      spi_bus_free(host_);
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::COMM_ERROR);
    }

    initialized_ = true;
    ESP_LOGI(BUS_TAG, "SPI interface initialized successfully");
    return tmc51x0::Result<void>();
  }

  /**
   * @brief Get communication mode (always SPI for this interface)
   * @return CommMode::SPI
   */
  tmc51x0::CommMode GetMode() const noexcept {
    return tmc51x0::CommMode::SPI;
  }

  /**
   * @brief Ensure SPI interface is initialized, initializing if necessary
   * @return true if initialized (or successfully initialized), false if initialization failed
   * 
   * This is a convenience method that checks initialization status and calls
   * Initialize() if needed. Useful for lazy initialization patterns.
   */
  bool EnsureInitialized() noexcept {
    if (initialized_) {
      return true;
    }
    auto result = Initialize();
    return result.IsOk();
  }

  /**
   * @brief Deinitialize the SPI interface
   * @return Result<void> indicating success or error code
   */
  tmc51x0::Result<void> Deinitialize() noexcept {
    if (!initialized_) {
      return tmc51x0::Result<void>();
    }

    if (device_handle_) {
      esp_err_t ret = spi_bus_remove_device(device_handle_);
      if (ret != ESP_OK) {
        ESP_LOGE(BUS_TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        return tmc51x0::Result<void>(tmc51x0::ErrorCode::COMM_ERROR);
      }
      device_handle_ = nullptr;
    }

    esp_err_t ret = spi_bus_free(host_);
    if (ret != ESP_OK) {
      ESP_LOGE(BUS_TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::COMM_ERROR);
    }

    initialized_ = false;
    ESP_LOGI(BUS_TAG, "SPI interface deinitialized");

    if (spi_mutex_ != nullptr) {
      vSemaphoreDelete(spi_mutex_);
      spi_mutex_ = nullptr;
    }
    return tmc51x0::Result<void>();
  }

  /**
   * @brief Perform SPI transfer
   * @param tx Transmit buffer
   * @param rx Receive buffer
   * @param length Number of bytes to transfer
   * @return true if successful, false otherwise
   *
   * ## Concurrency / RTOS Notes (Important)
   * - **Single-task assumption (current examples)**: This implementation assumes the
   *   driver is accessed from a single task. If you call into the driver from multiple
   *   tasks concurrently, you must add a mutex around `spi_device_transmit()` (or provide
   *   external serialization).
   * - **Not ISR-safe**: `spi_device_transmit()` may block; do not call this from an ISR.
   * - **Why serialization matters**: The core SPI comm layer uses per-instance scratch
   *   buffers and expects SPI transfers to be serialized.
   */
  tmc51x0::Result<void> SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) noexcept {
    // Caller must ensure Initialize() (or EnsureInitialized()) was invoked once before use.
    if (!initialized_ || !device_handle_) {
      ESP_LOGE(BUS_TAG, "SPI interface not initialized");
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::COMM_ERROR);
    }

    if (spi_mutex_ != nullptr) {
      xSemaphoreTake(spi_mutex_, portMAX_DELAY);
    }

    spi_transaction_t trans = {};
    trans.length = length * 8;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;

    esp_err_t ret = spi_device_transmit(device_handle_, &trans);

    if (spi_mutex_ != nullptr) {
      xSemaphoreGive(spi_mutex_);
    }
    if (ret != ESP_OK) {
      ESP_LOGE(BUS_TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::COMM_ERROR);
    }

    return tmc51x0::Result<void>();
  }

  /**
   * @brief Set GPIO pin state
   * @param pin The TMC51x0 control pin to control
   * @param signal The desired signal state (ACTIVE or INACTIVE)
   * @return true if the GPIO was set successfully, false otherwise
   *
   * Supports all TMC51x0 control pins: EN, DIR, STEP, REFL_STEP, REFR_DIR,
   * ENCA, ENCB, ENCN, DCEN, DCIN, DCO, CLK.
   *
   * @note Diagnostic pins (DIAG0, DIAG1) and DCO are read-only and cannot be set.
   * @note Encoder pins (ENCA, ENCB, ENCN) are read-only when used as encoder inputs.
   * @note Pin must be mapped using SetPinMapping() before use.
   */
  tmc51x0::Result<void> GpioSet(tmc51x0::TMC51x0CtrlPin pin, tmc51x0::GpioSignal signal) noexcept {
    // Diagnostic pins and DCO are read-only (outputs from TMC51x0)
    if (pin == tmc51x0::TMC51x0CtrlPin::DIAG0 || pin == tmc51x0::TMC51x0CtrlPin::DIAG1 ||
        pin == tmc51x0::TMC51x0CtrlPin::DCO) {
      ESP_LOGW(BUS_TAG, "Pin is read-only (output from TMC51x0)");
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::INVALID_VALUE);
    }
    // Encoder pins are read-only when used as encoder inputs (SD_MODE=0)
    if (pin == tmc51x0::TMC51x0CtrlPin::ENCA || pin == tmc51x0::TMC51x0CtrlPin::ENCB ||
        pin == tmc51x0::TMC51x0CtrlPin::ENCN) {
      ESP_LOGW(BUS_TAG, "Encoder pins are read-only (use DCEN/DCIN for DC Step mode)");
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::INVALID_VALUE);
    }

    gpio_num_t gpio_pin = GetPinMapping(pin);
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    if (gpio_pin == UNMAPPED_PIN) {
      ESP_LOGW(BUS_TAG, "Pin not mapped: %d", static_cast<int>(pin));
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::INVALID_VALUE);
    }

    // If user is trying to set SPI_MODE or SD_MODE pins, configure them as OUTPUT
    // (they default to INPUT for read-only mode on hardwired dev boards)
    if (pin == tmc51x0::TMC51x0CtrlPin::SPI_MODE || pin == tmc51x0::TMC51x0CtrlPin::SD_MODE) {
      gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
    }

    bool level = SignalToGpioLevel(pin, signal);
    gpio_set_level(gpio_pin, level ? 1 : 0);
    return tmc51x0::Result<void>();
  }

  /**
   * @brief Read GPIO pin state
   * @param pin The TMC51x0 control pin to read
   * @return Result<GpioSignal> containing the signal state, or error code
   *
   * Supports reading diagnostic pins (DIAG0, DIAG1), reference switch pins
   * (REFL_STEP, REFR_DIR), encoder pins (ENCA, ENCB, ENCN), and CLK pin.
   *
   * @note Pin must be mapped using SetPinMapping() before use.
   */
  tmc51x0::Result<tmc51x0::GpioSignal> GpioRead(tmc51x0::TMC51x0CtrlPin pin) noexcept {
    gpio_num_t gpio_pin = GetPinMapping(pin);
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    if (gpio_pin == UNMAPPED_PIN) {
      ESP_LOGW(BUS_TAG, "Pin not mapped: %d", static_cast<int>(pin));
      return tmc51x0::Result<tmc51x0::GpioSignal>(tmc51x0::ErrorCode::INVALID_VALUE);
    }

    int level = gpio_get_level(gpio_pin);
    tmc51x0::GpioSignal signal = GpioLevelToSignal(pin, level != 0);
    return tmc51x0::Result<tmc51x0::GpioSignal>(signal);
  }

  /**
   * @brief Debug logging
   */
  void DebugLog(int level, const char* tag, const char* format, va_list args) noexcept {
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

    // NOTE:
    // - `esp_log_writev()` does NOT add the "I (time) TAG:" prefix unless you pass a format
    //   string created via ESP-IDF log macros.
    // - To ensure logs show the standard ESP-IDF prefix, we format into a buffer and re-log
    //   via ESP_LOG_LEVEL().
    //
    // This is used by the core driver via TMC51X0_LOG_DEBUG() -> CommInterface::LogDebug().
    char msg[512];
    va_list args_copy;
    va_copy(args_copy, args);
    vsnprintf(msg, sizeof(msg), format, args_copy);
    va_end(args_copy);

    // Strip trailing newlines to avoid double-spacing (ESP_LOG_* adds its own newline).
    size_t len = strlen(msg);
    while (len > 0 && (msg[len - 1] == '\n' || msg[len - 1] == '\r')) {
      msg[len - 1] = '\0';
      --len;
    }

    ESP_LOG_LEVEL(esp_level, tag, "%s", msg);
  }

  /**
   * @brief Delay milliseconds
   */
  void DelayMs(uint32_t ms) noexcept {
    vTaskDelay(pdMS_TO_TICKS(ms));
  }

  /**
   * @brief Delay microseconds
   */
  void DelayUs(uint32_t us) noexcept {
    esp_rom_delay_us(us);
  }

  /**
   * @brief Set external clock frequency on CLK pin
   * @param frequency_hz Desired clock frequency in Hz (0 = use internal clock, set CLK pin to GND)
   * @return true if clock was configured successfully, false if not supported or failed
   *
   * **Internal Clock Mode (frequency_hz = 0):**
   * - Sets CLK pin to OUTPUT mode and drives it LOW (GND)
   * - Enables the internal 12 MHz oscillator
   * - Returns true if CLK pin was successfully configured
   * - Returns false if CLK pin is not mapped
   *
   * **External Clock Mode (frequency_hz > 0):**
   * - Currently not implemented (would require PWM generation)
   * - Returns false to indicate external clock must be provided by external hardware
   * - User should provide external clock signal on CLK pin
   *
   * @note CLK pin must be mapped using SetPinMapping() before calling this function.
   * @note For internal clock, the driver will use 12 MHz for all timing calculations.
   * @note For external clock, the actual frequency must match the frequency_hz parameter.
   */
  tmc51x0::Result<void> SetClkFreq(uint32_t frequency_hz) noexcept {
    gpio_num_t clk_pin = GetPinMapping(tmc51x0::TMC51x0CtrlPin::CLK);
    constexpr gpio_num_t UNMAPPED_PIN = static_cast<gpio_num_t>(-1);
    
    if (clk_pin == UNMAPPED_PIN) {
      // If CLK is hard-wired to GND for internal osc, treat as OK for frequency_hz == 0.
      if (frequency_hz == 0) {
        return tmc51x0::Result<void>();
      }
      ESP_LOGW(BUS_TAG, "CLK pin not mapped; external clock request unsupported");
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::UNSUPPORTED);
    }

    if (frequency_hz == 0) {
      // Internal clock mode: Set CLK pin to GND (LOW)
      gpio_set_direction(clk_pin, GPIO_MODE_OUTPUT);
      gpio_set_level(clk_pin, 0); // Drive LOW (GND) for internal oscillator
      ESP_LOGI(BUS_TAG, "CLK pin set to GND (internal 12 MHz oscillator enabled)");
      return tmc51x0::Result<void>();
    } else {
      // External clock mode: Not implemented via PWM
      // User must provide external clock signal on CLK pin
      ESP_LOGW(BUS_TAG, "External clock generation not implemented. Provide external clock signal on CLK pin (frequency: %u Hz)", frequency_hz);
      return tmc51x0::Result<void>(tmc51x0::ErrorCode::UNSUPPORTED);
    }
  }

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
  SemaphoreHandle_t spi_mutex_{nullptr};
  bool initialized_;

  /**
   * @brief Pin mapping array: maps TMC51x0CtrlPin enum to ESP32 GPIO numbers
   *
   * Array indices correspond to TMC51x0CtrlPin enum values.
   * -1 indicates the pin is not mapped.
   */
  gpio_num_t pin_mapping_[16]{}; // Updated to support all pin types (16 pins: EN through SD_MODE)

  bool configureGpioPins() noexcept {
    // GPIO pins are configured in Initialize() and SetPinMapping()
    return true;
  }
};
