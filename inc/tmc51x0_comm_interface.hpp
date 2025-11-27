/**
 * @file tmc51x0_comm_interface.hpp
 * @brief Communication interfaces for TMC51x0 stepper motor driver (TMC5130 & TMC5160) using SPI
 * and UART
 *
 * This file provides comprehensive communication interfaces for the TMC51x0
 * motor driver, supporting both SPI and UART protocols with register read/write
 * operations. It includes GPIO control interfaces and board-agnostic pin
 * management for different hardware implementations. Supports both TMC5130 and TMC5160 chips.
 *
 * ## Compile-Time Configuration
 *
 * **TMC51X0_DISABLE_DEBUG_LOGGING**: Define this macro to completely disable
 * all debug logging throughout the TMC51x0 library. This removes all logging
 * code from the binary at compile time, saving code size and improving
 * performance.
 *
 * Example usage:
 * ```cpp
 * #define TMC51X0_DISABLE_DEBUG_LOGGING
 * #include "tmc51x0_comm_interface.hpp"
 * ```
 *
 * @defgroup TMC51X0_CommInterface Communication Interfaces
 * @brief Core communication interface classes and protocols
 *
 * @defgroup TMC51X0_GPIOControl GPIO Control Interface
 * @brief GPIO pin control and signal management
 *
 * @defgroup TMC51X0_CommTypes Type Definitions
 * @brief Enums and type definitions for communication interfaces
 *
 * ## SPI Protocol
 *
 * TMC51x0 uses 40-bit SPI datagrams per datasheet section 4.1:
 * - Datagram structure: 8-bit address + 32-bit data = 40 bits (5 bytes)
 * - Bit positions: bit 39 (MSB, transmitted first) ... bit 0 (LSB, transmitted last)
 * - Bit 39: W (WRITE_notREAD bit) - 0 for read, 1 for write
 * - Bits 38-32: 7-bit register address
 * - Bits 31-0: 32-bit data (right-aligned)
 *
 * Byte structure (MSB transmitted first):
 * - Byte 0: W bit (bit 7) + 7-bit address (bits 6-0)
 *   - Read access: bit 7 = 0, address in bits 6-0
 *   - Write access: bit 7 = 1, address in bits 6-0 (add 0x80 to address)
 * - Bytes 1-4: 32-bit data (MSB-first, right-aligned)
 * - Bytes 5-7: Unused (optional, can be used for daisy-chaining multiple chips)
 *
 * Response structure (40 bits):
 * - Byte 0: SPI_STATUS (8 bits, bits 39-32) - Status flags
 * - Bytes 1-4: 32-bit data (bits 31-0, MSB-first)
 *
 * Read access behavior (pipelined):
 * - First transfer: Send address with dummy data, receive dummy response
 * - Second transfer: Send address again, receive actual data from previous read
 *
 * Write access behavior:
 * - Send address | 0x80, then 4 bytes of data
 * - Response contains SPI_STATUS + previously written data
 *
 * SPI Mode: Mode 3 (CPOL=1, CPHA=1)
 * Clock: MSB-first
 * Minimum: 40 SCK clock cycles required
 * CSN: Must stay low during entire transaction
 *
 * ## UART Protocol
 *
 * TMC51x0 uses UART single wire interface per datasheet section 5.1.
 * Each byte is transmitted LSB...MSB, highest byte transmitted first.
 *
 * Write Access Datagram (64 bits = 8 bytes total):
 * - Byte 0: Sync nibble (0x05 = 1,0,1,0) + Reserved (0)
 * - Byte 1: NODEADDR (8-bit node address, 0-254)
 * - Byte 2: RW bit (bit 7 = 1 for write) + 7-bit register address
 * - Bytes 3-6: 32-bit data (high byte to low byte, MSB-first)
 * - Byte 7: CRC8 checksum (calculated over bytes 0-6)
 *
 * Read Access Request Datagram (32 bits = 4 bytes total):
 * - Byte 0: Sync nibble (0x05) + Reserved (0)
 * - Byte 1: NODEADDR (8-bit node address)
 * - Byte 2: RW bit (bit 7 = 0 for read) + 7-bit register address
 * - Byte 3: CRC8 checksum (calculated over bytes 0-2)
 *
 * Read Access Reply Datagram (64 bits = 8 bytes total):
 * - Byte 0: Sync nibble (0x05) + Reserved (0)
 * - Byte 1: Master Address (0xFF)
 * - Byte 2: Register Address (0x00)
 * - Bytes 3-6: 32-bit data (high byte to low byte, MSB-first)
 * - Byte 7: CRC8 checksum (calculated over bytes 0-6)
 *
 * CRC8: CRC8-ATM polynomial (0x07), applied LSB to MSB.
 *
 * Baud Rate:
 * - Minimum: 9000 baud
 * - Maximum: fCLK/16
 * - Baud rate is automatically detected from sync frame timing
 *
 * Communication Reset:
 * - Pause time > 63 bit times between start bits resets communication
 * - Recovery time: 12 bit times of bus idle time
 *
 * SENDDELAY:
 * - Programmable delay time after read request before reply
 * - Default: 8 bit times
 * - Multi-node systems: Set SENDDELAY to min. 2 for all nodes
 */

#ifndef TMC51X0_COMM_INTERFACE_HPP
#define TMC51X0_COMM_INTERFACE_HPP

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace tmc51x0 {

/**
 * @brief Compile-time debug logging control for TMC51x0 library
 *
 * Define TMC51X0_DISABLE_DEBUG_LOGGING before including this header to
 * completely disable all debug logging. When disabled, all logDebug() calls are
 * optimized out at compile time, including argument evaluation.
 */
#ifndef TMC51X0_DISABLE_DEBUG_LOGGING
// Debug logging enabled - use actual function call
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage) - Intentional: compile-time logging control
#define TMC51X0_LOG_DEBUG(comm_obj, level, tag, ...) (comm_obj).LogDebug(level, tag, __VA_ARGS__)
#else
// Debug logging disabled - optimize out completely (arguments not evaluated)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage) - Intentional: compile-time logging control
#define TMC51X0_LOG_DEBUG(comm_obj, level, tag, ...) ((void)0)
#endif

/**
 * @brief Supported physical communication modes for TMC51x0
 */
enum class CommMode : uint8_t {
  SPI, ///< SPI (Serial Peripheral Interface) mode - 4-wire synchronous
       ///< communication
  UART ///< UART (Universal Asynchronous Receiver-Transmitter) mode - 2-wire
       ///< asynchronous communication
};

/**
 * @brief TMC51x0 control pin identifiers with board-agnostic naming
 *
 * These pin identifiers abstract the physical pin assignments to provide
 * a consistent interface regardless of board implementation.
 *
 * Pin functions depend on mode configuration (SPI_MODE, SD_MODE):
 * - SPI_MODE=1, SD_MODE=0: Internal ramp generator mode (motion controller)
 *   - REFL_STEP, REFR_DIR: Reference switch inputs
 *   - ENCA, ENCB, ENCN: Encoder inputs
 *   - DIAG0, DIAG1: Diagnostic outputs
 * - SPI_MODE=1, SD_MODE=1: External step/dir mode
 *   - REFL_STEP, REFR_DIR: STEP/DIR inputs (same physical pins)
 *   - DCEN, DCIN, DCO: DC Step control pins (same physical pins as ENCB, ENCA, ENCN)
 *   - DIAG0, DIAG1: Diagnostic outputs
 * - SPI_MODE=0, SD_MODE=0: UART single wire interface mode
 *   - DIAG1_SWP (pin 27): Single wire I/O (positive) - Use SWP alone for single wire UART
 *   - DIAG0_SWN (pin 26): Single wire I/O (negative) - Use SWP+SWN for RS485 differential bus
 *   - SDI_CFG1 (pin 15): Next address input (NAI) for sequential addressing
 *   - SDO_CFG0 (pin 16): Next address output (NAO) for sequential addressing
 *   - CSN_CFG3, SCK_CFG2, ENCB_CFG4, ENCA_CFG5: Configuration inputs (CFG functions)
 *   - Other pins: Not used for communication in UART mode
 *
 * @note Some pins have multiple functions depending on mode. See datasheet
 *       section 2.2 for complete pin function descriptions.
 */
enum class TMC51x0CtrlPin : uint8_t {
  // Basic control pins (always available)
  EN,   ///< Enable pin (DRV_ENN, pin 28) - Active HIGH disables power stage
  DIR,  ///< Direction pin (REFR_DIR, pin 18) - DIR input when SD_MODE=1, Right reference when SD_MODE=0
  STEP, ///< Step pin (REFL_STEP, pin 17) - STEP input when SD_MODE=1, Left reference when SD_MODE=0

  // Reference switch pins (when SD_MODE=0, internal ramp generator mode)
  // Same physical pins as STEP/DIR but used as reference switches
  REFL_STEP, ///< Left reference switch input (REFL_STEP, pin 17) - Used when SD_MODE=0
  REFR_DIR,  ///< Right reference switch input (REFR_DIR, pin 18) - Used when SD_MODE=0

  // Diagnostic output pins / UART pins (mode-dependent)
  DIAG0, ///< Diagnostic output 0 (DIAG0_SWN, pin 26) - Interrupt/STEP output when SD_MODE=0, SPI_MODE=1.
         ///< Single wire I/O (negative) when SD_MODE=0, SPI_MODE=0 (UART mode). Use with DIAG1_SWP for RS485 bus.
  DIAG1, ///< Diagnostic output 1 (DIAG1_SWP, pin 27) - Position-compare/DIR output when SD_MODE=0, SPI_MODE=1.
         ///< Single wire I/O (positive) when SD_MODE=0, SPI_MODE=0 (UART mode). Use alone for single wire UART.

  // Encoder input pins (when SD_MODE=0, internal ramp generator mode)
  // Same physical pins as DC Step pins but used as encoder inputs
  ENCA, ///< Encoder A-channel input (ENCA_DCIN_CFG5, pin 24) - Used when SD_MODE=0
  ENCB, ///< Encoder B-channel input (ENCB_DCEN_CFG4, pin 23) - Used when SD_MODE=0
  ENCN, ///< Encoder N-channel input (ENCN_DCO_CFG6, pin 25) - Used when SD_MODE=0

  // DC Step control pins (when SD_MODE=1, SPI_MODE=1, external step/dir mode)
  // Same physical pins as encoder pins but used for DC Step control
  DCEN, ///< DC Step enable input (ENCB_DCEN_CFG4, pin 23) - Used when SD_MODE=1, SPI_MODE=1
  DCIN, ///< DC Step gating input (ENCA_DCIN_CFG5, pin 24) - Used when SD_MODE=1, SPI_MODE=1
  DCO,  ///< DC Step ready output (ENCN_DCO_CFG6, pin 25) - Used when SD_MODE=1, SPI_MODE=1

  // Clock pin (optional external clock)
  CLK, ///< Clock input (CLK, pin 12) - External clock input (tie to GND for internal clock)

  // Mode configuration pins (if made available as control pins)
  // ⚠️ WARNING: These pins are typically hardwired and read at startup.
  // Only use these if you have connected these pins to GPIO outputs for dynamic control.
  // Changing these pins requires a chip reset to take effect.
  SPI_MODE, ///< SPI/UART mode select (pin 22) - HIGH=SPI, LOW=UART. Typically hardwired.
  SD_MODE   ///< Step/Dir mode select (pin 21) - HIGH=External step/dir, LOW=Internal ramp. Typically hardwired.
};

/**
 * @brief GPIO signal states with board-agnostic naming
 */
enum class GpioSignal : uint8_t {
  INACTIVE = 0, ///< Inactive signal state (logical low)
  ACTIVE = 1    ///< Active signal state (logical high)
};

/**
 * @brief Pin active level configuration structure
 *
 * This structure defines the physical GPIO level (HIGH or LOW) that corresponds
 * to the ACTIVE state for each TMC51x0 control pin. Default values are based
 * on the TMC51x0 datasheet specifications.
 *
 * Users can create an instance of this struct, modify specific pin active levels
 * if their board has inverters, NOT gates, or other logic that changes pin
 * polarity, and pass it to the communication interface constructor.
 *
 * Example usage:
 * @code
 * // Use defaults (datasheet-compliant)
 * tmc51x0::PinActiveLevels active_levels; // Uses all defaults
 *
 * // Override for custom board with inverter on EN pin
 * tmc5160::PinActiveLevels active_levels;
 * active_levels.en = true; // EN pin has inverter, so ACTIVE = HIGH
 *
 * // Pass to constructor
 * Esp32SPI spi(SPI2_HOST, pin_config, 4000000, active_levels);
 * @endcode
 */
struct PinActiveLevels {
  // Basic control pins (per TMC51x0 datasheet)
  bool en{false};  ///< EN pin (DRV_ENN, pin 28): LOW=enable (active LOW)
  bool dir{true};  ///< DIR pin (REFR_DIR, pin 18): HIGH=active (active HIGH)
  bool step{true}; ///< STEP pin (REFL_STEP, pin 17): HIGH=active (active HIGH)

  // Reference switch pins (when SD_MODE=0)
  bool ref_left{false};  ///< REFL_STEP (pin 17): LOW=active (typically active LOW)
  bool ref_right{false}; ///< REFR_DIR (pin 18): LOW=active (typically active LOW)

  // Diagnostic pins (read-only outputs from TMC51x0 - not typically configured)
  bool diag0{true}; ///< DIAG0 (pin 26): Read-only output (default HIGH, but not used for control)
  bool diag1{true}; ///< DIAG1 (pin 27): Read-only output (default HIGH, but not used for control)

  // Encoder pins (when SD_MODE=0, read-only inputs)
  bool enca{true}; ///< ENCA (pin 24): Read-only input (default HIGH, but not used for control)
  bool encb{true}; ///< ENCB (pin 23): Read-only input (default HIGH, but not used for control)
  bool encn{true}; ///< ENCN (pin 25): Read-only input (default HIGH, but not used for control)

  // DC Step pins (when SD_MODE=1, SPI_MODE=1)
  bool dcen{true}; ///< DCEN (pin 23): HIGH=active (active HIGH)
  bool dcin{true}; ///< DCIN (pin 24): HIGH=active (active HIGH)
  bool dco{true};  ///< DCO (pin 25): Read-only output (default HIGH, but not used for control)

  // Clock pin
  bool clk{true}; ///< CLK (pin 12): HIGH=active (active HIGH, if used as output)

  // Mode configuration pins (if available as control pins)
  bool spi_mode{true}; ///< SPI_MODE (pin 22): HIGH=SPI mode (active HIGH)
  bool sd_mode{true};  ///< SD_MODE (pin 21): HIGH=External Step/Dir (active HIGH)

  /**
   * @brief Get active level for a specific pin
   * @param pin The TMC51x0 control pin
   * @return The active level (true=HIGH, false=LOW) for the pin
   */
  [[nodiscard]] bool GetActiveLevel(TMC51x0CtrlPin pin) const noexcept {
    switch (pin) {
    case TMC51x0CtrlPin::EN:
      return en;
    case TMC51x0CtrlPin::DIR:
      return dir;
    case TMC51x0CtrlPin::STEP:
      return step;
    case TMC51x0CtrlPin::REFL_STEP:
      return ref_left;
    case TMC51x0CtrlPin::REFR_DIR:
      return ref_right;
    case TMC51x0CtrlPin::DIAG0:
      return diag0;
    case TMC51x0CtrlPin::DIAG1:
      return diag1;
    case TMC51x0CtrlPin::ENCA:
      return enca;
    case TMC51x0CtrlPin::ENCB:
      return encb;
    case TMC51x0CtrlPin::ENCN:
      return encn;
    case TMC51x0CtrlPin::DCEN:
      return dcen;
    case TMC51x0CtrlPin::DCIN:
      return dcin;
    case TMC51x0CtrlPin::DCO:
      return dco;
    case TMC51x0CtrlPin::CLK:
      return clk;
    case TMC51x0CtrlPin::SPI_MODE:
      return spi_mode;
    case TMC51x0CtrlPin::SD_MODE:
      return sd_mode;
    default:
      return true; // Default to HIGH
    }
  }

  /**
   * @brief Set active level for a specific pin
   * @param pin The TMC51x0 control pin
   * @param active_level The active level (true=HIGH, false=LOW)
   */
  void SetActiveLevel(TMC51x0CtrlPin pin, bool active_level) noexcept {
    switch (pin) {
    case TMC51x0CtrlPin::EN:
      en = active_level;
      break;
    case TMC51x0CtrlPin::DIR:
      dir = active_level;
      break;
    case TMC51x0CtrlPin::STEP:
      step = active_level;
      break;
    case TMC51x0CtrlPin::REFL_STEP:
      ref_left = active_level;
      break;
    case TMC51x0CtrlPin::REFR_DIR:
      ref_right = active_level;
      break;
    case TMC51x0CtrlPin::DIAG0:
      diag0 = active_level;
      break;
    case TMC51x0CtrlPin::DIAG1:
      diag1 = active_level;
      break;
    case TMC51x0CtrlPin::ENCA:
      enca = active_level;
      break;
    case TMC51x0CtrlPin::ENCB:
      encb = active_level;
      break;
    case TMC51x0CtrlPin::ENCN:
      encn = active_level;
      break;
    case TMC51x0CtrlPin::DCEN:
      dcen = active_level;
      break;
    case TMC51x0CtrlPin::DCIN:
      dcin = active_level;
      break;
    case TMC51x0CtrlPin::DCO:
      dco = active_level;
      break;
    case TMC51x0CtrlPin::CLK:
      clk = active_level;
      break;
    case TMC51x0CtrlPin::SPI_MODE:
      spi_mode = active_level;
      break;
    case TMC51x0CtrlPin::SD_MODE:
      sd_mode = active_level;
      break;
    default:
      break;
    }
  }
};

/**
 * @brief TMC51x0 GPIO pin configuration structure
 *
 * This structure allows configuring all TMC51x0 control pins in a single place.
 * Compound pins (pins that share the same physical GPIO) are automatically
 * handled - you only need to specify the GPIO once, and both logical pins
 * will be mapped to it.
 *
 * Compound pin relationships (same physical pin):
 * - Pin 17: REFL_STEP (SD_MODE=0) or STEP (SD_MODE=1) - specify via step_pin or ref_left_pin
 * - Pin 18: REFR_DIR (SD_MODE=0) or DIR (SD_MODE=1) - specify via dir_pin or ref_right_pin
 * - Pin 23: ENCB (SD_MODE=0) or DCEN (SD_MODE=1) - specify via enc_b_pin or dc_en_pin
 * - Pin 24: ENCA (SD_MODE=0) or DCIN (SD_MODE=1) - specify via enc_a_pin or dc_in_pin
 * - Pin 25: ENCN (SD_MODE=0) or DCO (SD_MODE=1) - specify via enc_n_pin or dc_out_pin
 *
 * @note Use -1 (or GPIO_NUM_NC equivalent) for pins that are not connected.
 * @note The ApplyPinConfig() method automatically handles compound pin mapping.
 */
struct TMC51x0PinConfig {
  // Basic control pins
  int en_pin{-1};   ///< EN pin (DRV_ENN, pin 28) - Required
  int dir_pin{-1};  ///< DIR pin (REFR_DIR, pin 18) - Optional, same as ref_right_pin
  int step_pin{-1}; ///< STEP pin (REFL_STEP, pin 17) - Optional, same as ref_left_pin

  // Reference switch pins (when SD_MODE=0)
  int ref_left_pin{-1};  ///< Left reference switch (REFL_STEP, pin 17) - Same as step_pin
  int ref_right_pin{-1}; ///< Right reference switch (REFR_DIR, pin 18) - Same as dir_pin

  // Diagnostic pins (read-only outputs from TMC51x0)
  int diag0_pin{-1}; ///< DIAG0 pin (DIAG0_SWN, pin 26) - Optional
  int diag1_pin{-1}; ///< DIAG1 pin (DIAG1_SWP, pin 27) - Optional

  // Encoder pins (when SD_MODE=0)
  int enc_a_pin{-1}; ///< Encoder A (ENCA_DCIN_CFG5, pin 24) - Same as dc_in_pin
  int enc_b_pin{-1}; ///< Encoder B (ENCB_DCEN_CFG4, pin 23) - Same as dc_en_pin
  int enc_n_pin{-1}; ///< Encoder N (ENCN_DCO_CFG6, pin 25) - Same as dc_out_pin

  // DC Step pins (when SD_MODE=1, SPI_MODE=1)
  int dc_in_pin{-1};  ///< DC Step gating input (ENCA_DCIN_CFG5, pin 24) - Same as enc_a_pin
  int dc_en_pin{-1};  ///< DC Step enable input (ENCB_DCEN_CFG4, pin 23) - Same as enc_b_pin
  int dc_out_pin{-1}; ///< DC Step ready output (ENCN_DCO_CFG6, pin 25) - Same as enc_n_pin

  // Clock pin
  int clk_pin{-1}; ///< Clock input (CLK, pin 12) - Optional

  // Mode configuration pins (if made available as control pins)
  // ⚠️ WARNING: These pins are typically hardwired and read at startup.
  // Only configure these if you have connected SPI_MODE (pin 22) and SD_MODE (pin 21)
  // to GPIO outputs for dynamic mode control. Changing these requires a chip reset.
  int spi_mode_pin{-1}; ///< SPI_MODE pin (pin 22) - Optional, typically hardwired. HIGH=SPI, LOW=UART
  int sd_mode_pin{
      -1}; ///< SD_MODE pin (pin 21) - Optional, typically hardwired. HIGH=External step/dir, LOW=Internal ramp

  /**
   * @brief Default constructor - all pins unmapped (-1)
   */
  TMC51x0PinConfig() = default;

  /**
   * @brief Constructor with basic pins
   * @param en EN pin (required)
   * @param dir DIR pin (optional, -1 if not used)
   * @param step STEP pin (optional, -1 if not used)
   */
  TMC51x0PinConfig(int en, int dir = -1, int step = -1) noexcept : en_pin(en), dir_pin(dir), step_pin(step) {}
};

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
 */
struct Esp32SpiPinConfig {
  // SPI bus pins (required for SPI communication)
  int spi_mosi{-1}; ///< SPI MOSI pin (Master Out, Slave In)
  int spi_miso{-1}; ///< SPI MISO pin (Master In, Slave Out)
  int spi_sclk{-1}; ///< SPI clock pin (SCLK)
  int spi_cs{-1};   ///< SPI chip select pin (CS)

  // TMC51x0 control pins (from TMC51x0PinConfig)
  TMC51x0PinConfig tmc51x0_pins; ///< TMC51x0 control pin configuration

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
  Esp32SpiPinConfig(int mosi, int miso, int sclk, int cs, const TMC51x0PinConfig& tmc_pins) noexcept
      : spi_mosi(mosi), spi_miso(miso), spi_sclk(sclk), spi_cs(cs), tmc51x0_pins(tmc_pins) {}
};

/**
 * @brief SPI_STATUS structure - status flags returned with each SPI datagram
 *
 * Per datasheet section 4.1.2, SPI_STATUS is transmitted with each SPI access
 * in bits 39 to 32 (byte 0 of response). New status information becomes latched
 * at the end of each access and is available with the next SPI transfer.
 */
struct SpiStatus {
  uint8_t value; ///< Raw SPI_STATUS byte value

  /**
   * @brief Extract SPI_STATUS from response byte
   * @param status_byte Byte 0 from SPI response (bits 39-32)
   * @return SpiStatus structure with parsed bits
   */
  static SpiStatus FromByte(uint8_t status_byte) noexcept {
    SpiStatus status{};
    status.value = status_byte;
    return status;
  }

  /**
   * @brief Check if any error flags are set
   * @return true if driver_error is set (reset_flag is informational, not an error)
   *
   * Note: Reset flag indicates the chip was reset (normal on power-up) and is cleared when read.
   * Only driver_error indicates an actual error condition.
   */
  [[nodiscard]] bool HasError() const noexcept {
    return DriverError(); // Only bit 1 (driver_error) is an error; bit 0 (reset) is informational
  }

  /**
   * @brief Get reset flag (bit 0)
   * @return true if reset has occurred
   */
  [[nodiscard]] bool ResetFlag() const noexcept {
    return (value & 0x01) != 0;
  }

  /**
   * @brief Get driver error flag (bit 1)
   * @return true if driver error occurred
   */
  [[nodiscard]] bool DriverError() const noexcept {
    return (value & 0x02) != 0;
  }

  /**
   * @brief Get StallGuard2 flag (bit 2)
   * @return true if StallGuard flag is active
   */
  [[nodiscard]] bool StallGuard2() const noexcept {
    return (value & 0x04) != 0;
  }

  /**
   * @brief Get standstill flag (bit 3)
   * @return true if motor is in standstill
   */
  [[nodiscard]] bool Standstill() const noexcept {
    return (value & 0x08) != 0;
  }

  /**
   * @brief Get velocity reached flag (bit 4)
   * @return true if target velocity reached (motion controller only)
   */
  [[nodiscard]] bool VelocityReached() const noexcept {
    return (value & 0x10) != 0;
  }

  /**
   * @brief Get position reached flag (bit 5)
   * @return true if target position reached (motion controller only)
   */
  [[nodiscard]] bool PositionReached() const noexcept {
    return (value & 0x20) != 0;
  }

  /**
   * @brief Get stop left switch flag (bit 6)
   * @return true if stop left switch is active (motion controller only)
   */
  [[nodiscard]] bool StopLeft() const noexcept {
    return (value & 0x40) != 0;
  }

  /**
   * @brief Get stop right switch flag (bit 7)
   * @return true if stop right switch is active (motion controller only)
   */
  [[nodiscard]] bool StopRight() const noexcept {
    return (value & 0x80) != 0;
  }

  /**
   * @brief Format status bits as compact string (bit names and values)
   * @return String with format "RST:0 STST:0 VEL:0 POS:0 STOP_L:0 STOP_R:0 SG2:0 DRV_ERR:0"
   */
  [[nodiscard]] std::string FormatStatusBits() const noexcept {
    char buf[128];
    snprintf(buf, sizeof(buf), "RST:%d STST:%d VEL:%d POS:%d STOP_L:%d STOP_R:%d SG2:%d DRV_ERR:%d",
             ResetFlag() ? 1 : 0, Standstill() ? 1 : 0, VelocityReached() ? 1 : 0, PositionReached() ? 1 : 0,
             StopLeft() ? 1 : 0, StopRight() ? 1 : 0, StallGuard2() ? 1 : 0, DriverError() ? 1 : 0);
    return std::string(buf);
  }

  /**
   * @brief Format status flags as human-readable string
   * @return String describing active flags (for debug logging)
   *
   * Note: Reset flag is informational (normal on power-up), not an error
   */
  [[nodiscard]] const char* ToString() const noexcept {
    // This is a simplified version - in practice, you'd want a buffer
    // For now, return a static description of key flags
    if (HasError()) {
      return "DRV_ERR"; // Only driver_error is an error
    }
    if (ResetFlag()) {
      return "RST"; // Reset is informational
    }
    return "OK";
  }
};

/**
 * @brief TMC5160 SPI command structure with union-based frame representation
 *
 * Represents a standard TMC5160 SPI command (40 bits: 8-bit address + 32-bit data).
 * Used for both single-chip and multi-chip (daisy-chain) communication.
 * In daisy-chain mode, multiple commands are sent in one SPI transfer.
 *
 * The frame is structured as:
 * - Byte 0: Address byte (bit 7 = write bit, bits 6-0 = register address)
 * - Bytes 1-4: 32-bit data value (MSB-first, big-endian)
 */
struct SpiCommand {
  /**
   * @brief Union for accessing the 40-bit SPI frame in different ways
   */
  union Frame {
    uint8_t bytes[5]; ///< Frame as 5 bytes (for direct byte access)
    struct {
      uint8_t address_byte;  ///< Address byte (bit 7 = write, bits 6-0 = address)
      uint8_t data_bytes[4]; ///< Data bytes (MSB to LSB)
    } fields;                ///< Frame as structured fields
    uint64_t raw;            ///< Frame as 64-bit value (for easy initialization, upper 24 bits unused)
  } frame;                   ///< The 40-bit SPI frame

  /**
   * @brief Get register address (bits 6-0 of address byte)
   * @return Register address (0x00-0x73)
   */
  [[nodiscard]] uint8_t GetAddress() const noexcept {
    return frame.fields.address_byte & 0x7F;
  }

  /**
   * @brief Check if this is a write command
   * @return true if write command (bit 7 of address byte is set)
   */
  [[nodiscard]] bool IsWrite() const noexcept {
    return (frame.fields.address_byte & 0x80) != 0;
  }

  /**
   * @brief Get 32-bit data value (for writes) or dummy data (for reads)
   * @return 32-bit value (MSB-first from bytes 1-4)
   */
  [[nodiscard]] uint32_t GetValue() const noexcept {
    return (static_cast<uint32_t>(frame.fields.data_bytes[0]) << 24) |
           (static_cast<uint32_t>(frame.fields.data_bytes[1]) << 16) |
           (static_cast<uint32_t>(frame.fields.data_bytes[2]) << 8) | static_cast<uint32_t>(frame.fields.data_bytes[3]);
  }

  /**
   * @brief Set the 5-byte frame from raw bytes
   * @param bytes Pointer to 5 bytes (MSB-first)
   */
  void SetFrame(const uint8_t* bytes) noexcept {
    for (size_t i = 0; i < 5; ++i) {
      frame.bytes[i] = bytes[i];
    }
  }

  /**
   * @brief Get the 5-byte frame as raw bytes
   * @param bytes Output buffer (must be at least 5 bytes)
   */
  void GetFrame(uint8_t* bytes) const noexcept {
    for (size_t i = 0; i < 5; ++i) {
      bytes[i] = frame.bytes[i];
    }
  }

  /**
   * @brief Construct a read command
   * @param addr Register address to read (0x00-0x73)
   */
  static SpiCommand Read(uint8_t addr) noexcept {
    SpiCommand cmd{};
    cmd.frame.raw = 0;                           // Initialize to zero
    cmd.frame.fields.address_byte = addr & 0x7F; // Clear write bit (bit 7 = 0)
    // Data bytes are already zero (dummy data for read)
    return cmd;
  }

  /**
   * @brief Construct a write command
   * @param addr Register address to write (0x00-0x73)
   * @param val 32-bit value to write
   */
  static SpiCommand Write(uint8_t addr, uint32_t val) noexcept {
    SpiCommand cmd{};
    cmd.frame.fields.address_byte = (addr & 0x7F) | 0x80;                      // Set write bit (bit 7 = 1)
    cmd.frame.fields.data_bytes[0] = static_cast<uint8_t>((val >> 24) & 0xFF); // MSB
    cmd.frame.fields.data_bytes[1] = static_cast<uint8_t>((val >> 16) & 0xFF);
    cmd.frame.fields.data_bytes[2] = static_cast<uint8_t>((val >> 8) & 0xFF);
    cmd.frame.fields.data_bytes[3] = static_cast<uint8_t>(val & 0xFF); // LSB
    return cmd;
  }
};

/**
 * @brief TMC5160 SPI response structure
 *
 * Contains the SPI_STATUS and data value from a single chip's response.
 * Used for both single-chip and multi-chip (daisy-chain) communication.
 * Responses are returned in the same order as commands were sent.
 */
struct SpiResponse {
  SpiStatus status; ///< SPI_STATUS flags from the chip
  uint32_t value;   ///< 32-bit data value (for reads) or write confirmation (for writes)
  bool success;     ///< true if no critical errors (reset_flag or driver_error)
};

/**
 * @brief Calculate CRC8 checksum for UART communication
 *
 * TMC5160 uses CRC8-ATM polynomial (x^8 + x^2 + x^1 + x^0 = 0x07) with initial value of zero.
 * The CRC is applied LSB to MSB, including the sync- and addressing byte.
 * The sync nibble is assumed to always be correct.
 *
 * Algorithm per datasheet section 5.2:
 * CRC = (CRC << 1) OR (CRC.7 XOR CRC.1 XOR CRC.0 XOR [new incoming bit])
 *
 * @param data Pointer to the data bytes to checksum
 * @param length Number of bytes to include in the checksum calculation
 * @return 8-bit CRC8 checksum value
 */
static constexpr uint8_t calculateCrc8(const uint8_t* data, size_t length) noexcept {
  uint8_t crc = 0; // Initial value is zero per datasheet

  // Process each byte LSB to MSB
  for (size_t i = 0; i < length; ++i) {
    uint8_t currentByte = data[i];

    // Process each bit LSB to MSB (j=0 is LSB, j=7 is MSB)
    for (uint8_t j = 0; j < 8; ++j) {
      // Check: (CRC >> 7) XOR (currentByte & 0x01)
      // This XORs the MSB of CRC with the LSB of currentByte
      if (((crc >> 7) ^ (currentByte & 0x01)) != 0) {
        crc = (crc << 1) ^ 0x07; // Polynomial 0x07 (CRC8-ATM)
      } else {
        crc = (crc << 1);
      }
      currentByte = currentByte >> 1; // Shift to next bit (LSB to MSB)
    }
  }

  return crc;
}

/**
 * @brief TMC5160 UART frame types
 */
enum class UartFrameType : uint8_t {
  WriteAccess, ///< Write access datagram (8 bytes: 7 bytes + CRC)
  ReadRequest, ///< Read access request datagram (4 bytes: 3 bytes + CRC)
  ReadReply    ///< Read access reply datagram (8 bytes: 7 bytes + CRC)
};

/**
 * @brief TMC5160 UART command/response frame structure with built-in CRC8
 *
 * Represents a TMC5160 UART frame per datasheet section 5.1.
 * Supports write access (8 bytes), read request (4 bytes), and read reply (8 bytes).
 * Automatically calculates and verifies CRC8 checksum.
 *
 * Frame Structure (per datasheet section 5.1):
 * - Write Access: Byte 0 (sync+rsv), Byte 1 (nodeaddr), Byte 2 (RW+addr), Bytes 3-6 (data), Byte 7 (CRC)
 * - Read Request: Byte 0 (sync+rsv), Byte 1 (nodeaddr), Byte 2 (RW+addr), Byte 3 (CRC)
 * - Read Reply: Byte 0 (sync+rsv), Byte 1 (0xFF), Byte 2 (addr=0), Bytes 3-6 (data), Byte 7 (CRC)
 *
 * Sync nibble: 0x05 (Bits 0-3 = 1,0,1,0 transmitted LSB first)
 */
struct UartFrame {
  /**
   * @brief Union for accessing UART frames in different ways
   */
  union Frame {
    uint8_t bytes[8]; ///< Frame as 8 bytes (maximum size)

    // Write Access Structure (8 bytes)
    struct {
      uint8_t sync_reserved; ///< Byte 0: Sync (0x05)
      uint8_t node_addr;     ///< Byte 1: Node Address
      uint8_t rw_address;    ///< Byte 2: RW bit (1) + 7-bit register address
      uint8_t data_bytes[4]; ///< Bytes 3-6: 32-bit data (MSB-first)
      uint8_t crc;           ///< Byte 7: CRC8 checksum
    } write_fields;

    // Read Request Structure (4 bytes)
    struct {
      uint8_t sync_reserved; ///< Byte 0: Sync (0x05)
      uint8_t node_addr;     ///< Byte 1: Node Address
      uint8_t rw_address;    ///< Byte 2: RW bit (0) + 7-bit register address
      uint8_t crc;           ///< Byte 3: CRC8 checksum
    } read_request_fields;

    // Read Reply Structure (8 bytes)
    struct {
      uint8_t sync_reserved; ///< Byte 0: Sync (0x05)
      uint8_t master_addr;   ///< Byte 1: Master Address (0xFF)
      uint8_t reg_addr;      ///< Byte 2: Register Address (0x00)
      uint8_t data_bytes[4]; ///< Bytes 3-6: 32-bit data (MSB-first)
      uint8_t crc;           ///< Byte 7: CRC8 checksum
    } read_reply_fields;

  } frame;

  UartFrameType type; ///< Frame type

  /**
   * @brief Get frame size in bytes based on type
   * @return Frame size: 8 bytes for Write/Reply, 4 bytes for ReadRequest
   */
  [[nodiscard]] size_t GetSize() const noexcept {
    return (type == UartFrameType::ReadRequest) ? 4 : 8;
  }

  /**
   * @brief Get register address from frame
   * @return Register address (0x00-0x73) or 0xFF if invalid
   */
  [[nodiscard]] uint8_t GetAddress() const noexcept {
    if (type == UartFrameType::ReadReply) {
      return frame.read_reply_fields.reg_addr;
    }
    // For ReadRequest and WriteAccess, address is in the same byte (Byte 2)
    // Need to mask off RW bit (bit 7)
    return frame.write_fields.rw_address & 0x7F;
  }

  /**
   * @brief Check if this is a write frame
   * @return true if write frame
   */
  [[nodiscard]] bool IsWrite() const noexcept {
    return type == UartFrameType::WriteAccess;
  }

  /**
   * @brief Get 32-bit data value from frame
   * @return 32-bit value (MSB-first) or 0 if ReadRequest
   */
  [[nodiscard]] uint32_t GetValue() const noexcept {
    if (type == UartFrameType::ReadRequest) {
      return 0;
    }
    // Both WriteAccess and ReadReply have data at offset 3
    return (static_cast<uint32_t>(frame.write_fields.data_bytes[0]) << 24) |
           (static_cast<uint32_t>(frame.write_fields.data_bytes[1]) << 16) |
           (static_cast<uint32_t>(frame.write_fields.data_bytes[2]) << 8) |
           static_cast<uint32_t>(frame.write_fields.data_bytes[3]);
  }

  /**
   * @brief Calculate and set CRC8 checksum for the frame
   * CRC8 is calculated over all bytes except the CRC byte itself
   */
  void CalculateCrc() noexcept {
    size_t frame_size = GetSize();
    size_t crc_length = frame_size - 1; // All bytes except CRC byte

    // Calculate CRC over bytes 0 to (frame_size-2)
    uint8_t calculated_crc = calculateCrc8(frame.bytes, crc_length);

    // Set CRC in the last byte
    frame.bytes[frame_size - 1] = calculated_crc;
  }

  /**
   * @brief Verify CRC8 checksum of the frame
   * @return true if CRC is valid, false otherwise
   */
  [[nodiscard]] bool VerifyCrc() const noexcept {
    size_t frame_size = GetSize();
    size_t crc_length = frame_size - 1; // All bytes except CRC byte

    uint8_t calculated_crc = calculateCrc8(frame.bytes, crc_length);

    // Compare with received CRC (last byte)
    return calculated_crc == frame.bytes[frame_size - 1];
  }

  /**
   * @brief Set the frame from raw bytes
   * @param bytes Pointer to frame bytes
   * @param frame_type Type of frame (determines size)
   */
  void SetFrame(const uint8_t* bytes, UartFrameType frame_type) noexcept {
    type = frame_type;
    size_t frame_size = GetSize();
    for (size_t i = 0; i < frame_size; ++i) {
      frame.bytes[i] = bytes[i];
    }
    // Zero out unused bytes for safety (if any)
    if (frame_size < 8) {
      for (size_t i = frame_size; i < 8; ++i) {
        frame.bytes[i] = 0;
      }
    }
  }

  /**
   * @brief Get the frame as raw bytes
   * @param bytes Output buffer (must be at least GetSize() bytes)
   */
  void GetFrame(uint8_t* bytes) const noexcept {
    size_t frame_size = GetSize();
    for (size_t i = 0; i < frame_size; ++i) {
      bytes[i] = frame.bytes[i];
    }
  }

  /**
   * @brief Construct a write access frame (8 bytes)
   * @param node_addr 8-bit node address (0-127)
   * @param reg_addr Register address to write (0x00-0x73)
   * @param value 32-bit value to write
   * @return UartFrame with CRC8 automatically calculated
   */
  static UartFrame Write(uint8_t node_addr, uint8_t reg_addr, uint32_t value) noexcept {
    UartFrame uart_frame{};
    uart_frame.type = UartFrameType::WriteAccess;

    // Byte 0: Sync nibble (0x05)
    uart_frame.frame.write_fields.sync_reserved = 0x05;

    // Byte 1: Node Address
    uart_frame.frame.write_fields.node_addr = node_addr;

    // Byte 2: RW bit (1) + Register Address
    uart_frame.frame.write_fields.rw_address = (reg_addr & 0x7F) | 0x80;

    // Bytes 3-6: Data (MSB-first)
    uart_frame.frame.write_fields.data_bytes[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    uart_frame.frame.write_fields.data_bytes[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    uart_frame.frame.write_fields.data_bytes[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    uart_frame.frame.write_fields.data_bytes[3] = static_cast<uint8_t>(value & 0xFF);

    // Byte 7: CRC (calculated)
    uart_frame.CalculateCrc();

    return uart_frame;
  }

  /**
   * @brief Construct a read request frame (4 bytes)
   * @param node_addr 8-bit node address (0-127)
   * @param reg_addr Register address to read (0x00-0x73)
   * @return UartFrame with CRC8 automatically calculated
   */
  static UartFrame ReadRequest(uint8_t node_addr, uint8_t reg_addr) noexcept {
    UartFrame uart_frame{};
    uart_frame.type = UartFrameType::ReadRequest;

    // Byte 0: Sync nibble (0x05)
    uart_frame.frame.read_request_fields.sync_reserved = 0x05;

    // Byte 1: Node Address
    uart_frame.frame.read_request_fields.node_addr = node_addr;

    // Byte 2: RW bit (0) + Register Address
    uart_frame.frame.read_request_fields.rw_address = reg_addr & 0x7F;

    // Byte 3: CRC (calculated)
    uart_frame.CalculateCrc();

    return uart_frame;
  }

  /**
   * @brief Construct a read reply frame from received bytes (8 bytes)
   * @param bytes Pointer to received frame bytes
   * @return UartFrame parsed from bytes
   */
  static UartFrame ReadReply(const uint8_t* bytes) noexcept {
    UartFrame uart_frame{};
    uart_frame.type = UartFrameType::ReadReply;
    uart_frame.SetFrame(bytes, UartFrameType::ReadReply);
    return uart_frame;
  }

  /**
   * @brief Check if frame is valid
   * @return true if valid
   */
  [[nodiscard]] bool IsValid() const noexcept {
    if (!VerifyCrc()) {
      return false;
    }
    if (type == UartFrameType::ReadReply) {
      // Verify Master Address (Byte 1) is 0xFF
      return frame.read_reply_fields.master_addr == 0xFF;
    }
    return true;
  }
};

/**
 * @brief CRTP-based communication interface for register read/write operations
 *
 * Defines the common API for higher-level code to read and write registers
 * without knowledge of the underlying transport (SPI or UART).
 * Also provides GPIO control interface for TMC5160 control pins.
 *
 * This template class uses the CRTP (Curiously Recurring Template Pattern) for
 * compile-time polymorphism, providing zero runtime overhead compared to
 * virtual functions.
 *
 * Example usage:
 * @code
 * class MySPI : public tmc5160::SpiCommInterface<MySPI> {
 * public:
 *   CommMode mode() const noexcept { return CommMode::SPI; }
 *   bool spiTransfer(...) { ... }
 *   bool gpioSet(...) { ... }
 *   // ... implement other required methods
 * };
 * @endcode
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class CommInterface {
public:
  /**
   * @brief Construct communication interface
   *
   * Note: Pin active level configuration is handled by the derived class.
   * The base class only deals with abstract signals (ACTIVE/INACTIVE).
   */
  CommInterface() noexcept = default;

  /**
   * @brief Get the underlying communication mode used by this interface
   * @return Communication mode (CommMode::SPI or CommMode::UART)
   */
  [[nodiscard]] CommMode GetMode() const noexcept {
    return static_cast<const Derived*>(this)->GetMode();
  }

  /**
   * @brief Read a 32-bit register from the TMC5160
   * @param address Register address (0x00-0x73)
   * @param value Reference to store the read value
   * @param daisy_chain_position Position in daisy chain (0 = first chip/single chip, default: 0)
   *                             Only used for SPI daisy-chaining. Ignored for UART.
   * @return true if read succeeded, false otherwise
   *
   * Each CommInterface instance can be shared by multiple TMC5160 drivers on the same bus.
   * For daisy-chaining, the daisy_chain_position parameter specifies which chip in the
   * chain to address. For multi-chip setups with separate CSN pins, use separate
   * CommInterface instances or set daisy_chain_position to 0.
   */
  bool ReadRegister(uint8_t address, uint32_t& value, uint8_t daisy_chain_position = 0) noexcept {
    return static_cast<Derived*>(this)->ReadRegister(address, value, daisy_chain_position);
  }

  /**
   * @brief Write a 32-bit register to the TMC5160
   * @param address Register address (0x00-0x73)
   * @param value 32-bit value to write
   * @param daisy_chain_position Position in daisy chain (0 = first chip/single chip, default: 0)
   *                             Only used for SPI daisy-chaining. Ignored for UART.
   * @return true if write succeeded, false otherwise
   *
   * Each CommInterface instance can be shared by multiple TMC5160 drivers on the same bus.
   * For daisy-chaining, the daisy_chain_position parameter specifies which chip in the
   * chain to address. For multi-chip setups with separate CSN pins, use separate
   * CommInterface instances or set daisy_chain_position to 0.
   */
  bool WriteRegister(uint8_t address, uint32_t value, uint8_t daisy_chain_position = 0) noexcept {
    return static_cast<Derived*>(this)->WriteRegister(address, value, daisy_chain_position);
  }

  /**
   * @brief Set GPIO pin signal state (output control)
   * @param pin The TMC51x0 control pin to control
   * @param signal The desired signal state (ACTIVE or INACTIVE)
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool GpioSet(TMC51x0CtrlPin pin, GpioSignal signal) noexcept {
    return static_cast<Derived*>(this)->GpioSet(pin, signal);
  }

  /**
   * @brief Read GPIO pin signal state (input state)
   * @param pin The TMC51x0 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully, false otherwise
   */
  bool GpioRead(TMC51x0CtrlPin pin, GpioSignal& signal) noexcept {
    return static_cast<Derived*>(this)->GpioRead(pin, signal);
  }

  /**
   * @brief Set GPIO pin to active state (convenience method)
   * @param pin The TMC51x0 control pin to set active
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool GpioSetActive(TMC51x0CtrlPin pin) noexcept {
    return GpioSet(pin, GpioSignal::ACTIVE);
  }

  /**
   * @brief Set GPIO pin to inactive state (convenience method)
   * @param pin The TMC51x0 control pin to set inactive
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool GpioSetInactive(TMC51x0CtrlPin pin) noexcept {
    return GpioSet(pin, GpioSignal::INACTIVE);
  }

protected:
  /**
   * @brief Debug logging function for detailed debugging information
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param args Variable arguments list
   */
  void DebugLog(int level, const char* tag, const char* format, va_list args) noexcept {
    static_cast<Derived*>(this)->DebugLog(level, tag, format, args);
  }

public:
  /**
   * @brief Delay execution for specified milliseconds
   * @param ms Milliseconds to delay
   */
  void DelayMs(uint32_t ms) noexcept {
    static_cast<Derived*>(this)->DelayMs(ms);
  }

  /**
   * @brief Delay execution for specified microseconds
   * @param us Microseconds to delay
   */
  void DelayUs(uint32_t us) noexcept {
    static_cast<Derived*>(this)->DelayUs(us);
  }

  /**
   * @brief Set external clock frequency on CLK pin (optional)
   * @param frequency_hz Desired clock frequency in Hz (0 = use internal clock, >0 = external clock frequency)
   * @return true if clock was configured successfully, false if not supported or failed
   *
   * This method is optional - derived classes can implement it if they support
   * providing an external clock signal on the CLK pin. If not implemented, this
   * method will return false, indicating that the internal oscillator should be used.
   *
   * **Clock Mode Selection:**
   *
   * - **Internal Clock**:
   *   - Pass `frequency_hz = 0` to explicitly use internal clock
   *   - CLK pin should be set to GND (low) to enable internal oscillator
   *   - Internal oscillator provides ~12MHz clock
   *   - Return true if CLK pin was successfully set to GND, false if not supported
   *   - f_clk in DriverConfig should still be set correctly (typically 12000000 Hz)
   *   - The driver uses f_clk for timing calculations regardless of clock source
   *
   * - **External Clock**:
   *   - Pass `frequency_hz > 0` to use external clock at specified frequency
   *   - CLK pin should receive clock signal from external source
   *   - Return true if clock signal was successfully provided on CLK pin
   *   - f_clk in DriverConfig must match the actual external clock frequency
   *   - Typical frequencies: 12MHz (default) or 24MHz (for higher performance)
   *
   * **Important:**
   * - The f_clk value in DriverConfig is used for all timing calculations (IHOLDDELAY, TPOWERDOWN, TZEROWAIT, etc.)
   * - f_clk must be set correctly regardless of whether using internal or external clock
   * - For internal clock, f_clk is typically 12000000 Hz (12 MHz)
   * - For external clock, f_clk must match the actual frequency provided
   * - Passing `frequency_hz = 0` allows users with external clock capability to switch back to internal clock
   *
   * **Implementation Guidelines:**
   * - If your system doesn't support clock control, return false (driver will assume internal clock)
   * - If your system supports clock control:
   *   - When `frequency_hz = 0`: Set CLK pin to GND (low) and return true
   *   - When `frequency_hz > 0`: Provide clock signal at specified frequency and return true
   *   - Return false only if the operation failed (e.g., invalid frequency, hardware error)
   *
   * @note This is called automatically during Initialize() with the f_clk value from DriverConfig.
   * @note If not implemented (returns false), the driver assumes internal clock (CLK pin tied to GND).
   * @note The internal clock has a fail-over circuit that protects against loss of external clock signal.
   * @note Per datasheet: "Tie to GND using short wire for internal clock or supply external clock."
   */
  bool SetClkFreq(uint32_t frequency_hz) noexcept {
    // Default implementation returns false (not supported / using internal clock)
    // Derived classes can override this if they support external clock generation
    // When frequency_hz = 0, this means "use internal clock" (set CLK pin to GND)
    (void)frequency_hz; // Suppress unused parameter warning
    return false;
  }

protected:
  /**
   * @brief Protected destructor
   */
  ~CommInterface() = default;

  // Allow moving
  CommInterface(CommInterface&&) = default;
  CommInterface& operator=(CommInterface&&) = default;

public:
  // Prevent copying
  CommInterface(const CommInterface&) = delete;
  CommInterface& operator=(const CommInterface&) = delete;
  /**
   * @brief Public debug logging wrapper for external classes
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param ... Variable arguments for format string
   */
#ifndef TMC51X0_DISABLE_DEBUG_LOGGING
  void LogDebug(int level, const char* tag, const char* format, ...) noexcept {
    va_list args{};  // va_start will properly initialize this
    va_start(args, format);

    // Modern C++ string handling - no manual memory management
    std::string format_str(format);
    
    // Ensure format string ends with newline
    if (format_str.empty() || format_str.back() != '\n') {
      format_str += '\n';
    }

    // Pass modified format string and va_list to DebugLog
    // DebugLog will handle the actual formatting (e.g., via esp_log_writev)
    DebugLog(level, tag, format_str.c_str(), args);

    va_end(args);
  }
#else
  // Debug logging disabled - function optimized out completely
  inline void LogDebug(int /*level*/, const char* /*tag*/, const char* /*format*/, ...) noexcept {
    // Empty function body - all logging optimized out
  }
#endif
};

/**
 * @brief CRTP-based SPI implementation of TMC5160CommInterface
 *
 * Uses a 4-wire SPI bus (mode 3) to exchange 40-bit datagrams per datasheet section 4.1.
 * The implementation uses 8 bytes (64 bits) for convenience, which is acceptable per datasheet
 * as additional bits beyond 40 are shifted through an internal shift register for daisy-chaining.
 * Data is sent MSB-first, big-endian.
 *
 * SPI Datagram Structure (40 bits):
 * - Bit 39: W (WRITE_notREAD bit) - 0 for read, 1 for write
 * - Bits 38-32: 7-bit register address
 * - Bits 31-0: 32-bit data (right-aligned)
 *
 * Byte structure (MSB transmitted first):
 * - Byte 0: W bit (bit 7) + 7-bit address (bits 6-0)
 *   - Read: address & 0x7F (bit 7 = 0)
 *   - Write: (address & 0x7F) | 0x80 (bit 7 = 1, add 0x80 to address)
 * - Bytes 1-4: 32-bit data (MSB-first)
 * - Bytes 5-7: Optional, unused (for daisy-chaining)
 *
 * Response (40 bits):
 * - Byte 0: SPI_STATUS (bits 39-32) - 8 status flags
 * - Bytes 1-4: 32-bit data (bits 31-0)
 *
 * Read access (pipelined per datasheet section 4.1.1):
 * - Uses dummy write data (can be 0)
 * - Read data is transferred back with the subsequent access
 * - Requires two transfers: first sends address, second receives data
 *
 * Write access:
 * - Address | 0x80, then 4 bytes of data
 * - Response mirrors previously written data
 *
 * SPI Signals (per datasheet section 4.2):
 * - SCK: Bus clock input
 * - SDI: Serial data input (latched on rising edge of SCK)
 * - SDO: Serial data output (driven following falling edge of SCK)
 * - CSN: Chip select input (active low, must stay low during entire transaction)
 *
 * Timing (per datasheet section 4.3):
 * - Minimum 40 SCK clock cycles required
 * - SPI Mode 3 (CPOL=1, CPHA=1)
 * - MSB transmitted first
 *
 * ## Daisy-Chaining Support
 *
 * The TMC5160 supports daisy-chaining multiple chips on a single SPI bus per datasheet section 4.2.
 * Each chip has an internal 40-bit shift register. Data on SDI is continuously shifted out on SDO
 * with a 40-clock cycle delay, allowing data to propagate through the chain.
 *
 * Hardware Setup for Daisy-Chaining:
 * - All chips share: CSN (tied together), SCK, and SDI (MOSI)
 * - MISO is daisy-chained: Master -> Chip0 SDO -> Chip1 SDI, Chip1 SDO -> Chip2 SDI, etc.
 * - Last chip's SDO connects back to Master MISO
 * - CS must be held low during the entire transfer (all bits must be sent while CS is low)
 *
 * How Daisy-Chaining Works:
 * - Data shifts continuously through the chain as long as CS is low and SCK is active
 * - Each chip's SDO outputs data with a 40-clock delay (data clocked into SDI appears on SDO
 *   after 40 clock cycles)
 * - Commands are latched when CS goes HIGH (rising edge) - each chip latches the 40 bits
 *   currently in its internal shift register at that moment
 * - To address chip k (where k=0 is first chip, k=1 is second chip, etc.):
 *   - Send 40 bits of command (first)
 *   - Then send 40·k bits of padding (dummy zeros) to shift the command to chip k
 *   - Total bits = 40·(k+1)
 *   - The command for chip k must be the FIRST 40 bits sent, which will propagate to chip k's
 *     shift register after (k+1)*40 clock cycles
 *
 * Response Reading (per datasheet):
 * - Response from chip k in a chain of n devices requires sending 40·(n-k) dummy bits total
 *   Formula: 40·(n-k) bits of padding + 40 bits of command = 40·(n-k) bits (command included in total)
 * - **CRITICAL**: Responses come back in REVERSE order (last device first, first device last)
 *   - Device n-1's response appears FIRST (at byte 0)
 *   - Device n-2's response appears second (at byte 5)
 *   - ...
 *   - Device 0's response appears LAST (at byte (n-1)*5)
 * - To read from device k in chain of n devices:
 *   - Send 40·(n-k) bits total (40·(n-k-1) padding + 40 command, or just 40 command if k=n-1)
 *   - Response from device k appears at offset (n-k-1)*5 bytes in received data
 *   - Example: For n=3, k=1: Send 40*(3-1)=80 bits, response at offset (3-1-1)*5=5 bytes
 *
 * **CRITICAL: Chain Length MUST Always Be Known**
 *
 * **Sending vs Receiving Calculations:**
 * - **Sending (Command Transmission)**: To address device k, send (k+1)*40 bits = (k+1)*5 bytes
 *   - This shifts the command through k devices to reach device k
 *   - This calculation only requires knowing k (device position)
 *
 * - **Receiving (Response Extraction)**: To receive response from device k in chain of n devices:
 *   - Total transfer size: 40·(n-k) bits = (n-k)*5 bytes (datasheet formula)
 *   - Response appears at offset (n-k-1)*5 bytes (reverse order: last device first)
 *   - This calculation REQUIRES knowing n (total chain length)
 *
 * - **Transfer Size**: For simultaneous send/receive, use max((k+1)*5, (n-k)*5) bytes
 *   - Sending requirement: (k+1)*5 bytes needed to shift command to device k
 *   - Receiving requirement: (n-k)*5 bytes needed to shift response back (datasheet formula)
 *   - Use max() to ensure both requirements are met:
 *     * For k < n/2: (n-k)*5 >= (k+1)*5, so (n-k)*5 dominates
 *     * For k >= n/2: (k+1)*5 > (n-k)*5, so (k+1)*5 dominates
 *   - Extra bytes beyond (k+1)*5 are padding (zeros) for full-duplex response extraction
 *
 * **Chain Length Requirements:**
 * - Chain length MUST be known for correct response extraction
 * - If `daisy_chain_position > 0` and chain length is unknown, it is automatically detected
 * - If detection fails, operation returns false (chain length is required)
 * - For single chip (`daisy_chain_position == 0`), chain length defaults to 1
 *
 * **Auto-Detection on First Access:**
 * - If `daisy_chain_position > 0` and chain length is unknown, it is automatically
 *   detected on the first ReadRegister() or WriteRegister() call
 * - Detection uses `AutoDetectChainLength()` which sends a unique command that loops back
 * - The detected length is stored and used for all subsequent operations
 * - If detection fails, operation returns false
 *
 * **Verification of User-Specified Length:**
 * - If user calls `SetDaisyChainLength(n)`, the specified length is verified against
 *   auto-detected length on first access
 * - If mismatch is detected, an error is logged: "DAISY CHAIN LENGTH MISMATCH!"
 * - The detected length is used (not the user-specified value) to ensure correctness
 *
 * **Why This Matters:**
 * - Incorrect chain length leads to wrong response extraction offset
 * - This causes reading wrong register values and incorrect SPI_STATUS flags
 * - Auto-detection ensures correctness without requiring user to manually specify length
 * - Verification catches configuration errors early
 *
 * For multi-chip simultaneous communication:
 * - Commands must be sent in REVERSE order: [cmd_n-1] [cmd_n-2] ... [cmd_0] (last device first)
 * - Total: n * 40 bits = n * 5 bytes (back-to-back, no padding needed)
 * - As data shifts continuously through the chain:
 *   - After (k+1)*40 clocks, device k has received cmd_k in its shift register
 *   - After n*40 clocks total, device 0 has cmd_0, device 1 has cmd_1, ..., device n-1 has cmd_n-1
 * - When CS goes HIGH (rising edge), each device latches the 40 bits currently in its shift
 * register
 * - The reverse order ensures that after n*40 clocks, each device has the correct command to latch
 * Example for 3-chip daisy chain (n=3, devices 0, 1, 2):
 * - Single chip addressing:
 *   - Address chip 0 (k=0): Send 40-bit command + 0 padding = 40 bits (5 bytes)
 *   - Address chip 1 (k=1): Send 40-bit command + 40 padding = 80 bits (10 bytes)
 *   - Address chip 2 (k=2): Send 40-bit command + 80 padding = 120 bits (15 bytes)
 * - Multi-chip simultaneous (all 3 chips):
 *   - Send: [cmd_2] [cmd_1] [cmd_0] = 120 bits (15 bytes, back-to-back, reverse order)
 *   - As data shifts continuously through the chain (CS held low):
 *     - After 40 clocks: Device 0 has cmd_2 in shift register, Device 1 has nothing, Device 2 has
 * nothing
 *     - After 80 clocks: Device 0 has cmd_1 (shifted in), Device 1 has cmd_2 (from Device 0's SDO),
 *       Device 2 has nothing
 *     - After 120 clocks: Device 0 has cmd_0 (shifted in), Device 1 has cmd_1 (from Device 0's
 * SDO), Device 2 has cmd_2 (from Device 1's SDO)
 *   - When CS goes HIGH (rising edge): Each device latches the 40 bits currently in its shift
 * register
 *     - Device 0 latches cmd_0 ✓
 *     - Device 1 latches cmd_1 ✓
 *     - Device 2 latches cmd_2 ✓
 * - Read response from chip k: Send 40·(n-k) dummy bits total (40·(n-k-1) padding + 40 command, or just 40 command if
 * k=n-1) Response appears at offset (n-k-1)*5 bytes (reverse order: last device first)
 *   **CRITICAL**: Responses come back in REVERSE order - device n-1 response first, then n-2, ...,
 * device 0 last
 *
 * Architecture:
 * - Each SpiCommInterface instance supports ONE SPI bus (shared by multiple TMC5160 drivers)
 * - For daisy-chaining: Each TMC5160 instance tracks its own position and passes it to
 * ReadRegister()/WriteRegister()
 * - For multi-CS setups: Create separate SpiCommInterface instances (one per chip)
 * - ReadRegister() and WriteRegister() automatically handle daisy-chaining based on position
 * parameter
 * - SetDaisyChainLength() must be called to enable optimal response extraction using datasheet
 * formula
 * - For higher-level multi-driver management, use TMC51x0DaisyChain class that manages multiple
 *   TMC5160 instances on a single SPI bus and automatically configures chain length
 *
 * Example usage:
 * @code
 * class MySPI : public tmc5160::SpiCommInterface<MySPI> {
 * public:
 *   CommMode mode() const noexcept { return CommMode::SPI; }
 *   bool spiTransfer(...) { ... }
 *   bool gpioSet(...) { ... }
 *   bool gpioRead(...) { ... }
 *   void debugLog(...) { ... }
 *   void delayMs(...) { ... }
 *   void delayUs(...) { ... }
 * };
 * @endcode
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class SpiCommInterface : public CommInterface<Derived> {
public:
  /**
   * @brief Construct SPI communication interface
   *
   * Note: Pin active level configuration is handled by the derived class.
   * The base class only deals with abstract signals (ACTIVE/INACTIVE).
   */
  SpiCommInterface() noexcept : CommInterface<Derived>() {}

  /**
   * @brief Set the total number of devices in the daisy chain
   * @param total_length Total number of devices in the chain (1 = single chip, 2 = two chips, etc.)
   *                     Set to 0 to disable daisy-chain mode (single chip, default)
   * @note This is CRITICAL for proper response extraction using the datasheet formula 40·(n-k)
   * @note This should be set once during initialization, before any register access
   * @note For optimal efficiency with TMC51x0DaisyChain, set this to match the chain length
   * @note If set to a non-zero value, it will be verified against auto-detected length on first
   * access. If mismatch is detected, ReadRegister/WriteRegister will return false.
   * @note If set to 0 and daisy_chain_position > 0, chain length will be auto-detected on first
   * access.
   */
  void SetDaisyChainLength(uint8_t total_length) noexcept {
    total_chain_length_ = total_length;
    user_specified_chain_length_ = total_length; // Track user-specified value
    chain_length_verified_ = false;              // Reset verification flag
  }

  /**
   * @brief Get the total number of devices in the daisy chain
   * @return Total chain length (0 = unknown/single chip, >0 = total number of devices)
   */
  [[nodiscard]] uint8_t GetDaisyChainLength() const noexcept {
    return total_chain_length_;
  }

  /**
   * @brief Auto-detect the daisy chain length by sending a unique command that loops back
   *
   * This method sends a command with a unique, recognizable pattern to position (max_devices+1),
   * which is beyond the last device. The command will shift through all devices and loop back
   * to the MCU via the last device's SDO. By searching for our exact command pattern in the
   * received data, we can determine the actual chain length.
   *
   * Algorithm:
   * 1. Create a unique command with a distinctive pattern (e.g., read register 0x73 with unique
   * data)
   * 2. Send command to position (max_devices+1) - beyond the last device
   * 3. Send enough padding: (max_devices+2)*40 bits total to ensure loopback is captured
   * 4. The command loops back after (n+1)*40 bits where n is the actual chain length
   * 5. Search received data for our exact command pattern
   * 6. When found at offset (n+1)*5, chain length = n
   *
   * @param max_devices Maximum number of devices to probe (default: 8, max: 255)
   * @return Detected chain length (0 = single chip or detection failed, >0 = number of devices)
   *
   * @note This method requires that devices are powered (but don't need to be initialized)
   * @note The command uses a read operation which is safe and doesn't modify device state
   * @note The detected length is automatically set via SetDaisyChainLength()
   *
   * @warning This method performs a full SPI transaction and may take time
   */
  uint8_t AutoDetectChainLength(uint8_t max_devices = 8) noexcept {
    if (max_devices == 0) {
      max_devices = 8; // Default to 8 devices
    }

    // Create a unique command pattern that we can reliably identify when it loops back
    // Use a read command to a register that's safe to read (GSTAT = 0x00)
    // But we'll use a unique address pattern to make it more distinctive
    // Actually, let's use a write command with a unique value that we can recognize
    // But writes modify state... better to use read with a unique address

    // Best approach: Use a read command with a distinctive address
    // We'll use address 0x73 (last register) which is safe to read
    // The command pattern will be: [0x73] [0x00] [0x00] [0x00] [0x00]
    // This is distinctive enough to identify

    // Actually, even better: Use a read to an address that's unlikely to appear in device responses
    // Let's use 0x73 (last register address) - this is distinctive
    SpiCommand cmd = SpiCommand::Read(0x73); // Read last register (0x73)

    // Get the command frame bytes so we can search for it
    uint8_t cmd_bytes[5];
    cmd.GetFrame(cmd_bytes);

    // Send command to position (max_devices+1) - beyond the last device
    // Total transfer: (max_devices+2)*40 bits to ensure we capture loopback
    size_t transfer_bytes = static_cast<size_t>(max_devices + 2) * 5;

    std::vector<uint8_t> tx_buf(transfer_bytes, 0);
    std::vector<uint8_t> rx_buf(transfer_bytes, 0);

    // Place command at the beginning (bytes 0-4)
    cmd.GetFrame(tx_buf.data());
    // Rest is padding (zeros) - already initialized to 0

    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 2, "SPI",
                      "AutoDetectChainLength: Probing up to %u devices, transfer_bytes=%zu, "
                      "cmd_pattern=%02X %02X %02X %02X %02X",
                      max_devices, transfer_bytes, cmd_bytes[0], cmd_bytes[1], cmd_bytes[2], cmd_bytes[3],
                      cmd_bytes[4]);

    // Perform SPI transfer
    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), transfer_bytes)) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI", "AutoDetectChainLength: SPI transfer failed");
      return 0;
    }

    // Search for our exact command pattern in the received data
    // The command loops back after n*40 bits, appearing at offset n*5 bytes
    // Since each device delays data by 40 clocks, our command should appear unmodified
    // at the loopback point

    uint8_t detected_length = 0;

    // Search backwards from max_devices to find where our command appears
    // For n devices, our command appears at offset n*5 after looping back
    // We require an EXACT match of all 5 bytes to confirm loopback
    for (uint8_t n = max_devices; n >= 1; --n) {
      size_t loopback_offset = static_cast<size_t>(n) * 5;

      if (loopback_offset + 4 < rx_buf.size()) {
        // Check if the 5-byte chunk at loopback_offset matches our command pattern EXACTLY
        // This is the only reliable way to confirm the command looped back correctly
        bool exact_match = true;

        for (uint8_t i = 0; i < 5; ++i) {
          if (rx_buf[loopback_offset + i] != cmd_bytes[i]) {
            exact_match = false;
            break;
          }
        }

        if (exact_match) {
          // Found our exact command pattern! This confirms it looped back after n devices
          detected_length = n;
          TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 2, "SPI",
                            "AutoDetectChainLength: Found EXACT command pattern match at offset "
                            "%zu (expected for n=%u), chain length = %u",
                            loopback_offset, n, detected_length);
          break;
        }
      }
    }

    // If exact match failed, log debug info to help diagnose
    if (detected_length == 0) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "AutoDetectChainLength: Exact command pattern not found in received data");
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                        "AutoDetectChainLength: Expected pattern: %02X %02X %02X %02X %02X", cmd_bytes[0], cmd_bytes[1],
                        cmd_bytes[2], cmd_bytes[3], cmd_bytes[4]);

      // Log first few potential loopback positions for debugging
      for (uint8_t n = 1; n <= 3 && n <= max_devices; ++n) {
        size_t loopback_offset = static_cast<size_t>(n) * 5;
        if (loopback_offset + 4 < rx_buf.size()) {
          TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                            "AutoDetectChainLength: At offset %zu (n=%u): %02X %02X %02X %02X %02X", loopback_offset, n,
                            rx_buf[loopback_offset], rx_buf[loopback_offset + 1], rx_buf[loopback_offset + 2],
                            rx_buf[loopback_offset + 3], rx_buf[loopback_offset + 4]);
        }
      }
    }

    // If we detected a length, set it (but don't overwrite user-specified value yet)
    // The caller will handle verification and update
    if (detected_length > 0) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 2, "SPI", "AutoDetectChainLength: Detected chain length = %u",
                        detected_length);
      // Only update if not user-specified, or if user-specified value matches
      if (user_specified_chain_length_ == 0 || user_specified_chain_length_ == detected_length) {
        total_chain_length_ = detected_length;
      }
    } else {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "AutoDetectChainLength: Command pattern not found, assuming single chip");
      // Don't reset total_chain_length_ if it was already set (e.g., to 1 for single-chip mode)
      // Only reset if it was 0 and user hasn't specified a length
      if (user_specified_chain_length_ == 0 && total_chain_length_ == 0) {
        // Keep it at 0 - caller will handle single-chip case
      }
    }

    return detected_length;
  }

  /**
   * @brief Get communication mode (always SPI for this interface)
   * @return CommMode::SPI
   */
  [[nodiscard]] CommMode GetMode() const noexcept {
    return CommMode::SPI;
  }

  /**
   * @brief Low-level SPI transfer for register read/write
   * @param tx Buffer containing bytes to transmit
   * @param rx Buffer to receive bytes from device
   * @param length Number of bytes to transfer
   * @return true if the SPI transfer completed successfully
   */
  bool SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) noexcept {
    return static_cast<Derived*>(this)->SpiTransfer(tx, rx, length);
  }

  /**
   * @brief Read a 32-bit register via SPI
   * @param address Register address (0x00-0x73)
   * @param value Reference to store the read value
   * @param daisy_chain_position Position in daisy chain (0 = first chip/single chip, default: 0)
   *                             Specifies which chip in the chain to address
   * @return true if read succeeded, false otherwise
   *
   * This method handles both single-chip and daisy-chain modes:
   * - Single chip (daisy_chain_position = 0): Standard 40-bit SPI transaction (5 bytes)
   * - Daisy-chain (daisy_chain_position > 0): Sends command with padding to shift
   *   it to the target device position in the chain
   *
   * **Daisy-Chain Transfer Logic:**
   *
   * **CRITICAL: Chain length MUST be known for correct operation**
   * - Chain length is automatically detected on first access if not manually set
   * - If detection fails, operation will fail (chain length is required)
   *
   * **Sending (Command Transmission):**
   * - Command is placed at bytes 0-4 (the 40-bit SPI frame)
   * - Padding (zeros) is added from byte 5 onwards
   * - To address device k: Send (k+1)*40 bits = (k+1)*5 bytes minimum
   *   - This shifts the command through k devices to reach device k
   *
   * **Receiving (Response Extraction):**
   * - To receive response from device k in chain of n devices:
   *   - Total transfer size: 40·(n-k) bits = (n-k)*5 bytes (datasheet formula)
   *   - Response appears at offset (n-k-1)*5 bytes (reverse order: last device first)
   *   - This requires knowing total chain length n
   *
   * **Transfer Size Calculation:**
   * - For simultaneous send/receive, use max((k+1)*5, (n-k)*5) bytes
   *   - Sending requirement: (k+1)*5 bytes to shift command to device k
   *   - Receiving requirement: (n-k)*5 bytes to shift response back (datasheet formula)
   *   - Use max() to ensure both requirements are met:
   *     * For k < n/2: (n-k)*5 >= (k+1)*5, so (n-k)*5 dominates
   *     * For k >= n/2: (k+1)*5 > (n-k)*5, so (k+1)*5 dominates
   *   - Extra bytes beyond (k+1)*5 are padding (zeros) for full-duplex behavior
   *
   * **Auto-Detection and Verification:**
   * - If `daisy_chain_position > 0` and chain length is unknown (`total_chain_length_ == 0`),
   *   chain length is automatically detected on first access
   * - If user has specified a chain length via `SetDaisyChainLength()`, it is verified
   *   against auto-detected length on first access
   * - If mismatch is detected, an error is logged and the detected length is used
   * - If detection fails, operation returns false (chain length is required)
   *
   * **Architecture Note:**
   * This SpiCommInterface instance can be shared by multiple TMC5160 drivers on the
   * same SPI bus. Each TMC5160 instance has its own daisy_chain_position_ and passes
   * it as a parameter when calling ReadRegister() or WriteRegister().
   *
   * CSN is handled automatically by the SPI hardware or derived class implementation.
   * For daisy-chaining, all chips share the same CSN (tied together).
   */
  bool ReadRegister(uint8_t address, uint32_t& value, uint8_t daisy_chain_position = 0) noexcept {
    // Log function call with arguments (level 3 = DEBUG, only shows at DEBUG log level)
    if (daisy_chain_position > 0) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI", "ReadRegister(0x%02X, daisy_chain=%u)", address,
                        daisy_chain_position);
    } else {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI", "ReadRegister(0x%02X)", address);
    }

    // CRITICAL: Chain length MUST be known for correct response extraction
    // Ensure chain length is known and verified (auto-detects if needed)
    if (!EnsureChainLengthKnown(daisy_chain_position, "ReadRegister")) {
      return false;
    }

    // Build command using SpiCommand structure (union-based frame)
    SpiCommand cmd = SpiCommand::Read(address);

    // CRITICAL: Chain length MUST be known at this point
    // If daisy_chain_position > 0, total_chain_length_ must be > 0 (detected or set)
    // If daisy_chain_position == 0, total_chain_length_ should be 1 (single chip)
    if (daisy_chain_position > 0 && total_chain_length_ == 0) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "ReadRegister: Chain length unknown for daisy_chain_position=%u. "
                        "Cannot proceed without chain length.",
                        daisy_chain_position);
      return false;
    }

    // Calculate transfer size and response offset using datasheet formula
    // Transfer size: max((k+1)*5, (n-k)*5) bytes
    //   - Sending: (k+1)*5 bytes needed to address device k
    //   - Receiving: (n-k)*5 bytes needed to shift response back (datasheet formula)
    //   - Use max() to ensure both requirements are met
    // Response offset: (n-k-1)*5 bytes (reverse order: last device first)
    uint8_t n = total_chain_length_;
    uint8_t k = daisy_chain_position;

    // Validate: k must be < n (device position must be less than total chain length)
    if (k >= n) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "ReadRegister: Invalid daisy_chain_position=%u for chain length=%u. "
                        "Position must be < chain length.",
                        k, n);
      return false;
    }

    // Calculate transfer size: max of sending and receiving requirements
    size_t sending_bytes = static_cast<size_t>(k + 1) * 5;   // Command must reach device k
    size_t receiving_bytes = static_cast<size_t>(n - k) * 5; // Response extraction (datasheet formula)
    size_t transfer_bytes = std::max(sending_bytes, receiving_bytes);

    // Response offset: (n-k-1)*5 bytes (based on reverse order of devices)
    size_t response_byte_offset = static_cast<size_t>(n - k - 1) * 5;

    // Validate transfer size meets receiving requirement
    if (transfer_bytes < receiving_bytes) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "ReadRegister: Transfer size %zu bytes < receiving requirement %zu bytes. "
                        "Response extraction may fail.",
                        transfer_bytes, receiving_bytes);
      return false;
    }

    // Validate response offset is within buffer bounds
    if (response_byte_offset + 4 >= transfer_bytes) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "ReadRegister: Response offset %zu + 4 >= transfer size %zu. "
                        "Cannot read full 5-byte response.",
                        response_byte_offset, transfer_bytes);
      return false;
    }

    std::vector<uint8_t> tx_buf(transfer_bytes, 0);
    std::vector<uint8_t> rx_buf(transfer_bytes, 0);

    // Place command at the beginning (bytes 0-4)
    // For daisy-chain position k, the command is placed first, then padding (zeros)
    // Padding structure:
    //   - Bytes 5 to (k+1)*5-1: Padding to shift command to device k
    //   - Bytes (k+1)*5 to transfer_bytes-1: Additional padding for full-duplex response extraction
    //     (only present if transfer_bytes > (k+1)*5, i.e., when (n-k)*5 > (k+1)*5)
    cmd.GetFrame(tx_buf.data());
    // Bytes 5 onwards are padding (zeros) - already initialized to 0 by vector constructor

    // First transaction: Send read command (TX1), receive status (RX1)
    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), transfer_bytes)) {
      return false;
    }

    // Log [TX1]/RX1 after first transfer
    // Show "=0x00000000" for reads to align with Write format (read command has no data, all zeros)
    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                      "Read 0x%02X=0x00000000: [TX1] %02X %02X %02X %02X %02X / RX1 %02X %02X %02X %02X %02X", address,
                      tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], rx_buf[response_byte_offset],
                      (response_byte_offset + 1 < rx_buf.size()) ? rx_buf[response_byte_offset + 1] : 0,
                      (response_byte_offset + 2 < rx_buf.size()) ? rx_buf[response_byte_offset + 2] : 0,
                      (response_byte_offset + 3 < rx_buf.size()) ? rx_buf[response_byte_offset + 3] : 0,
                      (response_byte_offset + 4 < rx_buf.size()) ? rx_buf[response_byte_offset + 4] : 0);

    // Minimum CSN high time: 2*tclk + 10ns (typically ~176ns with 12MHz clock)
    // Per datasheet: CSN must go high between pipelined read transfers
    // Use 10us delay to ensure TMC5160 has time to prepare pipelined data
    // Some registers (like GLOBAL_SCALER, X_COMPARE) may require longer delay
    // Note: ESP32 spi_device_transmit automatically handles CSN (pulls high after transfer)
    this->DelayUs(10);

    // Second transaction: Send address again (TX2), receive actual data (RX2 - pipelined read)
    // Per datasheet: Read data is transferred back with the subsequent access
    // Use same transfer size for daisy-chaining consistency
    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), transfer_bytes)) {
      return false;
    }

    // Log TX2/[RX2] after second transfer (RX2 contains the actual read data)
    // Align TX2 line with TX1 line by padding address field
    SpiStatus status = SpiStatus::FromByte(rx_buf[response_byte_offset]);
    std::string status_bits = status.FormatStatusBits();

    // Align TX2 bytes with TX1 bytes: "Read 0xXX=0x00000000: " (25) + "[TX1] " (6) = 31 chars to first byte
    // For TX2: "Read 0xXX=0x00000000: " (25) + "      " (6 spaces) + "TX2 " (4) = 35, but bytes should be at 31
    // Actually: align "TX2" label with "[TX1]" label, then bytes naturally align
    // "Read 0xXX=0x00000000: [TX1] " = 31, bytes at 31
    // "Read 0xXX=0x00000000:      TX2 " = 31 (25 + 6), bytes at 31 ✓
    TMC51X0_LOG_DEBUG(
        *static_cast<Derived*>(this), 3, "SPI",
        "Read 0x%02X:             TX2 %02X %02X %02X %02X %02X / [RX2] %02X %02X %02X %02X %02X (STATUS=0x%02X)",
        address, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], rx_buf[response_byte_offset],
        (response_byte_offset + 1 < rx_buf.size()) ? rx_buf[response_byte_offset + 1] : 0,
        (response_byte_offset + 2 < rx_buf.size()) ? rx_buf[response_byte_offset + 2] : 0,
        (response_byte_offset + 3 < rx_buf.size()) ? rx_buf[response_byte_offset + 3] : 0,
        (response_byte_offset + 4 < rx_buf.size()) ? rx_buf[response_byte_offset + 4] : 0,
        rx_buf[response_byte_offset]);

    // Log status bit breakdown with arrow pointing to STATUS byte
    // Calculate position: "Read 0xXX:            TX2 XX XX XX XX XX / [RX2] " = ~60 chars, then STATUS byte at ~68
    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                      "                                                   └─> %s", status_bits.c_str());

    // Extract response data based on daisy-chain position
    // IMPORTANT: Responses come back in REVERSE order (last device first, first device last)
    // Response offset is calculated above based on whether total_chain_length_ is known
    // - If total_chain_length_ > 0: Use datasheet formula, response at (n-k-1)*5 bytes
    // - If total_chain_length_ == 0: Use simplified approach, response at k*5 bytes (end of
    // transfer)

    // Extract SPI_STATUS from response byte 0 (bits 39-32 per datasheet section 4.1.2)
    // For daisy-chaining, this is at the calculated offset
    // (status was already extracted above for logging)
    if (response_byte_offset >= rx_buf.size()) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "Read register 0x%02X: Response offset %zu exceeds buffer size %zu", address,
                        response_byte_offset, rx_buf.size());
      return false;
    }

    // Log SPI_STATUS with detailed flag information
    // Note: RESET (bit 0) is informational (normal on power-up), only DRV_ERR (bit 1) is an error
    if (status.HasError()) {
      // Build error flags string (only actual errors)
      const char* error_flags = "DRV_ERR";

      // Build informational flags string (reset + status flags)
      char info_flags[64] = "";
      if (status.ResetFlag() || status.StallGuard2() || status.Standstill() || status.VelocityReached() ||
          status.PositionReached() || status.StopLeft() || status.StopRight()) {
        snprintf(info_flags, sizeof(info_flags), " [%s%s%s%s%s%s%s]", status.ResetFlag() ? "RST " : "",
                 status.StallGuard2() ? "SG2 " : "", status.Standstill() ? "STST " : "",
                 status.VelocityReached() ? "VEL " : "", status.PositionReached() ? "POS " : "",
                 status.StopLeft() ? "STOP_L " : "", status.StopRight() ? "STOP_R " : "");
      }

      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI", "Read 0x%02X: STATUS=0x%02X ERROR=%s%s", address,
                        status.value, error_flags, info_flags);
    } else {
      // Log response bytes (first 8 or all if less)
      size_t log_rx_bytes = (rx_buf.size() < 8) ? rx_buf.size() : 8;
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                        "Read register 0x%02X: RX[0..%zu] %02X %02X %02X %02X %02X %02X %02X %02X "
                        "| SPI_STATUS=0x%02X [%s%s%s%s%s%s%s%s]",
                        address, log_rx_bytes - 1, rx_buf[0], (log_rx_bytes > 1) ? rx_buf[1] : 0,
                        (log_rx_bytes > 2) ? rx_buf[2] : 0, (log_rx_bytes > 3) ? rx_buf[3] : 0,
                        (log_rx_bytes > 4) ? rx_buf[4] : 0, (log_rx_bytes > 5) ? rx_buf[5] : 0,
                        (log_rx_bytes > 6) ? rx_buf[6] : 0, (log_rx_bytes > 7) ? rx_buf[7] : 0, status.value,
                        status.ResetFlag() ? "RST " : "", status.DriverError() ? "DRV_ERR " : "",
                        status.StallGuard2() ? "SG2 " : "", status.Standstill() ? "STST " : "",
                        status.VelocityReached() ? "VEL " : "", status.PositionReached() ? "POS " : "",
                        status.StopLeft() ? "STOP_L " : "", status.StopRight() ? "STOP_R " : "");
    }

    // Extract 32-bit value from bytes (response_byte_offset+1) to (response_byte_offset+4)
    // Byte (response_byte_offset+0) contains SPI_STATUS (bits 39-32)
    // Bytes (response_byte_offset+1) to (response_byte_offset+4) contain data (bits 31-0)
    if (response_byte_offset + 4 >= rx_buf.size()) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "Read register 0x%02X: Data offset %zu+4 exceeds buffer size %zu", address,
                        response_byte_offset, rx_buf.size());
      return false;
    }

    // Extract 32-bit value from RX2 (bytes response_byte_offset+1 to response_byte_offset+4)
    value = (static_cast<uint32_t>(rx_buf[response_byte_offset + 1]) << 24) |
            (static_cast<uint32_t>(rx_buf[response_byte_offset + 2]) << 16) |
            (static_cast<uint32_t>(rx_buf[response_byte_offset + 3]) << 8) |
            static_cast<uint32_t>(rx_buf[response_byte_offset + 4]);

    // Return false if critical errors detected (but still extract value)
    // Note: Reset flag is informational (normal on power-up), not an error
    return !status.DriverError();
  }

  /**
   * @brief Write a 32-bit register via SPI
   * @param address Register address (0x00-0x73)
   * @param value 32-bit value to write
   * @param daisy_chain_position Position in daisy chain (0 = first chip/single chip, default: 0)
   *                             Specifies which chip in the chain to address
   * @return true if write succeeded, false otherwise
   *
   * This method handles both single-chip and daisy-chain modes:
   * - Single chip (daisy_chain_position = 0): Standard 40-bit SPI transaction (5 bytes)
   * - Daisy-chain (daisy_chain_position > 0): Sends command with padding to shift
   *   it to the target device position in the chain
   *
   * **Daisy-Chain Transfer Logic:**
   * Same as ReadRegister() - see ReadRegister() documentation for detailed explanation.
   *
   * **Key Points:**
   * - Chain length MUST be known (auto-detected if not set)
   * - Sending: (k+1)*5 bytes to address device k
   * - Receiving: (n-k)*5 bytes total, response at offset (n-k-1)*5 bytes
   * - Transfer size: max((k+1)*5, (n-k)*5) bytes
   * - Extra bytes beyond (k+1)*5 are padding (zeros) for full-duplex behavior
   *
   * **Auto-Detection and Verification:**
   * Same as ReadRegister() - chain length is auto-detected on first access if needed,
   * and user-specified length is verified against detected length.
   * If detection fails, operation returns false.
   *
   * **Architecture Note:**
   * This SpiCommInterface instance can be shared by multiple TMC5160 drivers on the
   * same SPI bus. Each TMC5160 instance has its own daisy_chain_position_ and passes
   * it as a parameter when calling ReadRegister() or WriteRegister().
   *
   * CSN is handled automatically by the SPI hardware or derived class implementation.
   * For daisy-chaining, all chips share the same CSN (tied together).
   */
  bool WriteRegister(uint8_t address, uint32_t value, uint8_t daisy_chain_position = 0) noexcept {
    // Log function call with arguments (level 3 = DEBUG, only shows at DEBUG log level)
    if (daisy_chain_position > 0) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI", "WriteRegister(0x%02X=0x%08X, daisy_chain=%u)", address,
                        value, daisy_chain_position);
    } else {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI", "WriteRegister(0x%02X=0x%08X)", address, value);
    }

    // CRITICAL: Chain length MUST be known for correct response extraction
    // Ensure chain length is known and verified (auto-detects if needed)
    if (!EnsureChainLengthKnown(daisy_chain_position, "WriteRegister")) {
      return false;
    }

    // Build command using SpiCommand structure (union-based frame)
    SpiCommand cmd = SpiCommand::Write(address, value);

    // CRITICAL: Chain length MUST be known at this point
    // If daisy_chain_position > 0, total_chain_length_ must be > 0 (detected or set)
    // If daisy_chain_position == 0, total_chain_length_ should be 1 (single chip)
    if (daisy_chain_position > 0 && total_chain_length_ == 0) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "WriteRegister: Chain length unknown for daisy_chain_position=%u. "
                        "Cannot proceed without chain length.",
                        daisy_chain_position);
      return false;
    }

    // Calculate transfer size and response offset using datasheet formula
    // Transfer size: max((k+1)*5, (n-k)*5) bytes
    //   - Sending: (k+1)*5 bytes needed to address device k
    //   - Receiving: (n-k)*5 bytes needed to shift response back (datasheet formula)
    //   - Use max() to ensure both requirements are met
    //   - Extra bytes beyond (k+1)*5 are padding (zeros) for full-duplex behavior
    // Response offset: (n-k-1)*5 bytes (reverse order: last device first)
    uint8_t n = total_chain_length_;
    uint8_t k = daisy_chain_position;

    // Validate: k must be < n (device position must be less than total chain length)
    if (k >= n) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "WriteRegister: Invalid daisy_chain_position=%u for chain length=%u. "
                        "Position must be < chain length.",
                        k, n);
      return false;
    }

    // Calculate transfer size: max of sending and receiving requirements
    size_t sending_bytes = static_cast<size_t>(k + 1) * 5;   // Command must reach device k
    size_t receiving_bytes = static_cast<size_t>(n - k) * 5; // Response extraction (datasheet formula)
    size_t transfer_bytes = std::max(sending_bytes, receiving_bytes);

    // Response offset: (n-k-1)*5 bytes (based on reverse order of devices)
    size_t response_byte_offset = static_cast<size_t>(n - k - 1) * 5;

    // Validate transfer size meets receiving requirement
    if (transfer_bytes < receiving_bytes) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "WriteRegister: Transfer size %zu bytes < receiving requirement %zu bytes. "
                        "Response extraction may fail.",
                        transfer_bytes, receiving_bytes);
      return false;
    }

    // Validate response offset is within buffer bounds
    if (response_byte_offset + 4 >= transfer_bytes) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "WriteRegister: Response offset %zu + 4 >= transfer size %zu. "
                        "Cannot read full 5-byte response.",
                        response_byte_offset, transfer_bytes);
      return false;
    }

    std::vector<uint8_t> tx_buf(transfer_bytes, 0);
    std::vector<uint8_t> rx_buf(transfer_bytes, 0);

    // Place command at the beginning (bytes 0-4)
    // For daisy-chain position k, the command is placed first, then padding (zeros)
    // Padding structure:
    //   - Bytes 5 to (k+1)*5-1: Padding to shift command to device k
    //   - Bytes (k+1)*5 to transfer_bytes-1: Additional padding for full-duplex response extraction
    //     (only present if transfer_bytes > (k+1)*5, i.e., when (n-k)*5 > (k+1)*5)
    cmd.GetFrame(tx_buf.data());
    // Bytes 5 onwards are padding (zeros) - already initialized to 0 by vector constructor

    // First transaction: Send write command (TX1), receive status (RX1)
    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), transfer_bytes)) {
      return false;
    }

    // Log [TX1]/RX1 after first transfer
    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                      "Write 0x%02X=0x%08X: [TX1] %02X %02X %02X %02X %02X / RX1 %02X %02X %02X %02X %02X", address,
                      value, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], rx_buf[response_byte_offset],
                      (response_byte_offset + 1 < rx_buf.size()) ? rx_buf[response_byte_offset + 1] : 0,
                      (response_byte_offset + 2 < rx_buf.size()) ? rx_buf[response_byte_offset + 2] : 0,
                      (response_byte_offset + 3 < rx_buf.size()) ? rx_buf[response_byte_offset + 3] : 0,
                      (response_byte_offset + 4 < rx_buf.size()) ? rx_buf[response_byte_offset + 4] : 0);

    // Extract response data based on daisy-chain position
    // IMPORTANT: Responses come back in REVERSE order (last device first, first device last)
    // Response offset is calculated above based on whether total_chain_length_ is known
    // - If total_chain_length_ > 0: Use datasheet formula, response at (n-k-1)*5 bytes
    // - If total_chain_length_ == 0: Use simplified approach, response at k*5 bytes (end of
    // transfer)
    if (response_byte_offset >= rx_buf.size()) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "Write register 0x%02X: Response offset %zu exceeds buffer size %zu", address,
                        response_byte_offset, rx_buf.size());
      return false;
    }

    // Extract SPI_STATUS from first transaction response
    // Per datasheet: First write response contains SPI_STATUS + dummy/previous data
    SpiStatus status1 = SpiStatus::FromByte(rx_buf[response_byte_offset]);

    // Note: RESET (bit 0) is informational (normal on power-up), only DRV_ERR (bit 1) is an error
    if (status1.HasError()) {
      // Build error flags string (only actual errors)
      const char* error_flags = "DRV_ERR";

      // Build informational flags string (reset + status flags)
      char info_flags[64] = "";
      if (status1.ResetFlag() || status1.StallGuard2() || status1.Standstill() || status1.VelocityReached() ||
          status1.PositionReached() || status1.StopLeft() || status1.StopRight()) {
        snprintf(info_flags, sizeof(info_flags), " [%s%s%s%s%s%s%s]", status1.ResetFlag() ? "RST " : "",
                 status1.StallGuard2() ? "SG2 " : "", status1.Standstill() ? "STST " : "",
                 status1.VelocityReached() ? "VEL " : "", status1.PositionReached() ? "POS " : "",
                 status1.StopLeft() ? "STOP_L " : "", status1.StopRight() ? "STOP_R " : "");
      }

      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI", "Write 0x%02X (TX1): STATUS=0x%02X ERROR=%s%s", address,
                        status1.value, error_flags, info_flags);
    } else {
      // Log response bytes (first 8 or all if less)
      size_t log_rx1_bytes = (rx_buf.size() < 8) ? rx_buf.size() : 8;
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                        "Write register 0x%02X (TX1): RX[0..%zu] %02X %02X %02X %02X %02X %02X %02X %02X | "
                        "SPI_STATUS=0x%02X [%s%s%s%s%s%s%s%s]",
                        address, log_rx1_bytes - 1, rx_buf[0], (log_rx1_bytes > 1) ? rx_buf[1] : 0,
                        (log_rx1_bytes > 2) ? rx_buf[2] : 0, (log_rx1_bytes > 3) ? rx_buf[3] : 0,
                        (log_rx1_bytes > 4) ? rx_buf[4] : 0, (log_rx1_bytes > 5) ? rx_buf[5] : 0,
                        (log_rx1_bytes > 6) ? rx_buf[6] : 0, (log_rx1_bytes > 7) ? rx_buf[7] : 0, status1.value,
                        status1.ResetFlag() ? "RST " : "", status1.DriverError() ? "DRV_ERR " : "",
                        status1.StallGuard2() ? "SG2 " : "", status1.Standstill() ? "STST " : "",
                        status1.VelocityReached() ? "VEL " : "", status1.PositionReached() ? "POS " : "",
                        status1.StopLeft() ? "STOP_L " : "", status1.StopRight() ? "STOP_R " : "");
    }

    // Minimum CSN high time: 2*tclk + 10ns (typically ~176ns with 12MHz clock)
    // Use 1us delay for safety
    this->DelayUs(1);

    // Second transaction: Send dummy read to receive write confirmation/status
    // The response from the second transaction contains the status/confirmation
    // for the write command sent in the first transaction
    // Clear tx_buf and place read command at the beginning (same position as write command)
    std::fill(tx_buf.begin(), tx_buf.end(), 0);
    SpiCommand read_cmd = SpiCommand::Read(address);
    read_cmd.GetFrame(tx_buf.data());
    // Padding (zeros) after byte 4 will shift this command to the target device

    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), transfer_bytes)) {
      return false;
    }

    // Log TX2/[RX2] after second transfer (RX2 contains the write confirmation)
    // Align TX2 line with TX1 line by padding address field
    SpiStatus status2 = SpiStatus::FromByte(rx_buf[response_byte_offset]);
    std::string status2_bits = status2.FormatStatusBits();

    // Align TX2 bytes with TX1 bytes: "Write 0xXX=0xXXXXXXXX: " (25) + "[TX1] " (6) = 31 chars to first byte
    // For TX2: "Write 0xXX: " (13) + padding to reach 31 = 18 spaces needed
    // But we want "TX2 " to align with "[TX1] ", so: "Write 0xXX: " (13) + 6 spaces + "TX2 " (4) = 23
    // To align bytes: "Write 0xXX: " (13) + 18 spaces = 31, then bytes start
    // Actually simpler: align "TX2" label with "[TX1]" label, then bytes naturally align
    // "Write 0xXX=0xXXXXXXXX: [TX1] " = 31, bytes at 31
    // "Write 0xXX:            TX2 " = 31 (13 + 18), bytes at 31 ✓
    TMC51X0_LOG_DEBUG(
        *static_cast<Derived*>(this), 3, "SPI",
        "Write 0x%02X:             TX2 %02X %02X %02X %02X %02X / [RX2] %02X %02X %02X %02X %02X (STATUS=0x%02X)",
        address, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], rx_buf[response_byte_offset],
        (response_byte_offset + 1 < rx_buf.size()) ? rx_buf[response_byte_offset + 1] : 0,
        (response_byte_offset + 2 < rx_buf.size()) ? rx_buf[response_byte_offset + 2] : 0,
        (response_byte_offset + 3 < rx_buf.size()) ? rx_buf[response_byte_offset + 3] : 0,
        (response_byte_offset + 4 < rx_buf.size()) ? rx_buf[response_byte_offset + 4] : 0,
        rx_buf[response_byte_offset]);

    // Log status bit breakdown with arrow pointing to STATUS byte
    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                      "                                                   └─> %s", status2_bits.c_str());

    // Validate response offset (status2 was already extracted above for logging)
    if (response_byte_offset >= rx_buf.size()) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                        "Write register 0x%02X (TX2): Response offset %zu exceeds buffer size %zu", address,
                        response_byte_offset, rx_buf.size());
      return false;
    }

    // Note: RESET (bit 0) is informational (normal on power-up), only DRV_ERR (bit 1) is an error
    // (status2 was already extracted above for logging)
    if (status2.HasError()) {
      // Build error flags string (only actual errors)
      const char* error_flags = "DRV_ERR";

      // Build informational flags string (reset + status flags)
      char info_flags[64] = "";
      if (status2.ResetFlag() || status2.StallGuard2() || status2.Standstill() || status2.VelocityReached() ||
          status2.PositionReached() || status2.StopLeft() || status2.StopRight()) {
        snprintf(info_flags, sizeof(info_flags), " [%s%s%s%s%s%s%s]", status2.ResetFlag() ? "RST " : "",
                 status2.StallGuard2() ? "SG2 " : "", status2.Standstill() ? "STST " : "",
                 status2.VelocityReached() ? "VEL " : "", status2.PositionReached() ? "POS " : "",
                 status2.StopLeft() ? "STOP_L " : "", status2.StopRight() ? "STOP_R " : "");
      }

      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI", "Write 0x%02X (TX2): STATUS=0x%02X ERROR=%s%s", address,
                        status2.value, error_flags, info_flags);
    } else {
      // Log response bytes (first 8 or all if less)
      size_t log_rx2_bytes = (rx_buf.size() < 8) ? rx_buf.size() : 8;
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                        "Write register 0x%02X (TX2): RX[0..%zu] %02X %02X %02X %02X %02X %02X %02X %02X | "
                        "SPI_STATUS=0x%02X [%s%s%s%s%s%s%s%s]",
                        address, log_rx2_bytes - 1, rx_buf[0], (log_rx2_bytes > 1) ? rx_buf[1] : 0,
                        (log_rx2_bytes > 2) ? rx_buf[2] : 0, (log_rx2_bytes > 3) ? rx_buf[3] : 0,
                        (log_rx2_bytes > 4) ? rx_buf[4] : 0, (log_rx2_bytes > 5) ? rx_buf[5] : 0,
                        (log_rx2_bytes > 6) ? rx_buf[6] : 0, (log_rx2_bytes > 7) ? rx_buf[7] : 0, status2.value,
                        status2.ResetFlag() ? "RST " : "", status2.DriverError() ? "DRV_ERR " : "",
                        status2.StallGuard2() ? "SG2 " : "", status2.Standstill() ? "STST " : "",
                        status2.VelocityReached() ? "VEL " : "", status2.PositionReached() ? "POS " : "",
                        status2.StopLeft() ? "STOP_L " : "", status2.StopRight() ? "STOP_R " : "");
    }

    // Per datasheet: Write access returns SPI_STATUS (byte 0) + previously written data (bytes 1-4)
    // "If the previous access was a write access, then the data read back mirrors the previously received write data."
    // Verify that the returned data matches what we wrote
    if (response_byte_offset + 4 < rx_buf.size()) {
      uint32_t returned_value = (static_cast<uint32_t>(rx_buf[response_byte_offset + 1]) << 24) |
                                (static_cast<uint32_t>(rx_buf[response_byte_offset + 2]) << 16) |
                                (static_cast<uint32_t>(rx_buf[response_byte_offset + 3]) << 8) |
                                static_cast<uint32_t>(rx_buf[response_byte_offset + 4]);

      if (returned_value != value) {
        TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                          "WriteRegister(0x%02X): Write verification failed - wrote 0x%08X, got back 0x%08X", address,
                          value, returned_value);
        // Don't fail the write operation, but log the mismatch for debugging
      } else {
        TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "SPI",
                          "WriteRegister(0x%02X): Write verification passed - wrote 0x%08X, got back 0x%08X", address,
                          value, returned_value);
      }
    }

    // Check for critical errors in the final status (status2 is the latched status after write)
    // Note: Reset flag is informational (normal on power-up), not an error
    if (status2.DriverError()) {
      return false;
    }

    return true;
  }

protected:
  /**
   * @brief Total number of devices in the daisy chain
   *
   * 0 = unknown/single chip (uses simplified approach)
   * >0 = total number of devices (uses datasheet formula 40·(n-k))
   *
   * This is CRITICAL for proper response extraction. When set correctly,
   * ReadRegister and WriteRegister use the optimal datasheet formula.
   */
  uint8_t total_chain_length_{0};          ///< Total number of devices in daisy chain (0 = unknown/single chip)
  uint8_t user_specified_chain_length_{0}; ///< User-specified chain length (0 = not specified, >0 =
                                           ///< user value)
  bool chain_length_verified_{false};      ///< Flag to track if chain length has been verified via
                                           ///< auto-detection

  /**
   * @brief Protected destructor
   */
  ~SpiCommInterface() = default;

  // Allow moving
  SpiCommInterface(SpiCommInterface&&) = default;
  SpiCommInterface& operator=(SpiCommInterface&&) = default;

public:
  // Prevent copying
  SpiCommInterface(const SpiCommInterface&) = delete;
  SpiCommInterface& operator=(const SpiCommInterface&) = delete;

private:
  /**
   * @brief Ensure chain length is known and verified for daisy-chain operations
   * @param daisy_chain_position Position in daisy chain (0 = first chip/single chip)
   * @param context Context string for logging (e.g., "ReadRegister", "WriteRegister")
   * @return true if chain length is known/verified, false if detection failed when required
   *
   * This function handles:
   * - Auto-detection if daisy_chain_position > 0 and chain length is unknown
   * - Setting chain length to 1 for single chip mode (daisy_chain_position == 0)
   * - Verification of user-specified chain length against auto-detected value
   * - Verification of set chain length against auto-detected value
   *
   * @note This function is defined in the class body, so it's implicitly inline.
   *       This allows compiler optimization while maintaining code clarity and
   *       avoiding duplication between ReadRegister and WriteRegister.
   */
  bool EnsureChainLengthKnown(uint8_t daisy_chain_position, const char* context) noexcept {
    // Auto-detect chain length on first access if needed
    if (daisy_chain_position > 0 && total_chain_length_ == 0) {
      uint8_t detected_length = AutoDetectChainLength(8); // Probe up to 8 devices
      if (detected_length == 0) {
        // Detection failed - cannot proceed without chain length
        TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                          "%s: Auto-detection failed, but daisy_chain_position=%u > 0. "
                          "Chain length is required for correct response extraction. Operation failed.",
                          context, daisy_chain_position);
        return false;
      }
    }

    // For single chip (daisy_chain_position == 0), chain length is not needed
    // But if it's set, ensure it's at least 1
    if (daisy_chain_position == 0 && total_chain_length_ == 0) {
      // Single chip mode - this is valid
      total_chain_length_ = 1;       // Set to 1 for single chip (n=1, k=0)
      chain_length_verified_ = true; // Single-chip mode doesn't need verification
    }

    // Verify chain length if user specified one
    if (!chain_length_verified_ && user_specified_chain_length_ > 0) {
      // User specified a chain length, verify it matches detected length
      uint8_t detected_length = AutoDetectChainLength(8);
      if (detected_length > 0 && detected_length != user_specified_chain_length_) {
        TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                          "%s: DAISY CHAIN LENGTH MISMATCH! "
                          "User specified: %u, Auto-detected: %u. "
                          "Response extraction will be incorrect. "
                          "Call SetDaisyChainLength(%u) to fix.",
                          context, user_specified_chain_length_, detected_length, detected_length);
        // Update to detected length and mark as verified (use detected value)
        total_chain_length_ = detected_length;
        chain_length_verified_ = true;
        // Continue with detected length (don't fail, but log error)
      } else if (detected_length > 0) {
        // Match confirmed
        chain_length_verified_ = true;
        TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 2, "SPI", "%s: Chain length verified: %u devices", context,
                          detected_length);
      }
    } else if (!chain_length_verified_ && total_chain_length_ > 0) {
      // Chain length was set but not verified yet, verify it now
      // Skip verification for single-chip mode (daisy_chain_position == 0, total_chain_length_ == 1)
      // Auto-detection can fail for single-chip setups, so we trust the set value
      if (daisy_chain_position == 0 && total_chain_length_ == 1) {
        // Single-chip mode - no verification needed
        chain_length_verified_ = true;
      } else {
        // Multi-chip mode - verify chain length
        uint8_t detected_length = AutoDetectChainLength(8);
        if (detected_length > 0 && detected_length != total_chain_length_) {
          TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI",
                            "%s: DAISY CHAIN LENGTH MISMATCH! "
                            "Specified: %u, Auto-detected: %u. "
                            "Updating to detected length.",
                            context, total_chain_length_, detected_length);
          total_chain_length_ = detected_length;
        }
        chain_length_verified_ = true;
      }
    }

    return true;
  }
};

/**
 * @brief CRTP-based UART implementation of TMC5160CommInterface
 *
 * Uses UART single wire interface per datasheet section 5.1.
 * Uses UART_TXD and UART_RXD signals; supports external transceivers via UART_TXEN.
 * Each byte is transmitted LSB...MSB, highest byte transmitted first.
 *
 * ## UART Mode Requirements
 *
 * For UART operation, the TMC5160 must be configured with:
 * - SD_MODE (pin 21): Must be LOW (0)
 * - SPI_MODE (pin 22): Must be LOW (0)
 * - When both are LOW, UART operation is enabled
 *
 * ## Pin Functions in UART Mode
 *
 * In UART mode (SD_MODE=0, SPI_MODE=0), certain pins take on special functions:
 * - SDI_CFG1 (pin 15) → NAI (Next Address Input): Input for address selection
 * - SDO_CFG0 (pin 16) → NAO (Next Address Output): Output that connects to next chip's NAI
 * - DIAG0_SWN (pin 26) → SWION (Single Wire I/O Negative): UART single wire interface
 * - DIAG1_SWP (pin 27) → SWIOP (Single Wire I/O Positive): UART single wire interface
 *
 * ## UART Daisy Chaining
 *
 * The TMC5160 supports daisy chaining up to 255 nodes in UART mode:
 * - First chip: NAI tied to GND → responds to address 0
 * - Each chip's NAO connects to the next chip's NAI
 * - After programming each chip, its NAO must be LOW to enable the next chip
 * - Program chips sequentially starting from address 0
 * - See datasheet section 5.4 for detailed addressing procedure
 *
 * UART Write Access Datagram (9 bytes per datasheet section 5.1.1):
 * - Byte 0: Sync nibble (bits 0-3: 1,0,1,0) + Reserved (bits 4-7) + NODEADDR (8 bits)
 *   - Sync pattern enables baud rate synchronization
 *   - Reserved bits are don't cares but included in CRC
 * - Byte 1: RW bit (bit 8 = 1 for write) + 7-bit register address
 * - Bytes 2-5: 32-bit data (high byte to low byte, MSB-first)
 * - Bytes 6-7: Reserved (don't cares but included in CRC)
 * - Byte 8: CRC8 checksum (CRC8-ATM polynomial 0x07, LSB to MSB)
 *
 * UART Read Access Request (5 bytes per datasheet section 5.1.2):
 * - Byte 0: Sync nibble + Reserved + NODEADDR
 * - Byte 1: RW bit (bit 8 = 0 for read) + 7-bit register address
 * - Bytes 2-3: Reserved (don't cares but included in CRC)
 * - Byte 4: CRC8 checksum
 *
 * UART Read Access Reply (9 bytes per datasheet section 5.1.2):
 * - Byte 0: Sync nibble + Reserved (0) + 0xFF (address code for master)
 * - Byte 1: Register address (0)
 * - Bytes 2-5: 32-bit data (high byte to low byte, MSB-first)
 * - Bytes 6-7: Reserved (0)
 * - Byte 8: CRC8 checksum
 *
 * CRC8: CRC8-ATM polynomial (x^8 + x^2 + x^1 + x^0 = 0x07)
 * - Applied LSB to MSB, including sync and addressing byte
 * - Sync nibble is assumed to always be correct
 *
 * Baud Rate (per datasheet section 5.1.1):
 * - Minimum: 9000 baud (assuming 20 MHz clock)
 * - Maximum: fCLK/16
 * - Automatically detected from sync frame timing
 * - Bit time calculated from start bit (1 to 0 transition) to end of sync frame
 *
 * Communication Reset (per datasheet section 5.1.1):
 * - Pause time > 63 bit times between start bits resets communication
 * - Minimum 12 bit times bus idle time required after reset
 * - Glitches (< 16 clock cycles) cause 12 bit time timeout
 *
 * SENDDELAY (per datasheet section 5.1.2):
 * - Programmable delay after read request before reply (multiples of 8 bit times)
 * - Default: 8 bit times
 * - Multi-node systems: Set SENDDELAY to minimum 2 for all nodes
 *
 * Example usage:
 * @code
 * class MyUART : public tmc5160::UartCommInterface<MyUART> {
 * public:
 *   CommMode mode() const noexcept { return CommMode::UART; }
 *   bool uartSend(...) { ... }
 *   bool uartReceive(...) { ... }
 *   bool gpioSet(...) { ... }
 *   bool gpioRead(...) { ... }
 *   void debugLog(...) { ... }
 *   void delayMs(...) { ... }
 *   void delayUs(...) { ... }
 * };
 * @endcode
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class UartCommInterface : public CommInterface<Derived> {
public:
  /**
   * @brief Construct UART communication interface with pin active level
   * configuration
   * @param en_active_level Physical GPIO level for EN pin when ACTIVE
   * (true=HIGH, false=LOW)
   * @param dir_active_level Physical GPIO level for DIR pin when ACTIVE
   * (true=HIGH, false=LOW)
   * @param step_active_level Physical GPIO level for STEP pin when ACTIVE
   * (true=HIGH, false=LOW)
   *
   * @note Multiple TMC5160 instances can share one UartCommInterface on the same
   *       UART bus. The node address is passed per transaction via ReadRegister()
   *       and WriteRegister() methods.
   */
  UartCommInterface() noexcept : CommInterface<Derived>() {}

  /**
   * @brief Get communication mode (always UART for this interface)
   * @return CommMode::UART
   */
  [[nodiscard]] CommMode GetMode() const noexcept {
    return CommMode::UART;
  }

  /**
   * @brief Set NAI (Next Address Input) pin state for daisy chaining
   * @param active true to set NAI active (high), false to set inactive (low)
   * @return true if NAI was set successfully
   *
   * For UART daisy chaining, NAI controls the addressing sequence.
   * - NAI is the SDI_CFG1 pin (pin 15) in UART mode
   * - When NAI is active (high), the slave address increments by one
   * - First chip in chain: NAI should be tied to GND (low) → responds to address 0
   * - Subsequent chips: NAI connected to previous chip's NAO
   *
   * Default implementation delegates to derived class. Override for
   * hardware-specific NAI pin control.
   *
   * @note In UART mode, SD_MODE=0 and SPI_MODE=0 must be set
   */
  bool SetNaiPin(bool active) noexcept {
    return static_cast<Derived*>(this)->SetNaiPin(active);
  }

  /**
   * @brief Read NAO (Next Address Output) pin state
   * @param active Reference to store NAO pin state
   * @return true if NAO was read successfully
   *
   * NAO is the output from one TMC5160 that connects to the next
   * TMC5160's NAI input in a daisy chain.
   * - NAO is the SDO_CFG0 pin (pin 16) in UART mode
   * - After programming a chip to its target address, NAO must be LOW
   *   to enable the next chip in the chain
   * - NAO HIGH: Next chip is not yet accessible
   * - NAO LOW: Next chip is accessible for programming
   *
   * Default implementation delegates to derived class. Override for
   * hardware-specific NAO pin reading.
   *
   * @note In UART mode, SD_MODE=0 and SPI_MODE=0 must be set
   */
  bool GetNaoPin(bool& active) noexcept {
    return static_cast<Derived*>(this)->GetNaoPin(active);
  }

  /**
   * @brief Send raw bytes via UART
   * @param data Pointer to data bytes to send
   * @param length Number of bytes to send
   * @return true if transmission succeeded
   */
  bool UartSend(const uint8_t* data, size_t length) noexcept {
    return static_cast<Derived*>(this)->UartSend(data, length);
  }

  /**
   * @brief Receive raw bytes via UART
   * @param data Pointer to buffer to store received bytes
   * @param length Number of bytes to receive
   * @return true if reception succeeded
   */
  bool UartReceive(uint8_t* data, size_t length) noexcept {
    return static_cast<Derived*>(this)->UartReceive(data, length);
  }

  /**
   * @brief Read a 32-bit register via UART
   * @param address Register address (0x00-0x73)
   * @param value Reference to store the read value
   * @param node_address UART node address (0-127) for multi-node addressing
   * @return true if read succeeded, false otherwise
   */
  bool ReadRegister(uint8_t address, uint32_t& value, uint8_t node_address = 0) noexcept {
    // Build read request frame (4 bytes)
    uint8_t node_addr = node_address & 0x7F;
    UartFrame read_request = UartFrame::ReadRequest(node_addr, address);

    // Get frame bytes (4 bytes)
    std::array<uint8_t, 8> tx_buf{}; // Max size for any frame is 8
    read_request.GetFrame(tx_buf.data());
    size_t tx_size = read_request.GetSize(); // 4 bytes

    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "UART",
                      "Read register 0x%02X (NodeAddr=0x%02X): TX %02X %02X %02X %02X", address, node_addr, tx_buf[0],
                      tx_buf[1], tx_buf[2], tx_buf[3]);

    if (!UartSend(tx_buf.data(), tx_size)) {
      return false;
    }

    // Receive read reply (8 bytes)
    std::array<uint8_t, 8> rx_buf{};
    if (!UartReceive(rx_buf.data(), 8)) {
      return false;
    }

    // Parse read reply using UartFrame structure
    UartFrame read_reply = UartFrame::ReadReply(rx_buf.data());

    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "UART",
                      "Read register 0x%02X: RX %02X %02X %02X %02X %02X %02X %02X %02X", address, rx_buf[0], rx_buf[1],
                      rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

    // Verify CRC8 and frame validity
    if (!read_reply.VerifyCrc()) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "UART", "Read register 0x%02X: CRC8 verification failed",
                        address);
      return false;
    }

    if (!read_reply.IsValid()) {
      TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 1, "UART", "Read register 0x%02X: Invalid frame structure",
                        address);
      return false;
    }

    // Extract 32-bit value
    value = read_reply.GetValue();

    return true;
  }

  /**
   * @brief Write a 32-bit register via UART
   * @param address Register address (0x00-0x73)
   * @param value 32-bit value to write
   * @param node_address UART node address (0-127) for multi-node addressing
   * @return true if write succeeded, false otherwise
   */
  bool WriteRegister(uint8_t address, uint32_t value, uint8_t node_address = 0) noexcept {
    // Build write access frame (8 bytes)
    uint8_t node_addr = node_address & 0x7F;
    UartFrame write_frame = UartFrame::Write(node_addr, address, value);

    // Get frame bytes (8 bytes)
    std::array<uint8_t, 8> tx_buf{};
    write_frame.GetFrame(tx_buf.data());
    size_t tx_size = write_frame.GetSize(); // 8 bytes

    TMC51X0_LOG_DEBUG(*static_cast<Derived*>(this), 3, "UART",
                      "Write register 0x%02X = 0x%08X (NodeAddr=0x%02X): TX %02X %02X %02X %02X "
                      "%02X %02X %02X %02X",
                      address, value, node_addr, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], tx_buf[5],
                      tx_buf[6], tx_buf[7]);

    if (!UartSend(tx_buf.data(), tx_size)) {
      return false;
    }

    // Write does NOT have a reply packet from the device (only updates internal counter).
    // We return true if send was successful.
    // Note: Some single-wire implementations receive their own TX (echo).
    // If so, the derived class or HAL should handle flushing the echo.
    // This interface assumes UartSend handles the transmission.

    return true;
  }

protected:
  /**
   * @brief Protected destructor
   */
  ~UartCommInterface() = default;

  // Allow moving
  UartCommInterface(UartCommInterface&&) = default;
  UartCommInterface& operator=(UartCommInterface&&) = default;

public:
  // Prevent copying
  UartCommInterface(const UartCommInterface&) = delete;
  UartCommInterface& operator=(const UartCommInterface&) = delete;
};

} // namespace tmc51x0

#endif // TMC51X0_COMM_INTERFACE_HPP
