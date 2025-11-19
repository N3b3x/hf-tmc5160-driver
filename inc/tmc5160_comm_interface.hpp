/**
 * @file tmc5160_comm_interface.hpp
 * @brief Communication interfaces for TMC5160 stepper motor driver using SPI
 * and UART
 *
 * This file provides comprehensive communication interfaces for the TMC5160
 * motor driver, supporting both SPI and UART protocols with register read/write
 * operations. It includes GPIO control interfaces and board-agnostic pin
 * management for different hardware implementations.
 *
 * ## Compile-Time Configuration
 *
 * **TMC5160_DISABLE_DEBUG_LOGGING**: Define this macro to completely disable
 * all debug logging throughout the TMC5160 library. This removes all logging
 * code from the binary at compile time, saving code size and improving
 * performance.
 *
 * Example usage:
 * ```cpp
 * #define TMC5160_DISABLE_DEBUG_LOGGING
 * #include "tmc5160_comm_interface.hpp"
 * ```
 *
 * @defgroup TMC5160_CommInterface Communication Interfaces
 * @brief Core communication interface classes and protocols
 *
 * @defgroup TMC5160_GPIOControl GPIO Control Interface
 * @brief GPIO pin control and signal management
 *
 * @defgroup TMC5160_CommTypes Type Definitions
 * @brief Enums and type definitions for communication interfaces
 *
 * ## SPI Protocol
 *
 * TMC5160 uses an 8-byte SPI protocol:
 * - Byte 0: Register address (bit 7 = 0 for read, 1 for write)
 * - Bytes 1-4: 32-bit data (MSB-first)
 * - Bytes 5-7: Unused (dummy bytes for read)
 *
 * SPI Mode: Mode 3 (CPOL=1, CPHA=1)
 * Clock: MSB-first
 *
 * ## UART Protocol
 *
 * TMC5160 uses a 9-byte UART protocol:
 * - Byte 0: Sync bit (bit 0) + Slave address (bits 1-7)
 * - Byte 1: Register address (bit 7 = 0 for read, 1 for write)
 * - Bytes 2-5: 32-bit data (MSB-first)
 * - Byte 6-7: Unused
 * - Byte 8: CRC8 checksum
 *
 * Baud rate: Typically 500000 bps
 */

#ifndef TMC5160_COMM_INTERFACE_HPP
#define TMC5160_COMM_INTERFACE_HPP

#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace tmc5160 {

/**
 * @brief Compile-time debug logging control for TMC5160 library
 *
 * Define TMC5160_DISABLE_DEBUG_LOGGING before including this header to
 * completely disable all debug logging. When disabled, all logDebug() calls are
 * optimized out at compile time, including argument evaluation.
 */
#ifndef TMC5160_DISABLE_DEBUG_LOGGING
// Debug logging enabled - use actual function call
#define TMC5160_LOG_DEBUG(comm_obj, level, tag, ...)                           \
  (comm_obj).LogDebug(level, tag, __VA_ARGS__)
#else
// Debug logging disabled - optimize out completely (arguments not evaluated)
#define TMC5160_LOG_DEBUG(comm_obj, level, tag, ...) ((void)0)
#endif

/**
 * @brief Supported physical communication modes for TMC5160
 */
enum class CommMode {
  SPI, ///< SPI (Serial Peripheral Interface) mode - 4-wire synchronous
       ///< communication
  UART ///< UART (Universal Asynchronous Receiver-Transmitter) mode - 2-wire
       ///< asynchronous communication
};

/**
 * @brief TMC5160 control pin identifiers with board-agnostic naming
 *
 * These pin identifiers abstract the physical pin assignments to provide
 * a consistent interface regardless of board implementation.
 */
enum class TMC5160CtrlPin {
  EN,  ///< Enable pin - Enables/disables motor driver outputs
  DIR, ///< Direction pin - Sets motor rotation direction
  STEP ///< Step pin - Step pulse input for external step/dir mode
};

/**
 * @brief GPIO signal states with board-agnostic naming
 */
enum class GpioSignal {
  INACTIVE = 0, ///< Inactive signal state (logical low)
  ACTIVE = 1    ///< Active signal state (logical high)
};

/**
 * @brief Calculate CRC8 checksum for UART communication
 *
 * TMC5160 uses CRC8 with polynomial 0x07 (CRC-8-CCITT).
 *
 * @param data Pointer to the data bytes to checksum
 * @param length Number of bytes to include in the checksum calculation
 * @return 8-bit CRC8 checksum value
 */
static constexpr uint8_t calculateCrc8(const uint8_t *data,
                                       size_t length) noexcept {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

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
template <typename Derived> class CommInterface {
public:
  /**
   * @brief Construct communication interface with pin active level
   * configuration
   * @param en_active_level Physical GPIO level for EN pin when ACTIVE
   * (true=HIGH, false=LOW)
   * @param dir_active_level Physical GPIO level for DIR pin when ACTIVE
   * (true=HIGH, false=LOW)
   * @param step_active_level Physical GPIO level for STEP pin when ACTIVE
   * (true=HIGH, false=LOW)
   */
  CommInterface(bool en_active_level, bool dir_active_level,
                bool step_active_level) noexcept
      : pinActiveLevels_{en_active_level, dir_active_level, step_active_level} {
  }

  /**
   * @brief Get the underlying communication mode used by this interface
   * @return Communication mode (CommMode::SPI or CommMode::UART)
   */
  CommMode GetMode() const noexcept {
    return static_cast<const Derived *>(this)->GetMode();
  }

  /**
   * @brief Read a 32-bit register from the TMC5160
   * @param address Register address (0x00-0x73)
   * @param value Reference to store the read value
   * @param chip_index Optional chip index for multi-chip setups (default: 0)
   * @return true if read succeeded, false otherwise
   */
  bool ReadRegister(uint8_t address, uint32_t &value,
                    uint8_t chip_index = 0) noexcept {
    return static_cast<Derived *>(this)->ReadRegister(address, value,
                                                       chip_index);
  }

  /**
   * @brief Write a 32-bit register to the TMC5160
   * @param address Register address (0x00-0x73)
   * @param value 32-bit value to write
   * @param chip_index Optional chip index for multi-chip setups (default: 0)
   * @return true if write succeeded, false otherwise
   */
  bool WriteRegister(uint8_t address, uint32_t value,
                     uint8_t chip_index = 0) noexcept {
    return static_cast<Derived *>(this)->WriteRegister(address, value,
                                                       chip_index);
  }

  /**
   * @brief Set GPIO pin signal state (output control)
   * @param pin The TMC5160 control pin to control
   * @param signal The desired signal state (ACTIVE or INACTIVE)
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool GpioSet(TMC5160CtrlPin pin, GpioSignal signal) noexcept {
    return static_cast<Derived *>(this)->GpioSet(pin, signal);
  }

  /**
   * @brief Read GPIO pin signal state (input state)
   * @param pin The TMC5160 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully, false otherwise
   */
  bool GpioRead(TMC5160CtrlPin pin, GpioSignal &signal) noexcept {
    return static_cast<Derived *>(this)->GpioRead(pin, signal);
  }

  /**
   * @brief Convert signal state to physical GPIO level
   * @param pin The TMC5160 control pin
   * @param signal The signal state (ACTIVE or INACTIVE)
   * @return Physical GPIO level (true=HIGH, false=LOW)
   */
  bool SignalToGpioLevel(TMC5160CtrlPin pin, GpioSignal signal) const noexcept {
    bool active_level = pinActiveLevels_[static_cast<int>(pin)];
    return (signal == GpioSignal::ACTIVE) ? active_level : !active_level;
  }

  /**
   * @brief Convert physical GPIO level to signal state
   * @param pin The TMC5160 control pin
   * @param gpio_level Physical GPIO level (true=HIGH, false=LOW)
   * @return Signal state (ACTIVE or INACTIVE)
   */
  GpioSignal GpioLevelToSignal(TMC5160CtrlPin pin,
                               bool gpio_level) const noexcept {
    bool active_level = pinActiveLevels_[static_cast<int>(pin)];
    return (gpio_level == active_level) ? GpioSignal::ACTIVE
                                        : GpioSignal::INACTIVE;
  }

  /**
   * @brief Set GPIO pin to active state (convenience method)
   * @param pin The TMC5160 control pin to set active
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool GpioSetActive(TMC5160CtrlPin pin) noexcept {
    return GpioSet(pin, GpioSignal::ACTIVE);
  }

  /**
   * @brief Set GPIO pin to inactive state (convenience method)
   * @param pin The TMC5160 control pin to set inactive
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool GpioSetInactive(TMC5160CtrlPin pin) noexcept {
    return GpioSet(pin, GpioSignal::INACTIVE);
  }

  /**
   * @brief Configure the active level for a specific pin
   * @param pin The TMC5160 control pin to configure
   * @param active_level The physical GPIO level that represents ACTIVE state
   * @return true if the configuration was successful, false otherwise
   */
  bool SetPinActiveLevel(TMC5160CtrlPin pin, bool active_level) noexcept {
    pinActiveLevels_[static_cast<int>(pin)] = active_level;
    return true;
  }

protected:
  /**
   * @brief Pin active level configuration storage
   *
   * Stores the physical GPIO level (HIGH or LOW) that corresponds to the
   * ACTIVE state for each TMC5160 control pin.
   *
   * Array indices: [EN, DIR, STEP]
   */
  bool pinActiveLevels_[3];

  /**
   * @brief Debug logging function for detailed debugging information
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param args Variable arguments list
   */
  void DebugLog(int level, const char *tag, const char *format,
                va_list args) noexcept {
    static_cast<Derived *>(this)->DebugLog(level, tag, format, args);
  }

public:
  /**
   * @brief Delay execution for specified milliseconds
   * @param ms Milliseconds to delay
   */
  void DelayMs(uint32_t ms) noexcept {
    static_cast<Derived *>(this)->DelayMs(ms);
  }

  /**
   * @brief Delay execution for specified microseconds
   * @param us Microseconds to delay
   */
  void DelayUs(uint32_t us) noexcept {
    static_cast<Derived *>(this)->DelayUs(us);
  }

protected:
  /**
   * @brief Protected destructor
   */
  ~CommInterface() = default;

  // Prevent copying
  CommInterface(const CommInterface &) = delete;
  CommInterface &operator=(const CommInterface &) = delete;

  // Allow moving
  CommInterface(CommInterface &&) = default;
  CommInterface &operator=(CommInterface &&) = default;

public:
  /**
   * @brief Public debug logging wrapper for external classes
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param ... Variable arguments for format string
   */
#ifndef TMC5160_DISABLE_DEBUG_LOGGING
  void LogDebug(int level, const char *tag, const char *format, ...) noexcept {
    va_list args;
    va_start(args, format);

    // Check if format string already ends with newline
    size_t format_len = strlen(format);
    const char *final_format = format;
    char *modified_format = nullptr;

    if (format_len == 0 || format[format_len - 1] != '\n') {
      // Allocate buffer for format + "\n"
      modified_format = new char[format_len + 2];
      strcpy(modified_format, format);
      strcat(modified_format, "\n");
      final_format = modified_format;
    }

    DebugLog(level, tag, final_format, args);

    if (modified_format) {
      delete[] modified_format;
    }

    va_end(args);
  }
#else
  // Debug logging disabled - function optimized out completely
  inline void LogDebug(int /*level*/, const char * /*tag*/,
                       const char * /*format*/, ...) noexcept {
    // Empty function body - all logging optimized out
  }
#endif
};

/**
 * @brief CRTP-based SPI implementation of TMC5160CommInterface
 *
 * Uses a 4-wire SPI bus (mode 3) to exchange 8-byte datagrams.
 * Data is sent MSB-first, big-endian.
 *
 * SPI Protocol:
 * - Read: Send address (bit 7 = 0), receive 4 bytes + 3 dummy bytes
 * - Write: Send address | 0x80 (bit 7 = 1), send 4 bytes
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
   * @brief Construct SPI communication interface with pin active level
   * configuration
   * @param en_active_level Physical GPIO level for EN pin when ACTIVE
   * (true=HIGH, false=LOW)
   * @param dir_active_level Physical GPIO level for DIR pin when ACTIVE
   * (true=HIGH, false=LOW)
   * @param step_active_level Physical GPIO level for STEP pin when ACTIVE
   * (true=HIGH, false=LOW)
   */
  SpiCommInterface(bool en_active_level, bool dir_active_level,
                   bool step_active_level) noexcept
      : CommInterface<Derived>(en_active_level, dir_active_level,
                               step_active_level) {}

  /**
   * @brief Get communication mode (always SPI for this interface)
   * @return CommMode::SPI
   */
  CommMode GetMode() const noexcept { return CommMode::SPI; }

  /**
   * @brief Low-level SPI transfer for register read/write
   * @param tx Buffer containing bytes to transmit
   * @param rx Buffer to receive bytes from device
   * @param length Number of bytes to transfer
   * @return true if the SPI transfer completed successfully
   */
  bool SpiTransfer(const uint8_t *tx, uint8_t *rx, size_t length) noexcept {
    return static_cast<Derived *>(this)->SpiTransfer(tx, rx, length);
  }

  /**
   * @brief Set chip select (CSN) pin state for daisy chaining
   * @param csn_pin_index Index of the CSN pin (0-based) for multi-chip setups
   * @param active true to assert CSN (active low), false to deassert
   * @return true if CSN was set successfully
   *
   * For multi-chip SPI setups, this allows selecting which TMC5160 chip
   * to communicate with. The default implementation assumes single-chip
   * operation (index 0). Override in derived class for multi-chip support.
   */
  bool SetChipSelect(uint8_t csn_pin_index, bool active) noexcept {
    // Default: single-chip operation, ignore chip index
    if (csn_pin_index == 0) {
      return true; // Single chip, CSN handled by hardware
    }
    // Multi-chip: delegate to derived class
    return static_cast<Derived *>(this)->SetChipSelect(csn_pin_index, active);
  }

  /**
   * @brief Read a 32-bit register via SPI with optional chip selection
   * @param address Register address (0x00-0x73)
   * @param value Reference to store the read value
   * @param csn_pin_index Optional chip select index for daisy chaining (default: 0)
   * @return true if read succeeded, false otherwise
   */
  bool ReadRegister(uint8_t address, uint32_t &value,
                    uint8_t csn_pin_index = 0) noexcept {
    // Select chip if multi-chip setup
    if (csn_pin_index > 0) {
      SetChipSelect(csn_pin_index, true); // Assert CSN
    }

    std::array<uint8_t, 8> tx_buf{};
    std::array<uint8_t, 8> rx_buf{};

    // First transaction: Send read address, receive dummy data
    tx_buf[0] = address & 0x7F; // Clear write bit
    for (size_t i = 1; i < 8; ++i) {
      tx_buf[i] = 0x00;
    }

    TMC5160_LOG_DEBUG(
        *static_cast<Derived *>(this), 3, "SPI",
        "Read register 0x%02X (CSN=%u): TX %02X %02X %02X %02X %02X %02X %02X %02X",
        address, csn_pin_index, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
        tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]);

    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), 8)) {
      if (csn_pin_index > 0) {
        SetChipSelect(csn_pin_index, false); // Deassert CSN on error
      }
      return false;
    }

    // Minimum CSN high time: 2*tclk + 10ns (typically ~176ns with 12MHz clock)
    // Use 1us delay for safety
    this->DelayUs(1);

    // Second transaction: Send address again, receive actual data
    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), 8)) {
      if (csn_pin_index > 0) {
        SetChipSelect(csn_pin_index, false); // Deassert CSN on error
      }
      return false;
    }

    // Deassert CSN if multi-chip setup
    if (csn_pin_index > 0) {
      SetChipSelect(csn_pin_index, false);
    }

    TMC5160_LOG_DEBUG(
        *static_cast<Derived *>(this), 3, "SPI",
        "Read register 0x%02X: RX %02X %02X %02X %02X %02X %02X %02X %02X",
        address, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4],
        rx_buf[5], rx_buf[6], rx_buf[7]);

    // Extract 32-bit value from bytes 1-4 (MSB-first)
    value = (static_cast<uint32_t>(rx_buf[1]) << 24) |
            (static_cast<uint32_t>(rx_buf[2]) << 16) |
            (static_cast<uint32_t>(rx_buf[3]) << 8) |
            static_cast<uint32_t>(rx_buf[4]);

    return true;
  }

  /**
   * @brief Write a 32-bit register via SPI with optional chip selection
   * @param address Register address (0x00-0x73)
   * @param value 32-bit value to write
   * @param csn_pin_index Optional chip select index for daisy chaining (default: 0)
   * @return true if write succeeded, false otherwise
   */
  bool WriteRegister(uint8_t address, uint32_t value,
                     uint8_t csn_pin_index = 0) noexcept {
    // Select chip if multi-chip setup
    if (csn_pin_index > 0) {
      SetChipSelect(csn_pin_index, true); // Assert CSN
    }

    std::array<uint8_t, 8> tx_buf{};
    std::array<uint8_t, 8> rx_buf{};

    // First transaction: Send write command, receive dummy/previous data
    tx_buf[0] = (address & 0x7F) | 0x80; // Set write bit
    tx_buf[1] = static_cast<uint8_t>((value >> 24) & 0xFF);
    tx_buf[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    tx_buf[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
    tx_buf[4] = static_cast<uint8_t>(value & 0xFF);
    for (size_t i = 5; i < 8; ++i) {
      tx_buf[i] = 0x00;
    }

    TMC5160_LOG_DEBUG(*static_cast<Derived *>(this), 3, "SPI",
                      "Write register 0x%02X = 0x%08X (CSN=%u): TX %02X %02X %02X %02X "
                      "%02X %02X %02X %02X",
                      address, value, csn_pin_index, tx_buf[0], tx_buf[1],
                      tx_buf[2], tx_buf[3], tx_buf[4], tx_buf[5], tx_buf[6],
                      tx_buf[7]);

    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), 8)) {
      if (csn_pin_index > 0) {
        SetChipSelect(csn_pin_index, false); // Deassert CSN on error
      }
      return false;
    }

    TMC5160_LOG_DEBUG(
        *static_cast<Derived *>(this), 3, "SPI",
        "Write register 0x%02X (TX1): RX %02X %02X %02X %02X %02X %02X %02X %02X",
        address, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4],
        rx_buf[5], rx_buf[6], rx_buf[7]);

    // Minimum CSN high time: 2*tclk + 10ns (typically ~176ns with 12MHz clock)
    // Use 1us delay for safety
    this->DelayUs(1);

    // Second transaction: Send dummy read to receive write confirmation/status
    // The response from the second transaction contains the status/confirmation
    // for the write command sent in the first transaction
    tx_buf[0] = address & 0x7F; // Read address (clear write bit)
    for (size_t i = 1; i < 8; ++i) {
      tx_buf[i] = 0x00;
    }

    if (!SpiTransfer(tx_buf.data(), rx_buf.data(), 8)) {
      if (csn_pin_index > 0) {
        SetChipSelect(csn_pin_index, false); // Deassert CSN on error
      }
      return false;
    }

    // Deassert CSN if multi-chip setup
    if (csn_pin_index > 0) {
      SetChipSelect(csn_pin_index, false);
    }

    TMC5160_LOG_DEBUG(
        *static_cast<Derived *>(this), 3, "SPI",
        "Write register 0x%02X (TX2): RX %02X %02X %02X %02X %02X %02X %02X %02X",
        address, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4],
        rx_buf[5], rx_buf[6], rx_buf[7]);

    return true;
  }

protected:
  /**
   * @brief Protected destructor
   */
  ~SpiCommInterface() = default;

  // Prevent copying
  SpiCommInterface(const SpiCommInterface &) = delete;
  SpiCommInterface &operator=(const SpiCommInterface &) = delete;

  // Allow moving
  SpiCommInterface(SpiCommInterface &&) = default;
  SpiCommInterface &operator=(SpiCommInterface &&) = default;
};

/**
 * @brief CRTP-based UART implementation of TMC5160CommInterface
 *
 * Uses UART_TXD and UART_RXD signals; supports external transceivers via
 * UART_TXEN. Frames consist of 9 bytes: sync+address, register address, 4-byte
 * data, 2 unused bytes, CRC8. LSB-first transmission; CRC8 checksum validation.
 *
 * UART Protocol:
 * - Byte 0: Sync bit (bit 0) + Slave address (bits 1-7)
 * - Byte 1: Register address (bit 7 = 0 for read, 1 for write)
 * - Bytes 2-5: 32-bit data (MSB-first)
 * - Bytes 6-7: Unused
 * - Byte 8: CRC8 checksum
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
   * @param slave_address 7-bit slave address (0-127)
   */
  UartCommInterface(bool en_active_level, bool dir_active_level,
                    bool step_active_level, uint8_t slave_address) noexcept
      : CommInterface<Derived>(en_active_level, dir_active_level,
                               step_active_level),
        slaveAddress_(slave_address & 0x7F) {}

  /**
   * @brief Get communication mode (always UART for this interface)
   * @return CommMode::UART
   */
  CommMode GetMode() const noexcept { return CommMode::UART; }

  /**
   * @brief Get current slave address
   * @return 7-bit slave address
   */
  uint8_t GetSlaveAddress() const noexcept { return slaveAddress_; }

  /**
   * @brief Set slave address
   * @param address 7-bit slave address (0-127)
   */
  void SetSlaveAddress(uint8_t address) noexcept {
    slaveAddress_ = address & 0x7F;
  }

  /**
   * @brief Set NAI (Next Address Input) pin state for daisy chaining
   * @param active true to set NAI active (high), false to set inactive (low)
   * @return true if NAI was set successfully
   *
   * For UART daisy chaining, NAI controls the addressing sequence.
   * When NAI is active, the slave address increments by one.
   *
   * Default implementation delegates to derived class. Override for
   * hardware-specific NAI pin control.
   */
  bool SetNaiPin(bool active) noexcept {
    return static_cast<Derived *>(this)->SetNaiPin(active);
  }

  /**
   * @brief Read NAO (Next Address Output) pin state
   * @param active Reference to store NAO pin state
   * @return true if NAO was read successfully
   *
   * NAO is the output from one TMC5160 that connects to the next
   * TMC5160's NAI input in a daisy chain.
   *
   * Default implementation delegates to derived class. Override for
   * hardware-specific NAO pin reading.
   */
  bool GetNaoPin(bool &active) noexcept {
    return static_cast<Derived *>(this)->GetNaoPin(active);
  }

  /**
   * @brief Send raw bytes via UART
   * @param data Pointer to data bytes to send
   * @param length Number of bytes to send
   * @return true if transmission succeeded
   */
  bool UartSend(const uint8_t *data, size_t length) noexcept {
    return static_cast<Derived *>(this)->UartSend(data, length);
  }

  /**
   * @brief Receive raw bytes via UART
   * @param data Pointer to buffer to store received bytes
   * @param length Number of bytes to receive
   * @return true if reception succeeded
   */
  bool UartReceive(uint8_t *data, size_t length) noexcept {
    return static_cast<Derived *>(this)->UartReceive(data, length);
  }

  /**
   * @brief Read a 32-bit register via UART
   * @param address Register address (0x00-0x73)
   * @param value Reference to store the read value
   * @return true if read succeeded, false otherwise
   */
  bool ReadRegister(uint8_t address, uint32_t &value) noexcept {
    std::array<uint8_t, 9> tx_buf{};
    std::array<uint8_t, 9> rx_buf{};

    // Build UART frame for read
    tx_buf[0] = (slaveAddress_ & 0x7F) | 0x01; // Sync bit + address
    tx_buf[1] = address & 0x7F;                // Register address (read)
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;
    tx_buf[4] = 0x00;
    tx_buf[5] = 0x00;
    tx_buf[6] = 0x00;
    tx_buf[7] = 0x00;
    tx_buf[8] = calculateCrc8(tx_buf.data(), 8);

    TMC5160_LOG_DEBUG(
        *static_cast<Derived *>(this), 3, "UART",
        "Read register 0x%02X: TX %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        address, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4],
        tx_buf[5], tx_buf[6], tx_buf[7], tx_buf[8]);

    if (!UartSend(tx_buf.data(), 9)) {
      return false;
    }

    if (!UartReceive(rx_buf.data(), 9)) {
      return false;
    }

    TMC5160_LOG_DEBUG(
        *static_cast<Derived *>(this), 3, "UART",
        "Read register 0x%02X: RX %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        address, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4],
        rx_buf[5], rx_buf[6], rx_buf[7], rx_buf[8]);

    // Verify CRC8
    uint8_t calculated_crc = calculateCrc8(rx_buf.data(), 8);
    if (calculated_crc != rx_buf[8]) {
      TMC5160_LOG_DEBUG(*static_cast<Derived *>(this), 1, "UART",
                        "CRC8 mismatch: calculated 0x%02X, received 0x%02X",
                        calculated_crc, rx_buf[8]);
      return false;
    }

    // Extract 32-bit value from bytes 2-5 (MSB-first)
    value = (static_cast<uint32_t>(rx_buf[2]) << 24) |
            (static_cast<uint32_t>(rx_buf[3]) << 16) |
            (static_cast<uint32_t>(rx_buf[4]) << 8) |
            static_cast<uint32_t>(rx_buf[5]);

    return true;
  }

  /**
   * @brief Write a 32-bit register via UART
   * @param address Register address (0x00-0x73)
   * @param value 32-bit value to write
   * @return true if write succeeded, false otherwise
   */
  bool WriteRegister(uint8_t address, uint32_t value) noexcept {
    std::array<uint8_t, 9> tx_buf{};
    std::array<uint8_t, 9> rx_buf{};

    // Build UART frame for write
    tx_buf[0] = (slaveAddress_ & 0x7F) | 0x01; // Sync bit + address
    tx_buf[1] = (address & 0x7F) | 0x80;       // Register address (write)
    tx_buf[2] = static_cast<uint8_t>((value >> 24) & 0xFF);
    tx_buf[3] = static_cast<uint8_t>((value >> 16) & 0xFF);
    tx_buf[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
    tx_buf[5] = static_cast<uint8_t>(value & 0xFF);
    tx_buf[6] = 0x00;
    tx_buf[7] = 0x00;
    tx_buf[8] = calculateCrc8(tx_buf.data(), 8);

    TMC5160_LOG_DEBUG(*static_cast<Derived *>(this), 3, "UART",
                      "Write register 0x%02X = 0x%08X: TX %02X %02X %02X %02X "
                      "%02X %02X %02X %02X %02X",
                      address, value, tx_buf[0], tx_buf[1], tx_buf[2],
                      tx_buf[3], tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7],
                      tx_buf[8]);

    if (!UartSend(tx_buf.data(), 9)) {
      return false;
    }

    if (!UartReceive(rx_buf.data(), 9)) {
      return false;
    }

    TMC5160_LOG_DEBUG(*static_cast<Derived *>(this), 3, "UART",
                      "Write register 0x%02X: RX %02X %02X %02X %02X %02X %02X "
                      "%02X %02X %02X",
                      address, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
                      rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7], rx_buf[8]);

    // Verify CRC8
    uint8_t calculated_crc = calculateCrc8(rx_buf.data(), 8);
    if (calculated_crc != rx_buf[8]) {
      TMC5160_LOG_DEBUG(*static_cast<Derived *>(this), 1, "UART",
                        "CRC8 mismatch: calculated 0x%02X, received 0x%02X",
                        calculated_crc, rx_buf[8]);
      return false;
    }

    return true;
  }

protected:
  /**
   * @brief Protected destructor
   */
  ~UartCommInterface() = default;

  // Prevent copying
  UartCommInterface(const UartCommInterface &) = delete;
  UartCommInterface &operator=(const UartCommInterface &) = delete;

  // Allow moving
  UartCommInterface(UartCommInterface &&) = default;
  UartCommInterface &operator=(UartCommInterface &&) = default;

private:
  uint8_t slaveAddress_; ///< 7-bit slave address for UART communication
};

} // namespace tmc5160

#endif // TMC5160_COMM_INTERFACE_HPP
