/**
 * @file tmc51x0_multi_node.hpp
 * @brief High-level multi-node manager for multiple TMC51x0 drivers on UART (TMC5130 & TMC5160)
 *
 * This file provides a TMC51x0MultiNode class that manages multiple TMC51x0 drivers
 * on a single UART bus using sequential addressing with NAI/NAO pins. It handles
 * proper device creation, sequential programming, and supports dynamic addition/removal
 * of devices. Supports both TMC5130 and TMC5160 chips.
 *
 * @defgroup TMC51X0_MultiNode Multi-Node Management
 * @brief High-level UART multi-node management
 */

#ifndef TMC51X0_MULTI_NODE_HPP
#define TMC51X0_MULTI_NODE_HPP

#include <array>
#include <cstdint>
#include <optional>

#include "tmc51x0.hpp"
#include "tmc51x0_comm_interface.hpp"

namespace tmc51x0 {

/**
 * @brief High-level manager for multiple TMC51x0 drivers in a UART multi-node configuration
 * @ingroup TMC51X0_MultiNode
 *
 * This class manages multiple TMC51x0 drivers on a single UART bus using sequential
 * addressing with NAI/NAO pins. It supports a fixed number of onboard devices (known
 * at construction) and allows dynamic addition/removal of extra devices up to a maximum
 * capacity.
 *
 * ## Key Features
 *
 * - **Onboard Devices**: Fixed number of devices created at construction time
 * - **Dynamic Devices**: Support for adding/removing extra devices at runtime
 * - **Sequential Programming**: Handles NAI/NAO pin control for sequential addressing
 * - **Individual Access**: Access individual drivers via operator[] using logical indices
 *
 * ## Architecture
 *
 * - **One UartCommInterface**: Shared by all TMC51x0 instances on the UART bus
 * - **Multiple TMC51x0 Instances**: One per device, each with its own programmed node address
 * - **Sequential Addressing**: Uses NAI/NAO pins for sequential programming per datasheet
 *
 * ## Important: Sequential Addressing (Per Datasheet)
 *
 * In UART multi-node mode, devices are programmed sequentially using NAI/NAO pins:
 *
 * **Initial State at Power-Up:**
 * - First chip: NAI tied to GND (hardware) → responds to address 0
 * - All other chips: NAI=HIGH (from previous chip's NAO) → respond to address 1
 *
 * **Programming Sequence (Per Datasheet Figure 5.1):**
 * - Program first chip (accessible at address 0) to address 254, NAO goes LOW
 * - Next chip becomes accessible at address 0 (because previous chip's NAO is LOW)
 * - Program second chip (accessible at address 0) to address 253, NAO goes LOW
 * - Continue: program chip i to address (254 - i)
 *
 * **Logical vs Physical Addressing:**
 * - Devices are accessed via operator[] using logical indices (0, 1, 2, ...)
 * - Physical addresses programmed into chips are (254, 253, 252, ...)
 * - This follows the datasheet procedure for addressing up to 255 nodes
 *
 * Users can create their own aliases/names in their code for better readability:
 *
 * @code
 * // Create multi-node manager with 3 onboard devices
 * tmc5160::TMC51x0MultiNode<MyUART, 5> nodes(uartComm, 3, 12'000'000);
 *
 * // Program all devices sequentially (required at startup)
 * nodes.ProgramSequentially();
 *
 * // Create user-friendly aliases for device addresses
 * auto& x_axis = nodes[0];  // Address 0 = X-axis motor
 * auto& y_axis = nodes[1];  // Address 1 = Y-axis motor
 * auto& z_axis = nodes[2];  // Address 2 = Z-axis motor
 *
 * // Use aliases for clearer code
 * x_axis.rampControl.SetTargetPosition(1000);
 * y_axis.rampControl.SetMaxSpeed(500.0f);
 * z_axis.motorControl.Enable();
 * @endcode
 *
 * ## Usage Example
 *
 * @code
 * // Create UART communication interface (shared by all devices)
 * class MyUART : public tmc5160::UartCommInterface<MyUART> { ... };
 * MyUART uartComm;
 * uartComm.Initialize();
 *
 * // Create multi-node manager with 3 onboard devices, capacity for 5 total
 * // Onboard devices are created with initial address 0 (will be programmed)
 * tmc5160::TMC51x0MultiNode<MyUART, 5> nodes(uartComm, 3, 12'000'000);
 *
 * // Program all devices sequentially (required at startup)
 * // This programs devices to addresses 254, 253, 252 per datasheet procedure
 * if (!nodes.ProgramSequentially()) {
 *   // Error handling
 * }
 *
 * // Initialize onboard devices
 * tmc51x0::DriverConfig cfg{};
 * cfg.motor.irun = 20;
 * cfg.motor.ihold = 10;
 * nodes.InitializeAll(cfg);
 *
 * // Add an extra device at runtime (must be index 3, next sequential index)
 * if (nodes.AddDevice(3)) { // Adds device at logical index 3
 *   // Program the new device (will be programmed to address 251 = 254 - 3)
 *   // Note: ProgramDevice() requires the device to be accessible at address 0
 *   // You may need to reprogram all devices or use ProgramSequentially() again
 *   nodes[3].Initialize(cfg);
 * }
 *
 * // Create user-friendly aliases
 * auto& motor_a = nodes[0];
 * auto& motor_b = nodes[1];
 * auto& motor_c = nodes[2];
 * auto& motor_d = nodes[3];
 *
 * // Access devices using aliases
 * motor_a.rampControl.SetTargetPosition(1000);
 * motor_b.rampControl.SetMaxSpeed(500.0f);
 *
 * // Remove extra device when done
 * nodes.RemoveDevice(3);
 * @endcode
 *
 * @tparam CommType The communication interface type (must be UartCommInterface<CommType>)
 * @tparam MaxDevices Maximum total capacity (onboard + extra devices, default: 8)
 */
template <typename CommType, size_t MaxDevices = 8>
class TMC51x0MultiNode {
public:
  /**
   * @brief Construct a multi-node manager
   * @param comm Reference to UART communication interface (shared by all devices)
   * @param num_onboard_devices Number of onboard devices (fixed, created at construction)
   * @param f_clk TMC5160 clock frequency in Hz (default: 12 MHz)
   *
   * @note Onboard devices are created immediately with initial address 0.
   *       Extra devices can be added/removed at runtime up to MaxDevices total.
   * @note After construction, call ProgramSequentially() to program all devices
   *       to addresses (254, 253, 252, ...) per datasheet procedure.
   */
  explicit TMC51x0MultiNode(CommType& comm, uint8_t num_onboard_devices,
                            uint32_t f_clk = ClockFreq::DEFAULT_F_CLK) noexcept
      : comm_(comm), num_onboard_devices_(num_onboard_devices), num_active_devices_(num_onboard_devices),
        f_clk_(f_clk) {
    // Validate num_onboard_devices
    if (num_onboard_devices == 0 || num_onboard_devices > MaxDevices) {
      num_onboard_devices_ = 1;
      num_active_devices_ = 1;
    }

    // Create onboard TMC5160 instances, one per device
    // Initial node address is 0 (will be programmed via ProgramSequentially())
    // The actual programmed addresses will be 254, 253, 252, ... per datasheet
    for (uint8_t i = 0; i < num_onboard_devices_; ++i) {
      drivers_[i] = std::make_optional<TMC51x0<CommType>>(comm_, f_clk_, 0, 0);
    }

    // Initialize extra device slots as empty
    for (size_t i = num_onboard_devices_; i < MaxDevices; ++i) {
      drivers_[i] = std::nullopt;
    }
  }

  /**
   * @brief Get the number of onboard devices (fixed at construction)
   * @return Number of onboard devices
   */
  [[nodiscard]] uint8_t GetNumOnboardDevices() const noexcept {
    return num_onboard_devices_;
  }

  /**
   * @brief Get the total number of active devices (onboard + extra)
   * @return Total number of active devices
   */
  [[nodiscard]] uint8_t GetNumActiveDevices() const noexcept {
    return num_active_devices_;
  }

  /**
   * @brief Get the maximum capacity (onboard + extra devices)
   * @return Maximum number of devices (MaxDevices template parameter)
   */
  [[nodiscard]] constexpr size_t GetMaxCapacity() const noexcept {
    return MaxDevices;
  }

  /**
   * @brief Check if a device slot is active (has a device instance)
   * @param index Device index (0 to MaxDevices-1)
   * @return true if device exists at this index, false otherwise
   */
  [[nodiscard]] bool IsDeviceActive(uint8_t index) const noexcept {
    if (index >= MaxDevices) {
      return false;
    }
    return drivers_[index].has_value();
  }

  /**
   * @brief Program all active devices sequentially using NAI/NAO addressing
   * @param send_delay SENDDELAY value for SLAVECONF (default: 2, minimum for multi-node)
   * @return true if all devices programmed successfully, false otherwise
   *
   * This method programs all active devices sequentially using the datasheet procedure:
   *
   * Initial state at power-up:
   * - First chip: NAI=GND (hardware) → responds to address 0
   * - All other chips: NAI=HIGH (from previous chip's NAO) → respond to address 1
   *
   * Programming sequence (per datasheet Figure 5.1):
   * 1. Program first chip (accessible at address 0) to address (254 - 0) = 254
   *    After programming, chip's NAO goes LOW → next chip becomes accessible at address 0
   * 2. Program second chip (accessible at address 0) to address (254 - 1) = 253
   *    After programming, chip's NAO goes LOW → next chip becomes accessible at address 0
   * 3. Continue for all devices: program chip i to address (254 - i)
   *
   * The devices are stored with their logical indices (0, 1, 2, ...) but programmed
   * with addresses (254, 253, 252, ...) as per datasheet specification.
   *
   * @note This must be called after construction and before using the devices.
   * @note For multi-node systems, SENDDELAY should be set to minimum 2 for all nodes.
   * @note The programmed addresses (254, 253, 252...) are stored internally but the
   *       logical device indices (0, 1, 2...) are used for access via operator[].
   */
  bool ProgramSequentially(uint8_t send_delay = 2) noexcept {
    // Count active devices to determine starting address
    uint8_t num_devices = 0;
    for (size_t i = 0; i < MaxDevices; ++i) {
      if (drivers_[i].has_value()) {
        num_devices++;
      }
    }

    if (num_devices == 0) {
      return false; // No devices to program
    }

    // Program devices in forward order (index 0, 1, 2, ...)
    // but assign addresses in reverse order (254, 253, 252, ...)
    // First chip: accessible at address 0 (NAI=GND)
    // After each programming, next chip becomes accessible at address 0

    uint8_t device_index = 0;
    for (size_t i = 0; i < MaxDevices; ++i) {
      if (!drivers_[i].has_value()) {
        continue; // Skip empty slots
      }

      // Calculate target address: start from 254 and count down
      // Device at index 0 gets address 254, index 1 gets 253, etc.
      uint8_t target_address = 254 - device_index;

      // Program device to target address
      // The device is currently accessible at address 0 (after previous chip's NAO went LOW)
      // or address 0 for the first chip (NAI=GND)
      if (!drivers_[i]->uartConfig.ConfigureUartNodeAddress(target_address, send_delay)) {
        return false; // Failed to program device
      }

      // Update the driver's node address to the programmed address
      drivers_[i]->SetUartNodeAddress(target_address);

      // After programming, the chip's NAO automatically goes LOW (per datasheet)
      // This makes the next chip accessible at address 0
      // Small delay to ensure NAO settles
      comm_.DelayMs(10);

      device_index++;
    }

    return true;
  }

  /**
   * @brief Program a single device at the specified logical index
   * @param index Logical device index (0, 1, 2, ...)
   * @param send_delay SENDDELAY value for SLAVECONF (default: 2)
   * @return true if device programmed successfully, false otherwise
   *
   * @note This method programs the device at logical index to address (254 - index).
   * @note The device must be accessible at address 0 (previous chip's NAO must be LOW).
   * @note For programming all devices, use ProgramSequentially() instead.
   */
  bool ProgramDevice(uint8_t index, uint8_t send_delay = 2) noexcept {
    if (index >= MaxDevices || !drivers_[index].has_value()) {
      return false;
    }

    // Calculate target address: 254 - index (per datasheet)
    uint8_t target_address = 254 - index;

    if (!drivers_[index]->uartConfig.ConfigureUartNodeAddress(target_address, send_delay)) {
      return false;
    }

    drivers_[index]->SetUartNodeAddress(target_address);
    return true;
  }

  /**
   * @brief Add an extra device at the specified logical index
   * @param index Logical device index (must be >= num_onboard_devices)
   * @return true if device was added successfully, false otherwise
   *
   * @note Index must be >= num_onboard_devices (cannot add before onboard devices)
   * @note Index must be < MaxDevices
   * @note Device slot must be empty (not already have a device)
   * @note Indices must be sequential - cannot skip indices
   *       (e.g., cannot add index 5 if index 4 is empty)
   *
   * @warning In UART multi-node mode, indices correspond to physical order (NAO→NAI).
   *          Indices MUST be sequential. The actual programmed addresses will be
   *          (254, 253, 252, ...) per datasheet procedure.
   */
  bool AddDevice(uint8_t index) noexcept {
    // Validate index
    if (index < num_onboard_devices_ || index >= MaxDevices) {
      return false;
    }

    // Check if slot is already occupied
    if (drivers_[index].has_value()) {
      return false;
    }

    // Enforce sequential indexing: cannot skip indices
    // All indices before this one (starting from num_onboard_devices) must be filled
    for (uint8_t i = num_onboard_devices_; i < index; ++i) {
      if (!drivers_[i].has_value()) {
        return false; // Cannot skip indices
      }
    }

    // Create device instance with initial address 0
    // The actual address will be programmed via ProgramSequentially() or ProgramDevice()
    drivers_[index] = std::make_optional<TMC51x0<CommType>>(comm_, f_clk_, 0, 0);
    num_active_devices_++;

    return true;
  }

  /**
   * @brief Remove an extra device at the specified logical index
   * @param index Logical device index
   * @return true if device was removed successfully, false otherwise
   *
   * @note Cannot remove onboard devices (index < num_onboard_devices)
   * @note Device slot must be active (have a device)
   * @note Can only remove from the end of the chain (highest index)
   *       to maintain sequential indexing
   *
   * @warning In UART multi-node mode, indices must remain sequential. You can only
   *          remove devices from the end of the chain (highest index).
   *          Removing a device in the middle would create a gap, which is
   *          not physically possible in a sequential addressing scheme.
   */
  bool RemoveDevice(uint8_t index) noexcept {
    // Cannot remove onboard devices
    if (index < num_onboard_devices_) {
      return false;
    }

    // Validate index
    if (index >= MaxDevices) {
      return false;
    }

    // Check if slot is actually active
    if (!drivers_[index].has_value()) {
      return false;
    }

    // Enforce sequential removal: can only remove from the end
    // Check if there are any devices after this index
    for (size_t i = index + 1; i < MaxDevices; ++i) {
      if (drivers_[i].has_value()) {
        return false; // Cannot remove device in the middle of the chain
      }
    }

    // Remove device
    drivers_[index] = std::nullopt;
    num_active_devices_--;

    return true;
  }

  /**
   * @brief Access individual TMC5160 driver by logical index
   * @param index Logical device index (0 = first device, 1 = second, etc.)
   * @return Reference to TMC5160 driver instance
   *
   * @note Index must be in range [0, num_active_devices-1] and device must be active.
   *       No bounds checking is performed for performance reasons.
   *       Use IsDeviceActive() to verify device exists before access.
   * @note The device's actual programmed address is (254 - index) per datasheet.
   */
  [[nodiscard]] TMC51x0<CommType>& operator[](uint8_t index) noexcept {
    return drivers_[index].value();
  }

  /**
   * @brief Const access to individual TMC5160 driver by logical index
   * @param index Logical device index (0 = first device, 1 = second, etc.)
   * @return Const reference to TMC5160 driver instance
   */
  [[nodiscard]] const TMC51x0<CommType>& operator[](uint8_t index) const noexcept {
    return drivers_[index].value();
  }

  /**
   * @brief Initialize all active devices with the same configuration
   * @param config Driver configuration (applied to all active devices)
   * @return true if all devices initialized successfully, false otherwise
   */
  bool InitializeAll(const DriverConfig& config = DriverConfig()) noexcept {
    bool all_success = true;
    for (size_t i = 0; i < MaxDevices; ++i) {
      if (drivers_[i].has_value()) {
        if (!drivers_[i]->Initialize(config)) {
          all_success = false;
        }
      }
    }
    return all_success;
  }

private:
  CommType& comm_;              ///< Shared UART communication interface
  uint8_t num_onboard_devices_; ///< Number of onboard devices (fixed)
  uint8_t num_active_devices_;  ///< Total number of active devices (onboard + extra)
  uint32_t f_clk_;              ///< TMC5160 clock frequency
  std::array<std::optional<TMC51x0<CommType>>, MaxDevices>
      drivers_; ///< TMC5160 driver instances (optional for dynamic devices)
};

} // namespace tmc51x0

#endif // TMC51X0_MULTI_NODE_HPP
