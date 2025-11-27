/**
 * @file tmc51x0_daisy_chain.hpp
 * @brief High-level daisy-chain manager for multiple TMC51x0 drivers (TMC5130 & TMC5160)
 *
 * This file provides a TMC51x0DaisyChain class that manages multiple TMC51x0 drivers
 * on a single SPI bus using daisy-chaining. It handles proper device creation,
 * chain length configuration, and supports dynamic addition/removal of devices.
 * Supports both TMC5130 and TMC5160 chips.
 *
 * @defgroup TMC51X0_DaisyChain Daisy-Chain Management
 * @brief High-level daisy-chain management
 */

#ifndef TMC51X0_DAISY_CHAIN_HPP
#define TMC51X0_DAISY_CHAIN_HPP

#include <array>
#include <cstdint>
#include <optional>

#include "tmc51x0.hpp"
#include "tmc51x0_comm_interface.hpp"

namespace tmc51x0 {

/**
 * @brief High-level manager for multiple TMC51x0 drivers in a daisy-chain configuration
 * @ingroup TMC51X0_DaisyChain
 *
 * This class manages multiple TMC51x0 drivers on a single SPI bus using daisy-chaining.
 * It supports a fixed number of onboard devices (known at construction) and allows
 * dynamic addition/removal of extra devices up to a maximum capacity.
 *
 * ## Key Features
 *
 * - **Onboard Devices**: Fixed number of devices created at construction time
 * - **Dynamic Devices**: Support for adding/removing extra devices at runtime
 * - **Proper Chain Length**: Automatically configures SpiCommInterface with total chain length
 * - **Individual Access**: Access individual drivers via operator[]
 *
 * ## Architecture
 *
 * - **One SpiCommInterface**: Shared by all TMC51x0 instances in the chain
 * - **Multiple TMC51x0 Instances**: One per device, each with its own position (0, 1, 2, ...)
 * - **Total Chain Length**: Automatically updated when devices are added/removed
 *
 * ## Important: Sequential Positioning
 *
 * In a daisy chain, devices are physically connected in sequence (MISO of one device
 * connects to MOSI of the next). Therefore, positions MUST be sequential (0, 1, 2, 3...).
 * The position corresponds to the physical order in the chain, not arbitrary numbering.
 *
 * - Onboard devices are always created at positions 0, 1, 2, ..., (num_onboard-1)
 * - Extra devices must be added sequentially starting from position num_onboard
 * - You cannot skip positions (e.g., cannot add device at position 5 if position 4 is empty)
 *
 * Users can create their own aliases/names in their code for better readability:
 *
 * @code
 * // Create daisy-chain with 3 onboard devices
 * tmc51x0::TMC51x0DaisyChain<MySPI, 5> chain(spiComm, 3, 12'000'000);
 *
 * // Create user-friendly aliases for device indices
 * auto& x_axis = chain[0];  // Position 0 = X-axis motor
 * auto& y_axis = chain[1];  // Position 1 = Y-axis motor
 * auto& z_axis = chain[2];  // Position 2 = Z-axis motor
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
 * // Create SPI communication interface (shared by all devices)
 * class MySPI : public tmc51x0::SpiCommInterface<MySPI> { ... };
 * MySPI spiComm;
 * spiComm.Initialize();
 *
 * // Create daisy-chain manager with 3 onboard devices, capacity for 5 total
 * // Onboard devices are created at positions 0, 1, 2 (sequential)
 * tmc51x0::TMC51x0DaisyChain<MySPI, 5> chain(spiComm, 3, 12'000'000);
 *
 * // Initialize onboard devices
 * tmc51x0::DriverConfig cfg{};
 * cfg.motor.irun = 20;
 * cfg.motor.ihold = 10;
 * chain.InitializeAll(cfg);
 *
 * // Add an extra device at runtime (must be position 3, next sequential position)
 * if (chain.AddDevice(3)) { // Adds device at position 3
 *   chain[3].Initialize(cfg);
 * }
 *
 * // Create user-friendly aliases
 * auto& motor_a = chain[0];
 * auto& motor_b = chain[1];
 * auto& motor_c = chain[2];
 * auto& motor_d = chain[3];
 *
 * // Access devices using aliases
 * motor_a.rampControl.SetTargetPosition(1000);
 * motor_b.rampControl.SetMaxSpeed(500.0f);
 *
 * // Remove extra device when done
 * chain.RemoveDevice(3);
 * @endcode
 *
 * @tparam CommType The communication interface type (must be SpiCommInterface<CommType>)
 * @tparam MaxDevices Maximum total capacity (onboard + extra devices, default: 8)
 */
template <typename CommType, size_t MaxDevices = 8>
class TMC51x0DaisyChain {
public:
  /**
   * @brief Construct a daisy-chain manager
   * @param comm Reference to SPI communication interface (shared by all devices)
   * @param num_onboard_devices Number of onboard devices (fixed, created at construction)
   * @param f_clk TMC51x0 clock frequency in Hz (default: 12 MHz)
   *
   * @note Onboard devices are created immediately and cannot be removed.
   *       Extra devices can be added/removed at runtime up to MaxDevices total.
   */
  explicit TMC51x0DaisyChain(CommType& comm, uint8_t num_onboard_devices,
                             uint32_t f_clk = ClockFreq::DEFAULT_F_CLK) noexcept
      : comm_(comm), num_onboard_devices_(num_onboard_devices), num_active_devices_(num_onboard_devices),
        f_clk_(f_clk) {
    // Validate num_onboard_devices
    if (num_onboard_devices == 0 || num_onboard_devices > MaxDevices) {
      num_onboard_devices_ = 1;
      num_active_devices_ = 1;
    }

    // Create onboard TMC51x0 instances, one per device
    // Each device has its position (0, 1, 2, ...) set in constructor
    for (uint8_t i = 0; i < num_onboard_devices_; ++i) {
      drivers_[i] = std::make_optional<TMC51x0<CommType>>(comm_, f_clk_, i);
    }

    // Initialize extra device slots as empty
    for (size_t i = num_onboard_devices_; i < MaxDevices; ++i) {
      drivers_[i] = std::nullopt;
    }

    // CRITICAL: Set the total chain length on the SpiCommInterface
    // This enables proper response extraction using datasheet formula 40·(n-k+1)
    UpdateChainLength();
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
   * @brief Add an extra device at the specified position
   * @param position Position in daisy chain (must be >= num_onboard_devices)
   * @return true if device was added successfully, false otherwise
   *
   * @note Position must be >= num_onboard_devices (cannot add before onboard devices)
   * @note Position must be < MaxDevices
   * @note Device slot must be empty (not already have a device)
   * @note Positions must be sequential - cannot skip positions in daisy chain
   *       (e.g., cannot add position 5 if position 4 is empty)
   * @note Chain length is automatically updated
   *
   * @warning In a daisy chain, positions correspond to physical order (MISO→MOSI).
   *          Positions MUST be sequential. Users can create aliases in their code
   *          for better readability, but the library uses numeric indices.
   */
  bool AddDevice(uint8_t position) noexcept {
    // Validate position
    if (position < num_onboard_devices_ || position >= MaxDevices) {
      return false;
    }

    // Check if slot is already occupied
    if (drivers_[position].has_value()) {
      return false;
    }

    // Enforce sequential positioning: cannot skip positions
    // All positions before this one (starting from num_onboard_devices) must be filled
    for (uint8_t i = num_onboard_devices_; i < position; ++i) {
      if (!drivers_[i].has_value()) {
        return false; // Cannot skip positions in daisy chain
      }
    }

    // Create device instance at specified position
    drivers_[position] = std::make_optional<TMC51x0<CommType>>(comm_, f_clk_, position);
    num_active_devices_++;

    // Update chain length on SpiCommInterface
    UpdateChainLength();

    return true;
  }

  /**
   * @brief Remove an extra device at the specified position
   * @param position Position in daisy chain
   * @return true if device was removed successfully, false otherwise
   *
   * @note Cannot remove onboard devices (position < num_onboard_devices)
   * @note Device slot must be active (have a device)
   * @note Can only remove from the end of the chain (highest position)
   *       to maintain sequential positioning
   * @note Chain length is automatically updated
   *
   * @warning In a daisy chain, positions must remain sequential. You can only
   *          remove devices from the end of the chain (highest position).
   *          Removing a device in the middle would create a gap, which is
   *          not physically possible in a daisy chain.
   */
  bool RemoveDevice(uint8_t position) noexcept {
    // Cannot remove onboard devices
    if (position < num_onboard_devices_) {
      return false;
    }

    // Validate position
    if (position >= MaxDevices) {
      return false;
    }

    // Check if slot is actually active
    if (!drivers_[position].has_value()) {
      return false;
    }

    // Enforce sequential removal: can only remove from the end
    // Check if there are any devices after this position
    for (size_t i = position + 1; i < MaxDevices; ++i) {
      if (drivers_[i].has_value()) {
        return false; // Cannot remove device in the middle of the chain
      }
    }

    // Remove device
    drivers_[position] = std::nullopt;
    num_active_devices_--;

    // Update chain length on SpiCommInterface
    UpdateChainLength();

    return true;
  }

  /**
   * @brief Access individual TMC5160 driver by index
   * @param index Device index (0 = first device, 1 = second, etc.)
   * @return Reference to TMC5160 driver instance
   *
   * @note Index must be in range [0, num_active_devices-1] and device must be active.
   *       No bounds checking is performed for performance reasons.
   *       Use IsDeviceActive() to verify device exists before access.
   */
  [[nodiscard]] TMC51x0<CommType>& operator[](uint8_t index) noexcept {
    return drivers_[index].value();
  }

  /**
   * @brief Const access to individual TMC51x0 driver by index
   * @param index Device index (0 = first device, 1 = second, etc.)
   * @return Const reference to TMC51x0 driver instance
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
  /**
   * @brief Update the chain length on SpiCommInterface
   *
   * This is called whenever devices are added or removed to ensure the
   * SpiCommInterface knows the correct total chain length for proper
   * response extraction using the datasheet formula 40·(n-k+1).
   */
  void UpdateChainLength() noexcept {
    // Find the highest active device position
    uint8_t max_position = 0;
    for (size_t i = 0; i < MaxDevices; ++i) {
      if (drivers_[i].has_value()) {
        max_position = static_cast<uint8_t>(i);
      }
    }

    // Total chain length is max_position + 1 (0-indexed to 1-indexed)
    uint8_t total_length = max_position + 1;
    comm_.SetDaisyChainLength(total_length);
  }

  CommType& comm_;              ///< Shared SPI communication interface
  uint8_t num_onboard_devices_; ///< Number of onboard devices (fixed)
  uint8_t num_active_devices_;  ///< Total number of active devices (onboard + extra)
  uint32_t f_clk_;              ///< TMC51x0 clock frequency
  std::array<std::optional<TMC51x0<CommType>>, MaxDevices>
      drivers_; ///< TMC51x0 driver instances (optional for dynamic devices)
};

} // namespace tmc51x0

#endif // TMC5160_DAISY_CHAIN_HPP
