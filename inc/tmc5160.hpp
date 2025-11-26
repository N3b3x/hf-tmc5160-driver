/**
 * @file tmc5160.hpp
 * @brief Main TMC5160 stepper motor driver interface and subsystem classes
 *
 * This file contains the primary TMC5160 class and all its subsystem interfaces
 * for comprehensive stepper motor control functionality including ramp control,
 * motor configuration, encoder integration, diagnostics, and protection
 * systems.
 *
 * @defgroup TMC5160_Core Core TMC5160 Driver
 * @brief Main TMC5160 driver class and core functionality
 *
 * @defgroup TMC5160_Subsystems Subsystem Interfaces
 * @brief Specialized subsystem classes for different aspects of motor control
 *
 * @defgroup TMC5160_Types Type Definitions
 * @brief Enums, structs, and type definitions used throughout the driver
 */

#ifndef TMC5160_HPP
#define TMC5160_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "tmc5160_comm_interface.hpp"
#include "tmc5160_config.hpp"
#include "tmc5160_registers.hpp"
#include "tmc5160_types.hpp"

namespace tmc5160 {

/**
 * @brief Main class representing a TMC5160 stepper motor driver
 * @ingroup TMC5160_Core
 *
 * The TMC5160 class provides a comprehensive high-level interface for
 * configuring and controlling the TMC5160 stepper motor driver chip. This class
 * abstracts the complex low-level register operations into intuitive,
 * easy-to-use methods for stepper motor control applications.
 *
 * ## Key Features
 *
 * The TMC5160 class supports a wide range of stepper motor control features:
 *
 * - **Ramp Modes**: Positioning, velocity, and hold modes
 * - **Current Control**: Configurable run and hold currents
 * - **Chopper Modes**: spreadCycle and stealthChop operation
 * - **Encoder Support**: Closed-loop control with encoder feedback
 * - **StallGuard2**: Stall detection and prevention
 * - **Protection Systems**: Short circuit, overtemperature, overvoltage
 * protection
 * - **Communication**: SPI or UART interface for platform independence
 *
 * ## Communication Interface
 *
 * The class uses a CRTP-based communication interface for communication, making
 * it completely agnostic to the physical communication layer. This allows the
 * same code to work with SPI, UART, or other communication methods by simply
 * providing the appropriate communication interface implementation.
 *
 * The driver is a template class that takes the communication interface type as
 * a template parameter, providing compile-time polymorphism with zero runtime
 * overhead.
 *
 * ## Usage Example
 *
 * @code
 * // Create communication interface (SPI example)
 * class MySPI : public tmc5160::SpiCommInterface<MySPI> {
 *   // ... implement required methods
 * };
 *
 * MySPI spiComm;
 *
 * // Create TMC5160 driver with template parameter
 * tmc5160::TMC5160<MySPI> driver(spiComm);
 *
 * // Initialize driver
 * tmc5160::DriverConfig cfg{};
 * cfg.motor.irun = 20;
 * cfg.motor.ihold = 10;
 * driver.Initialize(cfg);
 *
 * // Configure ramp control
 * driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
 * driver.rampControl.SetTargetPosition(1000);
 * driver.rampControl.SetMaxSpeed(1000.0f);
 * driver.rampControl.SetAcceleration(500.0f);
 *
 * // Enable motor
 * driver.motorControl.Enable();
 * @endcode
 *
 * @tparam CommType The communication interface type (must inherit from
 *                  SpiCommInterface<CommType> or UartCommInterface<CommType>)
 */
template <typename CommType>
class TMC5160 {
public:
  //================================================================================
  // @name Core Initialization and Management
  // @{
  //================================================================================

  /**
   * @brief Construct a TMC5160 driver instance
   * @param comm Reference to a user-implemented communication interface (SPI,
   * UART, etc)
   * @param f_clk TMC5160 clock frequency in Hz (default: 12 MHz)
   * @param daisy_chain_position Position in daisy chain (0 = first chip/single chip, 1 = second,
   * etc.) Only used for SPI daisy-chaining. Default: 0 (single chip)
   * @param uart_node_address UART node address (0-254). Only used for UART multi-node addressing.
   *                          Default: 0 (single node or first node).
   *                          For sequential programming via TMC5160MultiNode, this is set to 0
   *                          initially and updated automatically after ProgramSequentially().
   *                          For devices already programmed, specify the known address here.
   */
  explicit TMC5160(CommType& comm, uint32_t f_clk = ClockFreq::DEFAULT_F_CLK, uint8_t daisy_chain_position = 0,
                     uint8_t uart_node_address = 0) noexcept
        : comm_(comm), f_clk_(f_clk), daisy_chain_position_(daisy_chain_position),
          uart_node_address_(uart_node_address & 0xFF), send_delay_(0) {}

  /**
   * @brief Destructor for TMC5160, cleans up resources
   */
  ~TMC5160() noexcept {
    // Disable motor on destruction
    if (initialized_) {
      motorControl.Disable();
    }
  }

  /**
   * @brief Get the communication interface used by this TMC5160 instance
   * @return Reference to the communication interface (SPI, UART, etc)
   */
  [[nodiscard]] CommType& GetComm() noexcept {
    return comm_;
  }

  /**
   * @brief Set the daisy-chain position for this TMC5160 instance
   * @param position Position in daisy chain (0 = first chip/single chip, 1 = second, etc.)
   *
   * This method configures the position of this driver in a daisy-chained SPI setup.
   * The position is used when calling ReadRegister() and WriteRegister() to determine
   * the correct padding for daisy-chain communication.
   *
   * @note The daisy-chain position determines how many 40-bit dummy datagrams are
   *       sent before this chip's command, ensuring the command reaches the correct
   *       chip in the chain. Only applicable for SPI communication interfaces.
   */
  void SetDaisyChainPosition(uint8_t position) noexcept {
    daisy_chain_position_ = position;
  }

  /**
   * @brief Get the current daisy-chain position for this TMC5160 instance
   * @return Daisy-chain position (0 = first chip/single chip, 1 = second, etc.)
   */
  [[nodiscard]] uint8_t GetDaisyChainPosition() const noexcept {
    return daisy_chain_position_;
  }

  /**
   * @brief Set the UART node address for this TMC5160 instance
   * @param address UART node address (0-127)
   *
   * This method configures the node address of this driver in a UART multi-node setup.
   * The address is used when calling ReadRegister() and WriteRegister() to determine
   * the correct node address for UART communication.
   *
   * @note The node address must be programmed into the chip via SLAVECONF register
   *       (using UartConfig::ConfigureSlave()). This method only updates the software
   *       representation. Only applicable for UART communication interfaces.
   */
  void SetUartNodeAddress(uint8_t address) noexcept {
    uart_node_address_ = address & 0xFF; // Address range is 0-254 (8-bit)
  }

  /**
   * @brief Get the current UART node address for this TMC5160 instance
   * @return UART node address (0-127)
   */
  [[nodiscard]] uint8_t GetUartNodeAddress() const noexcept {
    return uart_node_address_;
  }

  /**
   * @brief Set the chip communication mode via SPI_MODE and SD_MODE pins
   * @param mode Chip communication mode (SPI_INTERNAL_RAMP, SPI_EXTERNAL_STEPDIR, UART_INTERNAL_RAMP)
   * @return true if mode was set successfully, false if pins are not configured
   *
   * This method controls the SPI_MODE (pin 22) and SD_MODE (pin 21) pins if they
   * are connected to GPIO outputs. These pins determine the TMC5160 operating mode.
   *
   * ⚠️ CRITICAL WARNINGS:
   * - These pins are typically hardwired and read at startup
   * - Only use this method if SPI_MODE and SD_MODE pins are connected to GPIO outputs
   * - Changing the mode requires a chip reset (power cycle or reset pin) to take effect
   * - The mode pins are read at startup, so changes won't be effective until reset
   * - Ensure pins are configured in TMC5160PinConfig (spi_mode_pin and sd_mode_pin)
   *
   * @note After calling this method, you must reset the chip for the new mode to take effect.
   *       The driver does not automatically reset the chip - you must handle this externally.
   *
   * @note Mode pin mapping:
   * - SPI_INTERNAL_RAMP: SPI_MODE=HIGH, SD_MODE=LOW
   * - SPI_EXTERNAL_STEPDIR: SPI_MODE=HIGH, SD_MODE=HIGH
   * - UART_INTERNAL_RAMP: SPI_MODE=LOW, SD_MODE=LOW
   */
  bool SetChipCommMode(ChipCommMode mode) noexcept;

  /**
   * @brief Get the current chip communication mode from SPI_MODE and SD_MODE pins
   * @param mode Reference to store the current mode
   * @return true if mode was read successfully, false if pins are not configured
   *
   * This method reads the current state of SPI_MODE (pin 22) and SD_MODE (pin 21) pins
   * if they are connected to GPIO inputs/outputs.
   *
   * @note This reads the current pin state, which may not reflect the actual chip mode
   *       if the chip hasn't been reset since the pins were changed.
   */
  bool GetChipCommMode(ChipCommMode& mode) const noexcept;

  /**
   * @brief Initialize the TMC5160 driver with configuration
   * @param config Driver configuration structure
   * @return true if initialization succeeded, false otherwise
   *
   * This method performs the complete initialization sequence:
   * 1. Clear reset and error flags
   * 2. Configure power stage parameters
   * 3. Configure motor current settings
   * 4. Configure chopper settings
   * 5. Configure stealthChop settings
   * 6. Configure short protection
   * 7. Set ramp mode to positioning
   * 8. Configure global settings
   */
  bool Initialize(const DriverConfig& config = DriverConfig()) noexcept;

  // @}

  //================================================================================
  // @name Subsystem Interfaces
  // @{
  //================================================================================

  /**
   * @brief Ramp control subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for controlling motor motion including positioning,
   * velocity control, and hold modes.
   */
  struct RampControl {
    /**
     * @brief Construct ramp control subsystem
     * @param driver Reference to parent TMC5160 driver instance
     */
    explicit RampControl(TMC5160& driver) noexcept : driver_(driver) {}

    /**
     * @brief Set the ramp mode
     * @param mode Ramp mode (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD)
     * @return true if set successfully, false otherwise
     */
    bool SetRampMode(RampMode mode) noexcept;

    /**
     * @brief Get current ramp mode
     * @param mode Reference to store the current ramp mode
     * @return true if read successfully, false otherwise
     */
    bool GetRampMode(RampMode& mode) noexcept;

    /**
     * @brief Set target position
     * @param value Target position value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     */
    bool SetTargetPosition(float value, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Get current position
     * @param unit Unit to return the position in (default: Steps)
     * @return Current position in specified unit, or 0 on error
     */
    float GetCurrentPosition(Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Get target position
     * @param unit Unit to return the position in (default: Steps)
     * @return Target position in specified unit, or 0 on error
     */
    float GetTargetPosition(Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set current position
     * @param value Position value
     * @param unit Unit of the value (default: Steps)
     * @param update_encoder If true, also update encoder position
     * @return true if set successfully, false otherwise
     */
    bool SetCurrentPosition(float value, Unit unit = Unit::Steps, bool update_encoder = false) noexcept;

    /**
     * @brief Set maximum speed
     * @param value Maximum speed value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     */
    bool SetMaxSpeed(float value, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set acceleration and deceleration
     * @param value Acceleration value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     */
    bool SetAcceleration(float value, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set acceleration and deceleration separately
     * @param accel_val Acceleration value
     * @param decel_val Deceleration value
     * @param unit Unit of the values (default: Steps)
     * @return true if set successfully, false otherwise
     */
    bool SetAccelerations(float accel_val, float decel_val, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set deceleration only (DMAX register)
     * @param value Deceleration value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     *
     * Sets only the deceleration rate (DMAX register) without affecting acceleration (AMAX).
     */
    bool SetDeceleration(float value, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set ramp speeds
     * @param start_speed Start speed value
     * @param stop_speed Stop speed value
     * @param transition_speed Transition speed value
     * @param unit Unit of the speed values (default: Steps)
     * @return true if set successfully, false otherwise
     */
    bool SetRampSpeeds(float start_speed, float stop_speed, float transition_speed, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Get current speed
     * @param unit Unit to return the speed in (default: Steps)
     * @return Current speed in specified unit, or 0.0f on error
     */
    float GetCurrentSpeed(Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Check if target position is reached
     * @return true if target position reached, false otherwise
     */
    bool IsTargetReached() noexcept;

    /**
     * @brief Check if target velocity is reached
     * @return true if target velocity reached, false otherwise
     */
    bool IsTargetVelocityReached() noexcept;

    /**
     * @brief Stop the motor
     * @return true if stop command sent successfully, false otherwise
     *
     * Stops the motor by setting VSTART and VMAX to 0.
     */
    bool Stop() noexcept;

    /**
     * @brief Configure reference switches/endstops
     * @param config Reference switch configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureReferenceSwitch(const ReferenceSwitchConfig& config) noexcept;

    /**
     * @brief Get current reference switch configuration
     * @param config Reference to store current configuration
     * @return true if read successfully, false otherwise
     */
    bool GetReferenceSwitchConfig(ReferenceSwitchConfig& config) noexcept;

    /**
     * @brief Set left switch active level (determines polarity)
     * @param active_level Active level (ACTIVE_LOW or ACTIVE_HIGH)
     * @return true if configured successfully, false otherwise
     *
     * Updates only the active level, preserving other settings.
     * Allows real-time polarity changes while keeping stop enable and latching configured.
     */
    bool SetLeftSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Set right switch active level (determines polarity)
     * @param active_level Active level (ACTIVE_LOW or ACTIVE_HIGH)
     * @return true if configured successfully, false otherwise
     *
     * Updates only the active level, preserving other settings.
     * Allows real-time polarity changes while keeping stop enable and latching configured.
     */
    bool SetRightSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Enable or disable motor stop on left switch
     * @param enable true to enable stop, false to disable
     * @return true if configured successfully, false otherwise
     *
     * Updates only stop enable, preserving other settings.
     * Allows real-time enable/disable of motor stop while keeping polarity and latching configured.
     */
    bool SetLeftSwitchStopEnable(bool enable) noexcept;

    /**
     * @brief Enable or disable motor stop on right switch
     * @param enable true to enable stop, false to disable
     * @return true if configured successfully, false otherwise
     *
     * Updates only stop enable, preserving other settings.
     * Allows real-time enable/disable of motor stop while keeping polarity and latching configured.
     */
    bool SetRightSwitchStopEnable(bool enable) noexcept;

    /**
     * @brief Set left switch latching mode
     * @param latch_mode Latching mode (DISABLED, ACTIVE_EDGE, INACTIVE_EDGE, BOTH_EDGES)
     * @return true if configured successfully, false otherwise
     *
     * Updates only latching mode, preserving other settings.
     */
    bool SetLeftSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept;

    /**
     * @brief Set right switch latching mode
     * @param latch_mode Latching mode (DISABLED, ACTIVE_EDGE, INACTIVE_EDGE, BOTH_EDGES)
     * @return true if configured successfully, false otherwise
     *
     * Updates only latching mode, preserving other settings.
     */
    bool SetRightSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept;

    /**
     * @brief Set stop mode (hard or soft stop)
     * @param stop_mode Stop mode (HARD_STOP or SOFT_STOP)
     * @return true if configured successfully, false otherwise
     *
     * Updates only stop mode, preserving other settings.
     */
    bool SetStopMode(ReferenceStopMode stop_mode) noexcept;

    /**
     * @brief Get latched position
     * @param unit Unit to return the position in (default: Steps)
     * @return Latched position in specified unit, or 0 on error
     *
     * Reads the position that was latched on the last reference switch event.
     */
    float GetLatchedPosition(Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set position comparison register
     * @param value Position value for comparison
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     *
     * When XACTUAL equals X_COMPARE, the position pulse output becomes high.
     */
    bool SetComparePosition(float value, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set power down delay
     * @param tpowerdown Power down delay (0-255, time range ~0 to 5.6 seconds)
     * @return true if set successfully, false otherwise
     *
     * Sets the delay before power down when motor enters standstill.
     * Minimum setting of 2 is required to allow automatic tuning of stealthChop PWM_OFFS_AUTO.
     */
    bool SetPowerDownDelay(uint8_t tpowerdown) noexcept;

    /**
     * @brief Set zero wait time
     * @param tzerowait Waiting time after ramping down to zero velocity in clock cycles (0-65535)
     * @return true if set successfully, false otherwise
     *
     * Sets the waiting time after ramping down to zero velocity before next
     * movement or direction inversion can start.
     */
    bool SetZeroWaitTime(uint16_t tzerowait) noexcept;

    /**
     * @brief Set first acceleration phase
     * @param a1 First acceleration value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     *
     * Sets the first acceleration phase. If 0.0f, AMAX is used for this phase.
     */
    bool SetFirstAcceleration(float a1, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set final deceleration phase
     * @param d1 Deceleration value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     *
     * Sets the final deceleration phase (D1).
     * Attention: Do not set 0 in positioning mode (datasheet 6.3.1).
     * If set to 0, the driver might behave unexpectedly in positioning mode.
     * A safe minimum (e.g., 100) is recommended if unsure.
     */
    bool SetFinalDeceleration(float d1, Unit unit = Unit::Steps) noexcept;

  private:
    TMC5160& driver_; ///< Reference to parent driver instance

    // Internal helper methods (used by unit-aware public methods)
    bool SetTargetPosition(int32_t position) noexcept;
    bool SetCurrentPosition(int32_t position, bool update_encoder = false) noexcept;
  } rampControl{*this};

  /**
   * @brief Motor control subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for controlling motor current, chopper configuration,
   * and stealthChop operation.
   */
  struct MotorControl {
    /**
     * @brief Construct motor control subsystem
     * @param driver Reference to parent TMC5160 driver instance
     */
    explicit MotorControl(TMC5160& driver) noexcept : driver_(driver) {}

    /**
     * @brief Enable the motor driver
     * @return true if enabled successfully, false otherwise
     */
    bool Enable() noexcept;

    /**
     * @brief Disable the motor driver
     * @return true if disabled successfully, false otherwise
     */
    bool Disable() noexcept;

    /**
     * @brief Set motor run and hold current
     * @param irun Run current (0-31, where 31 = 100% of global scaler)
     * @param ihold Hold current (0-31, where 31 = 100% of global scaler)
     * @return true if set successfully, false otherwise
     */
    bool SetCurrent(uint8_t irun, uint8_t ihold) noexcept;

    /**
     * @brief Configure chopper settings
     * @param config Chopper configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureChopper(const ChopperConfig& config) noexcept;

    /**
     * @brief Configure stealthChop settings
     * @param config StealthChop configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureStealthChop(const StealthChopConfig& config) noexcept;

    /**
     * @brief Set mode change speeds
     * @param pwm_thrs Speed threshold for stealthChop (steps/s)
     * @param cool_thrs Speed threshold for coolStep (steps/s)
     * @param high_thrs Speed threshold for high-speed mode (steps/s)
     * @return true if set successfully, false otherwise
     */
    bool SetModeChangeSpeeds(float pwm_thrs, float cool_thrs, float high_thrs) noexcept;

    /**
     * @brief Set CoolStep velocity threshold (TCOOLTHRS)
     * @param value Velocity threshold value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     */
    bool SetCoolStepThreshold(float value, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set High-Speed velocity threshold (THIGH)
     * @param value Velocity threshold value
     * @param unit Unit of the value (default: Steps)
     * @return true if set successfully, false otherwise
     */
    bool SetHighSpeedThreshold(float value, Unit unit = Unit::Steps) noexcept;

    /**
     * @brief Set global current scaler
     * @param scaler Global scaler value (32-256)
     * @return true if set successfully, false otherwise
     */
    bool SetGlobalScaler(uint16_t scaler) noexcept;

    /**
     * @brief Get global configuration
     * @param config Reference to store current GlobalConfig
     * @return true if read successfully, false otherwise
     */
    bool GetGlobalConfig(GlobalConfig& config) noexcept;

    /**
     * @brief Configure CoolStep current reduction
     * @param config CoolStep configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureCoolStep(const CoolStepConfig& config) noexcept;

    /**
     * @brief Configure dcStep automatic commutation
     * @param config dcStep configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureDcStep(const DcStepConfig& config) noexcept;

    /**
     * @brief Set microstep lookup table entry
     * @param index Lookup table index (0-7)
     * @param value Lookup table value (32-bit)
     * @return true if set successfully, false otherwise
     */
    bool SetMicrostepLookupTable(uint8_t index, uint32_t value) noexcept;

    /**
     * @brief Set microstep lookup table segmentation
     * @param width_sel_0 Width selection for segment 0 (0-3)
     * @param width_sel_1 Width selection for segment 1 (0-3)
     * @param width_sel_2 Width selection for segment 2 (0-3)
     * @param width_sel_3 Width selection for segment 3 (0-3)
     * @param lut_seg_start1 Start position for segment 1 (0-255)
     * @param lut_seg_start2 Start position for segment 2 (0-255)
     * @param lut_seg_start3 Start position for segment 3 (0-255)
     * @return true if set successfully, false otherwise
     */
    bool SetMicrostepLookupTableSegmentation(uint8_t width_sel_0, uint8_t width_sel_1, uint8_t width_sel_2,
                                             uint8_t width_sel_3, uint8_t lut_seg_start1, uint8_t lut_seg_start2,
                                             uint8_t lut_seg_start3) noexcept;

    /**
     * @brief Set microstep lookup table start current
     * @param start_current Start current value (0-255)
     * @return true if set successfully, false otherwise
     */
    bool SetMicrostepLookupTableStart(uint16_t start_current) noexcept;

    /**
     * @brief Setup motor from high-level specifications
     * @param motor_spec Motor specification structure
     * @param mechanical_system Optional mechanical system configuration
     * @return true if setup successfully, false otherwise
     *
     * Automatically calculates and sets motor current, chopper configuration,
     * and other parameters based on motor specifications.
     */
    bool SetupMotorFromSpec(const MotorSpec& motor_spec, const MechanicalSystem* mechanical_system = nullptr) noexcept;

    /**
     * @brief Configure global configuration (GCONF register)
     * @param config Global configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureGlobalConfig(const GlobalConfig& config) noexcept;

    /**
     * @brief Enable/Disable StealthChop (PWM mode)
     * @param enabled true to enable StealthChop, false for SpreadCycle
     * @return true if configured successfully, false otherwise
     */
    bool SetStealthChopEnabled(bool enabled) noexcept;

    /**
     * @brief Get chopper configuration
     * @param config Reference to store current ChopperConfig
     * @return true if read successfully, false otherwise
     */
    bool GetChopperConfig(ChopperConfig& config) noexcept;

    /**
     * @brief Get DIAG0 pin configuration
     * @param config Reference to store current Diag0Config
     * @return true if read successfully, false otherwise
     * 
     * Reads DIAG0 configuration from GCONF register using read-modify-write pattern.
     */
    bool GetDiag0Config(Diag0Config& config) noexcept;

    /**
     * @brief Set DIAG0 pin configuration
     * @param config Diag0Config structure with DIAG0 settings
     * @return true if configured successfully, false otherwise
     * 
     * Writes DIAG0 configuration to GCONF register using read-modify-write pattern.
     * Preserves all other GCONF bits.
     */
    bool SetDiag0Config(const Diag0Config& config) noexcept;

    /**
     * @brief Get DIAG1 pin configuration
     * @param config Reference to store current Diag1Config
     * @return true if read successfully, false otherwise
     * 
     * Reads DIAG1 configuration from GCONF register using read-modify-write pattern.
     */
    bool GetDiag1Config(Diag1Config& config) noexcept;

    /**
     * @brief Set DIAG1 pin configuration
     * @param config Diag1Config structure with DIAG1 settings
     * @return true if configured successfully, false otherwise
     * 
     * Writes DIAG1 configuration to GCONF register using read-modify-write pattern.
     * Preserves all other GCONF bits.
     * 
     * @note steps_skipped should not be enabled with other DIAG1 options (mutually exclusive).
     */
    bool SetDiag1Config(const Diag1Config& config) noexcept;

  private:
    TMC5160& driver_; ///< Reference to parent driver instance
  } motorControl{*this};

  /**
   * @brief Communication subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for configuring UART slave addressing and multi-chip setups.
   */
  struct Communication {
    /**
     * @brief Construct communication subsystem
     * @param driver Reference to parent TMC5160 driver instance
     */
    explicit Communication(TMC5160& driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure UART slave address and send delay
     * @param slave_address 7-bit slave address (0-127), same as UART node address
     * @param send_delay Number of bit times before replying to register read (0-15), stored locally
     * @return true if configured successfully, false otherwise
     * 
     * Note: Slave address and UART node address are the same value.
     * Send delay is stored locally since SLAVECONF register is write-only.
     */
    bool ConfigureSlaveAddress(uint8_t slave_address, uint8_t send_delay = 0) noexcept;


  private:
    TMC5160& driver_; ///< Reference to parent driver instance
  } communication{*this};

  /**
   * @brief Encoder subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for encoder configuration and reading encoder position.
   */
  struct Encoder {
    /**
     * @brief Construct encoder subsystem
     * @param driver Reference to parent TMC5160 driver instance
     */
    explicit Encoder(TMC5160& driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure encoder settings
     * @param config Encoder configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool Configure(const EncoderConfig& config) noexcept;

    /**
     * @brief Get current encoder configuration
     * @param config Reference to store current configuration
     * @return true if read successfully, false otherwise
     */
    bool GetEncoderConfig(EncoderConfig& config) noexcept;

    /**
     * @brief Set N channel active level (determines polarity)
     * @param active_level Active level (ACTIVE_LOW or ACTIVE_HIGH)
     * @return true if configured successfully, false otherwise
     *
     * Updates only the active level, preserving other settings.
     * Shares ReferenceSwitchActiveLevel enum with reference switches.
     */
    bool SetNChannelActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Set N channel sensitivity (edge/level detection)
     * @param sensitivity Sensitivity mode (ACTIVE_LEVEL, RISING_EDGE, FALLING_EDGE, BOTH_EDGES)
     * @return true if configured successfully, false otherwise
     *
     * Updates only sensitivity, preserving other settings.
     */
    bool SetNChannelSensitivity(EncoderNSensitivity sensitivity) noexcept;

    /**
     * @brief Set encoder clear mode
     * @param clear_mode Clear mode (DISABLED, ONCE, CONTINUOUS)
     * @return true if configured successfully, false otherwise
     *
     * Updates only clear mode, preserving other settings.
     */
    bool SetClearMode(EncoderClearMode clear_mode) noexcept;

    /**
     * @brief Set encoder prescaler mode
     * @param prescaler_mode Prescaler mode (BINARY or DECIMAL)
     * @return true if configured successfully, false otherwise
     *
     * Updates only prescaler mode, preserving other settings.
     */
    bool SetPrescalerMode(EncoderPrescalerMode prescaler_mode) noexcept;

    /**
     * @brief Get encoder position
     * @return Encoder position in steps, or 0 on error
     */
    int32_t GetPosition() noexcept;

    /**
     * @brief Set encoder resolution
     * @param motor_steps Number of steps per turn for the motor
     * @param enc_resolution Actual encoder resolution (pulses per turn)
     * @param inverted Whether encoder and motor rotations are inverted
     * @return true if exact match found, false if approximation used
     */
    bool SetResolution(int32_t motor_steps, int32_t enc_resolution, bool inverted = false) noexcept;

    /**
     * @brief Set encoder allowed deviation
     * @param steps Maximum number of steps deviation before warning
     * @return true if set successfully, false otherwise
     */
    bool SetAllowedDeviation(int32_t steps) noexcept;

    /**
     * @brief Check if encoder deviation detected
     * @return true if deviation detected, false otherwise
     */
    bool IsDeviationDetected() noexcept;

    /**
     * @brief Clear encoder deviation flag
     * @return true if cleared successfully, false otherwise
     */
    bool ClearDeviationFlag() noexcept;

    /**
     * @brief Get encoder latched position
     * @return Encoder position latched on N event, or 0 on error
     *
     * Reads the encoder position that was latched on the last N channel event.
     */
    int32_t GetLatchedPosition() noexcept;

  private:
    TMC5160& driver_; ///< Reference to parent driver instance
  } encoder{*this};

  /**
   * @brief Diagnostics subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for reading driver status, StallGuard values, and error
   * detection.
   */
  struct Diagnostics {
    /**
     * @brief Construct diagnostics subsystem
     * @param driver Reference to parent TMC5160 driver instance
     */
    explicit Diagnostics(TMC5160& driver) noexcept : driver_(driver) {}

    /**
     * @brief Get driver status
     * @return DriverStatus enumeration indicating current status
     */
    DriverStatus GetStatus() noexcept;

    /**
     * @brief Get global status flags (GSTAT)
     * @param reset Reference to store reset flag
     * @param drv_err Reference to store driver error flag
     * @param uv_cp Reference to store undervoltage charge pump flag
     * @return true if read successfully, false otherwise
     */
    bool GetGlobalStatus(bool& reset, bool& drv_err, bool& uv_cp) noexcept;

    /**
     * @brief Get StallGuard2 value
     * @return StallGuard2 value (0-1023), or 0 on error
     */
    uint16_t GetStallGuard() noexcept;

    /**
     * @brief Get StallGuard2 result from DRV_STATUS register
     * @param sg_result Reference to store StallGuard2 value (0-1023)
     * @return true if read successfully, false otherwise
     */
    bool GetStallGuardResult(uint16_t& sg_result) noexcept;

    /**
     * @brief Configure StallGuard2
     * @param config StallGuard configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureStallGuard(const StallGuardConfig& config) noexcept;

    /**
     * @brief Enable/Disable stop on stall (sg_stop in SW_MODE)
     * @param enable true to enable stop on stall, false to disable
     * @return true if configured successfully, false otherwise
     */
    bool EnableStopOnStall(bool enable) noexcept;

    /**
     * @brief Clear stall event flag (event_stop_sg in RAMP_STAT)
     * @return true if cleared successfully, false otherwise
     */
    bool ClearStallFlag() noexcept;

    /**
     * @brief Check if stall was detected (event_stop_sg in RAMP_STAT)
     * @return true if stall event detected, false otherwise
     */
    bool IsStallDetected() noexcept;

    /**
     * @brief Get driver status register value
     * @param status Reference to store the DRV_STATUS register value
     * @return true if read successfully, false otherwise
     */
    bool GetDriverStatusRegister(uint32_t& status) noexcept;

    /**
     * @brief Check if open load is detected on phase A
     * @return true if open load detected on phase A, false otherwise or on error
     *
     * Open load detection indicates an interrupted cable or connector issue.
     *
     * @note Requirements for reliable detection:
     * - Must operate in SpreadCycle mode (StealthChop disabled)
     * - Motor must be moving (minimum 4× microstep resolution in single direction)
     * - Use low or nominal motor velocity
     * - Cannot be detected in standstill (coils may have zero current)
     *
     * @warning Open load flags are informative only and do not cause driver action.
     * Also triggered by undervoltage, high velocity, short circuit, or overtemperature conditions.
     *
     * @see Datasheet section 11.3: Open Load Diagnostics
     */
    bool IsOpenLoadA() noexcept;

    /**
     * @brief Check if open load is detected on phase B
     * @return true if open load detected on phase B, false otherwise or on error
     *
     * Open load detection indicates an interrupted cable or connector issue.
     *
     * @note Requirements for reliable detection:
     * - Must operate in SpreadCycle mode (StealthChop disabled)
     * - Motor must be moving (minimum 4× microstep resolution in single direction)
     * - Use low or nominal motor velocity
     * - Cannot be detected in standstill (coils may have zero current)
     *
     * @warning Open load flags are informative only and do not cause driver action.
     * Also triggered by undervoltage, high velocity, short circuit, or overtemperature conditions.
     *
     * @see Datasheet section 11.3: Open Load Diagnostics
     */
    bool IsOpenLoadB() noexcept;

    /**
     * @brief Check if open load is detected on either phase
     * @param phase_a Reference to store phase A status (true if open load detected)
     * @param phase_b Reference to store phase B status (true if open load detected)
     * @return true if read successfully, false otherwise
     *
     * Convenience method to check both phases at once.
     *
     * @note Requirements for reliable detection:
     * - Must operate in SpreadCycle mode (StealthChop disabled)
     * - Motor must be moving (minimum 4× microstep resolution in single direction)
     * - Use low or nominal motor velocity
     * - Cannot be detected in standstill (coils may have zero current)
     *
     * @warning Open load flags are informative only and do not cause driver action.
     * Also triggered by undervoltage, high velocity, short circuit, or overtemperature conditions.
     *
     * @see Datasheet section 11.3: Open Load Diagnostics
     */
    bool CheckOpenLoad(bool& phase_a, bool& phase_b) noexcept;

    /**
     * @brief Get ramp status register value
     * @param status Reference to store the RAMP_STAT register value
     * @return true if read successfully, false otherwise
     */
    bool GetRampStatusRegister(uint32_t& status) noexcept;

    /**
     * @brief Get lost steps counter
     * @return Number of lost steps, or 0 on error
     *
     * Only valid when dcStep mode is enabled (SD_MODE = 1).
     */
    uint32_t GetLostSteps() noexcept;

    /**
     * @brief Perform sensorless homing using StallGuard2
     * @param direction Direction to search (true = positive, false = negative)
     * @param stall_threshold StallGuard2 threshold for stall detection
     * @param search_speed Speed for homing search in steps/s
     * @param final_position Reference to store final position after homing
     * @param timeout_ms Maximum time to wait for stall in milliseconds (default: 10000ms)
     * @return true if stall detected and motor stopped, false if timeout or error
     *
     * Moves motor in specified direction until stall is detected, then stops.
     * Uses a polling loop to monitor stall status.
     */
    bool PerformSensorlessHoming(bool direction, int8_t stall_threshold, float search_speed,
                                 int32_t& final_position, uint32_t timeout_ms = 10000) noexcept;

    /**
     * @brief Perform homing using a reference switch (Section 12.4)
     * @param direction Direction to search (true = positive, false = negative)
     * @param search_speed Speed for homing search in steps/s
     * @param switch_speed Speed for slow approach to switch in steps/s (unused, reserved for future two-phase approach)
     * @param final_position Reference to store final position after homing (will be 0 after successful homing)
     * @param use_left_switch true to use REFL, false to use REFR
     * @param timeout_ms Maximum time to wait for switch in milliseconds (default: 10000ms)
     * @return true if switch hit and homing completed successfully, false if timeout or error
     *
     * Implements complete homing procedure from datasheet section 12.4:
     * 1. Ensure switch is not pressed (user responsibility - move away before calling)
     * 2. Activate position latching and motor stop upon switch event
     * 3. Start motion ramp into direction of switch
     * 4. Wait for standstill after switch hit (poll VACTUAL/vzero)
     * 5. Switch to hold mode and calculate position difference
     * 6. Write difference to actual position register (sets home position to 0)
     *
     * After successful homing:
     * - Motor position is set to 0 (home position)
     * - Moving to position 0 will return motor to the switching point
     * - Stop function is disabled to allow moving away from switch
     *
     * @note Uses hard stop for precise homing (no overshoot)
     * @note Ensure switch is not pressed before calling (move away first)
     * @note After homing, configure switches for normal operation if needed
     */
    bool PerformSwitchHoming(bool direction, float search_speed, float switch_speed,
                             int32_t& final_position, bool use_left_switch, uint32_t timeout_ms = 10000) noexcept;

    /**
     * @brief Get actual time between microsteps
     * @return Time between microsteps in clock cycles, or 0 on error
     *
     * Read-only register showing actual time between microsteps.
     */
    uint32_t GetTimeBetweenMicrosteps() noexcept;

    /**
     * @brief Get microstep counter
     * @return Actual position in microstep table (0-1023), or 0 on error
     *
     * Read-only register showing actual position in the microstep table.
     */
    uint16_t GetMicrostepCounter() noexcept;

    /**
     * @brief Get microstep current
     * @param phase_a Reference to store phase A current (signed, -256 to 255)
     * @param phase_b Reference to store phase B current (signed, -256 to 255)
     * @return true if read successfully, false otherwise
     *
     * Read-only register showing actual microstep current for both phases.
     * Values are signed 9-bit as read from MSLUT (not scaled by current).
     */
    bool GetMicrostepCurrent(int16_t& phase_a, int16_t& phase_b) noexcept;

    /**
     * @brief Get PWM scale results
     * @param pwm_scale_sum Reference to store actual PWM duty cycle (0-255)
     * @param pwm_scale_auto Reference to store automatic regulation result (signed -255...+255)
     * @return true if read successfully, false otherwise
     *
     * Read-only register showing stealthChop PWM scale results.
     */
    bool GetPwmScale(uint8_t& pwm_scale_sum, int16_t& pwm_scale_auto) noexcept;

    /**
     * @brief Get automatically determined PWM values
     * @param pwm_ofs_auto Reference to store auto-determined offset (0-255)
     * @param pwm_grad_auto Reference to store auto-determined gradient (0-255)
     * @return true if read successfully, false otherwise
     *
     * Read-only register showing automatically determined PWM configuration values.
     */
    bool GetPwmAuto(uint8_t& pwm_ofs_auto, uint8_t& pwm_grad_auto) noexcept;

    /**
     * @brief Read GPIO input pins
     * @param input_status Reference to store parsed input pin states
     * @return true if read successfully, false otherwise
     *
     * Reads the state of all GPIO input pins and the IC version from register 0x04 (IOIN).
     */
    bool ReadInputStatus(InputStatus& input_status) noexcept;

    /**
     * @brief Read IC version
     * @param version Reference to store the 8-bit IC version
     * @return true if read successfully, false otherwise
     * 
     * Reads the VERSION field from IOIN register (0x04).
     * Expected value for TMC5160 is 0x30.
     */
    bool ReadIcVersion(uint8_t& version) noexcept;

    /**
     * @brief Read GPIO input pins (raw)
     * @param io_pins Reference to store raw IO pin register value
     * @return true if read successfully, false otherwise
     *
     * Reads the raw state of all GPIO input pins (register 0x04).
     */
    bool ReadGpioPins(uint32_t& io_pins) noexcept;

    /**
     * @brief Read factory configuration
     * @param fclktrim Reference to store FCLKTRIM value (0-31)
     * @return true if read successfully, false otherwise
     *
     * Reads the factory configuration/clock trim value.
     */
    bool ReadFactoryConfig(uint8_t& fclktrim) noexcept;

    /**
     * @brief Set SDO_CFG0 pin polarity (UART/Single Wire mode)
     * @param polarity Output pin polarity (false=normal/active high, true=inverted/active low)
     * @return true if set successfully, false otherwise
     *
     * Sets the polarity of the SDO_CFG0 pin when used as Next Address Output (NAO)
     * in single-wire UART chain mode.
     *
     * @note This affects the OUTPUT register (0x04), bit 0.
     *       The reset value is 1 (active low/inverted) for use as NAO.
     */
    bool SetSdoCfg0Polarity(bool polarity) noexcept;

    /**
     * @brief Read OTP configuration
     * @param otp_fclktrim Reference to store OTP FCLKTRIM (0-31)
     * @param otp_s2_level Reference to store OTP S2 level (0-1)
     * @param otp_bbm Reference to store OTP BBM (0-1)
     * @param otp_tbl Reference to store OTP TBL (0-1)
     * @return true if read successfully, false otherwise
     *
     * Reads the one-time programmable configuration memory.
     */
    bool ReadOtpConfig(uint8_t& otp_fclktrim, bool& otp_s2_level, bool& otp_bbm, bool& otp_tbl) noexcept;

    /**
     * @brief Get UART transmission counter
     * @return UART transmission counter value, or 0 on error
     *
     * Returns the number of UART transmissions since last read.
     */
    uint8_t GetUartTransmissionCount() noexcept;

    /**
     * @brief Read offset calibration results
     * @param phase_a Reference to store phase A offset (0-255)
     * @param phase_b Reference to store phase B offset (0-255)
     * @return true if read successfully, false otherwise
     *
     * Reads the results from offset calibration procedure.
     */
    bool ReadOffsetCalibration(uint8_t& phase_a, uint8_t& phase_b) noexcept;

    /**
     * @brief Run comprehensive startup verification
     * 
     * Performs a full verification of the driver setup including:
     * - IC Version check
     * - Input pin state logging
     * - Critical register checks
     * 
     * Logs all findings using the system logger.
     * @return true if basic verification passed (IC found), false otherwise
     */
    bool VerifySetup() noexcept;

    /**
     * @brief Automatically tune StallGuard threshold (SGT)
     * @param target_velocity Velocity to tune at
     * @param final_sgt Reference to store the tuned SGT value
     * @param min_sgt Minimum SGT to try (default: -10)
     * @param max_sgt Maximum SGT to try (default: 63)
     * @param acceleration Acceleration/deceleration (default: 3000.0f steps/s^2)
     * @param min_velocity Minimum velocity to verify tuning at (0 = disabled)
     * @param max_velocity Maximum velocity to verify tuning at (0 = disabled)
     * @param velocity_unit Unit for velocity and acceleration parameters (default: Steps)
     * @return true if tuning succeeded, false if failed (e.g., stall never cleared)
     * 
     * Implements the automatic tuning algorithm:
     * 1. Sets SGT to min_sgt
     * 2. Accelerates to target_velocity
     * 3. Monitors SG_RESULT
     * 4. Increases SGT until SG_RESULT is consistently > 0
     * 5. Verifies SGT at min_velocity and max_velocity if specified
     */
    bool TuneStallGuard(float target_velocity, int8_t& final_sgt, int8_t min_sgt = -10, int8_t max_sgt = 63, 
                        float acceleration = 3000.0f, float min_velocity = 0.0f, float max_velocity = 0.0f, 
                        Unit velocity_unit = Unit::Steps) noexcept;

  private:
    TMC5160& driver_; ///< Reference to parent driver instance
  } diagnostics{*this};

  /**
   * @brief UART configuration subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for configuring UART slave mode operation.
   */
  struct UartConfig {
    TMC5160* driver_; ///< Pointer to parent driver instance

    /**
     * @brief Construct UART configuration subsystem
     * @param driver Pointer to parent TMC5160 driver instance
     */
    explicit UartConfig(TMC5160* driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure UART slave settings
     * @param slave_address UART slave address (0-127)
     * @param send_delay Number of bit times before replying (0-15)
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureSlave(uint8_t slave_address, uint8_t send_delay) noexcept;
  } uartConfig{this};

  /**
   * @brief Protection subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for configuring protection systems including short circuit
   * detection and overtemperature protection.
   */
  struct Protection {
    /**
     * @brief Construct protection subsystem
     * @param driver Reference to parent TMC5160 driver instance
     */
    explicit Protection(TMC5160& driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure short protection levels
     * @param config Power stage parameters structure (contains short protection fields)
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureShortProtection(const PowerStageParameters& config) noexcept;

    /**
     * @brief Set short protection levels
     * @param s2vs_level Short to VS detector sensitivity (4-15)
     * @param s2g_level Short to GND detector sensitivity (2-15)
     * @param shortfilter Spike filtering bandwidth (0-3)
     * @param shortdelay Short detection delay (0-1)
     * @return true if set successfully, false otherwise
     */
    bool SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level, uint8_t shortfilter,
                                  uint8_t shortdelay) noexcept;

  private:
    TMC5160& driver_; ///< Reference to parent driver instance
  } protection{*this};

  // @}

protected:
  /**
   * @brief Get the appropriate address parameter for ReadRegister/WriteRegister
   * @return daisy_chain_position_ for SPI, uart_node_address_ for UART
   *
   * This method is protected so that subsystem classes can access it.
   */
  [[nodiscard]] uint8_t GetCommAddress() const noexcept {
    return (comm_.GetMode() == CommMode::UART) ? uart_node_address_ : daisy_chain_position_;
  }

  /**
   * @brief Reset the TMC5160 driver
   * @return true if reset succeeded, false otherwise
   *
   * Performs a software reset by writing to the GSTAT register.
   */
  bool Reset() noexcept;

  /**
   * @brief Check if the driver is initialized
   * @return true if initialized, false otherwise
   */
  [[nodiscard]] bool IsInitialized() const noexcept {
    return initialized_;
  }

private:
  CommType& comm_;               ///< Communication interface reference
  uint32_t f_clk_;               ///< TMC5160 clock frequency in Hz
  uint8_t daisy_chain_position_; ///< Position in daisy chain (0 = first chip/single chip)
  uint8_t uart_node_address_;    ///< UART node address / slave address (0-127) for multi-node addressing
  uint8_t send_delay_;           ///< UART send delay (0-15) stored locally from SLAVECONF register
  bool initialized_{false};      ///< Initialization status flag

  // Physical configuration for unit conversions
  MotorSpec motor_spec_;
  MechanicalSystem mechanical_system_;
  
  // Calculated motor current settings (stored internally, not in MotorSpec)
  uint16_t calculated_global_scaler_{0};
  uint8_t calculated_irun_{0};
  uint8_t calculated_ihold_{0};
  uint16_t current_microsteps_{256};

  /**
   * @brief Convert speed value to internal steps/s
   * @param value Speed value in specified unit
   * @param unit Unit of the value
   * @return Speed in steps/s
   */
  [[nodiscard]] float convertSpeedToSteps(float value, Unit unit) const noexcept;

  /**
   * @brief Convert acceleration value to internal steps/s²
   * @param value Acceleration value in specified unit
   * @param unit Unit of the value
   * @return Acceleration in steps/s²
   */
  [[nodiscard]] float convertAccelerationToSteps(float value, Unit unit) const noexcept;

  /**
   * @brief Convert position value to steps
   * @param value Position value in specified unit
   * @param unit Unit of the value
   * @return Position in steps
   */
  [[nodiscard]] float convertPositionToSteps(float value, Unit unit) const noexcept;

  /**
   * @brief Convert steps to specified unit (for position)
   * @param steps Position in steps
   * @param unit Target unit
   * @return Position in target unit
   */
  [[nodiscard]] float convertStepsToUnit(int32_t steps, Unit unit) const noexcept;

  /**
   * @brief Convert speed (steps/s) to specified unit
   * @param steps_per_sec Speed in steps/s
   * @param unit Target unit
   * @return Speed in target unit
   */
  [[nodiscard]] float convertSpeedToUnit(float steps_per_sec, Unit unit) const noexcept;

  /**
   * @brief Convert speed from Hz to internal TMC5160 units
   * @param speed_hz Speed in steps per second
   * @return Speed in internal TMC5160 units
   */
  [[nodiscard]] int32_t speedToInternal(float speed_hz) const noexcept;

  /**
   * @brief Convert speed from internal TMC5160 units to Hz
   * @param speed_internal Speed in internal TMC5160 units
   * @return Speed in steps per second
   */
  [[nodiscard]] float speedFromInternal(int32_t speed_internal) const noexcept;

  /**
   * @brief Convert acceleration from Hz/s to internal TMC5160 units
   * @param accel_hz Acceleration in steps per second squared
   * @return Acceleration in internal TMC5160 units
   */
  [[nodiscard]] int32_t accelToInternal(float accel_hz) const noexcept;

  /**
   * @brief Convert threshold speed to TSTEP format
   * @param speed_hz Speed threshold in steps per second
   * @return TSTEP value (0 if speed is 0)
   */
  [[nodiscard]] int32_t thresholdSpeedToTstep(float speed_hz) const noexcept;
};

} // namespace tmc5160

// Include template implementation
#define TMC5160_HEADER_INCLUDED
// NOLINTNEXTLINE(bugprone-suspicious-include) - Intentional: template
// implementation file
#include "../src/tmc5160.cpp"
#undef TMC5160_HEADER_INCLUDED

#endif // TMC5160_HPP
