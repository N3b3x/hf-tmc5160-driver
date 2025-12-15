/**
 * @file tmc51x0.hpp
 * @brief Main TMC51x0 stepper motor driver interface and subsystem classes
 * (TMC5130 & TMC5160)
 *
 * This file contains the primary TMC51x0 class and all its subsystem interfaces
 * for comprehensive stepper motor control functionality including ramp control,
 * motor configuration, encoder integration, diagnostics, and protection
 * systems. Supports both TMC5130 and TMC5160 chips with automatic detection.
 *
 * @defgroup TMC51X0_Core Core TMC51x0 Driver
 * @brief Main TMC51x0 driver class and core functionality
 *
 * @defgroup TMC51X0_Subsystems Subsystem Interfaces
 * @brief Specialized subsystem classes for different aspects of motor control
 *
 * @defgroup TMC51X0_Types Type Definitions
 * @brief Enums, structs, and type definitions used throughout the driver
 */

#ifndef TMC51X0_HPP
#define TMC51X0_HPP

#include <cstdint>
#include <string>

#include "tmc51x0_comm_interface.hpp"
#include "registers/tmc51x0_registers.hpp"
#include "tmc51x0_result.hpp"
#include "tmc51x0_types.hpp"

namespace tmc51x0 {

/**
 * @brief Main class representing a TMC51x0 stepper motor driver (TMC5130 &
 * TMC5160)
 * @ingroup TMC51X0_Core
 *
 * The TMC51x0 class provides a comprehensive high-level interface for
 * configuring and controlling TMC5130 and TMC5160 stepper motor driver chips.
 * This class abstracts the complex low-level register operations into
 * intuitive, easy-to-use methods for stepper motor control applications.
 *
 * ## Key Features
 *
 * The TMC51x0 class supports a wide range of stepper motor control features:
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
 * class MySPI : public tmc51x0::SpiCommInterface<MySPI> {
 *   // ... implement required methods
 * };
 *
 * MySPI spiComm;
 *
 * // Create TMC51x0 driver with template parameter
 * tmc51x0::TMC51x0<MySPI> driver(spiComm);
 *
 * // Initialize driver
 * tmc51x0::DriverConfig cfg{};
 * cfg.motor.irun = 20;
 * cfg.motor.ihold = 10;
 * driver.Initialize(cfg);
 *
 * // Configure ramp control
 * driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
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
template <typename CommType> class TMC51x0 {
public:
  //================================================================================
  // @name Core Initialization and Management
  // @{
  //================================================================================

  /**
   * @brief Construct a TMC51x0 driver instance
   * @param comm Reference to a user-implemented communication interface (SPI,
   * UART, etc)
   * @param daisy_chain_position Position in daisy chain (0 = first chip/single
   * chip, 1 = second, etc.) Only used for SPI daisy-chaining. Default: 0
   * (single chip)
   * @param uart_node_address UART node address (0-254). Only used for UART
   * multi-node addressing. Default: 0 (single node or first node). For
   * sequential programming via TMC51x0MultiNode, this is set to 0 initially and
   * updated automatically after ProgramSequentially(). For devices already
   * programmed, specify the known address here.
   *
   * @note The clock frequency (f_clk) is determined during Initialize() from
   * DriverConfig::external_clk_config:
   *       - If external_clk_config.frequency_hz = 0: Uses internal oscillator
   * (~12 MHz, CLK pin tied to GND)
   *       - If external_clk_config.frequency_hz > 0: Uses external clock at
   * specified frequency (CLK pin receives clock signal)
   *       - The internal clock has ±4% velocity precision and includes a
   * watchdog that switches back to internal clock if external clock signal is
   * missing for more than ~32 internal clock cycles
   *       - External clock range: 10-16 MHz recommended, up to 18 MHz with 50%
   * duty cycle
   */
  explicit TMC51x0(CommType &comm, uint8_t daisy_chain_position = 0,
                   uint8_t uart_node_address = 0) noexcept
      : comm_(comm), daisy_chain_position_(daisy_chain_position),
        uart_node_address_(uart_node_address & 0xFF) {}

  /**
   * @brief Destructor for TMC51x0, cleans up resources
   */
  ~TMC51x0() noexcept {
    // Disable motor on destruction
    if (initialized_) {
      motorControl.Disable();
    }
  }

  /**
   * @brief Get the communication interface used by this TMC51x0 instance
   * @return Reference to the communication interface (SPI, UART, etc)
   */
  [[nodiscard]] CommType &GetComm() noexcept { return comm_; }

  /**
   * @brief Initialize the TMC51x0 driver with configuration
   * @param config Driver configuration structure
   * @return Result<void> indicating success or specific error
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
   *
   * The configuration is stored internally as a snapshot of initialization
   * state. This snapshot represents what was configured during Initialize(),
   * not runtime state. Runtime changes (e.g., SetCurrent(), SetMaxSpeed()) do
   * not update this snapshot.
   *
   * @code
   * auto result = driver.Initialize(config);
   * if (!result) {  // Bool operator
   *     printf("Error: %s\n", result.ErrorMessage());
   * }
   * @endcode
   */
  Result<void> Initialize(const DriverConfig &config = DriverConfig()) noexcept;

  /**
   * @brief Get current driver configuration
   * @return Current driver configuration (including all runtime changes)
   *
   * Retrieves the current driver configuration including all runtime changes.
   * This includes the initial configuration plus any modifications made after
   * initialization.
   *
   * @note If the driver is not initialized, the returned config will contain
   * default values.
   */
  [[nodiscard]] DriverConfig GetDriverConfig() const noexcept {
    return driver_config_;
  }

  /**
   * @brief Get current driver configuration with initialization status
   * @param initialized Reference to store initialization status (true if
   * initialized, false otherwise)
   * @return Current driver configuration (including all runtime changes)
   *
   * Retrieves the current driver configuration including all runtime changes.
   * Also returns the initialization status through the `initialized` parameter.
   *
   * @note If the driver is not initialized, the returned config will contain
   * default values.
   */
  [[nodiscard]] DriverConfig GetDriverConfig(bool &initialized) const noexcept {
    initialized = initialized_;
    return driver_config_;
  }

  /**
   * @brief Get driver configuration as formatted string
   * @return String containing complete driver configuration and status
   * information
   *
   * Returns a human-readable string with all driver configuration, register
   * values, and current status. Useful for debugging and diagnostics.
   *
   * @note The returned string is allocated on the heap. Caller is responsible
   * for freeing.
   * @note For embedded systems, consider using a fixed-size buffer version
   * instead.
   */
  [[nodiscard]] std::string GetDriverConfigString() const noexcept;

  // @}

  //================================================================================
  // @name Subsystem Interfaces
  // @{
  //================================================================================

  //================================================================================
  //================================================================================
  //                                    RAMP CONTROL STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Ramp control subsystem
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for controlling motor motion including positioning,
   * velocity control, and hold modes.
   */
  struct RampControl {
    /**
     * @brief Construct ramp control subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit RampControl(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Set the ramp mode
     * @param mode Ramp mode (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD)
     * @return Result<void> indicating success or error
     */
    Result<void> SetRampMode(RampMode mode) noexcept;

    /**
     * @brief Get current ramp mode
     * @return Result<RampMode> containing the current ramp mode or error
     */
    Result<RampMode> GetRampMode() noexcept;

    /**
     * @brief Check if position target has been reached
     * @return Result<bool> containing true if position reached, false otherwise
     */
    Result<bool> IsPositionReached() noexcept;

    /**
     * @brief Check if velocity target has been reached
     * @return Result<bool> containing true if velocity reached, false otherwise
     */
    Result<bool> IsVelocityReached() noexcept;

    /**
     * @brief Check if motor is in standstill
     * @return Result<bool> containing true if motor is in standstill, false
     * otherwise
     */
    Result<bool> IsStandstill() noexcept;

    /**
     * @brief Get reference switch status
     * @param left_active Reference to store left switch active status
     * @param right_active Reference to store right switch active status
     * @param left_enabled Reference to store left switch enabled status
     * @param right_enabled Reference to store right switch enabled status
     * @return Result<bool> containing true if read successfully, false
     * otherwise
     */
    Result<bool> GetReferenceSwitchStatus(bool &right_active,
                                          bool &left_enabled,
                                          bool &right_enabled) noexcept;

    /**
     * @brief Set X_COMPARE register
     * @param position Position value to compare against
     * @param unit Unit of the position value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * Sets the position comparison register. When XACTUAL equals X_COMPARE,
     * the pos_reached flag is set in RAMP_STAT.
     */
    Result<void> SetXCompare(float position, Unit unit) noexcept;

    /**
     * @brief Get X_COMPARE register value (from local storage)
     * @param position Reference to store the position value
     * @param unit Unit to return the position in (default: Steps)
     * @return Result<float> containing the value or error
     *
     * Returns the locally tracked value of X_COMPARE register.
     * This register is write-only, so we track it locally.
     */
    Result<float> GetXCompare(Unit unit) const noexcept;

    /**
     * @brief Set target position (absolute)
     * @param value Target position value (absolute position from current
     * zero/home)
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * Sets an absolute target position. The position is relative to the current
     * zero/home position set via SetCurrentPosition(). If home is unknown, call
     * SetCurrentPosition(0.0f) at the current physical position to establish a
     * reference point.
     *
     * @note Internally, XTARGET/XACTUAL are in **microsteps** and therefore depend
     *       on the configured microstep resolution (CHOPCONF.MRES). When using
     *       Unit::Steps here, the value is interpreted as **motor full steps**
     *       and will be converted to microsteps using the current MRES setting.
     */
    Result<void> SetTargetPosition(float value, Unit unit) noexcept;

    /**
     * @brief Move relative to current position
     * @param offset Relative movement offset (positive = forward, negative =
     * backward)
     * @param unit Unit of the offset value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * Moves relative to the current position. Automatically calculates the new
     * target position by adding the offset to the current position. No need to
     * manually calculate steps.
     *
     * Example:
     *   MoveRelative(90.0f, Unit::Deg);  // Move 90 degrees from current
     * position MoveRelative(-45.0f, Unit::Deg);  // Move 45 degrees backward
     */
    Result<void> MoveRelative(float offset, Unit unit) noexcept;

    /**
     * @brief Get current position
     * @param unit Unit to return the position in (default: Steps)
     * @return Result<float> containing the current position or error
     *
     * Returns the current position relative to the zero/home position set via
     * SetCurrentPosition(). If home is unknown, call SetCurrentPosition(0.0f)
     * at the current physical position first.
     *
     * @note Internally, XACTUAL is in **microsteps** (depends on CHOPCONF.MRES).
     *       When requesting Unit::Steps, the returned value is in **motor full
     *       steps** (microsteps are converted back using the current MRES).
     */
    Result<float> GetCurrentPosition(Unit unit) noexcept;

    /**
     * @brief Get target position
     * @param unit Unit to return the position in (default: Steps)
     * @return Result<float> containing the target position or error
     */
    Result<float> GetTargetPosition(Unit unit) noexcept;

    /**
     * @brief Set current position
     * @param value Position value
     * @param unit Unit of the value (default: Steps)
     * @param update_encoder If true, also update encoder position
     * @return Result<void> indicating success or error
     */
    Result<void> SetCurrentPosition(float value, Unit unit,
                                    bool update_encoder = false) noexcept;

    /**
     * @brief Set maximum speed
     * @param value Maximum speed value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     */
    Result<void> SetMaxSpeed(float value, Unit unit) noexcept;

    /**
     * @brief Set acceleration and deceleration
     * @param value Acceleration value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     */
    Result<void> SetAcceleration(float value, Unit unit) noexcept;

    /**
     * @brief Set acceleration and deceleration separately
     * @param accel_val Acceleration value
     * @param decel_val Deceleration value
     * @param unit Unit of the values (default: Steps)
     * @return Result<void> indicating success or error
     */
    Result<void> SetAccelerations(float accel_val, float decel_val,
                                  Unit unit) noexcept;

    /**
     * @brief Set deceleration only (DMAX register)
     * @param value Deceleration value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * Sets only the deceleration rate (DMAX register) without affecting
     * acceleration (AMAX).
     */
    Result<void> SetDeceleration(float value, Unit unit) noexcept;

    /**
     * @brief Set ramp speeds
     * @param start_speed Start speed value
     * @param stop_speed Stop speed value
     * @param transition_speed Transition speed value
     * @param unit Unit of the speed values (default: Steps)
     * @return Result<void> indicating success or error
     */
    Result<void> SetRampSpeeds(float start_speed, float stop_speed,
                               float transition_speed, Unit unit) noexcept;

    /**
     * @brief Get current speed
     * @param speed Reference to store the current speed
     * @param unit Unit to return the speed in (default: Steps)
     * @return Result<float> containing the value or error
     */
    Result<float> GetCurrentSpeed(Unit unit) noexcept;

    /**
     * @brief Check if target position is reached
     * @return Result<bool> containing true if target position reached, false
     * otherwise
     */
    Result<bool> IsTargetReached() noexcept;

    /**
     * @brief Check if target velocity is reached
     * @return Result<bool> containing true if target velocity reached, false
     * otherwise
     */
    Result<bool> IsTargetVelocityReached() noexcept;

    /**
     * @brief Stop the motor
     * @return Result<void> indicating success or error
     *
     * Stops the motor by setting VSTART and VMAX to 0.
     */
    Result<void> Stop() noexcept;

    /**
     * @brief Configure reference switches/endstops
     * @param config Reference switch configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void>
    ConfigureReferenceSwitch(const ReferenceSwitchConfig &config) noexcept;

    /**
     * @brief Get current reference switch configuration
     * @return Result<ReferenceSwitchConfig> containing the configuration or
     * error
     */
    Result<ReferenceSwitchConfig> GetReferenceSwitchConfig() noexcept;

    /**
     * @brief Set left switch active level (determines polarity)
     * @param active_level Active level (ACTIVE_LOW or ACTIVE_HIGH)
     * @return Result<void> indicating success or error
     *
     * Updates only the active level, preserving other settings.
     * Allows real-time polarity changes while keeping stop enable and latching
     * configured.
     */
    Result<void>
    SetLeftSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Set right switch active level (determines polarity)
     * @param active_level Active level (ACTIVE_LOW or ACTIVE_HIGH)
     * @return Result<void> indicating success or error
     *
     * Updates only the active level, preserving other settings.
     * Allows real-time polarity changes while keeping stop enable and latching
     * configured.
     */
    Result<void>
    SetRightSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Enable or disable motor stop on left switch
     * @param enable true to enable stop, false to disable
     * @return Result<void> indicating success or error
     *
     * Updates only stop enable, preserving other settings.
     * Allows real-time enable/disable of motor stop while keeping polarity and
     * latching configured.
     */
    Result<void> SetLeftSwitchStopEnable(bool enable) noexcept;

    /**
     * @brief Enable or disable motor stop on right switch
     * @param enable true to enable stop, false to disable
     * @return Result<void> indicating success or error
     *
     * Updates only stop enable, preserving other settings.
     * Allows real-time enable/disable of motor stop while keeping polarity and
     * latching configured.
     */
    Result<void> SetRightSwitchStopEnable(bool enable) noexcept;

    /**
     * @brief Set left switch latching mode
     * @param latch_mode Latching mode (DISABLED, ACTIVE_EDGE, INACTIVE_EDGE,
     * BOTH_EDGES)
     * @return Result<void> indicating success or error
     *
     * Updates only latching mode, preserving other settings.
     */
    Result<void> SetLeftSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept;

    /**
     * @brief Set right switch latching mode
     * @param latch_mode Latching mode (DISABLED, ACTIVE_EDGE, INACTIVE_EDGE,
     * BOTH_EDGES)
     * @return Result<void> indicating success or error
     *
     * Updates only latching mode, preserving other settings.
     */
    Result<void>
    SetRightSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept;

    /**
     * @brief Set stop mode (hard or soft stop)
     * @param stop_mode Stop mode (HARD_STOP or SOFT_STOP)
     * @return Result<void> indicating success or error
     *
     * Updates only stop mode, preserving other settings.
     */
    Result<void> SetStopMode(ReferenceStopMode stop_mode) noexcept;

    /**
     * @brief Get latched position
     * @param position Reference to store the latched position
     * @param unit Unit to return the position in (default: Steps)
     * @return Result<float> containing the value or error
     *
     * Reads the position that was latched on the last reference switch event.
     */
    Result<float> GetLatchedPosition(Unit unit) noexcept;

    /**
     * @brief Set position comparison register
     * @param value Position value for comparison
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * When XACTUAL equals X_COMPARE, the position pulse output becomes high.
     */
    Result<void> SetComparePosition(float value, Unit unit) noexcept;

    /**
     * @brief Set power down delay (raw register value)
     * @param tpowerdown Power down delay (0-255, time range ~0 to 5.6 seconds)
     * @return Result<void> indicating success or error
     *
     * Sets the delay before power down when motor enters standstill.
     * Minimum setting of 2 is required to allow automatic tuning of stealthChop
     * PWM_OFFS_AUTO.
     *
     * @note For user-friendly API, use SetPowerDownDelayMs() instead.
     */
    Result<void> SetPowerDownDelay(uint8_t tpowerdown) noexcept;

    /**
     * @brief Set power down delay in milliseconds
     * @param delay_ms Power down delay in milliseconds (0.0 = instant,
     * automatically converted to register value 0-255)
     * @return Result<void> indicating success or error
     *
     * Sets the delay before power down when motor enters standstill.
     * The delay is automatically converted from milliseconds to register value
     * based on f_clk.
     *
     * Conversion: tpowerdown = round((delay_ms × f_clk) / (1000 × 2^18))
     *
     * @note Minimum setting of 2 register units is required to allow automatic
     * tuning of stealthChop PWM_OFFS_AUTO.
     * @note At 12 MHz: 1 register unit ≈ 21.85 ms, range ≈ 0-5.6 seconds
     */
    Result<void> SetPowerDownDelayMs(float delay_ms) noexcept;

    /**
     * @brief Set zero wait time (raw register value)
     * @param tzerowait Waiting time after ramping down to zero velocity in
     * clock cycles (0-65535)
     * @return Result<void> indicating success or error
     *
     * Sets the waiting time after ramping down to zero velocity before next
     * movement or direction inversion can start.
     *
     * @note For user-friendly API, use SetZeroWaitTimeMs() instead.
     */
    Result<void> SetZeroWaitTime(uint16_t tzerowait) noexcept;

    /**
     * @brief Set zero wait time in milliseconds
     * @param delay_ms Velocity-zero wait time in milliseconds (0.0 = no
     * waiting, automatically converted to register value 0-65535)
     * @return Result<void> indicating success or error
     *
     * Sets the waiting time after ramping down to zero velocity before next
     * movement or direction inversion can start.
     *
     * The delay is automatically converted from milliseconds to register value
     * based on f_clk.
     *
     * Conversion: tzerowait = round((delay_ms × f_clk) / (1000 × 2^18))
     *
     * @note Used only with the internal ramp generator (positioning / velocity
     * mode).
     * @note Has no effect in external Step/Dir mode (SD_MODE=1).
     * @note At 12 MHz: 1 register unit ≈ 21.85 ms, maximum ≈ 1430 seconds
     * (~23.8 minutes)
     */
    Result<void> SetZeroWaitTimeMs(float delay_ms) noexcept;

    /**
     * @brief Configure ramp generator from RampConfig structure
     * @param config Ramp configuration structure with all ramp parameters
     * @return Result<void> indicating success or error
     *
     * Configures all ramp parameters including velocities, accelerations, and
     * timing using the unit specifications from the config.
     */
    Result<void> ConfigureRamp(const RampConfig &config) noexcept;

    /**
     * @brief Set first acceleration phase
     * @param a1 First acceleration value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * Sets the first acceleration phase. If 0.0f, AMAX is used for this phase.
     */
    Result<void> SetFirstAcceleration(float a1, Unit unit) noexcept;

    /**
     * @brief Set final deceleration phase
     * @param d1 Deceleration value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * Sets the final deceleration phase (D1).
     * Attention: Do not set 0 in positioning mode (datasheet 6.3.1).
     * If set to 0, the driver might behave unexpectedly in positioning mode.
     * A safe minimum (e.g., 100) is recommended if unsure.
     */
    Result<void> SetFinalDeceleration(float d1, Unit unit) noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance

    // Internal helper methods (used by unit-aware public methods)
    /**
     * @brief Set XTARGET directly in register units
     * @param position Target position in **microsteps** (XTARGET register units)
     *
     * @note This bypasses unit conversion. XTARGET units depend on CHOPCONF.MRES
     *       (microstep resolution). Prefer the public float+Unit overload unless
     *       you intentionally work in raw register units.
     */
    Result<void> SetTargetPosition(int32_t position) noexcept;
    Result<void> SetCurrentPosition(int32_t position,
                                    bool update_encoder = false) noexcept;
  } rampControl{*this};

  //================================================================================
  //================================================================================
  //                                    MOTOR CONTROL STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Motor control subsystem
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for controlling motor current, chopper configuration,
   * and stealthChop operation.
   */
  struct MotorControl {
    /**
     * @brief Construct motor control subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit MotorControl(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Enable the motor driver
     * @return Result<void> indicating success or error
     */
    Result<void> Enable() noexcept;

    /**
     * @brief Disable the motor driver
     * @return Result<void> indicating success or error
     */
    Result<void> Disable() noexcept;

    /**
     * @brief Set motor run and hold current
     * @param irun Run current (0-31, where 31 = 100% of global scaler)
     * @param ihold Hold current (0-31, where 31 = 100% of global scaler)
     * @return Result<void> indicating success or error
     */
    Result<void> SetCurrent(uint8_t irun, uint8_t ihold) noexcept;

    /**
     * @brief Configure chopper settings
     * @param config Chopper configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureChopper(const ChopperConfig &config) noexcept;

    /**
     * @brief Change microstep resolution (CHOPCONF.MRES)
     * @param mres New microstep resolution
     * @param opts Options controlling rescaling behavior
     * @return Result<void> indicating success or error
     *
     * By default, this preserves physical meaning across the MRES change:
     * - Position registers (XACTUAL/XTARGET/X_COMPARE) are rescaled in microstep counts
     * - Motion profile registers are rewritten so speed/accel in user units remain unchanged
     * - Encoder scaling is recalculated if encoder is configured
     *
     * If the motor is not in standstill and opts.require_standstill is true (default),
     * the operation fails with INVALID_STATE.
     */
    Result<void> SetMicrostepResolution(
        MicrostepResolution mres,
        const MicrostepChangeOptions &opts = MicrostepChangeOptions{}) noexcept;

    /**
     * @brief Configure stealthChop settings
     * @param config StealthChop configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureStealthChop(const StealthChopConfig &config) noexcept;

    /**
     * @brief Configure power stage parameters (DRV_CONF register)
     * @param config Power stage parameters structure
     * @return Result<void> indicating success or error
     *
     * Configures MOSFET driver strength, break-before-make time,
     * over-temperature protection, and sense filter based on user-friendly
     * physical parameters.
     */
    Result<void>
    ConfigurePowerStage(const PowerStageParameters &config) noexcept;

    /**
     * @brief Configure motor current from motor specifications
     * @param motor_spec Motor specifications including current, sense resistor,
     * supply voltage
     * @return Result<void> indicating success or error
     *
     * Automatically calculates and sets IRUN, IHOLD, and GLOBAL_SCALER from
     * motor specifications. Also sets IHOLDDELAY if configured in motor_spec.
     */
    Result<void> ConfigureMotorCurrent(const MotorSpec &motor_spec) noexcept;

    /**
     * @brief Set mode change speeds
     * @param pwm_thrs Speed threshold for stealthChop
     * @param cool_thrs Speed threshold for coolStep
     * @param high_thrs Speed threshold for high-speed mode
     * @param unit Unit of the speed values (default: RevPerSec)
     * @return Result<void> indicating success or error
     */
    Result<void> SetModeChangeSpeeds(float pwm_thrs, float cool_thrs,
                                     float high_thrs, Unit unit) noexcept;

    /**
     * @brief Set CoolStep velocity threshold (TCOOLTHRS)
     * @param value Velocity threshold value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     */
    Result<void> SetCoolStepThreshold(float value, Unit unit) noexcept;

    /**
     * @brief Set High-Speed velocity threshold (THIGH)
     * @param value Velocity threshold value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     */
    Result<void> SetHighSpeedThreshold(float value, Unit unit) noexcept;

    /**
     * @brief Set StealthChop velocity threshold (TPWMTHRS)
     * @param value Velocity threshold value
     * @param unit Unit of the value (default: Steps)
     * @return Result<void> indicating success or error
     *
     * Sets the velocity threshold for switching between StealthChop and
     * SpreadCycle modes. Below this threshold, StealthChop is used (quiet
     * operation). Above this threshold, SpreadCycle is used (higher torque,
     * more noise).
     *
     * Setting to 0.0 disables the threshold (StealthChop always used if
     * enabled).
     *
     * @note This is automatically configured during Initialize() if
     * velocity_threshold is set in StealthChopConfig.
     */
    Result<void> SetStealthChopVelocityThreshold(float value,
                                                 Unit unit) noexcept;

    /**
     * @brief Get StealthChop velocity threshold (TPWMTHRS) from local storage
     * @param unit Unit of the returned threshold
     * @return Result<float> containing the threshold in requested units
     *
     * Returns the locally tracked value of TPWMTHRS register converted back to
     * a speed threshold. This is the velocity below which StealthChop may be
     * used (when StealthChop is enabled via GCONF.en_pwm_mode).
     *
     * @note TPWMTHRS uses the same TSTEP timebase conversion as TCOOLTHRS:
     * TSTEP = f_CLK / (speed_fullsteps_per_sec * 256).
     */
    Result<float> GetStealthChopVelocityThreshold(Unit unit) const noexcept;

    /**
     * @brief Set global current scaler
     * @param scaler Global scaler value (32-256)
     * @return Result<void> indicating success or error
     */
    Result<void> SetGlobalScaler(uint16_t scaler) noexcept;

    /**
     * @brief Get global configuration
     * @param config Reference to store current GlobalConfig
     * @return Result<GlobalConfig> containing the value or error
     */
    Result<GlobalConfig> GetGlobalConfig() noexcept;

    /**
     * @brief Configure CoolStep current reduction
     * @param config CoolStep configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureCoolStep(const CoolStepConfig &config) noexcept;

    /**
     * @brief Configure dcStep automatic commutation
     * @param config dcStep configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureDcStep(const DcStepConfig &config) noexcept;

    /**
     * @brief Set microstep lookup table entry
     * @param index Lookup table index (0-7)
     * @param value Lookup table value (32-bit)
     * @return Result<void> indicating success or error
     */
    Result<void> SetMicrostepLookupTable(uint8_t index,
                                         uint32_t value) noexcept;

    /**
     * @brief Set microstep lookup table segmentation
     * @param width_sel_0 Width selection for segment 0 (0-3)
     * @param width_sel_1 Width selection for segment 1 (0-3)
     * @param width_sel_2 Width selection for segment 2 (0-3)
     * @param width_sel_3 Width selection for segment 3 (0-3)
     * @param lut_seg_start1 Start position for segment 1 (0-255)
     * @param lut_seg_start2 Start position for segment 2 (0-255)
     * @param lut_seg_start3 Start position for segment 3 (0-255)
     * @return Result<void> indicating success or error
     */
    Result<void> SetMicrostepLookupTableSegmentation(
        uint8_t width_sel_0, uint8_t width_sel_1, uint8_t width_sel_2,
        uint8_t width_sel_3, uint8_t lut_seg_start1, uint8_t lut_seg_start2,
        uint8_t lut_seg_start3) noexcept;

    /**
     * @brief Set microstep lookup table start current
     * @param start_current Start current value (0-255)
     * @return Result<void> indicating success or error
     */
    Result<void> SetMicrostepLookupTableStart(uint16_t start_current) noexcept;

    /**
     * @brief Setup motor from high-level specifications
     * @param motor_spec Motor specification structure
     * @param mechanical_system Optional mechanical system configuration
     * @return Result<void> indicating success or error
     *
     * Automatically calculates and sets motor current, chopper configuration,
     * and other parameters based on motor specifications.
     */
    Result<void> SetupMotorFromSpec(
        const MotorSpec &motor_spec,
        const MechanicalSystem *mechanical_system = nullptr) noexcept;

    /**
     * @brief Configure global configuration (GCONF register)
     * @param config Global configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureGlobalConfig(const GlobalConfig &config) noexcept;

    /**
     * @brief Enable/Disable StealthChop (PWM mode)
     * @param enabled true to enable StealthChop, false for SpreadCycle
     * @return Result<void> indicating success or error
     */
    Result<void> SetStealthChopEnabled(bool enabled) noexcept;

    /**
     * @brief Get chopper configuration
     * @param config Reference to store current ChopperConfig
     * @return Result<ChopperConfig> containing the value or error
     */
    Result<ChopperConfig> GetChopperConfig() noexcept;

    /**
     * @brief Check if motor driver is enabled
     * @return Result<bool> containing true if motor is enabled (toff > 0),
     * false otherwise
     */
    Result<bool> IsEnabled() noexcept;

    /**
     * @brief Check if StealthChop is enabled
     * @return Result<bool> containing true if StealthChop is enabled, false
     * otherwise
     */
    Result<bool> IsStealthChopEnabled() noexcept;

    /**
     * @brief Check if StealthChop is calibrated and working
     * @return Result<bool> containing true if StealthChop is calibrated
     * (pwm_scale_auto != 0), false otherwise
     *
     * StealthChop requires calibration to work properly. If not calibrated,
     * the motor may not move or may have poor performance.
     */
    Result<bool> IsStealthChopCalibrated() noexcept;

    /**
     * @brief Get DIAG0 pin configuration
     * @param config Reference to store current Diag0Config
     * @return Result<Diag0Config> containing the value or error
     *
     * Reads DIAG0 configuration from GCONF register using read-modify-write
     * pattern.
     */
    Result<Diag0Config> GetDiag0Config() noexcept;

    /**
     * @brief Set DIAG0 pin configuration
     * @param config Diag0Config structure with DIAG0 settings
     * @return Result<void> indicating success or error
     *
     * Writes DIAG0 configuration to GCONF register using read-modify-write
     * pattern. Preserves all other GCONF bits.
     */
    Result<void> SetDiag0Config(const Diag0Config &config) noexcept;

    /**
     * @brief Get DIAG1 pin configuration
     * @param config Reference to store current Diag1Config
     * @return Result<Diag1Config> containing the value or error
     *
     * Reads DIAG1 configuration from GCONF register using read-modify-write
     * pattern.
     */
    Result<Diag1Config> GetDiag1Config() noexcept;

    /**
     * @brief Set DIAG1 pin configuration
     * @param config Diag1Config structure with DIAG1 settings
     * @return Result<void> indicating success or error
     *
     * Writes DIAG1 configuration to GCONF register using read-modify-write
     * pattern. Preserves all other GCONF bits.
     *
     * @note steps_skipped should not be enabled with other DIAG1 options
     * (mutually exclusive).
     */
    Result<void> SetDiag1Config(const Diag1Config &config) noexcept;

    /**
     * @brief Set coil currents for direct mode operation
     * @param coil_a Coil A current target (signed, range ±248 recommended for
     * safe operation)
     * @param coil_b Coil B current target (signed, range ±248 recommended for
     * safe operation)
     * @return Result<void> indicating success or error
     *
     * Sets coil current targets directly via XTARGET register when direct_mode
     * is enabled. In direct mode, XTARGET controls coil currents instead of
     * position:
     * - Bits 8..0: Coil A current (signed 9-bit)
     * - Bits 24..16: Coil B current (signed 9-bit)
     *
     * Current is scaled by IHOLD setting. The STEP/DIR inputs and motion
     * controller are not used in direct mode.
     *
     * @note Requires direct_mode to be enabled via ConfigureGlobalConfig().
     * @note For SpreadCycle mode, limit amplitude to ±248 for IHOLD up to 31
     * and hysteresis settings not exceeding 15 to keep sufficient regulation
     * margin.
     * @note Values are automatically constrained to valid 9-bit signed range
     * (-256 to +255).
     * @warning This method overwrites XTARGET register. Do not use with normal
     * position control.
     */
    Result<void> SetCoilCurrents(int16_t coil_a, int16_t coil_b) noexcept;

    /**
     * @brief Set motor power down delay (IHOLDDELAY) in milliseconds
     * @param total_delay_ms Total motor power down delay time in milliseconds
     * (0.0 = instant, automatically calculated to register value 0-15)
     * @return Result<void> indicating success or error
     *
     * Sets the total delay time for motor power down after motion as soon as
     * standstill is detected (stst=1) and TPOWERDOWN has expired.
     * The smooth transition avoids a motor jerk upon power down.
     *
     * The delay is automatically calculated from the desired total delay time,
     * clock frequency, and the number of current reduction steps (IRUN -
     * IHOLD).
     *
     * Calculation: IHOLDDELAY = round((total_delay_ms / (IRUN - IHOLD)) × f_clk
     * / (1000 × 2^18)) Total delay = (IRUN - IHOLD) × IHOLDDELAY × (2^18 /
     * f_clk)
     *
     * @note If IRUN == IHOLD, the delay is always 0 regardless of this setting.
     * @note At 12 MHz with 15 current steps: typical range is 200-700 ms total
     * delay.
     * @note The actual delay may differ slightly from the desired value due to
     * register quantization.
     * @note This reads the current IRUN and IHOLD values from the driver to
     * calculate the delay.
     */
    Result<void> SetIholdDelayMs(float total_delay_ms) noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } motorControl{*this};

  //================================================================================
  //================================================================================
  //                                    COMMUNICATION STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Communication subsystem
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for configuring UART node addressing and multi-chip
   * setups.
   */
  struct Communication {
    /**
     * @brief Construct communication subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Communication(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Set clock frequency on CLK pin
     * @param frequency_hz Clock frequency in Hz (0 = use internal clock, >0 =
     * external clock frequency)
     * @return Result<void> indicating success or error
     *
     * This method calls the communication interface's SetClkFreq() method to
     * configure the hardware clock pin. It does NOT update the driver's
     * internal f_clk_ value.
     *
     * **Clock Mode:**
     * - **Internal Clock**: Pass `frequency_hz = 0` to use internal oscillator
     * (CLK pin set to GND).
     * - **External Clock**: Pass `frequency_hz > 0` to provide external clock
     * signal (e.g., 12000000 for 12MHz).
     *
     * @note This is a low-level method that only configures the hardware pin.
     *       Use the overloaded version with ExternalClockConfig to also update
     * driver configuration.
     * @note If not supported (returns false), CLK pin should be tied to GND for
     * internal oscillator.
     */
    Result<void> SetClkFreq(uint32_t frequency_hz) noexcept {
      return driver_.comm_.SetClkFreq(frequency_hz);
    }

    /**
     * @brief Set clock frequency from ExternalClockConfig (high-level method)
     * @param config External clock configuration structure
     * @return Result<void> indicating success or error
     *
     * Configures the clock source (internal or external) and sets f_clk_ for
     * timing calculations. If config.frequency_hz = 0, uses internal 12 MHz
     * clock. If config.frequency_hz > 0, uses external clock at specified
     * frequency.
     *
     * This method:
     * - Updates the driver's internal clock frequency (f_clk_) for timing
     * calculations
     * - Optionally calls the communication interface's SetClkFreq() if
     * supported
     * - Updates the driver configuration
     *
     * **Clock Mode:**
     * - **Internal Clock**: Set `frequency_hz = 0` to use internal oscillator
     * (CLK pin tied to GND).
     * - **External Clock**: Set `frequency_hz > 0` to provide external clock
     * signal (e.g., 12000000 for 12MHz).
     *
     * @note This is automatically called during Initialize() with the value
     * from DriverConfig.
     * @note If SetClkFreq() is not supported (returns false), CLK pin should be
     * tied to GND for internal oscillator.
     * @note f_clk is used for all timing calculations regardless of clock
     * source.
     * @note Typical frequencies: 12MHz (default internal) or 24MHz (for higher
     * performance external).
     */
    Result<void> SetClkFreq(const ExternalClockConfig &config) noexcept;

    /**
     * @brief Configure UART node address and send delay (writes SLAVECONF
     * register)
     * @param node_address UART node address (0-254), same as slave address in
     * SLAVECONF
     * @param send_delay Number of bit times before replying to register read
     * (0-15), stored locally
     * @return Result<void> indicating success or error
     *
     * Writes the SLAVECONF register to configure the chip's UART node address
     * and send delay. Also updates the local software representation
     * (uart_node_address_ and send_delay_).
     *
     * @note This writes to hardware. For sequential programming, use
     * `uartConfig.ConfigureUartNodeAddress()` instead.
     * @note Send delay is stored locally since SLAVECONF register is
     * write-only.
     */
    Result<void> ConfigureUartNodeAddress(uint8_t node_address,
                                          uint8_t send_delay = 0) noexcept;

    /**
     * @brief Set the daisy-chain position for this TMC51x0 instance
     * @param position Position in daisy chain (0 = first chip/single chip, 1 =
     * second, etc.)
     *
     * This method configures the position of this driver in a daisy-chained SPI
     * setup. The position is used when calling ReadRegister() and
     * WriteRegister() to determine the correct padding for daisy-chain
     * communication.
     *
     * @note The daisy-chain position determines how many 40-bit dummy datagrams
     * are sent before this chip's command, ensuring the command reaches the
     * correct chip in the chain. Only applicable for SPI communication
     * interfaces.
     */
    void SetDaisyChainPosition(uint8_t position) noexcept {
      driver_.daisy_chain_position_ = position;
    }

    /**
     * @brief Get the current daisy-chain position for this TMC51x0 instance
     * @return Daisy-chain position (0 = first chip/single chip, 1 = second,
     * etc.)
     */
    [[nodiscard]] uint8_t GetDaisyChainPosition() const noexcept {
      return driver_.daisy_chain_position_;
    }

    /**
     * @brief Set the UART node address for this TMC51x0 instance
     * @param address UART node address (0-254)
     *
     * This method configures the node address of this driver in a UART
     * multi-node setup. The address is used when calling ReadRegister() and
     * WriteRegister() to determine the correct node address for UART
     * communication.
     *
     * @note The node address must be programmed into the chip via SLAVECONF
     * register (using ConfigureUartNodeAddress() or
     * uartConfig.ConfigureUartNodeAddress()). This method only updates the
     * software representation. Only applicable for UART communication
     * interfaces.
     */
    void SetUartNodeAddress(uint8_t address) noexcept {
      driver_.uart_node_address_ =
          address & 0xFF; // Address range is 0-254 (8-bit)
    }

    /**
     * @brief Get the current UART node address for this TMC51x0 instance
     * @return UART node address (0-254)
     */
    [[nodiscard]] uint8_t GetUartNodeAddress() const noexcept {
      return driver_.uart_node_address_;
    }

    /**
     * @brief Set the chip operating mode via SPI_MODE and SD_MODE pins
     * @param mode Operating mode (SPI_INTERNAL_RAMP, SPI_EXTERNAL_STEPDIR,
     * UART_INTERNAL_RAMP, STANDALONE_EXTERNAL_STEPDIR)
     * @return Result<void> indicating success or error
     *
     * This method controls the SPI_MODE (pin 22) and SD_MODE (pin 21) pins if
     * they are connected to GPIO outputs. These pins determine both the
     * communication interface (SPI/UART/Standalone) and the motion control
     * method (Internal ramp generator vs External step/dir).
     *
     * ⚠️ CRITICAL WARNINGS:
     * - These pins are typically hardwired and read at startup
     * - Only use this method if SPI_MODE and SD_MODE pins are connected to GPIO
     * outputs
     * - Changing the mode requires a chip reset (power cycle) to take effect
     * - The mode pins are read at startup, so changes won't be effective until
     * reset
     * - Ensure pins are configured in TMC51x0PinConfig (spi_mode_pin and
     * sd_mode_pin)
     *
     * @note After calling this method, you must reset the chip for the new mode
     * to take effect. The driver does not automatically reset the chip - you
     * must handle this externally.
     *
     * @note Operating mode pin mapping:
     * - SPI_INTERNAL_RAMP: SPI_MODE=HIGH, SD_MODE=LOW (SPI interface + internal
     * ramp generator)
     * - SPI_EXTERNAL_STEPDIR: SPI_MODE=HIGH, SD_MODE=HIGH (SPI interface +
     * external step/dir inputs)
     * - UART_INTERNAL_RAMP: SPI_MODE=LOW, SD_MODE=LOW (UART interface +
     * internal ramp generator)
     * - STANDALONE_EXTERNAL_STEPDIR: SPI_MODE=LOW, SD_MODE=HIGH (Standalone +
     * external step/dir, CFG pins configure driver)
     */
    Result<void> SetOperatingMode(ChipCommMode mode) noexcept;

    /**
     * @brief Get the current chip operating mode from SPI_MODE and SD_MODE pins
     * @param mode Reference to store the current mode
     * @return Result<ChipCommMode> containing the value or error
     *
     * This method reads the current state of SPI_MODE (pin 22) and SD_MODE (pin
     * 21) pins if they are connected to GPIO inputs/outputs.
     *
     * @note This reads the current pin state, which may not reflect the actual
     * chip mode if the chip hasn't been reset since the pins were changed.
     */
    Result<ChipCommMode> GetOperatingMode() const noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } communication{*this};

  //================================================================================
  //================================================================================
  //                                    ENCODER STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Encoder subsystem
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for encoder configuration and reading encoder position.
   */
  struct Encoder {
    /**
     * @brief Construct encoder subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Encoder(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure encoder settings
     * @param config Encoder configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void> Configure(const EncoderConfig &config) noexcept;

    /**
     * @brief Get current encoder configuration
     * @param config Reference to store current configuration
     * @return Result<EncoderConfig> containing the value or error
     */
    Result<EncoderConfig> GetEncoderConfig() noexcept;

    /**
     * @brief Set N channel active level (determines polarity)
     * @param active_level Active level (ACTIVE_LOW or ACTIVE_HIGH)
     * @return Result<void> indicating success or error
     *
     * Updates only the active level, preserving other settings.
     * Shares ReferenceSwitchActiveLevel enum with reference switches.
     */
    Result<void>
    SetNChannelActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Set N channel sensitivity (edge/level detection)
     * @param sensitivity Sensitivity mode (ACTIVE_LEVEL, RISING_EDGE,
     * FALLING_EDGE, BOTH_EDGES)
     * @return Result<void> indicating success or error
     *
     * Updates only sensitivity, preserving other settings.
     */
    Result<void>
    SetNChannelSensitivity(EncoderNSensitivity sensitivity) noexcept;

    /**
     * @brief Set encoder clear mode
     * @param clear_mode Clear mode (DISABLED, ONCE, CONTINUOUS)
     * @return Result<void> indicating success or error
     *
     * Updates only clear mode, preserving other settings.
     */
    Result<void> SetClearMode(EncoderClearMode clear_mode) noexcept;

    /**
     * @brief Set encoder prescaler mode
     * @param prescaler_mode Prescaler mode (BINARY or DECIMAL)
     * @return Result<void> indicating success or error
     *
     * Updates only prescaler mode, preserving other settings.
     */
    Result<void> SetPrescalerMode(EncoderPrescalerMode prescaler_mode) noexcept;

    /**
     * @brief Get encoder position
     * @param position Reference to store encoder position in steps
     * @return Result<int32_t> containing the value or error
     */
    Result<int32_t> GetPosition() noexcept;

    /**
     * @brief Set encoder resolution
     * @param motor_steps Number of steps per turn for the motor
     * @param enc_resolution Actual encoder resolution (pulses per turn)
     * @param inverted Whether encoder and motor rotations are inverted
     * @return Result<void> indicating success or error
     */
    Result<void> SetResolution(int32_t motor_steps, int32_t enc_resolution,
                               bool inverted = false) noexcept;

    /**
     * @brief Set encoder allowed deviation
     * @param steps Maximum number of steps deviation before warning
     * @return Result<void> indicating success or error
     */
    Result<void> SetAllowedDeviation(int32_t steps) noexcept;

    /**
     * @brief Check if encoder deviation detected
     * @return Result<bool> containing true if deviation detected, false
     * otherwise
     */
    Result<bool> IsDeviationDetected() noexcept;

    /**
     * @brief Clear encoder deviation flag
     * @return Result<void> indicating success or error
     */
    Result<void> ClearDeviationFlag() noexcept;

    /**
     * @brief Get encoder latched position
     * @param position Reference to store encoder position latched on N event
     * @return Result<int32_t> containing the value or error
     *
     * Reads the encoder position that was latched on the last N channel event.
     */
    Result<int32_t> GetLatchedPosition() noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } encoder{*this};

  //================================================================================
  //================================================================================
  //                                    DIAGNOSTICS STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Diagnostics subsystem
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for reading driver status, StallGuard values, and error
   * detection.
   */
  struct Diagnostics {
    /**
     * @brief Construct diagnostics subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Diagnostics(TMC51x0 &driver) noexcept : driver_(driver) {}

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
     * @return Result<bool> containing true if read successfully, false
     * otherwise
     */
    Result<bool> GetGlobalStatus(bool &drv_err, bool &uv_cp) noexcept;

    /**
     * @brief Get StallGuard2 value
     * @param value Reference to store StallGuard2 value (0-1023)
     * @return Result<uint16_t> containing the value or error
     */
    Result<uint16_t> GetStallGuard() noexcept;

    /**
     * @brief Get StallGuard2 result from DRV_STATUS register
     * @param sg_result Reference to store StallGuard2 value (0-1023)
     * @return Result<uint16_t> containing the value or error
     */
    Result<uint16_t> GetStallGuardResult() noexcept;

    /**
     * @brief Configure StallGuard2
     * @param config StallGuard configuration structure
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureStallGuard(const StallGuardConfig &config) noexcept;

    /**
     * @brief Enable/Disable stop on stall (sg_stop in SW_MODE)
     * @param enable true to enable stop on stall, false to disable
     * @return Result<void> indicating success or error
     */
    Result<void> EnableStopOnStall(bool enable) noexcept;

    /**
     * @brief Check if stop on stall is enabled
     * @return Result<bool> containing true if stop on stall is enabled, false
     * otherwise
     */
    Result<bool> IsStopOnStallEnabled() noexcept;

    /**
     * @brief Enable/Disable soft stop (en_softstop in SW_MODE)
     * @param enable true to enable soft stop, false to disable
     * @return Result<void> indicating success or error
     */
    Result<void> SetSoftStop(bool enable) noexcept;

    /**
     * @brief Check if soft stop is enabled
     * @return Result<bool> containing true if soft stop is enabled, false
     * otherwise
     */
    Result<bool> IsSoftStopEnabled() noexcept;

    /**
     * @brief Clear stall event flag (event_stop_sg in RAMP_STAT)
     * @return Result<void> indicating success or error
     */
    Result<void> ClearStallFlag() noexcept;

    /**
     * @brief Check if stall was detected
     * 
     * Automatically adapts based on whether stop-on-stall (sg_stop) is enabled:
     * - If sg_stop is enabled: Checks event_stop_sg flag (hardware sets this when stall detected)
     * - If sg_stop is disabled: Checks status_sg and SG_RESULT directly
     *   (status_sg indicates StallGuard2 is active, SG_RESULT <= 10 indicates high load/stall)
     * 
     * This method abstracts away the complexity of knowing which register to check
     * based on the driver configuration.
     * 
     * @return Result<bool> containing true if stall detected, false otherwise
     */
    Result<bool> IsStallDetected() noexcept;

    /**
     * @brief Get driver status register value
     * @param status Reference to store the DRV_STATUS register value
     * @return Result<uint32_t> containing the value or error
     */
    Result<uint32_t> GetDriverStatusRegister() noexcept;

    /**
     * @brief Check if open load is detected on phase A
     * @return Result<bool> containing true if open load detected on phase A,
     * false otherwise
     *
     * Open load detection indicates an interrupted cable or connector issue.
     *
     * @note Requirements for reliable detection:
     * - Must operate in SpreadCycle mode (StealthChop disabled)
     * - Motor must be moving (minimum 4× microstep resolution in single
     * direction)
     * - Use low or nominal motor velocity
     * - Cannot be detected in standstill (coils may have zero current)
     *
     * @warning Open load flags are informative only and do not cause driver
     * action. Also triggered by undervoltage, high velocity, short circuit, or
     * overtemperature conditions.
     *
     * @see Datasheet section 11.3: Open Load Diagnostics
     */
    Result<bool> IsOpenLoadA() noexcept;

    /**
     * @brief Check if open load is detected on phase B
     * @return Result<bool> containing true if open load detected on phase B,
     * false otherwise
     *
     * Open load detection indicates an interrupted cable or connector issue.
     *
     * @note Requirements for reliable detection:
     * - Must operate in SpreadCycle mode (StealthChop disabled)
     * - Motor must be moving (minimum 4× microstep resolution in single
     * direction)
     * - Use low or nominal motor velocity
     * - Cannot be detected in standstill (coils may have zero current)
     *
     * @warning Open load flags are informative only and do not cause driver
     * action. Also triggered by undervoltage, high velocity, short circuit, or
     * overtemperature conditions.
     *
     * @see Datasheet section 11.3: Open Load Diagnostics
     */
    Result<bool> IsOpenLoadB() noexcept;

    /**
     * @brief Check if open load is detected on either phase
     * @param phase_a Reference to store phase A status (true if open load
     * detected)
     * @param phase_b Reference to store phase B status (true if open load
     * detected)
     * @return Result<bool> containing true if read successfully, false
     * otherwise
     *
     * Convenience method to check both phases at once.
     *
     * @note Requirements for reliable detection:
     * - Must operate in SpreadCycle mode (StealthChop disabled)
     * - Motor must be moving (minimum 4× microstep resolution in single
     * direction)
     * - Use low or nominal motor velocity
     * - Cannot be detected in standstill (coils may have zero current)
     *
     * @warning Open load flags are informative only and do not cause driver
     * action. Also triggered by undervoltage, high velocity, short circuit, or
     * overtemperature conditions.
     *
     * @see Datasheet section 11.3: Open Load Diagnostics
     */
    Result<bool> CheckOpenLoad(bool &phase_a, bool &phase_b) noexcept;

    /**
     * @brief Get ramp status register value
     * @param status Reference to store the RAMP_STAT register value
     * @return Result<uint32_t> containing the value or error
     */
    Result<uint32_t> GetRampStatusRegister() noexcept;

    /**
     * @brief Get locally tracked TPWMTHRS register value (raw)
     * @return Raw TPWMTHRS register value (20-bit meaningful range)
     */
    uint32_t GetTpwmthrsRegisterValue() const noexcept;

    /**
     * @brief Get locally tracked TCOOLTHRS register value (raw)
     * @return Raw TCOOLTHRS register value (20-bit meaningful range)
     */
    uint32_t GetTcoolthrsRegisterValue() const noexcept;

    /**
     * @brief Clear specific bits in RAMP_STAT register
     * @param bits_to_clear Bits to clear (write 1 to clear corresponding bit)
     * @return Result<void> indicating success or error
     *
     * RAMP_STAT is a read-write-clear register. Writing 1 to a bit clears it.
     * Common bits to clear:
     * - event_stop_sg (bit 0): Stall event flag
     * - event_stop_l (bit 1): Left switch event flag
     * - event_stop_r (bit 2): Right switch event flag
     * - event_pos_reached (bit 3): Position reached flag
     * - velocity_reached (bit 4): Velocity reached flag
     */
    Result<void> ClearRampStatus(uint32_t bits_to_clear) noexcept;

    /**
     * @brief Set TCOOLTHRS register directly
     * @param threshold Velocity threshold in steps/s (0 = disable, max =
     * 0xFFFFF)
     * @return Result<void> indicating success or error
     *
     * Sets the lower threshold velocity for CoolStep and StallGuard2.
     * When TSTEP < TCOOLTHRS (velocity > threshold), CoolStep and StallGuard2
     * are enabled. Setting to 0 disables these features at all speeds.
     *
     * @note For high-level configuration, use ConfigureStallGuard() instead.
     */
    Result<void> SetTcoolthrs(float threshold, Unit unit) noexcept;

    /**
     * @brief Get TCOOLTHRS register value (from local storage)
     * @param threshold Reference to store the threshold value
     * @param unit Unit to return the threshold in (default: Steps)
     * @return Result<float> containing the value or error
     *
     * Returns the locally tracked value of TCOOLTHRS register.
     * This register is write-only, so we track it locally.
     */
    Result<float> GetTcoolthrs(Unit unit) const noexcept;

    /**
     * @brief Get lost steps counter
     * @param steps Reference to store the number of lost steps
     * @return Result<uint32_t> containing the value or error
     *
     * Only valid when dcStep mode is enabled (SD_MODE = 1).
     */
    Result<uint32_t> GetLostSteps() noexcept;

    /**
     * @brief Get actual time between microsteps
     * @param time Reference to store time between microsteps in clock cycles
     * @return Result<uint32_t> containing the value or error
     *
     * Read-only register showing actual time between microsteps.
     */
    Result<uint32_t> GetTimeBetweenMicrosteps() noexcept;

    /**
     * @brief Get microstep counter
     * @param counter Reference to store actual position in microstep table
     * (0-1023)
     * @return Result<uint16_t> containing the value or error
     *
     * Read-only register showing actual position in the microstep table.
     */
    Result<uint16_t> GetMicrostepCounter() noexcept;

    /**
     * @brief Get microstep current
     * @param phase_a Reference to store phase A current (signed, -256 to 255)
     * @param phase_b Reference to store phase B current (signed, -256 to 255)
     * @return Result<int16_t> containing the value or error
     *
     * Read-only register showing actual microstep current for both phases.
     * Values are signed 9-bit as read from MSLUT (not scaled by current).
     */
    Result<int16_t> GetMicrostepCurrent(int16_t &phase_b) noexcept;

    /**
     * @brief Get PWM scale results
     * @param pwm_scale_sum Reference to store actual PWM duty cycle (0-255)
     * @param pwm_scale_auto Reference to store automatic regulation result
     * (signed -255...+255)
     * @return Result<uint8_t> containing the value or error
     *
     * Read-only register showing stealthChop PWM scale results.
     */
    Result<uint8_t> GetPwmScale(int16_t &pwm_scale_auto) noexcept;

    /**
     * @brief Get automatically determined PWM values
     * @param pwm_ofs_auto Reference to store auto-determined offset (0-255)
     * @param pwm_grad_auto Reference to store auto-determined gradient (0-255)
     * @return Result<uint8_t> containing the value or error
     *
     * Read-only register showing automatically determined PWM configuration
     * values.
     */
    Result<uint8_t> GetPwmAuto(uint8_t &pwm_grad_auto) noexcept;

    /**
     * @brief Read GPIO input pins
     * @param input_status Reference to store parsed input pin states
     * @return Result<InputStatus> containing the value or error
     *
     * Reads the state of all GPIO input pins and the IC version from register
     * 0x04 (IOIN).
     */
    Result<InputStatus> ReadInputStatus() noexcept;

    /**
     * @brief Read IC version
     * @param version Reference to store the 8-bit IC version
     * @return Result<uint8_t> containing the value or error
     *
     * Reads the VERSION field from IOIN register (0x04).
     * Expected values: 0x11 for TMC5130, 0x30 for TMC5160.
     */
    Result<uint8_t> ReadIcVersion() noexcept;

    /**
     * @brief Read GPIO input pins (raw)
     * @param io_pins Reference to store raw IO pin register value
     * @return Result<uint32_t> containing the value or error
     *
     * Reads the raw state of all GPIO input pins (register 0x04).
     */
    Result<uint32_t> ReadGpioPins() noexcept;

    /**
     * @brief Read factory configuration
     * @param fclktrim Reference to store FCLKTRIM value (0-31)
     * @return Result<uint8_t> containing the value or error
     *
     * Reads the factory configuration/clock trim value.
     */
    Result<uint8_t> ReadFactoryConfig() noexcept;

    /**
     * @brief Set SDO_CFG0 pin polarity (UART/Single Wire mode)
     * @param polarity Output pin polarity (false=normal/active high,
     * true=inverted/active low)
     * @return Result<void> indicating success or error
     *
     * Sets the polarity of the SDO_CFG0 pin when used as Next Address Output
     * (NAO) in single-wire UART chain mode.
     *
     * @note This affects the OUTPUT register (0x04), bit 0.
     *       The reset value is 1 (active low/inverted) for use as NAO.
     */
    Result<void> SetSdoCfg0Polarity(bool polarity) noexcept;

    /**
     * @brief Read OTP configuration
     * @param otp_fclktrim Reference to store OTP FCLKTRIM (0-31)
     * @param otp_s2_level Reference to store OTP S2 level (0-1)
     * @param otp_bbm Reference to store OTP BBM (0-1)
     * @param otp_tbl Reference to store OTP TBL (0-1)
     * @return Result<uint8_t> containing the value or error
     *
     * Reads the one-time programmable configuration memory.
     */
    Result<uint8_t> ReadOtpConfig(bool &otp_s2_level, bool &otp_bbm,
                                  bool &otp_tbl) noexcept;

    /**
     * @brief Get UART transmission counter
     * @return UART transmission counter value, or 0 on error
     *
     * Returns the number of UART transmissions since last read.
     */
    Result<uint8_t> GetUartTransmissionCount() noexcept;

    /**
     * @brief Read offset calibration results
     * @param phase_a Reference to store phase A offset (0-255)
     * @param phase_b Reference to store phase B offset (0-255)
     * @return Result<uint8_t> containing the value or error
     *
     * Reads the results from offset calibration procedure.
     */
    Result<uint8_t> ReadOffsetCalibration(uint8_t &phase_b) noexcept;

    /**
     * @brief Run comprehensive startup verification
     *
     * Performs a full verification of the driver setup including:
     * - IC Version check
     * - Input pin state logging
     * - Critical register checks
     *
     * Logs all findings using the system logger.
     * @return Result<void> indicating success or error
     */
    Result<void> VerifySetup() noexcept;

    /**
     * @brief Get detected chip version
     * @return Chip version (0x11 = TMC5130, 0x30 = TMC5160, 0x00 = unknown)
     */
    uint8_t GetChipVersion() const noexcept { return driver_.chip_version_; }

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } diagnostics{*this};

  //================================================================================
  //================================================================================
  //                                    TUNING STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Tuning subsystem for automatic parameter optimization
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for automatically tuning driver parameters such as
   * StallGuard2 threshold (SGT) for optimal performance.
   */
  struct Tuning {
    /**
     * @brief Construct tuning subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Tuning(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Automatically tune StallGuard threshold (SGT) with comprehensive
     * velocity range analysis
     * @param target_velocity Target velocity for tuning (most important -
     * optimal SGT is determined here)
     * @param result Reference to store comprehensive tuning results
     * @param min_sgt Minimum SGT to try (default: -10)
     * @param max_sgt Maximum SGT to try (default: 63)
     * @param acceleration Acceleration/deceleration (default: 3000.0f
     * steps/s^2)
     * @param min_velocity Minimum velocity to verify tuning at (0 = disabled,
     * used to determine SGT range)
     * @param max_velocity Maximum velocity to verify tuning at (0 = disabled,
     * used to determine SGT range)
     * @param velocity_unit Unit for velocity parameters (default: RevPerSec)
     * @param acceleration_unit Unit for acceleration parameter (default:
     * RevPerSec, RPM is not valid)
     * @return Result<void> indicating success or error
     *
     * Implements a comprehensive automatic tuning algorithm that prioritizes
     * target velocity:
     * 1. Finds optimal SGT at target_velocity (primary goal - most important)
     * 2. Verifies optimal SGT works at min_velocity and max_velocity (if
     * specified)
     * 3. If min/max velocities don't work with optimal SGT, finds what
     * velocities DO work
     * 4. Reports all findings in the result structure
     *
     * The algorithm ensures:
     * - Target velocity gets the best possible SGT value (highest priority)
     * - Min/max velocities are tested to determine the usable SGT range
     * - If requested velocities aren't achievable, actual achievable velocities
     * are reported
     *
     * @note Target velocity is the most important parameter - optimal SGT is
     * determined here first
     * @note Min and max velocities are used to determine the range of SGT
     * values that work
     * @note If min/max velocities are not possible, actual_min_velocity and
     * actual_max_velocity in the result struct contain the velocities that DO
     * work with the optimal SGT
     */
    Result<void>
    TuneStallGuard(float target_velocity, StallGuardTuningResult &result,
                   int8_t min_sgt = -10, int8_t max_sgt = 63,
                   float acceleration = 0.06F, float min_velocity = 0.0F,
                   float max_velocity = 0.0F,
                   Unit velocity_unit = Unit::RevPerSec,
                   Unit acceleration_unit = Unit::RevPerSec) noexcept;

    /**
     * @brief Legacy overload: Automatically tune StallGuard threshold (SGT)
     * @param target_velocity Velocity to tune at
     * @param final_sgt Reference to store the tuned SGT value
     * @param min_sgt Minimum SGT to try (default: -10)
     * @param max_sgt Maximum SGT to try (default: 63)
     * @param acceleration Acceleration/deceleration (default: 3000.0f
     * steps/s^2)
     * @param min_velocity Minimum velocity to verify tuning at (0 = disabled)
     * @param max_velocity Maximum velocity to verify tuning at (0 = disabled)
     * @param velocity_unit Unit for velocity parameters (default: RevPerSec)
     * @param acceleration_unit Unit for acceleration parameter (default:
     * RevPerSec, RPM is not valid)
     * @return Result<void> indicating success or error
     *
     * @deprecated Use the overload that returns StallGuardTuningResult for
     * comprehensive results
     *
     * This is a convenience wrapper around the new comprehensive tuning
     * function. For better results and velocity range analysis, use the
     * StallGuardTuningResult version.
     */
    [[deprecated("Use TuneStallGuard with StallGuardTuningResult for "
                 "comprehensive results")]]
    Result<void>
    TuneStallGuard(float target_velocity, int8_t &final_sgt,
                   int8_t min_sgt = -10, int8_t max_sgt = 63,
                   float acceleration = 0.06F, float min_velocity = 0.0F,
                   float max_velocity = 0.0F,
                   Unit velocity_unit = Unit::RevPerSec,
                   Unit acceleration_unit = Unit::RevPerSec) noexcept;

    /**
     * @brief Comprehensive automatic StallGuard tuning with safe current margin
     * and optional encoder verification
     * @param target_velocity Target velocity for tuning (most important -
     * optimal SGT is determined here)
     * @param result Reference to store comprehensive tuning results
     * @param min_sgt Minimum SGT to try (default: 0, negative values may cause
     * false stalls)
     * @param max_sgt Maximum SGT to try (default: 63)
     * @param acceleration Acceleration/deceleration (default: 3000.0f
     * steps/s^2)
     * @param min_velocity Minimum velocity to verify tuning at (0 = disabled)
     * @param max_velocity Maximum velocity to verify tuning at (0 = disabled)
     * @param velocity_unit Unit for velocity parameters (default: RevPerSec)
     * @param acceleration_unit Unit for acceleration parameter (default:
     * RevPerSec, RPM is not valid)
     * @param safe_current_margin_mA Safe current margin in milliamps (0 = no
     * margin, use nominal current)
     * @return Result<void> indicating success or error
     *
     * This is an enhanced version of TuneStallGuard that implements
     * comprehensive automatic tuning following Trinamic application note AN-002
     * guidelines and industry best practices:
     *
     * **Key Features:**
     * - **Safe Current Margin**: Reduces motor current by specified margin (in
     * mA) for safer tuning and improved StallGuard sensitivity. Current is
     * automatically restored after tuning.
     * - **Comprehensive Preparation**: Disables interfering features (CoolStep,
     * StallGuard filter, stop-on-stall) during tuning, then restores them
     * afterward.
     * - **Systematic SGT Scanning**: Searches for optimal SGT value that
     * provides SG_RESULT in the ideal range (100-500) at no-load, ensuring
     * reliable stall detection.
     * - **Velocity Range Validation**: Tests optimal SGT at min/max velocities
     * to determine actual operating range.
     * - **Optional Encoder Verification**: If encoder is available, can verify
     * stall detection accuracy (requires external implementation).
     *
     * **Tuning Process:**
     * 1. Save current motor settings (IRUN, IHOLD, GLOBAL_SCALER, CoolStep
     * config)
     * 2. Apply safe current margin if specified (reduces current for safer
     * tuning)
     * 3. Disable CoolStep (SGMIN=0) to prevent current modulation during tuning
     * 4. Disable StallGuard filter (SFILT=0) for immediate response during
     * calibration
     * 5. Disable stop-on-stall to allow manual observation during tuning
     * 6. Find optimal SGT at target velocity (primary goal)
     * 7. Verify optimal SGT works at min/max velocities (if specified)
     * 8. Restore all saved settings
     *
     * **Current Margin Calculation:**
     * The function calculates the new current by subtracting
     * safe_current_margin_mA from the current motor current. It uses the
     * driver's current calculation functions to determine the appropriate IRUN
     * and GLOBAL_SCALER values. The current is constrained to ensure the motor
     * can still move (minimum IRUN=8 for StealthChop compatibility).
     *
     * @note Target velocity is the most important parameter - optimal SGT is
     * determined here first
     * @note If safe_current_margin_mA is 0, the motor current is not changed
     * @note Current margin is applied by recalculating IRUN/GLOBAL_SCALER from
     * the reduced current
     * @note All settings (current, CoolStep, filter) are automatically restored
     * after tuning
     * @note This function takes several seconds to complete (typically 5-30
     * seconds depending on SGT range)
     * @note For best results, ensure the motor is unloaded during tuning
     *
     * @see TuneStallGuard() for a simpler version without current margin
     * handling
     */
    Result<void>
    AutoTuneStallGuard(float target_velocity, StallGuardTuningResult &result,
                       int8_t min_sgt = 0, int8_t max_sgt = 63,
                       float acceleration = 0.06F, float min_velocity = 0.0F,
                       float max_velocity = 0.0F,
                       Unit velocity_unit = Unit::RevPerSec,
                       Unit acceleration_unit = Unit::RevPerSec,
                       uint16_t safe_current_margin_mA = 0) noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } tuning{*this};

  //================================================================================
  //================================================================================
  //                                    HOMING STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Homing subsystem with automatic settings caching
   * @ingroup TMC51X0_Subsystems
   *
   * Provides homing methods that automatically cache and restore
   * settings modified during homing operations.
   */
  struct Homing {
    /**
     * @brief Construct homing subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Homing(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Perform sensorless homing using StallGuard2 (with settings
     * caching)
     * @param direction Direction to search (true = positive, false = negative)
     * @param search_speed Search speed in steps/s
     * @param final_position Reference to store final position after homing
     * @param timeout_ms Maximum time to wait in milliseconds (default: 10000)
     * @return Result<void> indicating success or error
     *
     * This is a blocking function that automatically:
     * - Caches current settings (StealthChop, SW_MODE, ramp settings)
     * - Disables StealthChop if enabled (StallGuard requires SpreadCycle)
     * - Uses existing StallGuard configuration (SGT threshold from motor
     * config)
     * - Enables sg_stop and waits for stall event (SG_RESULT reaches threshold)
     * - Restores cached settings after homing completes
     *
     * @note StallGuard threshold (SGT) should be configured via Initialize() or
     * ConfigureStallGuard() before calling this method. The method uses the
     * existing SGT configuration.
     */
    Result<void> PerformSensorlessHoming(bool direction, float search_speed,
                                         int32_t &final_position,
                                         uint32_t timeout_ms = 10000) noexcept;

    /**
     * @brief Perform homing using a reference switch (with settings caching)
     * @param direction Direction to search (true = positive, false = negative)
     * @param search_speed Speed for homing search in steps/s
     * @param switch_speed Speed for slow approach (unused, reserved for future)
     * @param final_position Reference to store final position after homing
     * @param use_left_switch true to use REFL, false to use REFR
     * @param timeout_ms Maximum time to wait in milliseconds (default: 10000)
     * @return Result<void> indicating success or error
     *
     * This is a blocking function that automatically:
     * - Caches current settings (SW_MODE, ramp settings)
     * - Configures switches and performs homing
     * - Restores cached settings after homing completes
     */
    Result<void> PerformSwitchHoming(bool direction, float search_speed,
                                     float switch_speed,
                                     int32_t &final_position,
                                     bool use_left_switch,
                                     uint32_t timeout_ms = 10000) noexcept;

  private:
    TMC51x0 &driver_;
    HomingSettingsCache cache_{};

    Result<void> CacheCurrentSettings() noexcept;
    Result<void> RestoreCachedSettings() noexcept;
    Result<void>
    EnsureSpreadCycleForStallGuard() noexcept; // Disable StealthChop if needed
  } homing{*this};

  //================================================================================
  //================================================================================
  //                                    PRINTER STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Register printer subsystem for debugging
   * @ingroup TMC51X0_Subsystems
   *
   * Prints register values using the communication interface's LogDebug()
   * method. All methods use TMC51X0_LOG_DEBUG() macro which respects
   * TMC51X0_DISABLE_DEBUG_LOGGING.
   */
  struct Printer {
    /**
     * @brief Construct printer subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Printer(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Print GCONF register
     */
    void PrintGconf() noexcept;

    /**
     * @brief Print GSTAT register (read and clear)
     */
    void PrintGstat() noexcept;

    /**
     * @brief Print RAMP_STAT register
     */
    void PrintRampStat() noexcept;

    /**
     * @brief Print DRV_STATUS register
     */
    void PrintDrvStatus() noexcept;

    /**
     * @brief Print CHOPCONF register
     */
    void PrintChopconf() noexcept;

    /**
     * @brief Print PWMCONF register
     */
    void PrintPwmconf() noexcept;

    /**
     * @brief Print PWM_SCALE register
     */
    void PrintPwmScale() noexcept;

    /**
     * @brief Print SW_MODE register
     */
    void PrintSwMode() noexcept;

    /**
     * @brief Print IOIN register
     */
    void PrintIoin() noexcept;

    /**
     * @brief Print all common registers
     */
    void PrintAll() noexcept;

  private:
    TMC51x0 &driver_;

    void PrintRegisterField(const char *name, uint32_t value,
                            const char *format = "0x%08X") noexcept;
  } printer{*this};

  /**
   * @brief UART configuration subsystem
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for configuring UART node addressing for multi-node
   * operation.
   */
  struct UartConfig {
    TMC51x0 *driver_; ///< Pointer to parent driver instance

    /**
     * @brief Construct UART configuration subsystem
     * @param driver Pointer to parent TMC51x0 driver instance
     */
    explicit UartConfig(TMC51x0 *driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure UART node address and send delay (for sequential
     * programming)
     * @param node_address UART node address (0-254)
     * @param send_delay Number of bit times before replying (0-15)
     * @return Result<void> indicating success or error
     *
     * Writes to SLAVECONF register using address 0 (for sequential programming
     * via NAI/NAO pins). This is used during sequential programming when
     * devices are accessible at address 0.
     *
     * @note For normal operation, use
     * `communication.ConfigureUartNodeAddress()` instead.
     * @note Per datasheet, devices are typically programmed backwards from
     * address 254 (254, 253, 252, ...).
     */
    Result<void> ConfigureUartNodeAddress(uint8_t node_address,
                                          uint8_t send_delay) noexcept;
  } uartConfig{this};

  /**
   * @brief Protection subsystem
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for configuring protection systems including short circuit
   * detection and overtemperature protection.
   */
  struct Protection {
    /**
     * @brief Construct protection subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Protection(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure short protection levels
     * @param config Power stage parameters structure (contains short protection
     * fields)
     * @return Result<void> indicating success or error
     */
    Result<void>
    ConfigureShortProtection(const PowerStageParameters &config) noexcept;

    /**
     * @brief Set short protection levels
     * @param s2vs_level Short to VS detector sensitivity (4-15)
     * @param s2g_level Short to GND detector sensitivity (2-15)
     * @param shortfilter Spike filtering bandwidth (0-3)
     * @param shortdelay Short detection delay (0-1)
     * @return Result<void> indicating success or error
     */
    Result<void> SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level,
                                          uint8_t shortfilter,
                                          uint8_t shortdelay) noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
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
    return (comm_.GetMode() == CommMode::UART) ? uart_node_address_
                                               : daisy_chain_position_;
  }

  /**
   * @brief Reset the TMC51x0 driver
   * @return Result<void> indicating success or error
   *
   * Performs a software reset by writing to the GSTAT register.
   */
  Result<void> Reset() noexcept;

  /**
   * @brief Check if the driver is initialized
   * @return true if initialized, false otherwise
   */
  [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

private:
  CommType &comm_; ///< Communication interface reference
  uint32_t f_clk_{ClockFreq::DEFAULT_F_CLK}; ///< TMC51x0 clock frequency in Hz
  uint8_t daisy_chain_position_; ///< Position in daisy chain (0 = first
                                 ///< chip/single chip)
  uint8_t uart_node_address_;    ///< UART node address (0-254) for multi-node
                                 ///< addressing
  uint8_t send_delay_{
      0}; ///< UART send delay (0-15) stored locally from SLAVECONF register
  bool initialized_{false};                    ///< Initialization status flag
  uint8_t chip_version_{ChipVersion::TMC5160}; ///< Detected chip version (0x11
                                               ///< = TMC5130, 0x30 = TMC5160)

  // Physical configuration for unit conversions
  MotorSpec motor_spec_;
  MechanicalSystem mechanical_system_;

  // Driver configuration (updated on all runtime changes)
  // Represents the current runtime state, updated whenever any Configure*
  // method is called
  DriverConfig driver_config_;

  // Calculated motor current settings (stored internally, not in MotorSpec)
  uint16_t calculated_global_scaler_{0};
  uint8_t calculated_irun_{0};
  uint8_t calculated_ihold_{0};
  uint16_t current_microsteps_{256};

  // Write-only register tracking (for easy access to current values)
  // These registers cannot be read back, so we track them locally
  struct WriteOnlyRegisters {
    uint32_t x_compare{0};     ///< X_COMPARE (0x05) - Position comparison
    uint32_t short_conf{0};    ///< SHORT_CONF (0x09) - Short protection config
    uint32_t drv_conf{0};      ///< DRV_CONF (0x0A) - Driver configuration
    uint16_t global_scaler{0}; ///< GLOBAL_SCALER (0x0B) - Current scaling
    uint32_t ihold_irun{0};    ///< IHOLD_IRUN (0x10) - Current control
    uint8_t tpowerdown{0};     ///< TPOWERDOWN (0x11) - Power down delay
    uint32_t tpwmthrs{0};      ///< TPWMTHRS (0x13) - StealthChop threshold
    uint32_t tcoolthrs{0};     ///< TCOOLTHRS (0x14) - CoolStep threshold
    uint32_t thigh{0};         ///< THIGH (0x15) - High speed threshold
    uint32_t vstart{0};        ///< VSTART (0x23) - Start velocity
    uint32_t a_1{0};           ///< A_1 (0x24) - First acceleration
    uint32_t v_1{0};           ///< V_1 (0x25) - Transition velocity
    uint32_t amax{0};          ///< AMAX (0x26) - Max acceleration
    uint32_t vmax{0};          ///< VMAX (0x27) - Max velocity
    uint32_t dmax{0};          ///< DMAX (0x28) - Max deceleration
    uint32_t d_1{0};           ///< D_1 (0x2A) - First deceleration
    uint32_t vstop{0};         ///< VSTOP (0x2B) - Stop velocity
    uint32_t tzerowait{0};     ///< TZEROWAIT (0x2C) - Zero wait time
    uint32_t vdcmin{0};        ///< VDCMIN (0x33) - DcStep threshold
    uint32_t enc_const{0};     ///< ENC_CONST (0x3A) - Encoder constant
    uint32_t enc_deviation{0}; ///< ENC_DEVIATION (0x3D) - Encoder deviation
    uint32_t coolconf{0};      ///< COOLCONF (0x6D) - CoolStep config
    uint32_t dcctrl{0};        ///< DCCTRL (0x6E) - DcStep config
    uint32_t pwmconf{0};       ///< PWMCONF (0x70) - StealthChop config
    uint32_t slaveconf{0};     ///< SLAVECONF (0x03) - UART node address config
  } write_only_regs_;

  /**
   * @brief Convert speed value to internal steps/s
   * @param value Speed value in specified unit
   * @param unit Unit of the value
   * @return Speed in steps/s
   */
  [[nodiscard]] float convertSpeedToSteps(float value,
                                          Unit unit) const noexcept;

  /**
   * @brief Convert acceleration value to internal steps/s²
   * @param value Acceleration value in specified unit
   * @param unit Unit of the value
   * @return Acceleration in steps/s²
   */
  [[nodiscard]] float convertAccelerationToSteps(float value,
                                                 Unit unit) const noexcept;

  /**
   * @brief Convert position value to steps
   * @param value Position value in specified unit
   * @param unit Unit of the value
   * @return Position in steps
   */
  [[nodiscard]] float convertPositionToSteps(float value,
                                             Unit unit) const noexcept;

  /**
   * @brief Convert steps to specified unit (for position)
   * @param steps Position in steps
   * @param unit Target unit
   * @return Position in target unit
   */
  [[nodiscard]] float convertStepsToUnit(int32_t steps,
                                         Unit unit) const noexcept;

  /**
   * @brief Convert speed (steps/s) to specified unit
   * @param steps_per_sec Speed in steps/s
   * @param unit Target unit
   * @return Speed in target unit
   */
  [[nodiscard]] float convertSpeedToUnit(float steps_per_sec,
                                         Unit unit) const noexcept;

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
   * @brief Convert acceleration from internal TMC5160 units to steps/s²
   * @param accel_internal Acceleration in internal TMC5160 units
   * @return Acceleration in steps per second squared
   */
  [[nodiscard]] float accelFromInternal(int32_t accel_internal) const noexcept;

  /**
   * @brief Convert threshold speed to TSTEP format
   * @param speed_hz Speed threshold in steps per second
   * @return TSTEP value (0 if speed is 0)
   */
  [[nodiscard]] int32_t thresholdSpeedToTstep(float speed_hz) const noexcept;
};

} // namespace tmc51x0

// Include template implementation
#ifndef TMC51X0_COMPILING_SRC
#define TMC51X0_HEADER_INCLUDED
// NOLINTNEXTLINE(bugprone-suspicious-include) - Intentional: template
// implementation file
#include "../src/tmc51x0.cpp"
#undef TMC51X0_HEADER_INCLUDED
#endif

#endif // TMC51X0_HPP
