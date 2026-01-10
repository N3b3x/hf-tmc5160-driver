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
 * ## Operating Modes (SD_MODE) and API Validity
 *
 * The TMC51x0 family supports **two fundamentally different motion sources**:
 *
 * - **Internal ramp generator / motion controller** (**SD_MODE = 0**):
 *   The chip executes motion profiles from registers (XTARGET/VMAX/AMAX/…).
 * - **External Step/Dir** (**SD_MODE = 1**):
 *   Motion comes from STEP/DIR pins. The internal ramp generator is not the
 *   active motion source.
 *
 * This driver supports both, but **not every API makes sense in every mode**.
 * To prevent confusing behavior, motion-controller operations require
 * SD_MODE=0 and will return **ErrorCode::INVALID_STATE** if the chip reports
 * SD_MODE=1 (external Step/Dir) via IOIN.
 *
 * **Quick mode matrix**
 *
 * | Subsystem / Feature | SD_MODE=0 (Internal ramp) | SD_MODE=1 (External Step/Dir) |
 * |---------------------|---------------------------|--------------------------------|
 * | `rampControl` motion/profile setters (XTARGET/VMAX/AMAX/...) | ✅ | ❌ (INVALID_STATE) |
 * | `homing` (sensorless/switch homing routines) | ✅ | ❌ (INVALID_STATE) |
 * | `tuning` (StallGuard tuning routines) | ✅ | ❌ (INVALID_STATE) |
 * | `motorControl` (chopper, stealthChop, currents, LUT) | ✅ | ✅ (feature-dependent) |
 * | `thresholds` (TPWMTHRS/TCOOLTHRS/THIGH/VDCMIN) | ✅ | ✅ (feature-dependent) |
 * | `powerStage` (DRV_CONF/SHORT_CONF) | ✅ | ✅ |
 * | `encoder` (ABN encoder config/read) | ✅ | ❌ (INVALID_STATE) |
 * | `status` (GSTAT/DRV_STATUS/RAMP_STAT/IOIN/etc.) | ✅ | ✅ |
 * | `io` (SPI_MODE/SD_MODE pin helpers, IOIN/OUTPUT helpers) | ✅ | ✅ |
 *
 * Notes:
 * - In SD_MODE=1, you typically still configure chopper/StealthChop/CoolStep/
 *   StallGuard thresholds, but you do **not** command motion via XTARGET/VMAX.
 * - Some features (e.g., StallGuard/CoolStep) have additional datasheet
 *   requirements (SpreadCycle vs StealthChop, velocity thresholds, etc.).
 *
 * ## Usage Example
 *
 * @code
 * // Create communication interface (SPI example)
 * class MySPI : public tmc51x0::SpiCommInterface<MySPI> {
 *   // ... implement required methods
 * };
 *
 * MySPI spi_comm;
 *
 * // Create TMC51x0 driver with template parameter
 * tmc51x0::TMC51x0<MySPI> driver(spi_comm);
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
   * @brief Options for HardReset()
   */
  struct HardResetOptions {
    uint32_t power_off_ms{20};         ///< Power-off duration for power-cycle
    uint32_t power_on_settle_ms{20};   ///< Post power-on settle time before IO
    bool reinitialize{true};           ///< Re-run Initialize() after reset
    bool prefer_power_cycle{true};     ///< Use comm_.PowerCycle() when supported
    bool uart_assume_accessible_at_0{true}; ///< After power-cycle, assume the
                                            ///< device is reachable at UART
                                            ///< address 0 for re-init
  };

  /**
   * @brief Perform a hard reset (power-cycle when available, else software reset)
   * @param opts Hard reset options
   * @return Result<void> indicating success or error
   *
   * Behavior:
   * - If the platform implements CommInterface::PowerCycle(), this will power-cycle
   *   the device (true hard reset / POR semantics).
   * - Otherwise, it falls back to Reset() (software reset via GSTAT).
   * - If opts.reinitialize=true, Initialize(GetDriverConfig()) is called after reset.
   *
   * UART multi-node note:
   * - A power-cycle resets NODECONF in the chip. To re-initialize reliably, the
   *   device must be reachable at UART address 0 (typically the first chip with
   *   NAI tied to GND). For multi-node chains, prefer using TMC51x0MultiNode and
   *   ProgramSequentially() after power-up.
   */
  Result<void> HardReset(const HardResetOptions &opts = HardResetOptions()) noexcept;

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

  // NOTE: Heap-heavy string diagnostics are intentionally only available when
  // debug logging is enabled. For embedded targets, prefer LogConfigSummary()
  // and friends (which can be compiled out).
#ifndef TMC51X0_DISABLE_DEBUG_LOGGING
  /**
   * @brief Get driver configuration as formatted string (heap allocating)
   * @return String containing complete driver configuration and status
   * information
   *
   * Returns a human-readable string with all driver configuration, register
   * values, and current status. Useful for debugging and diagnostics.
   *
   * @note The returned string is allocated on the heap (std::string / std::to_string).
   * @note For embedded systems, prefer LogConfigSummary() / LogDerivedInitSummary()
   *       or implement a fixed-size buffer variant.
   */
  [[nodiscard]] std::string GetDriverConfigString() const noexcept;
#endif // TMC51X0_DISABLE_DEBUG_LOGGING

#ifndef TMC51X0_DISABLE_DEBUG_LOGGING
  /**
   * @brief Log a comprehensive, human-readable summary of a DriverConfig
   *
   * Uses the driver's communication interface for logging (TMC51X0_LOG_DEBUG).
   *
   * This is useful both before Initialize() (to verify the config you are about to apply)
   * and after Initialize() (to compare with derived/cached values).
   *
   * @note This method is only available when TMC51X0_DISABLE_DEBUG_LOGGING is not defined.
   *       When disabled, calls to this method are optimized out at compile time.
   */
  void LogConfigSummary(const DriverConfig &cfg, const char *tag = "TMC5160",
                        LogLevel lvl = LogLevel::Info) noexcept;

  /**
   * @brief Log a compact, table-style summary of derived/cached initialization values
   *
   * Includes f_clk, detected chip version, calculated currents (IRUN/IHOLD/GLOBAL_SCALER),
   * cached write-only registers (IHOLD_IRUN, GLOBAL_SCALER, DRV_CONF), and a decoded
   * IHOLDDELAY timing estimate.
   *
   * @note Requires that the driver has been initialized at least once (uses cached values).
   * @note This method is only available when TMC51X0_DISABLE_DEBUG_LOGGING is not defined.
   *       When disabled, calls to this method are optimized out at compile time.
   */
  void LogDerivedInitSummary(const char *tag = "TMC5160",
                             LogLevel lvl = LogLevel::Info) noexcept;

  /**
   * @brief Read silicon registers and log a comprehensive live status report
   *
   * Performs SPI/UART transactions to capture the current hardware state of the
   * chip (GSTAT, DRV_STATUS, IOIN, CHOPCONF, PWMCONF, etc.) and prints it in
   * a beautiful table format.
   *
   * This is useful for "recapturing" the actual live state from the driver
   * to verify it matches requested settings or to diagnose issues.
   *
   * @note This method is only available when TMC51X0_DISABLE_DEBUG_LOGGING is not defined.
   *       When disabled, calls to this method are optimized out at compile time.
   */
  void LogLiveStatusReport(const char *tag = "TMC5160",
                           LogLevel lvl = LogLevel::Info) noexcept;
#endif // TMC51X0_DISABLE_DEBUG_LOGGING

  /**
   * @brief Snapshot of motor-current related calculated/cached values.
   *
   * @details
   * The TMC5160 has several write-only registers (notably `GLOBAL_SCALER` and
   * `IHOLD_IRUN`). This driver maintains an internal cache of those values and
   * also stores the most recent calculated values from motor-current
   * configuration (IRUN/IHOLD/GLOBAL_SCALER).
   *
   * This struct exposes those values for application-level logging without
   * duplicating the calculation logic in the application.
   */
  struct MotorCurrentDebugInfo {
    bool initialized{false};          ///< True if Initialize() has completed successfully
    uint32_t f_clk_hz{0};             ///< Effective clock used for calculations
    MotorSpec motor_spec{};           ///< MotorSpec used for calculations
    uint8_t calculated_irun{0};       ///< Last calculated IRUN (0..31)
    uint8_t calculated_ihold{0};      ///< Last calculated IHOLD (0..31)
    uint16_t calculated_global_scaler{0}; ///< Last calculated GLOBAL_SCALER (32..256)
    uint16_t cached_global_scaler{0}; ///< Cached write-only GLOBAL_SCALER value
    uint32_t cached_ihold_irun{0};    ///< Cached write-only IHOLD_IRUN raw register value
  };

  /**
   * @brief Get motor-current calculated/cached values for application logging.
   *
   * @return A snapshot of motor-current related values.
   *
   * @note If the driver has not been initialized yet, values may be defaults.
   */
  [[nodiscard]] MotorCurrentDebugInfo GetMotorCurrentDebugInfo() const noexcept;

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
     * @brief Get current position in raw driver microsteps (XACTUAL)
     * @return Result<int32_t> containing XACTUAL in **microsteps** (native register unit)
     *
     * Use this when you want to inspect the exact microstep-level position used by the ramp generator.
     * This avoids any conversion/rounding to degrees/full-steps.
     */
    Result<int32_t> GetCurrentPositionMicrosteps() noexcept;

    /**
     * @brief Get target position in raw driver microsteps (XTARGET)
     * @return Result<int32_t> containing XTARGET in **microsteps** (native register unit)
     */
    Result<int32_t> GetTargetPositionMicrosteps() noexcept;

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
  //                                    SWITCHES STRUCT
  //================================================================================
  //================================================================================
  /**
   * @brief Reference switches / endstops subsystem (SW_MODE / XLATCH)
   * @ingroup TMC51X0_Subsystems
   *
   * Owns reference switch configuration, latching, and related status helpers.
   * (This used to live under RampControl; it is separated to keep RampControl
   * focused on motion profile programming.)
   */
  struct Switches {
    explicit Switches(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure reference switches / endstops (SW_MODE)
     * @param config Reference switch configuration
     * @return Result<void> indicating success or error
     *
     * Programs the reference switch logic in the SW_MODE register, including:
     * - input polarity / active level (left/right)
     * - enable/disable stop-on-switch per side
     * - latch configuration (XLATCH behavior)
     * - hard/soft stop behavior for the internal motion controller
     *
     * Use `GetReferenceSwitchStatus()` to read the current active state and
     * whether stop-on-switch is enabled.
     */
    Result<void> ConfigureReferenceSwitch(const ReferenceSwitchConfig &config) noexcept;

    /**
     * @brief Read reference switch configuration (SW_MODE)
     * @return Result<ReferenceSwitchConfig> containing the value or error
     */
    Result<ReferenceSwitchConfig> GetReferenceSwitchConfig() noexcept;

    /**
     * @brief Set left reference switch active level / polarity
     * @param active_level Active level (ACTIVE_LOW / ACTIVE_HIGH)
     * @return Result<void> indicating success or error
     *
     * Convenience helper that reads the current SW_MODE, updates the left
     * polarity field, and writes it back.
     */
    Result<void> SetLeftSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Set right reference switch active level / polarity
     * @param active_level Active level (ACTIVE_LOW / ACTIVE_HIGH)
     * @return Result<void> indicating success or error
     *
     * Convenience helper that reads the current SW_MODE, updates the right
     * polarity field, and writes it back.
     */
    Result<void> SetRightSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept;

    /**
     * @brief Enable/disable stop-on-left-switch (SW_MODE.stop_l_enable)
     * @param enable True to enable stop-on-switch
     * @return Result<void> indicating success or error
     */
    Result<void> SetLeftSwitchStopEnable(bool enable) noexcept;

    /**
     * @brief Enable/disable stop-on-right-switch (SW_MODE.stop_r_enable)
     * @param enable True to enable stop-on-switch
     * @return Result<void> indicating success or error
     */
    Result<void> SetRightSwitchStopEnable(bool enable) noexcept;

    /**
     * @brief Configure left switch latching behavior (SW_MODE.latch_l_active)
     * @param latch_mode Latch mode selection
     * @return Result<void> indicating success or error
     */
    Result<void> SetLeftSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept;

    /**
     * @brief Configure right switch latching behavior (SW_MODE.latch_r_active)
     * @param latch_mode Latch mode selection
     * @return Result<void> indicating success or error
     */
    Result<void> SetRightSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept;

    /**
     * @brief Set reference stop mode (hard/soft stop) for internal ramp generator
     * @param stop_mode Stop mode selection
     * @return Result<void> indicating success or error
     *
     * @warning Datasheet (SW_MODE): do not use soft stop in combination with
     *          StallGuard stop-on-stall (sg_stop). This API returns INVALID_STATE
     *          if soft-stop is requested while sg_stop is enabled.
     */
    Result<void> SetStopMode(ReferenceStopMode stop_mode) noexcept;

    /**
     * @brief Get reference switch active/enabled status (RAMP_STAT + SW_MODE)
     * @param right_active Output: true if right switch is currently active
     * @param left_enabled Output: true if left stop-on-switch is enabled
     * @param right_enabled Output: true if right stop-on-switch is enabled
     * @return Result<bool> true on success, or error
     *
     * - Active state is read from RAMP_STAT status bits.
     * - Enabled state is derived from SW_MODE stop-enable bits.
     */
    Result<bool> GetReferenceSwitchStatus(bool &right_active,
                                          bool &left_enabled,
                                          bool &right_enabled) noexcept;

    /**
     * @brief Read latched position from XLATCH (unit-aware)
     * @param unit Unit to return the position in
     * @return Result<float> containing the latched position or error
     *
     * XLATCH latches the internal position counter (microsteps). This helper
     * converts it into the requested user unit based on the current microstep
     * resolution and mechanical system configuration.
     */
    Result<float> GetLatchedPosition(Unit unit) noexcept;

  private:
    TMC51x0 &driver_;
  } switches{*this};

  //================================================================================
  //================================================================================
  //                                    EVENTS STRUCT
  //================================================================================
  //================================================================================
  /**
   * @brief Motion events / status outputs (X_COMPARE, RAMP_STAT clear)
   * @ingroup TMC51X0_Subsystems
   */
  struct Events {
    explicit Events(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Program the X_COMPARE position compare threshold (unit-aware)
     * @param position Compare position
     * @param unit Unit of the compare position
     * @return Result<void> indicating success or error
     *
     * Note: X_COMPARE is a **write-only** register. The driver caches the last
     * written value; use `GetXCompare()` to retrieve that cached value.
     */
    Result<void> SetXCompare(float position, Unit unit) noexcept;

    /**
     * @brief Get last programmed X_COMPARE threshold (cached, unit-aware)
     * @param unit Unit to return the cached compare value in
     * @return Result<float> containing the cached value
     *
     * This does not read hardware (X_COMPARE is write-only); it returns the
     * last value written by `SetXCompare()` (or other APIs writing X_COMPARE).
     */
    Result<float> GetXCompare(Unit unit) const noexcept;

    /**
     * @brief Clear specific bits in RAMP_STAT
     * @param bits_to_clear Bitmask of RAMP_STAT bits to clear (write-1-to-clear)
     * @return Result<void> indicating success or error
     */
    Result<void> ClearRampStatus(uint32_t bits_to_clear) noexcept;

  private:
    TMC51x0 &driver_;
  } events{*this};

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
     * @brief Configure microstep lookup table with a preset waveform
     * @param preset Waveform preset (DEFAULT_SINE, PURE_SINE, etc.)
     * @return Result<void> indicating success or error
     *
     * Programs all MSLUT registers (MSLUT0-7, MSLUTSEL, MSLUTSTART) with
     * optimized values for the selected waveform type.
     *
     * Available presets:
     * - DEFAULT_SINE: TMC5160 power-on default (slightly modified sine)
     * - PURE_SINE: Mathematical pure sine wave (smooth, standard torque)
     *
     * @note The lookup table defines the first quarter of the sine wave;
     *       the driver mirrors it for the remaining three quarters.
     *
     * @see Datasheet Section 18: Microstep Lookup Table
     */
    Result<void> ConfigureMicrostepLutPreset(MicrostepLutPreset preset) noexcept;

    /**
     * @brief Configure microstep lookup table from raw data
     * @param lut Array of 8 uint32_t values for MSLUT0-7
     * @param w0 Width select for segment 0 (0-3)
     * @param w1 Width select for segment 1 (0-3)
     * @param w2 Width select for segment 2 (0-3)
     * @param w3 Width select for segment 3 (0-3)
     * @param x1 Segment 1 start (0-255)
     * @param x2 Segment 2 start (0-255)
     * @param x3 Segment 3 start (0-255)
     * @param start_sin Start sine value at entry 0 (0-255)
     * @param start_sin90 Sine value at entry 256 / 90° (0-255)
     * @return Result<void> indicating success or error
     *
     * Programs all MSLUT registers with custom values for advanced waveform
     * shaping. Use this for custom motor characteristics or torque ripple
     * compensation.
     *
     * @see Datasheet Section 18: Microstep Lookup Table
     */
    Result<void> ConfigureMicrostepLutCustom(
        const uint32_t lut[8], uint8_t w0, uint8_t w1, uint8_t w2, uint8_t w3,
        uint8_t x1, uint8_t x2, uint8_t x3, uint8_t start_sin,
        uint8_t start_sin90) noexcept;

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
  //                                    THRESHOLDS STRUCT
  //================================================================================
  //================================================================================
  /**
   * @brief Velocity thresholds / mode thresholds (TPWMTHRS, TCOOLTHRS, THIGH)
   * @ingroup TMC51X0_Subsystems
   *
   * Consolidates all "threshold" knobs so users don't have to guess whether a
   * given threshold belongs to motor control, StallGuard, or diagnostics.
   */
  struct Thresholds {
    explicit Thresholds(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Set StealthChop velocity threshold (TPWMTHRS)
     * @param value Velocity threshold value
     * @param unit Unit of the value
     * @return Result<void> indicating success or error
     */
    Result<void> SetStealthChopVelocityThreshold(float value, Unit unit) noexcept;

    /**
     * @brief Get StealthChop velocity threshold (TPWMTHRS) from local tracking
     * @param unit Unit of the returned threshold
     * @return Result<float> containing the threshold in requested units
     */
    Result<float> GetStealthChopVelocityThreshold(Unit unit) const noexcept;

    /**
     * @brief Set StallGuard/CoolStep threshold velocity (TCOOLTHRS)
     * @param threshold Threshold velocity
     * @param unit Unit of threshold
     * @return Result<void> indicating success or error
     */
    Result<void> SetTcoolthrs(float threshold, Unit unit) noexcept;

    /**
     * @brief Get cached StallGuard/CoolStep threshold velocity (TCOOLTHRS)
     * @param unit Unit to return the cached value in
     * @return Result<float> containing the cached value or error
     */
    Result<float> GetTcoolthrs(Unit unit) const noexcept;

    /**
     * @brief Set High-Speed velocity threshold (THIGH)
     * @param value Velocity threshold value
     * @param unit Unit of the value
     * @return Result<void> indicating success or error
     */
    Result<void> SetHighSpeedThreshold(float value, Unit unit) noexcept;

    /**
     * @brief Set DcStep velocity threshold (VDCMIN)
     * @param value Velocity threshold value (0 disables)
     * @param unit Unit of the value
     * @return Result<void> indicating success or error
     *
     * VDCMIN is part of the ramp-generator driver feature control register set.
     * Per datasheet, VDCMIN-based DcStep enable is only valid when using the
     * **internal ramp generator** (SD_MODE=LOW). In external STEP/DIR mode,
     * DcStep is enabled via the external DCEN pin instead.
     *
     * @retval INVALID_STATE if called while in external STEP/DIR mode
     */
    Result<void> SetDcStepVelocityThreshold(float value, Unit unit) noexcept;

    /**
     * @brief Get cached DcStep velocity threshold (VDCMIN)
     * @param unit Unit to return the cached value in
     * @return Result<float> containing the cached value or error
     *
     * This returns the locally tracked value (VDCMIN is treated as write-only by
     * this driver).
     */
    Result<float> GetDcStepVelocityThreshold(Unit unit) const noexcept;

    /**
     * @brief Get locally tracked VDCMIN register value (raw)
     * @return Raw VDCMIN register value
     */
    uint32_t GetVdcminRegisterValue() const noexcept;

    /**
     * @brief Convenience: set TPWMTHRS, TCOOLTHRS, and THIGH in one call
     * @param pwm_thrs Speed threshold for StealthChop (TPWMTHRS)
     * @param cool_thrs Speed threshold for StallGuard/CoolStep (TCOOLTHRS)
     * @param high_thrs Speed threshold for high-speed mode (THIGH)
     * @param unit Unit of the speed values
     * @return Result<void> indicating success or error
     */
    Result<void> SetModeChangeSpeeds(float pwm_thrs, float cool_thrs,
                                     float high_thrs, Unit unit) noexcept;

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
     * @brief Get locally tracked THIGH register value (raw)
     * @return Raw THIGH register value (20-bit meaningful range)
     * @note THIGH defines the upper velocity limit for StallGuard2/CoolStep.
     *       Above this velocity, SG and CoolStep are disabled.
     */
    uint32_t GetThighRegisterValue() const noexcept;

    //==========================================================================
    // StealthChop/SpreadCycle Transition Hysteresis
    //==========================================================================

    /**
     * @brief Configure StealthChop to SpreadCycle transition with hysteresis
     * @param lower_speed Speed below which StealthChop activates (lower threshold)
     * @param upper_speed Speed above which SpreadCycle activates (upper threshold)
     * @param unit Speed unit for both thresholds
     * @return Result<void> indicating success or error
     *
     * Creates a velocity hysteresis band to prevent mode oscillation near the
     * threshold. The motor will:
     * - Switch to SpreadCycle when accelerating above `upper_speed`
     * - Switch to StealthChop when decelerating below `lower_speed`
     *
     * Implementation note: The TMC5160 has only one TPWMTHRS register. This
     * method dynamically updates TPWMTHRS based on current chopper mode to
     * create the hysteresis effect. Call `UpdateModeHysteresis()` periodically
     * or after velocity changes for best results.
     *
     * Example: SetModeHysteresis(50, 70, Unit::RPM) creates a 20 RPM dead band:
     * - StealthChop → SpreadCycle at 70 RPM (accelerating)
     * - SpreadCycle → StealthChop at 50 RPM (decelerating)
     *
     * @note Requires GCONF.en_pwm_mode = 1 (StealthChop enabled globally)
     *
     * @see Datasheet section 9.2: Switching between SpreadCycle and StealthChop
     */
    Result<void> SetModeHysteresis(float lower_speed, float upper_speed,
                                   Unit unit) noexcept;

    /**
     * @brief Update TPWMTHRS based on current chopper mode for hysteresis
     * @return Result<void> indicating success or error
     *
     * Call this method periodically to maintain hysteresis behavior. It reads
     * the current chopper mode (DRV_STATUS.stealth) and updates TPWMTHRS to
     * the appropriate threshold:
     * - If in StealthChop: set TPWMTHRS to upper threshold (switch at higher speed)
     * - If in SpreadCycle: set TPWMTHRS to lower threshold (switch at lower speed)
     *
     * @note Only effective after calling SetModeHysteresis() to configure the
     *       upper and lower thresholds.
     */
    Result<void> UpdateModeHysteresis() noexcept;

    /**
     * @brief Disable mode hysteresis (use single TPWMTHRS threshold)
     * @return Result<void> indicating success or error
     *
     * Clears the hysteresis configuration and returns to standard single-threshold
     * mode switching behavior.
     */
    Result<void> DisableModeHysteresis() noexcept;

    /**
     * @brief Check if mode hysteresis is currently configured
     * @return true if hysteresis is enabled with valid upper/lower thresholds
     */
    bool IsModeHysteresisEnabled() const noexcept;

  private:
    TMC51x0 &driver_;
    
    // Hysteresis state tracking (internal units: TSTEP register values)
    // Note: TSTEP is inverse of velocity (lower TSTEP = higher speed)
    uint32_t hysteresis_to_spreadcycle_tstep_{0};  // Switch to SpreadCycle when TSTEP < this (higher velocity)
    uint32_t hysteresis_to_stealthchop_tstep_{0};  // Switch to StealthChop when TSTEP > this (lower velocity)
    bool hysteresis_enabled_{false};
  } thresholds{*this};

  //================================================================================
  //================================================================================
  //                                    POWER STAGE STRUCT
  //================================================================================
  //================================================================================
  /**
   * @brief Power stage + protection subsystem (DRV_CONF, SHORT_CONF)
   * @ingroup TMC51X0_Subsystems
   *
   * Groups board/power-electronics configuration and short protection, keeping
   * `motorControl` focused on motor drive behavior.
   */
  struct PowerStage {
    explicit PowerStage(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure power stage parameters (DRV_CONF register)
     * @param config Power stage parameters structure
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigurePowerStage(const PowerStageParameters &config) noexcept;

    /**
     * @brief Configure short protection levels from PowerStageParameters
     * @param config Power stage parameters structure (contains short protection fields)
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureShortProtection(const PowerStageParameters &config) noexcept;

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
    TMC51x0 &driver_;
  } powerStage{*this};

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
     * @brief Configure UART node address and send delay (writes NODECONF
     * register)
     * @param node_address UART node address (0-254), same as NODECONF.NODEADDR
     * NODECONF
     * @param send_delay Number of bit times before replying to register read
     * (0-15), stored locally
     * @return Result<void> indicating success or error
     *
     * Writes the NODECONF register to configure the chip's UART node address
     * and send delay. Also updates the local software representation
     * (uart_node_address_ and send_delay_).
     *
     * @note This writes to hardware. For sequential programming, use
     * `uartConfig.ConfigureUartNodeAddress()` instead.
     * @note Send delay is stored locally since NODECONF register is
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
     * @note The node address must be programmed into the chip via NODECONF
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

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } communication{*this};

  //================================================================================
  //================================================================================
  //                                    IO / PINS STRUCT
  //================================================================================
  //================================================================================
  /**
   * @brief Chip IO / mode pins / IOIN helpers
   * @ingroup TMC51X0_Subsystems
   *
   * Groups "pin-state / IOIN / mode pins" operations so they don't get mixed
   * into status monitoring or communication addressing.
   */
  struct Io {
    explicit Io(TMC51x0 &driver) noexcept : driver_(driver) {}

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
     * @brief Read the current mode-pin state (SPI_MODE/SD_MODE)
     * @return Result<ChipCommMode> containing the value or error
     *
     * This method reads the current state of SPI_MODE (pin 22) and SD_MODE (pin
     * 21) pins if they are connected to GPIO inputs/outputs.
     *
     * @note This reads the current pin state, which may not reflect the actual
     * chip mode if the chip hasn't been reset since the pins were changed.
     */
    Result<ChipCommMode> GetOperatingMode() const noexcept;

    /**
     * @brief Read GPIO input pins (parsed) from IOIN
     * @return Result<InputStatus> containing the value or error
     */
    Result<InputStatus> ReadInputStatus() noexcept;

    /**
     * @brief Read IC version from IOIN
     * @return Result<uint8_t> containing the value or error
     */
    Result<uint8_t> ReadIcVersion() noexcept;

    /**
     * @brief Read GPIO input pins (raw) from IOIN
     * @return Result<uint32_t> containing the value or error
     */
    Result<uint32_t> ReadGpioPins() noexcept;

    /**
     * @brief Set SDO_CFG0 pin polarity (UART/Single Wire mode)
     * @param polarity Output pin polarity (false=normal/active high,
     * true=inverted/active low)
     * @return Result<void> indicating success or error
     */
    Result<void> SetSdoCfg0Polarity(bool polarity) noexcept;

  private:
    TMC51x0 &driver_;
  } io{*this};

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
     *
     * @note Encoder registers are only functional when the chip is using the
     *       **internal ramp generator** (SD_MODE=LOW). In external STEP/DIR
     *       mode the encoder pins are repurposed (DCEN/DCIN/DCO) and encoder
     *       functionality is not available.
     *
     * @retval INVALID_STATE if called while in external STEP/DIR mode
     */
    Result<void> Configure(const EncoderConfig &config) noexcept;

    /**
     * @brief Get current encoder configuration
     * @param config Reference to store current configuration
     * @return Result<EncoderConfig> containing the value or error
     *
     * @retval INVALID_STATE if called while in external STEP/DIR mode
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
     * @brief Enable/disable latching XACTUAL with encoder position
     * @param enable true to latch XACTUAL together with X_ENC on N-event
     * @return Result<void> indicating success or error
     *
     * When enabled, the motor position (XACTUAL) is latched into X_LATCH
     * simultaneously with the encoder position on N-channel events.
     * This is useful for precise position verification and homing.
     *
     * Updates only latch_x_act, preserving other ENCMODE settings.
     *
     * @see Datasheet Section 20: Encoder Interface, ENCMODE.latch_x_act
     */
    Result<void> SetLatchXactualEnabled(bool enable) noexcept;

    /**
     * @brief Check if XACTUAL latching is enabled
     * @return Result<bool> true if latch_x_act is enabled, false otherwise
     */
    Result<bool> IsLatchXactualEnabled() noexcept;

    /**
     * @brief Get encoder position
     * @param position Reference to store encoder position in steps
     * @return Result<int32_t> containing the value or error
     *
     * @retval INVALID_STATE if called while in external STEP/DIR mode
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
     * @brief Check if an encoder N-event was detected (ENC_STATUS.n_event)
     * @return Result<bool> true if an N-event is latched, false otherwise
     *
     * ENC_STATUS.n_event is a write-1-to-clear flag set by the hardware on an
     * N-channel event (as configured by ENCMODE).
     *
     * @retval INVALID_STATE if called while in external STEP/DIR mode
     */
    Result<bool> IsNEventDetected() noexcept;

    /**
     * @brief Clear the encoder N-event flag (ENC_STATUS.n_event, W1C)
     * @return Result<void> indicating success or error
     *
     * @retval INVALID_STATE if called while in external STEP/DIR mode
     */
    Result<void> ClearNEventFlag() noexcept;

    /**
     * @brief Get encoder latched position
     * @param position Reference to store encoder position latched on N event
     * @return Result<int32_t> containing the value or error
     *
     * Reads the encoder position that was latched on the last N channel event.
     *
     * @retval INVALID_STATE if called while in external STEP/DIR mode
     */
    Result<int32_t> GetLatchedPosition() noexcept;

    //==========================================================================
    // Encoder Deviation Detection
    //==========================================================================

    /**
     * @brief Check if encoder deviation warning is active
     * @return Result<bool> true if deviation between X_ENC and XACTUAL exceeds
     *         the threshold set by SetAllowedDeviation()
     *
     * Deviation detection compares the encoder position (X_ENC) with the
     * internal motor position (XACTUAL). If they differ by more than
     * ENC_DEVIATION steps, a warning is triggered.
     *
     * This is useful for:
     * - Detecting lost steps (motor missed steps due to overload)
     * - Detecting mechanical slippage
     * - Closed-loop position verification
     *
     * @note Set ENC_DEVIATION to 0 to disable deviation detection.
     * @note The warning cannot be cleared while deviation persists.
     *
     * @see SetAllowedDeviation()
     */
    Result<bool> IsDeviationWarning() noexcept;

    /**
     * @brief Clear encoder deviation warning flag
     * @return Result<void> indicating success or error
     *
     * Attempts to clear the deviation warning flag. The flag will only
     * clear if the deviation is no longer present (X_ENC and XACTUAL
     * are within ENC_DEVIATION steps).
     *
     * @note This writes '1' to the deviation_warn bit in ENC_STATUS (W1C).
     */
    Result<void> ClearDeviationWarning() noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } encoder{*this};

  //================================================================================
  //================================================================================
  //                                    STATUS STRUCT
  //================================================================================
  //================================================================================

  /**
   * @brief Status / monitoring subsystem (read-only)
   * @ingroup TMC51X0_Subsystems
   *
   * Provides methods for reading status/telemetry and configuration/OTP info.
   */
  struct Status {
    /**
     * @brief Construct diagnostics subsystem
     * @param driver Reference to parent TMC51x0 driver instance
     */
    explicit Status(TMC51x0 &driver) noexcept : driver_(driver) {}

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
     * @brief Clear global status flags (GSTAT)
     * @param clear_reset Clear the reset flag (IC was reset since last read)
     * @param clear_drv_err Clear the driver error flag (short/overtemp occurred)
     * @param clear_uv_cp Clear the undervoltage charge pump flag
     * @return Result<void> indicating success or error
     *
     * GSTAT is a write-1-to-clear (W1C) register. Writing '1' to a bit clears
     * that flag. Use this after handling a fault to acknowledge it.
     *
     * @note Reading GSTAT also clears the flags, so GetGlobalStatus() implicitly
     *       clears. Use this method when you want selective clearing.
     *
     * @see Datasheet section 4.1: GSTAT Register
     */
    Result<void> ClearGlobalStatus(bool clear_reset = true,
                                   bool clear_drv_err = true,
                                   bool clear_uv_cp = true) noexcept;

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

    //==========================================================================
    // Temperature Status (DRV_STATUS bits 25-26)
    //==========================================================================

    /**
     * @brief Check if overtemperature shutdown is active
     * @return Result<bool> true if chip is in overtemperature shutdown
     *
     * When true, the bridge drivers are disabled due to excessive die temperature.
     * The motor will not be driven until temperature drops below the threshold.
     *
     * @note The overtemperature threshold is set via DRV_CONF.OTSELECT:
     *       0=150°C, 1=143°C, 2=136°C, 3=120°C
     *
     * @see Datasheet section 5.1: Overtemperature Protection
     */
    Result<bool> IsOvertemperature() noexcept;

    /**
     * @brief Check if overtemperature pre-warning is active
     * @return Result<bool> true if chip temperature is approaching shutdown threshold
     *
     * Pre-warning typically activates ~20°C before shutdown. Use this to
     * implement thermal management (reduce current, add cooling, slow motion).
     *
     * @note Can be routed to DIAG0 pin via GCONF.diag0_otpw for hardware interrupt.
     *
     * @see Datasheet section 5.1: Overtemperature Protection
     */
    Result<bool> IsOvertempWarning() noexcept;

    /**
     * @brief Get both temperature status flags at once
     * @param overtemp Reference to store overtemperature shutdown status
     * @param prewarning Reference to store pre-warning status
     * @return Result<void> indicating success or error
     *
     * More efficient than calling IsOvertemperature() and IsOvertempWarning()
     * separately as it only requires one register read.
     */
    Result<void> GetTemperatureStatus(bool &overtemp, bool &prewarning) noexcept;

    //==========================================================================
    // Chopper Mode Status (DRV_STATUS bits 14-15)
    //==========================================================================

    /**
     * @brief Check if StealthChop mode is currently active
     * @return Result<bool> true if motor is being driven in StealthChop mode
     *
     * StealthChop is active when:
     * - GCONF.en_pwm_mode = 1 (StealthChop enabled globally)
     * - Velocity is below TPWMTHRS threshold
     * - Motor is not in standstill with IHOLD=0
     *
     * @see Datasheet section 9: StealthChop
     */
    Result<bool> IsStealthChopActive() noexcept;

    /**
     * @brief Check if fullstep mode is currently active
     * @return Result<bool> true if motor is running in fullstep mode
     *
     * Fullstep becomes active when:
     * - Velocity exceeds THIGH threshold AND CHOPCONF.vhighfs=1
     * - OR dcStep is active
     *
     * In fullstep mode, the motor runs at native full steps (not microstepping).
     *
     * @see Datasheet section 8: High Velocity Operation
     */
    Result<bool> IsFullstepActive() noexcept;

    //==========================================================================
    // Current Status (DRV_STATUS bits 16-20)
    //==========================================================================

    /**
     * @brief Get actual motor current scale (CS_ACTUAL)
     * @return Result<uint8_t> containing current scale (0-31)
     *
     * Returns the actual current scaling being applied to the motor, which
     * may differ from IRUN due to:
     * - CoolStep automatic current reduction (when load is light)
     * - Current ramping (IHOLDDELAY transition between IRUN and IHOLD)
     * - Standstill (showing IHOLD value)
     *
     * To convert to actual current: I = (CS_ACTUAL + 1) / 32 * I_max
     *
     * @see Datasheet section 12: CoolStep Smart Current Control
     */
    Result<uint8_t> GetActualCurrentScale() noexcept;

    //==========================================================================
    // Short Circuit Status (DRV_STATUS bits 12-13, 27-28)
    //==========================================================================

    /**
     * @brief Check for short to supply (VS) condition
     * @param phase_a Reference to store phase A short-to-VS status
     * @param phase_b Reference to store phase B short-to-VS status
     * @return Result<void> indicating success or error
     *
     * Short to VS occurs when a low-side FET output is unexpectedly high.
     * This can indicate:
     * - Damaged motor winding shorting to supply
     * - PCB fault
     * - Damaged low-side FET
     *
     * @note Sensitivity is controlled by SHORT_CONF.S2VS_LEVEL
     * @warning Short detection causes immediate driver shutdown for protection.
     *
     * @see Datasheet section 5.2: Short Protection
     */
    Result<void> IsShortToSupply(bool &phase_a, bool &phase_b) noexcept;

    /**
     * @brief Check for short to ground (GND) condition
     * @param phase_a Reference to store phase A short-to-GND status
     * @param phase_b Reference to store phase B short-to-GND status
     * @return Result<void> indicating success or error
     *
     * Short to GND occurs when a high-side FET output is unexpectedly low.
     * This can indicate:
     * - Damaged motor winding shorting to ground
     * - PCB fault
     * - Damaged high-side FET
     *
     * @note Sensitivity is controlled by SHORT_CONF.S2G_LEVEL
     * @warning Short detection causes immediate driver shutdown for protection.
     *
     * @see Datasheet section 5.2: Short Protection
     */
    Result<void> IsShortToGround(bool &phase_a, bool &phase_b) noexcept;

    /**
     * @brief Get all short circuit status flags at once
     * @param s2vs_a Short to VS on phase A
     * @param s2vs_b Short to VS on phase B
     * @param s2g_a Short to GND on phase A
     * @param s2g_b Short to GND on phase B
     * @return Result<void> indicating success or error
     *
     * More efficient than calling individual short check methods separately.
     */
    Result<void> GetShortCircuitStatus(bool &s2vs_a, bool &s2vs_b,
                                       bool &s2g_a, bool &s2g_b) noexcept;

    /**
     * @brief Get ramp status register value
     * @param status Reference to store the RAMP_STAT register value
     * @return Result<uint32_t> containing the value or error
     */
    Result<uint32_t> GetRampStatusRegister() noexcept;

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
     * @brief Read factory configuration
     * @param fclktrim Reference to store FCLKTRIM value (0-31)
     * @return Result<uint8_t> containing the value or error
     *
     * Reads the factory configuration/clock trim value.
     */
    Result<uint8_t> ReadFactoryConfig() noexcept;

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

    //==========================================================================
    // OTP Programming (One-Time Programmable Memory)
    //==========================================================================

    /**
     * @brief Program a single OTP bit (PERMANENT - USE WITH EXTREME CAUTION!)
     * @param byte_index OTP byte index (0 only for TMC5160)
     * @param bit_index Bit index within the byte (0-7)
     * @param confirm_permanent Must be true to confirm permanent operation
     * @return Result<void> indicating success or error
     *
     * ⚠️ CRITICAL WARNING: OTP memory can only be programmed ONCE. Bits can only
     * be set (0→1), never cleared. This operation is IRREVERSIBLE.
     *
     * OTP bit mapping for byte 0 (TMC5160):
     * - Bits 4..0: OTP_FCLKTRIM - Clock frequency trim (factory calibrated!)
     * - Bit 5: OTP_S2_LEVEL - Short detection level (0: level=6, 1: level=12)
     * - Bit 6: OTP_BBM - Break-before-make (0: BBMCLKS=4, 1: BBMCLKS=2)
     * - Bit 7: OTP_TBL - Blank time (0: TBL=2 ~3µs, 1: TBL=1 ~2µs)
     *
     * @note Requires minimum 10ms programming time per bit (handled internally).
     * @note The function will fail if confirm_permanent is false.
     *
     * @see Datasheet Section 6: One-Time Programmable Memory
     */
    Result<void> ProgramOtpBit(uint8_t byte_index, uint8_t bit_index,
                               bool confirm_permanent) noexcept;

    /**
     * @brief Program OTP FCLKTRIM value (PERMANENT - USE WITH EXTREME CAUTION!)
     * @param fclktrim Clock trim value (0-31, affects internal oscillator)
     * @param confirm_permanent Must be true to confirm permanent operation
     * @return Result<void> indicating success or error
     *
     * ⚠️ CRITICAL WARNING: This permanently changes the internal clock frequency.
     * The factory already programs this for 12MHz. Only use if you need a
     * different clock frequency AND understand the implications.
     *
     * @note This programs bits 4..0 of OTP byte 0. Since OTP bits can only be
     *       set (not cleared), you can only increase the FCLKTRIM value from
     *       the current OTP setting.
     */
    Result<void> ProgramOtpFclktrim(uint8_t fclktrim,
                                    bool confirm_permanent) noexcept;

    /**
     * @brief Program OTP short detection level default (PERMANENT!)
     * @param high_level true for level=12 (less sensitive), false for level=6
     * @param confirm_permanent Must be true to confirm permanent operation
     * @return Result<void> indicating success or error
     *
     * ⚠️ WARNING: This permanently changes the reset default for S2G_LEVEL
     * and S2VS_LEVEL. Once set to high (1), it cannot be changed back.
     *
     * @note Programs bit 5 of OTP byte 0.
     */
    Result<void> ProgramOtpS2Level(bool high_level,
                                   bool confirm_permanent) noexcept;

    /**
     * @brief Program OTP BBM (break-before-make) default (PERMANENT!)
     * @param short_bbm true for BBMCLKS=2 (shorter), false for BBMCLKS=4
     * @param confirm_permanent Must be true to confirm permanent operation
     * @return Result<void> indicating success or error
     *
     * ⚠️ WARNING: This permanently changes the reset default for BBMCLKS.
     * Once set to short (1), it cannot be changed back.
     *
     * @note Programs bit 6 of OTP byte 0.
     */
    Result<void> ProgramOtpBbm(bool short_bbm,
                               bool confirm_permanent) noexcept;

    /**
     * @brief Program OTP TBL (blank time) default (PERMANENT!)
     * @param short_tbl true for TBL=1 (~2µs), false for TBL=2 (~3µs)
     * @param confirm_permanent Must be true to confirm permanent operation
     * @return Result<void> indicating success or error
     *
     * ⚠️ WARNING: This permanently changes the reset default for TBL.
     * Once set to short (1), it cannot be changed back.
     *
     * @note Programs bit 7 of OTP byte 0.
     */
    Result<void> ProgramOtpTbl(bool short_tbl,
                               bool confirm_permanent) noexcept;

  private:
    TMC51x0 &driver_; ///< Reference to parent driver instance
  } status{*this};

  //================================================================================
  //================================================================================
  //                                    STALLGUARD STRUCT
  //================================================================================
  //================================================================================
  /**
   * @brief StallGuard2 subsystem (COOLCONF/DRV_STATUS + SW_MODE interactions)
   * @ingroup TMC51X0_Subsystems
   *
   * Owns StallGuard configuration and control signals that affect motion
   * behavior (stop-on-stall, soft-stop), keeping `diagnostics` focused on
   * monitoring.
   */
  struct StallGuard {
    explicit StallGuard(TMC51x0 &driver) noexcept : driver_(driver) {}

    /**
     * @brief Read StallGuard2 value (SG_RESULT)
     * @return Result<uint16_t> containing the value or error
     */
    Result<uint16_t> GetStallGuard() noexcept;

    /**
     * @brief Read StallGuard2 result from DRV_STATUS (SG_RESULT)
     * @return Result<uint16_t> containing the value or error
     */
    Result<uint16_t> GetStallGuardResult() noexcept;

    /**
     * @brief Configure StallGuard2 parameters (COOLCONF)
     * @param config StallGuard configuration
     * @return Result<void> indicating success or error
     */
    Result<void> ConfigureStallGuard(const StallGuardConfig &config) noexcept;

    /**
     * @brief Enable/disable stop-on-stall behavior
     * @param enable True to enable stop on stall
     * @return Result<void> indicating success or error
     *
     * @warning Datasheet (SW_MODE): do not combine stop-on-stall (sg_stop) with
     *          soft-stop (en_softstop). This API returns INVALID_STATE if
     *          enabling sg_stop while soft-stop is enabled.
     */
    Result<void> EnableStopOnStall(bool enable) noexcept;

    /**
     * @brief Check whether stop-on-stall is enabled
     * @return Result<bool> containing true if enabled, false otherwise
     */
    Result<bool> IsStopOnStallEnabled() noexcept;

    /**
     * @brief Enable/disable soft-stop behavior (instead of hard stop)
     * @param enable True to enable soft stop
     * @return Result<void> indicating success or error
     *
     * @warning Datasheet (SW_MODE): do not combine soft-stop (en_softstop) with
     *          stop-on-stall (sg_stop). This API returns INVALID_STATE if
     *          enabling soft-stop while sg_stop is enabled.
     */
    Result<void> SetSoftStop(bool enable) noexcept;

    /**
     * @brief Check whether soft-stop is enabled
     * @return Result<bool> containing true if enabled, false otherwise
     */
    Result<bool> IsSoftStopEnabled() noexcept;

    /**
     * @brief Clear stall event flag (RAMP_STAT)
     * @return Result<void> indicating success or error
     */
    Result<void> ClearStallFlag() noexcept;

    /**
     * @brief Check if stall has been detected (RAMP_STAT)
     * @return Result<bool> containing true if stall event is set
     */
    Result<bool> IsStallDetected() noexcept;

    /**
     * @brief Set StallGuard/CoolStep threshold velocity (TCOOLTHRS, unit-aware)
     * @param threshold Threshold velocity
     * @param unit Unit of threshold
     * @return Result<void> indicating success or error
     */
  private:
    TMC51x0 &driver_;
  } stallGuard{*this};

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
     * @param min_sgt Minimum SGT to try (default: -20, range: -64 to +63)
     * @param max_sgt Maximum SGT to try (default: +20, range: -64 to +63)
     * @param acceleration Acceleration/deceleration in rev/s² (default: 5.0)
     * @param min_velocity Minimum velocity to verify tuning at (0 = disabled,
     * used to determine SGT range)
     * @param max_velocity Maximum velocity to verify tuning at (0 = disabled,
     * used to determine SGT range)
     * @param velocity_unit Unit for velocity parameters (default: RPM)
     * @param acceleration_unit Unit for acceleration parameter (default:
     * RevPerSec, RPM is not valid for acceleration)
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
                   int8_t min_sgt = -20, int8_t max_sgt = 20,
                   float acceleration = 5.0F, float min_velocity = 0.0F,
                   float max_velocity = 0.0F,
                   Unit velocity_unit = Unit::RPM,
                   Unit acceleration_unit = Unit::RevPerSec) noexcept;

    /**
     * @brief Comprehensive automatic StallGuard tuning with current reduction
     * and optional encoder verification
     * @param target_velocity Target velocity for tuning (most important -
     * optimal SGT is determined here)
     * @param result Reference to store comprehensive tuning results
     * @param min_sgt Minimum SGT to try (default: -20, range: -64 to +63)
     * @param max_sgt Maximum SGT to try (default: +20, range: -64 to +63)
     * @param acceleration Acceleration/deceleration in rev/s² (default: 5.0)
     * @param min_velocity Minimum velocity to verify tuning at (0 = disabled)
     * @param max_velocity Maximum velocity to verify tuning at (0 = disabled)
     * @param velocity_unit Unit for velocity parameters (default: RPM)
     * @param acceleration_unit Unit for acceleration parameter (default:
     * RevPerSec, RPM is not valid for acceleration)
     * @param current_reduction_factor Current reduction factor as percentage
     * (0.0-1.0, where 0.3 = 30% of current motor current). Default: 0.3 (30%)
     * per Duet3D best practices for stall detection. Set to 0 to disable.
     * @return Result<void> indicating success or error
     *
     * This is an enhanced version of TuneStallGuard that implements
     * comprehensive automatic tuning following Trinamic application note AN-002
     * guidelines and industry best practices:
     *
     * **Key Features:**
     * - **Current Reduction**: Reduces motor current for safer tuning and
     * improved StallGuard sensitivity using percentage-based reduction. Current
     * is automatically restored after tuning.
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
     * 2. Apply current reduction if specified (percentage-based) - reduces
     * current for safer tuning and improved StallGuard sensitivity
     * 3. Disable CoolStep (SGMIN=0) to prevent current modulation during tuning
     * 4. Disable StallGuard filter (SFILT=0) for immediate response during
     * calibration
     * 5. Disable stop-on-stall to allow manual observation during tuning
     * 6. Find optimal SGT at target velocity (primary goal)
     * 7. Verify optimal SGT works at min/max velocities (if specified)
     * 8. Restore all saved settings
     *
     * **Current Reduction:**
     * If current_reduction_factor > 0, reduces current to
     * (current_motor_current * current_reduction_factor). Example: 0.3 = 30% of
     * current motor current. The function uses the driver's current calculation
     * functions to determine the appropriate IRUN and GLOBAL_SCALER values. The
     * current is constrained to ensure the motor can still move (minimum
     * IRUN=8 for StealthChop compatibility).
     *
     * @note Target velocity is the most important parameter - optimal SGT is
     * determined here first
     * @note If current_reduction_factor is 0, the motor current is not changed
     * @note Current reduction is applied by recalculating IRUN/GLOBAL_SCALER
     * from the reduced current
     * @note All settings (current, CoolStep, filter) are automatically restored
     * after tuning
     * @note This function takes several seconds to complete (typically 5-30
     * seconds depending on SGT range)
     * @note For best results, ensure the motor is unloaded during tuning
     *
     * @see TuneStallGuard() for a simpler version without current reduction
     * handling
     */
    Result<void>
    AutoTuneStallGuard(float target_velocity, StallGuardTuningResult &result,
                       int8_t min_sgt = -20, int8_t max_sgt = 20,
                       float acceleration = 5.0F, float min_velocity = 0.0F,
                       float max_velocity = 0.0F,
                       Unit velocity_unit = Unit::RPM,
                       Unit acceleration_unit = Unit::RevPerSec,
                       float current_reduction_factor = 0.3F) noexcept;

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

    // Bounds finding helpers
    enum class BoundsMethod { StallGuard, Encoder, Switch };
    enum class HomePlacement { None, AtMin, AtMax, AtCenter, AtOffsetFromMin };

    struct BoundsResult {
      bool success{false};
      bool bounded{false};    ///< true if bounds found on both sides
      bool cancelled{false};
      float min_bound{0.0F};  ///< In caller-provided position_unit
      float max_bound{0.0F};  ///< In caller-provided position_unit
    };

    using CancelCallback = bool (*)();

    struct HomeConfig {
      HomePlacement mode{HomePlacement::AtCenter};
      float offset{0.0F};         ///< Used only for AtOffsetFromMin
      Unit offset_unit{Unit::Deg};
    };

    struct BoundsOptions {
      // Units
      Unit speed_unit{Unit::RPM};
      Unit position_unit{Unit::Deg};
      Unit accel_unit{Unit::RevPerSec}; ///< Accel/decel unit (interpreted as rev/s² when used for acceleration fields)

      // Motion behavior
      float search_speed{0.0F};    ///< In speed_unit
      float search_span{360.0F};   ///< Max distance to attempt per direction, in position_unit
      float backoff_distance{5.0F};///< Backoff after bound hit, in position_unit
      uint32_t timeout_ms{30000};  ///< Timeout per direction
      bool search_positive_first{true}; ///< Bounds finding: when true, search + direction first; when false, search - direction first.
      bool preflight_clear_active_switch{true}; ///< Switch homing: if selected switch is active at start, attempt a bounded move away to clear it.

      // Optional accel/decel override (applies to bounds finding and homing moves that configure positioning ramps).
      // If 0, the routines use the cached driver acceleration/deceleration, or a conservative default.
      float search_accel{0.0F}; ///< Acceleration override in accel_unit (0 = use cached/default)
      float search_decel{0.0F}; ///< Deceleration override in accel_unit (0 = use cached/default)

      // StallGuard-only (ignored for Encoder/Switch)
      float current_reduction_factor{0.3F};   ///< 0..1 (percentage of configured current). Ignored if current_reduction_target_mA > 0.
      uint16_t current_reduction_target_mA{0};///< Absolute RMS current target during bounds (0 = disabled).
      const StallGuardConfig* stallguard_override{nullptr}; ///< Optional override (SGT/min_velocity/filter/etc.). If null, uses configured StallGuard settings.
    };

    /**
     * @brief Perform sensorless homing using StallGuard2 (with settings
     * caching)
     * @param direction Direction to search (true = positive, false = negative)
     * @param opt Homing options (unit-aware). Uses opt.search_speed, opt.search_span, opt.timeout_ms.
     * @param final_position Reference to store final position after homing (in steps).
     *        Note: if `opt.backoff_distance > 0`, the motor backs off and the **post-backoff** point becomes home (XACTUAL=0).
     * @return Result<void> indicating success or error
     *
     * This is a blocking function that automatically:
     * - Caches current settings (StealthChop, SW_MODE, ramp settings)
     * - Disables StealthChop if enabled (StallGuard requires SpreadCycle)
     * - Uses existing StallGuard configuration (SGT threshold from motor
     * config)
     * - Enables sg_stop and waits for stall event
     * - Restores cached settings after homing completes
     *
     * @note StallGuard threshold (SGT) should be configured via Initialize() or
     *       ConfigureStallGuard() before calling this method. The method uses the
     *       existing SGT configuration (or opt.stallguard_override if provided).
     */
     /**
     * @brief Perform sensorless homing using StallGuard2 (span-capped, unit-aware)
     *
     * Uses `BoundsOptions.search_speed` (in `speed_unit`) and `BoundsOptions.search_span`
     * (in `position_unit`) to perform a **single-direction** homing move. If StallGuard
     * stop triggers, the current position is defined as home (XACTUAL=0).
     *
     * `final_position` returns the **pre-zero** position in steps (i.e., the position at the
     * moment homing finished, after any optional backoff, before XACTUAL is reset to 0).
     *
     * @note `search_span` is a hard cap on travel; this prevents excessive motion even
     *       if `timeout_ms` is large.
     */
    Result<void> PerformSensorlessHoming(bool direction, const BoundsOptions& opt,
                                         int32_t &final_position,
                                         CancelCallback should_cancel = nullptr) noexcept;

    /**
     * @brief Perform homing using a reference switch (with settings caching)
     * @param direction Direction to search (true = positive, false = negative)
     * @param opt Homing options (unit-aware). Uses opt.search_speed, opt.search_span, opt.timeout_ms.
     * @param final_position Reference to store final position after homing (in steps).
     *        Note: if `opt.backoff_distance > 0`, the motor backs off and the **post-backoff** point becomes home (XACTUAL=0).
     * @param use_left_switch true to use REFL, false to use REFR
     * @return Result<void> indicating success or error
     *
     * This is a blocking function that automatically:
     * - Caches current settings (SW_MODE, ramp settings)
     * - Configures switches and performs homing
     * - Restores cached settings after homing completes
     */
    /**
     * @brief Perform homing using a reference switch (span-capped, unit-aware)
     *
     * Uses `BoundsOptions.search_speed` and `BoundsOptions.search_span` to perform a
     * **single-direction** homing move. When the selected switch triggers, the
     * current position is defined as home (XACTUAL=0).
     *
     * `final_position` returns the **pre-zero** position in steps (i.e., the position at the
     * moment homing finished, after any optional backoff, before XACTUAL is reset to 0).
     *
     * `BoundsOptions.search_span` is the max relative travel cap.
     * StallGuard-only fields in `BoundsOptions` are ignored.
     */
    Result<void> PerformSwitchHoming(bool direction, const BoundsOptions& opt,
                                     int32_t &final_position,
                                     bool use_left_switch,
                                     CancelCallback should_cancel = nullptr) noexcept;

    /**
     * @brief Perform homing using encoder index pulse (N-channel) with latch_x_act
     * @param direction Direction to search (true = positive, false = negative)
     * @param opt Homing options (unit-aware). Uses opt.search_speed, opt.search_span, opt.timeout_ms.
     * @param final_position Reference to store final position after homing (in steps).
     *        This is the latched XACTUAL at the moment the N-event fired, providing
     *        precise motor position at index pulse detection.
     * @return Result<void> indicating success or error
     *
     * This method uses the encoder's N-channel (index pulse) for high-precision homing:
     * - Enables ENCMODE.latch_x_act so XACTUAL is latched to XLATCH on N-event
     * - Clears N-event flag and searches for the index pulse
     * - When N-event fires, reads XLATCH for the exact motor position at that instant
     * - This provides microsecond-accurate position capture vs polling uncertainty
     *
     * Prerequisites:
     * - Encoder must be configured (Configure() or SetNChannelSensitivity())
     * - Motor must have an encoder with index pulse (N channel)
     *
     * @note The latched position is typically more accurate than polling-based
     *       homing because it captures XACTUAL at the hardware level when N fires.
     *
     * @see SetLatchXactualEnabled(), IsNEventDetected(), Switches::GetLatchedPosition()
     */
    Result<void> PerformEncoderIndexHoming(bool direction, const BoundsOptions& opt,
                                           int32_t &final_position,
                                     CancelCallback should_cancel = nullptr) noexcept;

    /**
     * @brief Find motion bounds using StallGuard2 (sensorless hard-stop detection).
     *
     * This is a **blocking** routine that actively moves the motor to discover the
     * reachable travel range around the current location, then optionally establishes
     * a new coordinate frame via @p home.
     *
     * **High-level behavior**
     * - Requires **internal ramp mode** (SD_MODE=0). Returns `INVALID_STATE` otherwise.
     * - Caches and restores driver settings that are modified during the procedure
     *   (ramp settings, SW_MODE, StealthChop/CoolStep, etc.).
     * - Defines a local coordinate frame at the start by setting the current
     *   position to 0 in `opt.position_unit`.
     * - Performs two span-capped searches (± `opt.search_span`) in the requested order
     *   (`opt.search_positive_first`), attempting to stop on a StallGuard2 stall.
     *
     * **StallGuard specifics**
     * - Ensures **SpreadCycle** is active (StallGuard2 is not reliable in StealthChop).
     * - Disables **CoolStep** during the scan to avoid current modulation affecting detection.
     * - Configures `SW_MODE.sg_stop=1` and `SW_MODE.en_softstop=0` (hard stop), and disables
     *   reference-switch stop/latch sources so StallGuard is the only stop cause.
     * - If `opt.stallguard_override` is non-null, it is used temporarily and restored on exit.
     * - `StallGuardConfig.min_velocity` must be > 0 (enforced), because StallGuard2 is only
     *   meaningful above a minimum velocity threshold.
     *
     * **Backoff semantics**
     * - Unlike switch/encoder methods, this routine does **not** physically back off after a stall hit.
     * - After both sides complete, it applies `opt.backoff_distance` **numerically** to compute a
     *   "safe" interval (min increased, max decreased). If this would invert the interval,
     *   the function returns `INVALID_VALUE`.
     *
     * **Home placement**
     * - If `home.mode != HomePlacement::None`, the routine may move to the selected home position
     *   (min/max/center/offset-from-min), then redefines that position as 0.
     * - Center/offset placement requires both bounds; if only one side is detected, the placement
     *   is skipped and the origin remains at the start position.
     *
     * @param opt Bounds-search options (units, span/speed, timeouts, optional current reduction, etc.).
     * @param home Optional home placement policy (defaults to `AtCenter`).
     * @param should_cancel Optional cancellation callback polled during motion; when it returns true,
     *        motion is stopped and the function returns `CANCELLED`.
     * @return `Result<BoundsResult>`:
     * - On success (`OK`): returns `BoundsResult` containing bounds in `opt.position_unit`.
     * - On error: returns an error code such as `INVALID_STATE`, `INVALID_VALUE`, `TIMEOUT`,
     *   `CANCELLED`, or a communication error.
     *
     * @warning This function **moves the motor** and can hit hard mechanical stops. Ensure your
     *          mechanism can tolerate this and use conservative speed/accel/current settings.
     *
     * @note `BoundsResult.bounded` is true only when a stall is detected on **both** sides.
     *       When a side does not stall within `search_span`, the corresponding bound value
     *       will be the final position reached at the end of the span-capped move.
     */
    Result<BoundsResult>
    FindBoundsStallGuard(const BoundsOptions& opt, const HomeConfig& home = {},
                         CancelCallback should_cancel = nullptr) noexcept;

    /**
     * @brief Find motion bounds using encoder feedback (detects "no further motion" via stalled encoder).
     *
     * This routine performs two span-capped searches (± `opt.search_span`). For each direction, it
     * monitors the encoder position during motion and considers a bound detected when the encoder
     * position stops changing for a short window while motion has already started.
     *
     * **Behavior**
     * - Requires **internal ramp mode** (SD_MODE=0).
     * - Requires the encoder subsystem to be configured (otherwise returns `INVALID_STATE`).
     * - Establishes a local coordinate frame by setting the current position to 0 in `opt.position_unit`.
     * - For each direction:
     *   - Command a relative move (± `opt.search_span`) using either `opt.search_speed` or cached VMAX,
     *     with optional accel/decel overrides.
     *   - Detect a bound when the encoder position does not change by a minimum delta for a short time
     *     after motion begins.
     *   - On detection, performs a **physical backoff move** of `opt.backoff_distance` away from the bound
     *     and records the post-backoff position as the bound.
     *
     * **Home placement**
     * After both directions complete, `home` is applied via the same policy as the StallGuard variant.
     *
     * @param opt Bounds-search options (units, span/speed, backoff, timeouts, accel/decel overrides).
     * @param home Optional home placement policy (defaults to `AtCenter`).
     * @param should_cancel Optional cancellation callback polled during motion; when it returns true,
     *        motion is stopped and the function returns `CANCELLED`.
     * @return `Result<BoundsResult>` on success, otherwise an error code.
     *
     * @warning This function **moves the motor**. Ensure your encoder is correctly wired and configured;
     *          a missing/failed encoder can prevent detection and lead to large travel up to `search_span`.
     *
     * @note When a side does not "stall" within `search_span`, `BoundsResult.bounded` will be false and
     *       that side's bound is the final span endpoint (i.e., not a detected limit).
     */
    Result<BoundsResult>
    FindBoundsEncoder(const BoundsOptions& opt, const HomeConfig& home = {},
                      CancelCallback should_cancel = nullptr) noexcept;

    /**
     * @brief Find motion bounds using reference switches (REFL/REFR stop events).
     *
     * This routine configures the TMC51x0 reference-switch stop sources and performs two span-capped
     * searches. A bound is detected when the corresponding stop event fires; the routine then backs off
     * from the switch and records the post-backoff position as the bound.
     *
     * **Behavior**
     * - Requires **internal ramp mode** (SD_MODE=0).
     * - Establishes a local coordinate frame by setting the current position to 0 in `opt.position_unit`.
     * - Preflight handling: if a switch is already active at the start and `opt.preflight_clear_active_switch`
     *   is true, the routine attempts a bounded move away (using `opt.backoff_distance`) to clear it.
     *   If preflight is disabled or `backoff_distance` is not configured, returns `INVALID_STATE`.
     * - During bounds finding, enables both `SW_MODE.stop_l_enable` and `SW_MODE.stop_r_enable`
     *   (hard stop), disables latching, and clears stale stop/latch state between passes.
     * - On each direction search:
     *   - Positive direction expects the **right** stop event; negative direction expects the **left** stop event.
     *   - After a hit, performs a **physical backoff move** of `opt.backoff_distance` away from the switch.
     *
     * **Home placement**
     * After both directions complete, `home` is applied via the same policy as the other variants.
     *
     * @param opt Bounds-search options (units, span/speed, backoff, timeouts, accel/decel overrides).
     * @param home Optional home placement policy (defaults to `AtCenter`).
     * @param should_cancel Optional cancellation callback polled during motion; when it returns true,
     *        motion is stopped and the function returns `CANCELLED`.
     * @return `Result<BoundsResult>` on success, otherwise an error code.
     *
     * @warning This function **moves the motor** and relies on correctly wired/functional limit switches.
     *
     * @note When a side does not trigger within `search_span`, `BoundsResult.bounded` will be false and
     *       that side's bound is the final span endpoint (i.e., not a detected limit).
     */
    Result<BoundsResult>
    FindBoundsSwitch(const BoundsOptions& opt, const HomeConfig& home = {},
                     CancelCallback should_cancel = nullptr) noexcept;

    /**
     * @brief Dispatch bounds finding by method.
     *
     * Convenience wrapper around:
     * - `FindBoundsStallGuard()` when `method == BoundsMethod::StallGuard`
     * - `FindBoundsEncoder()` when `method == BoundsMethod::Encoder`
     * - `FindBoundsSwitch()` when `method == BoundsMethod::Switch`
     *
     * @param method Bounds detection method to use.
     * @param opt Bounds-search options.
     * @param home Optional home placement policy (defaults to `AtCenter`).
     * @param should_cancel Optional cancellation callback polled during motion.
     * @return `Result<BoundsResult>` on success, otherwise an error code (e.g. `INVALID_STATE` if
     *         an unknown method is provided).
     */
    Result<BoundsResult>
    FindBounds(BoundsMethod method, const BoundsOptions& opt, const HomeConfig& home = {},
               CancelCallback should_cancel = nullptr) noexcept;

  private:
    TMC51x0 &driver_;
    HomingSettingsCache cache_{};

    Result<void> CacheCurrentSettings() noexcept;
    Result<void> RestoreCachedSettings() noexcept;
    Result<void>
    EnsureSpreadCycleForStallGuard() noexcept; // Disable StealthChop if needed

    Result<void> ApplyHomePlacement(BoundsResult& out, float raw_min, float raw_max,
                                    const HomeConfig& home, const BoundsOptions& opt) noexcept;
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
     * Writes to NODECONF register using address 0 (for sequential programming
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
  /**
   * @brief Check whether the chip is currently in internal ramp-generator mode
   * (SD_MODE=0).
   *
   * This reads IOIN.SD_MODE from the chip. If SD_MODE=1 (external Step/Dir),
   * the internal ramp generator is not the active motion source and many
   * motion-controller registers will not behave as users expect.
   *
   * @return Result<bool> with true if SD_MODE=0 (internal ramp), false if
   * SD_MODE=1 (external Step/Dir), or an error on communication failure.
   */
  Result<bool> IsInternalRampMode() noexcept;

  /**
   * @brief Require internal ramp-generator mode (SD_MODE=0) for motion-controller
   * operations.
   *
   * @return Result<void> OK if internal ramp mode, INVALID_STATE if in external
   * Step/Dir mode, COMM_ERROR if IOIN cannot be read.
   */
  Result<void> RequireInternalRampMode() noexcept;

  CommType &comm_; ///< Communication interface reference
  uint32_t f_clk_{ClockFreq::DEFAULT_F_CLK}; ///< TMC51x0 clock frequency in Hz
  uint8_t daisy_chain_position_; ///< Position in daisy chain (0 = first
                                 ///< chip/single chip)
  uint8_t uart_node_address_;    ///< UART node address (0-254) for multi-node
                                 ///< addressing
  uint8_t send_delay_{
      0}; ///< UART send delay (0-15) stored locally from NODECONF register
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
    uint32_t a1{0};            ///< A1 (0x24) - First acceleration
    uint32_t v1{0};            ///< V1 (0x25) - Transition velocity
    uint32_t amax{0};          ///< AMAX (0x26) - Max acceleration
    uint32_t vmax{0};          ///< VMAX (0x27) - Max velocity
    uint32_t dmax{0};          ///< DMAX (0x28) - Max deceleration
    uint32_t d1{0};            ///< D1 (0x2A) - First deceleration
    uint32_t vstop{0};         ///< VSTOP (0x2B) - Stop velocity
    uint32_t tzerowait{0};     ///< TZEROWAIT (0x2C) - Zero wait time
    uint32_t vdcmin{0};        ///< VDCMIN (0x33) - DcStep threshold
    uint32_t enc_const{0};     ///< ENC_CONST (0x3A) - Encoder constant
    uint32_t enc_deviation{0}; ///< ENC_DEVIATION (0x3D) - Encoder deviation
    uint32_t coolconf{0};      ///< COOLCONF (0x6D) - CoolStep config
    uint32_t dcctrl{0};        ///< DCCTRL (0x6E) - DcStep config
    uint32_t pwmconf{0};       ///< PWMCONF (0x70) - StealthChop config
    uint32_t nodeconf{0};      ///< NODECONF (0x03) - UART node address config
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
