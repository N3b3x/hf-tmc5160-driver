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
template <typename CommType> class TMC5160 {
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
   */
  explicit TMC5160(CommType &comm,
                   uint32_t f_clk = ClockFreq::DEFAULT_F_CLK) noexcept
      : comm_(comm), f_clk_(f_clk), initialized_(false) {}

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
  [[nodiscard]] CommType &GetComm() noexcept { return comm_; }

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
  bool Initialize(const DriverConfig &config = DriverConfig()) noexcept;

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
  [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

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
    TMC5160 *driver_; ///< Pointer to parent driver instance

    /**
     * @brief Construct ramp control subsystem
     * @param driver Pointer to parent TMC5160 driver instance
     */
    explicit RampControl(TMC5160 *driver) noexcept : driver_(driver) {}

    /**
     * @brief Set the ramp mode
     * @param mode Ramp mode (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD)
     * @return true if set successfully, false otherwise
     */
    bool SetRampMode(RampMode mode) noexcept;

    /**
     * @brief Set target position
     * @param position Target position in steps
     * @return true if set successfully, false otherwise
     */
    bool SetTargetPosition(int32_t position) noexcept;

    /**
     * @brief Get current position
     * @return Current position in steps, or 0 on error
     */
    int32_t GetCurrentPosition() noexcept;

    /**
     * @brief Set current position
     * @param position Position value in steps
     * @param update_encoder If true, also update encoder position
     * @return true if set successfully, false otherwise
     */
    bool SetCurrentPosition(int32_t position,
                            bool update_encoder = false) noexcept;

    /**
     * @brief Set maximum speed
     * @param speed Maximum speed in steps per second
     * @return true if set successfully, false otherwise
     */
    bool SetMaxSpeed(float speed) noexcept;

    /**
     * @brief Set acceleration and deceleration
     * @param acceleration Acceleration in steps per second squared
     * @return true if set successfully, false otherwise
     */
    bool SetAcceleration(float acceleration) noexcept;

    /**
     * @brief Set acceleration and deceleration separately
     * @param acceleration Acceleration in steps per second squared
     * @param deceleration Deceleration in steps per second squared
     * @return true if set successfully, false otherwise
     */
    bool SetAccelerations(float acceleration, float deceleration) noexcept;

    /**
     * @brief Set ramp speeds
     * @param start_speed Start speed in steps per second
     * @param stop_speed Stop speed in steps per second
     * @param transition_speed Transition speed in steps per second
     * @return true if set successfully, false otherwise
     */
    bool SetRampSpeeds(float start_speed, float stop_speed,
                       float transition_speed) noexcept;

    /**
     * @brief Get current speed
     * @return Current speed in steps per second, or 0.0f on error
     */
    float GetCurrentSpeed() noexcept;

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
  } rampControl{this};

  /**
   * @brief Motor control subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for controlling motor current, chopper configuration,
   * and stealthChop operation.
   */
  struct MotorControl {
    TMC5160 *driver_; ///< Pointer to parent driver instance

    /**
     * @brief Construct motor control subsystem
     * @param driver Pointer to parent TMC5160 driver instance
     */
    explicit MotorControl(TMC5160 *driver) noexcept : driver_(driver) {}

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
    bool ConfigureChopper(const ChopperConfig &config) noexcept;

    /**
     * @brief Configure stealthChop settings
     * @param config StealthChop configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureStealthChop(const StealthChopConfig &config) noexcept;

    /**
     * @brief Set mode change speeds
     * @param pwm_thrs Speed threshold for stealthChop (steps/s)
     * @param cool_thrs Speed threshold for coolStep (steps/s)
     * @param high_thrs Speed threshold for high-speed mode (steps/s)
     * @return true if set successfully, false otherwise
     */
    bool SetModeChangeSpeeds(float pwm_thrs, float cool_thrs,
                             float high_thrs) noexcept;

    /**
     * @brief Set global current scaler
     * @param scaler Global scaler value (32-256)
     * @return true if set successfully, false otherwise
     */
    bool SetGlobalScaler(uint16_t scaler) noexcept;
  } motorControl{this};

  /**
   * @brief Encoder subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for encoder configuration and reading encoder position.
   */
  struct Encoder {
    TMC5160 *driver_; ///< Pointer to parent driver instance

    /**
     * @brief Construct encoder subsystem
     * @param driver Pointer to parent TMC5160 driver instance
     */
    explicit Encoder(TMC5160 *driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure encoder settings
     * @param config Encoder configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool Configure(const EncoderConfig &config) noexcept;

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
    bool SetResolution(int32_t motor_steps, int32_t enc_resolution,
                       bool inverted = false) noexcept;

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
  } encoder{this};

  /**
   * @brief Diagnostics subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for reading driver status, StallGuard values, and error
   * detection.
   */
  struct Diagnostics {
    TMC5160 *driver_; ///< Pointer to parent driver instance

    /**
     * @brief Construct diagnostics subsystem
     * @param driver Pointer to parent TMC5160 driver instance
     */
    explicit Diagnostics(TMC5160 *driver) noexcept : driver_(driver) {}

    /**
     * @brief Get driver status
     * @return DriverStatus enumeration indicating current status
     */
    DriverStatus GetStatus() noexcept;

    /**
     * @brief Get StallGuard2 value
     * @return StallGuard2 value (0-1023), or 0 on error
     */
    uint16_t GetStallGuard() noexcept;

    /**
     * @brief Configure StallGuard2
     * @param config StallGuard configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureStallGuard(const StallGuardConfig &config) noexcept;

    /**
     * @brief Get driver status register value
     * @param status Reference to store the DRV_STATUS register value
     * @return true if read successfully, false otherwise
     */
    bool GetDriverStatusRegister(uint32_t &status) noexcept;

    /**
     * @brief Get ramp status register value
     * @param status Reference to store the RAMP_STAT register value
     * @return true if read successfully, false otherwise
     */
    bool GetRampStatusRegister(uint32_t &status) noexcept;
  } diagnostics{this};

  /**
   * @brief Protection subsystem
   * @ingroup TMC5160_Subsystems
   *
   * Provides methods for configuring protection systems including short circuit
   * detection and overtemperature protection.
   */
  struct Protection {
    TMC5160 *driver_; ///< Pointer to parent driver instance

    /**
     * @brief Construct protection subsystem
     * @param driver Pointer to parent TMC5160 driver instance
     */
    explicit Protection(TMC5160 *driver) noexcept : driver_(driver) {}

    /**
     * @brief Configure short protection levels
     * @param config Short protection configuration structure
     * @return true if configured successfully, false otherwise
     */
    bool ConfigureShortProtection(const ShortProtectionConfig &config) noexcept;

    /**
     * @brief Set short protection levels
     * @param s2vs_level Short to VS detector sensitivity (4-15)
     * @param s2g_level Short to GND detector sensitivity (2-15)
     * @param shortfilter Spike filtering bandwidth (0-3)
     * @param shortdelay Short detection delay (0-1)
     * @return true if set successfully, false otherwise
     */
    bool SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level,
                                  uint8_t shortfilter,
                                  uint8_t shortdelay) noexcept;
  } protection{this};

  // @}

private:
  CommType &comm_;   ///< Communication interface reference
  uint32_t f_clk_;   ///< TMC5160 clock frequency in Hz
  bool initialized_; ///< Initialization status flag

  /**
   * @brief Convert speed from Hz to internal TMC5160 units
   * @param speed_hz Speed in steps per second
   * @return Speed in internal TMC5160 units
   */
  int32_t speedToInternal(float speed_hz) const noexcept;

  /**
   * @brief Convert speed from internal TMC5160 units to Hz
   * @param speed_internal Speed in internal TMC5160 units
   * @return Speed in steps per second
   */
  float speedFromInternal(int32_t speed_internal) const noexcept;

  /**
   * @brief Convert acceleration from Hz/s to internal TMC5160 units
   * @param accel_hz Acceleration in steps per second squared
   * @return Acceleration in internal TMC5160 units
   */
  int32_t accelToInternal(float accel_hz) const noexcept;

  /**
   * @brief Convert threshold speed to TSTEP format
   * @param speed_hz Speed threshold in steps per second
   * @return TSTEP value (0 if speed is 0)
   */
  int32_t thresholdSpeedToTstep(float speed_hz) const noexcept;
};

} // namespace tmc5160

// Include template implementation
#define TMC5160_HEADER_INCLUDED
// NOLINTNEXTLINE(bugprone-suspicious-include) - Intentional: template
// implementation file
#include "../src/tmc5160.cpp"
#undef TMC5160_HEADER_INCLUDED

#endif // TMC5160_HPP
