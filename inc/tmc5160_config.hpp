/**
 * @file tmc5160_config.hpp
 * @brief Configuration structures for TMC5160 stepper motor driver
 *
 * This file contains configuration structures used for initializing
 * and configuring the TMC5160 driver.
 *
 * @defgroup TMC5160_Config Configuration Structures
 * @brief Configuration structures for driver initialization
 */

#ifndef TMC5160_CONFIG_HPP
#define TMC5160_CONFIG_HPP

#include "tmc5160_types.hpp"
#include <cstdint>

namespace tmc5160 {

/**
 * @brief TMC5160 clock frequency constants
 */
namespace ClockFreq {
constexpr uint32_t DEFAULT_F_CLK = 12000000U; ///< Typical internal clock frequency in Hz (12 MHz)
constexpr uint32_t MIN_F_CLK = 8000000U;      ///< Minimum clock frequency in Hz
constexpr uint32_t MAX_F_CLK = 16000000U;     ///< Maximum clock frequency in Hz
} // namespace ClockFreq

/**
 * @brief Microstep constants
 */
namespace Microsteps {
constexpr uint16_t USTEP_COUNT = 256U; ///< Number of microsteps per full step
} // namespace Microsteps

/**
 * @brief Driver initialization configuration structure
 *
 * Complete configuration structure for initializing the TMC5160 driver.
 * Contains all necessary parameters for power stage, motor, and driver operation.
 *
 * ## Automatic Current Calculation
 *
 * **IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated** from `motor_spec` parameters
 * during `Initialize()`. These values are stored internally and NOT in `motor_spec`.
 *
 * **Required Parameters** for automatic calculation:
 * - `motor_spec.sense_resistor_mohm` (e.g., 50 for 0.05Ω) - **MUST be non-zero**
 * - `motor_spec.supply_voltage_mv` (e.g., 24000 for 24V) - **MUST be non-zero**
 * - `motor_spec.rated_current_ma` or `motor_spec.run_current_ma` (if run_current_ma=0, uses rated_current_ma)
 *
 * **Optional Parameters**:
 * - `motor_spec.winding_resistance_mohm` (for StealthChop lower limit validation)
 * - `motor_spec.winding_inductance_uh` (for motor characterization)
 * - `motor_spec.hold_current_ma` (if 0, auto-calculates as 30% of run current)
 * - `motor_spec.scaler_adjustment_percent`, `irun_adjustment_percent`, `ihold_adjustment_percent` (for fine-tuning)
 *
 * **DO NOT** manually set IRUN, IHOLD, or GLOBAL_SCALER - they are calculated automatically.
 *
 * ## ESP32 Platform Configuration
 *
 * For ESP32 examples, use helper functions from `esp32_tmc5160_bus_config.hpp`:
 * - `ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg)`
 * - `ConfigureDriverFromMotor_17HS4401S_Direct(cfg)`
 * - `ConfigureDriverFromMotor_AppliedMotion_5034(cfg)`
 *
 * These functions automatically configure all parameters from compile-time motor/platform specifications.
 *
 * See `tmc5160_motor_calc.hpp` for calculation details.
 * See `docs/configuration.md` for configuration guide.
 */
struct DriverConfig {
  PowerStageParameters power_stage;                 ///< Power stage configuration (includes short protection)
  MotorDirection direction{MotorDirection::NORMAL}; ///< Motor direction (normal or inverse)
  ChopperConfig chopper;                            ///< Chopper configuration (SpreadCycle or Classic mode)
  StealthChopConfig stealthchop;                    ///< StealthChop configuration
  GlobalConfig global_config;                       ///< Global configuration (GCONF register)
  RampConfig ramp_config;                           ///< Ramp generator configuration (with unit support)
  uint32_t f_clk;                                   ///< TMC5160 clock frequency in Hz (default: 12 MHz)
  
  MotorSpec motor_spec;                             ///< Motor specifications (physical parameters for automatic current calculation)
  MechanicalSystem mechanical;                      ///< Mechanical system configuration (gearing, leadscrew, etc.) for unit conversions

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * Motor current settings (IRUN, IHOLD, GLOBAL_SCALER) are calculated automatically during Initialize()
   * if motor_spec.sense_resistor_mohm and motor_spec.supply_voltage_mv are set.
   */
  DriverConfig() noexcept : f_clk(ClockFreq::DEFAULT_F_CLK) {}
};

} // namespace tmc5160

#endif // TMC5160_CONFIG_HPP
