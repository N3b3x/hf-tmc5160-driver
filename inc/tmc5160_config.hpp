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
 * If `motor_spec.global_scaler == 0` or `motor_spec.irun == 0`, the driver will automatically
 * calculate IRUN, IHOLD, and GLOBAL_SCALER from `motor_spec.user` parameters using datasheet equations.
 * This requires:
 * - `motor_spec.sense_resistor_mohm` (e.g., 50 for 0.05Ω)
 * - `motor_spec.supply_voltage_mv` (e.g., 24000 for 24V)
 * - `motor_spec.rated_current_ma` or `motor_spec.run_current_ma`
 * - `motor_spec.winding_resistance_mohm` (for StealthChop lower limit validation)
 * - `motor_spec.scaler_adjustment_percent`, `irun_adjustment_percent`, `ihold_adjustment_percent` (optional fine-tuning)
 *
 * See `tmc5160_motor_calc.hpp` for calculation details.
 */
struct DriverConfig {
  PowerStageParameters power_stage;                 ///< Power stage configuration (includes short protection)
  MotorDirection direction{MotorDirection::NORMAL}; ///< Motor direction (normal or inverse)
  ChopperConfig chopper;                            ///< Chopper configuration
  StealthChopConfig stealthchop;                    ///< StealthChop configuration
  GlobalConfig global_config;                       ///< Global configuration (GCONF register)
  RampParameters ramp_params;                       ///< Additional ramp parameters
  uint32_t f_clk;                                   ///< TMC5160 clock frequency in Hz (default: 12 MHz)
  
  MotorSpec motor_spec;                             ///< Motor specifications (user parameters + calculated current settings)
  MechanicalSystem mechanical;                      ///< Mechanical system configuration (gearing, leadscrew, etc.)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  DriverConfig() noexcept : f_clk(ClockFreq::DEFAULT_F_CLK) {}
};

} // namespace tmc5160

#endif // TMC5160_CONFIG_HPP
