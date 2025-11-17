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
constexpr uint32_t DEFAULT_F_CLK =
    12000000U; ///< Typical internal clock frequency in Hz (12 MHz)
constexpr uint32_t MIN_F_CLK = 8000000U;  ///< Minimum clock frequency in Hz
constexpr uint32_t MAX_F_CLK = 16000000U; ///< Maximum clock frequency in Hz
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
 * Contains all necessary parameters for power stage, motor, and driver
 * operation.
 */
struct DriverConfig {
  PowerStageParameters power_stage; ///< Power stage configuration
  MotorParameters motor;            ///< Motor current configuration
  MotorDirection direction;         ///< Motor direction (normal or inverse)
  ChopperConfig chopper;            ///< Chopper configuration
  StealthChopConfig stealthchop;    ///< StealthChop configuration
  ShortProtectionConfig
      short_protection; ///< Short circuit protection configuration
  uint32_t f_clk;       ///< TMC5160 clock frequency in Hz (default: 12 MHz)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  DriverConfig()
      : power_stage(), motor(), direction(MotorDirection::NORMAL), chopper(),
        stealthchop(), short_protection(), f_clk(ClockFreq::DEFAULT_F_CLK) {}
};

} // namespace tmc5160

#endif // TMC5160_CONFIG_HPP
