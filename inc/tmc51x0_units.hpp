/**
 * @file tmc51x0_units.hpp
 * @brief Unit conversion functions for TMC51x0 driver (TMC5130 & TMC5160)
 *
 * This file provides free functions for converting between physical units
 * (millimeters, degrees, RPM) and internal driver units (steps, steps/s).
 * All functions are in the tmc51x0 namespace and follow PascalCase naming.
 *
 * @defgroup TMC51X0_Units Unit Conversions
 * @brief Functions for converting between physical and driver units
 */

#ifndef TMC51X0_UNITS_HPP
#define TMC51X0_UNITS_HPP

#include <cmath>
#include <cstdint>

namespace tmc51x0 {

/**
 * @brief Convert steps to millimeters
 * @param steps Number of steps
 * @param steps_per_rev Steps per revolution of the motor
 * @param lead_screw_pitch_mm Lead screw pitch in millimeters (for lead screws)
 * @return Distance in millimeters
 *
 * For lead screws: distance = (steps / steps_per_rev) * lead_screw_pitch_mm
 */
constexpr float StepsToMm(int32_t steps, float steps_per_rev, float lead_screw_pitch_mm) noexcept {
  if (steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (static_cast<float>(steps) / steps_per_rev) * lead_screw_pitch_mm;
}

/**
 * @brief Convert millimeters to steps
 * @param mm Distance in millimeters
 * @param steps_per_rev Steps per revolution of the motor
 * @param lead_screw_pitch_mm Lead screw pitch in millimeters
 * @return Number of steps
 */
constexpr int32_t MmToSteps(float mm, float steps_per_rev, float lead_screw_pitch_mm) noexcept {
  if (lead_screw_pitch_mm == 0.0F || steps_per_rev == 0.0F) {
    return 0;
  }
  return static_cast<int32_t>(std::round((mm / lead_screw_pitch_mm) * steps_per_rev));
}

/**
 * @brief Convert steps to degrees
 * @param steps Number of steps
 * @param steps_per_rev Steps per revolution of the motor
 * @return Angle in degrees
 */
constexpr float StepsToDegrees(int32_t steps, float steps_per_rev) noexcept {
  if (steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (static_cast<float>(steps) / steps_per_rev) * 360.0F;
}

/**
 * @brief Convert degrees to steps
 * @param degrees Angle in degrees
 * @param steps_per_rev Steps per revolution of the motor
 * @return Number of steps
 */
constexpr int32_t DegreesToSteps(float degrees, float steps_per_rev) noexcept {
  if (steps_per_rev == 0.0F) {
    return 0;
  }
  return static_cast<int32_t>(std::round((degrees / 360.0F) * steps_per_rev));
}

/**
 * @brief Convert RPM to steps per second
 * @param rpm Revolutions per minute
 * @param steps_per_rev Steps per revolution of the motor
 * @return Speed in steps per second
 */
constexpr float RpmToStepsPerSec(float rpm, float steps_per_rev) noexcept {
  return (rpm / 60.0F) * steps_per_rev;
}

/**
 * @brief Convert steps per second to RPM
 * @param steps_per_sec Speed in steps per second
 * @param steps_per_rev Steps per revolution of the motor
 * @return Speed in RPM
 */
constexpr float StepsPerSecToRpm(float steps_per_sec, float steps_per_rev) noexcept {
  if (steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (steps_per_sec / steps_per_rev) * 60.0F;
}

/**
 * @brief Convert millimeters per second to steps per second
 * @param mm_per_sec Speed in millimeters per second
 * @param steps_per_rev Steps per revolution of the motor
 * @param lead_screw_pitch_mm Lead screw pitch in millimeters
 * @return Speed in steps per second
 */
constexpr float MmPerSecToStepsPerSec(float mm_per_sec, float steps_per_rev, float lead_screw_pitch_mm) noexcept {
  if (lead_screw_pitch_mm == 0.0F || steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (mm_per_sec / lead_screw_pitch_mm) * steps_per_rev;
}

/**
 * @brief Convert steps per second to millimeters per second
 * @param steps_per_sec Speed in steps per second
 * @param steps_per_rev Steps per revolution of the motor
 * @param lead_screw_pitch_mm Lead screw pitch in millimeters
 * @return Speed in millimeters per second
 */
constexpr float StepsPerSecToMmPerSec(float steps_per_sec, float steps_per_rev, float lead_screw_pitch_mm) noexcept {
  if (steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (steps_per_sec / steps_per_rev) * lead_screw_pitch_mm;
}

/**
 * @brief Convert acceleration from mm/s² to steps/s²
 * @param accel_mm_per_sec2 Acceleration in millimeters per second squared
 * @param steps_per_rev Steps per revolution of the motor
 * @param lead_screw_pitch_mm Lead screw pitch in millimeters
 * @return Acceleration in steps per second squared
 */
constexpr float AccelerationMmToSteps(float accel_mm_per_sec2, float steps_per_rev,
                                      float lead_screw_pitch_mm) noexcept {
  if (lead_screw_pitch_mm == 0.0F || steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (accel_mm_per_sec2 / lead_screw_pitch_mm) * steps_per_rev;
}

/**
 * @brief Convert acceleration from steps/s² to mm/s²
 * @param accel_steps_per_sec2 Acceleration in steps per second squared
 * @param steps_per_rev Steps per revolution of the motor
 * @param lead_screw_pitch_mm Lead screw pitch in millimeters
 * @return Acceleration in millimeters per second squared
 */
constexpr float AccelerationStepsToMm(float accel_steps_per_sec2, float steps_per_rev,
                                      float lead_screw_pitch_mm) noexcept {
  if (steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (accel_steps_per_sec2 / steps_per_rev) * lead_screw_pitch_mm;
}

/**
 * @brief Convert belt drive distance (teeth) to steps
 * @param teeth Number of belt teeth
 * @param steps_per_rev Steps per revolution of the motor
 * @param belt_pulley_teeth Number of teeth on the motor pulley
 * @return Number of steps
 */
constexpr int32_t BeltTeethToSteps(uint32_t teeth, float steps_per_rev, uint16_t belt_pulley_teeth) noexcept {
  if (belt_pulley_teeth == 0 || steps_per_rev == 0.0F) {
    return 0;
  }
  return static_cast<int32_t>(
      std::round((static_cast<float>(teeth) / static_cast<float>(belt_pulley_teeth)) * steps_per_rev));
}

/**
 * @brief Convert steps to belt drive distance (teeth)
 * @param steps Number of steps
 * @param steps_per_rev Steps per revolution of the motor
 * @param belt_pulley_teeth Number of teeth on the motor pulley
 * @return Number of belt teeth
 */
constexpr float StepsToBeltTeeth(int32_t steps, float steps_per_rev, uint16_t belt_pulley_teeth) noexcept {
  if (steps_per_rev == 0.0F) {
    return 0.0F;
  }
  return (static_cast<float>(steps) / steps_per_rev) * static_cast<float>(belt_pulley_teeth);
}

} // namespace tmc51x0

#endif // TMC51X0_UNITS_HPP
