/**
 * @file tmc51x0_unit_types.hpp
 * @brief Type-safe unit structures for TMC51x0 driver (TMC5130 & TMC5160)
 *
 * This file provides type-safe unit wrappers that prevent mixing up position,
 * velocity, and acceleration units. Each struct carries both the value and
 * its unit, enabling compile-time enforcement of correct unit usage.
 *
 * The structs also provide static conversion methods that perform unit
 * conversions given the necessary mechanical parameters.
 *
 * @defgroup TMC51X0_UnitTypes Type-Safe Unit Wrappers
 * @brief Type-safe wrappers for position, velocity, and acceleration units
 */

#ifndef TMC51X0_UNIT_TYPES_HPP
#define TMC51X0_UNIT_TYPES_HPP

#include <cmath>
#include <cstdint>

namespace tmc51x0 {

//==============================================================================
// Mathematical Constants
//==============================================================================
namespace UnitConstants {
constexpr float PI = 3.14159265359F;
constexpr float TWO_PI = 2.0F * PI;
constexpr float DEGREES_PER_REV = 360.0F;
constexpr float SECONDS_PER_MINUTE = 60.0F;
} // namespace UnitConstants

//==============================================================================
// Mechanical System Parameters (for conversions)
//==============================================================================

/**
 * @brief Parameters needed for unit conversions
 *
 * This struct holds all the mechanical system parameters needed to convert
 * between physical units and motor steps. Users must provide these values
 * for accurate conversions.
 */
struct ConversionParams {
  float steps_per_rev{200.0f};      ///< Motor full steps per revolution (typically 200)
  float gear_ratio{1.0f};           ///< Gear ratio (output revs / motor revs), >= 1 for reduction
  float lead_screw_pitch_mm{0.0f};  ///< Lead screw pitch in mm (0 if not lead screw)
  float belt_pitch_mm{0.0f};        ///< Belt pitch in mm (0 if not belt drive)
  uint16_t belt_pulley_teeth{0};    ///< Belt pulley teeth count (0 if not belt drive)
  uint16_t microsteps{256};         ///< Microsteps per full step (for position conversions)

  constexpr ConversionParams() noexcept = default;

  /**
   * @brief Create params for a direct drive motor
   */
  static constexpr ConversionParams DirectDrive(float steps_per_rev, uint16_t microsteps = 256) noexcept {
    ConversionParams p;
    p.steps_per_rev = steps_per_rev;
    p.microsteps = microsteps;
    return p;
  }

  /**
   * @brief Create params for a geared motor
   */
  static constexpr ConversionParams Geared(float steps_per_rev, float gear_ratio, uint16_t microsteps = 256) noexcept {
    ConversionParams p;
    p.steps_per_rev = steps_per_rev;
    p.gear_ratio = gear_ratio;
    p.microsteps = microsteps;
    return p;
  }

  /**
   * @brief Create params for a lead screw system
   */
  static constexpr ConversionParams LeadScrew(float steps_per_rev, float pitch_mm, float gear_ratio = 1.0f,
                                              uint16_t microsteps = 256) noexcept {
    ConversionParams p;
    p.steps_per_rev = steps_per_rev;
    p.lead_screw_pitch_mm = pitch_mm;
    p.gear_ratio = gear_ratio;
    p.microsteps = microsteps;
    return p;
  }

  /**
   * @brief Create params for a belt drive system
   */
  static constexpr ConversionParams BeltDrive(float steps_per_rev, float belt_pitch_mm, uint16_t pulley_teeth,
                                              float gear_ratio = 1.0f, uint16_t microsteps = 256) noexcept {
    ConversionParams p;
    p.steps_per_rev = steps_per_rev;
    p.belt_pitch_mm = belt_pitch_mm;
    p.belt_pulley_teeth = pulley_teeth;
    p.gear_ratio = gear_ratio;
    p.microsteps = microsteps;
    return p;
  }

  /**
   * @brief Get effective full steps per output revolution (accounting for gear ratio)
   */
  [[nodiscard]] constexpr float effectiveStepsPerRev() const noexcept {
    return steps_per_rev * gear_ratio;
  }

  /**
   * @brief Get mm per output revolution (for linear systems)
   */
  [[nodiscard]] constexpr float mmPerRev() const noexcept {
    if (lead_screw_pitch_mm > 0.0f) {
      return lead_screw_pitch_mm;
    }
    if (belt_pitch_mm > 0.0f && belt_pulley_teeth > 0) {
      return belt_pitch_mm * static_cast<float>(belt_pulley_teeth);
    }
    return 0.0f;
  }
};

//==============================================================================
// Position Units
//==============================================================================

/**
 * @brief Position unit enumeration
 *
 * Units specifically for position/distance values.
 */
enum class PositionUnit : uint8_t {
  Steps,        ///< Motor full steps (not microsteps)
  Microsteps,   ///< Microsteps (register-native, depends on MRES)
  Revolutions,  ///< Motor shaft revolutions (output shaft if geared)
  Radians,      ///< Radians (angular position)
  Degrees,      ///< Degrees (angular position)
  Millimeters   ///< Millimeters (linear position, requires mechanical config)
};

/**
 * @brief Type-safe position value with unit
 *
 * Encapsulates a position value along with its unit to prevent
 * accidental mixing of units at compile time.
 *
 * @example
 * Position pos = Position::Mm(100.0f);       // 100mm
 * Position deg = Position::Deg(90.0f);       // 90 degrees
 * Position rev = Position::Revolutions(2.5f); // 2.5 revolutions
 *
 * // Convert to steps
 * auto params = ConversionParams::LeadScrew(200, 2.0f);
 * float steps = pos.toSteps(params);
 */
struct Position {
  float value{0.0f};
  PositionUnit unit{PositionUnit::Steps};

  constexpr Position() noexcept = default;
  constexpr Position(float v, PositionUnit u) noexcept : value(v), unit(u) {}

  // Factory methods for clarity
  static constexpr Position Steps(float v) noexcept { return {v, PositionUnit::Steps}; }
  static constexpr Position Microsteps(float v) noexcept { return {v, PositionUnit::Microsteps}; }
  static constexpr Position Revolutions(float v) noexcept { return {v, PositionUnit::Revolutions}; }
  static constexpr Position Rad(float v) noexcept { return {v, PositionUnit::Radians}; }
  static constexpr Position Deg(float v) noexcept { return {v, PositionUnit::Degrees}; }
  static constexpr Position Mm(float v) noexcept { return {v, PositionUnit::Millimeters}; }

  /**
   * @brief Convert this position to motor full steps
   * @param params Conversion parameters (mechanical system config)
   * @return Position in motor full steps
   */
  [[nodiscard]] float toSteps(const ConversionParams& params) const noexcept {
    const float eff_steps = params.effectiveStepsPerRev();
    switch (unit) {
      case PositionUnit::Steps:
        return value;
      case PositionUnit::Microsteps:
        return (params.microsteps > 0) ? (value / static_cast<float>(params.microsteps)) : value;
      case PositionUnit::Revolutions:
        return value * eff_steps;
      case PositionUnit::Radians:
        return (value / UnitConstants::TWO_PI) * eff_steps;
      case PositionUnit::Degrees:
        return (value / UnitConstants::DEGREES_PER_REV) * eff_steps;
      case PositionUnit::Millimeters: {
        float mm_per_rev = params.mmPerRev();
        return (mm_per_rev > 0.0f) ? ((value / mm_per_rev) * eff_steps) : 0.0f;
      }
      default:
        return value;
    }
  }

  /**
   * @brief Convert this position to microsteps (register-native units)
   * @param params Conversion parameters
   * @return Position in microsteps
   */
  [[nodiscard]] float toMicrosteps(const ConversionParams& params) const noexcept {
    return toSteps(params) * static_cast<float>(params.microsteps);
  }

  /**
   * @brief Create a Position from steps
   * @param steps Value in motor full steps
   * @param params Conversion parameters
   * @param target_unit Target unit for the result
   * @return Position in the target unit
   */
  [[nodiscard]] static Position fromSteps(float steps, const ConversionParams& params,
                                          PositionUnit target_unit) noexcept {
    const float eff_steps = params.effectiveStepsPerRev();
    if (eff_steps == 0.0f) return {0.0f, target_unit};

    float result = 0.0f;
    switch (target_unit) {
      case PositionUnit::Steps:
        result = steps;
        break;
      case PositionUnit::Microsteps:
        result = steps * static_cast<float>(params.microsteps);
        break;
      case PositionUnit::Revolutions:
        result = steps / eff_steps;
        break;
      case PositionUnit::Radians:
        result = (steps / eff_steps) * UnitConstants::TWO_PI;
        break;
      case PositionUnit::Degrees:
        result = (steps / eff_steps) * UnitConstants::DEGREES_PER_REV;
        break;
      case PositionUnit::Millimeters: {
        float mm_per_rev = params.mmPerRev();
        result = (mm_per_rev > 0.0f) ? ((steps / eff_steps) * mm_per_rev) : 0.0f;
        break;
      }
    }
    return {result, target_unit};
  }

  /**
   * @brief Convert to a different unit
   */
  [[nodiscard]] Position convertTo(PositionUnit target_unit, const ConversionParams& params) const noexcept {
    float steps = toSteps(params);
    return fromSteps(steps, params, target_unit);
  }
};

//==============================================================================
// Velocity Units
//==============================================================================

/**
 * @brief Velocity unit enumeration
 *
 * Units specifically for velocity/speed values.
 */
enum class VelocityUnit : uint8_t {
  StepsPerSec,     ///< Motor full steps per second
  RPM,             ///< Revolutions per minute
  RevPerSec,       ///< Revolutions per second
  RadPerSec,       ///< Radians per second
  DegPerSec,       ///< Degrees per second
  MmPerSec         ///< Millimeters per second (linear, requires mechanical config)
};

/**
 * @brief Type-safe velocity value with unit
 *
 * Encapsulates a velocity value along with its unit to prevent
 * accidental mixing of units at compile time.
 *
 * @example
 * Velocity vel = Velocity::RPM(60.0f);       // 60 RPM
 * Velocity rps = Velocity::RevPerSec(1.0f);  // 1 revolution per second
 *
 * // Convert to steps/s
 * auto params = ConversionParams::DirectDrive(200);
 * float steps_per_sec = vel.toStepsPerSec(params);
 */
struct Velocity {
  float value{0.0f};
  VelocityUnit unit{VelocityUnit::StepsPerSec};

  constexpr Velocity() noexcept = default;
  constexpr Velocity(float v, VelocityUnit u) noexcept : value(v), unit(u) {}

  // Factory methods for clarity
  static constexpr Velocity StepsPerSec(float v) noexcept { return {v, VelocityUnit::StepsPerSec}; }
  static constexpr Velocity RPM(float v) noexcept { return {v, VelocityUnit::RPM}; }
  static constexpr Velocity RevPerSec(float v) noexcept { return {v, VelocityUnit::RevPerSec}; }
  static constexpr Velocity RadPerSec(float v) noexcept { return {v, VelocityUnit::RadPerSec}; }
  static constexpr Velocity DegPerSec(float v) noexcept { return {v, VelocityUnit::DegPerSec}; }
  static constexpr Velocity MmPerSec(float v) noexcept { return {v, VelocityUnit::MmPerSec}; }

  /**
   * @brief Convert this velocity to motor full steps per second
   * @param params Conversion parameters (mechanical system config)
   * @return Velocity in steps/s
   */
  [[nodiscard]] float toStepsPerSec(const ConversionParams& params) const noexcept {
    const float eff_steps = params.effectiveStepsPerRev();
    switch (unit) {
      case VelocityUnit::StepsPerSec:
        return value;
      case VelocityUnit::RPM:
        return (value / UnitConstants::SECONDS_PER_MINUTE) * eff_steps;
      case VelocityUnit::RevPerSec:
        return value * eff_steps;
      case VelocityUnit::RadPerSec:
        return (value / UnitConstants::TWO_PI) * eff_steps;
      case VelocityUnit::DegPerSec:
        return (value / UnitConstants::DEGREES_PER_REV) * eff_steps;
      case VelocityUnit::MmPerSec: {
        float mm_per_rev = params.mmPerRev();
        return (mm_per_rev > 0.0f) ? ((value / mm_per_rev) * eff_steps) : 0.0f;
      }
      default:
        return value;
    }
  }

  /**
   * @brief Create a Velocity from steps/s
   * @param steps_per_sec Value in motor full steps per second
   * @param params Conversion parameters
   * @param target_unit Target unit for the result
   * @return Velocity in the target unit
   */
  [[nodiscard]] static Velocity fromStepsPerSec(float steps_per_sec, const ConversionParams& params,
                                                VelocityUnit target_unit) noexcept {
    const float eff_steps = params.effectiveStepsPerRev();
    if (eff_steps == 0.0f) return {0.0f, target_unit};

    float result = 0.0f;
    switch (target_unit) {
      case VelocityUnit::StepsPerSec:
        result = steps_per_sec;
        break;
      case VelocityUnit::RPM:
        result = (steps_per_sec / eff_steps) * UnitConstants::SECONDS_PER_MINUTE;
        break;
      case VelocityUnit::RevPerSec:
        result = steps_per_sec / eff_steps;
        break;
      case VelocityUnit::RadPerSec:
        result = (steps_per_sec / eff_steps) * UnitConstants::TWO_PI;
        break;
      case VelocityUnit::DegPerSec:
        result = (steps_per_sec / eff_steps) * UnitConstants::DEGREES_PER_REV;
        break;
      case VelocityUnit::MmPerSec: {
        float mm_per_rev = params.mmPerRev();
        result = (mm_per_rev > 0.0f) ? ((steps_per_sec / eff_steps) * mm_per_rev) : 0.0f;
        break;
      }
    }
    return {result, target_unit};
  }

  /**
   * @brief Convert to a different unit
   */
  [[nodiscard]] Velocity convertTo(VelocityUnit target_unit, const ConversionParams& params) const noexcept {
    float steps_per_sec = toStepsPerSec(params);
    return fromStepsPerSec(steps_per_sec, params, target_unit);
  }
};

//==============================================================================
// Acceleration Units
//==============================================================================

/**
 * @brief Acceleration unit enumeration
 *
 * Units specifically for acceleration values.
 * Note: RPM is NOT valid for acceleration (it's rev/min, not rev/min²).
 */
enum class AccelerationUnit : uint8_t {
  StepsPerSec2,    ///< Motor full steps per second squared
  RevPerSec2,      ///< Revolutions per second squared
  RadPerSec2,      ///< Radians per second squared
  DegPerSec2,      ///< Degrees per second squared
  MmPerSec2        ///< Millimeters per second squared (linear, requires mechanical config)
};

/**
 * @brief Type-safe acceleration value with unit
 *
 * Encapsulates an acceleration value along with its unit to prevent
 * accidental mixing of units at compile time.
 *
 * @example
 * Acceleration acc = Acceleration::RevPerSec2(2.0f);  // 2 rev/s²
 * Acceleration lin = Acceleration::MmPerSec2(100.0f); // 100 mm/s²
 *
 * // Convert to steps/s²
 * auto params = ConversionParams::DirectDrive(200);
 * float steps_per_sec2 = acc.toStepsPerSec2(params);
 */
struct Acceleration {
  float value{0.0f};
  AccelerationUnit unit{AccelerationUnit::StepsPerSec2};

  constexpr Acceleration() noexcept = default;
  constexpr Acceleration(float v, AccelerationUnit u) noexcept : value(v), unit(u) {}

  // Factory methods for clarity
  static constexpr Acceleration StepsPerSec2(float v) noexcept { return {v, AccelerationUnit::StepsPerSec2}; }
  static constexpr Acceleration RevPerSec2(float v) noexcept { return {v, AccelerationUnit::RevPerSec2}; }
  static constexpr Acceleration RadPerSec2(float v) noexcept { return {v, AccelerationUnit::RadPerSec2}; }
  static constexpr Acceleration DegPerSec2(float v) noexcept { return {v, AccelerationUnit::DegPerSec2}; }
  static constexpr Acceleration MmPerSec2(float v) noexcept { return {v, AccelerationUnit::MmPerSec2}; }

  /**
   * @brief Convert this acceleration to motor full steps per second squared
   * @param params Conversion parameters (mechanical system config)
   * @return Acceleration in steps/s²
   */
  [[nodiscard]] float toStepsPerSec2(const ConversionParams& params) const noexcept {
    const float eff_steps = params.effectiveStepsPerRev();
    switch (unit) {
      case AccelerationUnit::StepsPerSec2:
        return value;
      case AccelerationUnit::RevPerSec2:
        return value * eff_steps;
      case AccelerationUnit::RadPerSec2:
        return (value / UnitConstants::TWO_PI) * eff_steps;
      case AccelerationUnit::DegPerSec2:
        return (value / UnitConstants::DEGREES_PER_REV) * eff_steps;
      case AccelerationUnit::MmPerSec2: {
        float mm_per_rev = params.mmPerRev();
        return (mm_per_rev > 0.0f) ? ((value / mm_per_rev) * eff_steps) : 0.0f;
      }
      default:
        return value;
    }
  }

  /**
   * @brief Create an Acceleration from steps/s²
   * @param steps_per_sec2 Value in motor full steps per second squared
   * @param params Conversion parameters
   * @param target_unit Target unit for the result
   * @return Acceleration in the target unit
   */
  [[nodiscard]] static Acceleration fromStepsPerSec2(float steps_per_sec2, const ConversionParams& params,
                                                     AccelerationUnit target_unit) noexcept {
    const float eff_steps = params.effectiveStepsPerRev();
    if (eff_steps == 0.0f) return {0.0f, target_unit};

    float result = 0.0f;
    switch (target_unit) {
      case AccelerationUnit::StepsPerSec2:
        result = steps_per_sec2;
        break;
      case AccelerationUnit::RevPerSec2:
        result = steps_per_sec2 / eff_steps;
        break;
      case AccelerationUnit::RadPerSec2:
        result = (steps_per_sec2 / eff_steps) * UnitConstants::TWO_PI;
        break;
      case AccelerationUnit::DegPerSec2:
        result = (steps_per_sec2 / eff_steps) * UnitConstants::DEGREES_PER_REV;
        break;
      case AccelerationUnit::MmPerSec2: {
        float mm_per_rev = params.mmPerRev();
        result = (mm_per_rev > 0.0f) ? ((steps_per_sec2 / eff_steps) * mm_per_rev) : 0.0f;
        break;
      }
    }
    return {result, target_unit};
  }

  /**
   * @brief Convert to a different unit
   */
  [[nodiscard]] Acceleration convertTo(AccelerationUnit target_unit, const ConversionParams& params) const noexcept {
    float steps_per_sec2 = toStepsPerSec2(params);
    return fromStepsPerSec2(steps_per_sec2, params, target_unit);
  }
};

//==============================================================================
// Backward Compatibility with Legacy Unit Enum
//==============================================================================

// Forward declaration - the full Unit enum is in tmc51x0_types.hpp
// These helpers convert between legacy Unit enum and new type-safe enums

/**
 * @brief Convert legacy Unit enum value to VelocityUnit
 */
inline constexpr VelocityUnit ToVelocityUnit(uint8_t legacy_unit) noexcept {
  // Legacy Unit: Steps=0, Rad=1, Deg=2, Mm=3, RPM=4, RevPerSec=5
  switch (legacy_unit) {
    case 0: return VelocityUnit::StepsPerSec;
    case 1: return VelocityUnit::RadPerSec;
    case 2: return VelocityUnit::DegPerSec;
    case 3: return VelocityUnit::MmPerSec;
    case 4: return VelocityUnit::RPM;
    case 5: return VelocityUnit::RevPerSec;
    default: return VelocityUnit::StepsPerSec;
  }
}

/**
 * @brief Convert legacy Unit enum value to PositionUnit
 */
inline constexpr PositionUnit ToPositionUnit(uint8_t legacy_unit) noexcept {
  // Legacy Unit: Steps=0, Rad=1, Deg=2, Mm=3, RPM=4, RevPerSec=5
  switch (legacy_unit) {
    case 0: return PositionUnit::Steps;
    case 1: return PositionUnit::Radians;
    case 2: return PositionUnit::Degrees;
    case 3: return PositionUnit::Millimeters;
    case 4: return PositionUnit::Revolutions;  // RPM → Revolutions for position
    case 5: return PositionUnit::Revolutions;  // RevPerSec → Revolutions for position
    default: return PositionUnit::Steps;
  }
}

/**
 * @brief Convert legacy Unit enum value to AccelerationUnit
 */
inline constexpr AccelerationUnit ToAccelerationUnit(uint8_t legacy_unit) noexcept {
  // Legacy Unit: Steps=0, Rad=1, Deg=2, Mm=3, RPM=4, RevPerSec=5
  switch (legacy_unit) {
    case 0: return AccelerationUnit::StepsPerSec2;
    case 1: return AccelerationUnit::RadPerSec2;
    case 2: return AccelerationUnit::DegPerSec2;
    case 3: return AccelerationUnit::MmPerSec2;
    case 4: return AccelerationUnit::RevPerSec2;  // RPM invalid for accel, map to RevPerSec2
    case 5: return AccelerationUnit::RevPerSec2;
    default: return AccelerationUnit::StepsPerSec2;
  }
}

//==============================================================================
// Human-Readable Conversion Helpers
//==============================================================================

inline constexpr const char* ToString(PositionUnit u) noexcept {
  switch (u) {
    case PositionUnit::Steps: return "steps";
    case PositionUnit::Microsteps: return "µsteps";
    case PositionUnit::Revolutions: return "rev";
    case PositionUnit::Radians: return "rad";
    case PositionUnit::Degrees: return "°";
    case PositionUnit::Millimeters: return "mm";
    default: return "?";
  }
}

inline constexpr const char* ToString(VelocityUnit u) noexcept {
  switch (u) {
    case VelocityUnit::StepsPerSec: return "steps/s";
    case VelocityUnit::RPM: return "RPM";
    case VelocityUnit::RevPerSec: return "rev/s";
    case VelocityUnit::RadPerSec: return "rad/s";
    case VelocityUnit::DegPerSec: return "°/s";
    case VelocityUnit::MmPerSec: return "mm/s";
    default: return "?";
  }
}

inline constexpr const char* ToString(AccelerationUnit u) noexcept {
  switch (u) {
    case AccelerationUnit::StepsPerSec2: return "steps/s²";
    case AccelerationUnit::RevPerSec2: return "rev/s²";
    case AccelerationUnit::RadPerSec2: return "rad/s²";
    case AccelerationUnit::DegPerSec2: return "°/s²";
    case AccelerationUnit::MmPerSec2: return "mm/s²";
    default: return "?";
  }
}

//==============================================================================
// Standalone Conversion Functions (for backward compatibility)
//==============================================================================

/**
 * @brief Convert steps to millimeters
 * @param steps Number of motor full steps
 * @param params Conversion parameters
 * @return Distance in millimeters
 */
inline float StepsToMm(float steps, const ConversionParams& params) noexcept {
  return Position::fromSteps(steps, params, PositionUnit::Millimeters).value;
}

/**
 * @brief Convert millimeters to steps
 * @param mm Distance in millimeters
 * @param params Conversion parameters
 * @return Number of motor full steps
 */
inline float MmToSteps(float mm, const ConversionParams& params) noexcept {
  return Position::Mm(mm).toSteps(params);
}

/**
 * @brief Convert steps to degrees
 * @param steps Number of motor full steps
 * @param params Conversion parameters
 * @return Angle in degrees
 */
inline float StepsToDegrees(float steps, const ConversionParams& params) noexcept {
  return Position::fromSteps(steps, params, PositionUnit::Degrees).value;
}

/**
 * @brief Convert degrees to steps
 * @param degrees Angle in degrees
 * @param params Conversion parameters
 * @return Number of motor full steps
 */
inline float DegreesToSteps(float degrees, const ConversionParams& params) noexcept {
  return Position::Deg(degrees).toSteps(params);
}

/**
 * @brief Convert RPM to steps per second
 * @param rpm Revolutions per minute
 * @param params Conversion parameters
 * @return Speed in steps per second
 */
inline float RpmToStepsPerSec(float rpm, const ConversionParams& params) noexcept {
  return Velocity::RPM(rpm).toStepsPerSec(params);
}

/**
 * @brief Convert steps per second to RPM
 * @param steps_per_sec Speed in steps per second
 * @param params Conversion parameters
 * @return Speed in RPM
 */
inline float StepsPerSecToRpm(float steps_per_sec, const ConversionParams& params) noexcept {
  return Velocity::fromStepsPerSec(steps_per_sec, params, VelocityUnit::RPM).value;
}

/**
 * @brief Convert mm/s to steps/s
 * @param mm_per_sec Speed in millimeters per second
 * @param params Conversion parameters
 * @return Speed in steps per second
 */
inline float MmPerSecToStepsPerSec(float mm_per_sec, const ConversionParams& params) noexcept {
  return Velocity::MmPerSec(mm_per_sec).toStepsPerSec(params);
}

/**
 * @brief Convert steps/s to mm/s
 * @param steps_per_sec Speed in steps per second
 * @param params Conversion parameters
 * @return Speed in millimeters per second
 */
inline float StepsPerSecToMmPerSec(float steps_per_sec, const ConversionParams& params) noexcept {
  return Velocity::fromStepsPerSec(steps_per_sec, params, VelocityUnit::MmPerSec).value;
}

/**
 * @brief Convert mm/s² to steps/s²
 * @param mm_per_sec2 Acceleration in millimeters per second squared
 * @param params Conversion parameters
 * @return Acceleration in steps per second squared
 */
inline float AccelMmToSteps(float mm_per_sec2, const ConversionParams& params) noexcept {
  return Acceleration::MmPerSec2(mm_per_sec2).toStepsPerSec2(params);
}

/**
 * @brief Convert steps/s² to mm/s²
 * @param steps_per_sec2 Acceleration in steps per second squared
 * @param params Conversion parameters
 * @return Acceleration in millimeters per second squared
 */
inline float AccelStepsToMm(float steps_per_sec2, const ConversionParams& params) noexcept {
  return Acceleration::fromStepsPerSec2(steps_per_sec2, params, AccelerationUnit::MmPerSec2).value;
}

} // namespace tmc51x0

#endif // TMC51X0_UNIT_TYPES_HPP
