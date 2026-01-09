/**
 * @file tmc51x0_unit_types.hpp
 * @brief Type-safe unit structures for TMC51x0 driver (TMC5130 & TMC5160)
 *
 * This file provides type-safe unit wrappers that prevent mixing up position,
 * velocity, and acceleration units. Each struct carries both the value and
 * its unit, enabling compile-time enforcement of correct unit usage.
 *
 * @defgroup TMC51X0_UnitTypes Type-Safe Unit Wrappers
 * @brief Type-safe wrappers for position, velocity, and acceleration units
 */

#ifndef TMC51X0_UNIT_TYPES_HPP
#define TMC51X0_UNIT_TYPES_HPP

#include <cstdint>

namespace tmc51x0 {

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
  Revolutions,  ///< Motor shaft revolutions
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
 * Velocity lin = Velocity::MmPerSec(10.0f);  // 10 mm/s
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
};

//==============================================================================
// Backward Compatibility Helpers
//==============================================================================

/**
 * @brief Convert legacy Unit enum to VelocityUnit
 *
 * Provides backward compatibility with existing code using the Unit enum.
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
 * @brief Convert legacy Unit enum to PositionUnit
 *
 * Provides backward compatibility with existing code using the Unit enum.
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
 * @brief Convert legacy Unit enum to AccelerationUnit
 *
 * Provides backward compatibility with existing code using the Unit enum.
 * Note: RPM is not valid for acceleration and maps to RevPerSec2.
 */
inline constexpr AccelerationUnit ToAccelerationUnit(uint8_t legacy_unit) noexcept {
  // Legacy Unit: Steps=0, Rad=1, Deg=2, Mm=3, RPM=4, RevPerSec=5
  switch (legacy_unit) {
    case 0: return AccelerationUnit::StepsPerSec2;
    case 1: return AccelerationUnit::RadPerSec2;
    case 2: return AccelerationUnit::DegPerSec2;
    case 3: return AccelerationUnit::MmPerSec2;
    case 4: return AccelerationUnit::RevPerSec2;  // RPM → RevPerSec2 (best guess)
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

} // namespace tmc51x0

#endif // TMC51X0_UNIT_TYPES_HPP
