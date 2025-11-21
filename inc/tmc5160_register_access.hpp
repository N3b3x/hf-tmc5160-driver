/**
 * @file tmc5160_register_access.hpp
 * @brief Register access type definitions and utilities for TMC5160
 *
 * This file provides enums and helper functions for register access types,
 * generated from the X-MACRO register definitions following TMC9660 pattern.
 *
 * @defgroup TMC5160_RegisterAccess Register Access Types
 * @brief Register access type definitions and utilities
 */

#ifndef TMC5160_REGISTER_ACCESS_HPP
#define TMC5160_REGISTER_ACCESS_HPP

#include <cstdint>
#include "tmc5160_register_defs.hpp"

namespace tmc5160 {

/**
 * @brief Register access type enumeration
 */
enum class RegisterAccess : uint8_t {
  READ_ONLY = 0,      ///< Register is read-only (R)
  WRITE_ONLY = 1,     ///< Register is write-only (W)
  READ_WRITE = 2,     ///< Register is read-write (RW)
  READ_WRITE_CLEAR = 3 ///< Register is read-write with clear behavior (RWC)
};

/**
 * @brief Register category enumeration
 */
enum class RegisterCategory : uint8_t {
  CONFIG,      ///< Configuration registers
  STATUS,      ///< Status/result registers
  MOTION,       ///< Motion control registers
  CURRENT,      ///< Current control registers
  CHOPPER,      ///< Chopper configuration
  ENCODER,      ///< Encoder registers
  MICROSTEP,    ///< Microstep lookup table
  PROTECTION,   ///< Protection and safety
  OTP,          ///< One-time programmable memory
  IO            ///< Input/Output pins
};

/**
 * @brief Register information structure
 */
struct RegisterInfo {
  uint8_t address;              ///< Register address (0x00-0x73)
  RegisterAccess access;         ///< Access type
  RegisterCategory category;    ///< Register category
  const char* name;              ///< Register name
  const char* description;       ///< Register description
};

// Helper macros to map access identifier to enum (must be defined before REGISTER_LIST usage)
#define ACCESS_TO_ENUM(access) ACCESS_TO_ENUM_IMPL(access)
#define ACCESS_TO_ENUM_IMPL(access) ACCESS_TO_ENUM_##access
#define ACCESS_TO_ENUM_R RegisterAccess::READ_ONLY
#define ACCESS_TO_ENUM_W RegisterAccess::WRITE_ONLY
#define ACCESS_TO_ENUM_RW RegisterAccess::READ_WRITE
#define ACCESS_TO_ENUM_RWC RegisterAccess::READ_WRITE_CLEAR

/**
 * @brief Get register access type from address
 * @param address Register address (0x00-0x73)
 * @return RegisterAccess enum value, or READ_WRITE if address not found
 * 
 * This function is generated from the X-MACRO definitions in tmc5160_register_defs.hpp.
 * Access types are verified against the TMC5160A datasheet Rev 1.18.
 */
constexpr RegisterAccess GetRegisterAccess(uint8_t address) noexcept {
  switch (address) {
    #define X(addr, name, access, category, desc) \
      case addr: return ACCESS_TO_ENUM(access);
    REGISTER_LIST(X)
    #undef X
    default: return RegisterAccess::READ_WRITE; // Unknown - default to R/W
  }
}

// Clean up helper macros
#undef ACCESS_TO_ENUM
#undef ACCESS_TO_ENUM_IMPL
#undef ACCESS_TO_ENUM_R
#undef ACCESS_TO_ENUM_W
#undef ACCESS_TO_ENUM_RW
#undef ACCESS_TO_ENUM_RWC

/**
 * @brief Check if register is readable
 * @param address Register address
 * @return true if register can be read, false otherwise
 */
constexpr bool IsRegisterReadable(uint8_t address) noexcept {
  RegisterAccess access = GetRegisterAccess(address);
  return (access == RegisterAccess::READ_ONLY ||
          access == RegisterAccess::READ_WRITE ||
          access == RegisterAccess::READ_WRITE_CLEAR);
}

/**
 * @brief Check if register is writable
 * @param address Register address
 * @return true if register can be written, false otherwise
 */
constexpr bool IsRegisterWritable(uint8_t address) noexcept {
  RegisterAccess access = GetRegisterAccess(address);
  return (access == RegisterAccess::WRITE_ONLY ||
          access == RegisterAccess::READ_WRITE ||
          access == RegisterAccess::READ_WRITE_CLEAR);
}

/**
 * @brief Get access type string representation
 * @param access RegisterAccess enum value
 * @return String representation ("R", "W", "RW", or "RWC")
 */
constexpr const char* GetAccessTypeString(RegisterAccess access) noexcept {
  switch (access) {
    case RegisterAccess::READ_ONLY: return "R";
    case RegisterAccess::WRITE_ONLY: return "W";
    case RegisterAccess::READ_WRITE: return "RW";
    case RegisterAccess::READ_WRITE_CLEAR: return "RWC";
    default: return "?";
  }
}

} // namespace tmc5160

#endif // TMC5160_REGISTER_ACCESS_HPP
