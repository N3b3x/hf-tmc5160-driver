/**
 * @file tmc51x0_register_defs.cpp
 * @brief Tmc51X0 Register Defs definitions
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */

#include "../inc/registers/tmc51x0_register_defs.hpp"
#include <cstddef>

namespace tmc51x0 {

const char *GetRegisterDef(uint8_t address) {
  switch (address) {
#define X(addr, name, access, category, desc)                                  \
  case addr:                                                                   \
    return #name ": " desc;
    REGISTER_LIST(X)
#undef X
  default:
    return nullptr;
  }
}

} // namespace tmc51x0
