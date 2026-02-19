#===============================================================================
# TMC51x0 Driver - Build Settings
# Shared variables for target name, includes, sources, and dependencies.
# This file is the SINGLE SOURCE OF TRUTH for the driver version.
#===============================================================================

include_guard(GLOBAL)

set(HF_TMC51X0_TARGET_NAME "hf_tmc51x0")

#===============================================================================
# Versioning (single source of truth)
#===============================================================================
set(HF_TMC51X0_VERSION_MAJOR 1)
set(HF_TMC51X0_VERSION_MINOR 0)
set(HF_TMC51X0_VERSION_PATCH 0)
set(HF_TMC51X0_VERSION "${HF_TMC51X0_VERSION_MAJOR}.${HF_TMC51X0_VERSION_MINOR}.${HF_TMC51X0_VERSION_PATCH}")

#===============================================================================
# Generate version header from template (into build directory)
#===============================================================================
set(HF_TMC51X0_VERSION_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/../inc/tmc51x0_version.h.in")
set(HF_TMC51X0_VERSION_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc51x0_generated")
set(HF_TMC51X0_VERSION_HEADER     "${HF_TMC51X0_VERSION_HEADER_DIR}/tmc51x0_version.h")

file(MAKE_DIRECTORY "${HF_TMC51X0_VERSION_HEADER_DIR}")

if(EXISTS "${HF_TMC51X0_VERSION_TEMPLATE}")
    configure_file(
        "${HF_TMC51X0_VERSION_TEMPLATE}"
        "${HF_TMC51X0_VERSION_HEADER}"
        @ONLY
    )
    message(STATUS "TMC51x0 driver v${HF_TMC51X0_VERSION} — generated tmc51x0_version.h in ${HF_TMC51X0_VERSION_HEADER_DIR}")
else()
    message(WARNING "tmc51x0_version.h.in not found at ${HF_TMC51X0_VERSION_TEMPLATE}")
endif()

#===============================================================================
# Public include directories
#===============================================================================
# Note: src/ is included because tmc51x0.ipp is in src/ and is included
# by tmc51x0.hpp using a relative path.
set(HF_TMC51X0_PUBLIC_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../inc"
    "${CMAKE_CURRENT_LIST_DIR}/../inc/features"
    "${CMAKE_CURRENT_LIST_DIR}/../inc/registers"
    "${CMAKE_CURRENT_LIST_DIR}/../src"
    "${HF_TMC51X0_VERSION_HEADER_DIR}"
)

#===============================================================================
# Source files (empty for header-only)
#===============================================================================
set(HF_TMC51X0_SOURCE_FILES)

#===============================================================================
# ESP-IDF component dependencies
#===============================================================================
set(HF_TMC51X0_IDF_REQUIRES driver)
