# =============================================================================
# TMC51x0 Driver Component - CMakeLists.txt
# =============================================================================
# This component wraps the hardware-agnostic TMC51x0 driver for ESP-IDF.
# The driver is header-only (template-based), so no source files need to be
# compiled. We just need to set up the include directories.
#
# Note: src/ is included because tmc51x0.ipp is in src/ and is included
# by tmc51x0.hpp using a relative path.
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Path Configuration
# =============================================================================
# Set the source root path relative to this component directory
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src")
set(INC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../inc")

message(STATUS "TMC51x0 Driver - ESP32 Component Configuration")
message(STATUS "Source Root: ${SRC_ROOT}")

# =============================================================================
# Source File Discovery and Validation
# =============================================================================
# Driver is header-only: implementation is in src/tmc51x0.ipp
# No separate .cpp to compile
set(DRIVER_SRCS
    # No source files - template implementation is included in header
)

# Check which source files actually exist and add them
set(EXISTING_SRCS "")
foreach(src_file ${DRIVER_SRCS})
    if(EXISTS "${src_file}")
        list(APPEND EXISTING_SRCS "${src_file}")
        message(STATUS "Found source: ${src_file}")
    else()
        message(WARNING "Source file not found: ${src_file}")
    endif()
endforeach()

# =============================================================================
# Include Directories Configuration
# =============================================================================
# Define include directories for the driver component
set(DRIVER_INCLUDE_DIRS
    "${INC_ROOT}"                           # Main driver headers
    "${INC_ROOT}/features"                  # Feature-specific headers
    "${INC_ROOT}/registers"                 # Register definition headers
    "${SRC_ROOT}"                           # Template implementation files (.ipp)
)

# =============================================================================
# Component Dependencies
# =============================================================================
# Define required ESP-IDF components for the driver
set(DRIVER_REQUIRES
    driver              # ESP-IDF driver framework
)

# =============================================================================
# Component Registration
# =============================================================================
# Register the driver component with ESP-IDF build system
idf_component_register(
    SRCS ${EXISTING_SRCS}                 # Source files (validated above)
    INCLUDE_DIRS ${DRIVER_INCLUDE_DIRS}   # Include directories defined above
    REQUIRES ${DRIVER_REQUIRES}           # Component dependencies defined above
)

# =============================================================================
# Compiler Configuration
# =============================================================================
# Header-only component becomes an INTERFACE library when SRCS is empty.
# Use INTERFACE so that targets linking this component get C++20.
target_compile_features(${COMPONENT_LIB} INTERFACE cxx_std_20)

