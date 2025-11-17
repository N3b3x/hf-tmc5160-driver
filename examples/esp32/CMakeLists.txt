# =============================================================================
# TMC5160 Driver Examples - Project Root CMakeLists.txt
# =============================================================================
# This CMakeLists.txt serves as the main project configuration for TMC5160
# driver examples. It defines shared variables, validates build parameters,
# and integrates with the ESP-IDF build system.
#
# Key Features:
# - Shared variable system for consistent defaults across components
# - Build parameter validation with helpful error messages
# - Integration with build_app.sh script for app type management
# - ESP-IDF project configuration and naming
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Shared Default Variables
# =============================================================================
# Define default values for requirements phase (used by components)
set(DEFAULT_APP_TYPE "basic_positioning" CACHE STRING "Default app type for requirements phase")
set(DEFAULT_BUILD_TYPE "Release" CACHE STRING "Default build type for requirements phase")
set(DEFAULT_SOURCE_FILE "main.cpp" CACHE STRING "Default source file for requirements phase")

# =============================================================================
# Build Parameter Validation
# =============================================================================
# Validate that required variables are provided by build_app.sh
if(NOT DEFINED APP_TYPE)
    message(FATAL_ERROR 
        "APP_TYPE not defined. Please use build_app.sh to build this project.\n"
        "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} ${DEFAULT_BUILD_TYPE}\n"
        "Use './scripts/build_app.sh list' to see available app types."
    )
endif()

if(NOT DEFINED BUILD_TYPE)
    message(FATAL_ERROR 
        "BUILD_TYPE not defined. Please use build_app.sh to build this project.\n"
        "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} ${DEFAULT_BUILD_TYPE}\n"
        "Use './scripts/build_app.sh list' to see available app types."
    )
endif()

# =============================================================================
# Build Configuration and Status Messages
# =============================================================================
message(STATUS "Building app type: ${APP_TYPE}")
message(STATUS "Build type: ${BUILD_TYPE}")

# =============================================================================
# Global Variable Configuration
# =============================================================================
# Set these as global variables for all components to access
set(APP_TYPE "${APP_TYPE}" CACHE STRING "App type to build" FORCE)
set(BUILD_TYPE "${BUILD_TYPE}" CACHE STRING "Build type (Debug/Release)" FORCE)

# =============================================================================
# ESP-IDF Integration
# =============================================================================
# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# =============================================================================
# Project Configuration
# =============================================================================
# Set project name based on app type for unique identification
project(esp32_tmc5160_${APP_TYPE}_app)

