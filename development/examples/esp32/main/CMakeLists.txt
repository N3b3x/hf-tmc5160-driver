# =============================================================================
# TMC51x0 Driver Examples - Main Component CMakeLists.txt (TMC5130 & TMC5160)
# =============================================================================
# This CMakeLists.txt is designed to work with the build_app.sh script
# which handles app type validation and source file discovery.
# 
# The component supports ESP-IDF's two-phase build process:
# 1. Requirements phase: Uses defaults to allow component discovery
# 2. Build phase: Enforces proper usage with helpful error messages
# =============================================================================

# =============================================================================
# Variable Validation and Defaults
# =============================================================================
# Set defaults for requirements phase, validate during build phase
if(NOT DEFINED APP_TYPE)
    # During requirements phase, use a default to allow component discovery
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        # This is the requirements phase - use default
        set(APP_TYPE "${DEFAULT_APP_TYPE}")
        message(STATUS "APP_TYPE not defined during requirements phase, using default: ${APP_TYPE}")
    else()
        # This is the actual build phase - fail with helpful error
        message(FATAL_ERROR 
            "APP_TYPE not defined. Please use build_app.sh to build this project.\n"
            "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} Release\n"
            "Use './scripts/build_app.sh list' to see available app types."
        )
    endif()
endif()

if(NOT DEFINED APP_SOURCE_FILE)
    # During requirements phase, use a default to allow component discovery
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        # This is the requirements phase - use default
        set(APP_SOURCE_FILE "${DEFAULT_SOURCE_FILE}")
        message(STATUS "APP_SOURCE_FILE not defined during requirements phase, using default: ${APP_SOURCE_FILE}")
    else()
        # This is the actual build phase - fail with helpful error
        message(FATAL_ERROR 
            "APP_SOURCE_FILE not defined. Please use build_app.sh to build this project.\n"
            "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} ${DEFAULT_BUILD_TYPE}\n"
            "Use './scripts/build_app.sh list' to see available app types."
        )
    endif()
endif()

# =============================================================================
# Build Configuration and Status Messages
# =============================================================================
message(STATUS "Building app type: ${APP_TYPE}")
message(STATUS "Using source file: ${APP_SOURCE_FILE}")

# =============================================================================
# Include Directories Configuration
# =============================================================================
# Set include paths relative to current repository structure
set(INC_BASE_PATH "../../../inc")
set(SRC_BASE_PATH "../../../src")

# Define include directories for the main component
set(MAIN_INCLUDE_DIRS
    "."                                    # Current directory (main/)
    "${INC_BASE_PATH}"                     # Main driver headers
    "${SRC_BASE_PATH}"                     # Driver implementation headers
)


# =============================================================================
# Component Dependencies
# =============================================================================
# Define required ESP-IDF components for the main application
set(MAIN_REQUIRES
    driver
    esp_timer
    freertos
    hf_tmc51x0
    esp_wifi
    nvs_flash
    mbedtls
)

# =============================================================================
# Component Registration
# =============================================================================
# Register the main component with ESP-IDF build system
# Only include the source file for the specific app being built
set(COMPONENT_SRCS "${APP_SOURCE_FILE}")

# Add ESP-NOW test unit additional sources if building that app
if(APP_TYPE STREQUAL "fatigue_test_espnow_unit")
    list(APPEND COMPONENT_SRCS
        "fatigue_test_espnow/espnow_receiver.cpp"
        "fatigue_test_espnow/espnow_peer_store.cpp"
    )
endif()

# Add ESP-NOW protocol test unit additional sources (only needs receiver, no bounds finders)
if(APP_TYPE STREQUAL "espnow_protocol_test_unit")
    list(APPEND COMPONENT_SRCS
        "fatigue_test_espnow/espnow_receiver.cpp"
        "fatigue_test_espnow/espnow_peer_store.cpp"
    )
endif()

idf_component_register(
    SRCS ${COMPONENT_SRCS}                 # Source files (main + additional for ESP-NOW apps)
    INCLUDE_DIRS ${MAIN_INCLUDE_DIRS}      # Include directories defined above
    REQUIRES ${MAIN_REQUIRES}              # Component dependencies defined above
)

# =============================================================================
# Compiler Configuration
# =============================================================================
# Set C++ standard to C++17 (required for modern C++ features)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)

# =============================================================================
# Build Type Specific Compiler Flags
# =============================================================================
# Set compiler flags based on build type (Debug vs Release)
if(BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -O0
        -g3
        -DDEBUG
    )
else()
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -O2
        -g
        -DNDEBUG
    )
endif()

# =============================================================================
# Example Type Compile Definitions
# =============================================================================
# Add compile definitions for each example type to enable conditional compilation
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    "EXAMPLE_TYPE_${APP_TYPE}=1"           # Enable specific example type features
)

# =============================================================================
# ESP-NOW Pairing Secret Injection
# =============================================================================
# The secret is passed from build_app.sh via -D ESPNOW_PAIRING_SECRET_HEX
# If not provided:
#   - Debug builds: Use placeholder (with warning)
#   - Release builds: Compile error with helpful message
# =============================================================================
if(DEFINED ESPNOW_PAIRING_SECRET_HEX)
    message(STATUS "ESP-NOW pairing secret: configured")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        ESPNOW_PAIRING_SECRET_HEX="${ESPNOW_PAIRING_SECRET_HEX}"
    )
else()
    message(STATUS "ESP-NOW pairing secret: not configured (using build-type defaults)")
endif()
