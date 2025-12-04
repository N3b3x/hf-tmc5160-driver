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

# Add Adafruit component include paths for ESP-NOW UI board
if(APP_TYPE STREQUAL "fatigue_test_espnow_ui")
    list(APPEND MAIN_INCLUDE_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_GFX"
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_EPD"
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_SH1106_ESPIDF"
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/EC11_Encoder/inc"
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_BusIO_ESPIDF"
    )
endif()

# Add Adafruit component include paths for slideshow app
if(APP_TYPE STREQUAL "fatigue_test_espnow_slideshow")
    list(APPEND MAIN_INCLUDE_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_GFX"
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_EPD"
        "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_BusIO_ESPIDF"
    )
endif()

# =============================================================================
# Component Dependencies
# =============================================================================
# Define required ESP-IDF components for the main application
# Slideshow app has different requirements, so handle it separately
if(APP_TYPE STREQUAL "fatigue_test_espnow_slideshow")
    # Slideshow app requirements (no WiFi, NVS, TMC driver needed)
    set(MAIN_REQUIRES
        driver
        esp_timer
        freertos
        Adafruit_GFX
        Adafruit_EPD
        Adafruit_BusIO_ESPIDF
        fatfs
        esp_driver_sdspi
    )
else()
    # Default requirements for other apps
    set(MAIN_REQUIRES
        driver
        esp_timer
        freertos
        hf_tmc51x0
        esp_wifi
        nvs_flash
    )
endif()

# Add Adafruit components for ESP-NOW UI board
if(APP_TYPE STREQUAL "fatigue_test_espnow_ui")
    list(APPEND MAIN_REQUIRES
        Adafruit_GFX          # Adafruit GFX graphics library
        Adafruit_EPD          # Adafruit EPD e-ink display library (for slideshow app)
        Adafruit_BusIO_ESPIDF # Adafruit BusIO ESP-IDF native implementation (I2C + SPI)
        Adafruit_SH1106_ESPIDF # SH1106 OLED driver for ESP-IDF
        EC11_Encoder          # EC11 rotary encoder driver
    )
endif()

# =============================================================================
# Component Registration
# =============================================================================
# Register the main component with ESP-IDF build system
# For ESP-NOW apps, we need to include additional source files
set(COMPONENT_SRCS "${APP_SOURCE_FILE}")

# Add ESP-NOW test unit additional sources if building that app
if(APP_TYPE STREQUAL "fatigue_test_espnow_unit")
    list(APPEND COMPONENT_SRCS
        "fatigue_test_espnow/test_unit/bounds_finder_stallguard.cpp"
        "fatigue_test_espnow/test_unit/bounds_finder_encoder.cpp"
        "fatigue_test_espnow/test_unit/espnow_receiver.cpp"
    )
endif()

# Add ESP-NOW UI board additional sources if building that app
if(APP_TYPE STREQUAL "fatigue_test_espnow_ui")
    list(APPEND COMPONENT_SRCS
        "fatigue_test_espnow/ui_board/button.cpp"
        "fatigue_test_espnow/ui_board/settings.cpp"
        "fatigue_test_espnow/ui_board/espnow_protocol.cpp"
        "fatigue_test_espnow/ui_board/ui_oled.cpp"      # OLED-based UI (replaces e-ink UI)
        "fatigue_test_espnow/ui_board/oled_menu.cpp"   # Menu system for OLED
        # "fatigue_test_espnow/ui_board/ui.cpp"        # E-ink UI (kept for slideshow app)
    )
endif()

# Add slideshow app additional sources
if(APP_TYPE STREQUAL "fatigue_test_espnow_slideshow")
    list(APPEND COMPONENT_SRCS
        "fatigue_test_espnow/ui_slideshow_board/button.cpp"
        "fatigue_test_espnow/ui_slideshow_board/sd_card.cpp"
        "fatigue_test_espnow/ui_slideshow_board/image_loader.cpp"
        "fatigue_test_espnow/ui_slideshow_board/slideshow.cpp"
    )
endif()

# Debug: Print MAIN_REQUIRES for slideshow app
if(APP_TYPE STREQUAL "fatigue_test_espnow_slideshow")
    message(STATUS "Slideshow MAIN_REQUIRES: ${MAIN_REQUIRES}")
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
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

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
