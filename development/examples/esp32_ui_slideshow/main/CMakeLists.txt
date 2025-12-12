# =============================================================================
# ESP32 UI Slideshow - Main Component CMakeLists.txt
# =============================================================================
# Standalone slideshow application with e-ink display
# No TMC driver dependencies
# =============================================================================

# =============================================================================
# Variable Validation and Defaults
# =============================================================================
if(NOT DEFINED APP_TYPE)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(APP_TYPE "${DEFAULT_APP_TYPE}")
        message(STATUS "APP_TYPE not defined during requirements phase, using default: ${APP_TYPE}")
    else()
        message(FATAL_ERROR 
            "APP_TYPE not defined. Please use build_app.sh to build this project.\n"
            "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} Release\n"
        )
    endif()
endif()

if(NOT DEFINED APP_SOURCE_FILE)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(APP_SOURCE_FILE "${DEFAULT_SOURCE_FILE}")
        message(STATUS "APP_SOURCE_FILE not defined during requirements phase, using default: ${APP_SOURCE_FILE}")
    else()
        message(FATAL_ERROR 
            "APP_SOURCE_FILE not defined. Please use build_app.sh to build this project.\n"
        )
    endif()
endif()

message(STATUS "Building app type: ${APP_TYPE}")
message(STATUS "Using source file: ${APP_SOURCE_FILE}")

# =============================================================================
# Include Directories Configuration
# =============================================================================
set(MAIN_INCLUDE_DIRS
    "."                                    # Current directory (main/)
    "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_GFX"
    "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_EPD"
    "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_SH1106_ESPIDF"
    "${CMAKE_CURRENT_SOURCE_DIR}/../components/Adafruit_BusIO_ESPIDF"
    "${IDF_PATH}/components/fatfs/vfs"    # For esp_vfs_fat.h
)

# =============================================================================
# Component Dependencies
# =============================================================================
# No TMC driver dependencies - standalone display application
set(MAIN_REQUIRES
    driver
    esp_timer
    freertos
    Adafruit_GFX          # Adafruit GFX graphics library
    Adafruit_EPD          # Adafruit EPD e-ink display library
    Adafruit_BusIO_ESPIDF # Adafruit BusIO ESP-IDF native implementation (I2C + SPI)
    Adafruit_SH1106_ESPIDF # SH1106 OLED driver (for future OLED support)
    fatfs                 # FAT filesystem support
    esp_driver_sdspi      # SD card SPI driver
    vfs                   # Virtual filesystem
)

# =============================================================================
# Component Registration
# =============================================================================
# APP_SOURCE_FILE is relative to project root (e.g., "main/main.cpp")
# Since we're in the main/ directory, we need to extract just the filename
# Remove the "main/" prefix if present
string(REPLACE "main/" "" MAIN_SOURCE_FILE "${APP_SOURCE_FILE}")
# If no prefix was found, use the filename directly
get_filename_component(MAIN_SOURCE_FILE "${MAIN_SOURCE_FILE}" NAME)
set(COMPONENT_SRCS "${MAIN_SOURCE_FILE}")

# Add slideshow source files
if(APP_TYPE STREQUAL "ui_slideshow")
    list(APPEND COMPONENT_SRCS
        "button.cpp"
        "sd_card.cpp"
        "image_loader.cpp"
        "slideshow.cpp"
    )
endif()

idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${MAIN_INCLUDE_DIRS}
    REQUIRES ${MAIN_REQUIRES}
)

# =============================================================================
# Compiler Configuration
# =============================================================================
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# =============================================================================
# Build Type Specific Compiler Flags
# =============================================================================
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
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    "EXAMPLE_TYPE_${APP_TYPE}=1"
)

