/**
 * @file config.hpp
 * @brief Configuration for E-Ink Slideshow Board
 * 
 * Hardware Configuration:
 * - Display: Adafruit 2.9" ThinkInk FeatherWing Tricolor E-Ink Display
 *   - Physical dimensions: 296x128 pixels (horizontal)
 *   - Portrait mode: 128x296 pixels (rotated)
 *   - Controller: IL0373
 *   - Colors: Black, White, Red (tricolor)
 * 
 * - SD Card: MicroSD card slot (SPI interface)
 *   - Format: FAT32
 *   - Interface: SPI
 * 
 * - Buttons: Three buttons for navigation
 *   - UP: Previous image
 *   - SELECT: Toggle auto-advance / Favorite
 *   - DOWN: Next image
 * 
 * IMPORTANT: Configure the GPIO pin numbers below to match your hardware wiring!
 */

#pragma once

#include <cstdint>
#include "driver/gpio.h"

// ------------- E-INK DISPLAY CONFIG -------------

// E-ink display pins (Adafruit ThinkInk FeatherWing)
static constexpr int EINK_DC_PIN    = 12;   // Data/Command pin
static constexpr int EINK_RESET_PIN = 13;   // Reset pin
static constexpr int EINK_CS_PIN    = 14;   // Chip Select pin
static constexpr int EINK_BUSY_PIN  = 15;   // Busy pin (optional, for status checking)

// SPI bus pins for E-ink display
static constexpr gpio_num_t SPI_SCK_PIN  = GPIO_NUM_18;  // SPI Clock pin
static constexpr gpio_num_t SPI_MOSI_PIN = GPIO_NUM_23;  // SPI MOSI (Master Out Slave In)
static constexpr gpio_num_t SPI_MISO_PIN = GPIO_NUM_19;  // SPI MISO (Master In Slave Out)

// Display dimensions (portrait mode after rotation)
static constexpr uint16_t DISPLAY_WIDTH = 128;   // Portrait width
static constexpr uint16_t DISPLAY_HEIGHT = 296;  // Portrait height

// ------------- SD CARD CONFIG -------------

// SD card SPI pins (can share SPI bus with display, but needs separate CS)
static constexpr gpio_num_t SD_CS_PIN = GPIO_NUM_5;   // SD card chip select
// SD card uses same SPI bus (SCK, MOSI, MISO) as e-ink display

// SD card mount point
static constexpr const char* SD_MOUNT_POINT = "/sdcard";

// Image directory on SD card
static constexpr const char* IMAGE_DIRECTORY = "/sdcard/images";

// ------------- BUTTON CONFIG -------------

// Button GPIOs (must be RTC-capable if you want them as deep sleep wake sources)
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_9;   // Previous image
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_10;  // Toggle auto-advance / Favorite
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_11;  // Next image

// Button debounce time
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

// ------------- SLIDESHOW SETTINGS -------------

// Auto-advance delay (seconds)
static constexpr uint32_t AUTO_ADVANCE_DELAY_SEC = 10;

// Inactivity timeout before deep sleep (seconds)
static constexpr uint32_t INACTIVITY_TIMEOUT_SEC = 300;  // 5 minutes

// Maximum number of images to cache in memory
static constexpr size_t MAX_IMAGE_FILES = 100;

// Supported image file extensions
static constexpr const char* IMAGE_EXTENSIONS[] = { ".bmp", ".BMP" };
static constexpr size_t NUM_IMAGE_EXTENSIONS = sizeof(IMAGE_EXTENSIONS) / sizeof(IMAGE_EXTENSIONS[0]);

