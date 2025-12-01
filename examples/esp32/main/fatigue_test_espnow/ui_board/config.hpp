/**
 * @file config.hpp
 * @brief Configuration for UI board (remote controller)
 * 
 * Hardware Configuration:
 * - Display: Adafruit 2.9" ThinkInk FeatherWing Tricolor E-Ink Display
 *   - Physical dimensions: 296x128 pixels (horizontal)
 *   - Portrait mode: 128x296 pixels (rotated)
 *   - Controller: IL0373
 *   - Colors: Black, White, Red (tricolor)
 * 
 * - Buttons: Three buttons on the side of the display
 *   - Top button: UP (navigate up, increase value)
 *   - Middle button: SELECT (confirm, enter menu)
 *   - Bottom button: DOWN (navigate down, decrease value, stop)
 * 
 * IMPORTANT: Configure the GPIO pin numbers below to match your hardware wiring!
 */

#pragma once

#include <cstdint>
#include "driver/gpio.h"

// ------------- GPIO CONFIG (ADJUST FOR YOUR BOARD) -------------

// Button GPIOs (must be RTC-capable if you want them as deep sleep wake sources)
// Button layout: Top=UP, Middle=SELECT, Bottom=DOWN
// Configure these pins to match your hardware wiring
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_4;   // Top button (UP)
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_5;   // Middle button (SELECT)
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_6;   // Bottom button (DOWN)

// E-ink display pins (Adafruit ThinkInk FeatherWing)
// Configure these pins to match your hardware wiring
static constexpr int EINK_DC_PIN    = 10;   // Data/Command pin
static constexpr int EINK_RESET_PIN = 11;   // Reset pin
static constexpr int EINK_CS_PIN    = 9;    // Chip Select pin
static constexpr int EINK_BUSY_PIN  = 12;   // Busy pin (optional, for status checking)

// SPI bus pins for E-ink display (must match your hardware wiring)
// These are the SPI bus pins (SCK, MOSI, MISO) - CS is defined above as EINK_CS_PIN
static constexpr gpio_num_t SPI_SCK_PIN  = GPIO_NUM_18;  // SPI Clock pin
static constexpr gpio_num_t SPI_MOSI_PIN = GPIO_NUM_23;  // SPI MOSI (Master Out Slave In)
static constexpr gpio_num_t SPI_MISO_PIN = GPIO_NUM_19;  // SPI MISO (Master In Slave Out)

// ------------- ESPNOW CONFIG -------------

// Placeholder MAC of the test unit (receiver). Fill with real MAC later.
// This should be set to the MAC address of your test unit ESP32
static constexpr uint8_t TEST_UNIT_MAC[6] = { 0x24, 0x6F, 0x28, 0x00, 0x00, 0x01 };

// ------------- APP LOGIC -------------

// Inactivity timeout before going to deep sleep (seconds)
static constexpr uint32_t INACTIVITY_TIMEOUT_SEC = 60;
