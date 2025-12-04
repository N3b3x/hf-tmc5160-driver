/**
 * @file config.hpp
 * @brief Configuration for UI board (remote controller)
 * 
 * Hardware Configuration:
 * - Display: 1.3" SH1106 OLED Display (I2C)
 *   - Resolution: 128x64 pixels
 *   - Interface: I2C (typically 0x3C or 0x3D address)
 *   - Driver: SH1106
 * 
 * - Input: EC11 Rotary Encoder Module
 *   - TRA (Phase A): Quadrature encoder output A
 *   - TRB (Phase B): Quadrature encoder output B
 *   - PSH (Push button): Encoder integrated switch
 *   - Pulses per revolution: 20
 * 
 * - Physical Buttons:
 *   - BACK: Hardware button for navigation back
 *   - CONFIRM: Hardware button for critical action confirmation
 * 
 * - Legacy Buttons (for e-ink slideshow application):
 *   - UP, SELECT, DOWN: Repurposed for slideshow navigation
 * 
 * IMPORTANT: Configure the GPIO pin numbers below to match your hardware wiring!
 * 
 * ESP32C6 Beetle Board Pin Reference:
 * - I2C0: Default SDA=GPIO21, SCL=GPIO22 (can be remapped)
 * - GPIO 4, 5, 6: Available for buttons/encoder (verify board-specific restrictions)
 * - GPIO 7, 8, 9, 10: Available for additional functions
 */

#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// ------------- OLED DISPLAY CONFIG (SH1106, I2C) -------------

// I2C bus configuration for OLED display
static constexpr i2c_port_t OLED_I2C_PORT = I2C_NUM_0;  // Use I2C0
static constexpr gpio_num_t OLED_SDA_PIN  = GPIO_NUM_21;  // I2C Data line (SDA)
static constexpr gpio_num_t OLED_SCL_PIN  = GPIO_NUM_22;  // I2C Clock line (SCL)
static constexpr uint32_t   OLED_I2C_FREQ = 400000;      // I2C frequency: 400kHz (fast mode)
static constexpr uint8_t    OLED_I2C_ADDR = 0x3C;        // SH1106 I2C address (try 0x3D if 0x3C doesn't work)

// OLED display dimensions
static constexpr uint16_t OLED_WIDTH  = 128;
static constexpr uint16_t OLED_HEIGHT = 64;

// ------------- EC11 ROTARY ENCODER CONFIG -------------

// EC11 encoder pins (must support GPIO interrupts)
static constexpr gpio_num_t ENCODER_TRA_PIN = GPIO_NUM_4;   // Phase A (CLK) - quadrature input
static constexpr gpio_num_t ENCODER_TRB_PIN = GPIO_NUM_5;   // Phase B (DT) - quadrature input
static constexpr gpio_num_t ENCODER_PSH_PIN = GPIO_NUM_6;    // Push button (SW) - encoder click

// Encoder configuration
static constexpr uint8_t ENCODER_PULSES_PER_REV = 20;      // 20 pulses per full rotation
static constexpr uint32_t ENCODER_DEBOUNCE_MS = 50;         // Debounce time for encoder rotation
static constexpr uint32_t ENCODER_BUTTON_DEBOUNCE_MS = 100; // Debounce time for encoder button

// ------------- PHYSICAL BUTTONS CONFIG -------------

// Physical buttons on the OLED module board
static constexpr gpio_num_t BTN_BACK_GPIO    = GPIO_NUM_7;   // BACK button (navigate back)
static constexpr gpio_num_t BTN_CONFIRM_GPIO = GPIO_NUM_8;   // CONFIRM button (critical actions)

// Button debounce configuration
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;           // Debounce time for physical buttons

// ------------- LEGACY BUTTONS (for e-ink slideshow) -------------

// Legacy button GPIOs (repurposed for e-ink slideshow application)
// These are kept for the separate e-ink slideshow app
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_9;   // PREV button for slideshow
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_10;  // LOVE button for slideshow
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_11;  // NEXT button for slideshow

// ------------- E-INK DISPLAY CONFIG (for slideshow app) -------------

// E-ink display pins (Adafruit ThinkInk FeatherWing)
// Used only in the separate e-ink slideshow application
static constexpr int EINK_DC_PIN    = 12;   // Data/Command pin
static constexpr int EINK_RESET_PIN = 13;   // Reset pin
static constexpr int EINK_CS_PIN    = 14;   // Chip Select pin
static constexpr int EINK_BUSY_PIN  = 15;   // Busy pin (optional, for status checking)

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
