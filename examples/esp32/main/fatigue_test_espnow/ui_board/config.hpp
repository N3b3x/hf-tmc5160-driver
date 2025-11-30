/**
 * @file config.hpp
 * @brief Configuration for UI board (remote controller)
 */

#pragma once

#include <cstdint>
#include "driver/gpio.h"

// ------------- GPIO CONFIG (ADJUST FOR YOUR BOARD) -------------

// Button GPIOs (must be RTC-capable if you want them as deep sleep wake sources)
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_4;
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_6;

// E-ink display pins (placeholder; adjust to your wiring)
// If using Adafruit ThinkInk FeatherWing, map these to your Feather/Beetle pins.
static constexpr int EINK_DC_PIN   = 10;
static constexpr int EINK_RESET_PIN = 11;
static constexpr int EINK_CS_PIN    = 9;
static constexpr int EINK_BUSY_PIN  = 12;

// ------------- ESPNOW CONFIG -------------

// Placeholder MAC of the test unit (receiver). Fill with real MAC later.
// This should be set to the MAC address of your test unit ESP32
static constexpr uint8_t TEST_UNIT_MAC[6] = { 0x24, 0x6F, 0x28, 0x00, 0x00, 0x01 };

// ------------- APP LOGIC -------------

// Inactivity timeout before going to deep sleep (seconds)
static constexpr uint32_t INACTIVITY_TIMEOUT_SEC = 60;
