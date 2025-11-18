/**
 * @file stealthchop.cpp
 * @brief StealthChop silent operation example for TMC5160 stepper motor driver
 *
 * This example demonstrates silent operation using stealthChop mode.
 * The motor runs silently at low speeds using PWM mode.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5
 * - Control: EN=2, DIR=4, STEP=15
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "StealthChop";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 StealthChop Silent Operation Example");

  // Create SPI communication interface
  Esp32SPI spi(SPI2_HOST,
               GPIO_NUM_23, // MOSI
               GPIO_NUM_19, // MISO
               GPIO_NUM_18, // SCLK
               GPIO_NUM_5,  // CS
               GPIO_NUM_2,  // EN
               GPIO_NUM_4,  // DIR
               GPIO_NUM_15, // STEP
               4000000);    // 4 MHz SPI clock

  // Initialize SPI interface
  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC5160 driver instance
  tmc5160::TMC5160 driver(spi);

  // Configure driver for stealthChop
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;
  cfg.chopper.mres = 4; // 16 microsteps for smooth operation
  cfg.stealthchop.pwm_autoscale = true;
  cfg.stealthchop.pwm_autograd = true;

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Configure stealthChop thresholds
  // Below 100 steps/s: stealthChop mode (silent)
  // Above 100 steps/s: spreadCycle mode (more torque)
  driver.motorControl.SetModeChangeSpeeds(100.0f, 0.0f, 0.0f);

  // Configure positioning mode with low speed for stealthChop
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(1000);
  driver.rampControl.SetMaxSpeed(50.0f); // Low speed = stealthChop
  driver.rampControl.SetAcceleration(100.0f);

  ESP_LOGI(TAG, "StealthChop configured: speed=50 steps/s (silent mode)");

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled, running silently in stealthChop mode");

  // Wait for target reached
  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(TAG, "Target position reached");
  ESP_LOGI(TAG, "Motor ran silently using stealthChop PWM mode");

  // Disable motor
  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Motor disabled");
}
