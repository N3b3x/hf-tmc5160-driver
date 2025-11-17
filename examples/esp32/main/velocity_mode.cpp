/**
 * @file velocity_mode.cpp
 * @brief Velocity mode example for TMC5160 stepper motor driver
 *
 * This example demonstrates velocity mode operation where the motor runs
 * continuously at a specified speed.
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

static const char *TAG = "VelocityMode";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Velocity Mode Example");

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

  // Configure driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Set velocity mode
  driver.rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
  driver.rampControl.SetMaxSpeed(500.0f); // 500 steps/s forward
  driver.rampControl.SetAcceleration(200.0f);

  ESP_LOGI(
      TAG,
      "Velocity mode configured: speed=500 steps/s, acceleration=200 steps/s²");

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled, running continuously at 500 steps/s");
  ESP_LOGI(TAG, "Motor will run until stopped or disabled");

  // Monitor speed
  for (int i = 0; i < 100; ++i) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second

    float speed = driver.rampControl.GetCurrentSpeed();
    ESP_LOGI(TAG, "Current speed: %.2f steps/s", speed);
  }

  // Stop motor
  driver.rampControl.Stop();
  ESP_LOGI(TAG, "Motor stopped");

  // Disable motor
  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Motor disabled");
}
