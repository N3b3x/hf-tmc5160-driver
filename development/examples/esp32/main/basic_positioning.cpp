/**
 * @file basic_positioning.cpp
 * @brief Basic positioning mode example for TMC5160 stepper motor driver
 *
 * This example demonstrates basic stepper motor control in positioning mode.
 * The motor moves to a target position with configurable speed and
 * acceleration.
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

static const char *TAG = "BasicPositioning";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Basic Positioning Example");

  // Create SPI communication interface
  // Modify pin numbers to match your hardware
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
  cfg.motor.irun = 20;          // Run current (0-31, recommended 16-31)
  cfg.motor.ihold = 10;         // Hold current (0-31, typically 70% of irun)
  cfg.motor.global_scaler = 32; // Global current scaler (32-256)
  cfg.chopper.toff = 5;         // Chopper off time
  cfg.chopper.mres = 4;         // 16 microsteps
  cfg.chopper.intpol = true;    // Enable interpolation

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Configure ramp control
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(1000); // Move 1000 steps
  driver.rampControl.SetMaxSpeed(1000.0f);    // 1000 steps/s
  driver.rampControl.SetAcceleration(500.0f); // 500 steps/s²
  driver.rampControl.SetRampSpeeds(0.0f, 0.1f,
                                   0.0f); // start, stop, transition speeds

  ESP_LOGI(TAG, "Ramp control configured: target=1000, max_speed=1000 steps/s, "
                "accel=500 steps/s²");

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled, starting motion...");

  // Wait for target position to be reached
  int timeout_count = 0;
  const int max_timeout = 1000; // 10 seconds timeout (100ms * 1000)

  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms

    // Get current position and speed
    int32_t position = driver.rampControl.GetCurrentPosition();
    float speed = driver.rampControl.GetCurrentSpeed();

    if (timeout_count % 10 == 0) { // Log every second
      ESP_LOGI(TAG, "Position: %ld steps, Speed: %.2f steps/s", position,
               speed);
    }

    timeout_count++;
    if (timeout_count > max_timeout) {
      ESP_LOGW(TAG, "Timeout waiting for target position");
      break;
    }
  }

  if (driver.rampControl.IsTargetReached()) {
    ESP_LOGI(TAG, "Target position reached!");
    int32_t final_position = driver.rampControl.GetCurrentPosition();
    ESP_LOGI(TAG, "Final position: %ld steps", final_position);
  }

  // Disable motor
  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Motor disabled");

  // Example complete
  ESP_LOGI(TAG, "Example completed successfully");
}
