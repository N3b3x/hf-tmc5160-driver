/**
 * @file sensorless_homing.cpp
 * @brief Sensorless homing example for TMC5160
 *
 * This example demonstrates sensorless homing using StallGuard2 stall detection.
 * The motor moves toward a mechanical stop until stall is detected.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 * - Mechanical stop for homing
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

static const char *TAG = "SensorlessHoming";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Sensorless Homing Example");

  // Create SPI communication interface
  Esp32SPI spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
               GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15, 4000000);

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
  cfg.motor.global_scaler = 32;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Configure StallGuard2 for homing
  tmc5160::StallGuardConfig sg_config{};
  sg_config.sgt = -10;    // Stall threshold (tune for your motor)
  sg_config.sfilt = true; // Enable filter for stability
  sg_config.semin = 2;   // Minimum SG value for CoolStep
  sg_config.semax = 5;   // Hysteresis for CoolStep

  ESP_LOGI(TAG, "Configuring StallGuard2: sgt=%d", sg_config.sgt);
  if (!driver.diagnostics.ConfigureStallGuard(sg_config)) {
    ESP_LOGE(TAG, "Failed to configure StallGuard2");
    return;
  }

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled");

  // Perform sensorless homing in negative direction
  ESP_LOGI(TAG, "Starting sensorless homing...");
  int32_t home_position = 0;
  float search_speed = 500.0f; // steps/s

  if (!driver.diagnostics.PerformSensorlessHoming(false, // negative direction
                                                    -10,   // stall threshold
                                                    search_speed, home_position)) {
    ESP_LOGE(TAG, "Sensorless homing failed");
    return;
  }

  ESP_LOGI(TAG, "Homing completed! Final position: %d steps", home_position);

  // Set current position as home (0)
  driver.rampControl.SetCurrentPosition(0);
  ESP_LOGI(TAG, "Home position set to 0");

  // Optional: Move away from stop slightly
  ESP_LOGI(TAG, "Moving 100 steps away from stop...");
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(100);
  driver.rampControl.SetMaxSpeed(500.0f);
  driver.rampControl.SetAcceleration(1000.0f);

  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Set new zero position
  driver.rampControl.SetCurrentPosition(0);
  ESP_LOGI(TAG, "New home position set (100 steps away from mechanical stop)");

  // Monitor StallGuard value
  ESP_LOGI(TAG, "Monitoring StallGuard2 value...");
  for (int i = 0; i < 10; i++) {
    uint16_t sg_value = driver.diagnostics.GetStallGuard();
    ESP_LOGI(TAG, "StallGuard2 value: %u", sg_value);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  ESP_LOGI(TAG, "Sensorless homing example completed successfully");
}

