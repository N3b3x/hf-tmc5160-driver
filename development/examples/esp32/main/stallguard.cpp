/**
 * @file stallguard.cpp
 * @brief StallGuard2 stall detection example for TMC5160 stepper motor driver
 *
 * This example demonstrates StallGuard2 stall detection and handling.
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

static const char *TAG = "StallGuard";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 StallGuard2 Stall Detection Example");

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

  // Configure StallGuard2
  tmc5160::StallGuardConfig sg_cfg{};
  sg_cfg.sgt = 0;   // Threshold (tune for your motor - lower = more sensitive)
  sg_cfg.semin = 0; // Minimum SG value
  sg_cfg.semax = 0; // Hysteresis
  sg_cfg.sfilt = false; // Filter disabled

  if (!driver.diagnostics.ConfigureStallGuard(sg_cfg)) {
    ESP_LOGE(TAG, "Failed to configure StallGuard2");
    return;
  }

  ESP_LOGI(TAG, "StallGuard2 configured: threshold=%d", sg_cfg.sgt);

  // Set velocity mode
  driver.rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
  driver.rampControl.SetMaxSpeed(500.0f);
  driver.rampControl.SetAcceleration(200.0f);

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled, monitoring StallGuard2 value");
  ESP_LOGI(TAG, "Stall will be detected when SG value drops below threshold");

  // Monitor StallGuard value
  const uint16_t stall_threshold = 100; // Adjust based on your motor
  int check_count = 0;

  while (check_count < 1000) {     // Monitor for up to 10 seconds
    vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms

    uint16_t sg_value = driver.diagnostics.GetStallGuard();

    if (check_count % 100 == 0) { // Log every second
      ESP_LOGI(TAG, "StallGuard value: %u", sg_value);
    }

    if (sg_value < stall_threshold) {
      ESP_LOGW(TAG, "Stall detected! SG value: %u (threshold: %u)", sg_value,
               stall_threshold);
      driver.rampControl.Stop();
      ESP_LOGI(TAG, "Motor stopped due to stall detection");
      break;
    }

    check_count++;
  }

  // Disable motor
  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Motor disabled");
}
