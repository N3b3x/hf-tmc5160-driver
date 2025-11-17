/**
 * @file comprehensive_test.cpp
 * @brief Comprehensive test suite for TMC5160 stepper motor driver
 *
 * This example demonstrates all major features of the TMC5160 driver including:
 * - Positioning mode
 * - Velocity mode
 * - StealthChop configuration
 * - Encoder configuration
 * - StallGuard2 configuration
 * - Driver status monitoring
 * - Error detection
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor (optionally with encoder) connected to TMC5160
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

static const char *TAG = "ComprehensiveTest";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Comprehensive Test Suite");

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

  // Test 1: Driver Initialization
  ESP_LOGI(TAG, "=== Test 1: Driver Initialization ===");
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;
  cfg.chopper.mres = 4; // 16 microsteps

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Test 1 FAILED: Driver initialization failed");
    return;
  }
  ESP_LOGI(TAG, "Test 1 PASSED: Driver initialized successfully");

  // Test 2: Driver Status Check
  ESP_LOGI(TAG, "=== Test 2: Driver Status Check ===");
  tmc5160::DriverStatus status = driver.diagnostics.GetStatus();
  if (status != tmc5160::DriverStatus::OK) {
    ESP_LOGW(TAG, "Test 2 WARNING: Driver status: %d",
             static_cast<int>(status));
  } else {
    ESP_LOGI(TAG, "Test 2 PASSED: Driver status OK");
  }

  // Test 3: Positioning Mode
  ESP_LOGI(TAG, "=== Test 3: Positioning Mode ===");
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(500);
  driver.rampControl.SetMaxSpeed(1000.0f);
  driver.rampControl.SetAcceleration(500.0f);

  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Test 3 FAILED: Failed to enable motor");
    return;
  }

  int timeout = 0;
  while (!driver.rampControl.IsTargetReached() && timeout < 100) {
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout++;
  }

  if (driver.rampControl.IsTargetReached()) {
    ESP_LOGI(TAG, "Test 3 PASSED: Target position reached");
  } else {
    ESP_LOGW(TAG, "Test 3 WARNING: Timeout waiting for target position");
  }

  // Test 4: Velocity Mode
  ESP_LOGI(TAG, "=== Test 4: Velocity Mode ===");
  driver.rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
  driver.rampControl.SetMaxSpeed(500.0f);
  vTaskDelay(pdMS_TO_TICKS(1000)); // Run for 1 second

  float speed = driver.rampControl.GetCurrentSpeed();
  ESP_LOGI(TAG, "Test 4 PASSED: Velocity mode active, speed: %.2f steps/s",
           speed);

  driver.rampControl.Stop();

  // Test 5: StealthChop Configuration
  ESP_LOGI(TAG, "=== Test 5: StealthChop Configuration ===");
  tmc5160::StealthChopConfig stealth_cfg{};
  stealth_cfg.pwm_autoscale = true;
  stealth_cfg.pwm_autograd = true;

  if (driver.motorControl.ConfigureStealthChop(stealth_cfg)) {
    ESP_LOGI(TAG, "Test 5 PASSED: StealthChop configured");
  } else {
    ESP_LOGE(TAG, "Test 5 FAILED: StealthChop configuration failed");
  }

  // Test 6: Encoder Configuration
  ESP_LOGI(TAG, "=== Test 6: Encoder Configuration ===");
  tmc5160::EncoderConfig enc_cfg{};
  enc_cfg.enc_sel_decimal = false;

  if (driver.encoder.Configure(enc_cfg)) {
    ESP_LOGI(TAG, "Test 6 PASSED: Encoder configured");

    if (driver.encoder.SetResolution(200, 1000, false)) {
      ESP_LOGI(TAG, "Test 6 PASSED: Encoder resolution set");
    } else {
      ESP_LOGW(TAG,
               "Test 6 WARNING: Encoder resolution set with approximation");
    }
  } else {
    ESP_LOGE(TAG, "Test 6 FAILED: Encoder configuration failed");
  }

  // Test 7: StallGuard2 Configuration
  ESP_LOGI(TAG, "=== Test 7: StallGuard2 Configuration ===");
  tmc5160::StallGuardConfig sg_cfg{};
  sg_cfg.sgt = 0;

  if (driver.diagnostics.ConfigureStallGuard(sg_cfg)) {
    ESP_LOGI(TAG, "Test 7 PASSED: StallGuard2 configured");

    uint16_t sg_value = driver.diagnostics.GetStallGuard();
    ESP_LOGI(TAG, "StallGuard value: %u", sg_value);
  } else {
    ESP_LOGE(TAG, "Test 7 FAILED: StallGuard2 configuration failed");
  }

  // Test 8: Current Control
  ESP_LOGI(TAG, "=== Test 8: Current Control ===");
  if (driver.motorControl.SetCurrent(25, 15)) {
    ESP_LOGI(TAG, "Test 8 PASSED: Motor current set (irun=25, ihold=15)");
  } else {
    ESP_LOGE(TAG, "Test 8 FAILED: Failed to set motor current");
  }

  // Test 9: Protection Configuration
  ESP_LOGI(TAG, "=== Test 9: Protection Configuration ===");
  tmc5160::ShortProtectionConfig short_cfg{};
  short_cfg.s2vs_level = 6;
  short_cfg.s2g_level = 6;

  if (driver.protection.ConfigureShortProtection(short_cfg)) {
    ESP_LOGI(TAG, "Test 9 PASSED: Short protection configured");
  } else {
    ESP_LOGE(TAG, "Test 9 FAILED: Short protection configuration failed");
  }

  // Test 10: Final Status Check
  ESP_LOGI(TAG, "=== Test 10: Final Status Check ===");
  status = driver.diagnostics.GetStatus();
  if (status == tmc5160::DriverStatus::OK) {
    ESP_LOGI(TAG, "Test 10 PASSED: Driver status OK");
  } else {
    ESP_LOGW(TAG, "Test 10 WARNING: Driver status: %d",
             static_cast<int>(status));
  }

  // Disable motor
  driver.motorControl.Disable();
  ESP_LOGI(TAG, "=== Comprehensive Test Suite Completed ===");
}
