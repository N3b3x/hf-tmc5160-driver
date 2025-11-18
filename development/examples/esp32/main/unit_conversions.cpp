/**
 * @file unit_conversions.cpp
 * @brief Unit conversion example for TMC5160 stepper motor driver
 *
 * This example demonstrates how to use unit conversion functions to work with
 * physical units (millimeters, RPM) instead of raw steps.
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
#include "../../../inc/tmc5160_units.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "UnitConversions";

// Motor and mechanical system parameters
constexpr uint16_t STEPS_PER_REV = 200;      // 1.8° stepper motor
constexpr float LEAD_SCREW_PITCH_MM = 2.0f;  // 2mm pitch lead screw

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Unit Conversions Example");

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

  // Configure ramp control
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);

  // Example 1: Set target position in millimeters
  float target_mm = 10.0f; // Move 10mm
  ESP_LOGI(TAG, "Moving to position: %.2f mm", target_mm);
  driver.rampControl.SetTargetPositionMm(target_mm, STEPS_PER_REV,
                                          LEAD_SCREW_PITCH_MM);

  // Example 2: Set maximum speed in RPM
  float max_rpm = 100.0f; // 100 RPM
  ESP_LOGI(TAG, "Setting maximum speed: %.2f RPM", max_rpm);
  driver.rampControl.SetMaxSpeedRpm(max_rpm, STEPS_PER_REV);

  // Example 3: Set acceleration in mm/s²
  float accel_mm_per_sec2 = 50.0f;
  float accel_steps_per_sec2 = tmc5160::AccelerationMmToSteps(
      accel_mm_per_sec2, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
  ESP_LOGI(TAG, "Setting acceleration: %.2f mm/s² (%.2f steps/s²)",
           accel_mm_per_sec2, accel_steps_per_sec2);
  driver.rampControl.SetAcceleration(accel_steps_per_sec2);

  // Set ramp speeds
  driver.rampControl.SetRampSpeeds(0.0f, 0.1f, 0.0f);

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled, starting motion...");

  // Monitor position in millimeters
  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));

    int32_t steps = driver.rampControl.GetCurrentPosition();
    float mm = tmc5160::StepsToMm(steps, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
    float speed_steps_per_sec = driver.rampControl.GetCurrentSpeed();
    float rpm = tmc5160::StepsPerSecToRpm(speed_steps_per_sec, STEPS_PER_REV);

    ESP_LOGI(TAG, "Position: %.2f mm (%d steps), Speed: %.2f RPM (%.2f steps/s)",
             mm, steps, rpm, speed_steps_per_sec);
  }

  ESP_LOGI(TAG, "Target position reached!");

  // Example 4: Convert degrees to steps
  float angle_degrees = 90.0f;
  int32_t steps_for_90deg =
      tmc5160::DegreesToSteps(angle_degrees, STEPS_PER_REV);
  ESP_LOGI(TAG, "90 degrees = %d steps", steps_for_90deg);

  // Example 5: Convert belt drive distance
  uint32_t belt_teeth = 100;
  uint16_t pulley_teeth = 20;
  int32_t steps_for_belt =
      tmc5160::BeltTeethToSteps(belt_teeth, STEPS_PER_REV, pulley_teeth);
  ESP_LOGI(TAG, "%u belt teeth = %d steps (with %u-tooth pulley)", belt_teeth,
           steps_for_belt, pulley_teeth);

  ESP_LOGI(TAG, "Example completed successfully");
}

