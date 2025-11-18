/**
 * @file motor_setup_from_spec.cpp
 * @brief Motor setup from specifications example for TMC5160
 *
 * This example demonstrates how to setup a motor using high-level specifications
 * instead of manually calculating driver parameters.
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

static const char *TAG = "MotorSetupFromSpec";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Motor Setup from Specifications Example");

  // Create SPI communication interface
  Esp32SPI spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
               GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15, 4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC5160 driver instance
  tmc5160::TMC5160 driver(spi);

  // Define motor specifications (NEMA 17 example)
  tmc5160::MotorSpec motor_spec{};
  motor_spec.steps_per_rev = 200;           // 1.8° per step
  motor_spec.rated_current_ma = 1500;      // 1.5A rated current
  motor_spec.rated_voltage_mv = 12000;      // 12V rated voltage
  motor_spec.winding_resistance_mohm = 3200; // 3.2Ω per phase
  motor_spec.winding_inductance_uh = 2800;   // 2.8mH per phase

  ESP_LOGI(TAG, "Motor Specifications:");
  ESP_LOGI(TAG, "  Steps per rev: %u", motor_spec.steps_per_rev);
  ESP_LOGI(TAG, "  Rated current: %u mA", motor_spec.rated_current_ma);
  ESP_LOGI(TAG, "  Rated voltage: %u mV", motor_spec.rated_voltage_mv);
  ESP_LOGI(TAG, "  Winding resistance: %u mΩ", motor_spec.winding_resistance_mohm);
  ESP_LOGI(TAG, "  Winding inductance: %u μH", motor_spec.winding_inductance_uh);

  // Define mechanical system (lead screw example)
  tmc5160::MechanicalSystem mech_system{};
  mech_system.system_type = tmc5160::MechanicalSystemType::LeadScrew;
  mech_system.lead_screw_pitch_mm = 2.0f; // 2mm pitch

  ESP_LOGI(TAG, "Mechanical System: Lead Screw, Pitch: %.2f mm",
           mech_system.lead_screw_pitch_mm);

  // Initialize driver with basic config
  tmc5160::DriverConfig cfg{};
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  // Setup motor from specifications
  ESP_LOGI(TAG, "Setting up motor from specifications...");
  if (!driver.motorControl.SetupMotorFromSpec(motor_spec, &mech_system)) {
    ESP_LOGE(TAG, "Failed to setup motor from specifications");
    return;
  }

  ESP_LOGI(TAG, "Motor setup completed successfully");

  // Configure motion parameters
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetMaxSpeedRpm(100.0f, motor_spec.steps_per_rev);
  driver.rampControl.SetAcceleration(500.0f);
  driver.rampControl.SetRampSpeeds(0.0f, 0.1f, 0.0f);

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled");

  // Move to a target position (using physical units)
  float target_mm = 10.0f;
  ESP_LOGI(TAG, "Moving to position: %.2f mm", target_mm);
  driver.rampControl.SetTargetPositionMm(target_mm, motor_spec.steps_per_rev,
                                         mech_system.lead_screw_pitch_mm);

  // Wait for completion
  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));
    int32_t steps = driver.rampControl.GetCurrentPosition();
    ESP_LOGI(TAG, "Current position: %d steps", steps);
  }

  ESP_LOGI(TAG, "Target reached! Example completed successfully");
}

