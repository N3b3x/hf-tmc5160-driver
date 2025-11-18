/**
 * @file encoder.cpp
 * @brief Encoder closed-loop control example for TMC5160 stepper motor driver
 *
 * This example demonstrates encoder-based closed-loop control with deviation
 * detection.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor with encoder connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5
 * - Control: EN=2, DIR=4, STEP=15
 * - Encoder: A, B, N signals connected to TMC5160 encoder inputs
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Encoder";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Encoder Closed-Loop Control Example");

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

  // Configure encoder
  tmc5160::EncoderConfig enc_cfg{};
  enc_cfg.enc_sel_decimal = false; // Binary mode
  enc_cfg.pol_n = true;            // N channel active high
  enc_cfg.ignore_ab = true;        // Ignore A/B polarity

  if (!driver.encoder.Configure(enc_cfg)) {
    ESP_LOGE(TAG, "Failed to configure encoder");
    return;
  }

  // Set encoder resolution: 200 steps/rev motor, 1000 pulses/rev encoder
  if (!driver.encoder.SetResolution(200, 1000, false)) {
    ESP_LOGW(TAG, "Encoder resolution set with approximation");
  } else {
    ESP_LOGI(TAG, "Encoder resolution set exactly");
  }

  // Set allowed deviation (10 steps tolerance)
  driver.encoder.SetAllowedDeviation(10);

  ESP_LOGI(TAG,
           "Encoder configured: 200 steps/rev motor, 1000 pulses/rev encoder");

  // Configure positioning mode
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(1000);
  driver.rampControl.SetMaxSpeed(1000.0f);
  driver.rampControl.SetAcceleration(500.0f);

  // Enable motor
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }

  ESP_LOGI(TAG, "Motor enabled, starting motion with encoder feedback");

  // Monitor encoder deviation during motion
  while (!driver.rampControl.IsTargetReached()) {
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check for encoder deviation
    if (driver.encoder.IsDeviationDetected()) {
      ESP_LOGW(TAG, "Encoder deviation detected - step loss may have occurred");
      driver.encoder.ClearDeviationFlag();
    }

    // Read encoder position
    int32_t enc_pos = driver.encoder.GetPosition();
    int32_t motor_pos = driver.rampControl.GetCurrentPosition();

    ESP_LOGI(TAG, "Motor position: %ld, Encoder position: %ld", motor_pos,
             enc_pos);
  }

  ESP_LOGI(TAG, "Target position reached");

  // Final encoder check
  int32_t final_motor_pos = driver.rampControl.GetCurrentPosition();
  int32_t final_enc_pos = driver.encoder.GetPosition();
  ESP_LOGI(TAG, "Final motor position: %ld, Final encoder position: %ld",
           final_motor_pos, final_enc_pos);

  // Disable motor
  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Motor disabled");
}
