/**
 * @file stallguard_tuning.cpp
 * @brief Automatic StallGuard2 Tuning Tool
 *
 * This tool automatically tunes the StallGuard2 Threshold (SGT) for a specific
 * motor and velocity configuration. It implements the tuning algorithm described
 * in the datasheet and research papers:
 * 
 * 1. Moves the motor at a constant velocity
 * 2. Monitors the SG_RESULT (StallGuard value)
 * 3. Adjusts SGT until a stable non-zero SG_RESULT is obtained
 * 
 * USAGE:
 * 1. Ensure the motor is free to move (no load or minimal load).
 * 2. Run this tool.
 * 3. The tool will output the found optimal SGT value.
 * 4. Use this SGT value in your application.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp32_tmc5160_bus_config.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SGT_Tuning";

//=============================================================================
// MOTOR SELECTION
//=============================================================================
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;

// Tuning Parameters
static constexpr float TUNING_VELOCITY_STEPS_S = 30000.0f; // Target velocity for tuning
static constexpr float TUNING_ACCELERATION_STEPS_S2 = 3000.0f; // Acceleration/deceleration (realistic for stepper motors)

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting StallGuard2 Tuning Tool...");

  // 1. Initialize SPI Bus
  auto pin_config = tmc5160_test_config::GetDefaultPinConfig();
  Esp32SPI spi(tmc5160_test_config::SPI_HOST, pin_config, tmc5160_test_config::SPI_CLOCK_SPEED_HZ);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus");
    return;
  }

  // 2. Initialize Driver
  tmc5160::TMC5160<Esp32SPI> driver(spi);
  tmc5160::DriverConfig driver_config;

  // Configure based on selected motor
  if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
    driver_config.motor_spec.irun = Motor::IRUN;
    driver_config.motor_spec.ihold = Motor::IHOLD;
    driver_config.motor_spec.global_scaler = Motor::GLOBAL_SCALER;
    driver_config.chopper.toff = Motor::TOFF;
    driver_config.chopper.hend = Motor::HEND;
    driver_config.chopper.hstrt = Motor::HSTRT;
    driver_config.chopper.mres = Motor::MRES;
    
    // Set physical specs for unit conversions
    driver_config.motor_spec.steps_per_rev = Motor::MOTOR_FULL_STEPS;
    driver_config.motor_spec.rated_current_ma = Motor::RATED_CURRENT_MA;
    driver_config.mechanical.gear_ratio = Motor::GEAR_RATIO;
    
  } else {
    ESP_LOGE(TAG, "Unsupported motor type selected for this example");
    return;
  }

  if (!driver.Initialize(driver_config)) {
    ESP_LOGE(TAG, "Failed to initialize driver");
    return;
  }

  // 3. Verify Setup
  if (!driver.diagnostics.VerifySetup()) {
    ESP_LOGE(TAG, "Setup verification failed");
    return;
  }

  // 4. Enable Motor
  ESP_LOGI(TAG, "Enabling motor...");
  if (!driver.motorControl.Enable()) {
    ESP_LOGE(TAG, "Failed to enable motor");
    return;
  }
  
  // Ensure motor is stopped before starting
  driver.rampControl.Stop();
  vTaskDelay(pdMS_TO_TICKS(500));
  
  // 5. Configure SpreadCycle (Required for StallGuard2)
  // Disable stealthChop (PWM_CONF) or ensure we switch to SpreadCycle at velocity
  // For tuning, we want to force SpreadCycle.
  driver.motorControl.SetStealthChopEnabled(false);
  
  // Set velocity thresholds for StallGuard
  // TCOOLTHRS needs to be set such that StallGuard is active at tuning velocity
  // 1000 steps/s threshold for activation ensures StallGuard is active at 40k steps/s
  driver.motorControl.SetModeChangeSpeeds(100.0f, 1000.0f, 0.0f); // PWM_THRS, COOL_THRS, HIGH_THRS

  ESP_LOGI(TAG, "Starting Auto-Tuning Sequence...");
  ESP_LOGI(TAG, "Target Velocity: %.2f steps/s", TUNING_VELOCITY_STEPS_S);
  ESP_LOGI(TAG, "Acceleration: %.2f steps/s²", TUNING_ACCELERATION_STEPS_S2);
  
  int8_t optimal_sgt = 0;
  // TuneStallGuard with new signature: target_vel, result_sgt, min_sgt, max_sgt, accel, min_vel, max_vel, unit
  bool success = driver.diagnostics.TuneStallGuard(TUNING_VELOCITY_STEPS_S, optimal_sgt, -10, 63, 
                                                  TUNING_ACCELERATION_STEPS_S2, 0.0f, 0.0f, tmc5160::Unit::Steps);

  if (success) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "TUNING SUCCESSFUL");
    ESP_LOGI(TAG, "Optimal SGT: %d", optimal_sgt);
    ESP_LOGI(TAG, "==========================================");
    
    // Test run with found SGT
    ESP_LOGI(TAG, "Verifying with test run...");
    tmc5160::StallGuardConfig sg_config;
    sg_config.sgt = optimal_sgt;
    sg_config.sfilt = true; // Enable filter for verification/operation (reduces noise)
    driver.diagnostics.ConfigureStallGuard(sg_config);
    
    // Set explicit acceleration and deceleration (same value for both)
    driver.rampControl.SetAccelerations(TUNING_ACCELERATION_STEPS_S2, TUNING_ACCELERATION_STEPS_S2, tmc5160::Unit::Steps);
    
    driver.rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
    driver.rampControl.SetMaxSpeed(TUNING_VELOCITY_STEPS_S, tmc5160::Unit::Steps);
    
    // Monitor for a few seconds, but stop early if motor stalls
    ESP_LOGI(TAG, "Monitoring SG_RESULT (will stop if motor stalls)...");
    bool stall_detected = false;
    for(int i=0; i<50; i++) { // Run longer (5s)
        vTaskDelay(pdMS_TO_TICKS(100));
        uint16_t sg_val = driver.diagnostics.GetStallGuard();
        float current_speed = driver.rampControl.GetCurrentSpeed(tmc5160::Unit::Steps);
        ESP_LOGI(TAG, "SG_RESULT: %u, VACTUAL: %.1f steps/s", sg_val, current_speed);
        
        // Ignore low-speed stalls (resonance area)
        // Only trigger stop if speed is significant (> 5000 steps/s) AND SG=0
        if (sg_val == 0 && std::abs(current_speed) > 5000.0f) {
            ESP_LOGW(TAG, "Stall detected (SG=0) at V=%.1f! Stopping motor...", current_speed);
            stall_detected = true;
            driver.rampControl.Stop();
            // Wait for stop
            for(int j=0; j<50; j++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (std::abs(driver.rampControl.GetCurrentSpeed(tmc5160::Unit::Steps)) < 10.0f) break;
            }
            break;
        }
    }
    
    if (!stall_detected) {
        // No stall detected, stop normally
        driver.rampControl.Stop();
        vTaskDelay(pdMS_TO_TICKS(500)); // Allow time to decelerate
    }
    
  } else {
    ESP_LOGE(TAG, "Tuning Failed. Could not find stable SGT.");
  }

  ESP_LOGI(TAG, "Done. Reset to restart.");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
