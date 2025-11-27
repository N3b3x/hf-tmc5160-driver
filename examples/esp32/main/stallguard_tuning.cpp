/**
 * @file stallguard_tuning.cpp
 * @brief Automatic StallGuard2 Tuning Tool with Safe Current Margin
 *
 * This tool automatically tunes the StallGuard2 Threshold (SGT) for a specific
 * motor and velocity configuration using the comprehensive AutoTuneStallGuard function.
 * It implements the tuning algorithm following Trinamic application note AN-002 guidelines:
 * 
 * 1. Saves current motor settings (current, CoolStep, etc.)
 * 2. Applies safe current margin for safer tuning and improved sensitivity
 * 3. Disables interfering features (CoolStep, filter, stop-on-stall)
 * 4. Moves the motor at a constant velocity
 * 5. Monitors the SG_RESULT (StallGuard value) across SGT range
 * 6. Adjusts SGT until a stable non-zero SG_RESULT in ideal range (100-500) is obtained
 * 7. Validates at min/max velocities
 * 8. Restores all saved settings (except optimal SGT)
 * 
 * USAGE:
 * 1. Ensure the motor is free to move (no load or minimal load).
 * 2. Run this tool.
 * 3. The tool will output the found optimal SGT value and velocity range analysis.
 * 4. Use this SGT value in your application.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SGT_Tuning";

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// Test rig selection (compile-time constant) - automatically selects motor, board, and platform
// CORE DRIVER TEST RIG: Uses 17HS4401S gearbox motor, TMC51x0 EVAL board, reference switches, encoder
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER;

// Tuning Parameters (using revolutions per second for user-friendly units)
static constexpr float TUNING_VELOCITY_REV_S = 0.6f; // Target velocity: 0.6 rev/s (~36 RPM)
static constexpr float TUNING_ACCELERATION_REV_S2 = 0.06f; // Acceleration: 0.06 rev/s² (realistic for stepper motors)

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting StallGuard2 Tuning Tool...");

  // 1. Initialize SPI Bus
  auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
  Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, tmc51x0_test_config::SPI_CLOCK_SPEED_HZ);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus");
    return;
  }

  // 2. Initialize Driver
  tmc51x0::TMC51x0<Esp32SPI> driver(spi);
  tmc51x0::DriverConfig driver_config;

  // Configure driver from unified test rig selection
  tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(driver_config);

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
  // Using RevPerSec units (default) - 0.02 rev/s (1.2 RPM) threshold ensures StallGuard is active
  driver.motorControl.SetModeChangeSpeeds(0.002f, 0.02f, 0.0f, tmc51x0::Unit::RevPerSec); // PWM_THRS, COOL_THRS, HIGH_THRS

  ESP_LOGI(TAG, "Starting Comprehensive Auto-Tuning Sequence...");
  ESP_LOGI(TAG, "Target Velocity: %.2f rev/s (%.1f RPM)", TUNING_VELOCITY_REV_S, TUNING_VELOCITY_REV_S * 60.0f);
  ESP_LOGI(TAG, "Acceleration: %.3f rev/s²", TUNING_ACCELERATION_REV_S2);
  ESP_LOGI(TAG, "Using AutoTuneStallGuard with safe current margin handling");
  
  // Use comprehensive automatic tuning with safe current margin
  tmc51x0::StallGuardTuningResult result;
  // AutoTuneStallGuard: target_vel (most important), result, min_sgt, max_sgt, accel, min_vel, max_vel, unit, safe_current_margin_mA
  // For this example, we'll test a velocity range to demonstrate the feature
  float min_vel = TUNING_VELOCITY_REV_S * 0.3f;  // 30% of target
  float max_vel = TUNING_VELOCITY_REV_S * 1.5f;  // 150% of target
  // Safe current margin: reduce current by specified amount for safer tuning and improved StallGuard sensitivity
  // This helps avoid excessive torque during stall tests and makes StallGuard more responsive to load changes
  // Recommended: 15-25% of motor's rated current (e.g., 300mA for a 2A motor = 15% margin)
  // Set to 0 to disable current margin (use nominal current)
  uint16_t safe_current_margin_mA = 300; // Adjust based on your motor's rated current
  // Note: Unit::RevPerSec is now the default, so we can omit it, but showing it explicitly for clarity
  bool success = driver.tuning.AutoTuneStallGuard(TUNING_VELOCITY_REV_S, result, 0, 63, 
                                                     TUNING_ACCELERATION_REV_S2, min_vel, max_vel, 
                                                     tmc51x0::Unit::RevPerSec, safe_current_margin_mA);

  if (success) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "TUNING SUCCESSFUL");
    ESP_LOGI(TAG, "Optimal SGT (at target velocity): %d", result.optimal_sgt);
    ESP_LOGI(TAG, "SG_RESULT at target velocity: %u", result.target_velocity_sg_result);
    
    if (result.min_velocity_success) {
      ESP_LOGI(TAG, "Min velocity (%.3f rev/s, %.1f RPM): Works with SGT %d (SG_RESULT=%u)", 
               min_vel, min_vel * 60.0f, result.min_velocity_sgt, result.min_velocity_sg_result);
    } else {
      ESP_LOGW(TAG, "Min velocity (%.3f rev/s, %.1f RPM): Does NOT work with optimal SGT", min_vel, min_vel * 60.0f);
      if (result.actual_min_velocity > 0.0f) {
        ESP_LOGI(TAG, "  -> Actual working min velocity: %.3f rev/s (%.1f RPM) (SG_RESULT=%u)", 
                 result.actual_min_velocity, result.actual_min_velocity * 60.0f, result.min_velocity_sg_result);
      }
    }
    
    if (result.max_velocity_success) {
      ESP_LOGI(TAG, "Max velocity (%.3f rev/s, %.1f RPM): Works with SGT %d (SG_RESULT=%u)", 
               max_vel, max_vel * 60.0f, result.max_velocity_sgt, result.max_velocity_sg_result);
    } else {
      ESP_LOGW(TAG, "Max velocity (%.3f rev/s, %.1f RPM): Does NOT work with optimal SGT", max_vel, max_vel * 60.0f);
      if (result.actual_max_velocity > 0.0f) {
        ESP_LOGI(TAG, "  -> Actual working max velocity: %.3f rev/s (%.1f RPM) (SG_RESULT=%u)", 
                 result.actual_max_velocity, result.actual_max_velocity * 60.0f, result.max_velocity_sg_result);
      }
    }
    ESP_LOGI(TAG, "==========================================");
    
    // Test run with found SGT
    ESP_LOGI(TAG, "Verifying with test run...");
    tmc51x0::StallGuardConfig sg_config;
    sg_config.threshold = result.optimal_sgt;
    sg_config.enable_filter = true; // Enable filter for verification/operation (reduces noise)
    driver.diagnostics.ConfigureStallGuard(sg_config);
    
    // Set explicit acceleration and deceleration (same value for both)
    // Unit::RevPerSec is now the default, but showing it explicitly for clarity
    driver.rampControl.SetAccelerations(TUNING_ACCELERATION_REV_S2, TUNING_ACCELERATION_REV_S2, tmc51x0::Unit::RevPerSec);
    
    driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_POS);
    driver.rampControl.SetMaxSpeed(TUNING_VELOCITY_REV_S, tmc51x0::Unit::RevPerSec);
    
    // Monitor for a few seconds, but stop early if motor stalls
    ESP_LOGI(TAG, "Monitoring SG_RESULT (will stop if motor stalls)...");
    bool stall_detected = false;
    for(int i=0; i<50; i++) { // Run longer (5s)
        vTaskDelay(pdMS_TO_TICKS(100));
        uint16_t sg_val = 0;
        if (!driver.diagnostics.GetStallGuard(sg_val)) {
          sg_val = 0;
        }
        float current_speed = 0.0f;
        if (!driver.rampControl.GetCurrentSpeed(current_speed, tmc51x0::Unit::RevPerSec)) {
          current_speed = 0.0f;
        }
        ESP_LOGI(TAG, "SG_RESULT: %u, VACTUAL: %.3f rev/s (%.1f RPM)", sg_val, current_speed, current_speed * 60.0f);
        
        // Ignore low-speed stalls (resonance area)
        // Only trigger stop if speed is significant (> 0.1 rev/s ≈ 6 RPM) AND SG=0
        if (sg_val == 0 && std::abs(current_speed) > 0.1f) {
            ESP_LOGW(TAG, "Stall detected (SG=0) at V=%.3f rev/s (%.1f RPM)! Stopping motor...", current_speed, current_speed * 60.0f);
            stall_detected = true;
            driver.rampControl.Stop();
            // Wait for stop
            for(int j=0; j<50; j++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                float speed = 0.0f;
                if (!driver.rampControl.GetCurrentSpeed(speed, tmc51x0::Unit::RevPerSec)) {
                  speed = 0.0f;
                }
                if (std::abs(speed) < 0.01f) break; // ~0.6 RPM threshold
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
