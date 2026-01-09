/**
 * @file stallguard_tuning.cpp
 * @brief Automatic StallGuard2 Tuning Tool with Current Reduction
 *
 * This tool automatically tunes the StallGuard2 Threshold (SGT) for a specific
 * motor and velocity configuration using the comprehensive AutoTuneStallGuard function.
 * It implements the tuning algorithm following Trinamic application note AN-002 guidelines:
 * 
 * 1. Saves current motor settings (current, CoolStep, etc.)
 * 2. Applies current reduction for safer tuning and improved sensitivity
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

// FreeRTOS headers MUST come before tmc51x0.hpp (library uses FreeRTOS macros when ESP_PLATFORM defined)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include "esp_log.h"

static const char* TAG = "SGT_Tuning";

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// Test rig selection (compile-time constant) - automatically selects motor, board, and platform
// FATIGUE TEST RIG: Uses Applied Motion 5034-369 motor, TMC51x0 EVAL board, reference switches, encoder
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;

// Tuning Parameters (using RPM for user-friendly units)
// For Applied Motion 5034-369 NEMA 34 motor:
// - 60 RPM is above motor resonance zone (smoother operation)
// - 30 RPM causes vibration due to mid-band resonance (~100 Hz step frequency)
// - 5 rev/s² is the bounds search acceleration in fatigue test config
static constexpr float TUNING_VELOCITY_RPM = 60.0f;       // Target velocity for tuning (above resonance)
static constexpr float TUNING_ACCELERATION_REV_S2 = 5.0f; // Acceleration matching fatigue test config

// Unit definitions
static constexpr tmc51x0::Unit VELOCITY_UNIT = tmc51x0::Unit::RPM;
static constexpr tmc51x0::Unit ACCELERATION_UNIT = tmc51x0::Unit::RevPerSec;  // Acceleration must be in rev/s², not RPM

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting StallGuard2 Tuning Tool...");

  // 1. Initialize SPI Bus
  auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
  Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, tmc51x0_test_config::SPI_CLOCK_SPEED_HZ);

  auto spi_init_result = spi.Initialize();
  if (!spi_init_result) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus (ErrorCode: %d)", static_cast<int>(spi_init_result.Error()));
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
  if (!driver.status.VerifySetup()) {
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
  // For Applied Motion 5034: MIN_VELOCITY_RPM = 20 RPM
  // Set TCOOLTHRS to 20 RPM so StallGuard is active above this speed
  // PWM_THRS = 0 (no StealthChop), COOL_THRS = 20 RPM (SG min), HIGH_THRS = 0 (no limit)
  driver.thresholds.SetModeChangeSpeeds(0.0f, 20.0f, 0.0f, VELOCITY_UNIT); // PWM_THRS, COOL_THRS, HIGH_THRS

  ESP_LOGI(TAG, "Starting Comprehensive Auto-Tuning Sequence...");
  ESP_LOGI(TAG, "Target Velocity: %.2f RPM", TUNING_VELOCITY_RPM);
  ESP_LOGI(TAG, "Acceleration: %.3f rev/s²", TUNING_ACCELERATION_REV_S2);
  ESP_LOGI(TAG, "Using AutoTuneStallGuard with current reduction");
  ESP_LOGI(TAG, "NOTE: At %.2f RPM, one full revolution should take %.1f seconds", 
           TUNING_VELOCITY_RPM, 60.0f / TUNING_VELOCITY_RPM);
  
  // Use comprehensive automatic tuning with current reduction
  tmc51x0::StallGuardTuningResult result;
  // AutoTuneStallGuard: target_vel (most important), result, min_sgt, max_sgt, accel, min_vel, max_vel, unit, current_reduction_factor
  // 
  // For Applied Motion 5034-369:
  // - Target: 60 RPM (above motor resonance zone for smooth operation)
  // - Min velocity: 40 RPM (above resonance, above SG min of 20 RPM)
  // - Max velocity: 120 RPM (2x target for velocity range validation)
  float min_vel = 40.0f;  // Above resonance zone and SG min velocity
  float max_vel = 120.0f; // 2x target for range testing
  
  // Current reduction factor: reduces current to percentage of current motor current
  // This helps avoid excessive torque during stall tests and makes StallGuard more responsive to load changes
  // Recommended: 0.3 (30%) per Duet3D best practices for stall detection
  // Set to 0.0 to disable current reduction (use nominal current)
  // Matches TestConfig_AppliedMotion_5034::StallGuard::STALL_DETECTION_CURRENT_FACTOR
  float current_reduction_factor = 0.3f; // 30% of current motor current
  // Using RPM units for velocity parameters, RevPerSec for acceleration (RPM is not valid for acceleration)
  ESP_LOGI(TAG, "Starting AutoTuneStallGuard...");
  ESP_LOGI(TAG, "  Target velocity: %.2f %s", TUNING_VELOCITY_RPM, (VELOCITY_UNIT == tmc51x0::Unit::RPM) ? "RPM" : "units");
  ESP_LOGI(TAG, "  Acceleration: %.2f rev/s²", TUNING_ACCELERATION_REV_S2);
  ESP_LOGI(TAG, "  Velocity range: %.2f - %.2f %s", min_vel, max_vel, (VELOCITY_UNIT == tmc51x0::Unit::RPM) ? "RPM" : "units");
  ESP_LOGI(TAG, "  Current reduction factor: %.1f%%", current_reduction_factor * 100.0f);
  
  auto tune_result = driver.tuning.AutoTuneStallGuard(TUNING_VELOCITY_RPM, result, -63, 20, 
                                                     TUNING_ACCELERATION_REV_S2, min_vel, max_vel, 
                                                     VELOCITY_UNIT, ACCELERATION_UNIT, current_reduction_factor);

  if (tune_result) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "TUNING SUCCESSFUL");
    ESP_LOGI(TAG, "Optimal SGT (at target velocity): %d", result.optimal_sgt);
    ESP_LOGI(TAG, "SG_RESULT at target velocity: %u", result.target_velocity_sg_result);
    
    if (result.min_velocity_success) {
      ESP_LOGI(TAG, "Min velocity (%.2f RPM): Works with SGT %d (SG_RESULT=%u)", 
               min_vel, result.min_velocity_sgt, result.min_velocity_sg_result);
    } else {
      ESP_LOGW(TAG, "Min velocity (%.2f RPM): Does NOT work with optimal SGT", min_vel);
      if (result.actual_min_velocity > 0.0f) {
        ESP_LOGI(TAG, "  -> Actual working min velocity: %.2f RPM (SG_RESULT=%u)", 
                 result.actual_min_velocity, result.min_velocity_sg_result);
      }
    }
    
    if (result.max_velocity_success) {
      ESP_LOGI(TAG, "Max velocity (%.2f RPM): Works with SGT %d (SG_RESULT=%u)", 
               max_vel, result.max_velocity_sgt, result.max_velocity_sg_result);
    } else {
      ESP_LOGW(TAG, "Max velocity (%.2f RPM): Does NOT work with optimal SGT", max_vel);
      if (result.actual_max_velocity > 0.0f) {
        ESP_LOGI(TAG, "  -> Actual working max velocity: %.2f RPM (SG_RESULT=%u)", 
                 result.actual_max_velocity, result.max_velocity_sg_result);
      }
    }
    ESP_LOGI(TAG, "==========================================");
    
    // Test run with found SGT
    ESP_LOGI(TAG, "Verifying with test run...");
    tmc51x0::StallGuardConfig sg_config;
    sg_config.threshold = result.optimal_sgt;
    sg_config.enable_filter = true; // Enable filter for verification/operation (reduces noise)
    driver.stallGuard.ConfigureStallGuard(sg_config);
    
    // Set explicit acceleration and deceleration (same value for both)
    // Acceleration is in rev/s² (not RPM/s, as RPM/s is not a standard unit)
    driver.rampControl.SetAccelerations(TUNING_ACCELERATION_REV_S2, TUNING_ACCELERATION_REV_S2, ACCELERATION_UNIT);
    
    driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_POS);
    
    // Debug: Log the actual speed being set
    auto test_speed_result_before = driver.rampControl.GetCurrentSpeed(VELOCITY_UNIT);
    if (test_speed_result_before) {
      float test_speed = test_speed_result_before.Value();
      ESP_LOGI(TAG, "✓ Current speed before SetMaxSpeed: %.2f RPM", test_speed);
    } else {
      ESP_LOGW(TAG, "⚠ Could not read speed before SetMaxSpeed (ErrorCode: %d)", static_cast<int>(test_speed_result_before.Error()));
    }
    
    auto set_speed_result = driver.rampControl.SetMaxSpeed(TUNING_VELOCITY_RPM, VELOCITY_UNIT);
    if (!set_speed_result) {
      ESP_LOGE(TAG, "❌ Failed to set max speed (ErrorCode: %d)", static_cast<int>(set_speed_result.Error()));
    }
    
    // Debug: Verify the speed was set correctly
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for motor to start
    auto test_speed_result_after = driver.rampControl.GetCurrentSpeed(VELOCITY_UNIT);
    if (test_speed_result_after) {
      float test_speed = test_speed_result_after.Value();
      ESP_LOGI(TAG, "✓ Current speed after SetMaxSpeed: %.2f RPM (expected: %.2f RPM)", test_speed, TUNING_VELOCITY_RPM);
    } else {
      ESP_LOGW(TAG, "⚠ Could not read speed after SetMaxSpeed (ErrorCode: %d)", static_cast<int>(test_speed_result_after.Error()));
    }
    
    // Monitor for a few seconds, but stop early if motor stalls
    ESP_LOGI(TAG, "Monitoring SG_RESULT (will stop if motor stalls)...");
    bool stall_detected = false;
    for(int i=0; i<50; i++) { // Run longer (5s)
        vTaskDelay(pdMS_TO_TICKS(100));
        // Read StallGuard value
        auto sg_result = driver.stallGuard.GetStallGuard();
        uint16_t sg_val = 0;
        if (!sg_result) {
          ESP_LOGW(TAG, "⚠ Failed to read StallGuard (ErrorCode: %d), using 0", static_cast<int>(sg_result.Error()));
          sg_val = 0;
        } else {
          sg_val = sg_result.Value();
        }
        
        // Read current speed
        auto speed_result = driver.rampControl.GetCurrentSpeed(VELOCITY_UNIT);
        float current_speed = 0.0f;
        if (!speed_result) {
          ESP_LOGW(TAG, "⚠ Failed to read current speed (ErrorCode: %d), using 0", static_cast<int>(speed_result.Error()));
          current_speed = 0.0f;
        } else {
          current_speed = speed_result.Value();
        }
        
        ESP_LOGI(TAG, "SG_RESULT: %u, VACTUAL: %.2f RPM", sg_val, current_speed);
        
        // Ignore low-speed stalls (StallGuard unreliable below MIN_VELOCITY_RPM)
        // Only trigger stop if speed is above SG min velocity (20 RPM) AND SG=0
        if (sg_val == 0 && std::abs(current_speed) > 20.0f) {
            ESP_LOGW(TAG, "⚠️ Stall detected (SG=0) at V=%.2f RPM! Stopping motor...", current_speed);
            stall_detected = true;
            auto stop_result = driver.rampControl.Stop();
            if (!stop_result) {
              ESP_LOGE(TAG, "❌ Failed to stop motor (ErrorCode: %d)", static_cast<int>(stop_result.Error()));
            }
            // Wait for stop
            for(int j=0; j<50; j++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                auto verify_speed_result = driver.rampControl.GetCurrentSpeed(VELOCITY_UNIT);
                float speed = 0.0f;
                if (verify_speed_result) {
                  speed = verify_speed_result.Value();
                }
                if (std::abs(speed) < 0.6f) break; // ~0.6 RPM threshold
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
    ESP_LOGE(TAG, "❌ Tuning Failed. Could not find stable SGT.");
    ESP_LOGE(TAG, "   ErrorCode: %d", static_cast<int>(tune_result.Error()));
    ESP_LOGE(TAG, "   Possible causes:");
    ESP_LOGE(TAG, "     - Motor not connected or not powered");
    ESP_LOGE(TAG, "     - Velocity range too narrow or invalid");
    ESP_LOGE(TAG, "     - Acceleration too high for motor");
    ESP_LOGE(TAG, "     - SGT range (-64 to +63) may need adjustment");
  }

  // ========================================================================
  // CLEANUP: Stop motor and disable driver outputs
  // ========================================================================
  ESP_LOGI(TAG, "Stopping motor and disabling driver...");
  driver.rampControl.Stop();
  vTaskDelay(pdMS_TO_TICKS(500)); // Allow time to fully stop
  
  if (!driver.motorControl.Disable()) {
    ESP_LOGW(TAG, "Warning: Failed to disable motor outputs");
  } else {
    ESP_LOGI(TAG, "Motor disabled successfully");
  }

  ESP_LOGI(TAG, "==========================================");
  ESP_LOGI(TAG, "Tuning complete. Motor is now disabled.");
  ESP_LOGI(TAG, "Reset to restart tuning.");
  ESP_LOGI(TAG, "==========================================");
  
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
