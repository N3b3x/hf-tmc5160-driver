/**
 * @file gpio_combinations.cpp
 * @brief Comprehensive example demonstrating different GPIO pin combinations for TMC5160
 *
 * This example shows how to configure the TMC5160 driver with different combinations
 * of GPIO pins, demonstrating:
 * 1. Minimal configuration (only basic EN, DIR, STEP)
 * 2. Full configuration with all optional pins
 * 3. Configuration with reference switches
 * 4. Configuration with encoder interface
 * 5. Configuration with diagnostic pins
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 * - Optional: Reference switches, encoder, diagnostic connections
 *
 * Pin Mapping Reference (TMC5160 TQFP pins):
 * - Pin 17: REFL_STEP (STEP when SD_MODE=1, Left ref switch when SD_MODE=0)
 * - Pin 18: REFR_DIR (DIR when SD_MODE=1, Right ref switch when SD_MODE=0)
 * - Pin 21: SD_MODE (Mode select: Low=Internal controller, High=Step/Dir)
 * - Pin 22: SPI_MODE (Low=CFG/UART, High=SPI)
 * - Pin 23: ENCB_DCEN_CFG4 (Encoder B / DcStep enable)
 * - Pin 24: ENCA_DCIN_CFG5 (Encoder A / DcStep sync)
 * - Pin 25: ENCN_DCO_CFG6 (Encoder N / DcStep ready)
 * - Pin 26: DIAG0_SWN (Diagnostic 0 / Interrupt / UART negative)
 * - Pin 27: DIAG1_SWP (Diagnostic 1 / Position compare / UART positive)
 * - Pin 28: DRV_ENN (Driver enable, active low)
 * - Pin 12: CLK (External clock input, optional)
 *
 * @author Generated for TMC5160 Library
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GPIO_Combinations";

/**
 * @brief Example 1: Minimal configuration
 *
 * Only basic pins (EN, DIR, STEP) connected. This is the simplest setup
 * for basic motor control using the internal motion controller.
 */
void example_minimal_config() {
  ESP_LOGI(TAG, "=== Example 1: Minimal Configuration ===");

  // Minimal pin configuration - only basic control pins
  Esp32SPI spi(SPI2_HOST,
               GPIO_NUM_23, // MOSI
               GPIO_NUM_19, // MISO
               GPIO_NUM_18, // SCLK
               GPIO_NUM_5,  // CS
               GPIO_NUM_2,  // EN
               GPIO_NUM_4,  // DIR
               GPIO_NUM_15, // STEP
               4000000);    // 4 MHz SPI clock

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI");
    return;
  }

  tmc5160::TMC5160 driver(spi);
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver");
    return;
  }

  ESP_LOGI(TAG, "Minimal config: Using only EN, DIR, STEP pins");
  ESP_LOGI(TAG, "All other pins are not connected (GPIO_NUM_NC)");

  // Test basic control
  driver.motorControl.Enable();
  driver.rampControl.SetTargetPosition(1000);
  driver.rampControl.SetMaxSpeed(1000.0f);
  driver.rampControl.SetAcceleration(500.0f);

  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds

  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Minimal config example complete\n");
}

/**
 * @brief Example 2: Full pin configuration
 *
 * Demonstrates configuration with all optional pins connected.
 * This setup enables all features: reference switches, encoder,
 * diagnostics, mode control, etc.
 */
void example_full_config() {
  ESP_LOGI(TAG, "=== Example 2: Full Pin Configuration ===");

  // Full pin configuration structure
  TMC5160PinConfig pin_config{};
  
  // SPI pins (required)
  pin_config.mosi_pin = GPIO_NUM_23;
  pin_config.miso_pin = GPIO_NUM_19;
  pin_config.sclk_pin = GPIO_NUM_18;
  pin_config.cs_pin = GPIO_NUM_5;

  // Basic control pins
  pin_config.en_pin = GPIO_NUM_2;
  pin_config.dir_pin = GPIO_NUM_4;
  pin_config.step_pin = GPIO_NUM_15;

  // Reference switch pins (for homing/endstops)
  pin_config.refl_step_pin = GPIO_NUM_25;  // Left reference switch
  pin_config.refr_dir_pin = GPIO_NUM_26;   // Right reference switch

  // Encoder interface pins
  pin_config.enca_pin = GPIO_NUM_32;  // Encoder A
  pin_config.encb_pin = GPIO_NUM_33;  // Encoder B
  pin_config.encn_pin = GPIO_NUM_27;  // Encoder N (index)

  // Diagnostic/interrupt pins (open-drain, need pull-ups)
  pin_config.diag0_pin = GPIO_NUM_35;  // DIAG0 (fault interrupt)
  pin_config.diag1_pin = GPIO_NUM_34;  // DIAG1 (position compare)

  // Mode selection pins
  pin_config.sd_mode_pin = GPIO_NUM_21;   // Step/Dir vs Internal controller
  pin_config.spi_mode_pin = GPIO_NUM_22;  // SPI vs CFG/UART mode

  // Clock input (optional - usually not needed)
  pin_config.clk_pin = GPIO_NUM_NC;  // Not connected - using internal clock

  // Driver enable (hardware kill signal)
  pin_config.drv_enn_pin = GPIO_NUM_0;  // Hardware enable (can be same as EN)

  // Create SPI interface with full pin configuration
  Esp32SPI spi(SPI2_HOST, pin_config, 4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI");
    return;
  }

  tmc5160::TMC5160 driver(spi);
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver");
    return;
  }

  ESP_LOGI(TAG, "Full config: All pins configured");
  ESP_LOGI(TAG, "- Reference switches: GPIO %d (left), GPIO %d (right)",
           pin_config.refl_step_pin, pin_config.refr_dir_pin);
  ESP_LOGI(TAG, "- Encoder: GPIO %d (A), GPIO %d (B), GPIO %d (N)",
           pin_config.enca_pin, pin_config.encb_pin, pin_config.encn_pin);
  ESP_LOGI(TAG, "- Diagnostics: GPIO %d (DIAG0), GPIO %d (DIAG1)",
           pin_config.diag0_pin, pin_config.diag1_pin);
  ESP_LOGI(TAG, "- Mode pins: GPIO %d (SD_MODE), GPIO %d (SPI_MODE)",
           pin_config.sd_mode_pin, pin_config.spi_mode_pin);

  // Test reading diagnostic pins
  tmc5160::GpioSignal diag0_signal, diag1_signal;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG0_SWN, diag0_signal)) {
    ESP_LOGI(TAG, "DIAG0 state: %s",
             diag0_signal == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  }
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG1_SWP, diag1_signal)) {
    ESP_LOGI(TAG, "DIAG1 state: %s",
             diag1_signal == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  }

  // Test mode pin control
  ESP_LOGI(TAG, "Setting SD_MODE to internal controller mode (LOW)");
  spi.GpioSet(tmc5160::TMC5160CtrlPin::SD_MODE, tmc5160::GpioSignal::INACTIVE);

  driver.motorControl.Enable();
  driver.rampControl.SetTargetPosition(1000);
  driver.rampControl.SetMaxSpeed(1000.0f);
  driver.rampControl.SetAcceleration(500.0f);

  vTaskDelay(pdMS_TO_TICKS(2000));

  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Full config example complete\n");
}

/**
 * @brief Example 3: Configuration with reference switches only
 *
 * Demonstrates setup for homing/endstop functionality using reference switches.
 * This is useful for applications requiring position reference.
 */
void example_reference_switches_config() {
  ESP_LOGI(TAG, "=== Example 3: Reference Switches Configuration ===");

  TMC5160PinConfig pin_config{};
  
  // SPI pins
  pin_config.mosi_pin = GPIO_NUM_23;
  pin_config.miso_pin = GPIO_NUM_19;
  pin_config.sclk_pin = GPIO_NUM_18;
  pin_config.cs_pin = GPIO_NUM_5;

  // Basic control pins
  pin_config.en_pin = GPIO_NUM_2;
  pin_config.dir_pin = GPIO_NUM_4;
  pin_config.step_pin = GPIO_NUM_15;

  // Reference switch pins (for homing)
  pin_config.refl_step_pin = GPIO_NUM_25;  // Left endstop
  pin_config.refr_dir_pin = GPIO_NUM_26;   // Right endstop

  // All other pins not connected
  // (encoder, diagnostics, mode pins default to GPIO_NUM_NC)

  Esp32SPI spi(SPI2_HOST, pin_config, 4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI");
    return;
  }

  tmc5160::TMC5160 driver(spi);
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver");
    return;
  }

  ESP_LOGI(TAG, "Reference switches config:");
  ESP_LOGI(TAG, "- Left switch: GPIO %d", pin_config.refl_step_pin);
  ESP_LOGI(TAG, "- Right switch: GPIO %d", pin_config.refr_dir_pin);

  // Configure reference switches in the driver
  tmc5160::ReferenceSwitchConfig ref_config{};
  ref_config.stop_left_enable = true;
  ref_config.stop_right_enable = true;
  ref_config.pol_stop_left = false;   // Active high
  ref_config.pol_stop_right = false; // Active high
  ref_config.en_softstop = true;     // Use deceleration ramp

  driver.rampControl.ConfigureReferenceSwitch(ref_config);
  ESP_LOGI(TAG, "Reference switches configured for homing");

  // Test reading switch states
  tmc5160::GpioSignal left_switch, right_switch;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::REFL_STEP, left_switch)) {
    ESP_LOGI(TAG, "Left switch: %s",
             left_switch == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  }
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::REFR_DIR, right_switch)) {
    ESP_LOGI(TAG, "Right switch: %s",
             right_switch == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  }

  driver.motorControl.Enable();
  driver.rampControl.SetTargetPosition(1000);
  driver.rampControl.SetMaxSpeed(1000.0f);
  driver.rampControl.SetAcceleration(500.0f);

  vTaskDelay(pdMS_TO_TICKS(2000));

  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Reference switches config example complete\n");
}

/**
 * @brief Example 4: Configuration with encoder interface
 *
 * Demonstrates setup for closed-loop control using encoder feedback.
 * This enables position verification and closed-loop operation.
 */
void example_encoder_config() {
  ESP_LOGI(TAG, "=== Example 4: Encoder Interface Configuration ===");

  TMC5160PinConfig pin_config{};
  
  // SPI pins
  pin_config.mosi_pin = GPIO_NUM_23;
  pin_config.miso_pin = GPIO_NUM_19;
  pin_config.sclk_pin = GPIO_NUM_18;
  pin_config.cs_pin = GPIO_NUM_5;

  // Basic control pins
  pin_config.en_pin = GPIO_NUM_2;
  pin_config.dir_pin = GPIO_NUM_4;
  pin_config.step_pin = GPIO_NUM_15;

  // Encoder interface pins
  pin_config.enca_pin = GPIO_NUM_32;  // Encoder A
  pin_config.encb_pin = GPIO_NUM_33;   // Encoder B
  pin_config.encn_pin = GPIO_NUM_27;   // Encoder N (index)

  Esp32SPI spi(SPI2_HOST, pin_config, 4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI");
    return;
  }

  tmc5160::TMC5160 driver(spi);
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver");
    return;
  }

  ESP_LOGI(TAG, "Encoder config:");
  ESP_LOGI(TAG, "- Encoder A: GPIO %d", pin_config.enca_pin);
  ESP_LOGI(TAG, "- Encoder B: GPIO %d", pin_config.encb_pin);
  ESP_LOGI(TAG, "- Encoder N: GPIO %d", pin_config.encn_pin);

  // Configure encoder
  tmc5160::EncoderConfig enc_config{};
  enc_config.pol_a = false;
  enc_config.pol_b = false;
  enc_config.pol_n = true;  // Active high
  enc_config.clr_enc_x = true;
  enc_config.latch_x_act = true;

  driver.encoder.Configure(enc_config);
  ESP_LOGI(TAG, "Encoder configured for closed-loop control");

  // Set encoder resolution (example: 200 steps motor, 2048 CPR encoder)
  driver.encoder.SetResolution(200, 2048, false);

  driver.motorControl.Enable();
  driver.rampControl.SetTargetPosition(1000);
  driver.rampControl.SetMaxSpeed(1000.0f);
  driver.rampControl.SetAcceleration(500.0f);

  // Monitor encoder position
  vTaskDelay(pdMS_TO_TICKS(1000));
  int32_t encoder_pos = driver.encoder.GetPosition();
  ESP_LOGI(TAG, "Encoder position: %ld", encoder_pos);

  vTaskDelay(pdMS_TO_TICKS(1000));

  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Encoder config example complete\n");
}

/**
 * @brief Example 5: Configuration with diagnostic pins
 *
 * Demonstrates setup for fault detection and status monitoring using
 * diagnostic/interrupt pins.
 */
void example_diagnostic_pins_config() {
  ESP_LOGI(TAG, "=== Example 5: Diagnostic Pins Configuration ===");

  TMC5160PinConfig pin_config{};
  
  // SPI pins
  pin_config.mosi_pin = GPIO_NUM_23;
  pin_config.miso_pin = GPIO_NUM_19;
  pin_config.sclk_pin = GPIO_NUM_18;
  pin_config.cs_pin = GPIO_NUM_5;

  // Basic control pins
  pin_config.en_pin = GPIO_NUM_2;
  pin_config.dir_pin = GPIO_NUM_4;
  pin_config.step_pin = GPIO_NUM_15;

  // Diagnostic/interrupt pins (open-drain, need pull-ups)
  pin_config.diag0_pin = GPIO_NUM_35;  // DIAG0 (fault interrupt)
  pin_config.diag1_pin = GPIO_NUM_34;  // DIAG1 (position compare)

  Esp32SPI spi(SPI2_HOST, pin_config, 4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI");
    return;
  }

  tmc5160::TMC5160 driver(spi);
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver");
    return;
  }

  ESP_LOGI(TAG, "Diagnostic pins config:");
  ESP_LOGI(TAG, "- DIAG0: GPIO %d (fault interrupt)", pin_config.diag0_pin);
  ESP_LOGI(TAG, "- DIAG1: GPIO %d (position compare)", pin_config.diag1_pin);
  ESP_LOGI(TAG, "Note: These pins are open-drain and require pull-up resistors");

  driver.motorControl.Enable();

  // Monitor diagnostic pins periodically
  for (int i = 0; i < 10; i++) {
    tmc5160::GpioSignal diag0, diag1;
    
    if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG0_SWN, diag0)) {
      if (diag0 == tmc5160::GpioSignal::ACTIVE) {
        ESP_LOGW(TAG, "DIAG0 ACTIVE - Fault detected!");
        // Check driver status
        tmc5160::DriverStatus status = driver.diagnostics.GetStatus();
        ESP_LOGI(TAG, "Driver status: %d", static_cast<int>(status));
      }
    }
    
    if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG1_SWP, diag1)) {
      if (diag1 == tmc5160::GpioSignal::ACTIVE) {
        ESP_LOGI(TAG, "DIAG1 ACTIVE - Position compare match");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }

  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Diagnostic pins config example complete\n");
}

/**
 * @brief Example 6: Step/Dir mode configuration
 *
 * Demonstrates configuration for external step/dir control mode.
 * In this mode, the MCU provides step pulses and direction signals.
 */
void example_step_dir_mode_config() {
  ESP_LOGI(TAG, "=== Example 6: Step/Dir Mode Configuration ===");

  TMC5160PinConfig pin_config{};
  
  // SPI pins
  pin_config.mosi_pin = GPIO_NUM_23;
  pin_config.miso_pin = GPIO_NUM_19;
  pin_config.sclk_pin = GPIO_NUM_18;
  pin_config.cs_pin = GPIO_NUM_5;

  // Basic control pins
  pin_config.en_pin = GPIO_NUM_2;
  pin_config.dir_pin = GPIO_NUM_4;
  pin_config.step_pin = GPIO_NUM_15;

  // Mode selection pin - set SD_MODE high for Step/Dir mode
  pin_config.sd_mode_pin = GPIO_NUM_21;

  Esp32SPI spi(SPI2_HOST, pin_config, 4000000);

  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI");
    return;
  }

  // Set SD_MODE high for Step/Dir mode
  ESP_LOGI(TAG, "Setting SD_MODE to HIGH (Step/Dir mode)");
  spi.GpioSet(tmc5160::TMC5160CtrlPin::SD_MODE, tmc5160::GpioSignal::ACTIVE);

  tmc5160::TMC5160 driver(spi);
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;

  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver");
    return;
  }

  ESP_LOGI(TAG, "Step/Dir mode config:");
  ESP_LOGI(TAG, "- SD_MODE: GPIO %d (HIGH = Step/Dir mode)", pin_config.sd_mode_pin);
  ESP_LOGI(TAG, "- STEP pin: GPIO %d (step pulses)", pin_config.step_pin);
  ESP_LOGI(TAG, "- DIR pin: GPIO %d (direction)", pin_config.dir_pin);

  driver.motorControl.Enable();

  // Generate step pulses manually
  ESP_LOGI(TAG, "Generating step pulses...");
  for (int i = 0; i < 100; i++) {
    // Set direction
    spi.GpioSet(tmc5160::TMC5160CtrlPin::DIR,
                (i < 50) ? tmc5160::GpioSignal::ACTIVE : tmc5160::GpioSignal::INACTIVE);

    // Generate step pulse (rising edge)
    spi.GpioSet(tmc5160::TMC5160CtrlPin::STEP, tmc5160::GpioSignal::INACTIVE);
    vTaskDelay(pdMS_TO_TICKS(1));
    spi.GpioSet(tmc5160::TMC5160CtrlPin::STEP, tmc5160::GpioSignal::ACTIVE);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Step/Dir mode config example complete\n");
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 GPIO Combinations Example");
  ESP_LOGI(TAG, "This example demonstrates different GPIO pin configurations");
  ESP_LOGI(TAG, "========================================\n");

  vTaskDelay(pdMS_TO_TICKS(1000));

  // Run each example configuration
  example_minimal_config();
  vTaskDelay(pdMS_TO_TICKS(1000));

  example_full_config();
  vTaskDelay(pdMS_TO_TICKS(1000));

  example_reference_switches_config();
  vTaskDelay(pdMS_TO_TICKS(1000));

  example_encoder_config();
  vTaskDelay(pdMS_TO_TICKS(1000));

  example_diagnostic_pins_config();
  vTaskDelay(pdMS_TO_TICKS(1000));

  example_step_dir_mode_config();

  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "All GPIO combination examples complete!");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Summary:");
  ESP_LOGI(TAG, "- Example 1: Minimal (EN, DIR, STEP only)");
  ESP_LOGI(TAG, "- Example 2: Full configuration (all pins)");
  ESP_LOGI(TAG, "- Example 3: Reference switches (homing)");
  ESP_LOGI(TAG, "- Example 4: Encoder interface (closed-loop)");
  ESP_LOGI(TAG, "- Example 5: Diagnostic pins (fault detection)");
  ESP_LOGI(TAG, "- Example 6: Step/Dir mode (external control)");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Note: Modify GPIO pin numbers to match your hardware setup");
  ESP_LOGI(TAG, "Use GPIO_NUM_NC for pins that are not connected");
}
