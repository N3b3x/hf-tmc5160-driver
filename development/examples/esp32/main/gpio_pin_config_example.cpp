/**
 * @file gpio_pin_config_example.cpp
 * @brief Example demonstrating GPIO pin configuration for TMC5160 on ESP32-C6
 *
 * This example demonstrates how to configure and use all TMC5160 control pins
 * including diagnostic pins, reference switches, encoder pins, and CLK pin.
 *
 * ESP32-C6 Pin Configuration:
 * - SCK: GPIO5
 * - MOSI: GPIO6
 * - CLK16: GPIO10 (external clock)
 * - DRV_EN: GPIO11
 * - MISO: GPIO12
 * - CS_TMC: GPIO18
 * - DIAG0: GPIOXX (configure based on your board)
 * - DIAG1: GPIOXX (configure based on your board)
 *
 * Hardware Requirements:
 * - ESP32-C6 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32-C6 and TMC5160
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "GpioPinConfig";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 GPIO Pin Configuration Example");
  ESP_LOGI(TAG, "ESP32-C6 Pin Configuration:");

  // Create SPI communication interface with ESP32-C6 pin configuration
  // Note: DIR and STEP are not used in SPI mode with internal ramp generator (SD_MODE=0)
  Esp32SPI spi(SPI2_HOST,
               GPIO_NUM_6,                  // MOSI
               GPIO_NUM_12,                 // MISO
               GPIO_NUM_5,                  // SCLK
               GPIO_NUM_18,                 // CS_TMC
               GPIO_NUM_11,                 // DRV_EN (EN pin)
               static_cast<gpio_num_t>(-1), // DIR (not used in SPI mode with internal ramp generator)
               static_cast<gpio_num_t>(-1), // STEP (not used in SPI mode with internal ramp generator)
               4000000);                    // 4 MHz SPI clock

  // Configure additional pins using SetPinMapping()
  // Replace GPIO_NUM_XX with your actual GPIO numbers for DIAG0 and DIAG1
  // Example: If DIAG0 is on GPIO20 and DIAG1 is on GPIO21:
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::DIAG0, GPIO_NUM_20);
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::DIAG1, GPIO_NUM_21);

  // Configure CLK pin (GPIO10 for external clock)
  spi.SetPinMapping(tmc5160::TMC5160CtrlPin::CLK, GPIO_NUM_10);

  // Configure reference switch pins (if used)
  // These are the same physical pins as DIR/STEP but used as reference switches
  // when SD_MODE=0 (internal ramp generator mode)
  // Example: If REFL_STEP is on GPIO4 and REFR_DIR is on GPIO7:
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::REFL_STEP, GPIO_NUM_4);
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::REFR_DIR, GPIO_NUM_7);

  // Configure encoder pins (if used, when SD_MODE=0)
  // Example: If encoder pins are on GPIO8, GPIO9, GPIO10:
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::ENCA, GPIO_NUM_8);
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::ENCB, GPIO_NUM_9);
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::ENCN, GPIO_NUM_10);

  // Configure DC Step pins (if used, when SD_MODE=1, SPI_MODE=1)
  // Note: These are the same physical pins as encoder pins but used for DC Step control
  // Example: If DC Step pins are on GPIO8, GPIO9, GPIO10:
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::DCIN, GPIO_NUM_8);  // DC Step gating input
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::DCEN, GPIO_NUM_9);  // DC Step enable input
  // spi.SetPinMapping(tmc5160::TMC5160CtrlPin::DCO, GPIO_NUM_10); // DC Step ready output

  // Initialize SPI interface
  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }
  ESP_LOGI(TAG, "SPI interface initialized");

  // Create TMC5160 driver instance
  tmc5160::TMC5160<Esp32SPI> driver(spi);

  // Configure driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;
  cfg.motor.global_scaler = 32;
  cfg.chopper.toff = 5;
  cfg.chopper.mres = 4; // 16 microsteps
  cfg.chopper.intpol = true;

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }
  ESP_LOGI(TAG, "Driver initialized successfully");

  // Example 1: Read diagnostic pins (DIAG0, DIAG1)
  ESP_LOGI(TAG, "\n=== Reading Diagnostic Pins ===");
  tmc5160::GpioSignal diag0_signal, diag1_signal;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG0, diag0_signal)) {
    ESP_LOGI(TAG, "DIAG0: %s", diag0_signal == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  } else {
    ESP_LOGW(TAG, "DIAG0 pin not mapped or read failed");
  }

  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG1, diag1_signal)) {
    ESP_LOGI(TAG, "DIAG1: %s", diag1_signal == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  } else {
    ESP_LOGW(TAG, "DIAG1 pin not mapped or read failed");
  }

  // Example 2: Read reference switch pins (if configured)
  ESP_LOGI(TAG, "\n=== Reading Reference Switch Pins ===");
  tmc5160::GpioSignal ref_left, ref_right;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::REFL_STEP, ref_left)) {
    ESP_LOGI(TAG, "Left Reference Switch: %s", ref_left == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  }

  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::REFR_DIR, ref_right)) {
    ESP_LOGI(TAG, "Right Reference Switch: %s", ref_right == tmc5160::GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
  }

  // Example 3: Read encoder pins (if configured, when SD_MODE=0)
  ESP_LOGI(TAG, "\n=== Reading Encoder Pins ===");
  tmc5160::GpioSignal enca, encb, encn;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::ENCA, enca)) {
    ESP_LOGI(TAG, "Encoder A: %s", enca == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::ENCB, encb)) {
    ESP_LOGI(TAG, "Encoder B: %s", encb == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::ENCN, encn)) {
    ESP_LOGI(TAG, "Encoder N: %s", encn == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }

  // Example 3b: Read DC Step pins (if configured, when SD_MODE=1, SPI_MODE=1)
  ESP_LOGI(TAG, "\n=== Reading DC Step Pins ===");
  tmc5160::GpioSignal dcin, dcen, dco;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DCIN, dcin)) {
    ESP_LOGI(TAG, "DC Step gating input: %s", dcin == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DCEN, dcen)) {
    ESP_LOGI(TAG, "DC Step enable input: %s", dcen == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DCO, dco)) {
    ESP_LOGI(TAG, "DC Step ready output: %s", dco == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }

  // Example 4: Control EN pin (DRV_ENN)
  ESP_LOGI(TAG, "\n=== Controlling EN Pin ===");
  ESP_LOGI(TAG, "Disabling motor (EN = ACTIVE/HIGH)");
  spi.GpioSet(tmc5160::TMC5160CtrlPin::EN, tmc5160::GpioSignal::ACTIVE); // Disable (inverted logic)
  vTaskDelay(pdMS_TO_TICKS(100));

  ESP_LOGI(TAG, "Enabling motor (EN = INACTIVE/LOW)");
  spi.GpioSet(tmc5160::TMC5160CtrlPin::EN, tmc5160::GpioSignal::INACTIVE); // Enable
  vTaskDelay(pdMS_TO_TICKS(100));

  // Example 5: Read CLK pin state
  ESP_LOGI(TAG, "\n=== Reading CLK Pin ===");
  tmc5160::GpioSignal clk_signal;
  if (spi.GpioRead(tmc5160::TMC5160CtrlPin::CLK, clk_signal)) {
    ESP_LOGI(TAG, "CLK pin state: %s", clk_signal == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
  }

  // Example 6: Using driver's motor control methods (which use GPIO internally)
  ESP_LOGI(TAG, "\n=== Using Driver Motor Control ===");
  if (driver.motorControl.Enable()) {
    ESP_LOGI(TAG, "Motor enabled via driver API");
  }

  // Configure ramp control
  driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver.rampControl.SetTargetPosition(1000);
  driver.rampControl.SetMaxSpeed(1000.0F);
  driver.rampControl.SetAcceleration(500.0F);

  ESP_LOGI(TAG, "Motor configured for positioning mode");
  ESP_LOGI(TAG, "Target position: 1000 steps");

  // Monitor diagnostic pins during motion
  ESP_LOGI(TAG, "\n=== Monitoring Diagnostic Pins During Motion ===");
  int timeout = 0;
  while (!driver.rampControl.IsTargetReached() && timeout < 100) {
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read diagnostic pins
    if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG0, diag0_signal)) {
      if (diag0_signal == tmc5160::GpioSignal::ACTIVE) {
        ESP_LOGI(TAG, "DIAG0 triggered!");
      }
    }

    timeout++;
  }

  // Disable motor
  driver.motorControl.Disable();
  ESP_LOGI(TAG, "Motor disabled");

  ESP_LOGI(TAG, "\n=== GPIO Pin Configuration Example Complete ===");
  ESP_LOGI(TAG, "Summary:");
  ESP_LOGI(TAG, "  - All TMC5160 control pins can be configured via SetPinMapping()");
  ESP_LOGI(TAG, "  - Diagnostic pins (DIAG0, DIAG1) are read-only");
  ESP_LOGI(TAG, "  - Reference switch pins can be read/written depending on mode");
  ESP_LOGI(TAG, "  - Encoder pins can be read when configured");
  ESP_LOGI(TAG, "  - CLK pin can be configured for external clock");
  ESP_LOGI(TAG, "  - Pin functions depend on SPI_MODE and SD_MODE settings");
}
