/**
 * @file daisy_chain_example.cpp
 * @brief Example demonstrating daisy-chained TMC5160 drivers on a single SPI bus
 *
 * This example shows how to use multiple TMC5160 drivers on a single SPI bus
 * using daisy-chaining. All chips share the same CSN, SCK, and MOSI, while
 * MISO is daisy-chained.
 *
 * Note: For managing multiple devices, consider using TMC5160DaisyChain class
 * which provides automatic chain length configuration and sequential positioning.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - 2-3 TMC5160 stepper motor drivers (daisy-chained)
 * - Stepper motors connected to each TMC5160
 * - SPI connection: All chips share CSN, SCK, MOSI; MISO daisy-chained
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5 (shared by all chips)
 * - Control: EN=2, DIR=4, STEP=15 (shared by all chips, or separate per chip)
 *
 * Daisy-Chain Wiring:
 * - MCU MISO ──> Chip 1 SDO ──> Chip 2 SDI
 *                Chip 1 SDI <── MCU MOSI
 *                Chip 2 SDO ──> MCU MISO (if 2 chips) or Chip 3 SDI (if 3 chips)
 * - All chips: CSN, SCK, MOSI (SDI) tied together
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DaisyChainExample";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Daisy-Chain Example");
  ESP_LOGI(TAG, "This example demonstrates multiple TMC5160 drivers on a single SPI bus");

  // Create ONE SPI communication interface (shared by all chips)
  // All chips share the same CSN, SCK, MOSI; MISO is daisy-chained
  Esp32SPI spi(SPI2_HOST,
               GPIO_NUM_23, // MOSI (shared by all chips)
               GPIO_NUM_19, // MISO (daisy-chained: Chip1 SDO -> Chip2 SDI -> ...)
               GPIO_NUM_18, // SCLK (shared by all chips)
               GPIO_NUM_5,  // CS (shared by all chips - tied together)
               GPIO_NUM_2,  // EN (can be shared or separate per chip)
               GPIO_NUM_4,  // DIR (can be shared or separate per chip)
               GPIO_NUM_15, // STEP (can be shared or separate per chip)
               4000000);    // 4 MHz SPI clock

  // Initialize SPI interface
  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  ESP_LOGI(TAG, "SPI interface initialized");

  // Set chain length on SPI interface for proper response extraction
  // This enables the datasheet formula 40·(n-k+1) for optimal performance
  // IMPORTANT: This must match the actual number of devices in the physical chain
  spi.SetDaisyChainLength(2); // 2 devices in chain (update if adding more)

  // Create multiple TMC5160 driver instances
  // Each instance has its own daisy-chain position
  // Position 0 = first chip, Position 1 = second chip, etc.
  // IMPORTANT: Positions must be sequential and match physical chain order
  tmc5160::TMC5160 driver1(spi, 12'000'000, 0); // First chip (position 0)
  tmc5160::TMC5160 driver2(spi, 12'000'000, 1); // Second chip (position 1)
  // Uncomment for third chip:
  // tmc5160::TMC5160 driver3(spi, 12'000'000, 2); // Third chip (position 2)
  // Remember to update SetDaisyChainLength(3) if adding third chip

  ESP_LOGI(TAG, "Created TMC5160 driver instances:");
  ESP_LOGI(TAG, "  Driver 1: Position 0 (first chip)");
  ESP_LOGI(TAG, "  Driver 2: Position 1 (second chip)");

  // Configure driver settings
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;          // Run current (0-31)
  cfg.motor.ihold = 10;         // Hold current (0-31)
  cfg.motor.global_scaler = 32; // Global current scaler (32-256)
  cfg.chopper.toff = 5;         // Chopper off time
  cfg.chopper.mres = 4;         // 16 microsteps
  cfg.chopper.intpol = true;    // Enable interpolation

  // Initialize each driver
  // Each driver automatically uses its own daisy-chain position
  if (!driver1.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver 1 (position 0)");
    return;
  }
  ESP_LOGI(TAG, "Driver 1 initialized (position 0)");

  if (!driver2.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize driver 2 (position 1)");
    return;
  }
  ESP_LOGI(TAG, "Driver 2 initialized (position 1)");

  // Configure ramp control for each driver
  driver1.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver1.rampControl.SetTargetPosition(1000);  // Move 1000 steps
  driver1.rampControl.SetMaxSpeed(1000.0f);     // 1000 steps/s
  driver1.rampControl.SetAcceleration(500.0f);  // 500 steps/s²

  driver2.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
  driver2.rampControl.SetTargetPosition(2000);  // Move 2000 steps
  driver2.rampControl.SetMaxSpeed(1000.0f);     // 1000 steps/s
  driver2.rampControl.SetAcceleration(500.0f);  // 500 steps/s²

  ESP_LOGI(TAG, "Ramp control configured for both drivers");

  // Enable motors
  driver1.motorControl.Enable();
  driver2.motorControl.Enable();
  ESP_LOGI(TAG, "Motors enabled");

  // Monitor motion
  ESP_LOGI(TAG, "Monitoring motion...");
  for (int i = 0; i < 50; ++i) {
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay

    int32_t pos1 = driver1.rampControl.GetCurrentPosition();
    int32_t pos2 = driver2.rampControl.GetCurrentPosition();

    bool reached1 = driver1.rampControl.IsTargetReached();
    bool reached2 = driver2.rampControl.IsTargetReached();

    ESP_LOGI(TAG, "Driver 1: Position=%ld, Target Reached=%s", pos1,
             reached1 ? "YES" : "NO");
    ESP_LOGI(TAG, "Driver 2: Position=%ld, Target Reached=%s", pos2,
             reached2 ? "YES" : "NO");

    if (reached1 && reached2) {
      ESP_LOGI(TAG, "Both drivers reached their targets!");
      break;
    }
  }

  // Demonstrate reading from multiple chips
  ESP_LOGI(TAG, "Reading status from both chips...");

  // Read GSTAT register from each chip individually
  uint32_t gstat1 = 0;
  uint32_t gstat2 = 0;

  if (driver1.GetComm().ReadRegister(0x00, gstat1, 0)) {
    ESP_LOGI(TAG, "Chip 0 GSTAT: 0x%08lX", gstat1);
  } else {
    ESP_LOGE(TAG, "Failed to read from chip 0");
  }

  if (driver2.GetComm().ReadRegister(0x00, gstat2, 1)) {
    ESP_LOGI(TAG, "Chip 1 GSTAT: 0x%08lX", gstat2);
  } else {
    ESP_LOGE(TAG, "Failed to read from chip 1");
  }

  // Disable motors
  driver1.motorControl.Disable();
  driver2.motorControl.Disable();
  ESP_LOGI(TAG, "Motors disabled");

  ESP_LOGI(TAG, "Example completed successfully");
}

