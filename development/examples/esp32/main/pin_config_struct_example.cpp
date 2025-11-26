/**
 * @file pin_config_struct_example.cpp
 * @brief Example demonstrating TMC5160PinConfig struct for GPIO pin configuration
 *
 * This example shows how to use the TMC5160PinConfig struct to configure
 * all GPIO pins in a single place, with automatic handling of compound pins
 * (pins that share the same physical GPIO).
 *
 * ESP32-C6 Pin Configuration:
 * - SCK: GPIO5
 * - MOSI: GPIO6
 * - CLK16: GPIO10 (external clock)
 * - DRV_EN: GPIO11
 * - MISO: GPIO12
 * - CS_TMC: GPIO18
 * - DIAG0: GPIO20 (example)
 * - DIAG1: GPIO21 (example)
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

static const char* TAG = "PinConfigStruct";

//=============================================================================
// CONFIGURATION SELECTION - Change these to select motor, board, and platform
//=============================================================================
// Motor selection (compile-time constant)
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;

// Board selection (compile-time constant)
static constexpr tmc5160_test_config::BoardType SELECTED_BOARD = 
    tmc5160_test_config::BoardType::BOARD_TMC5160_EVAL;

// Platform selection (compile-time constant)
static constexpr tmc5160_test_config::PlatformType SELECTED_PLATFORM = 
    tmc5160_test_config::PlatformType::PLATFORM_TEST_RIG;

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Pin Configuration Struct Example");
  ESP_LOGI(TAG, "Demonstrating automatic compound pin handling");

  // Method 1: Using TMC5160PinConfig struct (recommended)
  // Compound pins are automatically handled - you only specify the GPIO once
  tmc5160::TMC5160PinConfig pin_config{};
  pin_config.en_pin = GPIO_NUM_11;    // EN pin (required)
  pin_config.diag0_pin = GPIO_NUM_20; // DIAG0 pin
  pin_config.diag1_pin = GPIO_NUM_21; // DIAG1 pin
  pin_config.clk_pin = GPIO_NUM_10;   // CLK pin

  // For compound pins, you only need to specify one - the other is automatically mapped
  // Example: If you specify step_pin, ref_left_pin is automatically set to the same GPIO
  pin_config.step_pin = GPIO_NUM_4; // This automatically maps REFL_STEP to GPIO4 as well

  // Or specify ref_left_pin instead - STEP will be automatically mapped
  // pin_config.ref_left_pin = GPIO_NUM_4; // Same result as above

  // Same for encoder/DC Step pins
  pin_config.enc_a_pin = GPIO_NUM_8;  // This automatically maps DCIN to GPIO8 as well
  pin_config.enc_b_pin = GPIO_NUM_9;  // This automatically maps DCEN to GPIO9 as well
  pin_config.enc_n_pin = GPIO_NUM_10; // This automatically maps DCO to GPIO10 as well

  // Create SPI interface with pin configuration struct
  Esp32SPI spi(SPI2_HOST,
               GPIO_NUM_6,  // MOSI
               GPIO_NUM_12, // MISO
               GPIO_NUM_5,  // SCLK
               GPIO_NUM_18, // CS_TMC
               pin_config,  // Pin configuration struct
               4000000);    // 4 MHz SPI clock

  ESP_LOGI(TAG, "Pin configuration applied:");
  ESP_LOGI(TAG, "  EN: GPIO%d", pin_config.en_pin);
  ESP_LOGI(TAG, "  STEP/REFL_STEP: GPIO%d (compound pin)", pin_config.step_pin);
  ESP_LOGI(TAG, "  ENCA/DCIN: GPIO%d (compound pin)", pin_config.enc_a_pin);
  ESP_LOGI(TAG, "  ENCB/DCEN: GPIO%d (compound pin)", pin_config.enc_b_pin);
  ESP_LOGI(TAG, "  ENCN/DCO: GPIO%d (compound pin)", pin_config.enc_n_pin);
  ESP_LOGI(TAG, "  DIAG0: GPIO%d", pin_config.diag0_pin);
  ESP_LOGI(TAG, "  DIAG1: GPIO%d", pin_config.diag1_pin);
  ESP_LOGI(TAG, "  CLK: GPIO%d", pin_config.clk_pin);

  // Verify compound pin mappings
  ESP_LOGI(TAG, "\nVerifying compound pin mappings:");
  gpio_num_t step_pin = spi.GetPinMapping(tmc5160::TMC5160CtrlPin::STEP);
  gpio_num_t ref_left_pin = spi.GetPinMapping(tmc5160::TMC5160CtrlPin::REFL_STEP);
  ESP_LOGI(TAG, "  STEP: GPIO%d, REFL_STEP: GPIO%d (should be same)", step_pin, ref_left_pin);

  gpio_num_t enca_pin = spi.GetPinMapping(tmc5160::TMC5160CtrlPin::ENCA);
  gpio_num_t dcin_pin = spi.GetPinMapping(tmc5160::TMC5160CtrlPin::DCIN);
  ESP_LOGI(TAG, "  ENCA: GPIO%d, DCIN: GPIO%d (should be same)", enca_pin, dcin_pin);

  // Initialize SPI interface
  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }
  ESP_LOGI(TAG, "SPI interface initialized");

  // Create TMC5160 driver instance
  tmc5160::TMC5160<Esp32SPI> driver(spi);

  // Configure driver using helper functions
  tmc5160::DriverConfig cfg{};
  
  // Configure motor
  if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Direct(cfg);
  } else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    tmc5160_test_config::ConfigureDriverFromMotor_AppliedMotion_5034(cfg);
  }
  
  // Apply board configuration
  tmc5160_test_config::ApplyBoardConfig<SELECTED_BOARD>(cfg);
  
  // Apply platform configuration
  tmc5160_test_config::ApplyPlatformConfig<SELECTED_PLATFORM>(cfg);
  
  // Override microstep resolution if needed
  cfg.chopper.mres = 4; // 16 microsteps

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }
  ESP_LOGI(TAG, "Driver initialized successfully");

  // Method 2: Override individual pins if needed (for custom routing)
  // This is useful if you need to reroute a compound pin to a different GPIO
  ESP_LOGI(TAG, "\n=== Overriding Individual Pin Mapping ===");
  ESP_LOGI(TAG, "Example: Rerouting DCIN to a different GPIO than ENCA");
  spi.SetPinMapping(tmc5160::TMC5160CtrlPin::DCIN, GPIO_NUM_7); // Override DCIN to GPIO7
  ESP_LOGI(TAG, "  ENCA: GPIO%d, DCIN: GPIO%d (now different)", enca_pin,
           spi.GetPinMapping(tmc5160::TMC5160CtrlPin::DCIN));

  // Method 3: Using constructor with helper (for simple configurations)
  ESP_LOGI(TAG, "\n=== Using Constructor Helper ===");
  tmc5160::TMC5160PinConfig simple_config(GPIO_NUM_11, // EN pin
                                          GPIO_NUM_4,  // DIR pin (also maps REFR_DIR)
                                          GPIO_NUM_5); // STEP pin (also maps REFL_STEP)

  ESP_LOGI(TAG, "Simple config created:");
  ESP_LOGI(TAG, "  EN: GPIO%d", simple_config.en_pin);
  ESP_LOGI(TAG, "  DIR/REFR_DIR: GPIO%d (compound)", simple_config.dir_pin);
  ESP_LOGI(TAG, "  STEP/REFL_STEP: GPIO%d (compound)", simple_config.step_pin);

  ESP_LOGI(TAG, "\n=== Summary ===");
  ESP_LOGI(TAG, "✓ TMC5160PinConfig struct simplifies pin configuration");
  ESP_LOGI(TAG, "✓ Compound pins are automatically handled");
  ESP_LOGI(TAG, "✓ SetPinMapping() still available for custom routing");
  ESP_LOGI(TAG, "✓ Legacy constructor still supported for backward compatibility");
}
