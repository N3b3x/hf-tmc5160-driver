---
layout: default
title: "GPIO Pin Configuration"
description: "Pin mapping, active levels, and mode pin setup for TMC51x0 control pins"
nav_order: 15
parent: "📚 Documentation"
permalink: /docs/gpio-pin-configuration/
---

# GPIO Pin Configuration Guide

## Overview

The TMC51x0 driver (TMC5130 & TMC5160) supports comprehensive GPIO pin configuration for all TMC51x0 control pins. Pin functions depend on the mode configuration (SPI_MODE, SD_MODE) as described in the TMC51x0 datasheet section 2.2.

## Supported Control Pins

### Basic Control Pins
- **EN** (DRV_ENN, pin 28): Enable pin - Active HIGH disables power stage (inverted logic)
- **DIR** (REFR_DIR, pin 18): Direction pin - DIR input when SD_MODE=1, Right reference when SD_MODE=0
- **STEP** (REFL_STEP, pin 17): Step pin - STEP input when SD_MODE=1, Left reference when SD_MODE=0

### Reference Switch Pins (when SD_MODE=0, internal ramp generator mode)
- **REFL_STEP**: Left reference switch input (same physical pin as STEP)
- **REFR_DIR**: Right reference switch input (same physical pin as DIR)

### Diagnostic Output Pins / UART Pins (mode-dependent)
- **DIAG0** (DIAG0_SWN, pin 26): 
  - **SPI_MODE=1, SD_MODE=0**: Diagnostic output 0 - Interrupt/STEP output
  - **SPI_MODE=0, SD_MODE=0**: Single wire I/O (negative) - Use with DIAG1_SWP for RS485 differential bus
- **DIAG1** (DIAG1_SWP, pin 27):
  - **SPI_MODE=1, SD_MODE=0**: Diagnostic output 1 - Position-compare/DIR output
  - **SPI_MODE=0, SD_MODE=0**: Single wire I/O (positive) - Use alone for single wire UART, or with DIAG0_SWN for RS485 bus

### Encoder Input Pins (when SD_MODE=0, internal ramp generator mode)
- **ENCA** (ENCA_DCIN_CFG5, pin 24): Encoder A-channel input
- **ENCB** (ENCB_DCEN_CFG4, pin 23): Encoder B-channel input
- **ENCN** (ENCN_DCO_CFG6, pin 25): Encoder N-channel input

### DC Step Control Pins (when SD_MODE=1, SPI_MODE=1, external step/dir mode)
**Note:** These are the same physical pins as encoder pins but used for DC Step control:
- **DCIN** (ENCA_DCIN_CFG5, pin 24): DC Step gating input for axis synchronization
- **DCEN** (ENCB_DCEN_CFG4, pin 23): DC Step enable input (leave open or tie to GND for normal operation)
- **DCO** (ENCN_DCO_CFG6, pin 25): DC Step ready output

### Clock Pin
- **CLK** (CLK, pin 12): Clock input - External clock input (tie to GND for internal clock)

### Mode Configuration Pins (Advanced - if available as control pins)
- **SPI_MODE** (pin 22): SPI/UART mode select - HIGH=SPI, LOW=UART. Typically hardwired.
- **SD_MODE** (pin 21): Step/Dir mode select - HIGH=External step/dir, LOW=Internal ramp. Typically hardwired.

**⚠️ WARNING**: These pins are typically hardwired and read at startup. Only configure these if you have connected SPI_MODE (pin 22) and SD_MODE (pin 21) to GPIO outputs for dynamic mode control. Changing these pins requires a chip reset to take effect.

## Pin Function Dependencies

### Mode Configuration

**SPI_MODE=1, SD_MODE=0** (Internal ramp generator mode - motion controller):
  - REFL_STEP, REFR_DIR: Reference switch inputs
  - ENCA, ENCB, ENCN: Encoder inputs
  - DIAG0, DIAG1: Diagnostic outputs
  - DIR, STEP: Not used (use -1 in constructor)
  - DCEN, DCIN, DCO: Not used (encoder pins take precedence)

**SPI_MODE=1, SD_MODE=1** (External step/dir mode):
  - REFL_STEP, REFR_DIR: STEP/DIR inputs (same physical pins as reference switches)
  - DCIN, DCEN, DCO: DC Step control pins (same physical pins as encoder pins)
  - DIAG0, DIAG1: Diagnostic outputs
  - ENCA, ENCB, ENCN: Not used (DC Step pins take precedence)

**SPI_MODE=0, SD_MODE=0** (UART single wire interface mode):
  - **DIAG1_SWP (pin 27)**: Single wire I/O (positive) - Use SWP for single wire UART
  - **DIAG0_SWN (pin 26)**: Single wire I/O (negative) - Use SWP+SWN for RS485 differential bus
  - **SDI_CFG1 (pin 15)**: Next address input (NAI) for sequential addressing
  - **SDO_CFG0 (pin 16)**: Next address output (NAO) for sequential addressing
  - **CSN_CFG3, SCK_CFG2, ENCB_CFG4, ENCA_CFG5**: Configuration inputs (CFG functions)
  - Other pins: Not used for communication in UART mode

## ESP32-C6 Pin Configuration Example

Based on your ESP32-C6 pinout:
- SCK: GPIO5
- MOSI: GPIO6
- CLK16: GPIO10 (external clock)
- DRV_EN: GPIO11
- MISO: GPIO12
- CS_TMC: GPIO18
- DIAG0, DIAG1: (configure based on your board)

### Method 1: Using TMC51x0PinConfig Struct (Recommended)

The `TMC51x0PinConfig` struct simplifies pin configuration and automatically handles compound pins:

```cpp
#include "esp32_tmc5160_bus.hpp"
#include "../../../inc/tmc51x0.hpp"

// Create pin configuration struct
tmc51x0::TMC51x0PinConfig pin_config{};
pin_config.en_pin = GPIO_NUM_11;    // EN pin (required)
pin_config.diag0_pin = GPIO_NUM_20; // DIAG0 pin
pin_config.diag1_pin = GPIO_NUM_21; // DIAG1 pin
pin_config.clk_pin = GPIO_NUM_10;   // CLK pin

// Compound pins: specify one, the other is automatically mapped
pin_config.step_pin = GPIO_NUM_4;   // Automatically maps REFL_STEP to GPIO4 as well
pin_config.enc_a_pin = GPIO_NUM_8;  // Automatically maps DCIN to GPIO8 as well

// Create SPI interface with pin configuration struct
Esp32SPI spi(SPI2_HOST,
             GPIO_NUM_6,   // MOSI
             GPIO_NUM_12,  // MISO
             GPIO_NUM_5,   // SCLK
             GPIO_NUM_18,  // CS_TMC
             pin_config,   // Pin configuration struct
             4000000);     // 4 MHz SPI clock
```

**Benefits:**
- All pins configured in one place
- Compound pins automatically handled
- Cleaner, more maintainable code

### Method 2: Using Legacy Constructor (Backward Compatible)

```cpp
// Create SPI interface with ESP32-C6 pin configuration
Esp32SPI spi(SPI2_HOST,
             GPIO_NUM_6,                          // MOSI
             GPIO_NUM_12,                         // MISO
             GPIO_NUM_5,                          // SCLK
             GPIO_NUM_18,                         // CS_TMC
             GPIO_NUM_11,                         // DRV_EN (EN pin)
             static_cast<gpio_num_t>(-1),         // DIR (not used in SPI mode with internal ramp generator)
             static_cast<gpio_num_t>(-1),         // STEP (not used in SPI mode with internal ramp generator)
             4000000);                            // 4 MHz SPI clock

// Configure additional pins
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG0, GPIO_NUM_XX); // Replace XX with your GPIO
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG1, GPIO_NUM_XX); // Replace XX with your GPIO
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::CLK, GPIO_NUM_10);   // CLK pin
```

### Method 3: Override Individual Pins (Custom Routing)

If you need to reroute a compound pin to a different GPIO:

```cpp
// After applying pin config, override individual pins if needed
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DCIN, GPIO_NUM_7); // Reroute DCIN to GPIO7 (different from ENCA)
```

## API Reference

### TMC51x0PinConfig Struct

Pin configuration structure that handles all GPIO pin assignments:

```cpp
struct TMC51x0PinConfig {
  // Basic control pins
  int en_pin{-1};   // EN pin (required)
  int dir_pin{-1};  // DIR pin (same as ref_right_pin)
  int step_pin{-1}; // STEP pin (same as ref_left_pin)

  // Reference switch pins (when SD_MODE=0)
  int ref_left_pin{-1};  // Left reference switch (same as step_pin)
  int ref_right_pin{-1}; // Right reference switch (same as dir_pin)

  // Diagnostic pins
  int diag0_pin{-1}; // DIAG0 pin
  int diag1_pin{-1}; // DIAG1 pin

  // Encoder pins (when SD_MODE=0)
  int enc_a_pin{-1}; // Encoder A (same as dc_in_pin)
  int enc_b_pin{-1}; // Encoder B (same as dc_en_pin)
  int enc_n_pin{-1}; // Encoder N (same as dc_out_pin)

  // DC Step pins (when SD_MODE=1, SPI_MODE=1)
  int dc_in_pin{-1};  // DC Step gating input (same as enc_a_pin)
  int dc_en_pin{-1};  // DC Step enable input (same as enc_b_pin)
  int dc_out_pin{-1}; // DC Step ready output (same as enc_n_pin)

  // Clock pin
  int clk_pin{-1}; // Clock input

  // Constructors
  TMC51x0PinConfig(); // Default: all pins unmapped
  TMC51x0PinConfig(int en, int dir = -1, int step = -1); // Helper for basic pins
};
```

**Compound Pin Relationships:**
- Pin 17: `step_pin` ↔ `ref_left_pin` (REFL_STEP/STEP)
- Pin 18: `dir_pin` ↔ `ref_right_pin` (REFR_DIR/DIR)
- Pin 23: `enc_b_pin` ↔ `dc_en_pin` (ENCB/DCEN)
- Pin 24: `enc_a_pin` ↔ `dc_in_pin` (ENCA/DCIN)
- Pin 25: `enc_n_pin` ↔ `dc_out_pin` (ENCN/DCO)

**Usage:**
```cpp
// Simple configuration
tmc51x0::TMC51x0PinConfig config(GPIO_NUM_11, GPIO_NUM_4, GPIO_NUM_5);

// Full configuration
tmc51x0::TMC51x0PinConfig config{};
config.en_pin = GPIO_NUM_11;
config.step_pin = GPIO_NUM_4;  // Automatically maps ref_left_pin
config.enc_a_pin = GPIO_NUM_8; // Automatically maps dc_in_pin
```

### ApplyPinConfig()
Apply a pin configuration structure (automatically handles compound pins):

```cpp
bool ApplyPinConfig(const TMC51x0PinConfig& pin_config);
```

**Parameters:**
- `pin_config`: Pin configuration structure

**Returns:** `true` if configuration was applied successfully

**Notes:**
- Automatically maps compound pins to the same GPIO
- Can be called multiple times to update configuration
- Individual pins can still be overridden with `SetPinMapping()`

### SetPinMapping()
Configure a TMC5160 control pin to an ESP32 GPIO pin (or override compound pin mappings).

```cpp
bool SetPinMapping(tmc51x0::TMC51x0CtrlPin pin, gpio_num_t gpio_pin);
```

**Parameters:**
- `pin`: TMC5160 control pin identifier (e.g., `TMC51x0CtrlPin::DIAG0`)
- `gpio_pin`: ESP32 GPIO pin number, or `-1` to disable/unmap

**Returns:** `true` if pin mapping was set successfully

**Notes:**
- Automatically configures GPIO direction based on pin type:
  - Diagnostic pins (DIAG0, DIAG1): Input with pullup
  - Encoder pins (ENCA, ENCB, ENCN): Input with pullup
  - Reference switch pins (REFL_STEP, REFR_DIR): Input with pullup, active LOW
  - CLK pin: Input (can be configured as output for PWM clock if needed)
  - Other pins: Output

### GetPinMapping()
Query the GPIO pin mapping for a TMC5160 control pin.

```cpp
gpio_num_t GetPinMapping(tmc51x0::TMC51x0CtrlPin pin) const;
```

**Parameters:**
- `pin`: TMC5160 control pin identifier

**Returns:** ESP32 GPIO pin number, or `-1` if not mapped

### GpioSet()
Set a GPIO pin to active or inactive state.

```cpp
bool GpioSet(tmc51x0::TMC51x0CtrlPin pin, tmc51x0::GpioSignal signal);
```

**Parameters:**
- `pin`: TMC5160 control pin identifier
- `signal`: `GpioSignal::ACTIVE` or `GpioSignal::INACTIVE`

**Returns:** `true` if GPIO was set successfully

**Notes:**
- Diagnostic pins (DIAG0, DIAG1) are read-only and cannot be set
- Pin must be mapped using `SetPinMapping()` before use
- Respects pin active level configuration (e.g., EN is active HIGH to disable)

### GpioRead()
Read the current state of a GPIO pin.

```cpp
bool GpioRead(tmc51x0::TMC51x0CtrlPin pin, tmc51x0::GpioSignal& signal);
```

**Parameters:**
- `pin`: TMC5160 control pin identifier
- `signal`: Reference to store the current signal state

**Returns:** `true` if GPIO was read successfully

**Notes:**
- Supports reading diagnostic pins, reference switch pins, encoder pins, and CLK pin
- Pin must be mapped using `SetPinMapping()` before use

## Pin Active Level Configuration

The active level for each pin can be configured to match your hardware:

```cpp
// EN pin is active HIGH to disable (inverted logic for DRV_ENN)
spi.SetPinActiveLevel(tmc51x0::TMC51x0CtrlPin::EN, true);

// Reference switches are typically active LOW
spi.SetPinActiveLevel(tmc51x0::TMC51x0CtrlPin::REFL_STEP, false);
spi.SetPinActiveLevel(tmc51x0::TMC51x0CtrlPin::REFR_DIR, false);
```

## CLK Pin Configuration

The CLK pin (GPIO10 on your ESP32-C6) can be used for external clock input:

```cpp
// Configure CLK pin
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::CLK, GPIO_NUM_10);

// Read CLK pin state
tmc51x0::GpioSignal clk_signal;
if (spi.GpioRead(tmc51x0::TMC51x0CtrlPin::CLK, clk_signal)) {
    // Process clock signal
}
```

**Note:** For PWM clock generation, you would need to configure the GPIO as output and use ESP-IDF's LEDC (LED PWM Controller) or MCPWM (Motor Control PWM) peripherals. The driver provides the GPIO interface; PWM configuration is handled by your application code.

## Diagnostic Pins

Diagnostic pins provide real-time status information:

- **DIAG0**: Interrupt or STEP output (when SD_MODE=0)
- **DIAG1**: Position-compare or DIR output (when SD_MODE=0)

These pins are read-only and can be monitored continuously:

```cpp
// Monitor diagnostic pins
tmc51x0::GpioSignal diag0, diag1;
while (true) {
    if (spi.GpioRead(tmc51x0::TMC51x0CtrlPin::DIAG0, diag0)) {
        if (diag0 == tmc51x0::GpioSignal::ACTIVE) {
            // Handle DIAG0 event
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

## Reference Switch Pins

Reference switch pins (REFL_STEP, REFR_DIR) are used for endstop detection when using the internal ramp generator:

```cpp
// Configure reference switch pins
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::REFL_STEP, GPIO_NUM_4);
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::REFR_DIR, GPIO_NUM_7);

// Read reference switch states
tmc51x0::GpioSignal left_ref, right_ref;
spi.GpioRead(tmc51x0::TMC51x0CtrlPin::REFL_STEP, left_ref);
spi.GpioRead(tmc51x0::TMC51x0CtrlPin::REFR_DIR, right_ref);

// Configure reference switches in driver
tmc51x0::ReferenceSwitchConfig ref_cfg{};
ref_cfg.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
ref_cfg.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
ref_cfg.latch_left = tmc51x0::ReferenceLatchMode::DISABLED;
ref_cfg.latch_right = tmc51x0::ReferenceLatchMode::DISABLED;
driver.switches.ConfigureReferenceSwitch(ref_cfg);
```

## Encoder Pins

Encoder pins are used for closed-loop control when using the internal ramp generator (SD_MODE=0):

```cpp
// Configure encoder pins (when SD_MODE=0)
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::ENCA, GPIO_NUM_8);
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::ENCB, GPIO_NUM_9);
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::ENCN, GPIO_NUM_10);

// Configure encoder in driver
tmc51x0::EncoderConfig enc_cfg{};
enc_cfg.prescaler_mode = tmc51x0::EncoderPrescalerMode::BINARY;
driver.encoder.Configure(enc_cfg);
driver.encoder.SetResolution(200, 1000, false);
```

**Note:** These are the same physical pins as DC Step pins. The function depends on SD_MODE.

## DC Step Pins

DC Step pins are used for DC Step control when using external step/dir mode (SD_MODE=1, SPI_MODE=1):

```cpp
// Configure DC Step pins (when SD_MODE=1, SPI_MODE=1)
// Note: These are the same physical pins as encoder pins
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DCIN, GPIO_NUM_8);  // DC Step gating input
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DCEN, GPIO_NUM_9);  // DC Step enable input
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DCO, GPIO_NUM_10); // DC Step ready output

// Control DC Step enable
spi.GpioSet(tmc51x0::TMC51x0CtrlPin::DCEN, tmc51x0::GpioSignal::ACTIVE); // Enable DC Step
spi.GpioSet(tmc51x0::TMC51x0CtrlPin::DCIN, tmc51x0::GpioSignal::ACTIVE); // Gate DC Step

// Read DC Step ready output
tmc51x0::GpioSignal dco_ready;
if (spi.GpioRead(tmc51x0::TMC51x0CtrlPin::DCO, dco_ready)) {
    if (dco_ready == tmc51x0::GpioSignal::ACTIVE) {
        // DC Step is ready
    }
}
```

**Note:** 
- DCEN and DCIN are inputs to TMC5160 (can be controlled via `GpioSet()`)
- DCO is an output from TMC5160 (read-only via `GpioRead()`)
- These pins share the same physical pins as encoder pins (ENCA, ENCB, ENCN)

## UART Mode Pin Configuration

When using UART mode (SPI_MODE=0, SD_MODE=0), DIAG0 and DIAG1 become UART communication pins:

### Single Wire UART
For single wire UART communication, use only DIAG1_SWP (SWP - positive):

```cpp
// Configure for single wire UART mode
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG1, GPIO_NUM_TX); // UART TX/RX (SWP)
// DIAG0 not needed for single wire mode
```

### RS485 Differential Bus
For RS485 differential bus communication, use both DIAG1_SWP (SWP) and DIAG0_SWN (SWN):

```cpp
// Configure for RS485 differential bus
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG1, GPIO_NUM_A); // RS485 A (SWP - positive)
spi.SetPinMapping(tmc51x0::TMC51x0CtrlPin::DIAG0, GPIO_NUM_B); // RS485 B (SWN - negative)
```

**Note:** In UART mode, the UART communication interface (`UartCommInterface`) handles the actual UART protocol. The GPIO pin mapping is for hardware connection reference only. The UART driver manages the communication pins automatically.

## Best Practices

1. **Pin Mapping**: Always configure pins using `SetPinMapping()` before use
2. **Active Levels**: Configure pin active levels to match your hardware (especially for EN pin which is inverted)
3. **Diagnostic Monitoring**: Use diagnostic pins for real-time error detection (SPI mode)
4. **Reference Switches**: Configure reference switches for safe operation
5. **Mode Awareness**: Understand which pins are active in your mode configuration (SPI_MODE, SD_MODE)

## Software Control of Mode Pins (Advanced)

If SPI_MODE (pin 22) and SD_MODE (pin 21) are connected to GPIO outputs instead of being hardwired, you can control the chip communication mode programmatically.

### Configuration

```cpp
#include "esp32_tmc5160_bus.hpp"
#include "../../../inc/tmc51x0.hpp"

// Configure pin config with mode pins
tmc51x0::TMC51x0PinConfig pin_config{};
pin_config.en_pin = GPIO_NUM_11;
pin_config.spi_mode_pin = GPIO_NUM_22;  // SPI_MODE pin (if available as GPIO)
pin_config.sd_mode_pin = GPIO_NUM_21;   // SD_MODE pin (if available as GPIO)

// Create SPI interface with pin config
Esp32SPI spi(SPI2_HOST, 
             GPIO_NUM_6,  // MOSI
             GPIO_NUM_2,  // MISO
             GPIO_NUM_5,  // SCK
             GPIO_NUM_18, // CS
             pin_config,
             4000000);

// Initialize SPI
spi.Initialize();

// Create driver
tmc51x0::TMC51x0<Esp32SPI> driver(spi);
```

### Changing Communication Mode

```cpp
// Set mode to SPI + Internal Ramp Generator
if (driver.io.SetOperatingMode(tmc51x0::ChipCommMode::SPI_INTERNAL_RAMP)) {
  ESP_LOGI(TAG, "Mode set to SPI + Internal Ramp");
  // ⚠️ CRITICAL: Must reset chip for mode change to take effect
  // The mode pins are read at startup, so changes require a reset
  // You must handle chip reset externally (power cycle or reset pin)
}

// Read current mode
auto current_mode_result = driver.io.GetOperatingMode();
if (current_mode_result) {
  ESP_LOGI(TAG, "Current mode: %d", static_cast<int>(current_mode_result.Value()));
}
```

### Important Warnings

⚠️ **CRITICAL REQUIREMENTS:**
- Mode pins are **read at startup** - changes require a chip reset to take effect
- You must reset the chip (power cycle or reset pin) after changing mode pins
- The driver does **NOT** automatically reset the chip - you must handle this externally
- Ensure pins are configured as outputs in `TMC51x0PinConfig` before use
- These pins are typically hardwired - only use software control if connected to GPIO

### Mode Pin States

| Mode | SPI_MODE (pin 22) | SD_MODE (pin 21) |
|------|-------------------|------------------|
| `SPI_INTERNAL_RAMP` | HIGH (1) | LOW (0) |
| `SPI_EXTERNAL_STEPDIR` | HIGH (1) | HIGH (1) |
| `UART_INTERNAL_RAMP` | LOW (0) | LOW (0) |
6. **UART Mode**: In UART mode, DIAG0/DIAG1 become UART pins - use SWP alone for single wire, or SWP+SWN for RS485

## See Also

- `examples/esp32/main/test_config/esp32_tmc5160_bus.hpp` - ESP32 SPI implementation with full GPIO support
- TMC5160 Datasheet Section 2.2 - Pin function descriptions

---

**Navigation**
[← Communication Interface](special_features_communication_interface.md) | [Back to Index](index.md) | [Next: Register Access →](special_features_register_access.md)

