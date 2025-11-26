# GPIO Pin Configuration Example

## Overview

The `gpio_pin_config_example.cpp` demonstrates how to configure and use all TMC5160 control pins on ESP32, including diagnostic pins, reference switches, encoder pins, and the CLK pin.

## Purpose

This example is ideal for:
- Understanding GPIO pin configuration
- Learning how to set up diagnostic pins
- Configuring reference switches (endstops)
- Setting up encoder connections
- Understanding CLK pin usage
- Testing pin functionality

## Key Features

- **Complete Pin Configuration**: Shows all available TMC5160 control pins
- **Diagnostic Pins**: DIAG0 and DIAG1 configuration and reading
- **Reference Switches**: Left and right reference switch setup
- **Encoder Pins**: Encoder signal configuration
- **CLK Pin**: External clock configuration
- **GPIO Reading**: Demonstrates reading pin states

## Hardware Requirements

- ESP32-C6 development board (or other ESP32 variant)
- TMC5160 stepper motor driver board
- Stepper motor connected to TMC5160
- SPI connection between ESP32 and TMC5160
- Optional: Reference switches, encoder, external clock source

## Pin Configuration

### ESP32-C6 Default Configuration

- **SPI**:
  - SCK: GPIO5
  - MOSI: GPIO6
  - MISO: GPIO12
  - CS_TMC: GPIO18

- **Control**:
  - DRV_EN: GPIO11
  - CLK16: GPIO10 (external clock, optional)

- **Diagnostic** (optional):
  - DIAG0: Configure based on your board
  - DIAG1: Configure based on your board

- **Reference Switches** (optional):
  - REFL_STEP: Configure based on your board
  - REFR_DIR: Configure based on your board

- **Encoder** (optional):
  - ENC_A: Configure based on your board
  - ENC_B: Configure based on your board
  - ENC_N: Configure based on your board

## How It Works

### Pin Configuration Methods

The example demonstrates two methods for configuring pins:

#### Method 1: Individual Pin Specification

```cpp
Esp32SPI spi(SPI2_HOST,
             GPIO_NUM_6,                  // MOSI
             GPIO_NUM_12,                 // MISO
             GPIO_NUM_5,                  // SCLK
             GPIO_NUM_18,                 // CS_TMC
             GPIO_NUM_11,                 // DRV_EN
             static_cast<gpio_num_t>(-1), // DIR (not used)
             static_cast<gpio_num_t>(-1), // STEP (not used)
             4000000);                    // SPI clock
```

#### Method 2: Pin Configuration Struct (Recommended)

```cpp
tmc5160::Esp32SpiPinConfig pin_config{};
pin_config.spi_mosi = 6;
pin_config.spi_miso = 12;
pin_config.spi_sclk = 5;
pin_config.spi_cs = 18;
pin_config.tmc5160_pins.en_pin = 11;
pin_config.tmc5160_pins.clk_pin = 10;
pin_config.tmc5160_pins.diag0_pin = 20;
pin_config.tmc5160_pins.diag1_pin = 21;

Esp32SPI spi(SPI2_HOST, pin_config, 4000000);
```

### Diagnostic Pins

Diagnostic pins provide real-time status information:

- **DIAG0**: Can indicate:
  - Driver errors (overtemperature, short circuit)
  - Overtemperature prewarning
  - Stall detection (if configured)
  - Reset status

- **DIAG1**: Can indicate:
  - Stall direction
  - Index pulse (encoder)
  - On-state detection
  - Steps skipped

### Reading Pin States

```cpp
tmc5160::GpioSignal diag0_signal;
if (spi.GpioRead(tmc5160::TMC5160CtrlPin::DIAG0, diag0_signal)) {
    ESP_LOGI(TAG, "DIAG0: %s", 
             diag0_signal == tmc5160::GpioSignal::ACTIVE ? "HIGH" : "LOW");
}
```

### Reference Switches

Reference switches (endstops) can be configured for homing:

```cpp
tmc5160::ReferenceSwitchConfig ref_cfg{};
ref_cfg.left_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;
ref_cfg.right_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;
ref_cfg.latch_left = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;
ref_cfg.latch_right = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;
ref_cfg.latch_left = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;
ref_cfg.latch_right = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;

driver.rampControl.ConfigureReferenceSwitch(ref_cfg);
```

### Encoder Configuration

Encoder pins can be configured for closed-loop control:

```cpp
tmc5160::EncoderConfig enc_cfg{};
enc_cfg.resolution = 4096;  // Pulses per revolution
enc_cfg.invert_direction = false;

driver.encoder.Configure(enc_cfg);
```

### CLK Pin

The CLK pin can be used for:
- External clock input (for precise timing)
- Internal clock (tie to GND)

For internal clock (default):
```cpp
// CLK pin should be tied to GND in hardware
// No software configuration needed
```

For external clock:
```cpp
// Connect external clock source to CLK pin
// Configure in GCONF register if needed
```

## Expected Behavior

### Startup

1. Pin configuration display
2. SPI interface initialization
3. Driver initialization
4. Pin state reading demonstration
5. Diagnostic pin status

### Runtime

- Periodic pin state reading
- Diagnostic information display
- Reference switch status (if configured)
- Encoder status (if configured)

## Code Structure

### Main Components

1. **Pin Configuration**: Sets up all GPIO pins
2. **SPI Interface**: Initializes SPI communication
3. **Driver Initialization**: Configures TMC5160 driver
4. **Pin Reading**: Demonstrates reading pin states
5. **Diagnostic Display**: Shows pin status information

### Key Functions

- `Esp32SPI::GpioRead()`: Read GPIO pin state
- `driver.rampControl.ConfigureReferenceSwitch()`: Configure endstops
- `driver.encoder.Configure()`: Configure encoder
- Pin state checking and display

## Customization

### Adding More Diagnostic Pins

```cpp
pin_config.tmc5160_pins.diag0_pin = 20;
pin_config.tmc5160_pins.diag1_pin = 21;
// Add more pins as needed
```

### Configuring Reference Switches

Modify reference switch configuration:

```cpp
ref_cfg.left_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_HIGH;
ref_cfg.right_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_HIGH;
ref_cfg.latch_left = tmc5160::ReferenceLatchMode::DISABLED;  // Non-latching
ref_cfg.latch_right = tmc5160::ReferenceLatchMode::DISABLED; // Non-latching
ref_cfg.latch_left = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;
ref_cfg.latch_right = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;
```

### Encoder Setup

Configure encoder for your specific encoder:

```cpp
enc_cfg.resolution = 16384;  // 14-bit encoder
enc_cfg.invert_direction = true;  // If encoder direction is reversed
```

## Troubleshooting

### Pins Not Reading

**Symptoms**: `GpioRead()` returns false or incorrect values

**Solutions**:
1. Verify pin numbers match hardware connections
2. Check if pins are configured in pin_config
3. Verify pins are not used for other purposes
4. Check GPIO pull-up/pull-down configuration

### Reference Switches Not Working

**Symptoms**: Switches don't trigger or trigger incorrectly

**Solutions**:
1. Verify switch wiring (NO vs NC)
2. Check polarity setting matches switch type
3. Verify pull-up/pull-down resistors
4. Test switch continuity with multimeter

### Encoder Not Detected

**Symptoms**: Encoder position not updating

**Solutions**:
1. Verify encoder wiring (A, B, N signals)
2. Check encoder power supply
3. Verify resolution setting matches encoder
4. Check encoder signal levels (3.3V vs 5V)

## Related Documentation

- [Pin Configuration Struct Example](pin_config_struct_example.md) - Using pin config struct
- [Core Test](core_comprehensive_test.md) - Basic GPIO testing
- [Diagnostics Test](diagnostics_comprehensive_test.md) - Diagnostic pin testing

