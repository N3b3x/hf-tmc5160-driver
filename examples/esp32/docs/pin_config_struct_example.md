# Pin Configuration Struct Example

## Overview

The `pin_config_struct_example.cpp` demonstrates how to use the `TMC5160PinConfig` and `Esp32SpiPinConfig` structs to configure all GPIO pins in a single place, with automatic handling of compound pins (pins that share the same physical GPIO).

## Purpose

This example is ideal for:
- Understanding pin configuration structs
- Learning about compound pins
- Simplifying pin configuration
- Avoiding pin assignment conflicts
- Organizing pin assignments

## Key Features

- **Pin Configuration Struct**: Single struct for all pin assignments
- **Compound Pin Handling**: Automatic mapping of shared GPIO pins
- **Type Safety**: Compile-time pin validation
- **Clean Configuration**: Organized pin assignments

## Hardware Requirements

- ESP32-C6 development board (or other ESP32 variant)
- TMC5160 stepper motor driver board
- Stepper motor connected to TMC5160
- SPI connection between ESP32 and TMC5160

## Pin Configuration

### ESP32-C6 Default Configuration

- **SPI**:
  - SCK: GPIO5
  - MOSI: GPIO6
  - MISO: GPIO12
  - CS_TMC: GPIO18

- **Control**:
  - DRV_EN: GPIO11
  - CLK16: GPIO10

- **Diagnostic**:
  - DIAG0: GPIO20 (example)
  - DIAG1: GPIO21 (example)

## Compound Pins

The TMC5160 has several pins that share the same physical GPIO:

### STEP / REFL_STEP

The STEP pin and REFL_STEP (left reference switch) share the same GPIO. When you specify one, the other is automatically mapped:

```cpp
pin_config.step_pin = GPIO_NUM_4;
// REFL_STEP is automatically set to GPIO_NUM_4 as well
```

Or specify REFL_STEP instead:

```cpp
pin_config.ref_left_pin = GPIO_NUM_4;
// STEP is automatically set to GPIO_NUM_4 as well
```

### DIR / REFR_DIR

The DIR pin and REFR_DIR (right reference switch) share the same GPIO:

```cpp
pin_config.dir_pin = GPIO_NUM_5;
// REFR_DIR is automatically set to GPIO_NUM_5 as well
```

### ENC_A / DC_STEP

The encoder A signal and DC_STEP share the same GPIO:

```cpp
pin_config.enc_a_pin = GPIO_NUM_6;
// DC_STEP is automatically set to GPIO_NUM_6 as well
```

### ENC_B / DC_DIR

The encoder B signal and DC_DIR share the same GPIO:

```cpp
pin_config.enc_b_pin = GPIO_NUM_7;
// DC_DIR is automatically set to GPIO_NUM_7 as well
```

## How It Works

### Using Esp32SpiPinConfig

The `Esp32SpiPinConfig` struct contains both SPI pins and TMC5160 control pins:

```cpp
tmc5160::Esp32SpiPinConfig pin_config{};

// SPI pins
pin_config.spi_mosi = 6;
pin_config.spi_miso = 12;
pin_config.spi_sclk = 5;
pin_config.spi_cs = 18;

// TMC5160 control pins
pin_config.tmc5160_pins.en_pin = 11;
pin_config.tmc5160_pins.clk_pin = 10;
pin_config.tmc5160_pins.diag0_pin = 20;
pin_config.tmc5160_pins.diag1_pin = 21;

// Compound pins - only specify one
pin_config.tmc5160_pins.step_pin = 4;  // REFL_STEP automatically set
pin_config.tmc5160_pins.dir_pin = 5;   // REFR_DIR automatically set
```

### Using TMC5160PinConfig Directly

You can also use `TMC5160PinConfig` directly:

```cpp
tmc5160::TMC5160PinConfig pin_config{};
pin_config.en_pin = GPIO_NUM_11;
pin_config.clk_pin = GPIO_NUM_10;
pin_config.step_pin = GPIO_NUM_4;  // REFL_STEP automatically set
```

### Automatic Compound Pin Mapping

The library automatically handles compound pins:

1. **Priority**: If both pins in a compound pair are specified, one takes priority
2. **Automatic Mapping**: Specifying one pin automatically sets the other
3. **Validation**: The library validates pin assignments to prevent conflicts

## Expected Behavior

### Startup

1. Pin configuration struct creation
2. Compound pin mapping demonstration
3. SPI interface initialization with pin config
4. Driver initialization
5. Pin assignment verification

### Runtime

- Pin state reading
- Compound pin relationship display
- Configuration verification

## Code Structure

### Main Components

1. **Pin Configuration Struct**: Single struct for all pins
2. **Compound Pin Demonstration**: Shows automatic mapping
3. **SPI Interface**: Uses pin config struct
4. **Pin Validation**: Verifies pin assignments

### Key Concepts

- **Compound Pins**: Pins that share GPIO (handled automatically)
- **Pin Priority**: Which pin takes precedence when both specified
- **Type Safety**: Compile-time pin number validation

## Customization

### Modifying Pin Assignments

Simply change values in the pin config struct:

```cpp
pin_config.tmc5160_pins.en_pin = GPIO_NUM_9;      // Change EN pin
pin_config.tmc5160_pins.diag0_pin = GPIO_NUM_22;  // Change DIAG0 pin
```

### Using Compound Pins

Choose which pin name to use based on your application:

```cpp
// If using STEP pin:
pin_config.tmc5160_pins.step_pin = GPIO_NUM_4;

// If using REFL_STEP pin:
pin_config.tmc5160_pins.ref_left_pin = GPIO_NUM_4;

// Both result in the same GPIO assignment
```

### Adding Optional Pins

Optional pins can be set to -1 if not used:

```cpp
pin_config.tmc5160_pins.enc_n_pin = -1;  // Encoder index not used
pin_config.tmc5160_pins.ref_right_pin = -1;  // Right endstop not used
```

## Troubleshooting

### Pin Conflicts

**Symptoms**: Pin assignment errors or unexpected behavior

**Solutions**:
1. Check for GPIO conflicts with other peripherals
2. Verify compound pins are not assigned separately
3. Ensure pin numbers are valid GPIO numbers
4. Check ESP32 pin restrictions (some pins have special functions)

### Compound Pins Not Working

**Symptoms**: One pin works but the other doesn't

**Solutions**:
1. Verify compound pin relationship (check datasheet)
2. Ensure only one pin in pair is specified
3. Check GPIO configuration matches intended use
4. Verify pin is not used elsewhere

### Pin Configuration Not Applied

**Symptoms**: Pins don't behave as configured

**Solutions**:
1. Verify pin config struct is passed to constructor
2. Check pin numbers match hardware connections
3. Verify SPI interface initialization succeeded
4. Check for pin mode conflicts (input vs output)

## Best Practices

1. **Use Pin Config Struct**: Prefer struct over individual parameters
2. **Specify One Compound Pin**: Don't specify both pins in a compound pair
3. **Use Descriptive Names**: Choose pin names that match your application
4. **Validate Pin Numbers**: Ensure GPIO numbers are valid for your ESP32 variant
5. **Document Pin Assignments**: Keep pin assignments documented

## Related Documentation

- [GPIO Pin Configuration Example](gpio_pin_config_example.md) - Individual pin configuration
- [esp32_tmc5160_bus_config.hpp](../../main/esp32_tmc5160_bus_config.hpp) - Standard pin configuration
- [Core Test](core_comprehensive_test.md) - Pin configuration testing

