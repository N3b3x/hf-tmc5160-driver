# ESP32 Examples and Tests Documentation

This directory contains comprehensive documentation for all ESP32 examples and test suites for the TMC5160 stepper motor driver library.

## Overview

The ESP32 examples demonstrate practical applications of the TMC5160 driver, while the comprehensive test suites provide thorough validation of all driver features. All examples and tests support multiple motor configurations selected at compile-time.

## Motor Configuration

All examples and tests support three motor configurations:

- **MOTOR_17HS4401S_GEARBOX** (default): 17HS4401S with 5.18:1 planetary gearbox
- **MOTOR_17HS4401S_DIRECT**: 17HS4401S direct drive (no gearbox)
- **MOTOR_APPLIED_MOTION_5034**: Applied Motion 5034-369 NEMA 34 (high torque)

See [Motor Configuration Guide](motor_configuration.md) for detailed specifications and selection instructions.

## Examples

Practical applications demonstrating real-world usage:

### Motion Control Examples

- **[Sinusoidal Motion](sinusoidal.md)** - Simple back-and-forth motion using positioning mode
- **[Bounds Finding & Sinusoidal Motion](bounds_finding_sinuous_motion.md)** - Fatigue testing platform with sensorless bounds finding and UART command interface

### Configuration Examples

- **[GPIO Pin Configuration](gpio_pin_config_example.md)** - Demonstrates GPIO pin setup and usage
- **[Pin Configuration Struct](pin_config_struct_example.md)** - Shows how to use the pin configuration struct

## Comprehensive Test Suites

Thorough validation of driver features:

### Single Motor Tests

- **[Internal Ramp Comprehensive Test](internal_ramp_comprehensive_test.md)** - **Main comprehensive test suite** - Combines core, motor control, ramp control, diagnostics, and protection tests into a single wholesome test suite for SPI internal ramp mode (with encoder and reference switches)
- **[Encoder Comprehensive Test](encoder_comprehensive_test.md)** - Encoder configuration, position reading, deviation detection (separate test suite)

### Multi-Motor Tests

- **[SPI Daisy Chain Comprehensive Test](spi_daisy_chain_comprehensive_test.md)** - Multi-motor SPI daisy chain configuration and coordination
- **[UART Multi-Node Comprehensive Test](uart_multi_node_comprehensive_test.md)** - Multi-motor UART network with node addressing

## Quick Start

1. **Select Your Motor**: Edit the `SELECTED_MOTOR` constant at the top of your example/test file
2. **Configure Pins**: Modify pin assignments in `esp32_tmc5160_bus_config.hpp` if needed
3. **Build**: Use ESP-IDF build system (`idf.py build`)
4. **Flash**: Flash to your ESP32 board (`idf.py flash`)
5. **Monitor**: View output via serial monitor (`idf.py monitor`)

## Hardware Requirements

- ESP32 development board (ESP32, ESP32-C3, ESP32-C6, etc.)
- TMC5160 stepper motor driver board
- Stepper motor (see motor configuration guide)
- Power supply: 12-36V DC (ensure adequate current capacity)
- SPI connection (for SPI examples/tests)
- UART connection (for UART examples/tests)

## Common Configuration

All examples and tests use shared configuration files:

- `esp32_tmc5160_bus_config.hpp` - Pin assignments, motor configurations, and test defaults
  - Default SPI pins: MOSI=6, MISO=2, SCLK=5, CS=18
  - Default control pins: EN=11, CLK=10, DIAG0=23, DIAG1=15
  - Default SPI clock: 500 kHz (some examples override this)
  - Motor configurations: 17HS4401S (geared/direct), Applied Motion 5034-369
- `esp32_tmc5160_bus.hpp` - ESP32-specific communication interface implementations

## Documentation Structure

Each example/test has its own documentation file covering:

- Purpose and use cases
- Hardware requirements
- Pin configuration
- Motor selection
- Key features demonstrated
- Usage instructions
- Expected behavior
- Troubleshooting tips

## Support

For issues or questions:

1. Check the specific example/test documentation
2. Review the main library documentation
3. Check hardware connections and power supply
4. Verify motor configuration matches your hardware

