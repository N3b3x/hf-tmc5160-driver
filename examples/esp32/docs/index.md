# ESP32 Examples and Tests Documentation

This directory contains comprehensive documentation for all ESP32 examples and test suites for the TMC5160 stepper motor driver library.

## Overview

The ESP32 examples demonstrate practical applications of the TMC51x0 driver (TMC5130 & TMC5160), while the comprehensive test suites provide thorough validation of all driver features. All examples and tests support multiple motor configurations selected at compile-time.

## Test Rig Configuration

All examples and tests use a unified test rig selection system that automatically configures motor, board, and platform settings:

- **TEST_RIG_CORE_DRIVER** (default for most examples): 17HS4401S motor (geared or direct drive), TMC51x0 EVAL board, reference switches, encoder
- **TEST_RIG_FATIGUE** (default for fatigue testing examples): Applied Motion 5034-369 NEMA 34 motor, TMC51x0 EVAL board, reference switches, encoder

See [Motor Configuration Guide](motor_configuration.md) for detailed specifications and selection instructions.

## Examples

Practical applications demonstrating real-world usage:

### Motion Control Examples

- **[Internal Ramp Sinusoidal](internal_ramp_sinusoidal.md)** - Simple back-and-forth motion using positioning mode (configurable via `SELECTED_TEST_RIG`)
- **[Fatigue Testing](fatigue_test.md)** - Point-to-point back-and-forth motion between bounds, ESP-NOW controlled, with StallGuard2 or encoder-based bounds detection

## Comprehensive Test Suites

Thorough validation of driver features:

### Single Motor Tests

- **[Internal Ramp Comprehensive Test](internal_ramp_comprehensive_test.md)** - **Main comprehensive test suite** - Combines core, motor control, ramp control, diagnostics, and protection tests into a single wholesome test suite for SPI internal ramp mode (with encoder and reference switches)
- Encoder configuration, position reading, and deviation detection are included in the internal ramp comprehensive test; there is no separate encoder-only test suite.

### Multi-Motor Tests

- **[SPI Daisy Chain Comprehensive Test](spi_daisy_chain_comprehensive_test.md)** - Multi-motor SPI daisy chain configuration and coordination
- **[UART Multi-Node Comprehensive Test](uart_multi_node_comprehensive_test.md)** - Multi-motor UART network with node addressing

## Quick Start

1. **Select Your Test Rig**: Edit the `SELECTED_TEST_RIG` constant at the top of your example/test file (`TEST_RIG_CORE_DRIVER` or `TEST_RIG_FATIGUE`)
2. **Configure Pins**: Modify pin assignments in `esp32_tmc51x0_test_config.hpp` if needed
3. **Build**: Use the build script (`./scripts/build_app.sh <app_name> Debug`) or ESP-IDF build system (`idf.py build`)
4. **Flash**: Flash using the flash script (`./scripts/flash_app.sh --port /dev/ttyACM0 flash <app_name> Debug`) or `idf.py flash`
5. **Monitor**: View output via serial monitor (`idf.py monitor` or `./scripts/flash_app.sh flash_monitor <app_name> Debug`)

For detailed setup, build, and flash instructions, see the [ESP32 README](../README.md) and [scripts/README.md](../scripts/README.md).

## Hardware Requirements

- ESP32 development board (ESP32, ESP32-C3, ESP32-C6, etc.)
- TMC5160 stepper motor driver board
- Stepper motor (see motor configuration guide)
- Power supply: 12-36V DC (ensure adequate current capacity)
- SPI connection (for SPI examples/tests)
- UART connection (for UART examples/tests)

## Common Configuration

All examples and tests use shared configuration files:

- `esp32_tmc51x0_test_config.hpp` - Pin assignments, test rig configurations, and test defaults
  - Default SPI pins: MOSI=6, MISO=2, SCLK=5, CS=18
  - Default control pins: EN=11, CLK=10, DIAG0=23, DIAG1=15
  - Default SPI clock: 500 kHz (some examples override this)
  - Test rig configurations: TEST_RIG_CORE_DRIVER (17HS4401S), TEST_RIG_FATIGUE (Applied Motion 5034-369)
- `esp32_tmc51x0_bus.hpp` - ESP32-specific communication interface implementations

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

