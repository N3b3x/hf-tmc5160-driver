# SPI Daisy Chain Comprehensive Test

## Overview

The `spi_daisy_chain_comprehensive_test.cpp` provides comprehensive testing for TMC5160 SPI daisy chain features including daisy chain setup, position management, and multi-motor coordination.

## Purpose

This test suite is ideal for:
- Validating SPI daisy chain configuration
- Testing multi-motor position management
- Verifying daisy chain communication
- Testing coordinated multi-motor motion
- Validating chain length configuration

## ⚠️ Important Warning

**MULTI-MOTOR HARDWARE REQUIRED**

This test suite requires **multiple TMC51x0 driver (TMC5130 & TMC5160)s** connected in a SPI daisy chain. **DO NOT run these tests on a single-motor setup.**

## Test Categories

### 1. Daisy Chain Setup Tests

Tests daisy chain configuration:
- Chain length configuration
- Driver initialization for each position
- Chain communication verification
- Position assignment

### 2. Position Management Tests

Tests position management in daisy chain:
- Position reading for each driver
- Position setting for each driver
- Position synchronization
- Chain position verification

### 3. Multi-Motor Coordination Tests

Tests coordinated multi-motor operation:
- Simultaneous position control
- Coordinated motion
- Multi-motor status reading
- Chain-wide operations

## Hardware Requirements

- ESP32 development board
- **2+ TMC5160 stepper motor drivers** (daisy-chained via SPI)
- Stepper motors connected to each TMC5160
- SPI daisy chain wiring (see below)
- Chip must be in **SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Daisy Chain Wiring

### SPI Daisy Chain Connection

All chips share common SPI signals:
- **CSN**: Shared chip select (all chips tied together)
- **SCK**: Shared clock (all chips tied together)
- **MOSI (SDI)**: Shared master output (all chips tied together)

MISO signals are daisy-chained:
- **MCU MISO** → **Chip 1 SDO** → **Chip 2 SDI**
- **Chip 1 SDI** ← **MCU MOSI**
- **Chip 2 SDO** → **Chip 3 SDI** (if 3+ chips)
- **Chip N SDO** → **MCU MISO** (last chip)

### Example Wiring (2 Chips)

```
MCU          Chip 1          Chip 2
MOSI ──────── SDI ──────────── SDI
MISO ──────── SDO ──────────── SDI
SCK  ──────── SCK ──────────── SCK
CS   ──────── CSN ──────────── CSN
```

### Example Wiring (3 Chips)

```
MCU          Chip 1          Chip 2          Chip 3
MOSI ──────── SDI ──────────── SDI ──────────── SDI
MISO ──────── SDO ──────────── SDI ──────────── SDI
SCK  ──────── SCK ──────────── SCK ──────────── SCK
CS   ──────── CSN ──────────── CSN ──────────── CSN
```

## Pin Configuration

The test uses `GetDefaultPinConfig()` from `esp32_tmc51x0_test_config.hpp` (shared by all chips in the chain):

- **SPI**: MOSI=6, MISO=2, SCLK=5, CS=18 (shared)
- **Control**: EN=11 (can be shared or separate per chip)
- **Clock**: CLK=10 (tied to GND for internal clock)
- **Diagnostics**: DIAG0=23, DIAG1=15
- **SPI Clock**: From config (500 kHz typical)

**Note**: The source file comment lists alternative pins (MOSI=23, MISO=19, etc.) for a different hardware setup; the actual build uses the default test config pins.

## Test Configuration

Tests can be enabled/disabled:

```cpp
static constexpr bool ENABLE_DAISY_CHAIN_SETUP_TESTS = true;
static constexpr bool ENABLE_POSITION_MANAGEMENT_TESTS = true;
static constexpr bool ENABLE_MULTI_MOTOR_COORDINATION_TESTS = true;
```

### Chain Length Configuration

Set the number of devices in the chain:

```cpp
static constexpr uint8_t TEST_CHAIN_LENGTH = 2; // Number of devices
```

## Detailed Test Descriptions

### Daisy Chain Setup Tests

#### Test: Chain Length Configuration
- **Purpose**: Verify chain length can be set
- **Steps**:
  1. Set chain length
  2. Create driver instances for each position
  3. Verify chain length matches
- **Expected**: Chain length configured correctly

#### Test: Driver Initialization
- **Purpose**: Verify each driver in chain can be initialized
- **Steps**:
  1. Initialize driver at position 0
  2. Initialize driver at position 1
  3. Verify both initialized successfully
- **Expected**: All drivers initialize correctly

#### Test: Chain Communication
- **Purpose**: Verify communication works through chain
- **Steps**:
  1. Write to driver at position 0
  2. Write to driver at position 1
  3. Read back from both
  4. Verify values match
- **Expected**: Communication works correctly through chain

### Position Management Tests

#### Test: Position Reading
- **Purpose**: Verify position can be read for each driver
- **Steps**:
  1. Read position from driver 0
  2. Read position from driver 1
  3. Verify positions independent
- **Expected**: Positions read correctly for each driver

#### Test: Position Setting
- **Purpose**: Verify position can be set for each driver
- **Steps**:
  1. Set target position for driver 0
  2. Set target position for driver 1
  3. Verify positions set independently
- **Expected**: Positions set correctly for each driver

### Multi-Motor Coordination Tests

#### Test: Simultaneous Position Control
- **Purpose**: Verify multiple motors can be controlled simultaneously
- **Steps**:
  1. Set target positions for all motors
  2. Start motion on all motors
  3. Verify all motors move
  4. Check positions independently
- **Expected**: All motors move to targets simultaneously

#### Test: Coordinated Motion
- **Purpose**: Verify motors can move in coordination
- **Steps**:
  1. Set coordinated target positions
  2. Start motion
  3. Verify motors move together
  4. Check synchronization
- **Expected**: Motors move in coordination

## Code Structure

### Creating Daisy Chain Drivers

The test uses a `TestDriverHandle` with shared SPI and a vector of driver instances:

```cpp
struct TestDriverHandle {
  std::unique_ptr<Esp32SPI> spi;
  std::vector<std::unique_ptr<tmc51x0::TMC51x0<Esp32SPI>>> drivers;
};

// Create via create_daisy_chain_drivers()
auto handle = create_daisy_chain_drivers();
// Uses GetDefaultPinConfig() and SELECTED_TEST_RIG (TEST_RIG_CORE_DRIVER)

// Access drivers by index
handle->drivers[0]->rampControl.SetTargetPosition(1000);
handle->drivers[1]->rampControl.SetTargetPosition(2000);
```

Chain length is set via `TEST_CHAIN_LENGTH` (default: 2).

## Expected Behavior

### Test Execution

1. **Initialization**: SPI interface and chain setup
2. **Setup Tests**: Chain configuration verification
3. **Position Tests**: Position management testing
4. **Coordination Tests**: Multi-motor coordination
5. **Summary**: Test results displayed

### Typical Output

```
I (1234) SPI_DaisyChain_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) SPI_DaisyChain_Test: ║          SPI Daisy Chain Comprehensive Test Suite                            ║
I (1236) SPI_DaisyChain_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) SPI_DaisyChain_Test: ⚠️ MULTI-MOTOR HARDWARE REQUIRED
I (1238) SPI_DaisyChain_Test: [PASS] Chain Length Configuration: Chain length set to 2
I (1239) SPI_DaisyChain_Test: [PASS] Driver Initialization: All drivers initialized
...
```

## Troubleshooting

### Chain Communication Fails

**Symptoms**: Can't communicate with drivers in chain

**Solutions**:
1. Verify daisy chain wiring (MISO chain)
2. Check CS pin is shared correctly
3. Verify SCK and MOSI are shared
4. Check chain length matches actual hardware
5. Verify SPI clock speed is appropriate

### Wrong Driver Responding

**Symptoms**: Writing to position 0 affects position 1

**Solutions**:
1. Verify chain length is set correctly
2. Check driver position assignments
3. Verify MISO daisy chain wiring
4. Check for wiring errors

### Position Not Independent

**Symptoms**: Positions affect each other

**Solutions**:
1. Verify each driver has independent position
2. Check driver initialization
3. Verify chain position assignments
4. Check for register write issues

## Related Documentation

- [UART Multi-Node Test](uart_multi_node_comprehensive_test.md) - UART multi-motor setup
- [Special Features: Multi-Chip](../../../docs/special_features_multi_chip.md) - Detailed daisy chain guide

