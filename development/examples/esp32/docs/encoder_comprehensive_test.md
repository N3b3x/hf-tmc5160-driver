# Encoder Comprehensive Test

## Overview

The `encoder_comprehensive_test.cpp` provides comprehensive testing for TMC5160 encoder features including encoder configuration, resolution setting, position reading, deviation detection, and latched position functionality.

## Purpose

This test suite is ideal for:
- Validating encoder configuration
- Testing encoder resolution settings
- Verifying encoder position reading
- Testing deviation detection
- Validating latched position functionality
- Testing closed-loop control setup

## Test Categories

### 1. Encoder Configuration Tests

Tests encoder setup:
- Encoder enable/disable
- Encoder selection (decimal/binary)
- Clear on constant (continuous clear)
- Clear once (single clear)
- Configuration verification

### 2. Encoder Resolution Tests

Tests encoder resolution setting:
- Resolution configuration
- Resolution reading
- Resolution verification
- Resolution limits

### 3. Encoder Position Tests

Tests encoder position reading:
- Encoder position reading
- Position updates
- Position accuracy
- Position wrapping

### 4. Deviation Detection Tests

Tests deviation detection:
- Deviation threshold configuration
- Deviation reading
- Deviation detection
- Deviation limits

### 5. Latched Position Tests

Tests latched position:
- Position latching
- Latched position reading
- Latch trigger
- Latch clearing

## Hardware Requirements

- ESP32 development board
- TMC5160 stepper motor driver board
- Single stepper motor with encoder connected to TMC5160
- Encoder signals (A, B, N) connected to TMC5160 encoder inputs
- SPI connection between ESP32 and TMC5160
- Chip must be in **SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Pin Configuration

Default pin configuration:

- **SPI**: MOSI=23, MISO=19, SCLK=18, CS=5
- **Control**: EN=2, DIR=4, STEP=15
- **Encoder**: ENC_A, ENC_B, ENC_N (configured via code)

## Motor Selection

Uses `MotorConfig_17HS4401S` by default. See [Motor Configuration Guide](motor_configuration.md) for options.

## Test Configuration

Tests can be enabled/disabled:

```cpp
static constexpr bool ENABLE_ENCODER_CONFIG_TESTS = true;
static constexpr bool ENABLE_ENCODER_RESOLUTION_TESTS = true;
static constexpr bool ENABLE_ENCODER_POSITION_TESTS = true;
static constexpr bool ENABLE_DEVIATION_DETECTION_TESTS = true;
static constexpr bool ENABLE_LATCHED_POSITION_TESTS = true;
```

## Encoder Setup

### Encoder Wiring

The TMC5160 supports incremental encoders with A, B, and optional N (index) signals:

- **ENC_A**: Encoder A signal (quadrature)
- **ENC_B**: Encoder B signal (quadrature)
- **ENC_N**: Encoder index/N signal (optional)

### Encoder Types

The TMC5160 supports:
- **Incremental Encoders**: Standard quadrature encoders
- **Resolution**: Configurable (typically 1024-16384 pulses/rev)

### Encoder Configuration

Encoder configuration includes:
- **Resolution**: Pulses per revolution
- **Direction**: Normal or inverted
- **Clear Mode**: Continuous or single clear
- **Selection**: Decimal or binary encoding

## Detailed Test Descriptions

### Encoder Configuration Tests

#### Test: Encoder Enable
- **Purpose**: Verify encoder can be enabled
- **Steps**:
  1. Configure encoder
  2. Enable encoder
  3. Verify encoder active
- **Expected**: Encoder enables successfully

#### Test: Encoder Selection
- **Purpose**: Verify encoder selection (decimal/binary)
- **Steps**:
  1. Set encoder selection mode
  2. Verify configuration
  3. Test both modes
- **Expected**: Encoder selection works correctly

### Encoder Resolution Tests

#### Test: Resolution Configuration
- **Purpose**: Verify encoder resolution can be set
- **Steps**:
  1. Set encoder resolution
  2. Read back resolution
  3. Verify resolution matches
- **Expected**: Resolution set correctly

#### Test: Resolution Limits
- **Purpose**: Verify resolution limits
- **Steps**:
  1. Set minimum resolution
  2. Set maximum resolution
  3. Verify limits enforced
- **Expected**: Resolution limits work correctly

### Encoder Position Tests

#### Test: Position Reading
- **Purpose**: Verify encoder position can be read
- **Steps**:
  1. Enable encoder
  2. Move motor
  3. Read encoder position
  4. Verify position updates
- **Expected**: Encoder position reflects actual position

#### Test: Position Accuracy
- **Purpose**: Verify encoder position accuracy
- **Steps**:
  1. Move motor known distance
  2. Read encoder position
  3. Compare with motor position
  4. Verify accuracy
- **Expected**: Encoder position matches motor position

### Deviation Detection Tests

#### Test: Deviation Configuration
- **Purpose**: Verify deviation threshold can be set
- **Steps**:
  1. Set deviation threshold
  2. Verify configuration
  3. Test deviation detection
- **Expected**: Deviation threshold set correctly

#### Test: Deviation Detection
- **Purpose**: Verify deviation is detected
- **Steps**:
  1. Configure deviation threshold
  2. Create position error (if possible)
  3. Verify deviation detected
  4. Check deviation flag
- **Expected**: Deviation detected when threshold exceeded

### Latched Position Tests

#### Test: Position Latching
- **Purpose**: Verify position can be latched
- **Steps**:
  1. Enable position latching
  2. Trigger latch
  3. Read latched position
  4. Verify position latched
- **Expected**: Position latches correctly

## Expected Behavior

### Test Execution

1. **Initialization**: Driver and encoder setup
2. **Configuration Tests**: Encoder configuration
3. **Resolution Tests**: Resolution setting
4. **Position Tests**: Position reading
5. **Deviation Tests**: Deviation detection
6. **Latch Tests**: Position latching
7. **Summary**: Test results displayed

### Typical Output

```
I (1234) Encoder_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) Encoder_Test: ║              Encoder Comprehensive Test Suite                                   ║
I (1236) Encoder_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) Encoder_Test: [PASS] Encoder Configuration: Encoder enabled
I (1238) Encoder_Test: [PASS] Resolution Configuration: Resolution set to 4096
I (1239) Encoder_Test: [PASS] Position Reading: Position read successfully
...
```

## Troubleshooting

### Encoder Not Detected

**Symptoms**: Encoder position doesn't update

**Solutions**:
1. Verify encoder wiring (A, B signals)
2. Check encoder power supply
3. Verify encoder signal levels (3.3V vs 5V)
4. Check encoder resolution setting
5. Verify encoder is enabled

### Position Not Accurate

**Symptoms**: Encoder position doesn't match motor position

**Solutions**:
1. Verify encoder resolution matches encoder specs
2. Check encoder direction (may need inversion)
3. Verify encoder mounting (aligned correctly)
4. Check for encoder signal noise
5. Verify encoder resolution calculation

### Deviation Not Detected

**Symptoms**: Deviation detection doesn't trigger

**Solutions**:
1. Verify deviation threshold is set appropriately
2. Check if position error exceeds threshold
3. Verify deviation detection is enabled
4. Check encoder position reading is working
5. Verify motor position reading is accurate

## Related Documentation

- [GPIO Pin Configuration Example](gpio_pin_config_example.md) - Encoder pin setup
- [Core Test](core_comprehensive_test.md) - Basic encoder testing

