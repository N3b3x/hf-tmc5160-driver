# Ramp Control Comprehensive Test

## Overview

The `ramp_control_comprehensive_test.cpp` provides comprehensive testing for TMC5160 ramp control features including all ramp modes, position control, speed control, ramp speeds, power-down delay, zero wait time, first acceleration phase, reference switch configuration, and unit conversions.

## Purpose

This test suite is ideal for:
- Testing all ramp modes (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD)
- Validating position control accuracy
- Testing speed control and acceleration
- Verifying ramp speed settings
- Testing reference switch/endstop functionality
- Validating unit conversions (mm, RPM, degrees)

## Test Categories

### 1. Ramp Mode Tests

Tests all ramp generator modes:
- POSITIONING mode
- VELOCITY_POS mode
- VELOCITY_NEG mode
- HOLD mode
- Mode switching

### 2. Position Control Tests

Tests position control features:
- Target position setting
- Current position reading
- Relative position moves
- Position reached detection
- Position accuracy

### 3. Speed Control Tests

Tests speed control:
- Maximum speed (VMAX) setting
- Acceleration (AMAX) setting
- Deceleration (DMAX) setting
- Speed reading (VACTUAL)
- Speed limits

### 4. Ramp Speed Tests

Tests ramp speed parameters:
- Start velocity (VSTART)
- Stop velocity (VSTOP)
- Transition velocity (V1)
- Speed relationships

### 5. Power-Down Delay Tests

Tests power-down behavior:
- Power-down delay configuration
- Power-down timing
- Standby behavior

### 6. Zero Wait Time Tests

Tests zero wait time:
- Zero wait configuration
- Wait time behavior
- Direction change handling

### 7. First Acceleration Tests

Tests first acceleration phase:
- First acceleration configuration
- Acceleration phase behavior

### 8. Reference Switch Tests

Tests reference switch/endstop functionality:
- Left reference switch
- Right reference switch
- Switch polarity
- Switch latching
- Soft stop configuration

### 9. Unit Conversion Tests

Tests unit conversion functions:
- Steps to degrees
- Degrees to steps
- Steps to millimeters
- Millimeters to steps
- RPM conversion

## Hardware Requirements

- ESP32 development board
- TMC5160 stepper motor driver board
- Single stepper motor connected to TMC5160
- SPI connection between ESP32 and TMC5160
- Optional: Reference switches (endstops)
- Chip must be in **SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Pin Configuration

Default pin configuration (from `esp32_tmc5160_bus_config.hpp`):

- **SPI**: MOSI=6, MISO=2, SCLK=5, CS=18
- **Control**: EN=11
- **Clock**: CLK=10 (tied to GND for internal clock)
- **Diagnostics**: DIAG0=23, DIAG1=15
- **SPI Clock**: 500 kHz (from config)
- **Reference Switches**: Configured via code (optional)

## Motor Selection

Uses `MotorConfig_17HS4401S` by default. See [Motor Configuration Guide](motor_configuration.md) for options.

## Test Configuration

Tests can be enabled/disabled:

```cpp
static constexpr bool ENABLE_RAMP_MODE_TESTS = true;
static constexpr bool ENABLE_POSITION_CONTROL_TESTS = true;
static constexpr bool ENABLE_SPEED_CONTROL_TESTS = true;
static constexpr bool ENABLE_RAMP_SPEED_TESTS = true;
static constexpr bool ENABLE_POWERDOWN_TESTS = true;
static constexpr bool ENABLE_ZEROWAIT_TESTS = true;
static constexpr bool ENABLE_FIRST_ACCEL_TESTS = true;
static constexpr bool ENABLE_REFERENCE_SWITCH_TESTS = true;
static constexpr bool ENABLE_UNIT_CONVERSION_TESTS = true;
```

## Detailed Test Descriptions

### Ramp Mode Tests

#### Test: POSITIONING Mode
- **Purpose**: Verify positioning mode operation
- **Steps**:
  1. Set ramp mode to POSITIONING
  2. Set target position
  3. Verify motor moves to target
  4. Check position reached flag
- **Expected**: Motor moves to target position accurately

#### Test: VELOCITY_POS Mode
- **Purpose**: Verify positive velocity mode
- **Steps**:
  1. Set ramp mode to VELOCITY_POS
  2. Set VMAX
  3. Verify motor moves in positive direction
  4. Check velocity matches VMAX
- **Expected**: Motor moves forward at set velocity

#### Test: VELOCITY_NEG Mode
- **Purpose**: Verify negative velocity mode
- **Steps**:
  1. Set ramp mode to VELOCITY_NEG
  2. Set VMAX
  3. Verify motor moves in negative direction
  4. Check velocity matches VMAX
- **Expected**: Motor moves backward at set velocity

#### Test: HOLD Mode
- **Purpose**: Verify hold mode stops motion
- **Steps**:
  1. Start motion in velocity mode
  2. Switch to HOLD mode
  3. Verify motor stops
  4. Check velocity goes to zero
- **Expected**: Motor stops and holds position

### Position Control Tests

#### Test: Target Position Setting
- **Purpose**: Verify target position can be set
- **Steps**:
  1. Set target position
  2. Read back XTARGET register
  3. Verify value matches
- **Expected**: Target position set correctly

#### Test: Current Position Reading
- **Purpose**: Verify current position can be read
- **Steps**:
  1. Move motor
  2. Read current position
  3. Verify position updates
- **Expected**: Current position reflects actual position

#### Test: Relative Position Moves
- **Purpose**: Verify relative moves work
- **Steps**:
  1. Get current position
  2. Perform relative move
  3. Verify new position
- **Expected**: Relative move adds to current position

#### Test: Position Reached Detection
- **Purpose**: Verify position reached flag
- **Steps**:
  1. Set target position
  2. Wait for motion
  3. Check position_reached flag
- **Expected**: Flag sets when target reached

### Speed Control Tests

#### Test: Maximum Speed Setting
- **Purpose**: Verify VMAX can be set
- **Steps**:
  1. Set VMAX value
  2. Read back VMAX register
  3. Verify value matches
- **Expected**: VMAX set correctly

#### Test: Acceleration Setting
- **Purpose**: Verify AMAX can be set
- **Steps**:
  1. Set AMAX value
  2. Read back AMAX register
  3. Verify value matches
- **Expected**: AMAX set correctly

#### Test: Speed Reading
- **Purpose**: Verify VACTUAL can be read
- **Steps**:
  1. Start motion
  2. Read VACTUAL
  3. Verify velocity > 0
- **Expected**: VACTUAL reflects actual velocity

### Reference Switch Tests

#### Test: Left Reference Switch
- **Purpose**: Verify left endstop functionality
- **Steps**:
  1. Configure left reference switch
  2. Trigger switch (or simulate)
  3. Verify stop behavior
- **Expected**: Motor stops at left switch

#### Test: Right Reference Switch
- **Purpose**: Verify right endstop functionality
- **Steps**:
  1. Configure right reference switch
  2. Trigger switch (or simulate)
  3. Verify stop behavior
- **Expected**: Motor stops at right switch

#### Test: Switch Polarity
- **Purpose**: Verify switch polarity configuration
- **Steps**:
  1. Configure switch polarity
  2. Test active HIGH and LOW
  3. Verify behavior matches polarity
- **Expected**: Switch responds according to polarity

### Unit Conversion Tests

#### Test: Steps to Degrees
- **Purpose**: Verify step-to-degree conversion
- **Steps**:
  1. Convert known step value to degrees
  2. Verify calculation
  3. Check accuracy
- **Expected**: Conversion accurate

#### Test: Degrees to Steps
- **Purpose**: Verify degree-to-step conversion
- **Steps**:
  1. Convert known degree value to steps
  2. Verify calculation
  3. Check accuracy
- **Expected**: Conversion accurate

#### Test: RPM Conversion
- **Purpose**: Verify RPM conversion
- **Steps**:
  1. Convert velocity to RPM
  2. Verify calculation
  3. Check accuracy
- **Expected**: RPM conversion accurate

## Expected Behavior

### Test Execution

1. **Initialization**: Driver and motor setup
2. **Mode Testing**: Each ramp mode tested
3. **Position Testing**: Position control validated
4. **Speed Testing**: Speed control validated
5. **Switch Testing**: Reference switches tested (if configured)
6. **Summary**: Test results displayed

### Typical Output

```
I (1234) RampControl_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) RampControl_Test: ║              Ramp Control Comprehensive Test Suite                            ║
I (1236) RampControl_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) RampControl_Test: [PASS] POSITIONING Mode: Position control working
I (1238) RampControl_Test: [PASS] VELOCITY_POS Mode: Forward velocity working
I (1239) RampControl_Test: [PASS] Target Position: Position set correctly
...
```

## Troubleshooting

### Position Not Accurate

**Symptoms**: Motor doesn't reach target position

**Solutions**:
1. Check microstep resolution
2. Verify gearbox ratio (if applicable)
3. Check for mechanical binding
4. Verify acceleration/deceleration settings
5. Check position reached timeout

### Speed Not Matching

**Symptoms**: Actual speed doesn't match VMAX

**Solutions**:
1. Verify VMAX setting
2. Check acceleration limits
3. Verify motor can reach speed
4. Check for mechanical limitations
5. Verify power supply voltage

### Reference Switches Not Working

**Symptoms**: Switches don't stop motor

**Solutions**:
1. Verify switch wiring
2. Check switch polarity configuration
3. Verify switches enabled in SW_MODE
4. Check switch signal levels
5. Test switch continuity

## Related Documentation

- [Sinusoidal Example](sinusoidal.md) - Positioning mode example
- [Bounds Finding Example](bounds_finding_sinuous_motion.md) - Advanced positioning
- [Core Test](core_comprehensive_test.md) - Basic ramp testing

