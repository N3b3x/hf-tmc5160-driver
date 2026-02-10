---
layout: default
title: "Internal Ramp Comprehensive Test"
description: "Comprehensive test suite covering all TMC51x0 driver features in SPI internal ramp mode"
nav_order: 4
parent: "ESP32 Examples"
permalink: /docs/esp32-examples/internal-ramp-comprehensive-test/
---

# Internal Ramp Comprehensive Test

## Overview

The `internal_ramp_comprehensive_test.cpp` provides comprehensive testing for TMC51x0 driver (TMC5130 & TMC5160) covering all major features in a single wholesome test suite. This merged test combines core initialization, motor control, ramp control, diagnostics, and protection features into one integrated test suite designed for SPI internal ramp mode operation.

## Purpose

This test suite is ideal for:
- Comprehensive validation of all TMC51x0 driver (TMC5130 & TMC5160) features
- Testing complete motor control workflows
- Validating feature interactions and compatibility
- End-to-end testing of motor control systems
- Production validation and quality assurance

## Test Structure

The test suite is organized into five main categories:

### 1. Core Tests
- Driver initialization and basic setup
- Register read/write operations
- Motor parameter configuration
- Ramp parameter setup
- Global configuration settings

### 2. Motor Control Tests
- Enable/disable functionality
- Current control (IRUN, IHOLD)
- Chopper configuration (SpreadCycle)
- StealthChop configuration and automatic tuning
- Mode change speeds (PWM, CoolStep, high-speed thresholds)
- Global scaler
- Freewheeling modes
- CoolStep configuration
- DCStep configuration
- Microstep lookup table (LUT)
- Motor setup from specifications

### 3. Ramp Control Tests
- All ramp modes (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD)
- Position control (target, current, relative moves)
- Speed control (max speed, acceleration, deceleration)
- Ramp speeds (start, stop, transition)
- Power-down delay and zero wait time
- First acceleration phase
- Reference switch/endstop configuration
- Unit conversions (mm, RPM, degrees)

### 4. Diagnostics Tests
- Driver status monitoring
- StallGuard2 configuration and reading
- Lost steps detection
- Phase currents
- PWM scale
- Microstep counter and current
- Time between microsteps
- GPIO pin reading
- Factory and OTP configuration
- UART transmission count
- Offset calibration
- Sensorless homing
- Open load detection

### 5. Protection Tests
- Short circuit protection configuration
- Overtemperature protection monitoring

## Hardware Requirements

- ESP32 development board (ESP32, ESP32-C3, ESP32-C6, etc.)
- TMC5160 stepper motor driver board (Evaluation Kit recommended)
- 17HS4401S-PG518 geared stepper motor (5.18:1 gearbox) or compatible
- AS5047U encoder (optional, for encoder feedback)
- Two reference switches (endstops) - configured but do NOT stop motor unless testing that feature
- SPI connection between ESP32 and TMC5160
- Power supply: 12-36V DC (ensure adequate current capacity)
- **Chip must be in SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Pin Configuration

Default pin configuration (from `esp32_tmc51x0_test_config.hpp`):

- **SPI**: MOSI=6, MISO=2, SCLK=5, CS=18
- **Control**: EN=11
- **Clock**: CLK=10 (tied to GND for internal clock)
- **Diagnostics**: DIAG0=23, DIAG1=15
- **SPI Clock**: 500 kHz (from config)
- **Reference Switches**: Configured via platform config (left/right endstops)
- **Encoder**: AS5047U configured via platform config

## Motor Selection

Uses unified test rig selection (`SELECTED_TEST_RIG`) which automatically configures motor, board, and platform. Default is `TEST_RIG_CORE_DRIVER` (17HS4401S gearbox):

```cpp
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER;
```

Available test rigs:
- `TEST_RIG_CORE_DRIVER` (default): 17HS4401S with 5.18:1 gearbox, TMC51x0 EVAL board, reference switches, encoder
- `TEST_RIG_FATIGUE`: Applied Motion 5034-369 NEMA 34, TMC51x0 EVAL board, reference switches, encoder

See [Motor Configuration Guide](motor_configuration.md) for detailed specifications.

## Platform Configuration

The test uses `PLATFORM_CORE_DRIVER_TEST_RIG` (via `TEST_RIG_CORE_DRIVER`) which includes:
- Reference switches (endstops) with proper configuration
- AS5047U encoder support
- Mechanical system configuration (gearbox)

**Important Note**: Reference switches are configured but do NOT stop the motor during normal tests. They only stop the motor when explicitly testing the reference switch feature (`test_reference_switch_configuration`). This prevents unintended stops during comprehensive testing.

## Test Configuration

Tests can be enabled/disabled via compile-time constants at the top of the file:

### Core Tests
```cpp
static constexpr bool ENABLE_INITIALIZATION_TESTS = true;
static constexpr bool ENABLE_REGISTER_ACCESS_TESTS = true;
static constexpr bool ENABLE_MOTOR_PARAMETER_TESTS = true;
static constexpr bool ENABLE_RAMP_PARAMETER_TESTS = true;
static constexpr bool ENABLE_GLOBAL_CONFIG_TESTS = true;
```

### Motor Control Tests
```cpp
static constexpr bool ENABLE_ENABLE_DISABLE_TESTS = true;
static constexpr bool ENABLE_CURRENT_CONTROL_TESTS = true;
static constexpr bool ENABLE_CHOPPER_TESTS = true;
static constexpr bool ENABLE_STEALTHCHOP_TESTS = true;
static constexpr bool ENABLE_MODE_CHANGE_SPEED_TESTS = true;
static constexpr bool ENABLE_GLOBAL_SCALER_TESTS = true;
static constexpr bool ENABLE_FREEWHEELING_TESTS = true;
static constexpr bool ENABLE_COOLSTEP_TESTS = true;
static constexpr bool ENABLE_DCSTEP_TESTS = true;
static constexpr bool ENABLE_LUT_TESTS = true;
static constexpr bool ENABLE_MOTOR_SETUP_TESTS = true;
```

### Ramp Control Tests
```cpp
static constexpr bool ENABLE_RAMP_MODE_TESTS = true;
static constexpr bool ENABLE_POSITION_CONTROL_TESTS = true;
static constexpr bool ENABLE_SPEED_CONTROL_TESTS = true;
static constexpr bool ENABLE_RAMP_PARAMETER_TESTS_RAMP = true;
static constexpr bool ENABLE_REFERENCE_SWITCH_TESTS = true;
static constexpr bool ENABLE_UNIT_CONVERSION_TESTS = true;
```

### Diagnostics Tests
```cpp
static constexpr bool ENABLE_DRIVER_STATUS_TESTS = true;
static constexpr bool ENABLE_STALLGUARD_TESTS = true;
static constexpr bool ENABLE_LOST_STEPS_TESTS = true;
static constexpr bool ENABLE_PHASE_CURRENT_TESTS = true;
static constexpr bool ENABLE_PWM_SCALE_TESTS = true;
static constexpr bool ENABLE_MICROSTEP_DIAGNOSTICS_TESTS = true;
static constexpr bool ENABLE_GPIO_TESTS = true;
static constexpr bool ENABLE_FACTORY_OTP_TESTS = true;
static constexpr bool ENABLE_UART_COUNT_TESTS = true;
static constexpr bool ENABLE_OFFSET_CALIBRATION_TESTS = true;
static constexpr bool ENABLE_SENSORLESS_HOMING_TESTS = true;
static constexpr bool ENABLE_OPEN_LOAD_TESTS = true;
```

### Protection Tests
```cpp
static constexpr bool ENABLE_SHORT_CIRCUIT_TESTS = true;
static constexpr bool ENABLE_OVERTEMPERATURE_TESTS = true;
```

### Encoder Tests
```cpp
static constexpr bool ENABLE_ENCODER_CONFIG_TESTS = true;
static constexpr bool ENABLE_ENCODER_RESOLUTION_TESTS = true;
static constexpr bool ENABLE_ENCODER_POSITION_TESTS = true;
static constexpr bool ENABLE_DEVIATION_DETECTION_TESTS = true;
static constexpr bool ENABLE_LATCHED_POSITION_TESTS = true;
```

## Test Execution

### Running Tests

1. **Build the test**: `./scripts/build_app.sh internal_ramp_comprehensive_test Debug` or `idf.py build`
2. **Flash to ESP32**: `./scripts/flash_app.sh --port /dev/ttyACM0 flash internal_ramp_comprehensive_test Debug` or `idf.py flash`
3. **Monitor output**: `idf.py monitor` or `./scripts/flash_app.sh flash_monitor internal_ramp_comprehensive_test Debug`

### Test Output Format

Tests use a structured output format with verbose logging:

```
I (1234) InternalRamp_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) InternalRamp_Test: ║         ESP32 TMC5160 INTERNAL RAMP COMPREHENSIVE TEST SUITE                  ║
I (1236) InternalRamp_Test: ║                         HardFOC TMC51x0 Driver (TMC5130 & TMC5160) Tests                           ║
I (1237) InternalRamp_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1238) InternalRamp_Test: 
I (1239) InternalRamp_Test: Configuration:
I (1240) InternalRamp_Test:   Motor: 17HS4401S-PG518 (gearbox)
I (1241) InternalRamp_Test:   Board: TMC5160 Evaluation Kit
I (1242) InternalRamp_Test:   Platform: Test Rig (with AS5047U encoder and reference switches)
I (1243) InternalRamp_Test:   Communication Mode: SPI Internal Ramp (SPI_MODE=HIGH, SD_MODE=LOW)
I (1244) InternalRamp_Test: 
I (1245) InternalRamp_Test: [SUCCESS] PASSED: driver_initialization (123.45 ms)
I (1246) InternalRamp_Test: [SUCCESS] PASSED: register_read_write (45.67 ms)
...
```

### Test Results Summary

At the end of execution, a comprehensive summary is displayed:

```
╔══════════════════════════════════════════════════════════════════════════════╗
║                          TEST SUMMARY                                         ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ Total Tests: 45                                                               ║
║ Passed: 43                                                                   ║
║ Failed: 2                                                                    ║
║ Success: 95.56%                                                               ║
║ Total Time: 1234.56 ms                                                       ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

The test also includes a feature compatibility section at the end:

```
╔══════════════════════════════════════════════════════════════════════════════╗
║                    FEATURE COMPATIBILITY NOTES                               ║
╚══════════════════════════════════════════════════════════════════════════════╝

Features that can be used together:
  • StealthChop + CoolStep: Compatible, CoolStep adjusts current based on StallGuard
  • StealthChop + DCStep: Compatible, DCStep activates at higher speeds
  • SpreadCycle + CoolStep: Compatible, standard combination
  • SpreadCycle + DCStep: Compatible, DCStep for high-speed operation
  • StallGuard2 + Sensorless Homing: Compatible, StallGuard2 enables sensorless homing
  • Reference Switches + Encoder: Compatible, both can be used for position feedback
  • Microstep LUT + Interpolation: Compatible, LUT customizes microstep waveform

Features that are mutually exclusive:
  • StealthChop vs SpreadCycle: Only one chopper mode active at a time
  • CoolStep vs DCStep: Both adjust current, but for different speed ranges
    (Can be configured with non-overlapping velocity thresholds)

Feature interactions:
  • Mode Change Speeds: Control transitions between StealthChop and SpreadCycle
  • Global Scaler: Affects all current-based features (CoolStep, DCStep, etc.)
  • Freewheeling: Only active in StealthChop mode
  • Open Load Detection: Requires SpreadCycle mode (StealthChop disabled)
```

## Key Test Features

### Core Initialization
- Driver initialization with full configuration
- Register access verification
- Motor parameter validation
- Ramp parameter setup
- Global configuration testing

### Motor Control
- Enable/disable with state verification
- Current control (IRUN, IHOLD) with calculation verification
- Chopper configuration (SpreadCycle mode)
- StealthChop with automatic tuning sequence (AT#1 and AT#2)
- Mode change speeds for smooth transitions
- Global scaler for current range adjustment
- Freewheeling modes
- CoolStep load-adaptive current reduction
- DCStep for high-speed operation
- Microstep lookup table customization
- Motor setup from specifications

### Ramp Control
- All ramp modes (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD)
- Position control with target and current position
- Speed control with acceleration/deceleration
- Ramp speed parameters (VSTART, VSTOP, V1)
- Power-down delay and zero wait time
- First acceleration phase
- Reference switch configuration (with stop enable only when testing)
- Unit conversions (mm, RPM, steps)

### Diagnostics
- Driver status monitoring
- StallGuard2 configuration and reading (requires SpreadCycle)
- Lost steps detection
- Phase current reading
- PWM scale monitoring (StealthChop calibration)
- Microstep diagnostics (counter, current, timing)
- GPIO pin reading
- Factory/OTP configuration reading
- UART transmission count
- Offset calibration
- Sensorless homing (requires SpreadCycle and StallGuard2)
- Open load detection (requires SpreadCycle)

### Protection
- Short circuit protection configuration (S2VS, S2G)
- Overtemperature protection monitoring (OTPW, OT)

### Encoder Features
- **Configuration**: setup, enable/disable, clearing options
- **Resolution**: setting and verifying pulses per revolution
- **Position**: reading current position, accuracy verification
- **Deviation**: configuring thresholds, detecting deviations between motor and encoder
- **Latching**: position latching on events

## Important Notes

### Reference Switches

**Reference switches are configured but do NOT stop the motor during normal tests.** This prevents unintended stops during comprehensive testing. The switches only stop the motor when explicitly testing the reference switch feature (`test_reference_switch_configuration`).

### StealthChop vs SpreadCycle

- **StealthChop**: Silent operation, requires automatic tuning (AT sequence)
- **SpreadCycle**: Required for StallGuard2, CoolStep, and open load detection
- Mode change speeds control automatic transitions between modes

### StallGuard2 Requirements

StallGuard2 requires:
- **SpreadCycle mode** (StealthChop disabled)
- Motor movement (minimum velocity threshold)
- Proper threshold (SGT) tuning

### Feature Compatibility

The test suite includes comprehensive feature compatibility notes at the end, documenting:
- Features that work together
- Mutually exclusive features
- Feature interactions and dependencies

## Expected Behavior

### Test Execution Flow

1. **Initialization**: Driver and SPI interface setup with mode pin verification
2. **Core Tests**: Basic initialization and configuration validation
3. **Motor Control Tests**: All motor control features tested sequentially
4. **Ramp Control Tests**: Ramp generator features validated
5. **Diagnostics Tests**: Diagnostic features tested (some require mode switching)
6. **Protection Tests**: Protection features configured and monitored
7. **Encoder Tests**: Encoder configuration, resolution, position, deviation, and latching validated
8. **Summary**: Comprehensive test results and feature compatibility notes

### Typical Test Duration

- Full test suite: ~30-60 seconds (depending on enabled tests)
- Individual test categories: ~5-10 seconds each
- StealthChop AT sequence: ~2 seconds (includes standstill and motion phases)

## Troubleshooting

### Common Issues

#### Tests Failing
**Symptoms**: Tests report failures

**Solutions**:
1. Check hardware connections (SPI, power, motor)
2. Verify power supply (12-36V, adequate current)
3. Check SPI communication (verify CS pin)
4. Verify motor is connected and not mechanically bound
5. Check chip communication mode (SPI_INTERNAL_RAMP)
6. Verify motor configuration matches hardware

#### StealthChop Not Calibrating
**Symptoms**: PWM_SCALE_AUTO remains 0

**Solutions**:
1. Ensure IRUN ≥ 8 (minimum for StealthChop)
2. Wait for standstill calibration (130ms+ for AT#1)
3. Move motor for motion calibration (AT#2 requires motion)
4. Check motor current is adequate
5. Verify StealthChop is enabled (GCONF.en_pwm_mode = 1)

#### StallGuard2 Not Working
**Symptoms**: SG_RESULT always 0 or doesn't change

**Solutions**:
1. **Verify SpreadCycle mode**: StealthChop must be disabled
2. Check TCOOLTHRS threshold (must be below motor velocity)
3. Verify SGT threshold is appropriate for your motor/load
4. Check motor is actually moving
5. Verify StallGuard2 is enabled in GCONF

#### Reference Switches Stopping Motor Unexpectedly
**Symptoms**: Motor stops during tests (not in reference switch test)

**Solutions**:
1. This should not happen - switches are configured with stop disabled
2. Check if `test_reference_switch_configuration` is running
3. Verify switch wiring (may have noise or false triggers)
4. Check switch polarity configuration

#### Open Load Detection Not Working
**Symptoms**: Open load not detected

**Solutions**:
1. Verify SpreadCycle mode (StealthChop disabled)
2. Motor must be moving (minimum 4× microstep resolution)
3. Check motor wiring for actual open load condition
4. Verify open load detection is enabled

## Customization

### Enabling/Disabling Test Categories

Modify test enable flags at the top of the file:

```cpp
static constexpr bool ENABLE_STEALTHCHOP_TESTS = false;  // Disable StealthChop tests
```

### Modifying Test Parameters

Change test values in the configuration section:

```cpp
static constexpr uint8_t TEST_IRUN = 25;  // Change test IRUN value
static constexpr float TEST_MAX_SPEED = STEPS_PER_REV * 2.0f;  // Change test speed
```

### Adding Custom Tests

Add new test functions following the existing pattern:

```cpp
bool test_custom_feature() noexcept {
  ESP_LOGI(TAG, "Testing custom feature...");
  
  auto handle = create_test_driver();
  if (!handle) {
    return false;
  }
  
  // Test code here
  if (condition) {
    ESP_LOGI(TAG, "✓ Custom feature test passed");
    return true;
  } else {
    ESP_LOGE(TAG, "✗ Custom feature test failed");
    return false;
  }
}
```

Then add to the appropriate test section in `app_main()`.

## Related Documentation

- [Motor Configuration Guide](motor_configuration.md) - Motor selection and configuration
- [Driver Configuration Guide](driver_configuration_guide.md) - Driver setup details
- [Dev Board Setup](dev_board_setup.md) - Hardware setup instructions

## Test Coverage

This comprehensive test suite covers:

- ✅ Core initialization and configuration
- ✅ Register access (read/write)
- ✅ Motor control (enable/disable, current, chopper)
- ✅ StealthChop and automatic tuning
- ✅ SpreadCycle mode
- ✅ CoolStep and DCStep
- ✅ All ramp modes
- ✅ Position and speed control
- ✅ Reference switches
- ✅ Diagnostics (StallGuard2, lost steps, phase currents)
- ✅ Sensorless homing
- ✅ Open load detection
- ✅ Protection features
- ✅ Encoder configuration and operation
- ✅ Feature compatibility validation

The test suite provides thorough validation of all TMC51x0 driver (TMC5130 & TMC5160) features in a single integrated test, making it ideal for production validation and comprehensive system testing.

---

**Navigation:** [← Previous: Fatigue Testing](fatigue_test.md) | [Index](index.md) | [Next: SPI Daisy Chain Test →](spi_daisy_chain_comprehensive_test.md)
