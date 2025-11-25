# Diagnostics Comprehensive Test

## Overview

The `diagnostics_comprehensive_test.cpp` provides comprehensive testing for TMC5160 diagnostics features including driver status, StallGuard2, lost steps detection, phase currents, PWM scale, microstep diagnostics, GPIO pin reading, factory/OTP configuration, UART transmission count, offset calibration, and sensorless homing.

## Purpose

This test suite is ideal for:
- Validating driver status monitoring
- Testing StallGuard2 configuration and operation
- Verifying lost steps detection
- Testing phase current reading
- Validating PWM scale monitoring
- Testing microstep diagnostics
- Verifying GPIO pin reading
- Testing sensorless homing

## Test Categories

### 1. Driver Status Tests

Tests driver status monitoring:
- Status register reading
- Error flag detection
- Warning flag detection
- Status bit interpretation

### 2. StallGuard2 Tests

Tests StallGuard2 sensorless load detection:
- StallGuard2 configuration
- SG_RESULT reading
- Stall threshold setting
- Stall detection behavior
- **Important**: Requires SpreadCycle mode (StealthChop disabled)

### 3. Lost Steps Tests

Tests lost steps detection:
- Lost steps configuration
- Lost steps reading
- Step loss detection

### 4. Phase Current Tests

Tests phase current reading:
- Current reading (IA, IB)
- Current calculation
- Current monitoring

### 5. PWM Scale Tests

Tests PWM scale monitoring:
- PWM_SCALE reading
- PWM_SCALE_AUTO reading
- StealthChop calibration status
- PWM amplitude monitoring

### 6. Microstep Diagnostics Tests

Tests microstep diagnostics:
- Microstep counter reading
- Microstep current reading
- Time between microsteps (TSTEP)
- Microstep position

### 7. GPIO Tests

Tests GPIO pin reading:
- DIAG0 pin reading
- DIAG1 pin reading
- GPIO signal reading
- Pin state monitoring

### 8. Factory/OTP Tests

Tests factory and OTP configuration:
- Factory configuration reading
- OTP configuration reading
- Configuration verification

### 9. UART Count Tests

Tests UART transmission counting:
- UART transmission count reading
- Count verification

### 10. Offset Calibration Tests

Tests offset calibration:
- Offset calibration reading
- Calibration values

### 11. Sensorless Homing Tests

Tests sensorless homing using StallGuard2:
- Homing configuration
- Homing execution
- Stall detection
- Position setting
- **Important**: Requires SpreadCycle mode

## Hardware Requirements

- ESP32 development board
- TMC5160 stepper motor driver board
- Single stepper motor connected to TMC5160
- SPI connection between ESP32 and TMC5160
- Optional: Mechanical stops for homing tests
- Chip must be in **SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Pin Configuration

Default pin configuration (from `esp32_tmc5160_bus_config.hpp`):

- **SPI**: MOSI=6, MISO=2, SCLK=5, CS=18
- **Control**: EN=11
- **Clock**: CLK=10 (tied to GND for internal clock)
- **Diagnostics**: DIAG0=23, DIAG1=15
- **SPI Clock**: 500 kHz (from config)

## Motor Selection

Uses `MotorConfig_17HS4401S` by default. See [Motor Configuration Guide](motor_configuration.md) for options.

## Test Configuration

Tests can be enabled/disabled:

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
```

## Important Notes

### StallGuard2 and StealthChop

**StallGuard2 and StealthChop are mutually exclusive!**

- StallGuard2 requires **SpreadCycle mode** (voltage-driven mode)
- StealthChop is a voltage-driven PWM mode that prevents back-EMF measurement
- To use StallGuard2, you **must** disable StealthChop (`GCONF.en_pwm_mode = 0`)
- After StallGuard2 operations, you can re-enable StealthChop if desired

### Sensorless Homing

Sensorless homing uses StallGuard2 to detect mechanical stops:
- Requires SpreadCycle mode
- Motor must be moving to detect stalls
- Stall threshold must be tuned for your motor/load
- Homing direction can be positive or negative

## Detailed Test Descriptions

### Driver Status Tests

#### Test: Driver Status Reading
- **Purpose**: Verify driver status can be read
- **Steps**:
  1. Read DRV_STATUS register
  2. Parse status bits
  3. Verify status interpretation
- **Expected**: Status read successfully, bits interpreted correctly

#### Test: Error Flag Detection
- **Purpose**: Verify error flags can be detected
- **Steps**:
  1. Read status register
  2. Check error flags (overtemperature, short circuit, etc.)
  3. Verify flag interpretation
- **Expected**: Error flags detected correctly

### StallGuard2 Tests

#### Test: StallGuard2 Configuration
- **Purpose**: Verify StallGuard2 can be configured
- **Steps**:
  1. Disable StealthChop (enable SpreadCycle)
  2. Configure StallGuard2 threshold (SGT)
  3. Set TCOOLTHRS (velocity threshold)
  4. Verify configuration
- **Expected**: StallGuard2 configured correctly

**Note**: StealthChop must be disabled for StallGuard2 to work.

#### Test: SG_RESULT Reading
- **Purpose**: Verify StallGuard2 result can be read
- **Steps**:
  1. Configure StallGuard2
  2. Move motor
  3. Read SG_RESULT
  4. Verify result changes with load
- **Expected**: SG_RESULT reflects motor load

#### Test: Stall Detection
- **Purpose**: Verify stall can be detected
- **Steps**:
  1. Configure StallGuard2 with appropriate threshold
  2. Move motor into mechanical stop
  3. Verify stall detected
  4. Check stall flag
- **Expected**: Stall detected when motor hits stop

### Lost Steps Tests

#### Test: Lost Steps Reading
- **Purpose**: Verify lost steps can be read
- **Steps**:
  1. Configure lost steps detection
  2. Read lost steps count
  3. Verify reading works
- **Expected**: Lost steps count readable

### Phase Current Tests

#### Test: Phase Current Reading
- **Purpose**: Verify phase currents can be read
- **Steps**:
  1. Enable motor
  2. Read IA and IB currents
  3. Verify current values
- **Expected**: Phase currents readable

### PWM Scale Tests

#### Test: PWM_SCALE Reading
- **Purpose**: Verify PWM scale can be read
- **Steps**:
  1. Enable StealthChop
  2. Read PWM_SCALE
  3. Verify scale value
- **Expected**: PWM_SCALE readable

#### Test: StealthChop Calibration Status
- **Purpose**: Verify StealthChop calibration status
- **Steps**:
  1. Enable StealthChop
  2. Read PWM_SCALE_AUTO
  3. Verify calibration status
- **Expected**: Calibration status readable

### Sensorless Homing Tests

#### Test: Sensorless Homing
- **Purpose**: Verify sensorless homing works
- **Steps**:
  1. Disable StealthChop (enable SpreadCycle)
  2. Configure StallGuard2
  3. Execute homing in negative direction
  4. Verify stall detected
  5. Set home position
  6. Execute homing in positive direction
  7. Verify stall detected
- **Expected**: Homing detects stalls and sets position correctly

**Note**: Requires SpreadCycle mode and mechanical stops.

## Expected Behavior

### Test Execution

1. **Initialization**: Driver setup
2. **Status Tests**: Driver status monitoring
3. **StallGuard2 Tests**: StallGuard2 configuration and testing (SpreadCycle mode)
4. **Diagnostic Tests**: Various diagnostic features
5. **Homing Tests**: Sensorless homing (if enabled)
6. **Summary**: Test results displayed

### Typical Output

```
I (1234) Diagnostics_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) Diagnostics_Test: ║            Diagnostics Comprehensive Test Suite                                 ║
I (1236) Diagnostics_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) Diagnostics_Test: [PASS] Driver Status: Status read successfully
I (1238) Diagnostics_Test: [PASS] StallGuard2 Configuration: SG2 configured
I (1239) Diagnostics_Test: ⚠️ Switching to SpreadCycle for StallGuard2 tests
I (1240) Diagnostics_Test: [PASS] SG_RESULT Reading: SG_RESULT=128
...
```

## Troubleshooting

### StallGuard2 Not Working

**Symptoms**: SG_RESULT always 0 or doesn't change

**Solutions**:
1. **Verify SpreadCycle mode**: StealthChop must be disabled
2. Check TCOOLTHRS threshold (must be below motor velocity)
3. Verify SGT threshold is appropriate
4. Check motor is actually moving
5. Verify StallGuard2 is enabled in GCONF

### Sensorless Homing Fails

**Symptoms**: Homing doesn't detect stalls

**Solutions**:
1. Verify SpreadCycle mode is active
2. Check StallGuard2 threshold (SGT) is tuned correctly
3. Verify motor current is adequate
4. Check mechanical stops are present
5. Verify TCOOLTHRS is set correctly

### PWM_SCALE_AUTO Always 0

**Symptoms**: StealthChop not calibrating

**Solutions**:
1. Ensure IRUN ≥ 8 (minimum for StealthChop)
2. Wait for standstill calibration (130ms+)
3. Move motor for motion calibration
4. Check motor current is adequate

## Related Documentation

- [Motor Control Test](motor_control_comprehensive_test.md) - StealthChop configuration
- [Bounds Finding Example](bounds_finding_sinuous_motion.md) - Sensorless homing usage
- [Special Features: Sensorless Homing](../../../docs/special_features_sensorless_homing.md) - Detailed homing guide

