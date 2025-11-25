# Motor Control Comprehensive Test

## Overview

The `motor_control_comprehensive_test.cpp` provides comprehensive testing for TMC5160 motor control features including enable/disable, current control, chopper configuration, StealthChop, mode change speeds, global scaler, freewheeling, CoolStep, DCStep, microstep lookup table, and motor setup from specifications.

## Purpose

This test suite is ideal for:
- Validating motor enable/disable functionality
- Testing current control (IRUN, IHOLD)
- Verifying chopper configuration
- Testing StealthChop operation
- Validating mode change speed thresholds
- Testing CoolStep and DCStep features
- Verifying motor setup from specifications

## Test Categories

### 1. Enable/Disable Tests

Tests motor enable/disable functionality:
- Enable motor
- Disable motor
- Enable state verification
- Disable state verification

### 2. Current Control Tests

Tests current setting and control:
- IRUN configuration
- IHOLD configuration
- Global scaler configuration
- Current calculation verification
- Current limits testing

### 3. Chopper Tests

Tests chopper configuration:
- TOFF setting
- HEND and HSTRT configuration
- TBL (blank time) setting
- Chopper mode verification

### 4. StealthChop Tests

Tests StealthChop operation:
- StealthChop enable/disable
- PWM frequency configuration
- PWM offset configuration
- Auto-calibration testing
- Calibration status verification

### 5. Mode Change Speed Tests

Tests velocity thresholds for mode switching:
- PWM threshold (StealthChop ↔ SpreadCycle)
- CoolStep threshold
- High-speed threshold
- Threshold hysteresis

### 6. Global Scaler Tests

Tests global current scaling:
- Global scaler configuration
- Current range adjustment
- Scaling verification

### 7. Freewheeling Tests

Tests freewheeling modes:
- Freewheeling enable/disable
- Freewheeling mode selection
- Power-down behavior

### 8. CoolStep Tests

Tests CoolStep load-adaptive current reduction:
- CoolStep configuration
- StallGuard2 dependency
- Current reduction behavior
- Threshold configuration

### 9. DCStep Tests

Tests DCStep (DC motor emulation):
- DCStep configuration
- Velocity threshold
- Current control

### 10. Microstep Lookup Table Tests

Tests microstep lookup table:
- LUT segmentation
- LUT start position
- LUT configuration

### 11. Motor Setup Tests

Tests motor setup from specifications:
- Setup from motor specs
- Automatic parameter calculation
- Configuration verification

## Hardware Requirements

- ESP32 development board
- TMC5160 stepper motor driver board
- Single stepper motor connected to TMC5160
- SPI connection between ESP32 and TMC5160
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

## Detailed Test Descriptions

### Enable/Disable Tests

#### Test: Motor Enable
- **Purpose**: Verify motor can be enabled
- **Steps**:
  1. Disable motor
  2. Enable motor
  3. Verify enable state
  4. Check CHOPCONF.toff > 0 (indicates enabled)
- **Expected**: Motor enables successfully

#### Test: Motor Disable
- **Purpose**: Verify motor can be disabled
- **Steps**:
  1. Enable motor
  2. Disable motor
  3. Verify disable state
  4. Check CHOPCONF.toff = 0 (indicates disabled)
- **Expected**: Motor disables successfully

### Current Control Tests

#### Test: IRUN Configuration
- **Purpose**: Verify IRUN current setting
- **Steps**:
  1. Set IRUN value
  2. Verify configuration applied
  3. Check current calculation
- **Expected**: IRUN set correctly, current calculated properly

#### Test: IHOLD Configuration
- **Purpose**: Verify IHOLD current setting
- **Steps**:
  1. Set IHOLD value
  2. Verify configuration applied
  3. Check hold current calculation
- **Expected**: IHOLD set correctly

#### Test: Global Scaler
- **Purpose**: Verify global current scaling
- **Steps**:
  1. Set GLOBAL_SCALER value
  2. Verify current range adjustment
  3. Check scaling effect
- **Expected**: Global scaler affects current range

### Chopper Tests

#### Test: TOFF Configuration
- **Purpose**: Verify chopper off-time setting
- **Steps**:
  1. Set TOFF value
  2. Read back CHOPCONF register
  3. Verify TOFF matches
- **Expected**: TOFF configured correctly

#### Test: HEND and HSTRT
- **Purpose**: Verify hysteresis end and start
- **Steps**:
  1. Set HEND and HSTRT values
  2. Verify configuration
  3. Check hysteresis behavior
- **Expected**: Hysteresis configured correctly

### StealthChop Tests

#### Test: StealthChop Enable
- **Purpose**: Verify StealthChop can be enabled
- **Steps**:
  1. Disable StealthChop (enable SpreadCycle)
  2. Enable StealthChop
  3. Verify GCONF.en_pwm_mode = 1
  4. Check PWM configuration
- **Expected**: StealthChop enables successfully

#### Test: StealthChop Disable
- **Purpose**: Verify StealthChop can be disabled
- **Steps**:
  1. Enable StealthChop
  2. Disable StealthChop
  3. Verify GCONF.en_pwm_mode = 0
  4. Check SpreadCycle active
- **Expected**: StealthChop disables, SpreadCycle active

#### Test: PWM Frequency
- **Purpose**: Verify PWM frequency setting
- **Steps**:
  1. Set PWM frequency
  2. Verify PWM_CONF register
  3. Check frequency value
- **Expected**: PWM frequency set correctly

#### Test: StealthChop Calibration
- **Purpose**: Verify StealthChop auto-calibration
- **Steps**:
  1. Enable StealthChop with auto-calibration
  2. Wait for calibration
  3. Check PWM_SCALE_AUTO value
  4. Verify calibration status
- **Expected**: StealthChop calibrates automatically

### CoolStep Tests

#### Test: CoolStep Configuration
- **Purpose**: Verify CoolStep setup
- **Steps**:
  1. Configure CoolStep parameters
  2. Verify COOLCONF register
  3. Check StallGuard2 dependency
- **Expected**: CoolStep configured correctly

**Note**: CoolStep requires SpreadCycle mode (StealthChop disabled) and StallGuard2.

### DCStep Tests

#### Test: DCStep Configuration
- **Purpose**: Verify DCStep setup
- **Steps**:
  1. Configure DCStep parameters
  2. Verify DCSTEP register
  3. Check velocity threshold
- **Expected**: DCStep configured correctly

## Expected Behavior

### Test Execution

1. **Initialization**: Driver setup
2. **Test Categories**: Each category runs sequentially
3. **Results**: Test results collected
4. **Summary**: Final summary displayed

### Typical Output

```
I (1234) MotorControl_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) MotorControl_Test: ║              Motor Control Comprehensive Test Suite                            ║
I (1236) MotorControl_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) MotorControl_Test: [PASS] Motor Enable: Motor enabled successfully
I (1238) MotorControl_Test: [PASS] IRUN Configuration: IRUN set to 20
I (1239) MotorControl_Test: [PASS] StealthChop Enable: StealthChop enabled
...
```

## Troubleshooting

### Enable/Disable Failures

**Symptoms**: Motor won't enable or disable

**Solutions**:
1. Check EN pin connection
2. Verify EN pin polarity
3. Check power supply
4. Verify driver initialization

### Current Not Setting

**Symptoms**: Current doesn't match configured values

**Solutions**:
1. Verify sense resistor value (0.05Ω assumed)
2. Check IRUN/IHOLD values
3. Verify GLOBAL_SCALER setting
4. Check current calculation formula

### StealthChop Not Calibrating

**Symptoms**: PWM_SCALE_AUTO remains 0

**Solutions**:
1. Ensure IRUN ≥ 8 (minimum for StealthChop)
2. Wait for standstill calibration (130ms+)
3. Move motor for motion calibration
4. Check motor current is adequate

### CoolStep Not Working

**Symptoms**: CoolStep doesn't activate

**Solutions**:
1. Verify SpreadCycle mode (StealthChop disabled)
2. Check StallGuard2 is configured
3. Verify TCOOLTHRS threshold
4. Check motor is moving above threshold

## Related Documentation

- [Core Test](core_comprehensive_test.md) - Basic initialization
- [Diagnostics Test](diagnostics_comprehensive_test.md) - StallGuard2 testing
- [Motor Configuration Guide](motor_configuration.md) - Motor selection

