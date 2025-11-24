# Core Comprehensive Test

## Overview

The `core_comprehensive_test.cpp` provides comprehensive testing for TMC5160 core initialization and basic setup functionality. This test suite validates fundamental driver operations including initialization, register access, motor parameter configuration, and ramp parameter setup.

## Purpose

This test suite is ideal for:
- Validating driver initialization
- Testing register read/write operations
- Verifying motor parameter configuration
- Testing ramp parameter setup
- Validating global configuration settings
- Testing register persistence

## Test Categories

### 1. Initialization Tests

Tests driver initialization and basic setup:
- Driver initialization with configuration
- Configuration verification
- Mode pin verification
- Communication mode detection

### 2. Register Access Tests

Tests register read/write operations:
- Read-only register access
- Write-only register access
- Read-write register access
- Register persistence verification

### 3. Motor Parameter Tests

Tests motor configuration:
- Current settings (IRUN, IHOLD)
- Global scaler configuration
- Microstep resolution
- Chopper configuration
- Parameter verification

### 4. Ramp Parameter Tests

Tests ramp generator configuration:
- Maximum speed setting
- Acceleration/deceleration setting
- Ramp speeds (start, stop, transition)
- Parameter verification

### 5. Global Config Tests

Tests global configuration:
- GCONF register settings
- StealthChop enable/disable
- SpreadCycle configuration
- Configuration persistence

## Hardware Requirements

- ESP32 development board
- TMC5160 stepper motor driver board
- Single stepper motor connected to TMC5160
- SPI connection between ESP32 and TMC5160
- Chip must be in **SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Pin Configuration

Default pin configuration (can be modified):

- **SPI**: MOSI=23, MISO=19, SCLK=18, CS=5
- **Control**: EN=2, DIR=4, STEP=15

## Motor Selection

Uses `MotorConfig_17HS4401S` by default. Motor selection can be modified at the top of the file:

```cpp
namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
```

See [Motor Configuration Guide](motor_configuration.md) for options.

## Test Configuration

Tests can be enabled/disabled via compile-time constants:

```cpp
static constexpr bool ENABLE_INITIALIZATION_TESTS = true;
static constexpr bool ENABLE_REGISTER_ACCESS_TESTS = true;
static constexpr bool ENABLE_MOTOR_PARAMETER_TESTS = true;
static constexpr bool ENABLE_RAMP_PARAMETER_TESTS = true;
static constexpr bool ENABLE_GLOBAL_CONFIG_TESTS = true;
```

## Test Execution

### Running Tests

1. Build the test: `idf.py build`
2. Flash to ESP32: `idf.py flash`
3. Monitor output: `idf.py monitor`

### Test Output Format

Tests use a structured output format:

```
[PASS] Test Name: Description
[FAIL] Test Name: Description - Reason
[SKIP] Test Name: Description - Reason
```

### Test Results

At the end of execution, a summary is displayed:

```
╔══════════════════════════════════════════════════════════════════════════════╗
║                            TEST SUMMARY                                       ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ Total Tests: 25                                                               ║
║ Passed: 23                                                                   ║
║ Failed: 2                                                                    ║
║ Skipped: 0                                                                   ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

## Detailed Test Descriptions

### Initialization Tests

#### Test: Driver Initialization
- **Purpose**: Verify driver initializes correctly
- **Steps**:
  1. Create driver instance
  2. Configure driver settings
  3. Initialize driver
  4. Verify initialization success
- **Expected**: Driver initializes without errors

#### Test: Configuration Verification
- **Purpose**: Verify configuration is applied correctly
- **Steps**:
  1. Set motor parameters
  2. Read back configuration
  3. Compare with expected values
- **Expected**: Configuration matches expected values

#### Test: Mode Pin Verification
- **Purpose**: Verify chip communication mode
- **Steps**:
  1. Read mode pins (if configured)
  2. Verify communication mode
  3. Check mode matches expected
- **Expected**: Mode matches expected (SPI_INTERNAL_RAMP)

### Register Access Tests

#### Test: Read-Only Register Access
- **Purpose**: Verify read-only registers can be read but not written
- **Steps**:
  1. Read read-only register (e.g., DRV_STATUS)
  2. Attempt to write (should fail gracefully)
  3. Verify read value unchanged
- **Expected**: Read succeeds, write fails or is ignored

#### Test: Write-Only Register Access
- **Purpose**: Verify write-only registers can be written but read as 0
- **Steps**:
  1. Write to write-only register (e.g., IHOLD_IRUN)
  2. Attempt to read (should return 0)
  3. Verify write was accepted
- **Expected**: Write succeeds, read returns 0

#### Test: Read-Write Register Access
- **Purpose**: Verify read-write registers work correctly
- **Steps**:
  1. Read initial value
  2. Write new value
  3. Read back and verify
- **Expected**: Read and write both succeed, values match

#### Test: Register Persistence
- **Purpose**: Verify registers persist across operations
- **Steps**:
  1. Write value to register
  2. Perform other operations
  3. Read register and verify value unchanged
- **Expected**: Register value persists

### Motor Parameter Tests

#### Test: Current Settings
- **Purpose**: Verify IRUN and IHOLD configuration
- **Steps**:
  1. Set IRUN and IHOLD values
  2. Verify configuration applied
  3. Check current calculation
- **Expected**: Current settings match configured values

#### Test: Global Scaler
- **Purpose**: Verify GLOBAL_SCALER configuration
- **Steps**:
  1. Set GLOBAL_SCALER value
  2. Verify configuration applied
  3. Check current scaling
- **Expected**: Global scaler affects current range correctly

#### Test: Microstep Resolution
- **Purpose**: Verify microstep resolution setting
- **Steps**:
  1. Set MRES value
  2. Verify microstep resolution
  3. Check step resolution
- **Expected**: Microstep resolution matches MRES setting

#### Test: Chopper Configuration
- **Purpose**: Verify chopper parameters
- **Steps**:
  1. Set chopper parameters (TOFF, HEND, HSTRT, TBL)
  2. Read back configuration
  3. Verify parameters match
- **Expected**: Chopper parameters configured correctly

### Ramp Parameter Tests

#### Test: Maximum Speed
- **Purpose**: Verify VMAX setting
- **Steps**:
  1. Set maximum speed
  2. Read back and verify
  3. Check speed limits
- **Expected**: Maximum speed set correctly

#### Test: Acceleration/Deceleration
- **Purpose**: Verify AMAX and DMAX settings
- **Steps**:
  1. Set acceleration and deceleration
  2. Read back and verify
  3. Check acceleration limits
- **Expected**: Acceleration/deceleration set correctly

#### Test: Ramp Speeds
- **Purpose**: Verify VSTART, VSTOP, V1 settings
- **Steps**:
  1. Set ramp speeds
  2. Read back and verify
  3. Check speed relationships
- **Expected**: Ramp speeds configured correctly

### Global Config Tests

#### Test: StealthChop Configuration
- **Purpose**: Verify StealthChop enable/disable
- **Steps**:
  1. Enable StealthChop
  2. Verify GCONF.en_pwm_mode set
  3. Disable StealthChop
  4. Verify GCONF.en_pwm_mode cleared
- **Expected**: StealthChop state changes correctly

#### Test: SpreadCycle Configuration
- **Purpose**: Verify SpreadCycle mode
- **Steps**:
  1. Disable StealthChop (enables SpreadCycle)
  2. Verify SpreadCycle active
  3. Check chopper operation
- **Expected**: SpreadCycle operates correctly

## Expected Behavior

### Test Execution Flow

1. **Initialization**: Driver and SPI interface setup
2. **Test Execution**: Each test category runs sequentially
3. **Result Collection**: Test results collected
4. **Summary Display**: Final summary shown

### Typical Output

```
I (1234) Core_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) Core_Test: ║                    Core Comprehensive Test Suite                             ║
I (1236) Core_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) Core_Test: [PASS] Driver Initialization: Driver initialized successfully
I (1238) Core_Test: [PASS] Configuration Verification: All parameters verified
I (1239) Core_Test: [PASS] Register Access: Read-write registers working correctly
...
I (1500) Core_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1501) Core_Test: ║                            TEST SUMMARY                                       ║
I (1502) Core_Test: ╠══════════════════════════════════════════════════════════════════════════════╣
I (1503) Core_Test: ║ Total Tests: 25                                                               ║
I (1504) Core_Test: ║ Passed: 25                                                                   ║
I (1505) Core_Test: ║ Failed: 0                                                                    ║
I (1506) Core_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
```

## Troubleshooting

### Tests Failing

**Symptoms**: Tests report failures

**Solutions**:
1. Check hardware connections
2. Verify power supply (12-36V)
3. Check SPI communication
4. Verify motor is connected
5. Check chip communication mode

### Register Access Failures

**Symptoms**: Register read/write tests failing

**Solutions**:
1. Verify SPI communication working
2. Check CS pin connection
3. Verify chip is powered
4. Check for communication errors
5. Verify register addresses

### Configuration Not Persisting

**Symptoms**: Configuration changes don't persist

**Solutions**:
1. Check if register is write-only (can't verify by reading)
2. Verify register write succeeded
3. Check for register write protection
4. Verify chip is not resetting

## Customization

### Enabling/Disabling Tests

Modify test enable flags:

```cpp
static constexpr bool ENABLE_INITIALIZATION_TESTS = false;  // Disable initialization tests
```

### Adding Custom Tests

Add new test functions:

```cpp
bool test_custom_feature() noexcept {
    TEST_START("Custom Feature Test");
    
    // Test code here
    if (condition) {
        TEST_PASS("Custom feature works correctly");
        return true;
    } else {
        TEST_FAIL("Custom feature failed");
        return false;
    }
}
```

### Modifying Test Parameters

Change test values:

```cpp
static constexpr uint8_t TEST_IRUN = 25;  // Change test IRUN value
static constexpr float TEST_VMAX = 50000.0f;  // Change test VMAX value
```

## Related Documentation

- [Motor Control Test](motor_control_comprehensive_test.md) - Motor control features
- [Ramp Control Test](ramp_control_comprehensive_test.md) - Ramp generator features
- [Motor Configuration Guide](motor_configuration.md) - Motor selection

