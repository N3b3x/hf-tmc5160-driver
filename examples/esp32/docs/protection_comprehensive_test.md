# Protection Comprehensive Test

## Overview

The `protection_comprehensive_test.cpp` provides comprehensive testing for TMC5160 protection features including short circuit protection and overtemperature protection.

## Purpose

This test suite is ideal for:
- Validating short circuit protection configuration
- Testing overtemperature protection
- Verifying protection thresholds
- Testing protection response
- Ensuring safe operation

## Test Categories

### 1. Short Circuit Protection Tests

Tests short circuit protection:
- Short circuit configuration
- Protection levels (S2VS, S2G)
- Short filter settings
- Short delay configuration
- Protection verification

### 2. Overtemperature Protection Tests

Tests overtemperature protection:
- Overtemperature threshold configuration
- Prewarning threshold
- Protection response
- Temperature monitoring

## Hardware Requirements

- ESP32 development board
- TMC5160 stepper motor driver board
- Single stepper motor connected to TMC5160
- SPI connection between ESP32 and TMC5160
- Chip must be in **SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Pin Configuration

Default pin configuration:

- **SPI**: MOSI=23, MISO=19, SCLK=18, CS=5
- **Control**: EN=2, DIR=4, STEP=15

## Motor Selection

Uses `MotorConfig_17HS4401S` by default. See [Motor Configuration Guide](motor_configuration.md) for options.

## Test Configuration

Tests can be enabled/disabled:

```cpp
static constexpr bool ENABLE_SHORT_CIRCUIT_TESTS = true;
static constexpr bool ENABLE_OVERTEMPERATURE_TESTS = true;
```

## Important Safety Notes

⚠️ **WARNING**: Protection tests verify configuration but do not intentionally trigger protection events. Intentionally causing short circuits or overheating can damage hardware.

### Short Circuit Protection

Short circuit protection detects:
- **S2VS**: Short to supply voltage
- **S2G**: Short to ground

Protection levels:
- **Level 0-7**: Increasing sensitivity
- Higher levels = more sensitive = faster detection

### Overtemperature Protection

Overtemperature protection includes:
- **Overtemperature Shutdown**: Automatic shutdown at high temperature
- **Overtemperature Prewarning**: Warning before shutdown
- **Temperature Monitoring**: Real-time temperature reading

## Detailed Test Descriptions

### Short Circuit Protection Tests

#### Test: Short Circuit Configuration
- **Purpose**: Verify short circuit protection can be configured
- **Steps**:
  1. Configure short circuit protection levels
  2. Set short filter and delay
  3. Verify configuration applied
- **Expected**: Short circuit protection configured correctly

**Configuration Parameters**:
- **S2VS Level**: Short to supply detection sensitivity (0-7)
- **S2G Level**: Short to ground detection sensitivity (0-7)
- **Short Filter**: Filter time for short detection
- **Short Delay**: Delay before protection triggers

#### Test: Protection Levels
- **Purpose**: Verify protection levels work correctly
- **Steps**:
  1. Set different protection levels
  2. Verify levels applied
  3. Test level sensitivity
- **Expected**: Protection levels configured correctly

### Overtemperature Protection Tests

#### Test: Overtemperature Configuration
- **Purpose**: Verify overtemperature protection can be configured
- **Steps**:
  1. Configure overtemperature threshold
  2. Set prewarning threshold
  3. Verify configuration
- **Expected**: Overtemperature protection configured correctly

#### Test: Temperature Monitoring
- **Purpose**: Verify temperature can be monitored
- **Steps**:
  1. Read temperature from status register
  2. Verify temperature reading
  3. Monitor temperature changes
- **Expected**: Temperature readable and accurate

## Expected Behavior

### Test Execution

1. **Initialization**: Driver setup
2. **Short Circuit Tests**: Short circuit protection configuration
3. **Overtemperature Tests**: Overtemperature protection configuration
4. **Summary**: Test results displayed

### Typical Output

```
I (1234) Protection_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) Protection_Test: ║            Protection Comprehensive Test Suite                                 ║
I (1236) Protection_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) Protection_Test: [PASS] Short Circuit Configuration: Protection configured
I (1238) Protection_Test: [PASS] Overtemperature Configuration: Protection configured
...
```

## Protection Configuration

### Short Circuit Protection

```cpp
tmc5160::ShortProtectionConfig short_cfg{};
short_cfg.s2vs_level = 6;      // Short to supply sensitivity (0-7)
short_cfg.s2g_level = 6;        // Short to ground sensitivity (0-7)
short_cfg.shortfilter = 1;      // Filter time
short_cfg.shortdelay = false;   // No delay

driver.protection.ConfigureShortProtection(short_cfg);
```

### Overtemperature Protection

Overtemperature protection is typically configured via:
- **GCONF**: Overtemperature prewarning enable
- **Status Register**: Temperature reading and flags
- **Automatic**: Chip automatically shuts down at high temperature

## Troubleshooting

### Protection Not Triggering

**Symptoms**: Protection doesn't activate when expected

**Solutions**:
1. Verify protection levels are set appropriately
2. Check protection is enabled
3. Verify protection thresholds
4. Check for protection flags in status register
5. Verify hardware connections

### False Protection Triggers

**Symptoms**: Protection triggers incorrectly

**Solutions**:
1. Adjust protection levels (reduce sensitivity)
2. Increase filter time
3. Check for electrical noise
4. Verify motor wiring
5. Check power supply stability

### Overtemperature Warnings

**Symptoms**: Overtemperature warnings appear

**Solutions**:
1. Check motor current (may be too high)
2. Verify adequate cooling
3. Check ambient temperature
4. Reduce motor current if needed
5. Improve heat sinking

## Related Documentation

- [Motor Control Test](motor_control_comprehensive_test.md) - Current control
- [Diagnostics Test](diagnostics_comprehensive_test.md) - Status monitoring

