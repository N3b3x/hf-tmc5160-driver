# Internal Ramp Sinusoidal Motion Example

## Overview

The `internal_ramp_sinusoidal.cpp` example demonstrates simple back-and-forth motion using the TMC5160's internal ramp generator in positioning mode. The motor moves between two positions repeatedly, creating a continuous oscillating motion pattern. Motion parameters are specified in degrees.

## Purpose

This example is ideal for:
- Demonstrating basic positioning control
- Testing motor smoothness and StealthChop calibration
- Validating motor configuration and current settings
- Understanding the TMC5160 positioning mode operation
- Simple motion testing and validation

## Key Features

- **Positioning Mode Control**: Uses TMC5160's internal ramp generator in positioning mode
- **Back-and-Forth Motion**: Continuous oscillation between two positions (configurable travel distance in degrees)
- **Test Rig Selection**: Configurable via `SELECTED_TEST_RIG` (`TEST_RIG_FATIGUE` default, or `TEST_RIG_CORE_DRIVER`)
- **Comprehensive Diagnostics**: Extensive logging of motor status, StealthChop calibration, and diagnostics
- **Cycle Counting**: Tracks completed back-and-forth cycles
- **Real-Time Monitoring**: Periodic diagnostic output showing position, velocity, and status

## Hardware Requirements

- ESP32 development board (ESP32, ESP32-C3, ESP32-C6, etc.)
- TMC5160 stepper motor driver board
- Stepper motor (see [Motor Configuration Guide](motor_configuration.md))
- SPI connection between ESP32 and TMC5160
- Power supply: 12-36V DC (ensure adequate current capacity for selected motor)
- Chip must be in **SPI_INTERNAL_RAMP mode** (SPI_MODE=HIGH, SD_MODE=LOW)

## Pin Configuration

Default pin configuration (from `esp32_tmc51x0_test_config.hpp`):

- **SPI**: MOSI=6, MISO=2, SCLK=5, CS=18
- **Control**: EN=11
- **Clock**: CLK=10 (tied to GND for internal clock)
- **Diagnostics**: DIAG0=23, DIAG1=15
- **SPI Clock**: 1 MHz (this example uses 1 MHz, config default is 500 kHz)

## Motor / Test Rig Selection

Motor and platform are selected via unified test rig selection at the top of the file:

```cpp
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
```

Available options:
- `TEST_RIG_FATIGUE` (default) - Applied Motion 5034-369 NEMA 34, sinusoidal/back-and-forth testing
- `TEST_RIG_CORE_DRIVER` - 17HS4401S with 5.18:1 gearbox, core driver test rig

See [Motor Configuration Guide](motor_configuration.md) for detailed specifications.

## How It Works

### Initialization

1. **SPI Interface Setup**: Initializes SPI communication with the TMC5160
2. **Driver Configuration**: Configures motor current, chopper settings, and StealthChop based on selected motor
3. **StealthChop Verification**: Checks if StealthChop is enabled and calibrated
4. **StallGuard2 Disabling**: Ensures StallGuard2 is disabled for continuous motion
5. **Reference Switch Disabling**: Disables reference switches to prevent motion blocking
6. **Motor Enable**: Enables the motor driver

### Motion Control

The `BackAndForthMotion` class manages the motion:

1. **Configuration**: Sets travel distance, max velocity, and acceleration
2. **Start**: Initializes positioning mode and sets first target position
3. **Update Loop**: 
   - Checks if target position reached
   - Switches direction when target reached
   - Updates target position for next leg
   - Tracks cycle count

### Motion Parameters

Default motion parameters (from `BackAndForthMotion` class):
- **Travel Distance**: 180° each direction (configurable via `Config()`)
- **Max Velocity**: 30 RPM (configurable)
- **Acceleration**: 1 rev/s² (configurable)
- **Cycles**: Infinite (-1) or limited (configurable via `max_cycles`)

## Expected Behavior

### Startup Sequence

1. Driver initialization messages
2. Motor configuration display
3. StealthChop calibration status check
4. Diagnostic pin status
5. Motion parameter display
6. Motion start confirmation

### During Operation

- Motor moves smoothly back and forth
- Position updates every 50ms
- Diagnostic output every 1 second showing:
  - Current position (steps and degrees)
  - Actual velocity
  - Position change rate
  - Motor and output shaft revolutions
  - StallGuard2 status (if applicable)
  - Charge pump status

### StealthChop Calibration

If StealthChop is enabled but not yet calibrated:
- Motor may not move initially
- Calibration occurs automatically:
  - **AT#1**: Standstill calibration (130ms+ at standstill with IRUN current)
  - **AT#2**: Motion calibration (move at 60-300 RPM for ~400 fullsteps)
- Once calibrated, motion becomes smooth and quiet

## Diagnostic Output

The example provides extensive diagnostic information:

### Startup Diagnostics

- Motor selection confirmation
- Driver initialization status
- StealthChop calibration status
- Motor current settings
- DIAG pin status
- Reference switch status
- Charge pump status

### Runtime Diagnostics (Every 1 Second)

- **Position**: Current position in steps and degrees
- **Velocity**: Actual velocity in steps/s
- **Movement**: Position change rate and motor/output revolutions
- **Status**: Ramp status, StallGuard2 status, charge pump status
- **Warnings**: Any issues detected (low velocity, stopped motor, etc.)

## Troubleshooting

### Motor Not Moving

**Symptoms**: VACTUAL shows 0, position doesn't change

**Solutions**:
1. Check StealthChop calibration status - may need time to calibrate
2. Verify motor current (IRUN) is adequate (≥8 for StealthChop, 16-31 recommended)
3. Check EN pin is properly connected and enabled
4. Verify power supply voltage (12-36V)
5. Check charge pump status (uv_cp flag)

### StealthChop Not Calibrating

**Symptoms**: PWM_SCALE_AUTO remains 0 or very small

**Solutions**:
1. Ensure IRUN ≥ 8 (minimum for StealthChop)
2. Wait at standstill for AT#1 calibration (130ms+)
3. Move motor at moderate speed (60-300 RPM) for AT#2 calibration
4. Check motor wiring (phases may be swapped)

### Motor Stops Unexpectedly

**Symptoms**: Motor stops mid-motion

**Solutions**:
1. Check charge pump undervoltage (uv_cp flag)
2. Verify power supply stability
3. Check for overtemperature warnings
4. Verify reference switches are disabled
5. Check StallGuard2 stop is disabled

### Incorrect Motion Distance

**Symptoms**: Motor doesn't travel expected distance

**Solutions**:
1. Verify gearbox ratio matches motor configuration
2. Check microstep resolution (should be 256)
3. Verify steps_per_rev calculation matches motor specs
4. Check for mechanical binding or overload

## Code Structure

### Main Components

1. **BackAndForthMotion Class**: Manages the back-and-forth motion state machine
2. **Motor Configuration**: Selected via compile-time constant
3. **Diagnostic Functions**: Lambda functions for comprehensive diagnostics
4. **Main Loop**: Updates motion and provides periodic diagnostics

### Key Functions

- `BackAndForthMotion::Config()`: Configure motion parameters
- `BackAndForthMotion::Start()`: Initialize and start motion
- `BackAndForthMotion::Update()`: Update motion state (call periodically)
- `BackAndForthMotion::Stop()`: Stop motion
- `diagnose_diag_pins()`: Comprehensive diagnostic pin analysis

## Customization

### Changing Motion Parameters

Edit the motion configuration in `app_main()`:

```cpp
// Travel distance in degrees each direction
motion.Config(max_vel_rpm, accel_rev_s2, travel_dist_deg, max_cycles);

// Example: 180° travel, 30 RPM, 1 rev/s² accel, infinite cycles
motion.Config(30.0f, 1.0f, 180.0f, -1);
```

### Changing Cycle Count

Set maximum cycles in motion configuration (pass as fourth parameter to `Config()`):

```cpp
int max_cycles = 10; // Run for 10 cycles then stop
motion.Config(max_vel_rpm, accel_rev_s2, travel_dist_deg, max_cycles);
```

### Adjusting Diagnostic Frequency

Modify the diagnostic update interval:

```cpp
// Change from 1000ms to 500ms
if (current_time - last_diag_time >= 500) {
    // ... diagnostic code ...
}
```

## Related Documentation

- [Motor Configuration Guide](motor_configuration.md) - Motor selection and specifications
- [Fatigue Testing](fatigue_test.md) - Point-to-point back-and-forth motion between bounds with ESP-NOW control
- [Internal Ramp Comprehensive Test](internal_ramp_comprehensive_test.md) - Comprehensive ramp control and positioning testing

## Example Output

```
I (1234) Sinusoidal: TMC5160 Back-and-Forth Motion Example for NEMA 44mm Motors
I (1235) Sinusoidal: Using internal ramp generator with positioning control
I (1236) Sinusoidal: Selected Motor: 17HS4401S with 5.18:1 gearbox
I (1237) Sinusoidal: Driver initialized successfully
I (1238) Sinusoidal: Motor enabled
I (1239) Sinusoidal: === StealthChop Diagnostic ===
I (1240) Sinusoidal: GCONF.en_pwm_mode = 1 (1=enabled, 0=disabled/SpreadCycle)
I (1241) Sinusoidal: ⚠️ StealthChop is ENABLED - checking calibration...
I (1242) Sinusoidal: ✓ StealthChop appears calibrated (pwm_scale_auto=42)
I (1243) Sinusoidal: Starting back-and-forth motion for NEMA 44mm motor:
I (1244) Sinusoidal:   Max velocity: 132608.0 steps/s (0.50 RPS output)
I (1245) Sinusoidal:   Acceleration: 663040.0 steps/s²
I (1246) Sinusoidal:   Travel distance: 265216 microsteps (1.00 output revolutions per direction)
I (1247) Sinusoidal: Diagnostics: VACTUAL=13245.2 steps/s, XACTUAL=12543
I (1248) Sinusoidal:   ✓ Motor IS moving: position changed by 12543 steps in 1000 ms
```

---

**Navigation:** [← Previous: Driver Configuration Guide](driver_configuration_guide.md) | [Index](index.md) | [Next: Fatigue Testing →](fatigue_test.md)
