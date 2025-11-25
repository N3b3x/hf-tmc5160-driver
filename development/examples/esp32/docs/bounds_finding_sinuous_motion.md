# Bounds Finding & Sinusoidal Motion Example

## Overview

The `bounds_finding_sinuous_motion.cpp` example is a comprehensive fatigue testing platform that combines sensorless bounds finding with precise sinusoidal motion control. It's designed for cable/strain relief fatigue testing and other applications requiring controlled oscillatory motion.

## Purpose

This example is ideal for:
- **Fatigue Testing**: Long-duration oscillatory motion testing
- **Cable/Strain Relief Testing**: Testing flexible components under repeated motion
- **Sensorless Homing**: Finding motor limits without physical endstops
- **Precise Motion Control**: Frequency-tuned positioning with dwell times
- **Real-Time Parameter Adjustment**: UART command interface for live tuning

## Key Features

### 1. Smart Bounds Finding

- **Sensorless Homing**: Uses StallGuard2 to detect mechanical stops
- **Unbounded Detection**: Automatically detects if motor can rotate 360° without stalling
- **Default Range**: If unbounded, defaults to 5-355 degree range
- **SpreadCycle Mode**: Automatically switches to SpreadCycle for StallGuard2, then back to StealthChop

### 2. Frequency-Tuned Motion

- **Dynamic Trajectory Calculation**: Automatically calculates VMAX and AMAX based on:
  - Target frequency (Hz)
  - Travel distance between bounds
  - Dwell times at bounds
- **Position Mode Control**: Uses TMC5160 positioning mode for precise control
- **Trapezoidal Profile**: 1/3 accel, 1/3 constant velocity, 1/3 decel for smooth motion
- **No Center Dwell in Trajectory**: Trajectory calculation explicitly excludes center dwell for frequency tuning (continuous motion through center)
- **Note**: Center dwell state exists in code for backward compatibility but is not used in trajectory calculations

### 3. UART Command Interface

Real-time parameter adjustment via serial commands:
- Set frequency
- Set dwell times
- Set bounds
- Set cycle count
- Start/stop/reset motion
- Status queries

### 4. Cycle Tracking

- Accurate cycle counting (one cycle = min → max → min)
- Target cycle count support
- Automatic stop at center when target reached

## Hardware Requirements

- ESP32 development board
- TMC5160 stepper motor driver board
- Stepper motor (see [Motor Configuration Guide](motor_configuration.md))
- SPI connection between ESP32 and TMC5160
- Mechanical stops at both ends (optional - handles unbounded)
- UART debug port (typically UART_NUM_0 for USB serial)
- Power supply: 12-36V DC (ensure adequate current capacity)

## Pin Configuration

Default pin configuration (from `esp32_tmc5160_bus_config.hpp`):

- **SPI**: MOSI=6, MISO=2, SCLK=5, CS=18
- **Control**: EN=11
- **Clock**: CLK=10 (tied to GND for internal clock)
- **Diagnostics**: DIAG0=23, DIAG1=15
- **UART**: Uses default UART_NUM_0 (USB serial port)
- **SPI Clock**: 500 kHz (from config) or 1 MHz (sinusoidal example uses 1 MHz)

## Motor Selection

Motor selection is done via a `static constexpr` variable at the top of the file:

```cpp
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;
```

See [Motor Configuration Guide](motor_configuration.md) for options and specifications.

## How It Works

### Phase 1: Bounds Finding

1. **SpreadCycle Switch**: Temporarily switches to SpreadCycle mode for StallGuard2
2. **Negative Direction Search**: Moves negative up to 1.1 revolutions, checking for stall
3. **Positive Direction Search**: Moves positive up to 1.1 revolutions, checking for stall
4. **Unbounded Detection**: If no stall detected in either direction, assumes unbounded
5. **StealthChop Restore**: Switches back to StealthChop for normal operation

### Phase 2: Bounds Setup

**If Bounded**:
- Sets global bounds based on stall positions
- Moves to center position
- Sets home position to center
- Configures local bounds (default: 90% of global)

**If Unbounded**:
- Assumes 0-360 degree range
- Sets global bounds as 0-360 degrees
- Sets local bounds as 5-355 degrees (relative to center at 180°)

### Phase 3: Motion Control

The `FatigueTestMotion` class manages the motion:

1. **Trajectory Calculation**: Calculates VMAX and AMAX based on frequency and dwell times
2. **State Machine**: Manages motion states:
   - `MOVING_TO_MAX`: Moving toward maximum bound
   - `MOVING_TO_MIN`: Moving toward minimum bound
   - `DWELL_AT_MAX`: Dwell at maximum bound
   - `DWELL_AT_MIN`: Dwell at minimum bound
3. **Cycle Counting**: Tracks cycles (min → max → min = 1 cycle)
4. **Target Reached**: Stops at center when target cycle count reached

## Trajectory Calculation

The system automatically calculates motion parameters to achieve the target frequency:

### Formula

Given:
- Target frequency: `f` Hz
- Travel distance: `D` steps (one way)
- Dwell times: `T_dwell_min`, `T_dwell_max` (ms)
- **Note**: Center dwell is not included in trajectory calculation (deprecated)

Calculate:
1. **Target Period**: `T_period = 1/f` seconds
2. **Total Dwell**: `T_dwell_total = (T_dwell_min + T_dwell_max) / 1000` seconds (center dwell excluded)
3. **Motion Time**: `T_motion = T_period - T_dwell_total` seconds
4. **Leg Time**: `T_leg = T_motion / 2` seconds (one way)
5. **VMAX**: `VMAX = 1.5 * D / T_leg` steps/s (trapezoidal profile: 1/3 accel, 1/3 const, 1/3 decel)
6. **AMAX**: `AMAX = VMAX / (T_leg / 3)` steps/s²

### Profile Shape

Uses trapezoidal profile:
- 1/3 time accelerating
- 1/3 time at constant velocity
- 1/3 time decelerating

This provides smooth motion while maximizing speed within the time constraint.

## UART Command Interface

### Command Format

Commands follow Linux-like argument structure:

```
<command> [arguments]
```

### Available Commands

#### Frequency Control

```
-f <value> or --freq <value>
```

Sets oscillation frequency in Hz (0.0-10.0).

**Example**:
```
-f 0.5          # Set frequency to 0.5 Hz
--freq 1.0      # Set frequency to 1.0 Hz
```

#### Dwell Times

```
-d <min> <max> [center]
```

Sets dwell times in milliseconds at bounds. **Note**: Center dwell argument is accepted but ignored (feature deprecated - trajectory calculation uses no center dwell).

**Example**:
```
-d 2000 2000    # Dwell 2 seconds at min and max bounds
-d 1000 1500    # Dwell 1s at min, 1.5s at max
-d 2000 2000 500  # Center dwell argument (500ms) will be ignored with warning
```

#### Bounds

```
-b <min> <max> or --bounds <min> <max>
```

Sets angle bounds from center in degrees.

**Example**:
```
-b -60 60       # Set bounds to ±60 degrees from center
--bounds -90 90 # Set bounds to ±90 degrees from center
```

#### Cycle Count

```
-c <count> or --cycles <count>
```

Sets target cycle count (0 = infinite).

**Example**:
```
-c 1000         # Run for 1000 cycles
-c 0            # Run indefinitely
```

#### Actions

```
-a start|stop|reset
```

Control motion:
- `start`: Start motion
- `stop`: Stop motion
- `reset`: Reset cycle count

**Example**:
```
-a start        # Start motion
-a stop         # Stop motion
-a reset        # Reset cycle count to 0
```

#### Status

```
-s or --status
```

Shows current status including:
- Running state
- Bounded/unbounded status
- Frequency (target and estimated)
- Local and global bounds
- Cycle count
- Dwell times

**Example**:
```
-s              # Show status
--status        # Show status
```

#### Help

```
-h or --help
```

Shows help message with all available commands.

**Example**:
```
-h              # Show help
--help          # Show help
```

### Command Examples

**Complete Setup Sequence**:
```
-b -60 60       # Set bounds to ±60 degrees
-f 0.5          # Set frequency to 0.5 Hz
-d 2000 2000    # Set dwell times to 2 seconds
-c 1000         # Set target to 1000 cycles
-a start        # Start motion
```

**Real-Time Adjustment**:
```
-a stop         # Stop motion
-f 1.0          # Increase frequency to 1.0 Hz
-a start        # Resume with new frequency
```

**Status Check**:
```
-s              # Check current status
```

## Expected Behavior

### Startup Sequence

1. Motor selection confirmation
2. Driver initialization
3. StallGuard2 configuration
4. Motor enable
5. **Bounds Finding Phase**:
   - Searching negative direction...
   - Stall detected (or unbounded detected)
   - Searching positive direction...
   - Stall detected (or unbounded detected)
6. **Bounds Setup**:
   - Bounded mode: Moving to center, setting home
   - Unbounded mode: Setting default 5-355 range
7. **Default Configuration**:
   - Local bounds set
   - Frequency set
   - Dwell times set
   - Status display
8. **UART Interface Ready**:
   - Command interface initialized
   - Tasks started
   - Ready for commands

### During Operation

- Smooth back-and-forth motion between bounds
- Motion control task updates every 10ms
- UART command processing every 50ms
- Periodic status logging every 10 seconds
- Real-time parameter adjustment via UART

### Motion Characteristics

- **Smooth Acceleration/Deceleration**: Trapezoidal profile ensures smooth motion
- **Precise Timing**: Frequency-tuned to match target frequency
- **No Center Dwell**: Continuous motion through center
- **Accurate Cycles**: Cycle counting at center crossing

## Trajectory Calculation Details

### Example Calculation

Given:
- Frequency: 0.5 Hz
- Bounds: ±60 degrees (120 degrees total)
- Dwell: 2000ms at each bound
- Motor: 17HS4401S with gearbox (~265,216 steps/rev output)

Calculate:
1. **Distance**: 120° = 88,405 steps (one way)
2. **Period**: 1/0.5 = 2.0 seconds
3. **Dwell Total**: 2.0 + 2.0 = 4.0 seconds
4. **Motion Time**: 2.0 - 4.0 = **Invalid!** (frequency too high)
5. **Adjusted**: System will warn and use minimum motion time

**Corrected Example**:
- Frequency: 0.2 Hz
- Dwell: 500ms at each bound
- **Period**: 5.0 seconds
- **Dwell Total**: 1.0 second
- **Motion Time**: 4.0 seconds
- **Leg Time**: 2.0 seconds
- **VMAX**: 1.5 × 88,405 / 2.0 = 66,304 steps/s
- **AMAX**: 66,304 / (2.0/3) = 99,456 steps/s²

## Troubleshooting

### Bounds Not Found

**Symptoms**: System reports "Unbounded Mode"

**Solutions**:
1. Check if motor can actually rotate 360° (may be truly unbounded)
2. Verify StallGuard2 is working (check SG_RESULT values)
3. Ensure SpreadCycle mode is active during homing
4. Check motor current is adequate for stall detection
5. Verify mechanical stops are present and functional

### Frequency Too High

**Symptoms**: Warning "Requested frequency is impossible with given dwell times"

**Solutions**:
1. Reduce target frequency
2. Reduce dwell times
3. Increase travel distance (wider bounds)
4. System will use maximum safe speed (frequency will be lower)

### Motion Not Smooth

**Symptoms**: Jerky motion or overshoot

**Solutions**:
1. Check VMAX/AMAX values (may be too high)
2. Verify StealthChop is calibrated
3. Check motor current settings
4. Verify power supply stability
5. Check for mechanical binding

### UART Commands Not Working

**Symptoms**: Commands not recognized or not taking effect

**Solutions**:
1. Verify UART baud rate (115200)
2. Check serial port connection
3. Verify command format (use `-h` for help)
4. Check if motion is running (some commands require stopped state)
5. Verify command handler registration

### Cycles Not Counting

**Symptoms**: Cycle count doesn't increment

**Solutions**:
1. Verify motion is actually reaching bounds
2. Check if dwell times are preventing motion
3. Verify cycle counting logic (counts at center crossing)
4. Check motion state machine transitions

## Code Structure

### Main Components

1. **FatigueTestMotion Class**: Manages motion state machine and trajectory calculation
2. **UartCommandParser Class**: Handles UART command parsing and execution
3. **Motion Control Task**: Updates motion state every 10ms
4. **UART Command Task**: Processes commands every 50ms
5. **Main Loop**: Provides periodic status logging

### Key Classes and Functions

- `FatigueTestMotion::RecalculateTrajectory()`: Calculates VMAX/AMAX from frequency
- `FatigueTestMotion::Start()`: Starts motion with calculated parameters
- `FatigueTestMotion::Update()`: Updates motion state machine
- `FatigueTestMotion::SetFrequency()`: Updates frequency and recalculates trajectory
- `FatigueTestMotion::SetDwellTimes()`: Updates dwell times and recalculates trajectory
- `UartCommandParser::ProcessCommand()`: Parses and executes UART commands

## Customization

### Changing Default Parameters

Edit defaults in `app_main()`:

```cpp
// Default local bounds
motion.SetLocalBoundsFromCenterDegrees(-amplitude, amplitude);

// Default frequency
motion.SetFrequency(Test::Motion::FATIGUE_FREQ_HZ);

// Default dwell times
motion.SetDwellTimes(dwell, dwell, 0);
```

### Adding Custom Commands

Register new commands in `app_main()`:

```cpp
parser.RegisterCommand(
    {"-x", "--custom", "Custom command description", 1, 1}, 
    HandleCustomCommand
);
```

### Modifying Trajectory Profile

Edit `RecalculateTrajectory()` to change profile shape:

```cpp
// Change from 1/3-1/3-1/3 to different ratio
calculated_vmax_ = (2.0f * distance) / leg_time_s;  // Different profile
calculated_amax_ = calculated_vmax_ / (leg_time_s / 4.0f);
```

## Related Documentation

- [Motor Configuration Guide](motor_configuration.md) - Motor selection
- [Sinusoidal Example](sinusoidal.md) - Simpler motion example
- [Diagnostics Test](diagnostics_comprehensive_test.md) - StallGuard2 testing
- [Ramp Control Test](ramp_control_comprehensive_test.md) - Positioning mode testing

## Example Output

```
I (1234) FatigueTest: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) FatigueTest: ║         TMC5160 Fatigue Test Platform: Bounds Finding & Sinuous Motion      ║
I (1236) FatigueTest: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) FatigueTest: Selected Motor: 17HS4401S with 5.18:1 gearbox
I (1238) FatigueTest: Driver initialized successfully
I (1239) FatigueTest: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1240) FatigueTest: ║                    STEP 1: Finding Global Bounds                            ║
I (1241) FatigueTest: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1242) FatigueTest: Finding minimum bound (negative direction)...
I (1243) FatigueTest: Stall detected at min bound! SG_RESULT=3
I (1244) FatigueTest: Finding maximum bound (positive direction)...
I (1245) FatigueTest: Stall detected at max bound! SG_RESULT=2
I (1246) FatigueTest: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1247) FatigueTest: ║              STEP 2: Setting Global Bounds and Home                        ║
I (1248) FatigueTest: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1249) FatigueTest: === BOUNDED MODE ===
I (1250) FatigueTest: Global bounds: min=-45.00°, max=45.00° from center
I (1251) FatigueTest: Trajectory Recalculated: Dist=88405 steps, LegTime=2.000s
I (1252) FatigueTest:   Target Freq=0.20Hz, Est Freq=0.20Hz
I (1253) FatigueTest:   VMAX=66304.0, AMAX=99456.0
I (1254) FatigueTest: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1255) FatigueTest: ║                    System Ready - Use UART Commands to Control              ║
I (1256) FatigueTest: ╚══════════════════════════════════════════════════════════════════════════════╝
```

