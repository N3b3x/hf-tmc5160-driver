# Fatigue Testing: ESP-NOW Controlled Back-and-Forth Motion Between Bounds

## Overview

The current fatigue test implementation is the **`fatigue_test_espnow_unit`** example under `main/fatigue_test_espnow/`.
It implements a **two-device system**:

- **Fatigue Test Unit (this project)**:
  - Drives the TMC51x0 (TMC5160) stepper driver
  - Performs bounds finding (StallGuard2 or encoder-based)
  - Runs the back-and-forth fatigue motion between safe bounds
  - Communicates over ESP-NOW and UART for control and diagnostics
- **Remote GUI Controller (M5Dial)**:
  - Provides the user interface (touch + encoder)
  - Sends configuration and commands over ESP-NOW
  - Displays live status from the fatigue test unit

For detailed architecture and protocol information, see the docs under `main/fatigue_test_espnow/docs/`:

- `README.md` – High-level ESP-NOW system overview
- `PAIRING_PROTOCOL.md` – Secure pairing and HMAC authentication
- `BOUNDS_CACHING.md` – Bounds cache and de‑energize behavior

Legacy single-board UART-only examples (`fatigue_test_stallguard.cpp`, `fatigue_test_encoder.cpp`) have been **replaced** by `fatigue_test_espnow_unit`.

## Purpose

This system is ideal for:

- **Fatigue / endurance testing** of cables, flexures, and mechanical joints
- **Back-and-forth oscillatory motion** between mechanically detected bounds
- **Safe, repeatable motion** with StallGuard2 or encoder-based bounds detection
- **Remote operation** from a GUI board (M5Dial) via ESP-NOW
- **Fast re-start workflows** using a bounds cache and motor de-energize timer

## Key Features (Fatigue Test Unit)

### 1. Smart Bounds Finding with Safety Limits

- **Dual method support**:
  - StallGuard2-based sensorless bounds detection (`g_use_stallguard = true`)
  - Encoder-based bounds detection (`g_use_stallguard = false`)
- **Safety travel limits**:
  - Bounds search is constrained by a maximum travel span
  - If travel exceeds this span, or a TMC fault/reset is detected, bounds finding aborts and reports an error
- **Fault- and mode-aware**:
  - Monitors `GSTAT` (reset, UV_CP, DRV_ERR) during bounds finding
  - Aborts if the driver falls back to StealthChop while StallGuard2 is expected

### 2. Home Offset and Oscillation Range

After successful bounds finding:

- Computes the geometric center between min and max mechanical bounds
- Applies a **configurable center offset** to avoid stopping on a full-step detent:
  - `BOUNDS_FINDING_CENTER_OFFSET_DEG` in `main/fatigue_test_espnow/main.cpp`
  - Default: small move toward the negative (min) direction after homing
- Tightens the oscillation range inside the mechanical limits:
  - `OSCILLATION_EDGE_BACKOFF_DEG` defines how far inside the mechanical bounds the fatigue motion runs

### 3. Frequency-Tuned Point-to-Point Motion

The `FatigueTest::FatigueTestMotion` class manages point-to-point motion between the configured bounds:

- Uses TMC internal ramp **positioning mode** (`RampMode::POSITIONING`)
- **Trapezoidal motion profile**:
  - 1/3 accel, 1/3 constant velocity, 1/3 decel for smooth motion
- Parameters are driven from configuration (`g_settings.test_unit`):
  - Max velocity in RPM (`oscillation_vmax_rpm`)
  - Acceleration / deceleration in rev/s² (`oscillation_amax_rev_s2`)
  - Target cycle count (`cycle_amount`)
  - Local bounds around center
- Uses both `IsTargetReached()` and `IsStandstill()` before redefining zero or marking a move complete

### 4. Bounds Cache and De-Energize Timer

The `BoundsCache` namespace in `main.cpp` implements a time-based cache:

- **Bounds validity window**:
  - Default `DEFAULT_VALIDITY_MINUTES = 2` (configurable)
  - While valid, START commands can skip new bounds finding and reuse cached bounds
- **Motor de-energize timer**:
  - After bounds finding, the motor stays energized during the validity window
  - A timer de-energizes the motor after the window expires to prevent heating
- **Status reporting**:
  - `GetBoundsValidFlag_()` exposes cache state to the GUI (1 = valid, 0 = expired)

### 5. ESP-NOW Communication with GUI Board

Communication is defined in `espnow_protocol.hpp` and handled by `espnow_receiver.cpp`:

- **Message types** (examples):
  - Configuration updates (bounds, velocity, acceleration, cycles, cache time)
  - Control commands: START, STOP, PAUSE, RESUME, BOUNDS_FIND, RESET
  - Status updates: internal state, bounds valid flag, cycle counts, fault info
- **Pairing and security**:
  - Uses a shared HMAC-based pairing secret (injected via `secrets.local.yml` or `--secret`)
  - Pairing protocol is documented in `fatigue_test_espnow/docs/PAIRING_PROTOCOL.md`
- **FreeRTOS task separation**:
  - `espnow_command_task` – receives and enqueues commands
  - `motion_control_task` – applies motion settings and runs the state machine
  - `status_update_task` – periodically sends status to the GUI controller
  - `bounds_finding_task` – performs bounds finding in a dedicated task

### 6. UART Console (Optional Local Control)

The fatigue test unit also exposes a UART console (typically over USB serial) for:

- Debug logging (TMC status, bounds finding progress, motion state)
- Manual commands for development (pairing, bounds finding, starting/stopping motion)
- Diagnostics when ESP-NOW or the GUI board is unavailable

Command names and options mirror the ESP-NOW protocol where possible. See `fatigue_test_espnow/docs/README.md` for the full list.

## Hardware Requirements (Fatigue Test Unit)

- ESP32-C6 (or other supported target configured in `app_config.yml`)
- TMC5160 (or TMC51x0) stepper driver evaluation board
- Stepper motor (see [Motor Configuration Guide](motor_configuration.md))
- SPI connection between ESP32 and TMC51x0
- Optional encoder (for encoder-based bounds finding)
- Mechanical stops (for bounded mode)
- USB-serial connection (UART console)
- Power supply: 12–36 V DC with adequate current capacity

## Pin Configuration

Default pin configuration (from `esp32_tmc51x0_test_config.hpp`):

- **SPI**: MOSI=6, MISO=2, SCLK=5, CS=18
- **Control**: EN=11
- **Clock**: CLK=10 (tied to GND for internal clock)
- **Diagnostics**: DIAG0=23, DIAG1=15
- **UART**: Uses default UART_NUM_0 (USB serial port)
- **SPI Clock**: 500 kHz (from config) or 1 MHz (sinusoidal example uses 1 MHz)

## Test Rig Selection

Test rig selection is done via a `static constexpr` at the top of `main/fatigue_test_espnow/main.cpp`:

```cpp
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG =
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
```

Main rigs:

- **TEST_RIG_FATIGUE** (default): Applied Motion 5034-369 NEMA 34 motor, TMC51x0 EVAL board, reference switches, encoder
- **TEST_RIG_CORE_DRIVER**: 17HS4401S motor (geared or direct), TMC51x0 EVAL board, reference switches, encoder

The test rig automatically configures:

- Motor electrical parameters (current, sense resistors, microsteps)
- Chopper / StealthChop settings
- Encoder and reference switch wiring
- SPI clock and platform options

## How It Works

### Phase 1: Bounds Finding

1. **Position Reset**: Resets motor position to 0 for accurate tracking
2. **Maximum Bound Search** (positive direction):
   - Commands motor to +360° position
   - **360° Safety Limit**: Monitors position continuously - if rotation exceeds 360° from start, immediately stops and uses default bounds
   - Detects stall via StallGuard2 (stallguard variant) or encoder position monitoring (encoder variant)
   - If stall detected: backs off 5° and records maximum bound
   - If 360° reached without stall: marks as unbounded
3. **Minimum Bound Search** (negative direction):
   - Commands motor to -360° position
   - **360° Safety Limit**: Same safety check as maximum bound search
   - Detects stall via StallGuard2 (stallguard variant) or encoder position monitoring (encoder variant)
   - If stall detected: backs off 5° and records minimum bound
   - If -360° reached without stall: marks as unbounded
4. **Unbounded Detection**: If no stall detected in either direction at 360° limits, assumes unbounded and uses -175° to +175° default bounds
5. **StealthChop Restore**: Switches back to StealthChop for normal operation (stallguard variant only)

### Phase 2: Bounds Setup

**If Bounded**:
- Sets global bounds based on stall positions
- Moves to center position
- Sets home position to center
- Configures local bounds (default: 90% of global)

**If Unbounded**:
- No mechanical stops detected at ±360° limits
- Sets global bounds as -175° to +175° (relative to center at 0°)
- Sets local bounds within global bounds (default: 90% of global range)

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
2. **StallGuard2 variant**: Verify StallGuard2 is working (check SG_RESULT values)
3. **StallGuard2 variant**: Ensure SpreadCycle mode is active during homing
4. **Encoder variant**: Verify encoder is connected and reading position correctly
5. Check motor current is adequate for stall detection
6. Verify mechanical stops are present and functional
7. **Safety Limit Reached**: If you see "SAFETY LIMIT: Motor rotated X° (exceeds 360° limit)", the motor exceeded the safety limit - check for position tracking issues or mechanical problems

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

// Default frequency (example)
motion.SetFrequency(0.5f);

// Default dwell times (example)
motion.SetDwellTimes(2000, 2000, 0);
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
- [Internal Ramp Sinusoidal Example](internal_ramp_sinusoidal.md) - Simpler motion example
- [Internal Ramp Comprehensive Test](internal_ramp_comprehensive_test.md) - Comprehensive StallGuard2, ramp control, and positioning testing

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

---

**Navigation:** [← Previous: Internal Ramp Sinusoidal](internal_ramp_sinusoidal.md) | [Index](index.md) | [Next: Internal Ramp Comprehensive Test →](internal_ramp_comprehensive_test.md)
