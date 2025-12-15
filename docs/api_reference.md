---
layout: default
title: "📖 API Reference"
description: "Complete API documentation for the TMC51x0 driver (TMC5130 & TMC51x0)"
nav_order: 6
parent: "📚 Documentation"
permalink: /docs/api_reference/
---

# API Reference

Complete reference documentation for all public methods and types in the TMC51x0 driver (TMC5130 & TMC51x0).

## Source Code

- **Main Header**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)
- **Communication Interface**: [`inc/tmc51x0_comm_interface.hpp`](../inc/tmc51x0_comm_interface.hpp)
- **Registers**: [`inc/registers/tmc51x0_registers.hpp`](../inc/registers/tmc51x0_registers.hpp)
- **Types**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)
- **Unit Conversions**: [`inc/features/tmc51x0_units.hpp`](../inc/features/tmc51x0_units.hpp)

## TMC51x0 Class

Main driver class for interfacing with the TMC51x0 stepper motor controllers (TMC5130 & TMC51x0).

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

### Class Architecture

The `TMC51x0` class uses a **subsystem-based architecture** that organizes functionality into logical groups. This design makes it easy to discover and use features:

```cpp
tmc51x0::TMC51x0<CommType> driver(comm_interface);

// Subsystems provide organized access to functionality
driver.rampControl      // Motion planning, positioning, velocity control
driver.motorControl     // Current control, chopper modes, stealthChop
driver.encoder          // Encoder integration, closed-loop control
driver.diagnostics      // Status monitoring, StallGuard2, diagnostics
driver.tuning           // Automatic parameter optimization (SGT tuning)
driver.homing           // Sensorless and switch-based homing
driver.protection       // Safety systems, short circuit protection
driver.communication    // Multi-chip communication setup
driver.printer          // Debug register printing
```

### Constructor

| Method | Signature | Description |
|--------|-----------|-------------|
| `TMC51x0()` | `TMC51x0(CommType& comm, uint8_t daisy_chain_position = 0, uint8_t uart_node_address = 0)` | Construct driver instance. For SPI: use `daisy_chain_position` (0 = first chip). For UART: use `uart_node_address` (0-254). Clock frequency (f_clk) is determined automatically during `Initialize()` from `DriverConfig::external_clk_config`. |

### Core Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetComm()` | `CommType& GetComm() noexcept` | Reference to comm interface | Get communication interface reference |
| `Initialize()` | `Result<void> Initialize(const DriverConfig& config = DriverConfig()) noexcept` | `Result<void>` indicating success or error | Initialize driver with configuration |
| `Reset()` | `Result<void> Reset() noexcept` | `Result<void>` indicating success or error | Perform software reset |
| `IsInitialized()` | `bool IsInitialized() const noexcept` | `true` if initialized | Check initialization status |

**Note**: Per datasheet procedure, devices are programmed backwards from address 254 (254, 253, 252, ...). Logical device indices (0, 1, 2, ...) map to physical addresses (254, 253, 252, ...). Use `TMC51x0MultiNode` class for managing multiple devices.

**⚠️ Chip Communication Mode Control:**
- `communication.SetOperatingMode()` and `communication.GetOperatingMode()` control SPI_MODE (pin 22) and SD_MODE (pin 21) pins
- These pins are **typically hardwired** and read at startup
- Only use these methods if SPI_MODE and SD_MODE are connected to GPIO outputs
- Configure pins in `TMC51x0PinConfig` (spi_mode_pin, sd_mode_pin) before use
- **CRITICAL**: Mode changes require a chip reset (power cycle or reset pin) to take effect
- The mode pins are read at startup, so changes won't be effective until reset

## Subsystems

The TMC51x0 class organizes functionality into intuitive subsystems. Each subsystem provides a focused set of methods for a specific aspect of motor control.

---

## RampControl Subsystem

Ramp control and motion planning subsystem for precise motor positioning and velocity control.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

**Access**: `driver.rampControl`

**Purpose**: Controls motor motion including positioning, velocity control, hold modes, and reference switch configuration. All methods support physical unit conversions (Steps, Mm, Deg, RPM).

**Key Features**:
- **Ramp Modes**: Positioning, velocity (positive/negative), and hold modes
- **Unit-Aware API**: Work with physical units (mm, degrees, RPM) instead of raw steps
- **Advanced Motion Profiles**: Multi-phase acceleration (A1, AMAX, D1) for smooth motion
- **Reference Switches**: Configurable endstops with latching and stop-on-switch modes
- **Position Control**: Set target positions, read current position, check target reached
- **Velocity Control**: Set maximum speeds, read current velocity, check velocity reached

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `SetRampMode()` | `Result<void> SetRampMode(RampMode mode) noexcept` | `Result<void>` indicating success or error | Set ramp mode (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD) |
| `SetTargetPosition()` | `Result<void> SetTargetPosition(float value, Unit unit) noexcept` | `Result<void>` indicating success or error | Set target position for positioning mode (unit-aware, unit parameter required) |
| `GetCurrentPosition()` | `Result<float> GetCurrentPosition(Unit unit) noexcept` | `Result<float>` containing the value or error | Get current motor position (unit-aware, unit parameter required) |
| `GetTargetPosition()` | `Result<float> GetTargetPosition(Unit unit) noexcept` | `Result<float>` containing the value or error | Get target position (unit-aware, unit parameter required) |
| `SetCurrentPosition()` | `Result<void> SetCurrentPosition(float value, Unit unit, bool update_encoder = false) noexcept` | `Result<void>` indicating success or error | Set current position (unit-aware, unit parameter required, optionally update encoder) |
| `SetMaxSpeed()` | `Result<void> SetMaxSpeed(float value, Unit unit) noexcept` | `Result<void>` indicating success or error | Set maximum velocity (unit-aware, unit parameter required) |
| `SetAcceleration()` | `Result<void> SetAcceleration(float value, Unit unit) noexcept` | `Result<void>` indicating success or error | Set acceleration/deceleration (unit-aware, unit parameter required) |
| `SetAccelerations()` | `Result<void> SetAccelerations(float accel_val, float decel_val, Unit unit) noexcept` | `Result<void>` indicating success or error | Set acceleration and deceleration separately (unit-aware, unit parameter required) |
| `SetDeceleration()` | `Result<void> SetDeceleration(float value, Unit unit) noexcept` | `Result<void>` indicating success or error | Set deceleration only (DMAX register, unit-aware, unit parameter required) |
| `SetRampSpeeds()` | `Result<void> SetRampSpeeds(float start_speed, float stop_speed, float transition_speed, Unit unit) noexcept` | `Result<void>` indicating success or error | Set ramp start, stop, and transition speeds (unit-aware, unit parameter required) |
| `GetCurrentSpeed()` | `Result<float> GetCurrentSpeed(Unit unit) noexcept` | `Result<float>` containing the value or error | Get current motor velocity (unit-aware, unit parameter required) |
| `IsTargetReached()` | `Result<bool> IsTargetReached() noexcept` | `Result<bool>` containing true if target position reached, false otherwise | Check if target position reached |
| `IsTargetVelocityReached()` | `Result<bool> IsTargetVelocityReached() noexcept` | `Result<bool>` containing true if target velocity reached, false otherwise | Check if target velocity reached |
| `Stop()` | `Result<void> Stop() noexcept` | `Result<void>` indicating success or error | Stop motor immediately |
| `ConfigureReferenceSwitch()` | `Result<void> ConfigureReferenceSwitch(const ReferenceSwitchConfig& config) noexcept` | `Result<void>` indicating success or error | Configure reference switches/endstops (full configuration) |
| `GetReferenceSwitchConfig()` | `Result<ReferenceSwitchConfig> GetReferenceSwitchConfig() noexcept` | `Result<ReferenceSwitchConfig>` containing the value or error | Read current reference switch configuration |
| `SetLeftSwitchActiveLevel()` | `Result<void> SetLeftSwitchActiveLevel(ReferenceSwitchActiveLevel) noexcept` | `Result<void>` indicating success or error | Set left switch active level (real-time update) |
| `SetRightSwitchActiveLevel()` | `Result<void> SetRightSwitchActiveLevel(ReferenceSwitchActiveLevel) noexcept` | `Result<void>` indicating success or error | Set right switch active level (real-time update) |
| `SetLeftSwitchStopEnable()` | `Result<void> SetLeftSwitchStopEnable(bool enable) noexcept` | `Result<void>` indicating success or error | Enable/disable motor stop on left switch (real-time) |
| `SetRightSwitchStopEnable()` | `Result<void> SetRightSwitchStopEnable(bool enable) noexcept` | `Result<void>` indicating success or error | Enable/disable motor stop on right switch (real-time) |
| `SetLeftSwitchLatchMode()` | `Result<void> SetLeftSwitchLatchMode(ReferenceLatchMode) noexcept` | `Result<void>` indicating success or error | Set left switch latching mode (real-time update) |
| `SetRightSwitchLatchMode()` | `Result<void> SetRightSwitchLatchMode(ReferenceLatchMode) noexcept` | `Result<void>` indicating success or error | Set right switch latching mode (real-time update) |
| `SetStopMode()` | `Result<void> SetStopMode(ReferenceStopMode) noexcept` | `Result<void>` indicating success or error | Set stop mode (hard/soft) (real-time update) |
| `GetLatchedPosition()` | `Result<float> GetLatchedPosition(Unit unit) noexcept` | `Result<float>` containing the value or error | Get position latched on switch event (unit-aware, unit parameter required) |
| `SetXCompare()` | `Result<void> SetXCompare(float position, Unit unit) noexcept` | `Result<void>` indicating success or error | Set position comparison register (X_COMPARE, unit-aware, unit parameter required) |
| `GetXCompare()` | `Result<float> GetXCompare(Unit unit) const noexcept` | `Result<float>` containing the value or error | Get X_COMPARE register value from local storage (unit-aware, unit parameter required) |
| `SetPowerDownDelay()` | `Result<void> SetPowerDownDelay(uint8_t tpowerdown) noexcept` | `Result<void>` indicating success or error | Set power down delay (raw register value 0-255) |
| `SetPowerDownDelayMs()` | `Result<void> SetPowerDownDelayMs(float delay_ms) noexcept` | `Result<void>` indicating success or error | Set power down delay in milliseconds (automatically converted) |
| `SetZeroWaitTime()` | `Result<void> SetZeroWaitTime(uint16_t tzerowait) noexcept` | `Result<void>` indicating success or error | Set zero wait time after ramping down (raw register value 0-65535) |
| `SetZeroWaitTimeMs()` | `Result<void> SetZeroWaitTimeMs(float delay_ms) noexcept` | `Result<void>` indicating success or error | Set zero wait time in milliseconds (automatically converted) |
| `SetFirstAcceleration()` | `Result<void> SetFirstAcceleration(float a1, Unit unit) noexcept` | `Result<void>` indicating success or error | Set first acceleration phase A1 (unit-aware, unit parameter required, 0.0f = use AMAX) |
| `SetFinalDeceleration()` | `Result<void> SetFinalDeceleration(float d1, Unit unit) noexcept` | `Result<void>` indicating success or error | Set final deceleration phase D1 (unit-aware, unit parameter required, must not be 0 in positioning mode) |
| `ConfigureRamp()` | `Result<void> ConfigureRamp(const RampConfig& config) noexcept` | `Result<void>` indicating success or error | Configure ramp generator from RampConfig structure |

## MotorControl Subsystem

Motor control and configuration subsystem for current control, chopper modes, and stealthChop operation.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

**Access**: `driver.motorControl`

**Purpose**: Manages motor current settings, chopper configuration, stealthChop/sreadCycle modes, and motor enable/disable. Automatically calculates IRUN, IHOLD, and GLOBAL_SCALER from motor physical specifications.

**Key Features**:
- **Automatic Current Calculation**: Calculates IRUN, IHOLD, GLOBAL_SCALER from motor specs
- **Chopper Modes**: spreadCycle (high torque) and stealthChop (silent operation)
- **Mode Switching**: Automatic switching between modes based on velocity thresholds
- **Freewheeling**: Automatic freewheeling when motor is stopped
- **Microstep Configuration**: Configurable microstep resolution (1-256)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `Enable()` | `Result<void> Enable() noexcept` | `Result<void>` indicating success or error | Enable motor driver |
| `Disable()` | `Result<void> Disable() noexcept` | `Result<void>` indicating success or error | Disable motor driver |
| `SetCurrent()` | `Result<void> SetCurrent(uint8_t irun, uint8_t ihold) noexcept` | `Result<void>` indicating success or error | Set run current (0-31) and hold current (0-31) |
| `ConfigureChopper()` | `Result<void> ConfigureChopper(const ChopperConfig& config) noexcept` | `Result<void>` indicating success or error | Configure chopper settings (toff, hstrt, hend, tbl, mres, etc.) |
| `SetMicrostepResolution()` | `Result<void> SetMicrostepResolution(MicrostepResolution mres, const MicrostepChangeOptions& opts = MicrostepChangeOptions{}) noexcept` | `Result<void>` indicating success or error | Change microstep resolution (MRES). By default preserves physical meaning by rescaling position + ramp profile; requires standstill unless disabled. |
| `ConfigureStealthChop()` | `Result<void> ConfigureStealthChop(const StealthChopConfig& config) noexcept` | `Result<void>` indicating success or error | Configure stealthChop PWM mode |
| `SetModeChangeSpeeds()` | `Result<void> SetModeChangeSpeeds(float pwm_thrs, float cool_thrs, float high_thrs, Unit unit) noexcept` | `Result<void>` indicating success or error | Set velocity thresholds for mode switching (unit-aware, unit parameter required) |
| `SetStealthChopVelocityThreshold()` | `Result<void> SetStealthChopVelocityThreshold(float value, Unit unit) noexcept` | `Result<void>` indicating success or error | Set StealthChop velocity threshold (TPWMTHRS, unit-aware, unit parameter required) |
| `SetCoolStepThreshold()` | `Result<void> SetCoolStepThreshold(float value, Unit unit) noexcept` | `Result<void>` indicating success or error | Set CoolStep velocity threshold (TCOOLTHRS, unit-aware, unit parameter required) |
| `SetHighSpeedThreshold()` | `Result<void> SetHighSpeedThreshold(float value, Unit unit) noexcept` | `Result<void>` indicating success or error | Set High-Speed velocity threshold (THIGH, unit-aware, unit parameter required) |
| `SetGlobalScaler()` | `Result<void> SetGlobalScaler(uint16_t scaler) noexcept` | `Result<void>` indicating success or error | Set global current scaler (32-256) |
| `SetIholdDelayMs()` | `Result<void> SetIholdDelayMs(float total_delay_ms) noexcept` | `Result<void>` indicating success or error | Set motor power down delay (IHOLDDELAY) in milliseconds (automatically calculated) |
| `ConfigureCoolStep()` | `Result<void> ConfigureCoolStep(const CoolStepConfig& config) noexcept` | `Result<void>` indicating success or error | Configure CoolStep current reduction |
| `ConfigureDcStep()` | `Result<void> ConfigureDcStep(const DcStepConfig& config) noexcept` | `Result<void>` indicating success or error | Configure dcStep automatic commutation |
| `SetMicrostepLookupTable()` | `Result<void> SetMicrostepLookupTable(uint8_t index, uint32_t value) noexcept` | `Result<void>` indicating success or error | Set microstep lookup table entry (0-7) |
| `SetMicrostepLookupTableSegmentation()` | `Result<void> SetMicrostepLookupTableSegmentation(uint8_t width_sel_0, uint8_t width_sel_1, uint8_t width_sel_2, uint8_t width_sel_3, uint8_t lut_seg_start1, uint8_t lut_seg_start2, uint8_t lut_seg_start3) noexcept` | `Result<void>` indicating success or error | Set microstep lookup table segmentation |
| `SetMicrostepLookupTableStart()` | `Result<void> SetMicrostepLookupTableStart(uint16_t start_current) noexcept` | `Result<void>` indicating success or error | Set microstep lookup table start current |
| `SetupMotorFromSpec()` | `Result<void> SetupMotorFromSpec(const MotorSpec& motor_spec, const MechanicalSystem* mechanical_system = nullptr) noexcept` | `Result<void>` indicating success or error | Setup motor from high-level specifications |
| `ConfigureGlobalConfig()` | `Result<void> ConfigureGlobalConfig(const GlobalConfig& config) noexcept` | `Result<void>` indicating success or error | Configure global configuration (GCONF register) |

## Encoder Subsystem

Encoder integration and closed-loop control subsystem for position verification and step loss detection.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

**Access**: `driver.encoder`

**Purpose**: Provides encoder integration for closed-loop control, position verification, and step loss detection. Supports various encoder types (AB, ABZ, SPI) with configurable resolution and deviation detection.

**Key Features**:
- **Closed-Loop Control**: Encoder feedback for position verification
- **Deviation Detection**: Monitor and detect position deviation (step loss)
- **Automatic Compensation**: Optional automatic position correction
- **Flexible Configuration**: Supports various encoder types and resolutions
- **N-Channel Support**: Configurable N-channel (index) handling

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `Configure()` | `Result<void> Configure(const EncoderConfig& config) noexcept` | `Result<void>` indicating success or error | Configure encoder settings (polarity, filtering, etc.) |
| `GetEncoderConfig()` | `Result<EncoderConfig> GetEncoderConfig() noexcept` | `Result<EncoderConfig>` containing the value or error | Read current encoder configuration |
| `SetNChannelActiveLevel()` | `Result<void> SetNChannelActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept` | `Result<void>` indicating success or error | Set N channel active level (real-time) |
| `SetNChannelSensitivity()` | `Result<void> SetNChannelSensitivity(EncoderNSensitivity sensitivity) noexcept` | `Result<void>` indicating success or error | Set N channel sensitivity (real-time) |
| `SetClearMode()` | `Result<void> SetClearMode(EncoderClearMode clear_mode) noexcept` | `Result<void>` indicating success or error | Set encoder clear mode (real-time) |
| `SetPrescalerMode()` | `Result<void> SetPrescalerMode(EncoderPrescalerMode prescaler_mode) noexcept` | `Result<void>` indicating success or error | Set encoder prescaler mode (real-time) |
| `GetPosition()` | `Result<int32_t> GetPosition() noexcept` | `Result<int32_t>` containing the value or error | Get encoder position in steps |
| `SetResolution()` | `Result<void> SetResolution(int32_t motor_steps, int32_t enc_resolution, bool inverted = false) noexcept` | `Result<void>` indicating success or error | Set encoder resolution (motor steps per encoder resolution) |
| `SetAllowedDeviation()` | `Result<void> SetAllowedDeviation(int32_t steps) noexcept` | `Result<void>` indicating success or error | Set allowed encoder deviation threshold in steps |
| `IsDeviationDetected()` | `Result<bool> IsDeviationDetected() noexcept` | `Result<bool>` containing true if deviation detected, false otherwise | Check if encoder deviation detected |
| `ClearDeviationFlag()` | `Result<void> ClearDeviationFlag() noexcept` | `Result<void>` indicating success or error | Clear encoder deviation flag |
| `GetLatchedPosition()` | `Result<int32_t> GetLatchedPosition() noexcept` | `Result<int32_t>` containing the value or error | Get encoder position latched on N event |

## Diagnostics Subsystem

Driver status monitoring and diagnostics subsystem.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetStatus()` | `DriverStatus GetStatus() noexcept` | DriverStatus enum | Get driver status (OK, CP_UV, S2VSA, etc.) |
| `GetGlobalStatus()` | `Result<bool> GetGlobalStatus(bool& drv_err, bool& uv_cp) noexcept` | `Result<bool>` containing true if read successfully, false otherwise | Get global status flags (GSTAT) |
| `GetStallGuard()` | `Result<uint16_t> GetStallGuard() noexcept` | `Result<uint16_t>` containing the value or error | Get StallGuard2 value (0-1023) |
| `GetStallGuardResult()` | `Result<uint16_t> GetStallGuardResult() noexcept` | `Result<uint16_t>` containing the value or error | Get StallGuard2 result from DRV_STATUS register |
| `ConfigureStallGuard()` | `Result<void> ConfigureStallGuard(const StallGuardConfig& config) noexcept` | `Result<void>` indicating success or error | Configure StallGuard2 settings |
| `EnableStopOnStall()` | `Result<void> EnableStopOnStall(bool enable) noexcept` | `Result<void>` indicating success or error | Enable/disable stop on stall |
| `IsStopOnStallEnabled()` | `Result<bool> IsStopOnStallEnabled() noexcept` | `Result<bool>` containing true if stop on stall is enabled, false otherwise | Check if stop on stall is enabled |
| `SetSoftStop()` | `Result<void> SetSoftStop(bool enable) noexcept` | `Result<void>` indicating success or error | Enable/disable soft stop |
| `IsSoftStopEnabled()` | `Result<bool> IsSoftStopEnabled() noexcept` | `Result<bool>` containing true if soft stop is enabled, false otherwise | Check if soft stop is enabled |
| `ClearStallFlag()` | `Result<void> ClearStallFlag() noexcept` | `Result<void>` indicating success or error | Clear stall event flag |
| `IsStallDetected()` | `Result<bool> IsStallDetected() noexcept` | `Result<bool>` containing true if stall event detected, false otherwise | Check if stall was detected |
| `GetDriverStatusRegister()` | `Result<uint32_t> GetDriverStatusRegister() noexcept` | `Result<uint32_t>` containing the value or error | Read DRV_STATUS register |
| `IsOpenLoadA()` | `Result<bool> IsOpenLoadA() noexcept` | `Result<bool>` containing true if open load detected on phase A, false otherwise | Check for open load on phase A (requires SpreadCycle mode and motion) |
| `IsOpenLoadB()` | `Result<bool> IsOpenLoadB() noexcept` | `Result<bool>` containing true if open load detected on phase B, false otherwise | Check for open load on phase B (requires SpreadCycle mode and motion) |
| `CheckOpenLoad()` | `Result<bool> CheckOpenLoad(bool& phase_a, bool& phase_b) noexcept` | `Result<bool>` containing true if read successfully, false otherwise | Check both phases for open load simultaneously |
| `GetRampStatusRegister()` | `Result<uint32_t> GetRampStatusRegister() noexcept` | `Result<uint32_t>` containing the value or error | Read RAMP_STAT register |
| `ClearRampStatus()` | `Result<void> ClearRampStatus(uint32_t bits_to_clear) noexcept` | `Result<void>` indicating success or error | Clear specific bits in RAMP_STAT register |
| `GetLostSteps()` | `Result<uint32_t> GetLostSteps() noexcept` | `Result<uint32_t>` containing the value or error | Get lost steps counter (dcStep mode only) |
| `GetChipVersion()` | `uint8_t GetChipVersion() const noexcept` | Chip version (0x11 = TMC5130, 0x30 = TMC51x0) | Get detected chip version |

## Tuning Subsystem ⭐

Automatic parameter tuning for optimal driver performance.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

**Access**: `driver.tuning`

**Purpose**: Provides intelligent automatic tuning of driver parameters, particularly StallGuard2 threshold (SGT) optimization. The tuning algorithm prioritizes target velocity and provides comprehensive velocity range analysis.

**Key Features**:
- **Automatic SGT Tuning**: Intelligent StallGuard2 threshold optimization
- **Velocity Range Analysis**: Tests min/max velocities to determine usable SGT range
- **Target Velocity Priority**: Optimizes SGT primarily for your most important operating speed
- **Comprehensive Results**: Returns detailed tuning results including actual achievable velocities
- **Fallback Detection**: If requested velocities don't work, finds what velocities DO work

| Method | Signature | Return | Description |
|--------|-----------|--------|-------------|
| `TuneStallGuard()` | `Result<void> TuneStallGuard(float target_velocity, StallGuardTuningResult& result, int8_t min_sgt = -10, int8_t max_sgt = 63, float acceleration = 0.06F, float min_velocity = 0.0F, float max_velocity = 0.0F, Unit velocity_unit = Unit::RevPerSec, Unit acceleration_unit = Unit::RevPerSec) noexcept` | `Result<void>` indicating success or error | Automatically tune StallGuard threshold (SGT) with comprehensive velocity range analysis. Separate unit parameters for velocity and acceleration (RPM not valid for acceleration) |
| `TuneStallGuard()` (legacy) | `Result<void> TuneStallGuard(float target_velocity, int8_t& final_sgt, int8_t min_sgt = -10, int8_t max_sgt = 63, float acceleration = 0.06F, float min_velocity = 0.0F, float max_velocity = 0.0F, Unit velocity_unit = Unit::RevPerSec, Unit acceleration_unit = Unit::RevPerSec) noexcept` | `Result<void>` indicating success or error | Legacy overload - use StallGuardTuningResult version for comprehensive results. Separate unit parameters for velocity and acceleration (RPM not valid for acceleration) |
| `AutoTuneStallGuard()` | `Result<void> AutoTuneStallGuard(float target_velocity, StallGuardTuningResult& result, int8_t min_sgt = 0, int8_t max_sgt = 63, float acceleration = 0.06F, float min_velocity = 0.0F, float max_velocity = 0.0F, Unit velocity_unit = Unit::RevPerSec, Unit acceleration_unit = Unit::RevPerSec, uint16_t safe_current_margin_mA = 0) noexcept` | `Result<void>` indicating success or error | Comprehensive automatic StallGuard tuning with safe current margin handling (recommended). Separate unit parameters for velocity and acceleration (RPM not valid for acceleration) |

**Usage Example**:
```cpp
tmc51x0::StallGuardTuningResult result;
// Using AutoTuneStallGuard (recommended) with safe current margin
// Note: Separate unit parameters for velocity and acceleration
auto result_tune = driver.tuning.AutoTuneStallGuard(
    0.6f, result,                          // Target velocity: 0.6 rev/s (~36 RPM)
    0, 63,                                 // SGT search range
    0.06f,                                 // Acceleration: 0.06 rev/s²
    0.18f, 0.9f,                           // Velocity range: 0.18-0.9 rev/s (30%-150% of target)
    tmc51x0::Unit::RevPerSec,              // Velocity unit (default, can be omitted)
    tmc51x0::Unit::RevPerSec,              // Acceleration unit (default, can be omitted)
    300                                    // Safe current margin: 300mA
);

// Example with RPM for velocity, RevPerSec for acceleration (RPM not valid for acceleration)
auto result_tune2 = driver.tuning.AutoTuneStallGuard(
    36.0f, result,                         // Target velocity: 36 RPM
    0, 63,                                 // SGT search range
    0.06f,                                 // Acceleration: 0.06 rev/s² (must use RevPerSec)
    10.8f, 54.0f,                          // Velocity range: 10.8-54.0 RPM (30%-150% of target)
    tmc51x0::Unit::RPM,                    // Velocity unit: RPM
    tmc51x0::Unit::RevPerSec,              // Acceleration unit: RevPerSec (RPM not valid)
    300                                    // Safe current margin: 300mA
);

// Or using TuneStallGuard (simpler, no current margin)
auto result_tune3 = driver.tuning.TuneStallGuard(
    0.6f, result,                          // Target velocity: 0.6 rev/s
    0, 63,                                 // SGT search range
    0.06f,                                 // Acceleration: 0.06 rev/s²
    0.18f, 0.9f,                           // Velocity range to test
    tmc51x0::Unit::RevPerSec,              // Velocity unit (default)
    tmc51x0::Unit::RevPerSec               // Acceleration unit (default)
    // Unit::RevPerSec is default, can be omitted
);

if (result_tune) {
    // Use optimal SGT
    tmc51x0::StallGuardConfig sg_config;
    sg_config.threshold = result.optimal_sgt;
    driver.diagnostics.ConfigureStallGuard(sg_config);
    
    // Check velocity compatibility
    if (!result.min_velocity_success) {
        // Use actual_min_velocity instead
    }
}
```

## Homing Subsystem

Homing methods with automatic settings caching for endstop-free and switch-based homing.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

**Access**: `driver.homing`

**Purpose**: Provides both sensorless (StallGuard2-based) and switch-based homing operations. Automatically caches and restores driver settings modified during homing, ensuring your configuration is preserved.

**Key Features**:
- **Sensorless Homing**: Endstop-free homing using StallGuard2 stall detection
- **Switch-Based Homing**: Homing using reference switches/endstops
- **Automatic Settings Caching**: Preserves and restores driver settings during homing
- **Configurable Search Speed**: Adjustable homing speed and switch approach speed
- **Timeout Protection**: Configurable timeout to prevent infinite searches

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `PerformSensorlessHoming()` | `Result<void> PerformSensorlessHoming(bool direction, float search_speed, int32_t& final_position, uint32_t timeout_ms = 10000) noexcept` | `Result<void>` indicating success or error | Perform sensorless homing using StallGuard2 (uses existing SGT threshold from motor config) |
| `PerformSwitchHoming()` | `Result<void> PerformSwitchHoming(bool direction, float search_speed, float switch_speed, int32_t& final_position, bool use_left_switch, uint32_t timeout_ms = 10000) noexcept` | `Result<void>` indicating success or error | Perform homing using a reference switch |

### Printer Subsystem

Register printing methods for debugging.

| Method | Signature | Description |
|--------|-----------|-------------|
| `PrintGconf()` | `void PrintGconf() noexcept` | Print GCONF register |
| `PrintGstat()` | `void PrintGstat() noexcept` | Print GSTAT register (read and clear) |
| `PrintRampStat()` | `void PrintRampStat() noexcept` | Print RAMP_STAT register |
| `PrintDrvStatus()` | `void PrintDrvStatus() noexcept` | Print DRV_STATUS register |
| `PrintChopconf()` | `void PrintChopconf() noexcept` | Print CHOPCONF register |
| `PrintPwmconf()` | `void PrintPwmconf() noexcept` | Print PWMCONF register |
| `PrintPwmScale()` | `void PrintPwmScale() noexcept` | Print PWM_SCALE register |
| `PrintSwMode()` | `void PrintSwMode() noexcept` | Print SW_MODE register |
| `PrintIoin()` | `void PrintIoin() noexcept` | Print IOIN register |
| `PrintAll()` | `void PrintAll() noexcept` | Print all common registers |
| `GetTimeBetweenMicrosteps()` | `Result<uint32_t> GetTimeBetweenMicrosteps() noexcept` | `Result<uint32_t>` containing the value or error | Get actual time between microsteps in clock cycles (TSTEP register) |
| `GetMicrostepCounter()` | `Result<uint16_t> GetMicrostepCounter() noexcept` | `Result<uint16_t>` containing the value or error | Get actual position in microstep table (0-1023, MSCNT register) |
| `GetMicrostepCurrent()` | `Result<int16_t> GetMicrostepCurrent(int16_t& phase_b) noexcept` | `Result<int16_t>` containing the value or error | Get actual microstep current for both phases (MSCURACT register) |
| `GetPwmScale()` | `Result<uint8_t> GetPwmScale(int16_t& pwm_scale_auto) noexcept` | `Result<uint8_t>` containing the value or error | Get stealthChop PWM scale results (PWM_SCALE register) |
| `GetPwmAuto()` | `Result<uint8_t> GetPwmAuto(uint8_t& pwm_grad_auto) noexcept` | `Result<uint8_t>` containing the value or error | Get automatically determined PWM values (PWM_AUTO register) |
| `ReadInputStatus()` | `Result<InputStatus> ReadInputStatus() noexcept` | `Result<InputStatus>` containing the value or error | Read GPIO input pins (parsed structure) |
| `ReadIcVersion()` | `Result<uint8_t> ReadIcVersion() noexcept` | `Result<uint8_t>` containing the value or error | Read IC version from IOIN register |
| `ReadGpioPins()` | `Result<uint32_t> ReadGpioPins() noexcept` | `Result<uint32_t>` containing the value or error | Read GPIO input pin states (raw register value) |
| `ReadFactoryConfig()` | `Result<uint8_t> ReadFactoryConfig() noexcept` | `Result<uint8_t>` containing the value or error | Read factory configuration/clock trim (FACTORY_CONF register) |
| `SetSdoCfg0Polarity()` | `Result<void> SetSdoCfg0Polarity(bool polarity) noexcept` | `Result<void>` indicating success or error | Set SDO_CFG0 pin polarity (UART/Single Wire mode) |
| `ReadOtpConfig()` | `Result<uint8_t> ReadOtpConfig(bool& otp_s2_level, bool& otp_bbm, bool& otp_tbl) noexcept` | `Result<uint8_t>` containing the value or error | Read OTP configuration memory (OTP_READ register) |
| `GetUartTransmissionCount()` | `Result<uint8_t> GetUartTransmissionCount() noexcept` | `Result<uint8_t>` containing the value or error | Get UART transmission counter (IFCNT register) |
| `ReadOffsetCalibration()` | `Result<uint8_t> ReadOffsetCalibration(uint8_t& phase_b) noexcept` | `Result<uint8_t>` containing the value or error | Read offset calibration results (OFFSET_READ register) |
| `VerifySetup()` | `Result<void> VerifySetup() noexcept` | `Result<void>` indicating success or error | Run comprehensive startup verification |
| `SetTcoolthrs()` | `Result<void> SetTcoolthrs(float threshold, Unit unit) noexcept` | `Result<void>` indicating success or error | Set TCOOLTHRS register directly (unit-aware, unit parameter required) |
| `GetTcoolthrs()` | `Result<float> GetTcoolthrs(Unit unit) const noexcept` | `Result<float>` containing the value or error | Get TCOOLTHRS register value from local storage (unit-aware, unit parameter required) |

### Open Load Diagnostics

The TMC51x0 can detect open load conditions (interrupted cables, loose connectors) by checking if it can reach the desired motor coil current. This is useful for system debugging and detecting wiring issues.

**Requirements for Reliable Detection:**

1. **SpreadCycle Mode**: Open load detection only works reliably in SpreadCycle mode. StealthChop mode must be disabled (`en_pwm_mode = 0` in GCONF).

2. **Motor Motion**: The motor must be moving with a minimum of 4× the selected microstep resolution in a single direction. Detection cannot occur in standstill (coils may have zero current).

3. **Velocity**: Use low or nominal motor velocity for best results. High velocity settings may cause false triggers.

4. **Informative Only**: Open load flags (`ola`, `olb`) are informative and do not cause any driver action. They may also be triggered by:
   - Undervoltage conditions
   - High motor velocity settings
   - Short circuit conditions
   - Overtemperature conditions

**Usage Example:**

```cpp
// Ensure SpreadCycle mode is enabled (StealthChop disabled)
tmc51x0::GlobalConfig gconf{};
driver.motorControl.GetGlobalConfig(gconf);
if (gconf.en_pwm_mode) {
  gconf.en_pwm_mode = false;  // Disable StealthChop
  driver.motorControl.ConfigureGlobalConfig(gconf);
}

// Move motor at low/nominal velocity (minimum 4× microstep resolution)
driver.rampControl.SetMaxSpeed(0.02f);  // 0.02 rev/s (~1.2 RPM) - Unit::RevPerSec is default
driver.rampControl.SetTargetPosition(1024);  // At least 4× microstep resolution (256)

// Wait for motion to start
while (true) {
  auto reached = driver.rampControl.IsTargetReached();
  if (reached && reached.Value()) {
    break; // Target reached
  }
  if (!reached) {
    // Handle error
    break;
  }
  // Check for open load during motion
  auto open_a = driver.diagnostics.IsOpenLoadA();
  auto open_b = driver.diagnostics.IsOpenLoadB();
  bool phase_a_open = open_a && open_a.Value();
  bool phase_b_open = open_b && open_b.Value();
  
  if (phase_a_open || phase_b_open) {
    ESP_LOGW(TAG, "Open load detected: Phase A=%d, Phase B=%d", 
             phase_a_open, phase_b_open);
    // Check wiring, connectors, and motor connections
  }
  
  vTaskDelay(pdMS_TO_TICKS(10));
}

// Or check both phases at once
bool phase_a, phase_b;
if (driver.diagnostics.CheckOpenLoad(phase_a, phase_b)) {
  if (phase_a) ESP_LOGW(TAG, "Phase A open load detected");
  if (phase_b) ESP_LOGW(TAG, "Phase B open load detected");
}
```

**See Also**: Datasheet section 11.3: Open Load Diagnostics

## Communication Subsystem

Communication and multi-chip configuration subsystem for SPI daisy chaining and UART multi-node addressing.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

**Access**: `driver.communication`

**Purpose**: Manages multi-chip communication setup including SPI daisy chain configuration and UART node addressing. Essential for controlling multiple TMC51x0 drivers on a single bus.

**Key Features**:
- **SPI Daisy Chaining**: Configure position and chain length for SPI multi-chip setups
- **UART Multi-Node**: Configure node addresses for UART multi-device setups
- **Automatic Chain Detection**: Helper methods for detecting chain length
- **Mode Control**: Control SPI_MODE and SD_MODE pins (if GPIO-controlled)
- **Clock Configuration**: External clock frequency configuration

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `SetClkFreq()` | `Result<void> SetClkFreq(uint32_t frequency_hz) noexcept` | `Result<void>` indicating success or error | Set clock frequency on CLK pin (0 = internal clock, >0 = external clock frequency) - low-level method |
| `SetClkFreq()` | `Result<void> SetClkFreq(const ExternalClockConfig& config) noexcept` | `Result<void>` indicating success or error | Set clock frequency from ExternalClockConfig (high-level method, updates driver configuration) |
| `ConfigureUartNodeAddress()` | `Result<void> ConfigureUartNodeAddress(uint8_t node_address, uint8_t send_delay = 0) noexcept` | `Result<void>` indicating success or error | Configure UART node address and send delay (writes SLAVECONF register, also updates software state) |
| `SetUartNodeAddress()` | `void SetUartNodeAddress(uint8_t address) noexcept` | void | Set UART node address (0-254) - software only, does not write to hardware |
| `GetUartNodeAddress()` | `uint8_t GetUartNodeAddress() const noexcept` | Address (0-254) | Get current UART node address |
| `SetOperatingMode()` | `Result<void> SetOperatingMode(ChipCommMode mode) noexcept` | `Result<void>` indicating success or error | Set chip operating mode via SPI_MODE and SD_MODE pins (controls both communication interface and motion control method) |
| `GetOperatingMode()` | `Result<ChipCommMode> GetOperatingMode() const noexcept` | `Result<ChipCommMode>` containing the value or error | Get current chip operating mode from SPI_MODE and SD_MODE pins |
| `SetDaisyChainPosition()` | `void SetDaisyChainPosition(uint8_t position) noexcept` | void | Set daisy-chain position for SPI (0 = first chip, 1 = second, etc.) |
| `GetDaisyChainPosition()` | `uint8_t GetDaisyChainPosition() const noexcept` | Position (0-255) | Get current daisy-chain position for SPI |

## UartConfig Subsystem

UART configuration subsystem for multi-node addressing.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `ConfigureUartNodeAddress()` | `Result<void> ConfigureUartNodeAddress(uint8_t node_address, uint8_t send_delay) noexcept` | `Result<void>` indicating success or error | Configure SLAVECONF register with UART node address (0-254) and send delay (for sequential programming). Updates the driver's `uart_node_address_`. Per datasheet, devices are typically programmed backwards from 254. |

## Protection Subsystem

Protection and safety features subsystem.

**Location**: [`inc/tmc51x0.hpp`](../inc/tmc51x0.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `ConfigureShortProtection()` | `Result<void> ConfigureShortProtection(const PowerStageParameters& config) noexcept` | `Result<void>` indicating success or error | Configure short circuit protection using user-friendly voltage thresholds |
| `SetShortProtectionLevels()` | `Result<void> SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level, uint8_t shortfilter, uint8_t shortdelay) noexcept` | `Result<void>` indicating success or error | Set short protection levels directly (low-level API, register values) |

## Communication Interface Methods

Methods available through `GetComm()` for direct communication interface access.

**Location**: [`inc/tmc51x0_comm_interface.hpp`](../inc/tmc51x0_comm_interface.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetMode()` | `CommMode GetMode() const noexcept` | CommMode enum | Get communication mode (SPI or UART) |
| `ReadRegister()` | `Result<uint32_t> ReadRegister(uint8_t address, uint8_t daisy_chain_position = 0) noexcept` | `Result<uint32_t>` containing the value or error | Read 32-bit register. For SPI: daisy-chain position. For UART: node address. |
| `WriteRegister()` | `Result<void> WriteRegister(uint8_t address, uint32_t value, uint8_t daisy_chain_position = 0) noexcept` | `Result<void>` indicating success or error | Write 32-bit register. For SPI: daisy-chain position. For UART: node address. |
| `GpioSet()` | `Result<void> GpioSet(TMC51x0CtrlPin pin, GpioSignal signal) noexcept` | `Result<void>` indicating success or error | Set GPIO pin state |
| `GpioRead()` | `Result<GpioSignal> GpioRead(TMC51x0CtrlPin pin) noexcept` | `Result<GpioSignal>` containing the value or error | Read GPIO pin state |
| `GpioSetActive()` | `Result<void> GpioSetActive(TMC51x0CtrlPin pin) noexcept` | `Result<void>` indicating success or error | Set GPIO pin to active state |
| `GpioSetInactive()` | `Result<void> GpioSetInactive(TMC51x0CtrlPin pin) noexcept` | `Result<void>` indicating success or error | Set GPIO pin to inactive state |
| `SignalToGpioLevel()` | `bool SignalToGpioLevel(TMC51x0CtrlPin pin, GpioSignal signal) const noexcept` | GPIO level | Convert signal to physical GPIO level |
| `GpioLevelToSignal()` | `GpioSignal GpioLevelToSignal(TMC51x0CtrlPin pin, bool gpio_level) const noexcept` | GpioSignal enum | Convert GPIO level to signal |
| `SetPinActiveLevel()` | `Result<void> SetPinActiveLevel(TMC51x0CtrlPin pin, bool active_level) noexcept` | `Result<void>` indicating success or error | Configure pin active level |
| `DelayMs()` | `void DelayMs(uint32_t ms) noexcept` | void | Delay milliseconds |
| `DelayUs()` | `void DelayUs(uint32_t us) noexcept` | void | Delay microseconds |
| `LogDebug()` | `void LogDebug(int level, const char* tag, const char* format, ...) noexcept` | void | Debug logging |

## TMC51x0 Class Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
**Note**: Each `TMC51x0` instance tracks its own daisy-chain position (SPI) or UART node address. These values are automatically passed to `ReadRegister()` and `WriteRegister()` methods in the communication interface via `GetCommAddress()`.

## SpiCommInterface Methods (SPI Only)

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `SetDaisyChainLength()` | `void SetDaisyChainLength(uint8_t total_length) noexcept` | void | Set total number of devices in daisy chain (for proper response extraction) |
| `GetDaisyChainLength()` | `uint8_t GetDaisyChainLength() const noexcept` | Chain length (0-255) | Get current daisy chain length setting |
| `AutoDetectChainLength()` | `uint8_t AutoDetectChainLength(uint8_t max_devices = 8) noexcept` | Detected length (0-255) | Auto-detect daisy chain length by sending command that loops back |

**Note**: `SetDaisyChainLength()` is critical for proper response extraction using the datasheet formula `40·(n-k+1)`. The chain length represents the total number of devices in the daisy chain, not individual device positions.
| `SetNaiPin()` | `Result<void> SetNaiPin(bool active) noexcept` | `Result<void>` indicating success or error | Set NAI pin state for UART sequential addressing (UART only) |
| `GetNaoPin()` | `Result<bool> GetNaoPin() noexcept` | `Result<bool>` containing true if NAO is active, false otherwise | Read NAO pin state for UART sequential addressing (UART only) |

**Note**: `UartCommInterface` no longer stores node addresses. Multiple `TMC51x0` instances share one `UartCommInterface` on the same UART bus. Each `TMC51x0` instance stores its own `uart_node_address_` and passes it to `ReadRegister()`/`WriteRegister()` automatically.

**UART Addressing**: Per datasheet procedure (Figure 5.1), devices are programmed backwards from address 254 (254, 253, 252, ...). Logical device indices (0, 1, 2, ...) used in software correspond to physical addresses (254, 253, 252, ...) programmed into chips. Use `TMC51x0MultiNode` class for managing multiple devices with sequential programming.

## Enums

### RampMode

| Value | Description |
|-------|-------------|
| `POSITIONING` | Positioning mode - move to target position |
| `VELOCITY_POS` | Velocity mode - positive direction |
| `VELOCITY_NEG` | Velocity mode - negative direction |
| `HOLD` | Hold mode - maintain current position |

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

### DriverStatus

| Value | Description |
|-------|-------------|
| `OK` | No errors |
| `CP_UV` | Charge pump undervoltage |
| `S2VSA` | Short to supply phase A |
| `S2VSB` | Short to supply phase B |
| `S2GSA` | Short to ground phase A |
| `S2GSB` | Short to ground phase B |
| `OT` | Overtemperature |
| `OTPW` | Overtemperature pre-warning |
| `S2GA` | Short to ground phase A |
| `S2GB` | Short to ground phase B |

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

### CommMode

| Value | Description |
|-------|-------------|
| `SPI` | SPI communication mode |
| `UART` | UART communication mode |

**Location**: [`inc/tmc51x0_comm_interface.hpp`](../inc/tmc51x0_comm_interface.hpp)

### ChipCommMode

Chip communication and motion control mode configuration. Represents the combination of SPI_MODE (pin 22) and SD_MODE (pin 21) pins.

| Value | Description | SPI_MODE | SD_MODE |
|-------|-------------|----------|---------|
| `SPI_INTERNAL_RAMP` | SPI interface with internal ramp generator (motion controller) | HIGH | LOW |
| `SPI_EXTERNAL_STEPDIR` | SPI interface with external step/dir inputs | HIGH | HIGH |
| `UART_INTERNAL_RAMP` | UART interface with internal ramp generator (motion controller) | LOW | LOW |
| `STANDALONE_EXTERNAL_STEPDIR` | Standalone Step/Dir mode (no SPI/UART, CFG pins configure driver) | LOW | HIGH |

**⚠️ WARNING**: 
- These pins are **typically hardwired** and read at startup
- Only use `communication.SetOperatingMode()` if SPI_MODE and SD_MODE are connected to GPIO outputs
- Mode changes require a chip reset (power cycle or reset pin) to take effect
- The mode pins are read at startup, so changes won't be effective until reset

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

### TMC51x0CtrlPin

| Value | Description |
|-------|-------------|
| `EN` | Enable pin |
| `DIR` | Direction pin |
| `STEP` | Step pin |

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

### GpioSignal

| Value | Description |
|-------|-------------|
| `ACTIVE` | Active signal state |
| `INACTIVE` | Inactive signal state |

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

## Configuration Structures

### DriverConfig

Main driver configuration structure containing all parameters for initializing the TMC51x0 driver.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

**Key Features**:
- **Automatic Current Calculation**: IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated from `motor_spec` parameters during initialization
- **User-Friendly Parameters**: All configuration uses physical parameters (voltage, current, time) rather than raw register values
- **Compile-Time Configuration**: For ESP32 examples, use helper functions from `esp32_tmc5160_test_config.hpp`

| Field | Type | Description |
|-------|------|-------------|
| `motor_spec` | `MotorSpec` | Motor specifications (physical parameters for automatic current calculation) |
| `mechanical` | `MechanicalSystem` | Mechanical system configuration (gearing, leadscrew, etc.) for unit conversions |
| `power_stage` | `PowerStageParameters` | Power stage configuration (MOSFET parameters, BBM time, sense filter, over-temperature protection, short protection) |
| `chopper` | `ChopperConfig` | Chopper timing and microstep settings (SpreadCycle or Classic mode) |
| `stealthchop` | `StealthChopConfig` | StealthChop PWM configuration |
| `global_config` | `GlobalConfig` | Global configuration (GCONF register) |
| `ramp_config` | `RampConfig` | Ramp generator configuration (velocities, accelerations, timing with unit support) |
| `direction` | `MotorDirection` | Motor direction (NORMAL or INVERTED) |
| `external_clk_config` | `ExternalClockConfig` | External clock configuration (0 = use internal 12 MHz, >0 = external clock frequency) |
| `stallguard` | `StallGuardConfig` | StallGuard2 configuration (defaults: threshold=0, disabled) |
| `coolstep` | `CoolStepConfig` | CoolStep configuration (defaults: lower_threshold_sg=0, disabled) |
| `dcstep` | `DcStepConfig` | DcStep configuration (defaults: min_velocity=0, disabled) |
| `reference_switch_config` | `ReferenceSwitchConfig` | Reference switch configuration (defaults: stop disabled, latching disabled) |
| `encoder_config` | `EncoderConfig` | Encoder configuration (includes pulses per rev, invert direction, deviation) |
| `uart_config` | `UartConfig` | UART communication configuration (node address, send delay, only used in UART mode) |

**Important Notes**:
- **DO NOT** manually set IRUN, IHOLD, or GLOBAL_SCALER - these are calculated automatically from `motor_spec`
- **REQUIRED** for automatic calculation: `motor_spec.sense_resistor_mohm` and `motor_spec.supply_voltage_mv` must be non-zero
- Current settings are calculated during `Initialize()` and stored internally (not in `motor_spec`)
- **Clock frequency (f_clk)**: Automatically calculated from `external_clk_config.frequency_hz` during `Initialize()`
  - If `external_clk_config.frequency_hz = 0`: Uses internal oscillator (~12 MHz, CLK pin tied to GND)
  - If `external_clk_config.frequency_hz > 0`: Uses external clock at specified frequency (CLK pin receives clock signal)
- See [`configuration.md`](../docs/configuration.md) for detailed configuration guide

### ChopperConfig

User-friendly configuration for SpreadCycle and Classic chopper modes.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

SpreadCycle is a cycle-by-cycle current control providing superior microstepping quality. Classic mode is an alternative constant off-time chopper algorithm.

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `mode` | `ChopperMode` | - | Chopper mode (SPREAD_CYCLE recommended, CLASSIC alternative) |
| `toff` | `uint8_t` | 0-15 | Off time setting (0=disabled, 1-15=off time, 5=typical) |
| `tbl` | `uint8_t` | 0-3 | Blank time (ChopperBlankTime enum or 0-3, 2=typical) |
| `hstrt` | `uint8_t` | 0-7 | Hysteresis start (SpreadCycle only, 0-7, 4=typical) |
| `hend` | `uint8_t` | 0-15 | Hysteresis end (SpreadCycle) or offset (Classic), encoded |
| `tfd` | `uint8_t` | 0-15 | Fast decay time (Classic only, 0=slow decay only) |
| `disfdcc` | `bool` | - | Disable fast decay comparator (Classic only) |
| `tpfd` | `uint8_t` | 0-15 | Passive fast decay time (0=disabled, helps reduce resonances) |
| `mres` | `uint8_t` | 0-8 | Microstep resolution (MicrostepResolution enum or 0-8, 4=16 microsteps) |
| `intpol` | `bool` | - | Enable interpolation to 256 microsteps (recommended) |
| `dedge` | `bool` | - | Enable double edge step pulses (typically false) |
| `vhighfs` | `bool` | - | High velocity fullstep selection (false=normal, true=fullstep at high velocity) |
| `vhighchm` | `bool` | - | High velocity chopper mode (false=normal, true=Classic mode at high velocity) |
| `diss2g` | `bool` | - | Short to GND protection disable (false=protection ON, true=protection OFF) |
| `diss2vs` | `bool` | - | Short to supply protection disable (false=protection ON, true=protection OFF) |

**Enums**:

- **`ChopperMode`**: 
  - `SPREAD_CYCLE` - Patented high-performance algorithm (recommended)
  - `CLASSIC` - Constant off-time chopper mode (alternative)

- **`ChopperBlankTime`**: Comparator blank time
  - `TBL_16CLK` - 16 clock cycles (~1.33µs @ 12MHz)
  - `TBL_24CLK` - 24 clock cycles (~2.0µs @ 12MHz)
  - `TBL_36CLK` - 36 clock cycles (~3.0µs @ 12MHz, typical)
  - `TBL_54CLK` - 54 clock cycles (~4.5µs @ 12MHz, for high capacitive loads)

- **`MicrostepResolution`**: Microstep resolution
  - `MRES_256` - 256 microsteps per full step (highest resolution)
  - `MRES_128` - 128 microsteps per full step
  - `MRES_64` - 64 microsteps per full step
  - `MRES_32` - 32 microsteps per full step
  - `MRES_16` - 16 microsteps per full step (typical)
  - `MRES_8` - 8 microsteps per full step
  - `MRES_4` - 4 microsteps per full step
  - `MRES_2` - 2 microsteps per full step
  - `FULLSTEP` - Full step (no microstepping)

**Usage Notes**: 
- **SpreadCycle** (recommended): Superior microstepping quality, automatically determines optimum fast-decay phase.
- **Classic mode**: Alternative algorithm, requires more tuning (TFD, OFFSET).
- **Chopper frequency**: Most motors work optimally in 16-30kHz range.
- **TOFF calculation**: Based on target chopper frequency and slow decay percentage.
- **Hysteresis**: Start from low setting (HSTRT=0, HEND=0) and increase until motor runs smoothly.

**See Also**: 
- [Advanced Configuration Guide](../docs/special_features_advanced_configuration.md#spreadcycle-and-classic-chopper) for detailed tuning guide and examples

### StealthChopConfig

User-friendly configuration for StealthChop2 voltage PWM mode operation.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

StealthChop provides extremely quiet, noiseless operation for stepper motors, making it ideal for indoor or home use applications. StealthChop2 features automatic tuning that adapts operating parameters to the motor automatically.

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pwm_ofs` | `uint8_t` | 0-255 | PWM amplitude offset (0=disable scaling, 30=typical) |
| `pwm_grad` | `uint8_t` | 0-255 | PWM amplitude gradient (velocity compensation, 0=auto) |
| `pwm_freq` | `uint8_t` | 0-3 | PWM frequency selection (see StealthChopPwmFreq enum) |
| `pwm_autoscale` | `bool` | - | Enable automatic current scaling (recommended: true) |
| `pwm_autograd` | `bool` | - | Enable automatic gradient adaptation (recommended: true) |
| `pwm_reg` | `uint8_t` | 1-15 | Regulation loop coefficient (1=slow, 15=fast, 4=balanced) |
| `pwm_lim` | `uint8_t` | 0-15 | PWM amplitude limit for mode switching (12=default) |
| `freewheel` | `PWMFreewheel` | - | Freewheeling mode when I_HOLD=0 |

**Enums**:

- **`StealthChopPwmFreq`**: PWM frequency selection
  - `PWM_FREQ_0` - ~23.4kHz @ 12MHz clock
  - `PWM_FREQ_1` - ~35.1kHz @ 12MHz clock (recommended)
  - `PWM_FREQ_2` - ~46.9kHz @ 12MHz clock
  - `PWM_FREQ_3` - ~58.5kHz @ 12MHz clock

- **`StealthChopRegulationSpeed`**: Regulation speed (PWM amplitude change per half wave)
  - `VERY_SLOW` (1) - 0.5 increments (slowest, most stable)
  - `SLOW` (2) - 1 increment
  - `MODERATE` (4) - 2 increments (default, balanced)
  - `FAST` (8) - 4 increments
  - `VERY_FAST` (15) - 7.5 increments (fastest, may be less stable)

- **`StealthChopJerkReduction`**: Mode switching jerk reduction
  - `MAXIMUM` (8) - Maximum jerk reduction (smoothest switching)
  - `HIGH` (10) - High jerk reduction
  - `MODERATE` (12) - Moderate jerk reduction (default, balanced)
  - `LOW` (14) - Low jerk reduction
  - `MINIMUM` (15) - Minimum jerk reduction (fastest switching, may cause spikes)

**Automatic Tuning Requirements**:
- **AT#1**: Motor in standstill with nominal run current (≤130ms)
- **AT#2**: Motor moving at medium velocity (60-300 RPM typical, 8+ fullsteps)

**Important Notes**: 
- StealthChop requires motor to be at standstill when first enabled
- Keep motor stopped for at least 128 chopper periods after enabling
- StealthChop and StallGuard2 are mutually exclusive
- Lower current limit applies: IRUN ≥ 8 and current must exceed I_LOWER_LIMIT
- Open load detection should be performed in SpreadCycle before enabling StealthChop
- Motor stall during StealthChop can cause overcurrent - tune low-side overcurrent detection

**See Also**: 
- [Advanced Configuration Guide - StealthChop](../docs/special_features_advanced_configuration.md#stealthchop-voltage-pwm-mode) for detailed tuning guide and examples
- `GetPwmAuto()` method to read automatic tuning results (PWM_OFS_AUTO, PWM_GRAD_AUTO)
- `GetPwmScale()` method to monitor PWM_SCALE_SUM and PWM_SCALE_AUTO
- `SetModeChangeSpeeds()` method to configure TPWMTHRS (StealthChop velocity threshold)

### EncoderConfig

Encoder configuration structure with intuitive enum-based API.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `n_channel_active` | `ReferenceSwitchActiveLevel` | N channel active level (ACTIVE_LOW or ACTIVE_HIGH) |
| `require_a_high` | `bool` | Require A channel HIGH for N event validation (pol_A) |
| `require_b_high` | `bool` | Require B channel HIGH for N event validation (pol_B) |
| `ignore_ab_polarity` | `bool` | Ignore A and B polarity for N channel event (ignore_AB) |
| `n_sensitivity` | `EncoderNSensitivity` | N channel event sensitivity (ACTIVE_LEVEL, RISING_EDGE, FALLING_EDGE, BOTH_EDGES) |
| `clear_mode` | `EncoderClearMode` | Clear mode (DISABLED, ONCE, CONTINUOUS) |
| `clear_enc_x_on_event` | `bool` | Clear encoder counter X_ENC upon N-event (clr_enc_x) |
| `latch_xactual_with_enc` | `bool` | Also latch XACTUAL position together with X_ENC (latch_x_act) |
| `prescaler_mode` | `EncoderPrescalerMode` | Encoder prescaler divisor mode (BINARY or DECIMAL) |

**Enums:**

- **`EncoderNSensitivity`**: `ACTIVE_LEVEL`, `RISING_EDGE`, `FALLING_EDGE`, `BOTH_EDGES`
- **`EncoderClearMode`**: `DISABLED`, `ONCE`, `CONTINUOUS`
- **`EncoderPrescalerMode`**: `BINARY`, `DECIMAL`

**Note**: All fields must be explicitly set. Register values are calculated automatically by the `Configure()` method.

#### Encoder Configuration Usage

```cpp
// Initial configuration
tmc51x0::EncoderConfig enc_cfg{};
enc_cfg.n_channel_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH;
enc_cfg.n_sensitivity = tmc51x0::EncoderNSensitivity::RISING_EDGE;
enc_cfg.ignore_ab_polarity = true;
enc_cfg.clear_mode = tmc51x0::EncoderClearMode::ONCE;
enc_cfg.prescaler_mode = tmc51x0::EncoderPrescalerMode::BINARY;
driver.encoder.Configure(enc_cfg);

// Real-time configuration updates
driver.encoder.SetNChannelSensitivity(tmc51x0::EncoderNSensitivity::BOTH_EDGES);
driver.encoder.SetClearMode(tmc51x0::EncoderClearMode::CONTINUOUS);

// Set encoder resolution (automatically calculates ENC_CONST)
driver.encoder.SetResolution(200, 1000, false);  // 200 motor steps, 1000 encoder pulses

// Monitor encoder deviation
if (driver.encoder.IsDeviationDetected()) {
    // Handle step loss
    driver.encoder.ClearDeviationFlag();
}
```

### StallGuardConfig

User-friendly configuration for StallGuard2 load measurement and stall detection.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

StallGuard2 provides accurate measurement of motor load and can detect stalls. It's used for sensorless homing, CoolStep load-adaptive current reduction, and diagnostics.

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `threshold` | `int8_t` | -64 to +63 | StallGuard2 threshold value (0 = starting value, works with most motors) |
| `enable_filter` | `bool` | - | Enable StallGuard2 filter (reduces measurement rate 4x, smoother readings) |
| `min_velocity` | `float` | - | Lower velocity threshold for StallGuard2 operation (0 = no lower limit) |
| `max_velocity` | `float` | - | Upper velocity threshold for StallGuard2 operation (0 = no upper limit) |
| `velocity_unit` | `Unit` | - | Unit for velocity thresholds (Steps, Rad, Deg, Mm, RPM) |
| `stop_on_stall` | `bool` | - | Stop motor when stall detected (requires threshold tuned correctly) |

**Enums**:

- **`StallGuardSensitivity`**: 
  - `VERY_HIGH` (SGT = -32) - Very high sensitivity, detects stalls very easily
  - `HIGH` (SGT = -16) - High sensitivity, detects stalls easily
  - `MODERATE` (SGT = 0) - Moderate sensitivity, starting value (recommended)
  - `LOW` (SGT = 16) - Low sensitivity, requires more torque to detect stall
  - `VERY_LOW` (SGT = 32) - Very low sensitivity, requires significant torque

**Usage Notes**: 
- **Prerequisites**: StallGuard2 requires SpreadCycle mode (`en_pwm_mode=0`). StealthChop must be disabled.
- **Threshold Tuning**: Adjust `threshold` until SG_RESULT is between 0-100 at maximum load before stall.
- **Velocity Range**: Set `min_velocity` and `max_velocity` to match your typical operating speed range.
- **Filter**: Enable for smoother readings (CoolStep), disable for faster response (sensorless homing).
- **Stop on Stall**: Requires `min_velocity` to be set (TCOOLTHRS must be configured).

**See Also**: 
- [Advanced Configuration Guide](../docs/special_features_advanced_configuration.md#stallguard2-load-measurement) for detailed tuning guide and examples
- `TuneStallGuard()` with `StallGuardTuningResult` for comprehensive automatic tuning
- `GetStallGuard()` method to read SG_RESULT value
- `EnableStopOnStall()` method for real-time control

### StallGuardTuningResult

Comprehensive result structure from automatic SGT tuning with velocity range analysis.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `optimal_sgt` | `int8_t` | Optimal SGT value found at target velocity (primary result) |
| `tuning_success` | `bool` | True if optimal SGT was found at target velocity |
| `min_velocity_sgt` | `int8_t` | SGT that works at min velocity (if verified) |
| `min_velocity_success` | `bool` | True if requested min velocity works with optimal SGT |
| `max_velocity_sgt` | `int8_t` | SGT that works at max velocity (if verified) |
| `max_velocity_success` | `bool` | True if requested max velocity works with optimal SGT |
| `actual_min_velocity` | `float` | Actual min velocity that works (if requested min doesn't) |
| `actual_max_velocity` | `float` | Actual max velocity that works (if requested max doesn't) |
| `target_velocity_sg_result` | `uint16_t` | SG_RESULT value at target velocity with optimal SGT |
| `min_velocity_sg_result` | `uint16_t` | SG_RESULT value at min velocity (if verified) |
| `max_velocity_sg_result` | `uint16_t` | SG_RESULT value at max velocity (if verified) |

**Usage Notes**:
- Target velocity is the most important parameter - optimal SGT is determined here first
- Min and max velocities are used to determine the range of SGT values that work
- If requested velocities aren't achievable, `actual_min_velocity` and `actual_max_velocity` contain the velocities that DO work
- All velocity values are in the same unit as the tuning request

### PowerStageParameters (Short Protection Fields)

Short circuit protection is configured through the `PowerStageParameters` structure using user-friendly voltage thresholds and timing values. The driver automatically converts these to register values based on datasheet specifications.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `s2vs_voltage_mv` | `uint16_t` | 0 (auto) or 400-2000 | Short to VS voltage threshold in mV (0 = auto = 625mV, equivalent to S2VS_LEVEL=6) |
| `s2g_voltage_mv` | `uint16_t` | 0 (auto) or 400-2000 | Short to GND voltage threshold in mV (0 = auto = 625mV, equivalent to S2G_LEVEL=6). Minimum 1200mV if VS>52V |
| `shortfilter` | `uint8_t` | 0-3 | Spike filter bandwidth (0=100ns, 1=1µs default, 2=2µs, 3=3µs) |
| `short_detection_delay_us_x10` | `uint8_t` | 0 (auto) or 5-25 | Detection delay in 0.1µs units (0 = auto = 0.85µs, equivalent to shortdelay=0) |

**Datasheet Voltage Thresholds (typical values):**
- **S2VS_LEVEL=6**: 550-625-700mV (recommended for normal operation)
- **S2VS_LEVEL=15**: 1400-1560-1720mV (lowest sensitivity)
- **S2G_LEVEL=6 (VS<50V)**: 460-625-800mV (recommended for normal operation)
- **S2G_LEVEL=15 (VS<52V)**: 1200-1560-1900mV
- **S2G_LEVEL=15 (VS<55V)**: 850mV (minimum for VS>52V to prevent false triggers)

**Detection Delay Timing (typical values):**
- **shortdelay=0**: 0.5-0.85-1.1µs (normal, recommended for most applications)
- **shortdelay=1**: 1.1-1.6-2.2µs (high delay)

The driver automatically converts voltage thresholds to register levels using interpolation based on datasheet specifications.

### MotorSpec

Motor specification structure for high-level setup.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `steps_per_rev` | `uint16_t` | Steps per revolution (typically 200 for 1.8° motors) |
| `rated_current_ma` | `uint16_t` | Rated motor current in milliamps |
| `winding_resistance_mohm` | `uint32_t` | Winding resistance in milliohms (optional, 0 = not specified) |
| `winding_inductance_mh` | `float` | Winding inductance in millihenries (optional, 0 = not specified) |

### MechanicalSystem

Mechanical system configuration for unit conversions.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `system_type` | `MechanicalSystemType` | System type (DirectDrive, LeadScrew, BeltDrive, Gearbox) |
| `lead_screw_pitch_mm` | `float` | Lead screw pitch in mm (for LeadScrew) |
| `belt_pulley_teeth` | `uint16_t` | Number of teeth on motor pulley (for BeltDrive) |
| `belt_pitch_mm` | `float` | Belt pitch in mm (for BeltDrive) |
| `gear_ratio` | `float` | Gear ratio (output/input, for Gearbox) |

### CoolStepConfig

User-friendly configuration for CoolStep automatic current reduction feature.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

CoolStep automatically reduces motor current when load is low, saving energy and reducing heat. It uses StallGuard2 to measure motor load and adjusts current accordingly.

**User-Friendly Fields** (recommended):

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `lower_threshold_sg` | `uint16_t` | 0-1023 | Lower SG threshold for current increase (0 = use `semin`) |
| `upper_threshold_sg` | `uint16_t` | 0-1023 | Upper SG threshold for current decrease (0 = use `semax`) |
| `increment_step` | `CoolStepIncrementStep` | - | Current increment step width (enum) |
| `decrement_speed` | `CoolStepDecrementSpeed` | - | Current decrement speed (enum) |
| `min_current` | `CoolStepMinCurrent` | - | Minimum current percentage (enum) |
| `enable_filter` | `bool` | - | Enable StallGuard2 filter (reduces measurement rate 4x) |
| `min_velocity` | `float` | - | Lower velocity threshold for CoolStep activation |
| `max_velocity` | `float` | - | Upper velocity threshold for CoolStep activation |
| `velocity_unit` | `Unit` | - | Unit for velocity thresholds |

**Note**: Register values (`SEMIN`, `SEMAX`) are automatically calculated from `lower_threshold_sg` and `upper_threshold_sg` internally. You only need to specify the actual SG threshold values.

**Enums**:

- **`CoolStepIncrementStep`**: 
  - `STEP_1` - Increment by 1 step per measurement (slowest, smoothest)
  - `STEP_2` - Increment by 2 steps per measurement (recommended default)
  - `STEP_4` - Increment by 4 steps per measurement (fast response)
  - `STEP_8` - Increment by 8 steps per measurement (fastest, may oscillate)

- **`CoolStepDecrementSpeed`**: 
  - `EVERY_32` - Decrement every 32 measurements (slowest reduction, most stable)
  - `EVERY_8` - Decrement every 8 measurements (recommended default)
  - `EVERY_2` - Decrement every 2 measurements (fast reduction)
  - `EVERY_1` - Decrement every measurement (fastest, may oscillate)

- **`CoolStepMinCurrent`**: 
  - `HALF_IRUN` - Minimum current is 50% of IRUN (conservative, recommended)
  - `QUARTER_IRUN` - Minimum current is 25% of IRUN (aggressive, maximum savings)

**Threshold Conversion** (automatic):
- SG thresholds (0-1023) are automatically converted to register values internally:
  - `lower_threshold_sg / 32 = SEMIN` (register value 0-15)
  - `upper_threshold_sg / 32 - SEMIN - 1 = SEMAX` (register value 0-15)
- Example: `lower_threshold_sg = 64` → SEMIN = 2, `upper_threshold_sg = 256` → SEMAX = 5
- If `upper_threshold_sg` is not specified (0), a default hysteresis is used (SEMAX = 5)

**Usage Notes**: 
- **Prerequisites**: CoolStep requires SpreadCycle mode (`en_pwm_mode=0`). StealthChop must be disabled.
- **StallGuard2**: CoolStep uses StallGuard2 to measure load. Tune StallGuard2 threshold (`sgt`) first.
- **Velocity Range**: Set `min_velocity` and `max_velocity` to match your typical operating speed range.
- **Disabling**: Set `lower_threshold_sg = 0` to disable CoolStep.

**See Also**: 
- [Advanced Configuration Guide](../docs/special_features_advanced_configuration.md#coolstep-current-reduction) for detailed tuning guide and examples
- `GetStallGuard()` method to monitor SG_RESULT for threshold tuning

### ReferenceSwitchConfig

Reference switch/endstop configuration for homing and limit detection.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

**Enumerations**:

- `ReferenceSwitchActiveLevel`:
  - `ACTIVE_LOW` - Switch is active when signal is LOW (GND, typically pull-up resistor)
                       - Common for normally-closed switches and failsafe configurations
                       - Recommended for safety-critical applications (broken wire = HIGH = safe)
                       - Sets inverted polarity (pol_stop = 1)
  - `ACTIVE_HIGH` - Switch is active when signal is HIGH (VCC, typically pull-down resistor)
                        - Common for normally-open switches and photo interrupters
                        - Sets normal polarity (pol_stop = 0)
  
  **Note**: Active level must always be specified (ACTIVE_LOW or ACTIVE_HIGH).
            Use `stop_enable` to control whether the switch stops the motor.
            This allows configuring polarity while enabling/disabling stop functionality in real-time.

- `ReferenceStopMode`:
  - `HARD_STOP` - Abrupt stop (immediate, no deceleration, precise for homing)
  - `SOFT_STOP` - Soft stop using deceleration ramp (smooth, prevents mechanical shock)

- `ReferenceLatchMode`:
  - `DISABLED` - No position latching
  - `ACTIVE_EDGE` - Latch position on active edge (switch becomes active) - most common for homing
  - `INACTIVE_EDGE` - Latch position on inactive edge (switch becomes inactive)
  - `BOTH_EDGES` - Latch position on both active and inactive edges

**Configuration Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `left_switch_active` | `ReferenceSwitchActiveLevel` | Left switch active level (REFL) - must be ACTIVE_LOW or ACTIVE_HIGH (determines polarity) |
| `right_switch_active` | `ReferenceSwitchActiveLevel` | Right switch active level (REFR) - must be ACTIVE_LOW or ACTIVE_HIGH (determines polarity) |
| `left_switch_stop_enable` | `bool` | Enable motor stop on left switch (independent of active level)<br/>true = stop motor when active<br/>false = don't stop (but can still latch/read switch state) |
| `right_switch_stop_enable` | `bool` | Enable motor stop on right switch (independent of active level)<br/>true = stop motor when active<br/>false = don't stop (but can still latch/read switch state) |
| `stop_mode` | `ReferenceStopMode` | Stop mode (hard or soft) - only applies if stop is enabled |
| `swap_left_right` | `bool` | Swap left and right switch inputs (useful for reversed wiring) |
| `latch_left` | `ReferenceLatchMode` | Left switch latching mode (must be explicitly set) |
| `latch_right` | `ReferenceLatchMode` | Right switch latching mode (must be explicitly set) |
| `en_latch_encoder` | `bool` | Latch encoder position on switch event (for encoder N-channel as third switch) |

**Important Notes**: 
- **Active level must always be specified**: `left_switch_active` and `right_switch_active` must be `ACTIVE_LOW` or `ACTIVE_HIGH` (never disabled)
- **Polarity is determined by active level**: ACTIVE_LOW = inverted polarity (pol_stop = 1), ACTIVE_HIGH = normal polarity (pol_stop = 0)
- **Stop enable is independent**: `stop_enable` controls whether the motor stops (allows real-time enable/disable while keeping polarity configured)
- **Use case**: Configure active level once, then enable/disable motor stop in real-time. Switch state can still be read even when stop is disabled
- All register values (polarity, latching flags, soft stop) are computed inline when writing to the SW_MODE register

**Note**: All configuration fields must be explicitly set. There are no computed fields or auto-configuration methods.

**Usage Example**:

```cpp
// Configure switches manually - all fields must be explicitly set
tmc51x0::ReferenceSwitchConfig config{};
config.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;   // Active LOW (determines polarity)
config.left_switch_stop_enable = true;                                         // Enable motor stop
config.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH; // Active HIGH (determines polarity)
config.right_switch_stop_enable = false;                                       // Don't stop motor (but can still latch/read)
config.stop_mode = tmc51x0::ReferenceStopMode::SOFT_STOP;
config.latch_left = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;   // Latch on active edge
config.latch_right = tmc51x0::ReferenceLatchMode::BOTH_EDGES;   // Latch on both edges
driver.rampControl.ConfigureReferenceSwitch(config);

// Example: Switch configured but doesn't stop motor (only latches/reads position)
// Useful for reading switch state without stopping motor
tmc51x0::ReferenceSwitchConfig config_latch_only{};
config_latch_only.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;  // Must specify active level
config_latch_only.left_switch_stop_enable = false;  // Don't stop motor (but polarity is configured for reading)
config_latch_only.latch_left = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;  // But latch position
config_latch_only.latch_right = tmc51x0::ReferenceLatchMode::DISABLED;  // No latching on right
driver.rampControl.ConfigureReferenceSwitch(config_latch_only);

// Example: Real-time updates using convenience methods
// Initial configuration
tmc51x0::ReferenceSwitchConfig config{};
config.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
config.left_switch_stop_enable = true;
config.latch_left = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;
driver.rampControl.ConfigureReferenceSwitch(config);

// Later: Real-time updates using convenience methods (preserves other settings)
driver.rampControl.SetLeftSwitchStopEnable(false);  // Disable stop without changing polarity
driver.rampControl.SetLeftSwitchLatchMode(tmc51x0::ReferenceLatchMode::BOTH_EDGES);  // Change latch mode
driver.rampControl.SetStopMode(tmc51x0::ReferenceStopMode::HARD_STOP);  // Change stop mode
driver.rampControl.SetLeftSwitchActiveLevel(tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH);  // Change polarity

// Read current configuration
tmc51x0::ReferenceSwitchConfig current_config{};
if (driver.rampControl.GetReferenceSwitchConfig(current_config)) {
  // Use current_config for inspection or modification
}

// For homing (hard stop, precise)
tmc51x0::ReferenceSwitchConfig homing_config{};
homing_config.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
homing_config.left_switch_stop_enable = true;  // Enable stop for homing
homing_config.latch_left = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;  // Latch on active edge
homing_config.stop_mode = tmc51x0::ReferenceStopMode::HARD_STOP;  // Hard stop for precise homing
driver.rampControl.ConfigureReferenceSwitch(homing_config);
```

### DcStepConfig

User-friendly configuration for DcStep automatic commutation mode.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

DcStep allows the motor to run near its load limit without losing steps by automatically reducing velocity when overloaded. The motor operates in fullstep mode at the target velocity or at reduced velocity if overloaded.

| Field | Type | Description |
|-------|------|-------------|
| `min_velocity` | `float` | Minimum velocity threshold for DcStep activation (0 = disabled) |
| `velocity_unit` | `Unit` | Unit for velocity threshold (Steps, Rad, Deg, Mm, RPM) |
| `pwm_on_time_us` | `float` | PWM on-time limit in microseconds (0 = auto-calculate from blank time) |
| `stall_sensitivity` | `DcStepStallSensitivity` | Stall detection sensitivity (enum) |
| `stop_on_stall` | `bool` | Stop motor when stall detected (requires stall_sensitivity != DISABLED) |

**Enums**:

- **`DcStepStallSensitivity`**: 
  - `DISABLED` - Stall detection disabled (dc_sg = 0)
  - `LOW` - Low sensitivity - fewer false positives (dc_sg ≈ dc_time / 20)
  - `MODERATE` - Moderate sensitivity - balanced (dc_sg ≈ dc_time / 16, recommended)
  - `HIGH` - High sensitivity - detects stalls earlier (dc_sg ≈ dc_time / 12)

**Parameter Conversion** (automatic):
- PWM on-time (clock cycles) = `(pwm_on_time_us * f_clk) / 1e6` → DC_TIME register (0-1023)
- If `pwm_on_time_us = 0`: Auto-calculated from blank time (TBL) + 20 clock cycles
- DC_SG register (0-255) automatically calculated from DC_TIME based on `stall_sensitivity`

**Usage Notes**: 
- **Prerequisites**: DcStep requires SD_MODE=1 (external step/dir) or can be enabled via VDCMIN threshold.
- **CHOPCONF Settings**: `vhighfs` and `vhighchm` are automatically set to 1 for DcStep.
- **TOFF Setting**: Should be >2, preferably 8-15 for DcStep operation.
- **Velocity Range**: Set `min_velocity` to match your typical operating speed range.
- **PWM On-Time**: Should be slightly above blank time (TBL). Auto-calculation recommended.

**See Also**: 
- [Advanced Configuration Guide](../docs/special_features_advanced_configuration.md#dcstep-automatic-commutation) for detailed tuning guide and examples
- `GetLostSteps()` method to monitor step loss in DcStep mode

### GlobalConfig

Global configuration (GCONF register) structure.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `recalibrate` | `bool` | Zero crossing recalibration during driver disable |
| `en_short_standstill_timeout` | `bool` | Enable shorter timeout for standstill detection (true=2^18 clocks, false=2^20 clocks) |
| `en_stealthchop_mode` | `bool` | Enable StealthChop voltage PWM mode |
| `en_stealthchop_step_filter` | `bool` | Enable step input filtering for StealthChop optimization with external step source |
| `invert_direction` | `bool` | Invert motor direction (false=normal, true=inverse) |
| `diag0` | `Diag0Config` | DIAG0 pin configuration structure (see Diag0Config below) |
| `diag1` | `Diag1Config` | DIAG1 pin configuration structure (see Diag1Config below) |
| `en_small_step_frequency_hysteresis` | `bool` | Enable smaller hysteresis for step frequency comparison (true=1/32, false=1/16) |
| `enca_dcin_sequencer_stop` | `bool` | Enable ENCA_DCIN pin as sequencer stop input (stops ramp generator) |
| `direct_mode` | `bool` | Direct motor coil control via XTARGET |

### Diag0Config

DIAG0 pin configuration structure. Nested within `GlobalConfig` as `diag0` field.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

**Note**: DIAG0 pin behavior depends on SD_MODE setting:
- **SD_MODE=1** (External step/dir): Diagnostic outputs (error, otpw, stall)
- **SD_MODE=0** (Internal ramp): Can be used as STEP output

| Field | Type | Description |
|-------|------|-------------|
| `error` | `bool` | Bit 5: Enable DIAG0 on driver errors (OT, S2G, UV_CP) - SD_MODE=1 only |
| `otpw` | `bool` | Bit 6: Enable DIAG0 on overtemperature prewarning - SD_MODE=1 only |
| `stall_step` | `bool` | Bit 7: (SD_MODE=1) DIAG0 on stall, (SD_MODE=0) DIAG0 as STEP output (half frequency, dual edge) |
| `pushpull` | `bool` | Bit 12: Output mode (false=open collector active low, true=push pull active high) |

### Diag1Config

DIAG1 pin configuration structure. Nested within `GlobalConfig` as `diag1` field.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

**Note**: DIAG1 pin behavior depends on SD_MODE setting:
- **SD_MODE=1** (External step/dir): Diagnostic outputs (stall, index, onstate, steps_skipped)
- **SD_MODE=0** (Internal ramp): Can be used as DIR output or position compare signal

**Warning**: `steps_skipped` should not be enabled with other DIAG1 options (mutually exclusive).

| Field | Type | Description |
|-------|------|-------------|
| `stall_dir` | `bool` | Bit 8: (SD_MODE=1) DIAG1 on stall, (SD_MODE=0) DIAG1 as DIR output |
| `index` | `bool` | Bit 9: Enable DIAG1 on index position (microstep LUT position 0) - SD_MODE=1 only |
| `onstate` | `bool` | Bit 10: Enable DIAG1 when chopper is on (second half of fullstep) - SD_MODE=1 only |
| `steps_skipped` | `bool` | Bit 11: Enable output toggle when steps skipped in dcStep mode - SD_MODE=1 only |
| `pushpull` | `bool` | Bit 13: Output mode (false=open collector active low, true=push pull active high) |

### RampConfig

Ramp generator configuration structure for two-phase acceleration and deceleration.

**Location**: [`inc/tmc51x0_types.hpp`](../inc/tmc51x0_types.hpp)

**Unit Specifications** (critical for proper conversion):

| Field | Type | Default | Description |
|-------|------|--------|-------------|
| `velocity_unit` | `Unit` | `Steps` | Unit for all velocity parameters (vstart, vstop, vmax, v1). Supported: Steps, Rad, Deg, Mm, RPM |
| `acceleration_unit` | `Unit` | `Steps` | Unit for all acceleration parameters (amax, a1, dmax, d1). Supported: Steps, Rad, Deg, Mm (RPM not applicable) |

**Velocity Parameters** (unit specified by `velocity_unit` field):

| Field | Type | Default | Description |
|-------|------|--------|-------------|
| `vstart` | `float` | 0.0 | Start velocity (can be 0 if not used) |
| `vstop` | `float` | 10.0 | Stop velocity (must be >= VSTART, minimum 1 recommended) |
| `vmax` | `float` | 0.0 | Maximum velocity (must be set before motion) |
| `v1` | `float` | 0.0 | Transition velocity (switches between A1/AMAX and D1/DMAX, 0 = disabled) |

**Acceleration Parameters** (unit specified by `acceleration_unit` field):

| Field | Type | Default | Description |
|-------|------|--------|-------------|
| `amax` | `float` | 0.0 | Maximum acceleration (used above V1, must be set before motion) |
| `a1` | `float` | 0.0 | First acceleration (used between VSTART and V1, 0 = use AMAX) |
| `dmax` | `float` | 0.0 | Maximum deceleration (used above V1, 0 = uses AMAX value) |
| `d1` | `float` | 100.0 | First deceleration (used between VSTOP and V1, must not be 0 in positioning mode) |

**Timing Parameters** (in milliseconds):

| Field | Type | Default | Description |
|-------|------|--------|-------------|
| `tpowerdown_ms` | `float` | 437.0 | Power down delay in milliseconds (0-5600ms at 12MHz, automatically converted to register value, ~0.44s at 12MHz) |
| `tzerowait_ms` | `float` | 0.0 | Zero wait time in milliseconds (0-2000ms at 12MHz, automatically converted to register value). Prevents excessive jerk when changing direction. |

**Notes**:
- **CRITICAL**: `velocity_unit` and `acceleration_unit` fields specify the unit system for all velocity and acceleration values
- All velocity parameters (vstart, vstop, vmax, v1) use the unit specified by `velocity_unit`
- All acceleration parameters (amax, a1, dmax, d1) use the unit specified by `acceleration_unit`
- Parameters set to 0.0 will use driver defaults or be auto-calculated where applicable
- VSTOP must be >= VSTART to ensure successful motion termination
- D1 must not be 0 in positioning mode (defaults to 0.002 rev/s² if not set, equivalent to ~100 steps/s² for typical motor)
- Unit conversion requires `MotorSpec` and `MechanicalSystem` configuration for non-Step units
- See datasheet section 12 for detailed ramp generator operation

## Unit Conversion Functions

Free functions for converting between physical units and driver steps.

**Location**: [`inc/features/tmc51x0_units.hpp`](../inc/features/tmc51x0_units.hpp)

### Position Conversions

| Function | Signature | Returns | Description |
|----------|-----------|---------|-------------|
| `MmToSteps()` | `int32_t MmToSteps(float mm, uint16_t steps_per_rev, float lead_screw_pitch_mm) noexcept` | Steps | Convert millimeters to steps |
| `StepsToMm()` | `float StepsToMm(int32_t steps, uint16_t steps_per_rev, float lead_screw_pitch_mm) noexcept` | Millimeters | Convert steps to millimeters |
| `DegreesToSteps()` | `int32_t DegreesToSteps(float degrees, uint16_t steps_per_rev) noexcept` | Steps | Convert degrees to steps |
| `StepsToDegrees()` | `float StepsToDegrees(int32_t steps, uint16_t steps_per_rev) noexcept` | Degrees | Convert steps to degrees |
| `BeltTeethToSteps()` | `int32_t BeltTeethToSteps(uint32_t teeth, uint16_t steps_per_rev, uint16_t belt_pulley_teeth) noexcept` | Steps | Convert belt teeth to steps |
| `StepsToBeltTeeth()` | `float StepsToBeltTeeth(int32_t steps, uint16_t steps_per_rev, uint16_t belt_pulley_teeth) noexcept` | Belt teeth | Convert steps to belt teeth |

### Speed Conversions

| Function | Signature | Returns | Description |
|----------|-----------|---------|-------------|
| `RpmToStepsPerSec()` | `float RpmToStepsPerSec(float rpm, uint16_t steps_per_rev) noexcept` | Steps/s | Convert RPM to steps per second |
| `StepsPerSecToRpm()` | `float StepsPerSecToRpm(float steps_per_sec, uint16_t steps_per_rev) noexcept` | RPM | Convert steps per second to RPM |
| `MmPerSecToStepsPerSec()` | `float MmPerSecToStepsPerSec(float mm_per_sec, uint16_t steps_per_rev, float lead_screw_pitch_mm) noexcept` | Steps/s | Convert mm/s to steps/s |
| `StepsPerSecToMmPerSec()` | `float StepsPerSecToMmPerSec(float steps_per_sec, uint16_t steps_per_rev, float lead_screw_pitch_mm) noexcept` | mm/s | Convert steps/s to mm/s |

### Acceleration Conversions

| Function | Signature | Returns | Description |
|----------|-----------|---------|-------------|
| `AccelerationMmToSteps()` | `float AccelerationMmToSteps(float accel_mm_per_sec2, uint16_t steps_per_rev, float lead_screw_pitch_mm) noexcept` | Steps/s² | Convert mm/s² to steps/s² |
| `AccelerationStepsToMm()` | `float AccelerationStepsToMm(float accel_steps_per_sec2, uint16_t steps_per_rev, float lead_screw_pitch_mm) noexcept` | mm/s² | Convert steps/s² to mm/s² |

## Feature Implementation Summary

The TMC51x0 driver provides comprehensive coverage of all chipset features. This section summarizes the implementation status and available features.

### Core Functionality: 100% ✅

- **Ramp Control**: Complete - positioning, velocity, hold modes with configurable acceleration profiles
- **Current Control**: Complete - run/hold currents with global scaler
- **Chopper Modes**: Complete - spreadCycle and stealthChop operation
- **Encoder Support**: Complete - closed-loop control with deviation detection
- **StallGuard2**: Complete - stall detection with configurable thresholds
- **dcStep**: Complete - automatic commutation configuration
- **Protection**: Complete - short circuit, overtemperature, overvoltage protection

### Advanced Features: 100% ✅

- **Diagnostics**: Complete - access to all read-only diagnostic registers
  - `TSTEP` - Actual time between microsteps
  - `MSCNT` - Microstep table position
  - `MSCURACT` - Actual microstep current
  - `PWM_SCALE` - stealthChop PWM scale results
  - `PWM_AUTO` - Automatically determined PWM values
  - `ENC_LATCH` - Encoder position latched on N event
  - `IO_INPUT_OUTPUT` - GPIO pin read access
- **Factory Configuration**: Complete - clock trim read access
- **OTP Read**: Complete - OTP configuration read (programming intentionally not implemented)
- **UART Configuration**: Complete - slave address and send delay configuration
- **Offset Calibration**: Complete - phase offset calibration results

### Global Configuration (GCONF)

All GCONF register bits are now accessible through the `GlobalConfig` structure:

- `recalibrate` - Zero crossing recalibration
- `en_short_standstill_timeout` - Shorter timeout for standstill detection
- `en_stealthchop_mode` - Enable StealthChop voltage PWM mode
- `en_stealthchop_step_filter` - Step input filtering for StealthChop optimization
- `invert_direction` - Invert motor direction
- `diag0` - DIAG0 pin configuration (nested `Diag0Config` struct)
  - `diag0.error` - DIAG0 on driver errors
  - `diag0.otpw` - DIAG0 on overtemperature prewarning
  - `diag0.stall_step` - DIAG0 on stall/STEP output
  - `diag0.pushpull` - DIAG0 push-pull output mode
- `diag1` - DIAG1 pin configuration (nested `Diag1Config` struct)
  - `diag1.stall_dir` - DIAG1 on stall/DIR output
  - `diag1.index` - DIAG1 on index position
  - `diag1.onstate` - DIAG1 when chopper on
  - `diag1.steps_skipped` - DIAG1 on skipped steps
  - `diag1.pushpull` - DIAG1 push-pull output mode
- `en_small_step_frequency_hysteresis` - Smaller hysteresis for step frequency comparison
- `enca_dcin_sequencer_stop` - Enable ENCA_DCIN pin as sequencer stop input
- `direct_mode` - Direct motor coil control

**Usage:**
```cpp
tmc51x0::GlobalConfig gconf{};
gconf.en_short_standstill_timeout = true;
gconf.diag0.error = true;           // Access nested struct fields
gconf.diag0.pushpull = true;        // Enable push-pull output mode
gconf.diag1.stall_dir = true;      // Configure DIAG1
driver.motorControl.ConfigureGlobalConfig(gconf);
```

### Ramp Parameters

All ramp parameters are now accessible:

- `TPOWERDOWN` - Power down delay configuration
  - Method: `RampControl::SetPowerDownDelay(uint16_t tpowerdown)`
- `TZEROWAIT` - Zero wait time configuration
  - Method: `RampControl::SetZeroWaitTime(uint16_t tzerowait)`
- `A_1` - First acceleration phase
  - Method: `RampControl::SetFirstAcceleration(float a1)`

**Usage:**
```cpp
driver.rampControl.SetPowerDownDelay(10);  // 10 * 2^18 clocks
driver.rampControl.SetZeroWaitTime(100);   // 100 * 2^18 clocks
driver.rampControl.SetFirstAcceleration(500.0f); // 500 steps/s²
```

### Power Stage Enhancements

- `DRV_CONF.otselect` - Over temperature level selection
- `DRV_CONF.filt_isense` - Sense amplifier filter time constant

Both are accessible through the `PowerStageParameters` structure in `DriverConfig`.

### dcStep Enhancements

- `DCCTRL` register configuration with:
  - `dc_time` - dcStep time window
  - `dc_sg` - dcStep stallGuard threshold

**Usage:**
```cpp
tmc51x0::DcStepConfig dc_config{};
dc_config.dc_time = 100;
dc_config.dc_sg = 50;
driver.motorControl.ConfigureDcStep(dc_config);
```

## Error Handling

The driver uses `Result<T>` return types for explicit error handling, providing rich error information instead of simple boolean returns. This makes error handling explicit and helps prevent silent failures.

### Result<T> Types

- `Result<void>` for operations that don't return a value (e.g., `Initialize()`, `Enable()`)
- `Result<bool>` for boolean queries (e.g., `IsTargetReached()`)
- `Result<T>` for operations that return a value (e.g., `GetCurrentPosition()` returns `Result<float>`)

### Basic Error Handling Patterns

#### Pattern 1: Early Return (Recommended)

The most common pattern for critical operations - return immediately on error:

```cpp
// Void operations
auto init_result = driver.Initialize(cfg);
if (!init_result) {
    printf("Initialization error: %s\n", init_result.ErrorMessage());
    return -1; // or handle error appropriately
}

// Value operations
auto position = driver.rampControl.GetCurrentPosition();
if (!position) {
    printf("Error reading position: %s\n", position.ErrorMessage());
    return -1;
}
float pos = position.Value(); // Safe to use - we know it succeeded
```

#### Pattern 2: Boolean Queries

For methods returning `Result<bool>`, check both the result and the value:

```cpp
auto reached = driver.rampControl.IsTargetReached();
if (reached && reached.Value()) {
    // Target reached successfully
    printf("Target position reached\n");
} else if (!reached) {
    // Error occurred while checking
    printf("Error checking target: %s\n", reached.ErrorMessage());
    return -1;
} else {
    // Successfully checked, but target not reached yet
    // Continue waiting
}
```

#### Pattern 3: Error Logging with Continue

For non-critical operations where you want to log but continue:

```cpp
auto speed_result = driver.rampControl.SetMaxSpeed(100.0f);
if (!speed_result) {
    printf("Warning: Failed to set speed: %s\n", speed_result.ErrorMessage());
    // Continue with default or retry
}
```

#### Pattern 4: Using ValueOr() for Defaults

For operations where a default value is acceptable:

```cpp
auto position = driver.rampControl.GetCurrentPosition();
float pos = position.ValueOr(0.0f); // Use 0.0 if error occurred
// Note: This doesn't distinguish between actual 0.0 and error
// For production code, prefer explicit error checking
```

### Complete Example: Initialization with Full Error Handling

```cpp
// Initialize driver
tmc51x0::DriverConfig cfg{};
cfg.motor_spec.rated_current_ma = 2000;
cfg.motor_spec.sense_resistor_mohm = 50;
cfg.motor_spec.supply_voltage_mv = 24000;

auto init_result = driver.Initialize(cfg);
if (!init_result) {
    printf("Initialization error: %s\n", init_result.ErrorMessage());
    printf("Error code: %d\n", static_cast<int>(init_result.Error()));
    return -1;
}

// Configure ramp mode
auto mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
if (!mode_result) {
    printf("Error setting ramp mode: %s\n", mode_result.ErrorMessage());
    return -1;
}

// Set target position
auto pos_result = driver.rampControl.SetTargetPosition(1000);
if (!pos_result) {
    printf("Error setting target position: %s\n", pos_result.ErrorMessage());
    return -1;
}

// Enable motor
auto enable_result = driver.motorControl.Enable();
if (!enable_result) {
    printf("Error enabling motor: %s\n", enable_result.ErrorMessage());
    return -1;
}

// Wait for target with error checking
while (true) {
    auto reached = driver.rampControl.IsTargetReached();
    if (reached && reached.Value()) {
        printf("Target reached\n");
        break;
    }
    if (!reached) {
        printf("Error checking target: %s\n", reached.ErrorMessage());
        break;
    }
    // Target not reached yet, continue waiting
    driver.comm.DelayMs(10);
}
```

### Error Codes

The `Result<T>` type includes an `Error()` method that returns an `ErrorCode` enum:

```cpp
auto result = driver.Initialize(cfg);
if (!result) {
    tmc51x0::ErrorCode code = result.Error();
    switch (code) {
        case tmc51x0::ErrorCode::NOT_INITIALIZED:
            printf("Driver not initialized\n");
            break;
        case tmc51x0::ErrorCode::COMM_ERROR:
            printf("Communication error - check wiring\n");
            break;
        case tmc51x0::ErrorCode::INVALID_VALUE:
            printf("Invalid parameter value\n");
            break;
        // ... handle other error codes
        default:
            printf("Unknown error: %s\n", result.ErrorMessage());
            break;
    }
}
```

### Best Practices

1. **Always check return values**: Never ignore `Result<T>` return values
2. **Use early return**: For critical operations, return immediately on error
3. **Log errors**: Always log error messages for debugging
4. **Check boolean results properly**: For `Result<bool>`, check both the result and the value
5. **Handle errors appropriately**: Don't just log and continue - decide if the error is recoverable
6. **Use descriptive variable names**: `init_result`, `pos_result`, etc. make code more readable

### Common Error Handling Mistakes

**❌ Don't do this:**
```cpp
driver.Initialize(cfg);  // Error ignored!
driver.rampControl.SetTargetPosition(1000);  // Error ignored!
```

**✅ Do this:**
```cpp
auto init_result = driver.Initialize(cfg);
if (!init_result) {
    printf("Error: %s\n", init_result.ErrorMessage());
    return -1;
}

auto pos_result = driver.rampControl.SetTargetPosition(1000);
if (!pos_result) {
    printf("Error: %s\n", pos_result.ErrorMessage());
    return -1;
}
```

**❌ Don't do this:**
```cpp
auto reached = driver.rampControl.IsTargetReached();
if (reached.Value()) {  // Crashes if reached is an error!
    // ...
}
```

**✅ Do this:**
```cpp
auto reached = driver.rampControl.IsTargetReached();
if (reached && reached.Value()) {  // Check result first!
    // ...
}
```

## Helper Classes

### TMC51x0DaisyChain

High-level manager for multiple TMC51x0 drivers in a SPI daisy-chain configuration.

**Location**: [`inc/features/tmc51x0_daisy_chain.hpp`](../inc/features/tmc51x0_daisy_chain.hpp)

#### Constructor

| Method | Signature | Description |
|--------|-----------|-------------|
| `TMC51x0DaisyChain()` | `TMC51x0DaisyChain(CommType& comm, uint8_t num_onboard_devices, uint32_t f_clk = 12000000)` | Create daisy-chain manager with specified number of onboard devices. **Note**: `f_clk` parameter is optional (defaults to 12 MHz) but the actual clock frequency used is determined from `DriverConfig::external_clk_config` during `Initialize()`. |

#### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `operator[]` | `TMC51x0<CommType>& operator[](size_t index)` | Reference to driver | Access driver at specified position (0-based) |
| `InitializeAll()` | `Result<void> InitializeAll(const DriverConfig& config) noexcept` | `Result<void>` indicating success or error | Initialize all onboard devices with same config |
| `AddDevice()` | `Result<void> AddDevice(size_t position) noexcept` | `Result<void>` indicating success or error | Add extra device at specified position |
| `RemoveDevice()` | `Result<void> RemoveDevice(size_t position) noexcept` | `Result<void>` indicating success or error | Remove extra device at specified position |
| `GetNumDevices()` | `size_t GetNumDevices() const noexcept` | Number of devices | Get total number of devices (onboard + extra) |
| `GetOnboardCount()` | `size_t GetOnboardCount() const noexcept` | Number of onboard devices | Get number of onboard devices |

**Usage:**
```cpp
tmc51x0::TMC51x0DaisyChain<MySPI, 5> chain(spiComm, 3); // f_clk is optional
tmc51x0::DriverConfig cfg{};
cfg.motor_spec.rated_current_ma = 2000;
cfg.motor_spec.sense_resistor_mohm = 50;
cfg.motor_spec.supply_voltage_mv = 24000;
cfg.external_clk_config.frequency_hz = 0; // 0 = use internal 12 MHz clock

auto init_result = chain.InitializeAll(cfg);
if (!init_result) {
    printf("Chain initialization error: %s\n", init_result.ErrorMessage());
    return -1;
}

auto& motor_x = chain[0];
auto pos_result = motor_x.rampControl.SetTargetPosition(1000);
if (!pos_result) {
    printf("Error setting target position: %s\n", pos_result.ErrorMessage());
    return -1;
}
```

### TMC51x0MultiNode

High-level manager for multiple TMC51x0 drivers in a UART multi-node configuration.

**Location**: [`inc/features/tmc51x0_multi_node.hpp`](../inc/features/tmc51x0_multi_node.hpp)

#### Constructor

| Method | Signature | Description |
|--------|-----------|-------------|
| `TMC51x0MultiNode()` | `TMC51x0MultiNode(CommType& comm, uint8_t num_onboard_devices, uint32_t f_clk = 12000000)` | Create multi-node manager with specified number of onboard devices. **Note**: `f_clk` parameter is optional (defaults to 12 MHz) but the actual clock frequency used is determined from `DriverConfig::external_clk_config` during `Initialize()`. |

#### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `operator[]` | `TMC51x0<CommType>& operator[](size_t index)` | Reference to driver | Access driver at specified logical index (0-based) |
| `ProgramSequentially()` | `Result<void> ProgramSequentially(uint8_t send_delay = 2) noexcept` | `Result<void>` indicating success or error | Program all devices sequentially using NAI/NAO pins (required at startup) |
| `ProgramDevice()` | `Result<void> ProgramDevice(uint8_t index, uint8_t send_delay = 2) noexcept` | `Result<void>` indicating success or error | Program single device at specified index (must be accessible at address 0) |
| `InitializeAll()` | `Result<void> InitializeAll(const DriverConfig& config) noexcept` | `Result<void>` indicating success or error | Initialize all onboard devices with same config |
| `AddDevice()` | `Result<void> AddDevice(uint8_t index) noexcept` | `Result<void>` indicating success or error | Add extra device at specified logical index |
| `RemoveDevice()` | `Result<void> RemoveDevice(uint8_t index) noexcept` | `Result<void>` indicating success or error | Remove extra device at specified logical index |
| `GetNumDevices()` | `size_t GetNumDevices() const noexcept` | Number of devices | Get total number of devices (onboard + extra) |
| `GetOnboardCount()` | `size_t GetOnboardCount() const noexcept` | Number of onboard devices | Get number of onboard devices |

**Usage:**
```cpp
tmc51x0::TMC51x0MultiNode<MyUART, 5> nodes(uartComm, 3, 12'000'000);
nodes.ProgramSequentially(); // Required at startup
nodes.InitializeAll(cfg);
auto& motor_x = nodes[0];
motor_x.rampControl.SetTargetPosition(1000);
```

## Next Steps

- See [Examples](examples.md) for usage examples
- Check [Configuration](configuration.md) for configuration options
- Review [Multi-Chip Communication](special_features_multi_chip.md) for daisy-chaining

---

**Navigation**
⬅️ [Configuration](configuration.md) | [Next: Examples ➡️](examples.md) | [Back to Index](index.md)
