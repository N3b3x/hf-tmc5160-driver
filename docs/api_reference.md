---
layout: default
title: "📖 API Reference"
description: "Complete API documentation for the TMC5160 driver"
nav_order: 6
parent: "📚 Documentation"
permalink: /docs/api_reference/
---

# API Reference

Complete reference documentation for all public methods and types in the TMC5160 driver.

## Source Code

- **Main Header**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)
- **Communication Interface**: [`inc/tmc5160_comm_interface.hpp`](../inc/tmc5160_comm_interface.hpp)
- **Registers**: [`inc/tmc5160_registers.hpp`](../inc/tmc5160_registers.hpp)
- **Types**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

## TMC5160 Class

Main driver class for interfacing with the TMC5160 stepper motor controller.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Constructor

| Method | Signature | Description |
|--------|-----------|-------------|
| `TMC5160()` | `TMC5160(CommType& comm, uint32_t f_clk = 12000000)` | Construct driver instance with communication interface and clock frequency |

### Core Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetComm()` | `CommType& GetComm() noexcept` | Reference to comm interface | Get communication interface reference |
| `Initialize()` | `bool Initialize(const DriverConfig& config = DriverConfig()) noexcept` | `true` on success | Initialize driver with configuration |
| `Reset()` | `bool Reset() noexcept` | `true` on success | Perform software reset |
| `IsInitialized()` | `bool IsInitialized() const noexcept` | `true` if initialized | Check initialization status |

## RampControl Subsystem

Ramp control and motion planning subsystem.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `SetRampMode()` | `bool SetRampMode(RampMode mode) noexcept` | `true` on success | Set ramp mode (POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD) |
| `SetTargetPosition()` | `bool SetTargetPosition(int32_t position) noexcept` | `true` on success | Set target position for positioning mode |
| `GetCurrentPosition()` | `int32_t GetCurrentPosition() noexcept` | Position in steps (0 on error) | Get current motor position |
| `SetCurrentPosition()` | `bool SetCurrentPosition(int32_t position, bool update_encoder = false) noexcept` | `true` on success | Set current position (optionally update encoder) |
| `SetMaxSpeed()` | `bool SetMaxSpeed(float speed) noexcept` | `true` on success | Set maximum velocity (steps/s) |
| `SetAcceleration()` | `bool SetAcceleration(float acceleration) noexcept` | `true` on success | Set acceleration/deceleration (steps/s²) |
| `SetAccelerations()` | `bool SetAccelerations(float acceleration, float deceleration) noexcept` | `true` on success | Set acceleration and deceleration separately |
| `SetRampSpeeds()` | `bool SetRampSpeeds(float start_speed, float stop_speed, float transition_speed) noexcept` | `true` on success | Set ramp start, stop, and transition speeds |
| `GetCurrentSpeed()` | `float GetCurrentSpeed() noexcept` | Speed in steps/s (0.0f on error) | Get current motor velocity |
| `IsTargetReached()` | `bool IsTargetReached() noexcept` | `true` if reached | Check if target position reached |
| `IsTargetVelocityReached()` | `bool IsTargetVelocityReached() noexcept` | `true` if reached | Check if target velocity reached |
| `Stop()` | `bool Stop() noexcept` | `true` on success | Stop motor immediately |

## MotorControl Subsystem

Motor control and configuration subsystem.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `Enable()` | `bool Enable() noexcept` | `true` on success | Enable motor driver |
| `Disable()` | `bool Disable() noexcept` | `true` on success | Disable motor driver |
| `SetCurrent()` | `bool SetCurrent(uint8_t irun, uint8_t ihold) noexcept` | `true` on success | Set run current (0-31) and hold current (0-31) |
| `ConfigureChopper()` | `bool ConfigureChopper(const ChopperConfig& config) noexcept` | `true` on success | Configure chopper settings (toff, hstrt, hend, tbl, mres, etc.) |
| `ConfigureStealthChop()` | `bool ConfigureStealthChop(const StealthChopConfig& config) noexcept` | `true` on success | Configure stealthChop PWM mode |
| `SetModeChangeSpeeds()` | `bool SetModeChangeSpeeds(float pwm_thrs, float cool_thrs, float high_thrs) noexcept` | `true` on success | Set velocity thresholds for mode switching |
| `SetGlobalScaler()` | `bool SetGlobalScaler(uint16_t scaler) noexcept` | `true` on success | Set global current scaler (32-256) |

## Encoder Subsystem

Encoder integration and closed-loop control subsystem.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `Configure()` | `bool Configure(const EncoderConfig& config) noexcept` | `true` on success | Configure encoder settings (polarity, filtering, etc.) |
| `GetPosition()` | `int32_t GetPosition() noexcept` | Encoder position (0 on error) | Get encoder position |
| `SetResolution()` | `bool SetResolution(int32_t motor_steps, int32_t enc_resolution, bool inverted = false) noexcept` | `true` on success | Set encoder resolution (motor steps per encoder resolution) |
| `SetAllowedDeviation()` | `bool SetAllowedDeviation(int32_t deviation) noexcept` | `true` on success | Set allowed encoder deviation threshold |
| `IsDeviationDetected()` | `bool IsDeviationDetected() noexcept` | `true` if deviation detected | Check if encoder deviation detected |
| `ClearDeviationFlag()` | `bool ClearDeviationFlag() noexcept` | `true` on success | Clear encoder deviation flag |

## Diagnostics Subsystem

Driver status monitoring and diagnostics subsystem.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetStatus()` | `DriverStatus GetStatus() noexcept` | DriverStatus enum | Get driver status (OK, CP_UV, S2VSA, etc.) |
| `GetStallGuard()` | `uint16_t GetStallGuard() noexcept` | StallGuard value (0-1023) | Get StallGuard2 value |
| `ConfigureStallGuard()` | `bool ConfigureStallGuard(const StallGuardConfig& config) noexcept` | `true` on success | Configure StallGuard2 settings |
| `GetDriverStatusRegister()` | `bool GetDriverStatusRegister(uint32_t& status) noexcept` | `true` on success | Read DRV_STATUS register |
| `GetRampStatusRegister()` | `bool GetRampStatusRegister(uint32_t& status) noexcept` | `true` on success | Read RAMP_STAT register |

## Protection Subsystem

Protection and safety features subsystem.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `ConfigureShortProtection()` | `bool ConfigureShortProtection(const ShortProtectionConfig& config) noexcept` | `true` on success | Configure short circuit protection |
| `SetShortProtectionLevels()` | `bool SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level, uint8_t shortfilter, uint8_t shortdelay) noexcept` | `true` on success | Set short protection levels directly |

## Communication Interface Methods

Methods available through `GetComm()` for direct communication interface access.

**Location**: [`inc/tmc5160_comm_interface.hpp`](../inc/tmc5160_comm_interface.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetMode()` | `CommMode GetMode() const noexcept` | CommMode enum | Get communication mode (SPI or UART) |
| `ReadRegister()` | `bool ReadRegister(uint8_t address, uint32_t& value) noexcept` | `true` on success | Read 32-bit register |
| `WriteRegister()` | `bool WriteRegister(uint8_t address, uint32_t value) noexcept` | `true` on success | Write 32-bit register |
| `GpioSet()` | `bool GpioSet(TMC5160CtrlPin pin, GpioSignal signal) noexcept` | `true` on success | Set GPIO pin state |
| `GpioRead()` | `bool GpioRead(TMC5160CtrlPin pin, GpioSignal& signal) noexcept` | `true` on success | Read GPIO pin state |
| `GpioSetActive()` | `bool GpioSetActive(TMC5160CtrlPin pin) noexcept` | `true` on success | Set GPIO pin to active state |
| `GpioSetInactive()` | `bool GpioSetInactive(TMC5160CtrlPin pin) noexcept` | `true` on success | Set GPIO pin to inactive state |
| `SignalToGpioLevel()` | `bool SignalToGpioLevel(TMC5160CtrlPin pin, GpioSignal signal) const noexcept` | GPIO level | Convert signal to physical GPIO level |
| `GpioLevelToSignal()` | `GpioSignal GpioLevelToSignal(TMC5160CtrlPin pin, bool gpio_level) const noexcept` | GpioSignal enum | Convert GPIO level to signal |
| `SetPinActiveLevel()` | `bool SetPinActiveLevel(TMC5160CtrlPin pin, bool active_level) noexcept` | `true` on success | Configure pin active level |
| `DelayMs()` | `void DelayMs(uint32_t ms) noexcept` | void | Delay milliseconds |
| `DelayUs()` | `void DelayUs(uint32_t us) noexcept` | void | Delay microseconds |
| `LogDebug()` | `void LogDebug(int level, const char* tag, const char* format, ...) noexcept` | void | Debug logging |

## Enums

### RampMode

| Value | Description |
|-------|-------------|
| `POSITIONING` | Positioning mode - move to target position |
| `VELOCITY_POS` | Velocity mode - positive direction |
| `VELOCITY_NEG` | Velocity mode - negative direction |
| `HOLD` | Hold mode - maintain current position |

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

### CommMode

| Value | Description |
|-------|-------------|
| `SPI` | SPI communication mode |
| `UART` | UART communication mode |

**Location**: [`inc/tmc5160_comm_interface.hpp`](../inc/tmc5160_comm_interface.hpp)

### TMC5160CtrlPin

| Value | Description |
|-------|-------------|
| `EN` | Enable pin |
| `DIR` | Direction pin |
| `STEP` | Step pin |

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

### GpioSignal

| Value | Description |
|-------|-------------|
| `ACTIVE` | Active signal state |
| `INACTIVE` | Inactive signal state |

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

## Configuration Structures

### DriverConfig

Main driver configuration structure.

**Location**: [`inc/tmc5160_config.hpp`](../inc/tmc5160_config.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `motor` | `MotorConfig` | Motor current and scaler settings |
| `chopper` | `ChopperConfig` | Chopper timing and microstep settings |
| `stealthchop` | `StealthChopConfig` | StealthChop PWM configuration |
| `power_stage` | `PowerStageConfig` | Power stage driver strength and blanking |
| `short_protection` | `ShortProtectionConfig` | Short circuit protection levels |

### ChopperConfig

Chopper configuration structure.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `toff` | `uint8_t` | 0-15 | Chopper off time |
| `hstrt` | `uint8_t` | 0-7 | Chopper hysteresis start |
| `hend` | `uint8_t` | 0-15 | Chopper hysteresis end |
| `tbl` | `uint8_t` | 0-3 | Blanking time |
| `vsense` | `bool` | - | Voltage sense mode |
| `mres` | `uint8_t` | 0-8 | Microstep resolution |
| `intpol` | `bool` | - | Interpolation enable |
| `dedge` | `bool` | - | Double edge step |

### StealthChopConfig

StealthChop PWM configuration structure.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pwm_ofs` | `uint8_t` | 0-255 | PWM offset |
| `pwm_grad` | `uint8_t` | 0-255 | PWM gradient |
| `pwm_freq` | `uint8_t` | 0-3 | PWM frequency |
| `pwm_autoscale` | `bool` | - | Auto-scale PWM amplitude |
| `pwm_autograd` | `bool` | - | Auto-scale PWM gradient |
| `pwm_reg` | `uint8_t` | 0-15 | PWM register |
| `pwm_lim` | `uint8_t` | 0-15 | PWM limit |

### EncoderConfig

Encoder configuration structure.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `pol_a` | `bool` | Encoder A polarity |
| `pol_b` | `bool` | Encoder B polarity |
| `pol_n` | `bool` | Encoder N polarity |
| `ignore_ab` | `bool` | Ignore A/B signals |
| `clr_cont` | `bool` | Clear continuously |
| `clr_once` | `bool` | Clear once |
| `sensitivity` | `uint8_t` | Sensitivity (0-3) |
| `clr_enc_x` | `bool` | Clear encoder on XACTUAL |
| `latch_x_act` | `bool` | Latch XACTUAL |

### StallGuardConfig

StallGuard2 configuration structure.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `semin` | `uint8_t` | 0-15 | Minimum StallGuard value |
| `semax` | `uint8_t` | 0-15 | Maximum StallGuard value |
| `seup` | `uint8_t` | 0-3 | StallGuard up step |
| `sedn` | `uint8_t` | 0-3 | StallGuard down step |
| `seimin` | `bool` | - | Minimum current |
| `sfilt` | `bool` | - | StallGuard filter |
| `sgt` | `int8_t` | -64 to 63 | StallGuard threshold |

### ShortProtectionConfig

Short circuit protection configuration structure.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `s2vs_level` | `uint8_t` | 4-15 | Short to VS level |
| `s2g_level` | `uint8_t` | 2-15 | Short to GND level |
| `shortfilter` | `uint8_t` | 0-3 | Short filter time |
| `shortdelay` | `uint8_t` | 0-1 | Short delay |

## Error Handling

The driver uses boolean return values for error handling:
- `true` indicates success
- `false` indicates failure

Always check return values:

```cpp
if (!driver.rampControl.SetTargetPosition(1000)) {
    // Handle error
}
```

## Next Steps

- See [Examples](examples.md) for usage examples
- Check [Configuration](configuration.md) for configuration options

---

**Navigation**
⬅️ [Configuration](configuration.md) | [Next: Examples ➡️](examples.md) | [Back to Index](index.md)
