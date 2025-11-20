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
- **Unit Conversions**: [`inc/tmc5160_units.hpp`](../inc/tmc5160_units.hpp)

## TMC5160 Class

Main driver class for interfacing with the TMC5160 stepper motor controller.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Constructor

| Method | Signature | Description |
|--------|-----------|-------------|
| `TMC5160()` | `TMC5160(CommType& comm, uint32_t f_clk = 12000000, uint8_t daisy_chain_position = 0, uint8_t uart_node_address = 0)` | Construct driver instance. For SPI: daisy_chain_position. For UART: uart_node_address. |

### Core Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetComm()` | `CommType& GetComm() noexcept` | Reference to comm interface | Get communication interface reference |
| `SetDaisyChainPosition()` | `void SetDaisyChainPosition(uint8_t position) noexcept` | void | Set daisy-chain position for SPI (0 = first chip) |
| `GetDaisyChainPosition()` | `uint8_t GetDaisyChainPosition() const noexcept` | Position (0-255) | Get current daisy-chain position for SPI |
| `SetUartNodeAddress()` | `void SetUartNodeAddress(uint8_t address) noexcept` | void | Set UART node address (0-254) for UART multi-node |
| `GetUartNodeAddress()` | `uint8_t GetUartNodeAddress() const noexcept` | Address (0-254) | Get current UART node address |

**Note**: Per datasheet procedure, devices are programmed backwards from address 254 (254, 253, 252, ...). Logical device indices (0, 1, 2, ...) map to physical addresses (254, 253, 252, ...). Use `TMC5160MultiNode` class for managing multiple devices.
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
| `SetTargetPositionMm()` | `bool SetTargetPositionMm(float position_mm, uint16_t steps_per_rev, float lead_screw_pitch_mm) noexcept` | `true` on success | Set target position in millimeters |
| `SetMaxSpeedRpm()` | `bool SetMaxSpeedRpm(float rpm, uint16_t steps_per_rev) noexcept` | `true` on success | Set maximum speed in RPM |
| `ConfigureReferenceSwitch()` | `bool ConfigureReferenceSwitch(const ReferenceSwitchConfig& config) noexcept` | `true` on success | Configure reference switches/endstops |
| `GetLatchedPosition()` | `int32_t GetLatchedPosition() noexcept` | Latched position (0 on error) | Get position latched on switch event |
| `SetComparePosition()` | `bool SetComparePosition(int32_t position) noexcept` | `true` on success | Set position comparison register (X_COMPARE) |

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
| `SetFreewheelingMode()` | `bool SetFreewheelingMode(PWMFreewheel mode) noexcept` | `true` on success | Set freewheeling mode (NORMAL, ENABLED, SHORT_LS, SHORT_HS) |
| `ConfigureCoolStep()` | `bool ConfigureCoolStep(const CoolStepConfig& config) noexcept` | `true` on success | Configure CoolStep current reduction |
| `ConfigureDcStep()` | `bool ConfigureDcStep(const DcStepConfig& config) noexcept` | `true` on success | Configure dcStep automatic commutation |
| `SetMicrostepLookupTable()` | `bool SetMicrostepLookupTable(uint8_t index, uint32_t value) noexcept` | `true` on success | Set microstep lookup table entry (0-7) |
| `SetMicrostepLookupTableSegmentation()` | `bool SetMicrostepLookupTableSegmentation(uint8_t width_sel_0, uint8_t width_sel_1, uint8_t width_sel_2, uint8_t width_sel_3, uint8_t lut_seg_start1, uint8_t lut_seg_start2, uint8_t lut_seg_start3) noexcept` | `true` on success | Set microstep lookup table segmentation |
| `SetMicrostepLookupTableStart()` | `bool SetMicrostepLookupTableStart(uint16_t start_current) noexcept` | `true` on success | Set microstep lookup table start current |
| `SetupMotorFromSpec()` | `bool SetupMotorFromSpec(const MotorSpec& motor_spec, const MechanicalSystem* mechanical_system = nullptr) noexcept` | `true` on success | Setup motor from high-level specifications |

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
| `GetLostSteps()` | `uint32_t GetLostSteps() noexcept` | Lost steps count (0 on error) | Get lost steps counter (dcStep mode only) |
| `PerformSensorlessHoming()` | `bool PerformSensorlessHoming(bool direction, int8_t stall_threshold, float search_speed, int32_t& final_position) noexcept` | `true` on success | Perform sensorless homing using StallGuard2 |

## Communication Subsystem

UART slave addressing and multi-chip communication configuration.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `ConfigureSlaveAddress()` | `bool ConfigureSlaveAddress(uint8_t slave_address, uint8_t send_delay = 0) noexcept` | `true` on success | Configure UART slave address and send delay (deprecated, use `uartConfig.ConfigureSlave()` instead) |
| `GetSlaveAddress()` | `uint8_t GetSlaveAddress() noexcept` | Slave address (0-127) or 0xFF on error | Get current slave address from SLAVECONF register |
| `GetSendDelay()` | `uint8_t GetSendDelay() noexcept` | Send delay (0-15) or 0xFF on error | Get current send delay from SLAVECONF register |

## UartConfig Subsystem

UART configuration subsystem for multi-node addressing.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `ConfigureSlave()` | `bool ConfigureSlave(uint8_t slave_address, uint8_t send_delay) noexcept` | `true` on success | Configure SLAVECONF register with node address (0-254) and send delay. Updates the driver's `uart_node_address_`. Per datasheet, devices are typically programmed backwards from 254. |

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
| `ReadRegister()` | `bool ReadRegister(uint8_t address, uint32_t& value, uint8_t daisy_chain_position = 0) noexcept` | `true` on success | Read 32-bit register. For SPI: daisy-chain position. For UART: node address. |
| `WriteRegister()` | `bool WriteRegister(uint8_t address, uint32_t value, uint8_t daisy_chain_position = 0) noexcept` | `true` on success | Write 32-bit register. For SPI: daisy-chain position. For UART: node address. |
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

## TMC5160 Class Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `SetDaisyChainPosition()` | `void SetDaisyChainPosition(uint8_t position) noexcept` | void | Set daisy-chain position for this TMC5160 instance |
| `GetDaisyChainPosition()` | `uint8_t GetDaisyChainPosition() const noexcept` | Position (0-255) | Get current daisy-chain position |

**Note**: Each `TMC5160` instance tracks its own daisy-chain position. This position is automatically passed to `ReadRegister()` and `WriteRegister()` methods in the communication interface.

## SpiCommInterface Methods (SPI Only)

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `SetDaisyChainLength()` | `void SetDaisyChainLength(uint8_t total_length) noexcept` | void | Set total number of devices in daisy chain (for proper response extraction) |
| `GetDaisyChainLength()` | `uint8_t GetDaisyChainLength() const noexcept` | Chain length (0-255) | Get current daisy chain length setting |
| `AutoDetectChainLength()` | `uint8_t AutoDetectChainLength(uint8_t max_devices = 8) noexcept` | Detected length (0-255) | Auto-detect daisy chain length by sending command that loops back |

**Note**: `SetDaisyChainLength()` is critical for proper response extraction using the datasheet formula `40·(n-k+1)`. The chain length represents the total number of devices in the daisy chain, not individual device positions.
| `SetNaiPin()` | `bool SetNaiPin(bool active) noexcept` | `true` on success | Set NAI pin state for UART sequential addressing (UART only) |
| `GetNaoPin()` | `bool GetNaoPin(bool& active) noexcept` | `true` on success | Read NAO pin state for UART sequential addressing (UART only) |

**Note**: `UartCommInterface` no longer stores node addresses. Multiple `TMC5160` instances share one `UartCommInterface` on the same UART bus. Each `TMC5160` instance stores its own `uart_node_address_` and passes it to `ReadRegister()`/`WriteRegister()` automatically.

**UART Addressing**: Per datasheet procedure (Figure 5.1), devices are programmed backwards from address 254 (254, 253, 252, ...). Logical device indices (0, 1, 2, ...) used in software correspond to physical addresses (254, 253, 252, ...) programmed into chips. Use `TMC5160MultiNode` class for managing multiple devices with sequential programming.

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

### MotorSpec

Motor specification structure for high-level setup.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `steps_per_rev` | `uint16_t` | Steps per revolution (typically 200 for 1.8° motors) |
| `rated_current_ma` | `uint16_t` | Rated motor current in milliamps |
| `rated_voltage_mv` | `uint16_t` | Rated motor voltage in millivolts |
| `winding_resistance_mohm` | `uint32_t` | Winding resistance in milliohms (optional, 0 = not specified) |
| `winding_inductance_uh` | `uint32_t` | Winding inductance in microhenries (optional, 0 = not specified) |
| `holding_torque_mnm` | `uint32_t` | Holding torque in milliNewton-meters (optional, 0 = not specified) |

### MechanicalSystem

Mechanical system configuration for unit conversions.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `system_type` | `MechanicalSystemType` | System type (DirectDrive, LeadScrew, BeltDrive, Gearbox) |
| `lead_screw_pitch_mm` | `float` | Lead screw pitch in mm (for LeadScrew) |
| `belt_pulley_teeth` | `uint16_t` | Number of teeth on motor pulley (for BeltDrive) |
| `belt_pitch_mm` | `float` | Belt pitch in mm (for BeltDrive) |
| `gear_ratio` | `float` | Gear ratio (output/input, for Gearbox) |

### CoolStepConfig

CoolStep current reduction configuration.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `semin` | `uint8_t` | 0-15 | Minimum StallGuard2 value for CoolStep |
| `semax` | `uint8_t` | 0-15 | StallGuard2 hysteresis value |
| `seup` | `uint8_t` | 0-3 | Current increment step width |
| `sedn` | `uint8_t` | 0-3 | Current decrement step speed |
| `seimin` | `bool` | - | Minimum current for smart current control |
| `sfilt` | `bool` | - | Enable StallGuard2 filter |

### ReferenceSwitchConfig

Reference switch/endstop configuration.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `stop_left_enable` | `bool` | Enable automatic motor stop on left switch |
| `stop_right_enable` | `bool` | Enable automatic motor stop on right switch |
| `pol_stop_left` | `bool` | Left switch polarity (true=inverted/low active) |
| `pol_stop_right` | `bool` | Right switch polarity (true=inverted/low active) |
| `swap_left_right` | `bool` | Swap left and right switch inputs |
| `latch_left_active` | `bool` | Latch position on active edge of left switch |
| `latch_left_inactive` | `bool` | Latch position on inactive edge of left switch |
| `latch_right_active` | `bool` | Latch position on active edge of right switch |
| `latch_right_inactive` | `bool` | Latch position on inactive edge of right switch |
| `en_latch_encoder` | `bool` | Latch encoder position on switch event |
| `en_softstop` | `bool` | Enable soft stop using deceleration ramp |

### DcStepConfig

dcStep automatic commutation configuration.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `vdc_min` | `float` | Velocity threshold for enabling dcStep in steps/s (0.0f = disabled) |

## Unit Conversion Functions

Free functions for converting between physical units and driver steps.

**Location**: [`inc/tmc5160_units.hpp`](../inc/tmc5160_units.hpp)

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

The TMC5160 driver provides comprehensive coverage of all chipset features. This section summarizes the implementation status and available features.

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
- `faststandstill` - Standstill detection timeout
- `multistep_filt` - Step input filtering
- `diag0_error` - DIAG0 on driver errors
- `diag0_otpw` - DIAG0 on overtemperature prewarning
- `diag0_stall_step` - DIAG0 on stall/STEP output
- `diag1_stall_dir` - DIAG1 on stall/DIR output
- `diag1_index` - DIAG1 on index position
- `diag1_onstate` - DIAG1 when chopper on
- `diag1_steps_skipped` - DIAG1 on skipped steps
- `diag0_int_pushpull` - DIAG0 push-pull output
- `diag1_poscomp_pushpull` - DIAG1 push-pull output
- `small_hysteresis` - Small hysteresis for step frequency
- `stop_enable` - Emergency stop enable
- `direct_mode` - Direct motor coil control
- `test_mode` - Test mode

**Usage:**
```cpp
tmc5160::GlobalConfig gconf{};
gconf.faststandstill = true;
gconf.diag0_error = true;
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
tmc5160::DcStepConfig dc_config{};
dc_config.dc_time = 100;
dc_config.dc_sg = 50;
driver.motorControl.ConfigureDcStep(dc_config);
```

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
- Review [Multi-Chip Communication](special_features_multi_chip.md) for daisy-chaining

---

**Navigation**
⬅️ [Configuration](configuration.md) | [Next: Examples ➡️](examples.md) | [Back to Index](index.md)
