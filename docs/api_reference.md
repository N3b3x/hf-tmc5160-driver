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
| `TMC5160()` | `TMC5160(CommType& comm, uint32_t f_clk = 12000000, uint8_t daisy_chain_position = 0, uint8_t uart_node_address = 0)` | Construct driver instance. For SPI: use `daisy_chain_position` (0 = first chip). For UART: use `uart_node_address` (0-254). |

### Core Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `GetComm()` | `CommType& GetComm() noexcept` | Reference to comm interface | Get communication interface reference |
| `SetDaisyChainPosition()` | `void SetDaisyChainPosition(uint8_t position) noexcept` | void | Set daisy-chain position for SPI (0 = first chip) |
| `GetDaisyChainPosition()` | `uint8_t GetDaisyChainPosition() const noexcept` | Position (0-255) | Get current daisy-chain position for SPI |
| `SetUartNodeAddress()` | `void SetUartNodeAddress(uint8_t address) noexcept` | void | Set UART node address (0-254) for UART multi-node |
| `GetUartNodeAddress()` | `uint8_t GetUartNodeAddress() const noexcept` | Address (0-254) | Get current UART node address |
| `SetChipCommMode()` | `bool SetChipCommMode(ChipCommMode mode) noexcept` | `true` on success | Set chip communication mode via SPI_MODE and SD_MODE pins (if available as GPIO) |
| `GetChipCommMode()` | `bool GetChipCommMode(ChipCommMode& mode) const noexcept` | `true` on success | Get current chip communication mode from SPI_MODE and SD_MODE pins |

**Note**: Per datasheet procedure, devices are programmed backwards from address 254 (254, 253, 252, ...). Logical device indices (0, 1, 2, ...) map to physical addresses (254, 253, 252, ...). Use `TMC5160MultiNode` class for managing multiple devices.

**⚠️ Chip Communication Mode Control:**
- `SetChipCommMode()` and `GetChipCommMode()` control SPI_MODE (pin 22) and SD_MODE (pin 21) pins
- These pins are **typically hardwired** and read at startup
- Only use these methods if SPI_MODE and SD_MODE are connected to GPIO outputs
- Configure pins in `TMC5160PinConfig` (spi_mode_pin, sd_mode_pin) before use
- **CRITICAL**: Mode changes require a chip reset (power cycle or reset pin) to take effect
- The mode pins are read at startup, so changes won't be effective until reset

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
| `ConfigureReferenceSwitch()` | `bool ConfigureReferenceSwitch(const ReferenceSwitchConfig& config) noexcept` | `true` on success | Configure reference switches/endstops (full configuration) |
| `GetReferenceSwitchConfig()` | `bool GetReferenceSwitchConfig(ReferenceSwitchConfig& config) noexcept` | `true` on success | Read current reference switch configuration |
| `SetLeftSwitchActiveLevel()` | `bool SetLeftSwitchActiveLevel(ReferenceSwitchActiveLevel) noexcept` | `true` on success | Set left switch active level (real-time update) |
| `SetRightSwitchActiveLevel()` | `bool SetRightSwitchActiveLevel(ReferenceSwitchActiveLevel) noexcept` | `true` on success | Set right switch active level (real-time update) |
| `SetLeftSwitchStopEnable()` | `bool SetLeftSwitchStopEnable(bool enable) noexcept` | `true` on success | Enable/disable motor stop on left switch (real-time) |
| `SetRightSwitchStopEnable()` | `bool SetRightSwitchStopEnable(bool enable) noexcept` | `true` on success | Enable/disable motor stop on right switch (real-time) |
| `SetLeftSwitchLatchMode()` | `bool SetLeftSwitchLatchMode(ReferenceLatchMode) noexcept` | `true` on success | Set left switch latching mode (real-time update) |
| `SetRightSwitchLatchMode()` | `bool SetRightSwitchLatchMode(ReferenceLatchMode) noexcept` | `true` on success | Set right switch latching mode (real-time update) |
| `SetStopMode()` | `bool SetStopMode(ReferenceStopMode) noexcept` | `true` on success | Set stop mode (hard/soft) (real-time update) |
| `GetLatchedPosition()` | `float GetLatchedPosition(Unit unit) noexcept` | Latched position (0 on error) | Get position latched on switch event |
| `SetComparePosition()` | `bool SetComparePosition(int32_t position) noexcept` | `true` on success | Set position comparison register (X_COMPARE) |
| `SetPowerDownDelay()` | `bool SetPowerDownDelay(uint8_t tpowerdown) noexcept` | `true` on success | Set power down delay (0-255) |
| `SetZeroWaitTime()` | `bool SetZeroWaitTime(uint16_t tzerowait) noexcept` | `true` on success | Set zero wait time after ramping down (0-65535) |
| `SetFirstAcceleration()` | `bool SetFirstAcceleration(float a1) noexcept` | `true` on success | Set first acceleration phase A1 (steps/s², 0.0f = use AMAX) |

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
| `ConfigureGlobalConfig()` | `bool ConfigureGlobalConfig(const GlobalConfig& config) noexcept` | `true` on success | Configure global configuration (GCONF register) |

## Encoder Subsystem

Encoder integration and closed-loop control subsystem.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `Configure()` | `bool Configure(const EncoderConfig& config) noexcept` | `true` on success | Configure encoder settings (polarity, filtering, etc.) |
| `GetEncoderConfig()` | `bool GetEncoderConfig(EncoderConfig& config) noexcept` | `true` on success | Read current encoder configuration |
| `SetNChannelActiveLevel()` | `bool SetNChannelActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept` | `true` on success | Set N channel active level (real-time) |
| `SetNChannelSensitivity()` | `bool SetNChannelSensitivity(EncoderNSensitivity sensitivity) noexcept` | `true` on success | Set N channel sensitivity (real-time) |
| `SetClearMode()` | `bool SetClearMode(EncoderClearMode clear_mode) noexcept` | `true` on success | Set encoder clear mode (real-time) |
| `SetPrescalerMode()` | `bool SetPrescalerMode(EncoderPrescalerMode prescaler_mode) noexcept` | `true` on success | Set encoder prescaler mode (real-time) |
| `GetPosition()` | `int32_t GetPosition() noexcept` | Encoder position (0 on error) | Get encoder position |
| `SetResolution()` | `bool SetResolution(int32_t motor_steps, int32_t enc_resolution, bool inverted = false) noexcept` | `true` on success | Set encoder resolution (motor steps per encoder resolution) |
| `SetAllowedDeviation()` | `bool SetAllowedDeviation(int32_t deviation) noexcept` | `true` on success | Set allowed encoder deviation threshold |
| `IsDeviationDetected()` | `bool IsDeviationDetected() noexcept` | `true` if deviation detected | Check if encoder deviation detected |
| `ClearDeviationFlag()` | `bool ClearDeviationFlag() noexcept` | `true` on success | Clear encoder deviation flag |
| `GetLatchedPosition()` | `int32_t GetLatchedPosition() noexcept` | Encoder latched position (0 on error) | Get encoder position latched on N event |

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
| `IsOpenLoadA()` | `bool IsOpenLoadA() noexcept` | `true` if open load on phase A | Check for open load on phase A (requires SpreadCycle mode and motion) |
| `IsOpenLoadB()` | `bool IsOpenLoadB() noexcept` | `true` if open load on phase B | Check for open load on phase B (requires SpreadCycle mode and motion) |
| `CheckOpenLoad()` | `bool CheckOpenLoad(bool& phase_a, bool& phase_b) noexcept` | `true` on success | Check both phases for open load simultaneously |
| `GetRampStatusRegister()` | `bool GetRampStatusRegister(uint32_t& status) noexcept` | `true` on success | Read RAMP_STAT register |
| `GetLostSteps()` | `uint32_t GetLostSteps() noexcept` | Lost steps count (0 on error) | Get lost steps counter (dcStep mode only) |
| `PerformSensorlessHoming()` | `bool PerformSensorlessHoming(bool direction, int8_t stall_threshold, float search_speed, int32_t& final_position) noexcept` | `true` on success | Perform sensorless homing using StallGuard2 |
| `GetTimeBetweenMicrosteps()` | `uint32_t GetTimeBetweenMicrosteps() noexcept` | Time in clock cycles (0 on error) | Get actual time between microsteps (TSTEP register) |
| `GetMicrostepCounter()` | `uint16_t GetMicrostepCounter() noexcept` | Position in table (0-1023, 0 on error) | Get actual position in microstep table (MSCNT register) |
| `GetMicrostepCurrent()` | `bool GetMicrostepCurrent(int16_t& phase_a, int16_t& phase_b) noexcept` | `true` on success | Get actual microstep current for both phases (MSCURACT register) |
| `GetPwmScale()` | `bool GetPwmScale(uint8_t& pwm_scale_sum, int16_t& pwm_scale_auto) noexcept` | `true` on success | Get stealthChop PWM scale results (PWM_SCALE register) |
| `GetPwmAuto()` | `bool GetPwmAuto(uint8_t& pwm_ofs_auto, uint8_t& pwm_grad_auto) noexcept` | `true` on success | Get automatically determined PWM values (PWM_AUTO register) |
| `ReadGpioPins()` | `bool ReadGpioPins(uint32_t& io_pins) noexcept` | `true` on success | Read GPIO input pin states (IO_INPUT_OUTPUT register) |
| `ReadFactoryConfig()` | `bool ReadFactoryConfig(uint8_t& fclktrim) noexcept` | `true` on success | Read factory configuration/clock trim (FACTORY_CONF register) |
| `ReadOtpConfig()` | `bool ReadOtpConfig(uint8_t& otp_fclktrim, bool& otp_s2_level, bool& otp_bbm, bool& otp_tbl) noexcept` | `true` on success | Read OTP configuration memory (OTP_READ register) |
| `GetUartTransmissionCount()` | `uint8_t GetUartTransmissionCount() noexcept` | Transmission count (0 on error) | Get UART transmission counter (IFCNT register) |
| `ReadOffsetCalibration()` | `bool ReadOffsetCalibration(uint8_t& phase_a, uint8_t& phase_b) noexcept` | `true` on success | Read offset calibration results (OFFSET_READ register) |

### Open Load Diagnostics

The TMC5160 can detect open load conditions (interrupted cables, loose connectors) by checking if it can reach the desired motor coil current. This is useful for system debugging and detecting wiring issues.

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
tmc5160::GlobalConfig gconf{};
driver.motorControl.GetGlobalConfig(gconf);
if (gconf.en_pwm_mode) {
  gconf.en_pwm_mode = false;  // Disable StealthChop
  driver.motorControl.ConfigureGlobalConfig(gconf);
}

// Move motor at low/nominal velocity (minimum 4× microstep resolution)
driver.rampControl.SetMaxSpeed(1000.0f, tmc5160::Unit::Steps);  // Low velocity
driver.rampControl.SetTargetPosition(1024);  // At least 4× microstep resolution (256)

// Wait for motion to start
while (!driver.rampControl.IsTargetReached()) {
  // Check for open load during motion
  bool phase_a_open = driver.diagnostics.IsOpenLoadA();
  bool phase_b_open = driver.diagnostics.IsOpenLoadB();
  
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
| `ConfigureSlave()` | `bool ConfigureSlave(uint8_t slave_address, uint8_t send_delay) noexcept` | `true` on success | Configure SLAVECONF register with node address (0-127) and send delay. Updates the driver's `uart_node_address_`. Per datasheet, devices are typically programmed backwards from 254. |

## Protection Subsystem

Protection and safety features subsystem.

**Location**: [`inc/tmc5160.hpp`](../inc/tmc5160.hpp)

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `ConfigureShortProtection()` | `bool ConfigureShortProtection(const PowerStageParameters& config) noexcept` | `true` on success | Configure short circuit protection using user-friendly voltage thresholds |
| `SetShortProtectionLevels()` | `bool SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level, uint8_t shortfilter, uint8_t shortdelay) noexcept` | `true` on success | Set short protection levels directly (low-level API, register values) |

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

### ChipCommMode

Chip communication and motion control mode configuration. Represents the combination of SPI_MODE (pin 22) and SD_MODE (pin 21) pins.

| Value | Description | SPI_MODE | SD_MODE |
|-------|-------------|----------|---------|
| `SPI_INTERNAL_RAMP` | SPI interface with internal ramp generator (motion controller) | HIGH | LOW |
| `SPI_EXTERNAL_STEPDIR` | SPI interface with external step/dir inputs | HIGH | HIGH |
| `UART_INTERNAL_RAMP` | UART interface with internal ramp generator (motion controller) | LOW | LOW |

**⚠️ WARNING**: 
- These pins are **typically hardwired** and read at startup
- Only use `SetChipCommMode()` if SPI_MODE and SD_MODE are connected to GPIO outputs
- Mode changes require a chip reset (power cycle or reset pin) to take effect
- The mode pins are read at startup, so changes won't be effective until reset

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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

Main driver configuration structure containing all parameters for initializing the TMC5160 driver.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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
| `f_clk` | `uint32_t` | Clock frequency in Hz (default: 12000000) |

**Important Notes**:
- **DO NOT** manually set IRUN, IHOLD, or GLOBAL_SCALER - these are calculated automatically from `motor_spec`
- **REQUIRED** for automatic calculation: `motor_spec.sense_resistor_mohm` and `motor_spec.supply_voltage_mv` must be non-zero
- Current settings are calculated during `Initialize()` and stored internally (not in `motor_spec`)
- See [`configuration.md`](../docs/configuration.md) for detailed configuration guide

### ChopperConfig

User-friendly configuration for SpreadCycle and Classic chopper modes.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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
| `vsense` | `bool` | - | Voltage sensitivity (deprecated, ignored by hardware) |

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

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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
tmc5160::EncoderConfig enc_cfg{};
enc_cfg.n_channel_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_HIGH;
enc_cfg.n_sensitivity = tmc5160::EncoderNSensitivity::RISING_EDGE;
enc_cfg.ignore_ab_polarity = true;
enc_cfg.clear_mode = tmc5160::EncoderClearMode::ONCE;
enc_cfg.prescaler_mode = tmc5160::EncoderPrescalerMode::BINARY;
driver.encoder.Configure(enc_cfg);

// Real-time configuration updates
driver.encoder.SetNChannelSensitivity(tmc5160::EncoderNSensitivity::BOTH_EDGES);
driver.encoder.SetClearMode(tmc5160::EncoderClearMode::CONTINUOUS);

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

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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
- `GetStallGuard()` method to read SG_RESULT value
- `EnableStopOnStall()` method for real-time control

### PowerStageParameters (Short Protection Fields)

Short circuit protection is configured through the `PowerStageParameters` structure using user-friendly voltage thresholds and timing values. The driver automatically converts these to register values based on datasheet specifications.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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

User-friendly configuration for CoolStep automatic current reduction feature.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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
tmc5160::ReferenceSwitchConfig config{};
config.left_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;   // Active LOW (determines polarity)
config.left_switch_stop_enable = true;                                         // Enable motor stop
config.right_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_HIGH; // Active HIGH (determines polarity)
config.right_switch_stop_enable = false;                                       // Don't stop motor (but can still latch/read)
config.stop_mode = tmc5160::ReferenceStopMode::SOFT_STOP;
config.latch_left = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;   // Latch on active edge
config.latch_right = tmc5160::ReferenceLatchMode::BOTH_EDGES;   // Latch on both edges
driver.rampControl.ConfigureReferenceSwitch(config);

// Example: Switch configured but doesn't stop motor (only latches/reads position)
// Useful for reading switch state without stopping motor
tmc5160::ReferenceSwitchConfig config_latch_only{};
config_latch_only.left_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;  // Must specify active level
config_latch_only.left_switch_stop_enable = false;  // Don't stop motor (but polarity is configured for reading)
config_latch_only.latch_left = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;  // But latch position
config_latch_only.latch_right = tmc5160::ReferenceLatchMode::DISABLED;  // No latching on right
driver.rampControl.ConfigureReferenceSwitch(config_latch_only);

// Example: Real-time updates using convenience methods
// Initial configuration
tmc5160::ReferenceSwitchConfig config{};
config.left_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;
config.left_switch_stop_enable = true;
config.latch_left = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;
driver.rampControl.ConfigureReferenceSwitch(config);

// Later: Real-time updates using convenience methods (preserves other settings)
driver.rampControl.SetLeftSwitchStopEnable(false);  // Disable stop without changing polarity
driver.rampControl.SetLeftSwitchLatchMode(tmc5160::ReferenceLatchMode::BOTH_EDGES);  // Change latch mode
driver.rampControl.SetStopMode(tmc5160::ReferenceStopMode::HARD_STOP);  // Change stop mode
driver.rampControl.SetLeftSwitchActiveLevel(tmc5160::ReferenceSwitchActiveLevel::ACTIVE_HIGH);  // Change polarity

// Read current configuration
tmc5160::ReferenceSwitchConfig current_config{};
if (driver.rampControl.GetReferenceSwitchConfig(current_config)) {
  // Use current_config for inspection or modification
}

// For homing (hard stop, precise)
tmc5160::ReferenceSwitchConfig homing_config{};
homing_config.left_switch_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_LOW;
homing_config.left_switch_stop_enable = true;  // Enable stop for homing
homing_config.latch_left = tmc5160::ReferenceLatchMode::ACTIVE_EDGE;  // Latch on active edge
homing_config.stop_mode = tmc5160::ReferenceStopMode::HARD_STOP;  // Hard stop for precise homing
driver.rampControl.ConfigureReferenceSwitch(homing_config);
```

### DcStepConfig

User-friendly configuration for DcStep automatic commutation mode.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

| Field | Type | Description |
|-------|------|-------------|
| `recalibrate` | `bool` | Zero crossing recalibration during driver disable |
| `faststandstill` | `bool` | Standstill detection timeout (true=2^18 clocks, false=2^20 clocks) |
| `en_pwm_mode` | `bool` | Enable StealthChop voltage PWM mode |
| `multistep_filt` | `bool` | Enable step input filtering for StealthChop optimization |
| `shaft` | `bool` | Inverse motor direction |
| `diag0_error` | `bool` | Enable DIAG0 on driver errors (SD_MODE=1 only) |
| `diag0_otpw` | `bool` | Enable DIAG0 on overtemperature prewarning (SD_MODE=1 only) |
| `diag0_stall_step` | `bool` | DIAG0 on stall/STEP output |
| `diag1_stall_dir` | `bool` | DIAG1 on stall/DIR output |
| `diag1_index` | `bool` | Enable DIAG1 on index position (SD_MODE=1 only) |
| `diag1_onstate` | `bool` | Enable DIAG1 when chopper is on (SD_MODE=1 only) |
| `diag1_steps_skipped` | `bool` | Enable DIAG1 on skipped steps in dcStep mode (SD_MODE=1 only) |
| `diag0_int_pushpull` | `bool` | DIAG0 push-pull output mode |
| `diag1_poscomp_pushpull` | `bool` | DIAG1 push-pull output mode |
| `small_hysteresis` | `bool` | Small hysteresis for step frequency comparison |
| `stop_enable` | `bool` | Emergency stop enable (ENCA_DCIN stops sequencer) |
| `direct_mode` | `bool` | Direct motor coil control via XTARGET |
| `test_mode` | `bool` | Analog test output on ENCN_DCO (not for normal use) |

### RampConfig

Ramp generator configuration structure for two-phase acceleration and deceleration.

**Location**: [`inc/tmc5160_types.hpp`](../inc/tmc5160_types.hpp)

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
- D1 must not be 0 in positioning mode (defaults to 100 steps/s² if not set)
- Unit conversion requires `MotorSpec` and `MechanicalSystem` configuration for non-Step units
- See datasheet section 12 for detailed ramp generator operation

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

## Helper Classes

### TMC5160DaisyChain

High-level manager for multiple TMC5160 drivers in a SPI daisy-chain configuration.

**Location**: [`inc/tmc5160_daisy_chain.hpp`](../inc/tmc5160_daisy_chain.hpp)

#### Constructor

| Method | Signature | Description |
|--------|-----------|-------------|
| `TMC5160DaisyChain()` | `TMC5160DaisyChain(CommType& comm, uint8_t num_onboard_devices, uint32_t f_clk = 12000000)` | Create daisy-chain manager with specified number of onboard devices |

#### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `operator[]` | `TMC5160<CommType>& operator[](size_t index)` | Reference to driver | Access driver at specified position (0-based) |
| `InitializeAll()` | `bool InitializeAll(const DriverConfig& config) noexcept` | `true` on success | Initialize all onboard devices with same config |
| `AddDevice()` | `bool AddDevice(size_t position) noexcept` | `true` on success | Add extra device at specified position |
| `RemoveDevice()` | `bool RemoveDevice(size_t position) noexcept` | `true` on success | Remove extra device at specified position |
| `GetNumDevices()` | `size_t GetNumDevices() const noexcept` | Number of devices | Get total number of devices (onboard + extra) |
| `GetOnboardCount()` | `size_t GetOnboardCount() const noexcept` | Number of onboard devices | Get number of onboard devices |

**Usage:**
```cpp
tmc5160::TMC5160DaisyChain<MySPI, 5> chain(spiComm, 3, 12'000'000);
chain.InitializeAll(cfg);
auto& motor_x = chain[0];
motor_x.rampControl.SetTargetPosition(1000);
```

### TMC5160MultiNode

High-level manager for multiple TMC5160 drivers in a UART multi-node configuration.

**Location**: [`inc/tmc5160_multi_node.hpp`](../inc/tmc5160_multi_node.hpp)

#### Constructor

| Method | Signature | Description |
|--------|-----------|-------------|
| `TMC5160MultiNode()` | `TMC5160MultiNode(CommType& comm, uint8_t num_onboard_devices, uint32_t f_clk = 12000000)` | Create multi-node manager with specified number of onboard devices |

#### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `operator[]` | `TMC5160<CommType>& operator[](size_t index)` | Reference to driver | Access driver at specified logical index (0-based) |
| `ProgramSequentially()` | `bool ProgramSequentially() noexcept` | `true` on success | Program all devices sequentially using NAI/NAO pins (required at startup) |
| `ProgramDevice()` | `bool ProgramDevice(size_t index) noexcept` | `true` on success | Program single device at specified index (must be accessible at address 0) |
| `InitializeAll()` | `bool InitializeAll(const DriverConfig& config) noexcept` | `true` on success | Initialize all onboard devices with same config |
| `AddDevice()` | `bool AddDevice(size_t index) noexcept` | `true` on success | Add extra device at specified logical index |
| `RemoveDevice()` | `bool RemoveDevice(size_t index) noexcept` | `true` on success | Remove extra device at specified logical index |
| `GetNumDevices()` | `size_t GetNumDevices() const noexcept` | Number of devices | Get total number of devices (onboard + extra) |
| `GetOnboardCount()` | `size_t GetOnboardCount() const noexcept` | Number of onboard devices | Get number of onboard devices |

**Usage:**
```cpp
tmc5160::TMC5160MultiNode<MyUART, 5> nodes(uartComm, 3, 12'000'000);
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
