# TMC51x0 Register Access and Configuration Guide

This guide provides a comprehensive overview of the TMC51x0 (TMC5130 & TMC5160) registers, their access permissions, and recommended configuration strategies. It is based on the verified driver implementation and the TMC51x0 datasheet.

## 1. Register Map and Access Types

The TMC51x0 registers are divided into functional groups. Each register has specific access permissions:
- **RW**: Read and Write
- **R**: Read-Only
- **W**: Write-Only
- **RWC**: Read, Write-1-to-Clear

### 1.1 General Configuration (0x00...0x0F)

| Address | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `GCONF` | **RW** | Global configuration flags (direction, PWM mode, diagnostics). |
| `0x01` | `GSTAT` | **RWC** | Global status flags (reset, error, UV). Write 1 to clear flags. |
| `0x02` | `IFCNT` | **R** | Interface transmission counter (UART only). |
| `0x03` | `NODECONF`| **W** | UART node address and send delay configuration. |
| `0x04` | `IOIN` | **R** | Reads state of all digital input pins. |
| `0x04` | `OUTPUT` | **W** | Sets SDO_CFG0 pin polarity (Bit 0). Same address as IOIN! |
| `0x05` | `X_COMPARE`| **W** | Position comparison value for position pulse output. |
| `0x06` | `OTP_PROG` | **W** | OTP programming register (Factory use). |
| `0x07` | `OTP_READ` | **R** | Reads OTP memory contents. |
| `0x08` | `FACTORY_CONF`| **RW** | Factory clock trim (FCLKTRIM). |
| `0x09` | `SHORT_CONF`| **W** | Short circuit detection sensitivity and filtering. |
| `0x0A` | `DRV_CONF` | **W** | Driver strength, BBM time, and OT protection settings. |
| `0x0B` | `GLOBAL_SCALER`| **W** | Global current scaling factor (32-256). |
| `0x0C` | `OFFSET_READ`| **R** | Reads offset calibration results. |

**Usage Recommendations:**
- **`GCONF`**: Configure at startup. Use `driver.ConfigureGlobalConfig()`.
- **`GSTAT`**: Check for `uv_cp` (Charge Pump Undervoltage) if motor stops unexpectedly. Clear `reset` flag after initialization.
- **`IOIN`**: Use `driver.io.ReadInputStatus()` to verify wiring (e.g., `REFL`/`REFR` switches, `DRV_ENN`).
- **`GLOBAL_SCALER`**: Set to appropriate value (e.g., 160-256) to match motor current capability.

### 1.2 Velocity Dependent Driver Feature Control (0x10...0x1F)

| Address | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x10` | `IHOLD_IRUN`| **W** | Run and Hold current settings. |
| `0x11` | `TPOWERDOWN`| **W** | Delay before power down to stand still current. |
| `0x12` | `TSTEP` | **R** | Measured time between microsteps (inverse of velocity). |
| `0x13` | `TPWMTHRS` | **W** | Upper velocity threshold for StealthChop. |
| `0x14` | `TCOOLTHRS`| **W** | Lower velocity threshold for CoolStep/StallGuard. |
| `0x15` | `THIGH` | **W** | Velocity threshold for high-speed chopper/fullstep mode. |

**Usage Recommendations:**
- **`IHOLD_IRUN`**: Use `driver.motorControl.SetCurrent(irun, ihold)`. Typical `irun=16-31`, `ihold=0-16`.
- **Thresholds**: Use `driver.thresholds.SetModeChangeSpeeds()` to set `TPWMTHRS`, `TCOOLTHRS`, and `THIGH` (default unit: revolutions per second).
    - `TPWMTHRS`: Set typical cruising speed limit for silent operation.
    - `TCOOLTHRS`: Set above startup speed to enable StallGuard only when moving stably.

### 1.3 Ramp Generator Motion Control (0x20...0x2D)

| Address | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x20` | `RAMPMODE` | **RW** | 0=Positioning, 1=Vel(+), 2=Vel(-), 3=Hold. |
| `0x21` | `XACTUAL` | **RW** | Actual position (signed 32-bit). |
| `0x22` | `VACTUAL` | **R** | Actual velocity (signed 24-bit). |
| `0x23` | `VSTART` | **W** | Start velocity. Set > 0 (e.g., 100) to avoid jerky starts. |
| `0x24` | `A1` | **W** | First acceleration (Start -> V1). |
| `0x25` | `V1` | **W** | Threshold velocity for A1->AMAX transition. |
| `0x26` | `AMAX` | **W** | Max acceleration (standard). |
| `0x27` | `VMAX` | **W** | Target velocity. |
| `0x28` | `DMAX` | **W** | Max deceleration (standard). |
| `0x2A` | `D1` | **W** | Final deceleration (V1 -> Stop). **Do not set to 0!** |
| `0x2B` | `VSTOP` | **W** | Stop velocity. Set >= VSTART. |
| `0x2C` | `TZEROWAIT`| **W** | Wait time after stop before direction change. |
| `0x2D` | `XTARGET` | **RW** | Target position (signed 32-bit). |

**Usage Recommendations:**
- **Positioning**: Set `VMAX`, `AMAX`, `DMAX` first, then write `XTARGET` to start move.
- **Velocity Mode**: Use `driver.rampControl.SetMaxSpeed()` with positive/negative value to control direction.
- **S-Curve**: For smoother motion, set `V1 > 0` and use `A1`/`D1` for start/stop phases.
- **Safety**: Always ensure `D1 != 0` in positioning mode.

### 1.4 Ramp Generator Driver Features (0x30...0x36)

| Address | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x33` | `VDCMIN` | **W** | Velocity threshold for DcStep commutation. |
| `0x34` | `SW_MODE` | **RW** | Switch stop configuration (endstops, stall stop). |
| `0x35` | `RAMP_STAT`| **RWC** | Ramp status flags (reached, stop event, stall). |
| `0x36` | `XLATCH` | **R** | Position latched on switch/stall event. |

**Usage Recommendations:**
- **Homing**: Use `driver.homing.PerformSensorlessHoming()` or `driver.homing.PerformSwitchHoming()`.
- **Endstops**: Configure with `driver.switches.ConfigureReferenceSwitch()`. Use `driver.io.ReadInputStatus()` to verify wiring logic (NO/NC).

### 1.5 Encoder Registers (0x38...0x3C)

| Address | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x38` | `ENCMODE` | **RW** | Encoder configuration (polarity, prescaler). |
| `0x39` | `X_ENC` | **RW** | Actual encoder position. |
| `0x3A` | `ENC_CONST`| **W** | Encoder resolution matching factor. |
| `0x3B` | `ENC_STATUS`| **RWC** | Encoder events (N-channel, deviation). |
| `0x3C` | `ENC_LATCH`| **R** | Encoder position latched on N-event. |
| `0x3D` | `ENC_DEVIATION`| **W** | Max deviation for warning flag. |

**Usage Recommendations:**
- **Calibration**: Use `driver.encoder.SetResolution()` to automatically calculate `ENC_CONST`.
- **Verification**: Check `ENC_STATUS` periodically for deviation warnings if using closed loop.

### 1.6 Motor Driver Registers (0x60...0x7F)

| Address | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x60`..`0x67`| `MSLUT[0..7]`| **W** | Microstep lookup table entries. |
| `0x68` | `MSLUTSEL` | **W** | LUT segment configuration. |
| `0x69` | `MSLUTSTART`| **W** | Start current for LUT. |
| `0x6A` | `MSCNT` | **R** | Microstep counter (0-1023). |
| `0x6B` | `MSCURACT` | **R** | Actual microstep current (signed 9-bit). |
| `0x6C` | `CHOPCONF` | **RW** | Chopper configuration (TOFF, TBL, HSTRT, HEND). |
| `0x6D` | `COOLCONF` | **W** | CoolStep and StallGuard2 configuration. |
| `0x6E` | `DCCTRL` | **W** | DcStep configuration. |
| `0x6F` | `DRV_STATUS`| **R** | Driver status (StallGuard result, OT/Short flags). |
| `0x70` | `PWMCONF` | **W** | StealthChop PWM configuration. |
| `0x71` | `PWM_SCALE`| **R** | StealthChop regulator result. |
| `0x72` | `PWM_AUTO` | **R** | Auto-tuned PWM values. |
| `0x73` | `LOST_STEPS`| **R** | Lost step counter (DcStep only). |

**Usage Recommendations:**
- **`CHOPCONF`**: Set `TOFF > 0` to enable driver. `TOFF=0` disables driver.
- **`DRV_STATUS`**: Monitor `SG_RESULT` for load estimation. 0 = High Load, 1023 = Low Load.
- **`PWM_SCALE`**: Use to verify StealthChop calibration. If `PWM_SCALE_AUTO` is 0 or very low, calibration failed (increase standstill time).

## 2. Configuration Workflows

### 2.1 Startup Verification
Always perform a setup verification at startup to catch hardware issues early:
```cpp
// Verify IC connection and Input Pins
if (!driver.status.VerifySetup()) {
    // Handle error - check power/wiring
}
```

### 2.2 Silent Operation (StealthChop)
1. Set `GCONF.en_pwm_mode = 1`.
2. Set `TPWMTHRS` to a high value (e.g. 0.1 rev/s ~6 RPM, using `SetStealthChopVelocityThreshold()`).
3. **Crucial**: Allow motor to stand still for >130ms after enabling to allow `PWM_OFS_AUTO` calibration.
4. Monitor `PWM_SCALE` to confirm regulation is active.

### 2.3 High Torque / High Speed (SpreadCycle)
1. Set `GCONF.en_pwm_mode = 0`.
2. Configure `CHOPCONF` (`TOFF`, `TBL`, `HSTRT`, `HEND`) for your motor inductance.
3. This mode is louder but provides dynamic torque and prevents " runaway" calibration issues at high loads.

### 2.4 Sensorless Homing
1. Configure `StallGuard` (`COOLCONF`) with `SGT` (Threshold).
2. Set `TCOOLTHRS` low enough to cover homing speed.
3. Call `driver.homing.PerformSensorlessHoming()` (uses existing SGT threshold from motor config).
   - This internally sets `SW_MODE.sg_stop = 1`.
   - Moves motor until stall detected.
   - Stops and returns position.

## 3. Troubleshooting

- **Motor not moving**: Check `GSTAT.uv_cp` (Undervoltage), `CHOPCONF.toff` (Driver Enabled?), `DRV_ENN` pin state.
- **Motor stalling early**: Decrease acceleration (`AMAX`, `A1`). Check `Run Current` (`IRUN`).
- **StealthChop noise**: Check `PWM_SCALE_AUTO`. If near limits (+/- 255), adjust `PWM_GRAD`.
- **Position loss**: Check `DRV_STATUS` for `ot` (Overtemp) or `s2g` (Short) events which disable the bridge.
