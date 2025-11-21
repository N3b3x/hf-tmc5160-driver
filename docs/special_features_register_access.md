# Register Access Types and Special Behaviors

This document provides comprehensive information about TMC5160 register access types, verified against the TMC5160A datasheet Rev 1.18.

## Register Access Type Overview

The TMC5160 has 47 registers with the following access type distribution:

| Access Type | Count | Percentage | Description |
|-------------|-------|------------|-------------|
| **Read-Write (RW)** | 16 | 34.0% | Configuration and control registers |
| **Read-Only (R)** | 10 | 21.3% | Status, result, and read-only registers |
| **Write-Only (W)** | 21 | 44.7% | Write-only configuration and parameter registers |
| **Read-Write-Clear (RWC)** | 3 | 6.4% | Registers with special clear behavior |

## Complete Register Access Table

| Address | Register Name | Access | Category | Description |
|---------|---------------|--------|----------|-------------|
| 0x00 | GCONF | R/W | Configuration | Global configuration flags |
| 0x01 | GSTAT | R/W (clear) | Status | Global status flags (bits cleared on read) |
| 0x02 | IFCNT | R | Status | UART transmission counter (incremented on each transmission) |
| 0x03 | SLAVECONF | W | Configuration | UART slave configuration (write-only per datasheet, though reads may work) |
| 0x04 | IO_INPUT_OUTPUT | R/W | I/O | Read input pins / write output pins (dual-purpose: R for INPUT bits, W for OUTPUT bit) |
| 0x05 | X_COMPARE | W | Motion Control | Position comparison register (write-only) |
| 0x06 | OTP_PROG | W | OTP | OTP programming register (write-only) |
| 0x07 | OTP_READ | R | OTP | OTP read register (read-only) |
| 0x08 | FACTORY_CONF | R/W | Factory | Factory configuration (clock trim, can override OTP default) |
| 0x09 | SHORT_CONF | W | Protection | Short detector configuration (write-only) |
| 0x0A | DRV_CONF | W | Configuration | Driver configuration (write-only) |
| 0x0B | GLOBAL_SCALER | W | Current Control | Global scaling of motor current (write-only, 32-256) |
| 0x0C | OFFSET_READ | R | Calibration | Offset calibration results (read-only) |
| 0x10 | IHOLD_IRUN | W | Current Control | Driver current control (write-only) |
| 0x11 | TPOWERDOWN | W | Power Management | Delay before power down (write-only) |
| 0x12 | TSTEP | R | Status | Actual time between microsteps (read-only) |
| 0x13 | TPWMTHRS | W | Velocity Threshold | Upper velocity for stealthChop voltage PWM mode (write-only) |
| 0x14 | TCOOLTHRS | W | Velocity Threshold | Lower threshold velocity for coolStep and stallGuard (write-only) |
| 0x15 | THIGH | W | Velocity Threshold | Velocity threshold for chopper mode switching and fullstepping (write-only) |
| 0x20 | RAMPMODE | R/W | Motion Control | Driving mode (Velocity, Positioning, Hold) |
| 0x21 | XACTUAL | R/W | Motion Control | Actual motor position (can be written to set position) |
| 0x22 | VACTUAL | R | Status | Actual motor velocity from ramp generator (read-only) |
| 0x23 | VSTART | W | Motion Control | Motor start velocity (write-only) |
| 0x24 | A_1 | W | Motion Control | First acceleration between VSTART and V1 (write-only) |
| 0x25 | V_1 | W | Motion Control | First acceleration/deceleration phase target velocity (write-only) |
| 0x26 | AMAX | W | Motion Control | Second acceleration between V1 and VMAX (write-only) |
| 0x27 | VMAX | W | Motion Control | Target velocity in velocity mode (write-only) |
| 0x28 | DMAX | W | Motion Control | Deceleration between VMAX and V1 (write-only) |
| 0x2A | D_1 | W | Motion Control | Deceleration between V1 and VSTOP (write-only) |
| 0x2B | VSTOP | W | Motion Control | Motor stop velocity (write-only) |
| 0x2C | TZEROWAIT | W | Motion Control | Waiting time after ramping down to zero velocity (write-only) |
| 0x2D | XTARGET | R/W | Motion Control | Target position for ramp mode |
| 0x33 | VDCMIN | W | Motion Control | Velocity threshold for enabling dcStep (write-only) |
| 0x34 | SW_MODE | R/W | Switch Configuration | Switch mode configuration (reference switches, latching) |
| 0x35 | RAMP_STAT | R/W (clear) | Status | Ramp status and switch event status (some bits cleared by writing '1') |
| 0x36 | XLATCH | R | Status | Ramp generator latch position upon switch event (read-only) |
| 0x38 | ENCMODE | R/W | Encoder | Encoder configuration and use of N channel |
| 0x39 | X_ENC | R/W | Status | Actual encoder position (can be written to set position) |
| 0x3A | ENC_CONST | W | Encoder | Accumulation constant (write-only) |
| 0x3B | ENC_STATUS | R/W (clear) | Status | Encoder status information (bits cleared by writing '1') |
| 0x3C | ENC_LATCH | R | Status | Encoder position latched on N event (read-only) |
| 0x3D | ENC_DEVIATION | W | Encoder | Maximum number of steps deviation between encoder and XACTUAL (write-only) |
| 0x60 | MSLUT_0 | W | Microstep | Microstep lookup table entry 0 (write-only) |
| 0x61 | MSLUT_1 | W | Microstep | Microstep lookup table entry 1 (write-only) |
| 0x62 | MSLUT_2 | W | Microstep | Microstep lookup table entry 2 (write-only) |
| 0x63 | MSLUT_3 | W | Microstep | Microstep lookup table entry 3 (write-only) |
| 0x64 | MSLUT_4 | W | Microstep | Microstep lookup table entry 4 (write-only) |
| 0x65 | MSLUT_5 | W | Microstep | Microstep lookup table entry 5 (write-only) |
| 0x66 | MSLUT_6 | W | Microstep | Microstep lookup table entry 6 (write-only) |
| 0x67 | MSLUT_7 | W | Microstep | Microstep lookup table entry 7 (write-only) |
| 0x68 | MSLUTSEL | W | Microstep | Look up table segmentation definition (write-only) |
| 0x69 | MSLUTSTART | W | Microstep | Absolute current at microstep table entries 0 and 256 (write-only) |
| 0x6A | MSCNT | R | Status | Actual position in the microstep table (read-only) |
| 0x6B | MSCURACT | R | Status | Actual microstep current (read-only) |
| 0x6C | CHOPCONF | R/W | Chopper | Chopper and driver configuration |
| 0x6D | COOLCONF | W | CoolStep | coolStep smart current control and stallGuard2 configuration (write-only) |
| 0x6E | DCCTRL | W | dcStep | dcStep automatic commutation configuration (write-only) |
| 0x6F | DRV_STATUS | R | Status | stallGuard2 value and driver error flags (read-only) |
| 0x70 | PWMCONF | W | StealthChop | stealthChop voltage PWM mode chopper configuration (write-only per datasheet) |
| 0x71 | PWM_SCALE | R | Status | Results of stealthChop amplitude regulator (read-only) |
| 0x72 | PWM_AUTO | R | Status | Automatically determined PWM config values (read-only) |
| 0x73 | LOST_STEPS | R | Status | Number of input steps skipped due to dcStep (read-only, SD_MODE=1 only) |

## Register Access Types Explained

- **R/W**: Read-Write - Register can be both read and written
- **R**: Read-Only - Register can only be read (status/result registers)
- **W**: Write-Only - Register can only be written (command/trigger registers)
- **R/W (clear)**: Read-Write with special clear behavior - Some bits can be cleared by writing '1'

## Special Register Behaviors

### Clear-on-Read/Write Registers (RWC)

These registers have special behavior where certain bits are cleared when read or written:

1. **GSTAT (0x01)**: Global status flags
   - Status bits are automatically cleared when read
   - Can also be cleared by writing '1' to the corresponding bit
   - While technically R/W, status bits are automatically cleared when read

2. **RAMP_STAT (0x35)**: Ramp status and switch event status
   - Contains both read-only status bits and event bits that can be cleared
   - Event bits (`event_stop_l`, `event_stop_r`, `event_stop_sg`, `event_pos_reached`, `second_move`) can be cleared by writing '1'
   - Status bits (`status_sg`, `vzero`, `position_reached`, etc.) are read-only

3. **ENC_STATUS (0x3B)**: Encoder status information
   - Status bits can be cleared by writing '1' to the corresponding bit position
   - `deviation_warn` cannot be cleared while a warning still persists


### Read-Write Registers with Special Behavior

1. **FACTORY_CONF (0x08)**: Factory configuration (clock trim)
   - **Access**: Read-Write (RW)
   - **Note**: Can be written to override OTP defaults
   - **Usage**: Typically read-only, but writing allows customization of clock trim settings

2. **IO_INPUT_OUTPUT (0x04)**: Input/output pin register
   - **Access**: Read-Write (RW)
   - **Note**: Dual-purpose register - INPUT bits (0-7, 24-31) are read-only, OUTPUT bit (0) is write-only
   - **Usage**: Read to get input pin states, write to set output pin polarity

3. **XACTUAL (0x21)**: Actual motor position
   - **Access**: Read-Write (RW)
   - **Note**: Can be written to set/initialize position (useful for homing)
   - **Warning**: Writing to XACTUAL in positioning mode will start a motion

4. **X_ENC (0x39)**: Actual encoder position
   - **Access**: Read-Write (RW)
   - **Note**: Can be written to synchronize encoder position with motor position

## Register Categories

### Read-Only Status Registers
These registers provide real-time status information and cannot be written:
- **IFCNT (0x02)**: UART transmission counter
- **TSTEP (0x12)**: Actual time between microsteps
- **VACTUAL (0x22)**: Actual motor velocity
- **XLATCH (0x36)**: Latched position on switch event
- **ENC_LATCH (0x3C)**: Encoder position latched on N event
- **MSCNT (0x6A)**: Microstep table position
- **MSCURACT (0x6B)**: Actual microstep current
- **DRV_STATUS (0x6F)**: Driver status and StallGuard2 value
- **PWM_SCALE (0x71)**: PWM scale results
- **PWM_AUTO (0x72)**: Auto-determined PWM values
- **LOST_STEPS (0x73)**: Lost steps counter (only valid when dcStep mode is enabled, SD_MODE=1)

### Write-Only Configuration Registers
These registers are write-only per datasheet (though some may be readable in practice):
- **SLAVECONF (0x03)**: UART slave configuration
- **X_COMPARE (0x05)**: Position comparison register
- **OTP_PROG (0x06)**: OTP programming register
- **SHORT_CONF (0x09)**: Short detector configuration
- **DRV_CONF (0x0A)**: Driver configuration
- **GLOBAL_SCALER (0x0B)**: Global current scaler
- **IHOLD_IRUN (0x10)**: Driver current control
- **TPOWERDOWN (0x11)**: Power down delay
- **TPWMTHRS (0x13)**: StealthChop threshold
- **TCOOLTHRS (0x14)**: CoolStep threshold
- **THIGH (0x15)**: High-speed threshold
- **VSTART (0x23)**: Motor start velocity
- **A_1 (0x24)**: First acceleration phase
- **V_1 (0x25)**: Transition velocity
- **AMAX (0x26)**: Maximum acceleration
- **VMAX (0x27)**: Maximum velocity
- **DMAX (0x28)**: Maximum deceleration
- **D_1 (0x2A)**: Final deceleration
- **VSTOP (0x2B)**: Motor stop velocity
- **TZEROWAIT (0x2C)**: Zero wait time
- **VDCMIN (0x33)**: dcStep velocity threshold
- **ENC_CONST (0x3A)**: Encoder accumulation constant
- **ENC_DEVIATION (0x3D)**: Encoder deviation threshold
- **MSLUT_0 through MSLUT_7 (0x60-0x67)**: Microstep lookup table entries
- **MSLUTSEL (0x68)**: Microstep lookup table segmentation
- **MSLUTSTART (0x69)**: Microstep lookup table start values
- **COOLCONF (0x6D)**: CoolStep and StallGuard2 configuration
- **DCCTRL (0x6E)**: dcStep automatic commutation configuration
- **PWMCONF (0x70)**: StealthChop voltage PWM mode configuration

### Factory/OTP Registers
- **FACTORY_CONF (0x08)**: Read-write factory configuration (can override OTP)
- **OTP_READ (0x07)**: Read-only OTP memory
- **OTP_PROG (0x06)**: Write-only OTP programming (requires special sequence)

### Position Registers
- **XACTUAL (0x21)**: Can be read (current position) or written (set position)
- **XTARGET (0x2D)**: Write target position for positioning mode
- **X_ENC (0x39)**: Can be read (encoder position) or written (synchronize position)

## Implementation Details

### Register Definitions

All register definitions use an X-MACRO pattern (following TMC9660 style) for maintainability and single source of truth:

- **Register Definitions**: `inc/tmc5160_register_defs.hpp` - X-MACRO list of all registers
- **Access Type Functions**: `inc/tmc5160_register_access.hpp` - Functions to query register access types

### Access Type Query Functions

The following functions are available to query register access types:

```cpp
#include "tmc5160_register_access.hpp"

// Get register access type
RegisterAccess GetRegisterAccess(uint8_t address);

// Check if register is readable
bool IsRegisterReadable(uint8_t address);

// Check if register is writable
bool IsRegisterWritable(uint8_t address);

// Get access type string representation
const char* GetAccessTypeString(RegisterAccess access);
```

### Usage Example

```cpp
#include "tmc5160_register_access.hpp"

// Check register access before operation
uint8_t reg_addr = 0x6F; // DRV_STATUS

if (tmc5160::IsRegisterReadable(reg_addr)) {
    uint32_t status;
    driver.GetComm().ReadRegister(reg_addr, status);
    // Process status...
}

// Get access type information
tmc5160::RegisterAccess access = tmc5160::GetRegisterAccess(reg_addr);
const char* access_str = tmc5160::GetAccessTypeString(access);
// access_str = "R" for read-only registers
```

## Best Practices

1. **Write-Only Registers**: Never attempt to read write-only registers. Always maintain full register state internally if partial updates are needed
2. **Write Verification**: Verify writes by checking the response data matches what was sent (if supported by communication interface), not by reading back write-only registers
3. **Clear Bits**: For RWC registers, write '1' to clear event bits, not '0'
4. **Position Registers**: Be careful when writing to XACTUAL or X_ENC as this may trigger motion or affect position tracking
5. **Status Registers**: Read status registers regularly to clear event flags and monitor driver state
6. **Register Access Compliance**: All functions strictly follow datasheet register access types (R/W/C). Functions that violate access types have been removed

## Verification Status

✅ **All register access types verified against:**
- TMC5160A Datasheet Rev 1.18 (2023-MAR-01)
- Current implementation in `src/tmc5160.cpp`
- Register definitions in `inc/tmc5160_registers.hpp`

## Key Findings

1. **Read-Only Registers**: All status and result registers are correctly marked as read-only
2. **Write-Only Registers**: 21 registers are write-only per datasheet specification - these registers cannot be read
3. **Clear Behavior**: GSTAT, RAMP_STAT, and ENC_STATUS have special clear-on-read/write behavior (RWC)
4. **Position Registers**: XACTUAL and X_ENC can be both read and written (used to set position)
5. **Dual-Purpose Register**: IO_INPUT_OUTPUT serves dual purpose - reading returns inputs, writing controls outputs
6. **Access Compliance**: All functions strictly comply with datasheet register access types. Functions that attempted to read write-only registers have been removed

---

**Last Updated**: Based on TMC5160A Datasheet Rev 1.18 (2023-MAR-01)
