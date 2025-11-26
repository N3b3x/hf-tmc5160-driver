/**
 * @file tmc5160_register_defs.hpp
 * @brief X-MACRO definitions for TMC5160 registers
 *
 * This file uses the X-MACRO pattern to define all TMC5160 registers with their
 * addresses, access types, and metadata. This allows generating multiple data
 * structures from a single source of truth.
 *
 * Usage follows TMC9660 pattern:
 *   #define X(addr, name, access, category, desc) ...
 *   REGISTER_LIST(X)
 *   #undef X
 *
 * Access types:
 *   - R: Read-only
 *   - W: Write-only
 *   - RW: Read-Write
 *   - RWC: Read-Write with clear behavior (some bits cleared on read/write)
 *
 * Categories:
 *   - CONFIG: Configuration registers
 *   - STATUS: Status/result registers
 *   - MOTION: Motion control registers
 *   - CURRENT: Current control registers
 *   - CHOPPER: Chopper configuration
 *   - ENCODER: Encoder registers
 *   - MICROSTEP: Microstep lookup table
 *   - PROTECTION: Protection and safety
 *   - OTP: One-time programmable memory
 *   - IO: Input/Output pins
 */

#ifndef TMC5160_REGISTER_DEFS_HPP
#define TMC5160_REGISTER_DEFS_HPP

#include <cstdint>

//--------------------------------------
//  TMC5160 Register List
//--------------------------------------
#define REGISTER_LIST(X) \
    /* General configuration registers */ \
    X(0x00, GCONF, RW, CONFIG, "Global configuration flags") \
    X(0x01, GSTAT, RWC, STATUS, "Global status flags (clear on read)") \
    X(0x02, IFCNT, R, STATUS, "UART transmission counter") \
    X(0x03, SLAVECONF, W, CONFIG, "UART slave configuration (write-only per datasheet)") \
    X(0x04, IO_INPUT_OUTPUT, RW, IO, "Read input / write output pins") \
    X(0x05, X_COMPARE, W, MOTION, "Position comparison register (write-only)") \
    X(0x06, OTP_PROG, W, OTP, "OTP programming register") \
    X(0x07, OTP_READ, R, OTP, "OTP read register") \
    X(0x08, FACTORY_CONF, RW, STATUS, "Factory configuration (clock trim, can override OTP)") \
    X(0x09, SHORT_CONF, W, PROTECTION, "Short detector configuration (write-only)") \
    X(0x0A, DRV_CONF, W, CONFIG, "Driver configuration (write-only)") \
    X(0x0B, GLOBAL_SCALER, W, CURRENT, "Global scaling of motor current (write-only)") \
    X(0x0C, OFFSET_READ, R, STATUS, "Offset calibration results") \
    /* Velocity dependent driver feature control registers */ \
    X(0x10, IHOLD_IRUN, W, CURRENT, "Driver current control (write-only)") \
    X(0x11, TPOWERDOWN, W, CONFIG, "Delay before power down (write-only)") \
    X(0x12, TSTEP, R, STATUS, "Actual time between microsteps") \
    X(0x13, TPWMTHRS, W, CONFIG, "Upper velocity for stealthChop voltage PWM mode (write-only)") \
    X(0x14, TCOOLTHRS, W, CONFIG, "Lower threshold velocity for coolStep and stallGuard (write-only)") \
    X(0x15, THIGH, W, CONFIG, "Velocity threshold for chopper mode switching and fullstepping (write-only)") \
    /* Ramp generator motion control registers */ \
    X(0x20, RAMPMODE, RW, MOTION, "Driving mode (Velocity, Positioning, Hold)") \
    X(0x21, XACTUAL, RW, MOTION, "Actual motor position") \
    X(0x22, VACTUAL, R, STATUS, "Actual motor velocity from ramp generator") \
    X(0x23, VSTART, W, MOTION, "Motor start velocity (write-only)") \
    X(0x24, A_1, W, MOTION, "First acceleration between VSTART and V1 (write-only)") \
    X(0x25, V_1, W, MOTION, "First acceleration/deceleration phase target velocity (write-only)") \
    X(0x26, AMAX, W, MOTION, "Second acceleration between V1 and VMAX (write-only)") \
    X(0x27, VMAX, W, MOTION, "Target velocity in velocity mode (write-only)") \
    X(0x28, DMAX, W, MOTION, "Deceleration between VMAX and V1 (write-only)") \
    X(0x2A, D_1, W, MOTION, "Deceleration between V1 and VSTOP (write-only)") \
    X(0x2B, VSTOP, W, MOTION, "Motor stop velocity (write-only)") \
    X(0x2C, TZEROWAIT, W, MOTION, "Waiting time after ramping down to zero velocity (write-only)") \
    X(0x2D, XTARGET, RW, MOTION, "Target position for ramp mode") \
    /* Ramp generator driver feature control registers */ \
    X(0x33, VDCMIN, W, MOTION, "Velocity threshold for enabling dcStep (write-only)") \
    X(0x34, SW_MODE, RW, CONFIG, "Switch mode configuration") \
    X(0x35, RAMP_STAT, RWC, STATUS, "Ramp status and switch event status") \
    X(0x36, XLATCH, R, STATUS, "Ramp generator latch position upon switch event") \
    /* Encoder registers */ \
    X(0x38, ENCMODE, RW, ENCODER, "Encoder configuration and use of N channel") \
    X(0x39, X_ENC, RW, STATUS, "Actual encoder position") \
    X(0x3A, ENC_CONST, W, ENCODER, "Accumulation constant (write-only)") \
    X(0x3B, ENC_STATUS, RWC, STATUS, "Encoder status information") \
    X(0x3C, ENC_LATCH, R, STATUS, "Encoder position latched on N event") \
    X(0x3D, ENC_DEVIATION, W, ENCODER, "Maximum number of steps deviation between encoder and XACTUAL (write-only)") \
    /* Motor driver registers */ \
    X(0x60, MSLUT_0, W, MICROSTEP, "Microstep lookup table entry 0 (write-only)") \
    X(0x61, MSLUT_1, W, MICROSTEP, "Microstep lookup table entry 1 (write-only)") \
    X(0x62, MSLUT_2, W, MICROSTEP, "Microstep lookup table entry 2 (write-only)") \
    X(0x63, MSLUT_3, W, MICROSTEP, "Microstep lookup table entry 3 (write-only)") \
    X(0x64, MSLUT_4, W, MICROSTEP, "Microstep lookup table entry 4 (write-only)") \
    X(0x65, MSLUT_5, W, MICROSTEP, "Microstep lookup table entry 5 (write-only)") \
    X(0x66, MSLUT_6, W, MICROSTEP, "Microstep lookup table entry 6 (write-only)") \
    X(0x67, MSLUT_7, W, MICROSTEP, "Microstep lookup table entry 7 (write-only)") \
    X(0x68, MSLUTSEL, W, MICROSTEP, "Look up table segmentation definition (write-only)") \
    X(0x69, MSLUTSTART, W, MICROSTEP, "Absolute current at microstep table entries 0 and 256 (write-only)") \
    X(0x6A, MSCNT, R, STATUS, "Actual position in the microstep table") \
    X(0x6B, MSCURACT, R, STATUS, "Actual microstep current") \
    X(0x6C, CHOPCONF, RW, CHOPPER, "Chopper and driver configuration") \
    X(0x6D, COOLCONF, W, CONFIG, "coolStep smart current control and stallGuard2 configuration (write-only)") \
    X(0x6E, DCCTRL, W, CONFIG, "dcStep automatic commutation configuration (write-only)") \
    X(0x6F, DRV_STATUS, R, STATUS, "stallGuard2 value and driver error flags") \
    X(0x70, PWMCONF, W, CHOPPER, "stealthChop voltage PWM mode chopper configuration (write-only per datasheet)") \
    X(0x71, PWM_SCALE, R, STATUS, "Results of stealthChop amplitude regulator") \
    X(0x72, PWM_AUTO, R, STATUS, "Automatically determined PWM config values") \
    X(0x73, LOST_STEPS, R, STATUS, "Number of input steps skipped due to dcStep (SD_MODE=1 only)")

/**
 * @brief Get register definition string
 * @param address Register address
 * @return String containing register name and description, or nullptr if not found
 */
static const char* GetRegisterDef(uint8_t address) {
  switch (address) {
    #define X(addr, name, access, category, desc) \
      case addr: return #name ": " desc;
    REGISTER_LIST(X)
    #undef X
    default: return nullptr;
  }
}

#endif // TMC5160_REGISTER_DEFS_HPP
