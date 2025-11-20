/**
 * @file tmc5160_registers.hpp
 * @brief Register definitions and bitfield structures for TMC5160 stepper motor
 * driver
 *
 * This file contains all register addresses, bit field definitions, and
 * register structures for the TMC5160 stepper motor driver IC. All register
 * definitions are based on the TMC5160 datasheet and cross-referenced with
 * archived driver implementations.
 *
 * @defgroup TMC5160_Registers Register Definitions
 * @brief Register addresses and bitfield structures
 */

#ifndef TMC5160_REGISTERS_HPP
#define TMC5160_REGISTERS_HPP

#include <cstdint>

namespace tmc5160 {

/**
 * @brief TMC5160 register addresses
 *
 * All register addresses for the TMC5160 stepper motor driver.
 * Registers are organized by functional category.
 */
namespace Registers {
// General configuration registers
constexpr uint8_t GCONF = 0x00;           ///< Global configuration flags
constexpr uint8_t GSTAT = 0x01;           ///< Global status flags
constexpr uint8_t IFCNT = 0x02;           ///< UART transmission counter
constexpr uint8_t SLAVECONF = 0x03;       ///< UART slave configuration
constexpr uint8_t IO_INPUT_OUTPUT = 0x04; ///< Read input / write output pins
constexpr uint8_t X_COMPARE = 0x05;       ///< Position comparison register
constexpr uint8_t OTP_PROG = 0x06;        ///< OTP programming register
constexpr uint8_t OTP_READ = 0x07;        ///< OTP read register
constexpr uint8_t FACTORY_CONF = 0x08;    ///< Factory configuration (clock trim)
constexpr uint8_t SHORT_CONF = 0x09;      ///< Short detector configuration
constexpr uint8_t DRV_CONF = 0x0A;        ///< Driver configuration
constexpr uint8_t GLOBAL_SCALER = 0x0B;   ///< Global scaling of motor current
constexpr uint8_t OFFSET_READ = 0x0C;     ///< Offset calibration results

// Velocity dependent driver feature control registers
constexpr uint8_t IHOLD_IRUN = 0x10; ///< Driver current control
constexpr uint8_t TPOWERDOWN = 0x11; ///< Delay before power down
constexpr uint8_t TSTEP = 0x12;      ///< Actual time between microsteps
constexpr uint8_t TPWMTHRS = 0x13;   ///< Upper velocity for stealthChop voltage PWM mode
constexpr uint8_t TCOOLTHRS = 0x14;  ///< Lower threshold velocity for switching on smart energy
                                     ///< coolStep and stallGuard feature
constexpr uint8_t THIGH = 0x15;      ///< Velocity threshold for switching into a
                                     ///< different chopper mode and fullstepping

// Ramp generator motion control registers
constexpr uint8_t RAMPMODE = 0x20;  ///< Driving mode (Velocity, Positioning, Hold)
constexpr uint8_t XACTUAL = 0x21;   ///< Actual motor position
constexpr uint8_t VACTUAL = 0x22;   ///< Actual motor velocity from ramp generator
constexpr uint8_t VSTART = 0x23;    ///< Motor start velocity
constexpr uint8_t A_1 = 0x24;       ///< First acceleration between VSTART and V1
constexpr uint8_t V_1 = 0x25;       ///< First acceleration/deceleration phase target velocity
constexpr uint8_t AMAX = 0x26;      ///< Second acceleration between V1 and VMAX
constexpr uint8_t VMAX = 0x27;      ///< Target velocity in velocity mode
constexpr uint8_t DMAX = 0x28;      ///< Deceleration between VMAX and V1
constexpr uint8_t D_1 = 0x2A;       ///< Deceleration between V1 and VSTOP
constexpr uint8_t VSTOP = 0x2B;     ///< Motor stop velocity
constexpr uint8_t TZEROWAIT = 0x2C; ///< Waiting time after ramping down to zero velocity before
                                    ///< next movement or direction inversion can start
constexpr uint8_t XTARGET = 0x2D;   ///< Target position for ramp mode

// Ramp generator driver feature control registers
constexpr uint8_t VDCMIN = 0x33;    ///< Velocity threshold for enabling automatic commutation dcStep
constexpr uint8_t SW_MODE = 0x34;   ///< Switch mode configuration
constexpr uint8_t RAMP_STAT = 0x35; ///< Ramp status and switch event status
constexpr uint8_t XLATCH = 0x36;    ///< Ramp generator latch position upon programmable switch event

// Encoder registers
constexpr uint8_t ENCMODE = 0x38;       ///< Encoder configuration and use of N channel
constexpr uint8_t X_ENC = 0x39;         ///< Actual encoder position
constexpr uint8_t ENC_CONST = 0x3A;     ///< Accumulation constant
constexpr uint8_t ENC_STATUS = 0x3B;    ///< Encoder status information
constexpr uint8_t ENC_LATCH = 0x3C;     ///< Encoder position latched on N event
constexpr uint8_t ENC_DEVIATION = 0x3D; ///< Maximum number of steps deviation between encoder
                                        ///< counter and XACTUAL for deviation warning

// Motor driver registers
constexpr uint8_t MSLUT_0 = 0x60;    ///< Microstep table entry 0
constexpr uint8_t MSLUT_1 = 0x61;    ///< Microstep table entry 1
constexpr uint8_t MSLUT_2 = 0x62;    ///< Microstep table entry 2
constexpr uint8_t MSLUT_3 = 0x63;    ///< Microstep table entry 3
constexpr uint8_t MSLUT_4 = 0x64;    ///< Microstep table entry 4
constexpr uint8_t MSLUT_5 = 0x65;    ///< Microstep table entry 5
constexpr uint8_t MSLUT_6 = 0x66;    ///< Microstep table entry 6
constexpr uint8_t MSLUT_7 = 0x67;    ///< Microstep table entry 7
constexpr uint8_t MSLUTSEL = 0x68;   ///< Look up table segmentation definition
constexpr uint8_t MSLUTSTART = 0x69; ///< Absolute current at microstep table entries 0 and 256
constexpr uint8_t MSCNT = 0x6A;      ///< Actual position in the microstep table
constexpr uint8_t MSCURACT = 0x6B;   ///< Actual microstep current
constexpr uint8_t CHOPCONF = 0x6C;   ///< Chopper and driver configuration
constexpr uint8_t COOLCONF = 0x6D;   ///< coolStep smart current control register
                                     ///< and stallGuard2 configuration
constexpr uint8_t DCCTRL = 0x6E;     ///< dcStep automatic commutation configuration register
constexpr uint8_t DRV_STATUS = 0x6F; ///< stallGuard2 value and driver error flags
constexpr uint8_t PWMCONF = 0x70;    ///< stealthChop voltage PWM mode chopper configuration
constexpr uint8_t PWM_SCALE = 0x71;  ///< Results of stealthChop amplitude regulator
constexpr uint8_t PWM_AUTO = 0x72;   ///< Automatically determined PWM config values
constexpr uint8_t LOST_STEPS = 0x73; ///< Number of input steps skipped due to
                                     ///< dcStep. only with SD_MODE = 1
} // namespace Registers

/**
 * @brief Ramp mode enumeration values
 */
enum class RampMode : uint8_t {
  POSITIONING = 0x00,  ///< Positioning mode using all A, D and V parameters
  VELOCITY_POS = 0x01, ///< Positive VMAX, using AMAX acceleration
  VELOCITY_NEG = 0x02, ///< Negative VMAX, using AMAX acceleration
  HOLD = 0x03          ///< Velocity remains unchanged, unless stop event occurs
};

/**
 * @brief PWM freewheel mode enumeration values
 */
enum class PWMFreewheel : uint8_t {
  NORMAL = 0x00,   ///< Normal operation
  ENABLED = 0x01,  ///< Freewheeling
  SHORT_LS = 0x02, ///< Coil shorted using LS drivers
  SHORT_HS = 0x03  ///< Coil shorted using HS drivers
};

/**
 * @brief Encoder N channel sensitivity enumeration values
 */
enum class EncoderSensitivity : uint8_t {
  NO_EDGE = 0x00,      ///< N channel active while the N event is valid
  RISING_EDGE = 0x01,  ///< N channel active when the N event is activated
  FALLING_EDGE = 0x02, ///< N channel active when the N event is de-activated
  BOTH_EDGES = 0x03    ///< N channel active on N event activation and de-activation
};

/**
 * @brief General configuration register (GCONF)
 *
 * Global configuration flags for the TMC5160 driver.
 *
 * Bit assignments per datasheet:
 * - Bit 0: recalibrate - 1: Zero crossing recalibration during driver disable (via DRV_ENN or via
 * TOFF setting)
 * - Bit 1: faststandstill - Timeout for step execution until standstill detection: 1=Short time
 * (2^18 clocks), 0=Normal time (2^20 clocks)
 * - Bit 2: en_pwm_mode - 1: StealthChop voltage PWM mode enabled (depending on velocity
 * thresholds). Switch from off to on state while in stand-still and at IHOLD=nominal IRUN current,
 * only.
 * - Bit 3: multistep_filt - 1: Enable step input filtering for StealthChop optimization with
 * external step source (default=1)
 * - Bit 4: shaft - 1: Inverse motor direction
 * - Bit 5: diag0_error - (only with SD_MODE=1) 1: Enable DIAG0 active on driver errors: Over
 * temperature (ot), short to GND (s2g). DIAG0 always shows the reset-status, i.e., is active low
 * during reset condition.
 * - Bit 6: diag0_otpw - (only with SD_MODE=1) 1: Enable DIAG0 active on driver over temperature
 * prewarning (otpw)
 * - Bit 7: diag0_stall - (with SD_MODE=1) 1: Enable DIAG0 active on motor stall (set TCOOLTHRS
 * before using this feature) diag0_step - (with SD_MODE=0) 0: DIAG0 outputs interrupt signal, 1:
 * Enable DIAG0 as STEP output (half frequency, dual edge triggered) for external STEP/DIR driver
 * - Bit 8: diag1_stall - (with SD_MODE=1) 1: Enable DIAG1 active on motor stall (set TCOOLTHRS
 * before using this feature) diag1_dir - (with SD_MODE=0) 0: DIAG1 outputs position compare signal,
 * 1: Enable DIAG1 as DIR output for external STEP/DIR driver
 * - Bit 9: diag1_index - (only with SD_MODE=1) 1: Enable DIAG1 active on index position (microstep
 * look up table position 0)
 * - Bit 10: diag1_onstate - (only with SD_MODE=1) 1: Enable DIAG1 active when chopper is on (for
 * the coil which is in the second half of the fullstep)
 * - Bit 11: diag1_steps_skipped - (only with SD_MODE=1) 1: Enable output toggle when steps are
 * skipped in DcStep mode (increment of LOST_STEPS). Do not enable in conjunction with other DIAG1
 * options.
 * - Bit 12: diag0_int_pushpull - 0: SWN_DIAG0 is open collector output (active low), 1: Enable
 * SWN_DIAG0 push pull output (active high)
 * - Bit 13: diag1_poscomp_pushpull - 0: SWP_DIAG1 is open collector output (active low), 1: Enable
 * SWP_DIAG1 push pull output (active high)
 * - Bit 14: small_hysteresis - 0: Hysteresis for step frequency comparison is 1/16, 1: Hysteresis
 * for step frequency comparison is 1/32
 * - Bit 15: stop_enable - 0: Normal operation, 1: Emergency stop: ENCA_DCIN stops the sequencer
 * when tied high (no steps become executed by the sequencer, motor goes to standstill state)
 * - Bit 16: direct_mode - 0: Normal operation, 1: Motor coil currents and polarity directly
 * programmed via serial interface: Register XTARGET (0x2D) specifies signed coil A current
 * (bits 8..0) and coil B current (bits 24..16). In this mode, the current is scaled by IHOLD
 * setting. Velocity based current regulation of StealthChop is not available in this mode.
 * - Bit 17: test_mode - 0: Normal operation, 1: Enable analog test output on pin ENCN_DCO.
 * IHOLD[1..0] selects the function of ENCN_DCO: 0...2: T120, DAC, VDDH. Hint: Not for user, set to
 * 0 for normal operation!
 * - Bits 18-31: Reserved
 */
union GCONF_Register {
  uint32_t value;

  struct {
    uint32_t recalibrate : 1;            ///< Bit 0: 1=Zero crossing recalibration during driver disable (via
                                         ///< DRV_ENN or via TOFF setting)
    uint32_t faststandstill : 1;         ///< Bit 1: Standstill timeout: 1=Short time (2^18 clocks),
                                         ///< 0=Normal time (2^20 clocks)
    uint32_t en_pwm_mode : 1;            ///< Bit 2: 1=StealthChop voltage PWM mode enabled (depending on velocity
                                         ///< thresholds). Switch only in stand-still at IHOLD=nominal IRUN.
    uint32_t multistep_filt : 1;         ///< Bit 3: 1=Enable step input filtering for StealthChop
                                         ///< optimization with external step source (default=1)
    uint32_t shaft : 1;                  ///< Bit 4: 1=Inverse motor direction
    uint32_t diag0_error : 1;            ///< Bit 5: (only with SD_MODE=1) 1=Enable DIAG0 active on driver
                                         ///< errors (OT, S2G, UV_CP). DIAG0 always shows reset-status.
    uint32_t diag0_otpw : 1;             ///< Bit 6: (only with SD_MODE=1) 1=Enable DIAG0 active on driver over
                                         ///< temperature prewarning (otpw)
    uint32_t diag0_stall_step : 1;       ///< Bit 7: (SD_MODE=1) 1=Enable DIAG0 active on motor stall (set
                                         ///< TCOOLTHRS before using). (SD_MODE=0) 0=DIAG0 outputs
                                         ///< interrupt signal, 1=Enable DIAG0 as STEP output (half
                                         ///< frequency, dual edge triggered)
    uint32_t diag1_stall_dir : 1;        ///< Bit 8: (SD_MODE=1) 1=Enable DIAG1 active on motor stall (set
                                         ///< TCOOLTHRS before using). (SD_MODE=0) 0=DIAG1 outputs position
                                         ///< compare signal, 1=Enable DIAG1 as DIR output
    uint32_t diag1_index : 1;            ///< Bit 9: (only with SD_MODE=1) 1=Enable DIAG1 active on index
                                         ///< position (microstep look up table position 0)
    uint32_t diag1_onstate : 1;          ///< Bit 10: (only with SD_MODE=1) 1=Enable DIAG1 active when chopper is
                                         ///< on (for the coil which is in the second half of the fullstep)
    uint32_t diag1_steps_skipped : 1;    ///< Bit 11: (only with SD_MODE=1) 1=Enable output toggle when
                                         ///< steps are skipped in DcStep mode (increment of LOST_STEPS). Do
                                         ///< not enable in conjunction with other DIAG1 options.
    uint32_t diag0_int_pushpull : 1;     ///< Bit 12: SWN_DIAG0 output: 0=open collector (active low),
                                         ///< 1=push pull (active high)
    uint32_t diag1_poscomp_pushpull : 1; ///< Bit 13: SWP_DIAG1 output: 0=open collector (active
                                         ///< low), 1=push pull (active high)
    uint32_t small_hysteresis : 1;       ///< Bit 14: Step frequency comparison hysteresis: 0=1/16, 1=1/32
    uint32_t stop_enable : 1;            ///< Bit 15: 0=Normal operation, 1=Emergency stop: ENCA_DCIN stops the
                                         ///< sequencer when tied high (no steps become executed, motor goes to
                                         ///< standstill)
    uint32_t direct_mode : 1;            ///< Bit 16: 0=Normal operation, 1=Direct motor coil control: XTARGET
                                         ///< bits 8..0=coil A current, bits 24..16=coil B current (scaled by
                                         ///< IHOLD). Velocity based current regulation of StealthChop not
                                         ///< available.
    uint32_t test_mode : 1;              ///< Bit 17: 0=Normal operation, 1=Enable analog test output on pin
                                         ///< ENCN_DCO. IHOLD[1..0] selects function (0..2: T120, DAC, VDDH).
                                         ///< Hint: Not for user, set to 0!
    uint32_t reserved : 14;              ///< Bits 18-31: Reserved
  } bits;
};

/**
 * @brief Global status register (GSTAT)
 *
 * Global status flags indicating reset, driver errors, and charge pump status.
 */
union GSTAT_Register {
  uint32_t value;

  struct {
    uint32_t reset : 1;     ///< Indicates that the IC has been reset since the last
                            ///< read access to GSTAT
    uint32_t drv_err : 1;   ///< Indicates that the driver has been shut down due
                            ///< to overtemperature or short circuit detection
                            ///< since the last read access
    uint32_t uv_cp : 1;     ///< Indicates an undervoltage on the charge pump. The
                            ///< driver is disabled in this case.
    uint32_t reserved : 29; ///< Reserved bits
  } bits;
};

/**
 * @brief UART slave configuration register (SLAVECONF)
 *
 * Configuration for UART slave mode operation.
 */
union SLAVECONF_Register {
  uint32_t value;

  struct {
    uint32_t slaveaddr : 8; ///< Address of unit for the UART interface. The
                            ///< address becomes incremented by one when the
                            ///< external address pin NAI is active.
    uint32_t senddelay : 4; ///< Number of bit times before replying to a register
                            ///< read in UART mode. Set > 1 with multiple slaves.
    uint32_t reserved : 20; ///< Reserved bits
  } bits;
};

/**
 * @brief Input/output pin register (IO_INPUT_OUTPUT)
 *
 * Read input pins or write output pins configuration.
 */
union IOIN_Register {
  uint32_t value;

  struct {
    uint32_t refl_step : 1;      ///< Reference left / step input
    uint32_t refr_dir : 1;       ///< Reference right / direction input
    uint32_t encb_dcen_cfg4 : 1; ///< Encoder B / DCEN / CFG4
    uint32_t enca_dcin_cfg5 : 1; ///< Encoder A / DCIN / CFG5
    uint32_t drv_enn : 1;        ///< Driver enable (inverted)
    uint32_t enc_n_dco_cfg6 : 1; ///< Encoder N / DCO / CFG6
    uint32_t sd_mode : 1;        ///< 1=External step and dir source
    uint32_t swcomp_in : 1;      ///< Software comparator input
    uint32_t reserved : 16;      ///< Reserved bits
    uint32_t version : 8;        ///< IC version
  } bits;
};

/**
 * @brief OTP programming register (OTP_PROG)
 *
 * One-time programmable memory programming register.
 *
 * Bit assignments per datasheet:
 * - Bits 2..0: OTPBIT - Selection of OTP bit to be programmed to the selected byte location
 * (n=0..7: programs bit n to a logic 1)
 * - Bits 5..4: OTPBYTE - Set to 00
 * - Bits 15..8: OTPMAGIC - Set to 0xBD to enable programming. A programming time of minimum 10ms
 * per bit is recommended.
 */
union OTP_PROG_Register {
  uint32_t value;

  struct {
    uint32_t otpbit : 3;     ///< Bits 2..0: Selection of OTP bit to be programmed (n=0..7: programs bit
                             ///< n to logic 1)
    uint32_t reserved1 : 1;  ///< Bit 3: Reserved
    uint32_t otpbyte : 2;    ///< Bits 5..4: Selection of OTP byte. Set to 00
    uint32_t reserved2 : 2;  ///< Bits 7..6: Reserved
    uint32_t otpmagic : 8;   ///< Bits 15..8: Set to 0xBD to enable programming
    uint32_t reserved3 : 16; ///< Bits 31..16: Reserved bits
  } bits;
};

/**
 * @brief OTP read register (OTP_READ)
 *
 * One-time programmable configuration memory read register.
 *
 * OTP memory holds power-up defaults for certain registers. All OTP bits are
 * cleared to 0 by default. Programming can only set bits, clearing is not possible.
 *
 * Bit assignments per datasheet OTP memory map:
 * - Bit 7 (otp0.7): otp_TBL - Reset default for TBL (0: TBL=%10 ~3μs, 1: TBL=%01 ~2μs)
 * - Bit 6 (otp0.6): otp_BBM - Reset default for DRVCONF.BBMCLKS (0: BBMCLKS=4, 1: BBMCLKS=2)
 * - Bit 5 (otp0.5): otp_S2_LEVEL - Reset default for short-detection levels
 *                    (0: S2G_LEVEL=S2VS_LEVEL=6, 1: S2G_LEVEL=S2VS_LEVEL=12)
 * - Bits 4..0 (otp0.4..0): OTP_FCLKTRIM - Reset default for FCLKTRIM
 *                          (0: lowest frequency, 31: highest frequency)
 *                          Factory pre-programmed to 12MHz, differs between ICs!
 */
union OTP_READ_Register {
  uint32_t value;

  struct {
    uint32_t otp_fclktrim : 5; ///< Bits 4..0: Reset default for FCLKTRIM (0-31)
    uint32_t otp_S2_level : 1; ///< Bit 5: Reset default for Short detection levels
    uint32_t otp_bbm : 1;      ///< Bit 6: Reset default for DRVCONF.BBMCLKS
    uint32_t otp_tbl : 1;      ///< Bit 7: Reset default for TBL
    uint32_t reserved : 24;    ///< Reserved bits
  } bits;
};

/**
 * @brief Short detector configuration register (SHORT_CONF)
 *
 * Configuration for short circuit detection sensitivity and filtering.
 *
 * Bit assignments per datasheet:
 * - Bits 3..0: S2VS_LEVEL - Short to VS detector level for lowside FETs
 *   Checks for voltage drop in LS MOSFET and sense resistor.
 *   4 (highest sensitivity) ... 15 (lowest sensitivity)
 *   Hint: Settings from 1 to 3 will trigger during normal operation due to voltage drop on sense
 * resistor. (Reset Default: OTP 6 or 12)
 * - Bits 11..8: S2G_LEVEL - Short to GND detector level for highside FETs
 *   Checks for voltage drop on high side MOSFET
 *   2 (highest sensitivity) ... 15 (lowest sensitivity)
 *   Attention: Settings below 6 not recommended at >52V operation – false detection might result
 *   (Reset Default: OTP 6 or 12)
 * - Bits 17..16: SHORTFILTER - Spike filtering bandwidth for short detection
 *   0 (lowest, 100ns), 1 (1μs), 2 (2μs), 3 (3μs)
 *   Hint: A good PCB layout will allow using setting 0. Increase value, if erroneous short
 * detection occurs. (Reset Default = %01)
 * - Bit 18: shortdelay - Short detection delay
 *   0=750ns: normal, 1=1500ns: high
 *   The short detection delay shall cover the bridge switching time. 0 will work for most
 * applications. (Reset Default = 0)
 */
union SHORT_CONF_Register {
  uint32_t value;

  struct {
    uint32_t s2vs_level : 4;  ///< Bits 3..0: Short to VS detector for low side FETs (4-15, highest
                              ///< to lowest sensitivity)
    uint32_t reserved1 : 4;   ///< Bits 7..4: Reserved
    uint32_t s2g_level : 4;   ///< Bits 11..8: Short to GND detector for high side FETs (2-15, highest
                              ///< to lowest sensitivity)
    uint32_t reserved2 : 4;   ///< Bits 15..12: Reserved
    uint32_t shortfilter : 2; ///< Bits 17..16: Spike filtering bandwidth (0=100ns, 1=1μs, 2=2μs, 3=3μs)
    uint32_t shortdelay : 1;  ///< Bit 18: Short detection delay (0=750ns normal, 1=1500ns high)
    uint32_t reserved3 : 13;  ///< Bits 31..19: Reserved
  } bits;
};

/**
 * @brief Driver configuration register (DRV_CONF)
 *
 * Driver configuration for external MOSFETs, break-before-make control, and
 * sense amplifier filter settings.
 *
 * Bit assignments per datasheet:
 * - Bits 4..0: BBMTIME - Break-Before-Make delay
 *   0=shortest (100ns) ... 16 (200ns) ... 24=longest (375ns)
 *   >24 not recommended, use BBMCLKS instead
 *   (Reset Default = 0)
 * - Bits 11..8: BBMCLKS - Digital BBM time in clock cycles (0..15, typ. 83ns)
 *   The longer setting rules (BBMTIME vs. BBMCLKS)
 *   (Reset Default: OTP 4 or 2)
 * - Bits 17..16: OTSELECT - Overtemperature level for bridge disable
 *   00 = 150°C (default)
 *   01 = 143°C
 *   10 = 136°C (not recommended when VSA > 24V)
 *   11 = 120°C (not recommended, no hysteresis)
 *   (Reset Default = 00)
 * - Bits 19..18: DRVSTRENGTH - Gate driver current
 *   00 = weak (default)
 *   01 = weak+TC (medium above OTPW level)
 *   10 = medium
 *   11 = strong
 *   (Reset Default = 00)
 * - Bits 21..20: FILT_ISENSE - Filter time constant of sense amplifier
 *   00 = low - 100ns (default)
 *   01 = 200ns
 *   10 = 300ns
 *   11 = high - 400ns
 *   (Reset Default = 00)
 */
union DRV_CONF_Register {
  uint32_t value;

  struct {
    uint32_t bbmtime : 5;     ///< Break-Before-Make delay (0..24): 0=100ns ... 16=200ns ... 24=375ns.
                              ///< >24 not recommended, use BBMCLKS instead.
    uint32_t reserved1 : 3;   ///< Reserved
    uint32_t bbmclks : 4;     ///< Digital BBM time in clock cycles (0..15). Each step ≈83ns. Longer of
                              ///< BBMTIME or BBMCLKS prevails.
    uint32_t reserved2 : 4;   ///< Reserved
    uint32_t otselect : 2;    ///< Overtemperature (OTPW) bridge disable threshold: 0=150°C, 1=143°C,
                              ///< 2=136°C, 3=120°C
    uint32_t drvstrength : 2; ///< Gate driver current: 0=weak, 1=weak+TC, 2=medium, 3=strong
    uint32_t filt_isense : 2; ///< Sense amplifier filter: 0=100ns, 1=200ns, 2=300ns, 3=400ns
    uint32_t reserved3 : 10;  ///< Reserved
  } bits;
};

/**
 * @brief Offset calibration result register (OFFSET_READ)
 *
 * Results from offset calibration procedure.
 *
 * Bit assignments per datasheet:
 * - Bits 15..8: Phase A offset calibration result (signed)
 * - Bits 7..0: Phase B offset calibration result (signed)
 */
union OFFSET_READ_Register {
  uint32_t value;

  struct {
    uint32_t phase_b : 8;   ///< Phase B offset calibration result (bits 7..0, signed)
    uint32_t phase_a : 8;   ///< Phase A offset calibration result (bits 15..8, signed)
    uint32_t reserved : 16; ///< Reserved bits
  } bits;
};

/**
 * @brief Driver current control register (IHOLD_IRUN)
 *
 * Configuration for motor run current and standstill current.
 *
 * Bit assignments per datasheet:
 * - Bits 4..0: IHOLD - Standstill current (0=1/32...31=32/32)
 *   In combination with StealthChop mode, setting IHOLD=0 allows to choose
 *   freewheeling or coil short circuit for motor stand still.
 * - Bits 12..8: IRUN - Motor run current (0=1/32...31=32/32)
 *   Choose sense resistors so that normal IRUN is 16 to 31 for best microstep performance.
 * - Bits 19..16: IHOLDDELAY - Controls the number of clock cycles for motor
 *   power down after motion as soon as standstill is detected (stst=1) and
 *   TPOWERDOWN has expired. The smooth transition avoids a motor jerk upon power down.
 *   0: instant power down
 *   1..15: Delay per current reduction step in multiple of 2^18 clocks
 */
union IHOLD_IRUN_Register {
  uint32_t value;

  struct {
    uint32_t ihold : 5;      ///< Bits 4..0: Standstill current (0=1/32...31=32/32)
    uint32_t reserved1 : 3;  ///< Reserved bits (5..7)
    uint32_t irun : 5;       ///< Bits 12..8: Motor run current (0=1/32...31=32/32)
    uint32_t reserved2 : 3;  ///< Reserved bits (13..15)
    uint32_t iholddelay : 4; ///< Bits 19..16: Motor power down delay (0-15)
    uint32_t reserved3 : 12; ///< Reserved bits (20..31)
  } bits;
};

/**
 * @brief Switch mode configuration register (SW_MODE)
 *
 * Configuration for reference switches and stop events.
 *
 * Bit assignments per datasheet:
 * - Bit 0: stop_l_enable - Enable automatic motor stop during active left reference switch input
 * - Bit 1: stop_r_enable - Enable automatic motor stop during active right reference switch input
 * - Bit 2: pol_stop_l - Sets the active polarity of the left reference switch input
 * (0=non-inverted/high active, 1=inverted/low active)
 * - Bit 3: pol_stop_r - Sets the active polarity of the right reference switch input
 * (0=non-inverted/high active, 1=inverted/low active)
 * - Bit 4: swap_lr - Swap the left and the right reference switch input REFL and REFR
 * - Bit 5: latch_l_active - Activate latching of the position to XLATCH upon an active going edge
 * on REFL
 * - Bit 6: latch_l_inactive - Activate latching of the position to XLATCH upon an inactive going
 * edge on REFL
 * - Bit 7: latch_r_active - Activate latching of the position to XLATCH upon an active going edge
 * on REFR
 * - Bit 8: latch_r_inactive - Activate latching of the position to XLATCH upon an inactive going
 * edge on REFR
 * - Bit 9: en_latch_encoder - Latch encoder position to ENC_LATCH upon reference switch event
 * - Bit 10: sg_stop - Enable stop by StallGuard2 (also available in DcStep mode)
 * - Bit 11: en_softstop - Enable soft stop upon a stop event (uses deceleration ramp settings)
 */
union SW_MODE_Register {
  uint32_t value;

  struct {
    uint32_t stop_l_enable : 1;    ///< Enable automatic motor stop during active
                                   ///< left reference switch input
    uint32_t stop_r_enable : 1;    ///< Enable automatic motor stop during active
                                   ///< right reference switch input
    uint32_t pol_stop_l : 1;       ///< Sets the active polarity of the left reference
                                   ///< switch input (1=inverted, low active)
    uint32_t pol_stop_r : 1;       ///< Sets the active polarity of the right reference
                                   ///< switch input (1=inverted, low active)
    uint32_t swap_lr : 1;          ///< Swap the left and the right reference switch inputs
    uint32_t latch_l_active : 1;   ///< Activate latching of the position to
                                   ///< XLATCH upon an active going edge on REFL
    uint32_t latch_l_inactive : 1; ///< Activate latching of the position to XLATCH
                                   ///< upon an inactive going edge on REFL
    uint32_t latch_r_active : 1;   ///< Activate latching of the position to
                                   ///< XLATCH upon an active going edge on REFR
    uint32_t latch_r_inactive : 1; ///< Activate latching of the position to XLATCH
                                   ///< upon an inactive going edge on REFR
    uint32_t en_latch_encoder : 1; ///< Latch encoder position to ENC_LATCH upon
                                   ///< reference switch event
    uint32_t sg_stop : 1;          ///< Enable stop by stallGuard2 (also available in
                                   ///< dcStep mode)
    uint32_t en_softstop : 1;      ///< Enable soft stop upon a stop event (uses the
                                   ///< deceleration ramp settings)
    uint32_t reserved : 20;        ///< Reserved bits
  } bits;
};

/**
 * @brief Ramp status and switch event status register (RAMP_STAT)
 *
 * Status information about ramp generator and switch events.
 *
 * Bit assignments per datasheet:
 * - Bit 0: status_stop_l - Reference switch left status (1=active)
 * - Bit 1: status_stop_r - Reference switch right status (1=active)
 * - Bit 2: status_latch_l - Latch left ready (Write '1' to clear)
 * - Bit 3: status_latch_r - Latch right ready (Write '1' to clear)
 * - Bit 4: event_stop_l - Signals an active stop left condition due to stop switch (ORed to
 * interrupt output)
 * - Bit 5: event_stop_r - Signals an active stop right condition due to stop switch (ORed to
 * interrupt output)
 * - Bit 6: event_stop_sg - Signals an active StallGuard2 stop event (Write '1' to clear, ORed to
 * interrupt output)
 * - Bit 7: event_pos_reached - Signals that the target position has been reached (Write '1' to
 * clear, ORed to interrupt output)
 * - Bit 8: velocity_reached - Signals that the target velocity is reached
 * - Bit 9: position_reached - Signals that the target position is reached
 * - Bit 10: vzero - Signals that the actual velocity is 0
 * - Bit 11: t_zerowait_active - Signals that TZEROWAIT is active after a motor stop
 * - Bit 12: second_move - Signals that the automatic ramp required moving back in the opposite
 * direction (Write '1' to clear)
 * - Bit 13: status_sg - Signals an active StallGuard2 input from CoolStep driver or DcStep unit
 */
union RAMP_STAT_Register {
  uint32_t value;

  struct {
    uint32_t status_stop_l : 1;     ///< Reference switch left status (1=active)
    uint32_t status_stop_r : 1;     ///< Reference switch right status (1=active)
    uint32_t status_latch_l : 1;    ///< Latch left ready
    uint32_t status_latch_r : 1;    ///< Latch right ready
    uint32_t event_stop_l : 1;      ///< Signals an active stop left condition due to
                                    ///< stop switch
    uint32_t event_stop_r : 1;      ///< Signals an active stop right condition due
                                    ///< to stop switch
    uint32_t event_stop_sg : 1;     ///< Signals an active StallGuard2 stop event
    uint32_t event_pos_reached : 1; ///< Signals that the target position has
                                    ///< been reached
    uint32_t velocity_reached : 1;  ///< Signals that the target velocity is reached
    uint32_t position_reached : 1;  ///< Signals that the target position is reached
    uint32_t vzero : 1;             ///< Signals that the actual velocity is 0
    uint32_t t_zerowait_active : 1; ///< Signals that TZEROWAIT is active after
                                    ///< a motor stop
    uint32_t second_move : 1;       ///< Signals that the automatic ramp required
                                    ///< moving back in the opposite direction
    uint32_t status_sg : 1;         ///< Signals an active stallGuard2 input
    uint32_t reserved : 18;         ///< Reserved bits
  } bits;
};

/**
 * @brief Encoder configuration register (ENCMODE)
 *
 * Configuration for encoder interface and N channel event handling.
 *
 * Bit assignments per datasheet:
 * - Bit 0: pol_A - Required A polarity for an N channel event (0=neg., 1=pos.)
 * - Bit 1: pol_B - Required B polarity for an N channel event (0=neg., 1=pos.)
 * - Bit 2: pol_N - Defines active polarity of N (0=low active, 1=high active)
 * - Bit 3: ignore_AB - Ignore A and B polarity for N channel event
 *   (0: N event occurs only when polarities given by pol_N, pol_A and pol_B match)
 *   (1: Ignore A and B polarity for N channel event)
 * - Bit 4: clr_cont - Always latch or latch and clear X_ENC upon an N event
 * - Bit 5: clr_once - Latch or latch and clear X_ENC on the next N event following the write access
 * - Bit 6: pos_edge - N channel event sensitivity (positive edge)
 * - Bit 7: neg_edge - N channel event sensitivity (negative edge)
 *   Sensitivity encoding:
 *   - 00: N channel event is active during an active N event level
 *   - 01: N channel is valid upon active going N event
 *   - 10: N channel is valid upon inactive going N event
 *   - 11: N channel is valid upon active going and inactive going N event
 * - Bit 8: clr_enc_x - Clear encoder counter X_ENC upon N-event
 *   (0: Upon N event, X_ENC becomes latched to ENC_LATCH only)
 *   (1: Latch and additionally clear encoder counter X_ENC at N-event)
 * - Bit 9: latch_x_act - Also latch XACTUAL position together with X_ENC
 * - Bit 10: enc_sel_decimal - Encoder prescaler divisor
 *   (0: Binary mode - Counts ENC_CONST(fractional part) /65536)
 *   (1: Decimal mode - Counts in ENC_CONST(fractional part) /10000)
 */
union ENCMODE_Register {
  uint32_t value;

  struct {
    uint32_t pol_A : 1;           ///< Bit 0: Required A polarity for an N channel event (0=neg., 1=pos.)
    uint32_t pol_B : 1;           ///< Bit 1: Required B polarity for an N channel event (0=neg., 1=pos.)
    uint32_t pol_N : 1;           ///< Bit 2: Defines active polarity of N (0=low active, 1=high active)
    uint32_t ignore_AB : 1;       ///< Bit 3: Ignore A and B polarity for N channel event
    uint32_t clr_cont : 1;        ///< Bit 4: Always latch or latch and clear X_ENC upon an N event
    uint32_t clr_once : 1;        ///< Bit 5: Latch or latch and clear X_ENC on the next N event following
                                  ///< the write access
    uint32_t sensitivity : 2;     ///< Bits 7..6: N channel event sensitivity (pos_edge=bit6, neg_edge=bit7)
    uint32_t clr_enc_x : 1;       ///< Bit 8: Clear encoder counter X_ENC upon N-event
    uint32_t latch_x_act : 1;     ///< Bit 9: Also latch XACTUAL position together with X_ENC
    uint32_t enc_sel_decimal : 1; ///< Bit 10: Encoder prescaler divisor (0=binary, 1=decimal)
    uint32_t reserved : 21;       ///< Reserved bits (11..31)
  } bits;
};

/**
 * @brief Encoder status register (ENC_STATUS)
 *
 * Status information about encoder operation and deviation detection.
 *
 * Bit assignments per datasheet:
 * - Bit 0: n_event - Event detected. To clear the status bit, write with a 1 bit at the
 * corresponding position. This bit is ORed to the interrupt output signal.
 * - Bit 1: deviation_warn - Deviation warning. Deviation_warn cannot be cleared while a warning
 * still persists. Set ENC_DEVIATION zero to disable. This bit is ORed to the interrupt output
 * signal.
 */
union ENC_STATUS_Register {
  uint32_t value;

  struct {
    uint32_t n_event : 1;        ///< Bit 0: N event detected (Write '1' to clear)
    uint32_t deviation_warn : 1; ///< Bit 1: Deviation between X_ACTUAL and X_ENC detected (Write
                                 ///< '1' to clear, but only if warning no longer persists)
    uint32_t reserved : 30;      ///< Reserved bits (2..31)
  } bits;
};

/**
 * @brief Microstep lookup table segmentation definition register (MSLUTSEL)
 *
 * Defines four segments within each quarter MSLUT wave and their width control.
 *
 * Bit assignments per datasheet:
 * - Bits 1..0: W0 - LUT width select from ofs00 to ofs(X1-1)
 * - Bits 3..2: W1 - LUT width select from ofs(X1) to ofs(X2-1)
 * - Bits 5..4: W2 - LUT width select from ofs(X2) to ofs(X3-1)
 * - Bits 7..6: W3 - LUT width select from ofs(X3) to ofs255
 * - Bits 15..8: X1 - LUT segment 1 start
 * - Bits 23..16: X2 - LUT segment 2 start
 * - Bits 31..24: X3 - LUT segment 3 start
 *
 * Width control bit coding W0...W3:
 * - %00: MSLUT entry 0, 1 select: -1, +0
 * - %01: MSLUT entry 0, 1 select: +0, +1
 * - %10: MSLUT entry 0, 1 select: +1, +2
 * - %11: MSLUT entry 0, 1 select: +2, +3
 *
 * Segment boundaries:
 * - Segment 0: 0 to X1-1
 * - Segment 1: X1 to X2-1
 * - Segment 2: X2 to X3-1
 * - Segment 3: X3 to 255
 *
 * For defined response: 0 < X1 < X2 < X3
 */
union MSLUTSEL_Register {
  uint32_t value;

  struct {
    uint32_t w0 : 2; ///< Bits 1..0: LUT width select from ofs00 to ofs(X1-1)
    uint32_t w1 : 2; ///< Bits 3..2: LUT width select from ofs(X1) to ofs(X2-1)
    uint32_t w2 : 2; ///< Bits 5..4: LUT width select from ofs(X2) to ofs(X3-1)
    uint32_t w3 : 2; ///< Bits 7..6: LUT width select from ofs(X3) to ofs255
    uint32_t x1 : 8; ///< Bits 15..8: LUT segment 1 start
    uint32_t x2 : 8; ///< Bits 23..16: LUT segment 2 start
    uint32_t x3 : 8; ///< Bits 31..24: LUT segment 3 start
  } bits;
};

/**
 * @brief Actual microstep current register (MSCURACT)
 *
 * Read-only register showing actual microstep current for both phases.
 * Values are signed 9-bit (-256 to 255) as read from MSLUT (not scaled by current).
 *
 * Bit assignments per datasheet:
 * - Bits 8..0: CUR_B (signed) - Actual microstep current for motor phase B (sine wave)
 * - Bits 24..16: CUR_A (signed) - Actual microstep current for motor phase A (co-sine wave)
 */
union MSCURACT_Register {
  uint32_t value;

  struct {
    int32_t cur_b : 9;      ///< Bits 8..0: Actual microstep current phase B (signed, -256 to 255)
    uint32_t reserved1 : 7; ///< Reserved bits (9..15)
    int32_t cur_a : 9;      ///< Bits 24..16: Actual microstep current phase A (signed, -256 to 255)
    uint32_t reserved2 : 7; ///< Reserved bits (25..31)
  } bits;
};

/**
 * @brief Chopper and driver configuration register (CHOPCONF)
 *
 * Configuration for chopper timing, microstep resolution, and driver operation
 * modes.
 *
 * Bit assignments per datasheet:
 * - Bits 3..0: toff3..toff0 - Off time and driver enable (4 bits)
 *   %0000: Driver disable, all bridges off
 *   %0001: 1 – use only with TBL ≥ 2
 *   %0010...%1111: 2 ... 15
 * - Bits 6..4: hstrt2..hstrt0/TFD[2..0] - Hysteresis start value (chm=0) or fast decay time setting
 * (chm=1) (3 bits) chm=0: %000...%111: Add 1, 2, ..., 8 to hysteresis low value HEND chm=1: Fast
 * decay time setting TFD (MSB: fd3)
 * - Bits 10..7: hend3..hend0/HEND/OFFSET - Hysteresis low value (chm=0) or sine wave offset (chm=1)
 * (4 bits) chm=0: %0000...%1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 chm=1: %0000...%1111:
 * Offset is -3, -2, -1, 0, 1, ..., 12
 * - Bit 11: fd3/TFD[3] - MSB of fast decay time setting (chm=1) or reserved (chm=0)
 * - Bit 12: disfdcc - Fast decay mode (chm=1: disables current comparator usage for termination of
 * fast decay cycle)
 * - Bit 13: Reserved, set to 0
 * - Bit 14: chm - Chopper mode (0=Standard SpreadCycle, 1=Constant off time with fast decay time)
 * - Bits 16..15: tbl1..tbl0 - Blank time select (2 bits)
 *   %00...%11: Set comparator blank time to 16, 24, 36 or 54 clocks
 * - Bit 17: Reserved, set to 0
 * - Bit 18: vhighfs - High velocity fullstep selection (enables switching to fullstep when VHIGH is
 * exceeded)
 * - Bit 19: vhighchm - High velocity chopper mode (enables switching to chm=1 and fd=0 when VHIGH
 * is exceeded)
 * - Bits 23..20: tpfd3..tpfd0 - Passive fast decay time (4 bits)
 *   %0000: Disable
 *   %0001...%1111: 1 ... 15
 * - Bits 27..24: mres3..mres0 - Micro step resolution (4 bits)
 *   %0000: Native 256 microstep setting
 *   %0001...%1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP
 * - Bit 28: intpol - Interpolation to 256 microsteps (1: extrapolates MRES to 256 microsteps)
 * - Bit 29: dedge - Enable double edge step pulses (1: enables step impulse at each step edge)
 * - Bit 30: diss2g - Short to GND protection disable (1: disables protection)
 * - Bit 31: diss2vs - Short to supply protection disable (1: disables protection)
 */
union CHOPCONF_Register {
  uint32_t value;

  struct {
    uint32_t toff : 4;        ///< Bits 3..0: Off time and driver enable
    uint32_t hstrt_tfd : 3;   ///< Bits 6..4: HSTRT (chm=0) or TFD[2..0] (chm=1)
    uint32_t hend_offset : 4; ///< Bits 10..7: HEND (chm=0) or OFFSET (chm=1)
    uint32_t tfd_3 : 1;       ///< Bit 11: TFD[3] (chm=1) or reserved (chm=0)
    uint32_t disfdcc : 1;     ///< Bit 12: Fast decay mode (chm=1)
    uint32_t reserved1 : 1;   ///< Bit 13: Reserved, set to 0
    uint32_t chm : 1;         ///< Bit 14: Chopper mode (0=SpreadCycle, 1=Constant off time)
    uint32_t tbl : 2;         ///< Bits 16..15: Comparator blank time select
    uint32_t reserved2 : 1;   ///< Bit 17: Reserved, set to 0
    uint32_t vhighfs : 1;     ///< Bit 18: High velocity fullstep selection
    uint32_t vhighchm : 1;    ///< Bit 19: High velocity chopper mode
    uint32_t tpfd : 4;        ///< Bits 23..20: Passive fast decay time
    uint32_t mres : 4;        ///< Bits 27..24: Micro step resolution
    uint32_t intpol : 1;      ///< Bit 28: Interpolation to 256 microsteps
    uint32_t dedge : 1;       ///< Bit 29: Enable double edge step pulses
    uint32_t diss2g : 1;      ///< Bit 30: Short to GND protection disable
    uint32_t diss2vs : 1;     ///< Bit 31: Short to supply protection disable
  } bits;
};

/**
 * @brief coolStep smart current control and stallGuard2 configuration register
 * (COOLCONF)
 *
 * Configuration for smart current control and stallGuard2 stall detection.
 *
 * Bit assignments per datasheet:
 * - Bits 3..0: semin3..semin0 - Minimum StallGuard2 value for smart current control and smart
 * current enable (4 bits) %0000: smart current control CoolStep off %0001...%1111: 1 ... 15
 * - Bit 4: Reserved, set to 0
 * - Bits 6..5: seup1..seup0 - Current up step width (2 bits)
 *   %00...%11: 1, 2, 4, 8
 * - Bit 7: Reserved, set to 0
 * - Bits 11..8: semax3..semax0 - StallGuard2 hysteresis value for smart current control (4 bits)
 *   %0000...%1111: 0 ... 15
 * - Bit 12: Reserved, set to 0
 * - Bits 14..13: sedn1..sedn0 - Current down step speed (2 bits)
 *   %00: For each 32 StallGuard2 values decrease by one
 *   %01: For each 8 StallGuard2 values decrease by one
 *   %10: For each 2 StallGuard2 values decrease by one
 *   %11: For each StallGuard2 value decrease by one
 * - Bit 15: seimin - Minimum current for smart current control (1 bit)
 *   0: 1/2 of current setting (IRUN)
 *   1: 1/4 of current setting (IRUN)
 * - Bits 22..16: sgt6..sgt0 - StallGuard2 threshold value (7 bits, signed)
 *   -64 to +63: A higher value makes StallGuard2 less sensitive
 *   Zero (0) is the starting value working with most motors
 * - Bit 23: Reserved, set to 0
 * - Bit 24: sfilt - StallGuard2 filter enable (1 bit)
 *   0: Standard mode, high time resolution for StallGuard2
 *   1: Filtered mode, StallGuard2 signal updated for each four fullsteps
 * - Bits 31..25: Reserved, set to 0
 */
union COOLCONF_Register {
  uint32_t value;

  struct {
    uint32_t semin : 4;     ///< Bits 3..0: Minimum StallGuard2 value for smart current control
    uint32_t reserved1 : 1; ///< Bit 4: Reserved, set to 0
    uint32_t seup : 2;      ///< Bits 6..5: Current increment step width
    uint32_t reserved2 : 1; ///< Bit 7: Reserved, set to 0
    uint32_t semax : 4;     ///< Bits 11..8: StallGuard2 hysteresis value for smart current control
    uint32_t reserved3 : 1; ///< Bit 12: Reserved, set to 0
    uint32_t sedn : 2;      ///< Bits 14..13: Current decrement step speed
    uint32_t seimin : 1;    ///< Bit 15: Minimum current for smart current control
    int32_t sgt : 7;        ///< Bits 22..16: StallGuard2 threshold value (signed -64 to +63)
    uint32_t reserved4 : 1; ///< Bit 23: Reserved, set to 0
    uint32_t sfilt : 1;     ///< Bit 24: StallGuard2 filter enable
    uint32_t reserved5 : 7; ///< Bits 31..25: Reserved, set to 0
  } bits;
};

/**
 * @brief dcStep automatic commutation configuration register (DCCTRL)
 *
 * Configuration for dcStep automatic commutation and step loss detection.
 */
union DCCTRL_Register {
  uint32_t value;

  struct {
    uint32_t dc_time : 10;  ///< Upper PWM on time limit for commutation
    uint32_t reserved1 : 6; ///< Reserved bits
    uint32_t dc_sg : 8;     ///< Max. PWM on time for step loss detection using
                            ///< dcStep stallGuard2 in dcStep mode
    uint32_t reserved2 : 8; ///< Reserved bits
  } bits;
};

/**
 * @brief stallGuard2 value and driver error flags register (DRV_STATUS)
 *
 * Status information including stallGuard2 result, driver errors, and motor
 * current.
 *
 * Bit assignments per datasheet:
 * - Bits 9..0: SG_RESULT - StallGuard2 result or PWM on time for coil A in standstill (10 bits)
 *   Mechanical load measurement: Higher value means lower mechanical load. Value of 0 signals
 * highest load. Temperature measurement: In standstill, shows chopper on-time for motor coil A.
 * - Bits 11..10: Reserved, ignore these bits
 * - Bit 12: s2vsa - Short to supply indicator phase A
 * - Bit 13: s2vsb - Short to supply indicator phase B
 * - Bit 14: stealth - StealthChop indicator (1: Driver operates in StealthChop mode)
 * - Bit 15: fsactive - Full step active indicator (1: Driver has switched to fullstep)
 * - Bits 20..16: CS_ACTUAL - Actual motor current / smart energy current (5 bits)
 *   Actual current control scaling for monitoring smart energy current scaling
 * - Bits 23..21: Reserved, ignore these bits
 * - Bit 24: stallguard - StallGuard2 status (1: Motor stall detected or DcStep stall)
 * - Bit 25: ot - Overtemperature flag (1: Overtemperature limit reached, drivers disabled)
 * - Bit 26: otpw - Overtemperature pre-warning flag (1: Pre-warning threshold exceeded)
 * - Bit 27: s2ga - Short to ground indicator phase A
 * - Bit 28: s2gb - Short to ground indicator phase B
 * - Bit 29: ola - Open load indicator phase A
 * - Bit 30: olb - Open load indicator phase B
 * - Bit 31: stst - Standstill indicator (1: Motor standstill, occurs 2^20 clocks after last step
 * pulse)
 */
union DRV_STATUS_Register {
  uint32_t value;

  struct {
    uint32_t sg_result : 10; ///< Bits 9..0: StallGuard2 result or motor temperature estimation
    uint32_t reserved1 : 2;  ///< Bits 11..10: Reserved, ignore these bits
    uint32_t s2vsa : 1;      ///< Bit 12: Short to supply indicator phase A
    uint32_t s2vsb : 1;      ///< Bit 13: Short to supply indicator phase B
    uint32_t stealth : 1;    ///< Bit 14: StealthChop indicator
    uint32_t fsactive : 1;   ///< Bit 15: Full step active indicator
    uint32_t cs_actual : 5;  ///< Bits 20..16: Actual motor current / smart energy current
    uint32_t reserved2 : 3;  ///< Bits 23..21: Reserved, ignore these bits (bit 21 is also reserved
                             ///< per datasheet)
    uint32_t stallguard : 1; ///< Bit 24: StallGuard2 status
    uint32_t ot : 1;         ///< Bit 25: Overtemperature flag
    uint32_t otpw : 1;       ///< Bit 26: Overtemperature pre-warning flag
    uint32_t s2ga : 1;       ///< Bit 27: Short to ground indicator phase A
    uint32_t s2gb : 1;       ///< Bit 28: Short to ground indicator phase B
    uint32_t ola : 1;        ///< Bit 29: Open load indicator phase A
    uint32_t olb : 1;        ///< Bit 30: Open load indicator phase B
    uint32_t stst : 1;       ///< Bit 31: Standstill indicator
  } bits;
};

/**
 * @brief stealthChop voltage PWM mode chopper configuration register (PWMCONF)
 *
 * Configuration for stealthChop PWM mode operation.
 *
 * Bit assignments per datasheet:
 * - Bits 7..0: PWM_OFS - User defined amplitude (offset) (8 bits)
 *   User defined PWM amplitude offset (0-255) related to full motor current (CS_ACTUAL=31) in stand
 * still Reset default = 30
 * - Bits 15..8: PWM_GRAD - User defined amplitude gradient (8 bits)
 *   Velocity dependent gradient for PWM amplitude: PWM_GRAD * 256 / TSTEP
 * - Bits 17..16: pwm_freq1..pwm_freq0 - PWM frequency selection (2 bits)
 *   %00: fPWM=2/1024 fCLK (Reset default)
 *   %01: fPWM=2/683 fCLK
 *   %10: fPWM=2/512 fCLK
 *   %11: fPWM=2/410 fCLK
 * - Bit 18: pwm_autoscale - PWM automatic amplitude scaling (1 bit)
 *   0: User defined feed forward PWM amplitude
 *   1: Enable automatic current control (Reset default)
 * - Bit 19: pwm_autograd - PWM automatic gradient adaptation (1 bit)
 *   0: Fixed value for PWM_GRAD
 *   1: Automatic tuning (only with pwm_autoscale=1) (Reset default)
 * - Bits 21..20: freewheel1..freewheel0 - Stand still option when motor current setting is zero
 * (I_HOLD=0) (2 bits) %00: Normal operation %01: Freewheeling %10: Coil shorted using LS drivers
 *   %11: Coil shorted using HS drivers
 * - Bits 23..22: Reserved, set to 0
 * - Bits 27..24: PWM_REG - Regulation loop gradient (4 bits)
 *   User defined maximum PWM amplitude change per half wave when using pwm_autoscale=1 (1...15)
 *   1: 0.5 increments (slowest regulation)
 *   2: 1 increment
 *   4: 2 increments (Reset default)
 *   15: 7.5 increments (fastest regulation)
 * - Bits 31..28: PWM_LIM - PWM automatic scale amplitude limit when switching on (4 bits)
 *   Limit for PWM_SCALE_AUTO when switching back from SpreadCycle to StealthChop (Default = 12)
 */
union PWMCONF_Register {
  uint32_t value;

  struct {
    uint32_t pwm_ofs : 8;       ///< Bits 7..0: User defined PWM amplitude (offset)
    uint32_t pwm_grad : 8;      ///< Bits 15..8: User defined PWM amplitude (gradient)
    uint32_t pwm_freq : 2;      ///< Bits 17..16: PWM frequency selection
    uint32_t pwm_autoscale : 1; ///< Bit 18: Enable PWM automatic amplitude scaling
    uint32_t pwm_autograd : 1;  ///< Bit 19: PWM automatic gradient adaptation
    uint32_t freewheel : 2;     ///< Bits 21..20: Stand still option when I_HOLD=0
    uint32_t reserved1 : 2;     ///< Bits 23..22: Reserved, set to 0
    uint32_t pwm_reg : 4;       ///< Bits 27..24: Regulation loop gradient
    uint32_t pwm_lim : 4;       ///< Bits 31..28: PWM automatic scale amplitude limit when switching on
  } bits;
};

/**
 * @brief Results of stealthChop amplitude regulator register (PWM_SCALE)
 *
 * Read-only register showing actual PWM duty cycle and automatic regulation
 * results.
 *
 * Bit assignments per datasheet:
 * - Bits 7..0: PWM_SCALE_SUM - Actual PWM duty cycle (0-255)
 * - Bits 24..16: PWM_SCALE_AUTO - 9-bit signed offset added to calculated PWM duty cycle
 *   Result of automatic amplitude regulation based on current measurement (signed -255...+255)
 */
union PWM_SCALE_Register {
  uint32_t value;

  struct {
    uint32_t pwm_scale_sum : 8; ///< Bits 7..0: Actual PWM duty cycle (0-255)
    uint32_t reserved1 : 8;     ///< Reserved bits (8..15)
    int32_t pwm_scale_auto : 9; ///< Bits 24..16: Result of automatic amplitude regulation (signed
                                ///< -255...+255)
    uint32_t reserved2 : 7;     ///< Reserved bits (25..31)
  } bits;
};

/**
 * @brief stealthChop automatically generated values read out register
 * (PWM_AUTO)
 *
 * Read-only register showing automatically determined PWM configuration values.
 */
union PWM_AUTO_Register {
  uint32_t value;

  struct {
    uint32_t pwm_ofs_auto : 8;  ///< Automatically determined offset value
    uint32_t reserved1 : 8;     ///< Reserved bits
    uint32_t pwm_grad_auto : 8; ///< Automatically determined gradient value
    uint32_t reserved2 : 8;     ///< Reserved bits
  } bits;
};

} // namespace tmc5160

#endif // TMC5160_REGISTERS_HPP
