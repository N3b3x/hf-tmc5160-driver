/**
 * @file tmc5160_types.hpp
 * @brief Type definitions and enumerations for TMC5160 stepper motor driver
 *
 * This file contains all type definitions, enumerations, and structures
 * used by the TMC5160 driver library.
 *
 * @defgroup TMC5160_Types Type Definitions
 * @brief Enums, structs, and type definitions
 */

#ifndef TMC5160_TYPES_HPP
#define TMC5160_TYPES_HPP

#include <cstdint>
#include "tmc5160_registers.hpp"

namespace tmc5160 {

/**
 * @brief Input pin status structure
 * 
 * Represents the state of the TMC5160 input pins as read from the IOIN register.
 */
struct InputStatus {
  bool refl_step{false};      ///< Reference left / step input
  bool refr_dir{false};       ///< Reference right / direction input
  bool encb_dcen_cfg4{false}; ///< Encoder B / DCEN / CFG4
  bool enca_dcin_cfg5{false}; ///< Encoder A / DCIN / CFG5
  bool drv_enn{false};        ///< Driver enable (inverted)
  bool enc_n_dco_cfg6{false}; ///< Encoder N / DCO / CFG6
  bool sd_mode{false};        ///< SD_MODE pin (1=External step and dir source)
  bool swcomp_in{false};      ///< Software comparator input
  uint8_t version{0};         ///< IC version (should be 0x30 for TMC5160-TA)
};

/**
 * @brief Driver status enumeration
 *
 * Indicates the current status of the TMC5160 driver, including
 * error conditions and warnings.
 */
enum class DriverStatus : uint8_t {
  OK,        ///< No error condition
  CP_UV,     ///< Charge pump undervoltage
  S2VSA,     ///< Short to supply phase A
  S2VSB,     ///< Short to supply phase B
  S2GA,      ///< Short to ground phase A
  S2GB,      ///< Short to ground phase B
  OT,        ///< Overtemperature (error)
  OTHER_ERR, ///< GSTAT drv_err is set but none of the above conditions is found
  OTPW       ///< Overtemperature pre-warning
};

/**
 * @brief Motor direction enumeration
 */
enum class MotorDirection : uint8_t {
  NORMAL = 0, ///< Normal motor direction
  INVERSE = 1 ///< Inverse motor direction
};

/**
 * @brief Chip communication and motion control mode configuration
 *
 * This enumeration represents the combination of SPI_MODE and SD_MODE pins
 * that determine the TMC5160 operating mode. These pins are typically
 * hardwired and read at startup, but can be controlled via GPIO if connected.
 *
 * ⚠️ WARNING: Changing the mode requires a chip reset to take effect.
 * The mode pins are read at startup, so any changes must be followed by
 * a reset cycle (power cycle or reset pin).
 *
 * @note These modes correspond to the hardware pin configuration:
 * - SPI_MODE (pin 22): HIGH=SPI, LOW=UART
 * - SD_MODE (pin 21): HIGH=External step/dir, LOW=Internal ramp generator
 */
enum class ChipCommMode : uint8_t {
  SPI_INTERNAL_RAMP = 0, ///< SPI_MODE=HIGH, SD_MODE=LOW - SPI interface with internal ramp generator (motion controller)
  SPI_EXTERNAL_STEPDIR = 1, ///< SPI_MODE=HIGH, SD_MODE=HIGH - SPI interface with external step/dir inputs
  UART_INTERNAL_RAMP = 2 ///< SPI_MODE=LOW, SD_MODE=LOW - UART interface with internal ramp generator (motion controller)
};

/**
 * @brief Power stage parameters structure
 *
 * Configuration parameters for the power stage of the TMC5160 driver.
 */
struct PowerStageParameters {
  uint8_t drv_strength{2}; ///< MOSFET gate driver current (0 to 3)
  uint8_t bbm_time{0};     ///< Break Before Make duration specified in ns (0 to 24)
  uint8_t bbm_clks{4};     ///< Break Before Make duration specified in clock cycles (0 to 15)
  uint8_t otselect{0};     ///< Over temperature level selection for bridge disable (0-3)
  uint8_t filt_isense{0};  ///< Filter time constant of sense amplifier (0-3)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * All members use default member initializers, so constructor is empty.
   */
  PowerStageParameters() = default;
};

/**
 * @brief Motor parameters structure
 *
 * Configuration parameters for motor current and stealthChop operation.
 */
struct MotorParameters {
  uint16_t global_scaler{32};  ///< Global current scaling (32 to 256)
  uint8_t irun{16};            ///< Motor run current (0 to 31). For best performance don't set lower than 16
  uint8_t ihold{0};            ///< Standstill current (0 to 31). Set 70% of irun or lower
  uint8_t pwm_ofs_initial{30}; ///< Initial stealthChop PWM amplitude offset (0-255)
  uint8_t pwm_grad_initial{0}; ///< Initial stealthChop velocity dependent gradient for PWM amplitude

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * All members use default member initializers, so constructor is empty.
   */
  MotorParameters() = default;
};

/**
 * @brief Chopper configuration structure
 *
 * Configuration parameters for the chopper (current control) operation.
 */
struct ChopperConfig {
  uint8_t toff{5};   ///< Off time setting (0 to 15). 0 = Driver disabled
  uint8_t hstrt{4};  ///< Hysteresis start value (0 to 7)
  uint8_t hend{0};   ///< Hysteresis low value (0 to 15)
  uint8_t tbl{2};    ///< Comparator blank time (0 to 3)
  bool vsense{true}; ///< Resistor voltage sensitivity (false=Low, true=High)
  uint8_t mres{4};   ///< Microstep resolution (0=256, 1=128, ..., 8=fullstep)
  bool intpol{true}; ///< Enable interpolation to 256 microsteps
  bool dedge{false}; ///< Enable double edge step pulses
  bool chm{false};   ///< Chopper mode (false=spreadCycle, true=constant off time)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values for spreadCycle mode.
   * All members use default member initializers, so constructor is empty.
   */
  ChopperConfig() = default;
};

/**
 * @brief StealthChop configuration structure
 *
 * Configuration parameters for stealthChop PWM mode operation.
 */
struct StealthChopConfig {
  uint8_t pwm_ofs{30};      ///< PWM amplitude offset (0-255)
  uint8_t pwm_grad{0};      ///< PWM amplitude gradient (0-255)
  uint8_t pwm_freq{1};      ///< PWM frequency selection (0-3)
  bool pwm_autoscale{true}; ///< Enable PWM automatic amplitude scaling
  bool pwm_autograd{true};  ///< Enable PWM automatic gradient adaptation
  uint8_t pwm_reg{4};       ///< Regulation loop gradient (0-15)
  uint8_t pwm_lim{12};      ///< PWM automatic scale amplitude limit (0-15)
  PWMFreewheel freewheel{PWMFreewheel::NORMAL}; ///< Freewheeling mode when I_HOLD=0

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * All members use default member initializers, so constructor is empty.
   */
  StealthChopConfig() = default;
};

/**
 * @brief StallGuard configuration structure
 *
 * Configuration parameters for StallGuard2 stall detection.
 */
struct StallGuardConfig {
  int8_t sgt{0};      ///< StallGuard2 threshold value (-64 to 63, stored as 7-bit signed)
  uint8_t semin{0};   ///< Minimum stallGuard2 value for smart current control (0-15)
  uint8_t semax{0};   ///< StallGuard2 hysteresis value (0-15)
  uint8_t seup{0};    ///< Current increment step width (0-3)
  uint8_t sedn{0};    ///< Current decrement step speed (0-3)
  bool seimin{false}; ///< Minimum current for smart current control
  bool sfilt{false};  ///< Enable stallGuard2 filter

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * All members use default member initializers, so constructor is empty.
   */
  StallGuardConfig() = default;
};

/**
 * @brief Encoder configuration structure
 *
 * Configuration parameters for encoder interface operation.
 */
struct EncoderConfig {
  bool pol_a{false};           ///< Required A polarity for N channel event
  bool pol_b{false};           ///< Required B polarity for N channel event
  bool pol_n{true};            ///< Active polarity of N (false=low active, true=high active)
  bool ignore_ab{true};        ///< Ignore A and B polarity for N channel event
  bool clr_cont{false};        ///< Always latch or latch and clear X_ENC upon N event
  bool clr_once{false};        ///< Latch or latch and clear X_ENC on next N event
  uint8_t sensitivity{0};      ///< N channel event sensitivity (0-3)
  bool clr_enc_x{false};       ///< Clear encoder counter X_ENC upon N-event
  bool latch_x_act{false};     ///< Also latch XACTUAL position together with X_ENC
  bool enc_sel_decimal{false}; ///< Encoder prescaler divisor (false=binary,
                        ///< true=decimal)

  /**
   * @brief Default constructor
   *
   * Initializes with default values.
   * All members use default member initializers, so constructor is empty.
   */
  EncoderConfig() = default;
};

/**
 * @brief Short protection configuration structure
 *
 * Configuration parameters for short circuit detection.
 */
struct ShortProtectionConfig {
  uint8_t s2vs_level{6};  ///< Short to VS detector sensitivity (4 to 15, recommended 6-8)
  uint8_t s2g_level{6};   ///< Short to GND detector sensitivity (2 to 15, recommended 6-14)
  uint8_t shortfilter{1}; ///< Spike filtering bandwidth (0 to 3, default 1)
  uint8_t shortdelay{0};  ///< Short detection delay (0 to 1, default 0)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * All members use default member initializers, so constructor is empty.
   */
  ShortProtectionConfig() = default;
};

/**
 * @brief Mechanical system type enumeration
 *
 * Defines the type of mechanical system connected to the motor.
 */
enum class MechanicalSystemType : uint8_t {
  DirectDrive, ///< Direct drive (motor shaft directly connected)
  LeadScrew,   ///< Lead screw drive
  BeltDrive,   ///< Belt drive with pulleys
  Gearbox      ///< Gearbox reduction
};

/**
 * @brief Mechanical system configuration structure
 *
 * Configuration parameters for the mechanical system connected to the motor.
 * Used for unit conversions between physical units and steps.
 */
struct MechanicalSystem {
  MechanicalSystemType system_type{MechanicalSystemType::DirectDrive}; ///< Type of mechanical system
  float lead_screw_pitch_mm{0.0F};                                     ///< Lead screw pitch in mm (for LeadScrew)
  uint16_t belt_pulley_teeth{0}; ///< Number of teeth on motor pulley (for BeltDrive)
  float belt_pitch_mm{0.0F};     ///< Belt pitch in mm (for BeltDrive)
  float gear_ratio{1.0F};        ///< Gear ratio (output/input, for Gearbox)

  /**
   * @brief Default constructor
   *
   * Initializes with direct drive configuration.
   * All members use default member initializers, so constructor is empty.
   */
  MechanicalSystem() = default;
};

/**
 * @brief Motor specification structure
 *
 * High-level motor specifications for easy setup from physical parameters.
 * Used with SetupMotorFromSpec() to automatically configure the driver.
 */
struct MotorSpec {
  uint16_t steps_per_rev{200};         ///< Steps per revolution (typically 200 for 1.8° motors)
  uint16_t rated_current_ma{1500};     ///< Rated motor current in milliamps
  uint16_t rated_voltage_mv{12000};    ///< Rated motor voltage in millivolts
  uint32_t winding_resistance_mohm{0}; ///< Winding resistance in milliohms (optional, 0 = not specified)
  uint32_t winding_inductance_uh{0};   ///< Winding inductance in microhenries (optional, 0 = not specified)
  uint32_t holding_torque_mnm{0};      ///< Holding torque in milliNewton-meters (optional, 0 = not specified)

  /**
   * @brief Default constructor
   *
   * Initializes with common NEMA 17 motor defaults.
   * All members use default member initializers, so constructor is empty.
   */
  MotorSpec() = default;
};

/**
 * @brief Motor current configuration helper structure
 *
 * Configuration for motor current in physical units (milliamps).
 * Automatically calculates irun, ihold, and iholddelay values.
 */
struct MotorCurrentConfig {
  uint16_t run_current_ma{1500};      ///< Run current in milliamps
  uint16_t hold_current_ma{500};      ///< Hold current in milliamps
  uint16_t hold_current_delay_ms{10}; ///< Hold current delay in milliseconds

  /**
   * @brief Default constructor
   *
   * Initializes with default values.
   * All members use default member initializers, so constructor is empty.
   */
  MotorCurrentConfig() = default;
};

/**
 * @brief CoolStep configuration structure
 *
 * Configuration parameters for CoolStep current reduction feature.
 */
struct CoolStepConfig {
  uint8_t semin{0};   ///< Minimum StallGuard2 value for smart current control (0-15)
  uint8_t semax{0};   ///< StallGuard2 hysteresis value (0-15)
  uint8_t seup{0};    ///< Current increment step width (0-3)
  uint8_t sedn{0};    ///< Current decrement step speed (0-3)
  bool seimin{false}; ///< Minimum current for smart current control
  bool sfilt{false};  ///< Enable StallGuard2 filter

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * All members use default member initializers, so constructor is empty.
   */
  CoolStepConfig() = default;
};

/**
 * @brief Reference switch configuration structure
 *
 * Configuration parameters for reference switches/endstops.
 */
struct ReferenceSwitchConfig {
  bool stop_left_enable{false};     ///< Enable automatic motor stop on left switch
  bool stop_right_enable{false};    ///< Enable automatic motor stop on right switch
  bool pol_stop_left{false};        ///< Left switch polarity (true=inverted/low active)
  bool pol_stop_right{false};       ///< Right switch polarity (true=inverted/low active)
  bool swap_left_right{false};      ///< Swap left and right switch inputs
  bool latch_left_active{false};    ///< Latch position on active edge of left switch
  bool latch_left_inactive{false};  ///< Latch position on inactive edge of left switch
  bool latch_right_active{false};   ///< Latch position on active edge of right switch
  bool latch_right_inactive{false}; ///< Latch position on inactive edge of right switch
  bool en_latch_encoder{false};     ///< Latch encoder position on switch event
  bool en_softstop{true};           ///< Enable soft stop using deceleration ramp

  /**
   * @brief Default constructor
   *
   * Initializes with default values (switches disabled).
   * All members use default member initializers, so constructor is empty.
   */
  ReferenceSwitchConfig() = default;
};

/**
 * @brief dcStep configuration structure
 *
 * Configuration parameters for dcStep automatic commutation mode.
 */
struct DcStepConfig {
  float vdc_min{0.0F}; ///< Velocity threshold for enabling dcStep in steps/s (0.0F = disabled)
  uint16_t dc_time{0}; ///< Upper PWM on time limit for commutation (0-1023)
  uint8_t dc_sg{0};    ///< Max PWM on time for step loss detection (0-255)

  /**
   * @brief Default constructor
   *
   * Initializes with dcStep disabled.
   * All members use default member initializers, so constructor is empty.
   */
  DcStepConfig() = default;
};

/**
 * @brief Global configuration (GCONF) structure
 *
 * Configuration for global TMC5160 driver settings.
 * See GCONF_Register documentation for detailed bit descriptions and SD_MODE dependencies.
 */
struct GlobalConfig {
  bool recalibrate{false};      ///< Bit 0: Zero crossing recalibration during driver disable
  bool faststandstill{false};   ///< Bit 1: Standstill timeout (true=2^18 clocks, false=2^20 clocks)
  bool en_pwm_mode{true};       ///< Bit 2: Enable StealthChop voltage PWM mode (switch only in standstill
                                ///< at IHOLD=IRUN)
  bool multistep_filt{true};    ///< Bit 3: Enable step input filtering for StealthChop optimization
                       ///< (default=true recommended)
  bool shaft{false};            ///< Bit 4: Inverse motor direction (false=normal, true=inverse)
  bool diag0_error{false};      ///< Bit 5: (SD_MODE=1 only) Enable DIAG0 on driver errors (OT, S2G,
                                ///< UV_CP). Always shows reset-status.
  bool diag0_otpw{false};       ///< Bit 6: (SD_MODE=1 only) Enable DIAG0 on overtemperature prewarning
  bool diag0_stall_step{false}; ///< Bit 7: (SD_MODE=1) DIAG0 on stall. (SD_MODE=0) DIAG0 as STEP
                                ///< output (half frequency, dual edge)
  bool diag1_stall_dir{false};  ///< Bit 8: (SD_MODE=1) DIAG1 on stall. (SD_MODE=0) DIAG1 as DIR output
  bool diag1_index{false};      ///< Bit 9: (SD_MODE=1 only) Enable DIAG1 on index position (microstep LUT position 0)
  bool diag1_onstate{false};    ///< Bit 10: (SD_MODE=1 only) Enable DIAG1 when chopper is on (second
                                ///< half of fullstep coil)
  bool diag1_steps_skipped{false}; ///< Bit 11: (SD_MODE=1 only) Enable output toggle when steps skipped in dcStep mode.
                                   ///< Do not enable with other DIAG1 options.
  bool diag0_int_pushpull{false};  ///< Bit 12: SWN_DIAG0 output mode (false=open collector active
                                   ///< low, true=push pull active high)
  bool diag1_poscomp_pushpull{false}; ///< Bit 13: SWP_DIAG1 output mode (false=open collector
                                      ///< active low, true=push pull active high)
  bool small_hysteresis{false};       ///< Bit 14: Step frequency comparison hysteresis (false=1/16, true=1/32)
  bool stop_enable{false};            ///< Bit 15: Emergency stop: ENCA_DCIN stops sequencer when high (motor
                                      ///< goes to standstill)
  bool direct_mode{false};            ///< Bit 16: Direct coil control via XTARGET (bits 8..0=coil
                                      ///< A, 24..16=coil B, scaled by IHOLD)
  bool test_mode{false};              ///< Bit 17: Analog test output on ENCN_DCO (not for normal use, keep false)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   * All members use default member initializers, so constructor is empty.
   */
  GlobalConfig() = default;
};

/**
 * @brief Ramp parameters structure
 *
 * Additional ramp control parameters for fine-tuning motion profiles.
 */
struct RampParameters {
  uint8_t tpowerdown{20}; ///< Delay before power down (0-255, time range ~0 to 5.6 seconds)
  uint16_t tzerowait{0};  ///< Waiting time after ramping down to zero velocity (0-65535, time range
                          ///< ~0 to 2 seconds)
  float a1{0.0F};         ///< First acceleration between VSTART and V1 (steps/s², 0.0F = use AMAX)

  /**
   * @brief Default constructor
   *
   * Initializes with default values.
   * All members use default member initializers, so constructor is empty.
   */
  RampParameters() = default;
};

} // namespace tmc5160

#endif // TMC5160_TYPES_HPP
