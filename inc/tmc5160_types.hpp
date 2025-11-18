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

namespace tmc5160 {

/**
 * @brief Driver status enumeration
 *
 * Indicates the current status of the TMC5160 driver, including
 * error conditions and warnings.
 */
enum class DriverStatus {
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
enum class MotorDirection {
  NORMAL = 0, ///< Normal motor direction
  INVERSE = 1 ///< Inverse motor direction
};

/**
 * @brief Power stage parameters structure
 *
 * Configuration parameters for the power stage of the TMC5160 driver.
 */
struct PowerStageParameters {
  uint8_t drv_strength; ///< MOSFET gate driver current (0 to 3)
  uint8_t bbm_time; ///< Break Before Make duration specified in ns (0 to 24)
  uint8_t bbm_clks; ///< Break Before Make duration specified in clock cycles (0
                    ///< to 15)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  PowerStageParameters() : drv_strength(2), bbm_time(0), bbm_clks(4) {}
};

/**
 * @brief Motor parameters structure
 *
 * Configuration parameters for motor current and stealthChop operation.
 */
struct MotorParameters {
  uint16_t global_scaler; ///< Global current scaling (32 to 256)
  uint8_t irun; ///< Motor run current (0 to 31). For best performance don't set
                ///< lower than 16
  uint8_t ihold; ///< Standstill current (0 to 31). Set 70% of irun or lower
  uint8_t pwm_ofs_initial; ///< Initial stealthChop PWM amplitude offset (0-255)
  uint8_t pwm_grad_initial; ///< Initial stealthChop velocity dependent gradient
                            ///< for PWM amplitude

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  MotorParameters()
      : global_scaler(32), irun(16), ihold(0), pwm_ofs_initial(30),
        pwm_grad_initial(0) {}
};

/**
 * @brief Chopper configuration structure
 *
 * Configuration parameters for the chopper (current control) operation.
 */
struct ChopperConfig {
  uint8_t toff;  ///< Off time setting (0 to 15). 0 = Driver disabled
  uint8_t hstrt; ///< Hysteresis start value (0 to 7)
  uint8_t hend;  ///< Hysteresis low value (0 to 15)
  uint8_t tbl;   ///< Comparator blank time (0 to 3)
  bool vsense;   ///< Resistor voltage sensitivity (false=Low, true=High)
  uint8_t mres;  ///< Microstep resolution (0=256, 1=128, ..., 8=fullstep)
  bool intpol;   ///< Enable interpolation to 256 microsteps
  bool dedge;    ///< Enable double edge step pulses
  bool chm;      ///< Chopper mode (false=spreadCycle, true=constant off time)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values for spreadCycle mode.
   */
  ChopperConfig()
      : toff(5), hstrt(4), hend(0), tbl(2), vsense(true), mres(4), intpol(true),
        dedge(false), chm(false) {}
};

/**
 * @brief StealthChop configuration structure
 *
 * Configuration parameters for stealthChop PWM mode operation.
 */
struct StealthChopConfig {
  uint8_t pwm_ofs;    ///< PWM amplitude offset (0-255)
  uint8_t pwm_grad;   ///< PWM amplitude gradient (0-255)
  uint8_t pwm_freq;   ///< PWM frequency selection (0-3)
  bool pwm_autoscale; ///< Enable PWM automatic amplitude scaling
  bool pwm_autograd;  ///< Enable PWM automatic gradient adaptation
  uint8_t pwm_reg;    ///< Regulation loop gradient (0-15)
  uint8_t pwm_lim;    ///< PWM automatic scale amplitude limit (0-15)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  StealthChopConfig()
      : pwm_ofs(30), pwm_grad(0), pwm_freq(1), pwm_autoscale(true),
        pwm_autograd(true), pwm_reg(4), pwm_lim(12) {}
};

/**
 * @brief StallGuard configuration structure
 *
 * Configuration parameters for StallGuard2 stall detection.
 */
struct StallGuardConfig {
  uint8_t
      sgt; ///< StallGuard2 threshold value (-64 to 63, stored as 7-bit signed)
  uint8_t semin; ///< Minimum stallGuard2 value for smart current control (0-15)
  uint8_t semax; ///< StallGuard2 hysteresis value (0-15)
  uint8_t seup;  ///< Current increment step width (0-3)
  uint8_t sedn;  ///< Current decrement step speed (0-3)
  bool seimin;   ///< Minimum current for smart current control
  bool sfilt;    ///< Enable stallGuard2 filter

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  StallGuardConfig()
      : sgt(0), semin(0), semax(0), seup(0), sedn(0), seimin(false),
        sfilt(false) {}
};

/**
 * @brief Encoder configuration structure
 *
 * Configuration parameters for encoder interface operation.
 */
struct EncoderConfig {
  bool pol_a;     ///< Required A polarity for N channel event
  bool pol_b;     ///< Required B polarity for N channel event
  bool pol_n;     ///< Active polarity of N (false=low active, true=high active)
  bool ignore_ab; ///< Ignore A and B polarity for N channel event
  bool clr_cont;  ///< Always latch or latch and clear X_ENC upon N event
  bool clr_once;  ///< Latch or latch and clear X_ENC on next N event
  uint8_t sensitivity;  ///< N channel event sensitivity (0-3)
  bool clr_enc_x;       ///< Clear encoder counter X_ENC upon N-event
  bool latch_x_act;     ///< Also latch XACTUAL position together with X_ENC
  bool enc_sel_decimal; ///< Encoder prescaler divisor (false=binary,
                        ///< true=decimal)

  /**
   * @brief Default constructor
   *
   * Initializes with default values.
   */
  EncoderConfig()
      : pol_a(false), pol_b(false), pol_n(true), ignore_ab(true),
        clr_cont(false), clr_once(false), sensitivity(0), clr_enc_x(false),
        latch_x_act(false), enc_sel_decimal(false) {}
};

/**
 * @brief Short protection configuration structure
 *
 * Configuration parameters for short circuit detection.
 */
struct ShortProtectionConfig {
  uint8_t s2vs_level;  ///< Short to VS detector sensitivity (4 to 15,
                       ///< recommended 6-8)
  uint8_t s2g_level;   ///< Short to GND detector sensitivity (2 to 15,
                       ///< recommended 6-14)
  uint8_t shortfilter; ///< Spike filtering bandwidth (0 to 3, default 1)
  uint8_t shortdelay;  ///< Short detection delay (0 to 1, default 0)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  ShortProtectionConfig()
      : s2vs_level(6), s2g_level(6), shortfilter(1), shortdelay(0) {}
};

/**
 * @brief Mechanical system type enumeration
 *
 * Defines the type of mechanical system connected to the motor.
 */
enum class MechanicalSystemType {
  DirectDrive,  ///< Direct drive (motor shaft directly connected)
  LeadScrew,    ///< Lead screw drive
  BeltDrive,    ///< Belt drive with pulleys
  Gearbox       ///< Gearbox reduction
};

/**
 * @brief Mechanical system configuration structure
 *
 * Configuration parameters for the mechanical system connected to the motor.
 * Used for unit conversions between physical units and steps.
 */
struct MechanicalSystem {
  MechanicalSystemType system_type; ///< Type of mechanical system
  float lead_screw_pitch_mm;        ///< Lead screw pitch in mm (for LeadScrew)
  uint16_t belt_pulley_teeth;       ///< Number of teeth on motor pulley (for BeltDrive)
  float belt_pitch_mm;              ///< Belt pitch in mm (for BeltDrive)
  float gear_ratio;                 ///< Gear ratio (output/input, for Gearbox)

  /**
   * @brief Default constructor
   *
   * Initializes with direct drive configuration.
   */
  MechanicalSystem()
      : system_type(MechanicalSystemType::DirectDrive), lead_screw_pitch_mm(0.0f),
        belt_pulley_teeth(0), belt_pitch_mm(0.0f), gear_ratio(1.0f) {}
};

/**
 * @brief Motor specification structure
 *
 * High-level motor specifications for easy setup from physical parameters.
 * Used with SetupMotorFromSpec() to automatically configure the driver.
 */
struct MotorSpec {
  uint16_t steps_per_rev;      ///< Steps per revolution (typically 200 for 1.8° motors)
  uint16_t rated_current_ma;   ///< Rated motor current in milliamps
  uint16_t rated_voltage_mv;   ///< Rated motor voltage in millivolts
  uint32_t winding_resistance_mohm; ///< Winding resistance in milliohms (optional, 0 = not specified)
  uint32_t winding_inductance_uh;   ///< Winding inductance in microhenries (optional, 0 = not specified)
  uint32_t holding_torque_mnm;      ///< Holding torque in milliNewton-meters (optional, 0 = not specified)

  /**
   * @brief Default constructor
   *
   * Initializes with common NEMA 17 motor defaults.
   */
  MotorSpec()
      : steps_per_rev(200), rated_current_ma(1500), rated_voltage_mv(12000),
        winding_resistance_mohm(0), winding_inductance_uh(0),
        holding_torque_mnm(0) {}
};

/**
 * @brief Motor current configuration helper structure
 *
 * Configuration for motor current in physical units (milliamps).
 * Automatically calculates irun, ihold, and iholddelay values.
 */
struct MotorCurrentConfig {
  uint16_t run_current_ma;     ///< Run current in milliamps
  uint16_t hold_current_ma;    ///< Hold current in milliamps
  uint16_t hold_current_delay_ms; ///< Hold current delay in milliseconds

  /**
   * @brief Default constructor
   *
   * Initializes with default values.
   */
  MotorCurrentConfig()
      : run_current_ma(1500), hold_current_ma(500), hold_current_delay_ms(10) {}
};

/**
 * @brief CoolStep configuration structure
 *
 * Configuration parameters for CoolStep current reduction feature.
 */
struct CoolStepConfig {
  uint8_t semin;  ///< Minimum StallGuard2 value for smart current control (0-15)
  uint8_t semax;  ///< StallGuard2 hysteresis value (0-15)
  uint8_t seup;   ///< Current increment step width (0-3)
  uint8_t sedn;   ///< Current decrement step speed (0-3)
  bool seimin;    ///< Minimum current for smart current control
  bool sfilt;     ///< Enable StallGuard2 filter

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values.
   */
  CoolStepConfig()
      : semin(0), semax(0), seup(0), sedn(0), seimin(false), sfilt(false) {}
};

/**
 * @brief Reference switch configuration structure
 *
 * Configuration parameters for reference switches/endstops.
 */
struct ReferenceSwitchConfig {
  bool stop_left_enable;      ///< Enable automatic motor stop on left switch
  bool stop_right_enable;     ///< Enable automatic motor stop on right switch
  bool pol_stop_left;         ///< Left switch polarity (true=inverted/low active)
  bool pol_stop_right;        ///< Right switch polarity (true=inverted/low active)
  bool swap_left_right;       ///< Swap left and right switch inputs
  bool latch_left_active;     ///< Latch position on active edge of left switch
  bool latch_left_inactive;   ///< Latch position on inactive edge of left switch
  bool latch_right_active;    ///< Latch position on active edge of right switch
  bool latch_right_inactive;  ///< Latch position on inactive edge of right switch
  bool en_latch_encoder;      ///< Latch encoder position on switch event
  bool en_softstop;           ///< Enable soft stop using deceleration ramp

  /**
   * @brief Default constructor
   *
   * Initializes with default values (switches disabled).
   */
  ReferenceSwitchConfig()
      : stop_left_enable(false), stop_right_enable(false),
        pol_stop_left(false), pol_stop_right(false), swap_left_right(false),
        latch_left_active(false), latch_left_inactive(false),
        latch_right_active(false), latch_right_inactive(false),
        en_latch_encoder(false), en_softstop(true) {}
};

/**
 * @brief dcStep configuration structure
 *
 * Configuration parameters for dcStep automatic commutation mode.
 */
struct DcStepConfig {
  float vdc_min; ///< Velocity threshold for enabling dcStep in steps/s (0.0f = disabled)

  /**
   * @brief Default constructor
   *
   * Initializes with dcStep disabled.
   */
  DcStepConfig() : vdc_min(0.0f) {}
};

} // namespace tmc5160

#endif // TMC5160_TYPES_HPP
