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

//===============================================================================================================
//===============================================================================================================
//                                    ENUMERATIONS                                                
//===============================================================================================================
//===============================================================================================================

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
 * @brief Motor direction enumeration
 */
enum class MotorDirection : uint8_t {
  NORMAL = 0, ///< Normal motor direction
  INVERSE = 1 ///< Inverse motor direction
};

/**
 * @brief Unit enumeration
 *
 * Defines the unit of measurement for position, velocity, and acceleration.
 * Steps: Microsteps (driver native)
 * Rad: Radians (per second for velocity, per second^2 for accel)
 * Deg: Degrees (per second for velocity, per second^2 for accel)
 * Mm: Millimeters (linear only)
 * Rpm : Revolutions per Minute (Velocity only, typically)
 */
enum class Unit : uint8_t {
  Steps,      ///< Microsteps (driver native)
  Rad,        ///< Radians (per second for velocity, per second^2 for accel)
  Deg,        ///< Degrees (per second for velocity, per second^2 for accel)
  Mm,         ///< Millimeters (linear only)
  RPM         ///< Revolutions per Minute (Velocity only, typically)
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

//===============================================================================================================
//===============================================================================================================
//                                    RESPONSE STRUCTURES                                                
//===============================================================================================================
//===============================================================================================================

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

//===============================================================================================================
//===============================================================================================================
//                        HIGHER LEVEL - MECHANICAL SYSTEM CONFIGURATION STRUCTURES                                                
//===============================================================================================================
//===============================================================================================================

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

//===============================================================================================================
//===============================================================================================================
//                                    MOTOR SPECIFICATION STRUCTURE                                             
//===============================================================================================================
//===============================================================================================================

/**
 * @brief Motor specification structure
 *
 * High-level motor specifications for easy setup from physical parameters.
 * Includes motor electrical specs and driver hardware configuration (sense resistor, supply voltage).
 * Current settings (IRUN, IHOLD, GLOBAL_SCALER) can be automatically calculated from these parameters.
 *
 * @note If sense_resistor_mohm and supply_voltage_mv are set, current settings can be calculated
 *       automatically using CalculateMotorCurrent() from tmc5160_motor_calc.hpp
 */
/**
 * @brief Motor specification structure
 *
 * Contains motor physical parameters used to automatically calculate motor current settings
 * (IRUN, IHOLD, GLOBAL_SCALER) during initialization. Current settings are calculated
 * internally and not stored in this structure.
 */
struct MotorSpec {
  // Motor electrical specifications
  uint16_t steps_per_rev{200};         ///< Steps per revolution (typically 200 for 1.8° motors)
  uint16_t rated_current_ma{1500};     ///< Rated motor current in milliamps (RMS)
  uint16_t rated_voltage_mv{12000};    ///< Rated motor voltage in millivolts
  uint32_t winding_resistance_mohm{3}; ///< Winding resistance in milliohms (required for StealthChop lower limit calc)
  uint32_t winding_inductance_uh{0};  ///< Winding inductance in microhenries (optional, 0 = not specified)
  uint32_t holding_torque_mnm{0};      ///< Holding torque in milliNewton-meters (optional, 0 = not specified)

  // Desired current settings (used for calculation, not stored as register values)
  uint16_t run_current_ma{0};         ///< Desired run current in milliamps (0 = use rated_current_ma)
  uint16_t hold_current_ma{0};        ///< Desired hold current in milliamps (0 = auto-calculate as 30% of run)
  
  // Driver hardware configuration (required for automatic current calculation)
  uint32_t sense_resistor_mohm{50};    ///< Sense resistor value in milliohms (e.g., 50 for 0.05Ω, 0 = not specified)
  uint32_t supply_voltage_mv{24000};   ///< Motor supply voltage in millivolts (e.g., 24000 for 24V, 0 = not specified)

  /**
   * @brief Percentage adjustment for GLOBAL_SCALER calculation
   * 
   * Applied after GLOBAL_SCALER is calculated but before constraining to valid range (32-256).
   * Allows fine-tuning the calculated current scaling value.
   * 
   * Range: -50.0 to +50.0 (percentage)
   * Example: +10.0 means increase calculated GLOBAL_SCALER by 10%
   * 
   * @note Formula: adjusted_scaler = calculated_scaler * (1.0 + adjustment_percent / 100.0)
   * @note Default: 0.0 (no adjustment)
   */
  float scaler_adjustment_percent{0.0f};
  
  /**
   * @brief Percentage adjustment for IRUN calculation
   * 
   * Applied after IRUN is calculated but before constraining to valid range (0-31).
   * Allows fine-tuning the calculated run current.
   * 
   * Range: -50.0 to +50.0 (percentage)
   * Example: +5.0 means increase calculated IRUN by 5%
   * 
   * @note Formula: adjusted_irun = calculated_irun * (1.0 + adjustment_percent / 100.0)
   * @note Default: 0.0 (no adjustment)
   */
  float irun_adjustment_percent{0.0f};
  
  /**
   * @brief Percentage adjustment for IHOLD calculation
   * 
   * Applied after IHOLD is calculated but before constraining to valid range (0-31).
   * Allows fine-tuning the calculated hold current.
   * 
   * Range: -50.0 to +50.0 (percentage)
   * Example: -10.0 means decrease calculated IHOLD by 10%
   * 
   * @note Formula: adjusted_ihold = calculated_ihold * (1.0 + adjustment_percent / 100.0)
   * @note Default: 0.0 (no adjustment)
   */
  float ihold_adjustment_percent{0.0f};

  /**
   * @brief Default constructor
   *
   * Initializes with common NEMA 17 motor defaults.
   * Sense resistor defaults to 50mΩ (0.05Ω) which is common on TMC5160 boards.
   * Supply voltage defaults to 24V which is common for stepper motors.
   * Current settings (IRUN, IHOLD, GLOBAL_SCALER) are automatically calculated during initialization.
   */
  MotorSpec() = default;
};

//===============================================================================================================
//===============================================================================================================
//                              GATE DRIVER - POWER STAGE CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================

/**
 * @brief Sense amplifier filter time constant enumeration
 *
 * Filter time constant of sense amplifier to suppress ringing and coupling
 * from second coil operation. Increase setting if motor chopper noise occurs
 * due to cross-coupling of both coils.
 */
enum class SenseFilterTime : uint8_t {
  T100ns = 0, ///< ~100ns (reset default)
  T200ns = 1, ///< ~200ns
  T300ns = 2, ///< ~300ns
  T400ns = 3  ///< ~400ns
};

/**
 * @brief Over-temperature protection level enumeration
 *
 * Selects the over-temperature level for bridge disable.
 * Higher values provide more protection but trigger at lower temperatures.
 * 
 * @note Based on DRV_CONF register: OTSELECT bits
 */
enum class OverTempProtection : uint8_t {
  Level0 = 0, ///< 150°C threshold (lowest protection, highest temperature)
  Level1 = 1, ///< 143°C threshold
  Level2 = 2, ///< 136°C threshold (more protection)
  Level3 = 3  ///< 120°C threshold (highest protection, lowest temperature)
};

/**
 * @brief Power stage parameters structure
 *
 * Configuration parameters for the power stage of the TMC5160 driver.
 * Uses user-friendly physical parameters that are automatically converted
 * to register values during initialization.
 */
struct PowerStageParameters {
  /**
   * @brief MOSFET Miller charge in nanocoulombs (used to calculate DRVSTRENGTH)
   * 
   * 0 = auto-calculate from default (10nC for small MOSFETs).
   * Based on datasheet table 3.3:
   * - <10nC: DRVSTRENGTH = 0
   * - 10-20nC: DRVSTRENGTH = 0 or 1
   * - 20-40nC: DRVSTRENGTH = 1 or 2
   * - 40-60nC: DRVSTRENGTH = 2 or 3
   * - >60nC: DRVSTRENGTH = 3
   * 
   * Typical values:
   * - Small MOSFETs (e.g., BSC072N08NS5): ~6nC
   * - Medium MOSFETs: ~30nC
   * - Large MOSFETs: ~50nC+
   * 
   * @note Constrained to valid range: >= 0.0f
   */
  float mosfet_miller_charge_nc{10.0f};
  
  /**
   * @brief Break Before Make time in nanoseconds
   * 
   * 0 = auto-calculate from MOSFET Miller charge (100ns for small MOSFETs, lowest setting).
   * Minimum: 75-100ns (BBMTIME=0, shortest, reset default).
   * 
   * BBMTIME register (bits 4..0, range 0-24):
   * - BBMTIME=0: 75-100ns (shortest, typical 100ns)
   * - BBMTIME=16: 200ns
   * - BBMTIME=24: 375-500ns (longest, typical 375ns)
   * - Formula: time[ns] ≈ 100ns * 32 / (32 - BBMTIME)
   * - Used for times <= 200ns
   * 
   * BBMCLKS register (bits 11..8, range 0-15):
   * - Digital BBM time in clock cycles (typ. 83ns per cycle at 12MHz)
   * - Used for times > 200ns
   * - The longer setting rules (BBMTIME vs. BBMCLKS)
   * 
   * Datasheet hint: Choose the lowest setting safely covering the switching event.
   * Add roughly 30% headroom to cover production stray of MOSFETs and driver.
   * 
   * Typical values:
   * - Fast MOSFETs: 100ns (lowest, BBMTIME=0)
   * - Medium MOSFETs: 200ns (BBMTIME=16)
   * - Large MOSFETs: 200-400ns (BBMCLKS)
   * 
   * @note Constrained to valid range: >= 0
   */
  uint32_t bbm_time_ns{100};
  
  /**
   * @brief Sense amplifier filter time constant
   * 
   * 0 = 100ns (default)
   * 1 = 200ns
   * 2 = 300ns
   * 3 = 400ns
   */
  SenseFilterTime sense_filter{SenseFilterTime::T100ns};
  
  /**
   * @brief Over-temperature protection level for bridge disable
   * 
   * 0 = 150°C (default)
   * 1 = 143°C
   * 2 = 136°C
   * 3 = 120°C
   */
  OverTempProtection over_temp_protection{OverTempProtection::Level0};

  /**
   * @brief Short to VS detector voltage threshold in millivolts
   * 
   * 0 = auto-calculate to recommended default (625mV, equivalent to S2VS_LEVEL=6).
   * Detects voltage drop in LS MOSFET and sense resistor (VBM).
   * 
   * Datasheet voltage thresholds (typical values):
   * - S2VS_LEVEL=6: 550-625-700mV (recommended for normal operation)
   * - S2VS_LEVEL=15: 1400-1560-1720mV (lowest sensitivity)
   * 
   * Can be set sensitively to detect overcurrent (150-200% of nominal peak current).
   * Recommended: 550-700mV for normal operation, down to 400mV at low current scale.
   * 
   * @note CHOPCONF.diss2vs can disable this protection (handled in ChopperConfig).
   * @note Constrained to valid range: 0 (auto) or 400-2000mV
   * @note Register value (S2VS_LEVEL) is automatically calculated from voltage threshold
   */
  uint16_t s2vs_voltage_mv{0};
  
  /**
   * @brief Short to GND detector voltage threshold in millivolts
   * 
   * 0 = auto-calculate to recommended default (625mV for VS<50V, equivalent to S2G_LEVEL=6).
   * Detects voltage drop on high-side MOSFET (VS - VBM).
   * 
   * Datasheet voltage thresholds (typical values, depends on supply voltage):
   * - S2G_LEVEL=6 (VS<50V): 460-625-800mV (recommended for normal operation)
   * - S2G_LEVEL=15 (VS<52V): 1200-1560-1900mV
   * - S2G_LEVEL=15 (VS<55V): 850mV (minimum for VS>52V to prevent false triggers)
   * 
   * Recommended: 460-800mV for normal operation (VS<50V).
   * Minimum 1200mV if bridge supply voltage can exceed 52V (prevents false triggers at 55V).
   * 
   * @note CHOPCONF.diss2g can disable this protection (handled in ChopperConfig).
   * @note Constrained to valid range: 0 (auto) or 400-2000mV
   * @note Register value (S2G_LEVEL) is automatically calculated from voltage threshold
   */
  uint16_t s2g_voltage_mv{0};
  
  /**
   * @brief Spike filtering bandwidth for short detection (0-3)
   * 
   * 0 = lowest (100ns), 1 = 1µs (reset default), 2 = 2µs, 3 = 3µs.
   * Good PCB layout allows setting 0, increase if erroneous detection occurs.
   * Helps prevent spurious triggering from PCB layout or long motor cables.
   * 
   * @note Constrained to valid range: 0-3
   */
  uint8_t shortfilter{1};
  
  /**
   * @brief Short detection delay in microseconds (0.1µs resolution)
   * 
   * 0 = auto-calculate to recommended default (0.85µs, equivalent to shortdelay=0).
   * Total detection delay including 100ns filtering time.
   * Should cover bridge switching time.
   * 
   * Datasheet timing (typical values):
   * - shortdelay=0: 0.5-0.85-1.1µs (normal, recommended for most applications)
   * - shortdelay=1: 1.1-1.6-2.2µs (high delay)
   * 
   * Recommended: 5-11 (0.5-1.1µs) for most applications.
   * Stored as 0.1µs units (e.g., 8 = 0.8µs, 16 = 1.6µs).
   * 
   * @note Constrained to valid range: 0 (auto) or 5-25 (0.5-2.5µs)
   * @note Register value (shortdelay bit) is automatically calculated from delay time
   */
  uint8_t short_detection_delay_us_x10{0};

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values:
   * - MOSFET Miller charge: 10nC (typical for small MOSFETs)
   * - BBM time: 100ns (typical minimum)
   * - Sense filter: 100ns (reset default)
   * - Over-temperature protection: Level0 (150°C)
   * - Short protection: s2vs_voltage_mv=0 (auto=625mV), s2g_voltage_mv=0 (auto=625mV), shortfilter=1, short_detection_delay_us_x10=0 (auto=0.85µs)
   * 
   * @note Short detection retries 3 times before switching off motor continuously (ESD protection).
   * @note Once short detected, corresponding bridge (A or B) switches off and flag is set.
   * @note To restart after short: disable and re-enable driver.
   * @note Short protection cannot protect against all possible short events - short circuits should be avoided by design.
   */
  PowerStageParameters() = default;
};

//===============================================================================================================
//===============================================================================================================
//                                 CHOPPER AND STALLING CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================            

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

//===============================================================================================================
//===============================================================================================================
//                                 CHOPPER CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================            

/**
 * @brief Ramp configuration structure
 *
 * Configuration parameters for the ramp generator.
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

//===============================================================================================================
//===============================================================================================================
//                          ENCODER & LIMIT SWITCHES FEEDBACK CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================

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

//===============================================================================================================
//===============================================================================================================
//                                DIAGNOSTIC PIN CONFIGURATION STRUCTURES                                          
//===============================================================================================================
//===============================================================================================================

/**
 * @brief DIAG0 pin configuration structure
 *
 * Configuration for DIAG0 diagnostic output pin functionality.
 * DIAG0 pin behavior depends on SD_MODE setting:
 * - SD_MODE=1 (External step/dir): Diagnostic outputs (error, otpw, stall)
 * - SD_MODE=0 (Internal ramp): Can be used as STEP output
 *
 * @note Always shows reset-status (active low during reset condition).
 */
struct Diag0Config {
  bool error{false};      ///< Bit 5: Enable DIAG0 on driver errors (OT, S2G, UV_CP) - SD_MODE=1 only
  bool otpw{false};       ///< Bit 6: Enable DIAG0 on overtemperature prewarning - SD_MODE=1 only
  bool stall_step{false}; ///< Bit 7: (SD_MODE=1) DIAG0 on stall, (SD_MODE=0) DIAG0 as STEP output (half frequency, dual edge)
  bool pushpull{false};   ///< Bit 12: Output mode (false=open collector active low, true=push pull active high)

  /**
   * @brief Default constructor
   *
   * Initializes with all diagnostic features disabled (default).
   */
  Diag0Config() = default;
};

/**
 * @brief DIAG1 pin configuration structure
 *
 * Configuration for DIAG1 diagnostic output pin functionality.
 * DIAG1 pin behavior depends on SD_MODE setting:
 * - SD_MODE=1 (External step/dir): Diagnostic outputs (stall, index, onstate, steps_skipped)
 * - SD_MODE=0 (Internal ramp): Can be used as DIR output or position compare signal
 *
 * @note steps_skipped should not be enabled with other DIAG1 options (mutually exclusive).
 */
struct Diag1Config {
  bool stall_dir{false};      ///< Bit 8: (SD_MODE=1) DIAG1 on stall, (SD_MODE=0) DIAG1 as DIR output
  bool index{false};          ///< Bit 9: Enable DIAG1 on index position (microstep LUT position 0) - SD_MODE=1 only
  bool onstate{false};        ///< Bit 10: Enable DIAG1 when chopper is on (second half of fullstep) - SD_MODE=1 only
  bool steps_skipped{false};  ///< Bit 11: Enable output toggle when steps skipped in dcStep mode - SD_MODE=1 only
  bool pushpull{false};       ///< Bit 13: Output mode (false=open collector active low, true=push pull active high)

  /**
   * @brief Default constructor
   *
   * Initializes with all diagnostic features disabled (default).
   * 
   * @note steps_skipped should not be enabled with other DIAG1 options.
   */
  Diag1Config() = default;
};

//===============================================================================================================
//===============================================================================================================
//                                GLOBAL CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================

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

  /**
   * @brief Extract DIAG0 configuration from GlobalConfig
   * @return Diag0Config struct with current DIAG0 settings
   */
  [[nodiscard]] Diag0Config GetDiag0Config() const noexcept {
    Diag0Config diag0;
    diag0.error = diag0_error;
    diag0.otpw = diag0_otpw;
    diag0.stall_step = diag0_stall_step;
    diag0.pushpull = diag0_int_pushpull;
    return diag0;
  }

  /**
   * @brief Extract DIAG1 configuration from GlobalConfig
   * @return Diag1Config struct with current DIAG1 settings
   */
  [[nodiscard]] Diag1Config GetDiag1Config() const noexcept {
    Diag1Config diag1;
    diag1.stall_dir = diag1_stall_dir;
    diag1.index = diag1_index;
    diag1.onstate = diag1_onstate;
    diag1.steps_skipped = diag1_steps_skipped;
    diag1.pushpull = diag1_poscomp_pushpull;
    return diag1;
  }

  /**
   * @brief Set DIAG0 configuration in GlobalConfig
   * @param config Diag0Config struct with DIAG0 settings
   */
  void SetDiag0Config(const Diag0Config& config) noexcept {
    diag0_error = config.error;
    diag0_otpw = config.otpw;
    diag0_stall_step = config.stall_step;
    diag0_int_pushpull = config.pushpull;
  }

  /**
   * @brief Set DIAG1 configuration in GlobalConfig
   * @param config Diag1Config struct with DIAG1 settings
   */
  void SetDiag1Config(const Diag1Config& config) noexcept {
    diag1_stall_dir = config.stall_dir;
    diag1_index = config.index;
    diag1_onstate = config.onstate;
    diag1_steps_skipped = config.steps_skipped;
    diag1_poscomp_pushpull = config.pushpull;
  }
};

//===============================================================================================================
//===============================================================================================================

} // namespace tmc5160

#endif // TMC5160_TYPES_HPP
