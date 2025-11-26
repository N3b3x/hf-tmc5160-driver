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
  Temp150C = 0, ///< 150°C threshold (lowest protection, highest temperature)
  Temp143C = 1, ///< 143°C threshold
  Temp136C = 2, ///< 136°C threshold (more protection)
  Temp120C = 3  ///< 120°C threshold (highest protection, lowest temperature)
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
  OverTempProtection over_temp_protection{OverTempProtection::Temp150C};

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
   * - Over-temperature protection: Temp150C (150°C)
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
//                               CHOPPER CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================

/**
 * @brief Chopper mode enumeration
 *
 * Selects between SpreadCycle (patented high-performance algorithm) and Classic constant off-time chopper.
 */
enum class ChopperMode : bool {
  SPREAD_CYCLE = false,  ///< SpreadCycle mode (recommended) - superior microstepping quality
  CLASSIC = true         ///< Classic constant off-time mode - alternative chopper algorithm
};

/**
 * @brief Comparator blank time enumeration
 *
 * Blank time masks comparator input to block spikes from parasitic capacitances during switching.
 * Longer blank time provides more protection but may limit chopper frequency.
 * 
 * @note For most applications, TBL_24CLK or TBL_36CLK is good.
 * @note For highly capacitive loads (e.g., filter networks), TBL_36CLK or TBL_54CLK may be required.
 */
enum class ChopperBlankTime : uint8_t {
  TBL_16CLK = 0,  ///< 16 clock cycles (~1.33µs @ 12MHz)
  TBL_24CLK = 1,  ///< 24 clock cycles (~2.0µs @ 12MHz)
  TBL_36CLK = 2,  ///< 36 clock cycles (~3.0µs @ 12MHz, typical)
  TBL_54CLK = 3   ///< 54 clock cycles (~4.5µs @ 12MHz, for high capacitive loads)
};

/**
 * @brief Microstep resolution enumeration
 *
 * Defines the number of microsteps per full step.
 * Higher resolution provides smoother motion but may reduce maximum velocity.
 * 
 * @note Interpolation can be enabled to extrapolate to 256 microsteps regardless of MRES setting.
 */
enum class MicrostepResolution : uint8_t {
  MRES_256 = 0,   ///< 256 microsteps per full step (highest resolution)
  MRES_128 = 1,   ///< 128 microsteps per full step
  MRES_64 = 2,    ///< 64 microsteps per full step
  MRES_32 = 3,    ///< 32 microsteps per full step
  MRES_16 = 4,    ///< 16 microsteps per full step (typical)
  MRES_8 = 5,     ///< 8 microsteps per full step
  MRES_4 = 6,     ///< 4 microsteps per full step
  MRES_2 = 7,     ///< 2 microsteps per full step
  FULLSTEP = 8   ///< Full step (no microstepping)
};

/**
 * @brief Chopper configuration structure
 *
 * User-friendly configuration for SpreadCycle and Classic chopper modes.
 * SpreadCycle is a cycle-by-cycle current control providing superior microstepping quality.
 * Classic mode is an alternative constant off-time chopper algorithm.
 *
 * The chopper frequency is an important parameter. Most motors work optimally in 16-30kHz range.
 * Frequency is influenced by TOFF, TBL, hysteresis settings, motor inductance, and supply voltage.
 *
 * @note TOFF=0 completely disables all driver transistors (motor can free-wheel).
 * @note For StealthChop operation, TOFF is required to enable motor but not used for chopping.
 * @note TOFF=1 requires TBL ≥ 2.
 * @note SpreadCycle mode (recommended) provides superior microstepping quality with default settings.
 *
 * @see Datasheet section 8: SpreadCycle and Classic Chopper
 */
struct ChopperConfig {
  /**
   * @brief Chopper mode selection
   * 
   * - SPREAD_CYCLE (recommended): Patented high-performance algorithm with superior microstepping quality.
   *   Automatically determines optimum fast-decay phase length. Provides best results with default settings.
   * - CLASSIC: Constant off-time chopper mode. Alternative algorithm, requires more tuning.
   * 
   * @note SpreadCycle cycles: on → slow decay → fast decay → slow decay
   * @note Classic cycles: on → fast decay → slow decay
   */
  ChopperMode mode{ChopperMode::SPREAD_CYCLE};  ///< Chopper mode (SPREAD_CYCLE recommended)
  
  /**
   * @brief Off time (slow decay time) setting
   * 
   * Sets the slow decay time and limits maximum chopper frequency.
   * Higher values increase slow decay time and reduce chopper frequency.
   * 
   * Range: 0-15
   * - 0: Chopper off (driver disabled, motor can free-wheel)
   * - 1: Minimum off time (requires TBL ≥ 2)
   * - 2-15: Off time setting (typical: 3-5 for 16-30kHz chopper frequency)
   * 
   * Calculation for target chopper frequency:
   * t_OFF = (1 / f_chopper) * (slow_decay_percent / 100) * (1 / 2)
   * TOFF = (t_OFF * f_CLK - 12) / 32
   * 
   * Example (25kHz target, 50% slow decay, 12MHz clock):
   * t_OFF = (1/25000) * 0.5 * 0.5 = 10µs
   * TOFF = (10µs * 12MHz - 12) / 32 ≈ 3.0 → use 3
   * 
   * @note For StealthChop operation, any setting is OK (not used for chopping).
   * @note Higher motor velocities may benefit from TOFF=2 or 3 with short TBL=1 or 2.
   */
  uint8_t toff{5};  ///< Off time setting (0=disabled, 1-15=off time, 5=typical)
  
  /**
   * @brief Comparator blank time
   * 
   * Masks comparator input to block spikes from parasitic capacitances during switching.
   * Typically 1-2 microseconds. Longer blank time provides more protection but may limit frequency.
   * 
   * Can be specified as:
   * - uint8_t (0-3): Raw register value
   * - ChopperBlankTime enum: More intuitive (recommended)
   * 
   * Recommended: TBL_36CLK (~3.0µs @ 12MHz) for most applications.
   * 
   * @note For highly capacitive loads (filter networks), use TBL_36CLK or TBL_54CLK.
   * @note Lower blank time allows lower StealthChop current limit.
   */
  uint8_t tbl{2};  ///< Blank time (0-3 or ChopperBlankTime enum, 2=typical)
  
  /**
   * @brief Hysteresis start value (SpreadCycle mode only)
   * 
   * Adds to hysteresis end value to create effective hysteresis start.
   * Forces minimum current ripple into motor coils for best microstepping results.
   * 
   * Range: 0-7 (adds 1-8 to HEND)
   * - Lower values: Less current ripple (may reduce microstep accuracy)
   * - Higher values: More current ripple (may increase chopper noise)
   * - Typical: 4 (effective hysteresis = 4 when HEND=0)
   * 
   * Effective hysteresis = HSTRT + HEND (with encoding)
   * 
   * @note Only used in SpreadCycle mode (chm=0).
   * @note In Classic mode, this field controls fast decay time (TFD) instead.
   * @note Too low: Reduced microstep accuracy, humming/vibration at medium velocities.
   * @note Too high: Reduced chopper frequency, increased noise, no benefit.
   * @note Start from low setting (HSTRT=0, HEND=0) and increase until motor runs smoothly.
   */
  uint8_t hstrt{4};  ///< Hysteresis start (0-7, adds 1-8 to HEND, 4=typical)
  
  /**
   * @brief Hysteresis end value (SpreadCycle mode) or Sine wave offset (Classic mode)
   * 
   * **SpreadCycle mode (chm=0)**: Hysteresis end value after decrements.
   * - Encoded: 0-2 = negative (-3 to -1), 3 = zero, 4-15 = positive (1 to 12)
   * - Effective hysteresis = HSTRT + HEND (with encoding)
   * - Typical: 0 (effective end = -3, start = HSTRT+1-3 = HSTRT-2)
   * 
   * **Classic mode (chm=1)**: Sine wave offset for zero crossing correction.
   * - Encoded: 0-2 = negative (-3 to -1), 3 = zero, 4-15 = positive (1 to 12)
   * - Positive offset corrects for zero crossing error (typically required)
   * - Typical: 4-6 (positive offset 1-3) for smoothest operation
   * 
   * @note Sum HSTRT+HEND must be ≤16 for SpreadCycle (at CS ≤ 30).
   * @note HSTRT=0 and HEND=0 disables hysteresis (not recommended).
   */
  uint8_t hend{0};  ///< Hysteresis end (SpreadCycle) or offset (Classic), encoded 0-15
  
  /**
   * @brief Fast decay time (Classic mode only)
   * 
   * Sets fixed fast decay time following each on phase in Classic mode.
   * Should be long enough to follow falling slope but not cause excess ripple.
   * 
   * Range: 0-15 (uses hstrt_tfd bits + tfd_3 bit)
   * - 0: Slow decay only
   * - 1-15: Fast decay time duration
   * - Typical: Similar to slow decay time (TOFF) setting
   * 
   * @note Only used in Classic mode (chm=1).
   * @note In SpreadCycle mode, this field is controlled by hstrt instead.
   * @note Tune using oscilloscope or motor smoothness at different velocities.
   */
  uint8_t tfd{0};  ///< Fast decay time (Classic mode only, 0-15, 0=slow decay only)
  
  /**
   * @brief Disable fast decay comparator (Classic mode only)
   * 
   * Controls whether current comparator can terminate fast decay cycle.
   * 
   * - false: Enable comparator termination (fast decay ends when current reaches higher negative value)
   * - true: End fast decay by time only (fixed duration)
   * 
   * @note Only used in Classic mode (chm=1).
   * @note In SpreadCycle mode, this field is not used.
   */
  bool disfdcc{false};  ///< Disable fast decay comparator (Classic mode only)
  
  /**
   * @brief Passive fast decay time
   * 
   * Adds passive fast decay time after bridge polarity change.
   * Helps reduce mid-range resonances.
   * 
   * Range: 0-15
   * - 0: Disabled
   * - 1-15: Fast decay time in multiples of 128 clocks (~10µs per unit @ 12MHz)
   * 
   * @note Starting from 0, increase value if motor suffers from mid-range resonances.
   */
  uint8_t tpfd{0};  ///< Passive fast decay time (0=disabled, 1-15=time in 128 clock units)
  
  /**
   * @brief Microstep resolution
   * 
   * Defines number of microsteps per full step.
   * Higher resolution provides smoother motion but may reduce maximum velocity.
   * 
   * Can be specified as:
   * - uint8_t (0-8): Raw register value (0=256, 1=128, ..., 8=fullstep)
   * - MicrostepResolution enum: More intuitive (recommended)
   * 
   * Typical: MRES_16 (16 microsteps) for balanced performance.
   * 
   * @note Interpolation can be enabled to extrapolate to 256 microsteps regardless of MRES.
   * @note Higher resolution = smoother motion but lower max velocity.
   */
  uint8_t mres{4};  ///< Microstep resolution (0-8 or MicrostepResolution enum, 4=16 microsteps)
  
  /**
   * @brief Enable interpolation to 256 microsteps
   * 
   * Extrapolates MRES setting to 256 microsteps for smoother motion.
   * 
   * - true (recommended): Enable interpolation (smoother motion)
   * - false: Use native MRES resolution only
   * 
   * @note Interpolation works regardless of MRES setting.
   * @note Recommended for best microstepping quality.
   */
  bool intpol{true};  ///< Enable interpolation to 256 microsteps (recommended)
  
  /**
   * @brief Enable double edge step pulses
   * 
   * Enables step impulse at each step edge (both rising and falling).
   * 
   * - false: Normal step pulses (single edge)
   * - true: Double edge step pulses
   * 
   * @note Typically not needed for most applications.
   */
  bool dedge{false};  ///< Enable double edge step pulses (typically false)
  
  /**
   * @brief High velocity fullstep selection
   * 
   * Enables switching to fullstep when VHIGH is exceeded.
   * Switching takes place only at 45° position.
   * The fullstep target current uses the current value from the microstep table at the 45° position.
   * 
   * - false: Normal microstepping operation
   * - true: Switch to fullstep at high velocity (VHIGH threshold)
   * 
   * @note Can be combined with `vhighchm` for maximum velocity.
   * @note Automatically set to true when DcStep is configured.
   * @note Bit 18 in CHOPCONF register.
   */
  bool vhighfs{false};  ///< High velocity fullstep selection (false=normal, true=fullstep at high velocity)
  
  /**
   * @brief High velocity chopper mode
   * 
   * Enables switching to chm=1 (Classic mode) and fd=0 when VHIGH is exceeded.
   * This way, a higher velocity can be achieved.
   * 
   * - false: Normal chopper mode operation
   * - true: Switch to Classic mode (chm=1) with fd=0 at high velocity
   * 
   * @note Can be combined with `vhighfs` for maximum velocity.
   * @note If set, the TOFF setting automatically becomes doubled during high velocity operation
   *       in order to avoid doubling of the chopper frequency.
   * @note Automatically set to true when DcStep is configured.
   * @note Bit 19 in CHOPCONF register.
   */
  bool vhighchm{false};  ///< High velocity chopper mode (false=normal, true=Classic mode at high velocity)
  
  /**
   * @brief Short to GND protection disable
   * 
   * Controls whether short to GND protection is enabled.
   * 
   * - false: Short to GND protection is ON (recommended)
   * - true: Short to GND protection is DISABLED (use with caution)
   * 
   * @note Protection is configured via SHORT_CONF register (s2g_level).
   * @note This bit only disables the protection feature entirely.
   * @note Bit 30 in CHOPCONF register.
   * @warning Disabling protection can lead to driver damage in case of faults.
   */
  bool diss2g{false};  ///< Short to GND protection disable (false=protection ON, true=protection OFF)
  
  /**
   * @brief Short to supply protection disable
   * 
   * Controls whether short to VS (supply) protection is enabled.
   * 
   * - false: Short to VS protection is ON (recommended)
   * - true: Short to VS protection is DISABLED (use with caution)
   * 
   * @note Protection is configured via SHORT_CONF register (s2vs_level).
   * @note This bit only disables the protection feature entirely.
   * @note Bit 31 in CHOPCONF register.
   * @warning Disabling protection can lead to driver damage in case of faults.
   */
  bool diss2vs{false};  ///< Short to supply protection disable (false=protection ON, true=protection OFF)
  
  /**
   * @brief Resistor voltage sensitivity (deprecated)
   * 
   * @note Bit 17 (vsense) is reserved per datasheet and ignored.
   * @deprecated This field has no effect and is kept for backward compatibility only.
   */
  bool vsense{true};  ///< Voltage sensitivity (deprecated, ignored by hardware)

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values for SpreadCycle mode:
   * - mode: SPREAD_CYCLE (recommended)
   * - toff: 5 (typical for 16-30kHz chopper frequency)
   * - tbl: 2 (36 clocks, typical)
   * - hstrt: 4 (effective hysteresis = 4)
   * - hend: 0 (effective end = -3)
   * - mres: 4 (16 microsteps)
   * - intpol: true (enable interpolation)
   */
  ChopperConfig() = default;
  
  /**
   * @brief Helper constructor for SpreadCycle mode
   * 
   * Creates a SpreadCycle configuration with typical settings.
   * 
   * @param off_time Off time setting (0=disabled, 1-15, 5=typical)
   * @param blank_time Blank time (ChopperBlankTime enum or 0-3, TBL_36CLK=typical)
   * @param hysteresis_start Hysteresis start (0-7, 4=typical)
   * @param hysteresis_end Hysteresis end (0-15 encoded, 0=typical)
   * @param microstep_res Microstep resolution (MicrostepResolution enum or 0-8, MRES_16=typical)
   */
  ChopperConfig(uint8_t off_time, uint8_t blank_time = 2,
                uint8_t hysteresis_start = 4, uint8_t hysteresis_end = 0,
                uint8_t microstep_res = 4)
    : mode(ChopperMode::SPREAD_CYCLE)
    , toff(off_time)
    , tbl(blank_time)
    , hstrt(hysteresis_start)
    , hend(hysteresis_end)
    , tfd(0)
    , disfdcc(false)
    , tpfd(0)
    , mres(microstep_res)
    , intpol(true)
    , dedge(false)
    , vhighfs(false)
    , vhighchm(false)
    , diss2g(false)
    , diss2vs(false)
    , vsense(true)
  {}
  
  /**
   * @brief Helper constructor for Classic mode
   * 
   * Creates a Classic constant off-time chopper configuration.
   * 
   * @param off_time Off time setting (0=disabled, 1-15, 5=typical)
   * @param fast_decay_time Fast decay time (0-15, similar to off_time)
   * @param sine_offset Sine wave offset (0-15 encoded, 4-6=typical positive offset)
   * @param blank_time Blank time (ChopperBlankTime enum or 0-3, TBL_36CLK=typical)
   * @param disable_comparator Disable fast decay comparator (false=enable, true=time only)
   * @param microstep_res Microstep resolution (MicrostepResolution enum or 0-8, MRES_16=typical)
   */
  ChopperConfig(uint8_t off_time, uint8_t fast_decay_time, uint8_t sine_offset,
                uint8_t blank_time = 2, bool disable_comparator = false,
                uint8_t microstep_res = 4)
    : mode(ChopperMode::CLASSIC)
    , toff(off_time)
    , tbl(blank_time)
    , hstrt(0)  // Not used in Classic mode
    , hend(sine_offset)  // Used as OFFSET in Classic mode
    , tfd(fast_decay_time)
    , disfdcc(disable_comparator)
    , tpfd(0)
    , mres(microstep_res)
    , intpol(true)
    , dedge(false)
    , vhighfs(false)
    , vhighchm(false)
    , diss2g(false)
    , diss2vs(false)
    , vsense(true)
  {}
};

//===============================================================================================================
//===============================================================================================================
//                                 STEALTHCHOP CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================            


/**
 * @brief StealthChop PWM frequency enumeration
 *
 * Provides PWM frequency selection options for StealthChop operation.
 * Lower frequencies reduce current ripple but may limit high-velocity performance.
 * Higher frequencies improve high-velocity performance but increase dynamic power dissipation.
 * 
 * Recommended range: 20-50kHz for most applications.
 * 
 * @note Actual frequency depends on clock frequency (f_CLK).
 * @note For 12MHz internal clock:
 *   - PWM_FREQ_0: ~23.4kHz
 *   - PWM_FREQ_1: ~35.1kHz (recommended)
 *   - PWM_FREQ_2: ~46.9kHz
 *   - PWM_FREQ_3: ~58.5kHz
 */
enum class StealthChopPwmFreq : uint8_t {
  PWM_FREQ_0 = 0,  ///< fPWM = 2/1024 * fCLK (~23.4kHz @ 12MHz)
  PWM_FREQ_1 = 1,  ///< fPWM = 2/683 * fCLK (~35.1kHz @ 12MHz, recommended)
  PWM_FREQ_2 = 2,  ///< fPWM = 2/512 * fCLK (~46.9kHz @ 12MHz)
  PWM_FREQ_3 = 3   ///< fPWM = 2/410 * fCLK (~58.5kHz @ 12MHz)
};

/**
 * @brief StealthChop regulation speed enumeration
 *
 * Controls how fast the PWM amplitude regulation loop adapts to changes.
 * Specifies maximum PWM amplitude change per half wave when using automatic scaling.
 * 
 * Lower values provide stable, soft regulation (slower response).
 * Higher values provide faster adaptation but may be less stable.
 * 
 * @note Optimize for fastest required acceleration/deceleration ramp.
 * @note Higher acceleration during AT#2 requires higher regulation speed.
 */
enum class StealthChopRegulationSpeed : uint8_t {
  VERY_SLOW = 1,   ///< 0.5 increments per half wave (slowest, most stable)
  SLOW = 2,        ///< 1 increment per half wave
  MODERATE = 4,    ///< 2 increments per half wave (default, balanced)
  FAST = 8,        ///< 4 increments per half wave
  VERY_FAST = 15   ///< 7.5 increments per half wave (fastest, may be less stable)
};

/**
 * @brief StealthChop mode switching jerk reduction enumeration
 *
 * Controls current jerk reduction when switching from SpreadCycle to StealthChop.
 * Lower values reduce current spike during mode switching (smoother).
 * Higher values allow faster switching but may cause more current jerk.
 * 
 * @note Only affects switching from SpreadCycle to StealthChop.
 * @note Reduce value if experiencing current spikes during mode switching.
 */
enum class StealthChopJerkReduction : uint8_t {
  MAXIMUM = 8,     ///< Maximum jerk reduction (smoothest switching)
  HIGH = 10,       ///< High jerk reduction
  MODERATE = 12,   ///< Moderate jerk reduction (default, balanced)
  LOW = 14,        ///< Low jerk reduction
  MINIMUM = 15     ///< Minimum jerk reduction (fastest switching, may cause spikes)
};

/**
 * @brief StealthChop configuration structure
 *
 * User-friendly configuration for StealthChop2 voltage PWM mode operation.
 * StealthChop provides extremely quiet, noiseless operation for stepper motors,
 * making it ideal for indoor or home use applications.
 *
 * StealthChop2 features automatic tuning (AT) that adapts operating parameters
 * to the motor automatically. Two tuning phases are required:
 * - AT#1: Motor in standstill with nominal run current (≤130ms)
 * - AT#2: Motor moving at medium velocity (60-300 RPM typical, 8+ fullsteps)
 *
 * @note StealthChop requires motor to be at standstill when first enabled.
 * @note Keep motor stopped for at least 128 chopper periods after enabling StealthChop.
 * @note StealthChop and StallGuard2 are mutually exclusive (StallGuard2 requires SpreadCycle).
 * @note Lower current limit applies: IRUN ≥ 8 and current must exceed I_LOWER_LIMIT.
 * @note Open load detection should be performed in SpreadCycle before enabling StealthChop.
 * @note Motor stall during StealthChop can cause overcurrent - tune low-side overcurrent detection.
 *
 * @see Datasheet section 7: StealthChop™
 */
struct StealthChopConfig {
  /**
   * @brief PWM amplitude offset
   * 
   * Initial value for PWM amplitude (offset) for velocity-based scaling.
   * Used as initialization value for automatic tuning (PWM_OFS_AUTO).
   * 
   * Range: 0-255
   * - 0: Special mode - Disables scaling down motor current below lower measurement threshold.
   *      Prevents motor going out of regulation but also prevents power down below regulation limit.
   *      Use only when power supply voltage can vary up and down by factor of 2 or more.
   * - 1-255: Normal mode - Allows automatic scaling to low PWM duty cycles even below regulation threshold.
   *          Enables low standstill current settings based on IHOLD_IRUN register.
   * - Typical: 30 (default) for most motors
   * 
   * Calculation (for manual mode, pwm_autoscale=0):
   * PWM_OFS = 374 * R_COIL * I_COIL / V_M
   * 
   * @note With automatic tuning (pwm_autoscale=1), this is used as initial value.
   * @note After AT#1, actual value is stored in PWM_OFS_AUTO (read via GetPwmAuto()).
   * @note PWM_OFS = 0 should only be used under certain conditions (see above).
   */
  uint8_t pwm_ofs{30};  ///< PWM amplitude offset (0=special mode, 1-255=normal, 30=typical)
  
  /**
   * @brief PWM amplitude gradient
   * 
   * Velocity-dependent factor to compensate for back EMF.
   * Used as initialization value for automatic tuning (PWM_GRAD_AUTO).
   * 
   * Range: 0-255
   * - Typical: 0 (default) - let automatic tuning determine
   * 
   * Calculation (for manual mode, pwm_autoscale=0):
   * PWM_GRAD = C_BEMF * 2π * f_CLK * 1.46 / (V_M * MSPR)
   * 
   * @note With automatic tuning (pwm_autograd=1), this is used as initial value.
   * @note After AT#2, actual value is stored in PWM_GRAD_AUTO (read via GetPwmAuto()).
   */
  uint8_t pwm_grad{0};  ///< PWM amplitude gradient (0-255, velocity compensation)
  
  /**
   * @brief PWM frequency selection
   * 
   * Chopper frequency selection for StealthChop operation.
   * Lower frequencies reduce current ripple but may limit high-velocity performance.
   * Higher frequencies improve high-velocity performance but increase power dissipation.
   * 
   * Can be specified as:
   * - uint8_t (0-3): Raw register value
   * - StealthChopPwmFreq enum: More intuitive (recommended)
   * 
   * Recommended: PWM_FREQ_1 (~35kHz @ 12MHz clock) for most applications
   * Use lowest setting giving good results.
   * 
   * @note Actual frequency depends on clock frequency (f_CLK).
   * @note Lower blank time (TBL) allows lower current limit.
   * @note During AT#1, driver may reduce frequency if current cannot be reached.
   */
  uint8_t pwm_freq{1};  ///< PWM frequency (0-3 or StealthChopPwmFreq enum, 1=recommended)
  
  /**
   * @brief Enable automatic current scaling
   * 
   * Enables automatic current regulation using current measurement feedback.
   * 
   * - true (recommended): Automatic scaling with current regulator
   *   - Adapts to motor heating, supply voltage changes
   *   - Responds to motor stall and load changes
   *   - Requires current measurement (sense resistors)
   * 
   * - false: Feed-forward velocity-controlled mode
   *   - Very stable amplitude
   *   - Does not react to supply voltage changes or motor stall
   *   - Does not require current measurement
   *   - Only for well-known motor and operating conditions
   * 
   * @note Recommended: Use automatic mode (true) unless current regulation is not satisfying.
   * @note Non-automatic mode requires careful programming and well-known conditions.
   */
  bool pwm_autoscale{true};  ///< Enable automatic current scaling (recommended)
  
  /**
   * @brief Enable automatic gradient adaptation
   * 
   * Enables automatic tuning of PWM_GRAD_AUTO during AT#2 phase.
   * 
   * - true (recommended): Automatic gradient tuning
   *   - Driver optimizes PWM_GRAD_AUTO during AT#2
   *   - Adapts to motor characteristics automatically
   * 
   * - false: Use PWM_GRAD from register
   *   - Use pre-determined PWM_GRAD value
   *   - Reduces amplitude jitter
   *   - Requires manual tuning
   * 
   * @note Set to false if you have pre-determined PWM_GRAD and want to reduce jitter.
   */
  bool pwm_autograd{true};  ///< Enable automatic gradient adaptation (recommended)
  
  /**
   * @brief Regulation speed
   * 
   * Controls how fast the PWM amplitude regulation loop adapts to changes.
   * Specifies maximum PWM amplitude change per half wave when using automatic scaling.
   * 
   * Can be specified as:
   * - uint8_t (1-15): Raw register value
   *   - 1: 0.5 increments per half wave (slowest, most stable)
   *   - 2: 1 increment per half wave
   *   - 4: 2 increments per half wave (default, balanced)
   *   - 8: 4 increments per half wave
   *   - 15: 7.5 increments per half wave (fastest, may be less stable)
   * - StealthChopRegulationSpeed enum: More intuitive (recommended)
   * 
   * Lower values: Stable, soft regulation (slower response)
   * Higher values: Faster adaptation (may be less stable)
   * 
   * @note Should be as small as possible for stability, but large enough for quick reaction.
   * @note Higher acceleration during AT#2 requires higher regulation speed.
   * @note Optimize for fastest required acceleration/deceleration ramp.
   */
  uint8_t pwm_reg{4};  ///< Regulation speed (1-15 or StealthChopRegulationSpeed enum, 4=balanced)
  
  /**
   * @brief Mode switching jerk reduction
   * 
   * Controls current jerk reduction when switching from SpreadCycle to StealthChop.
   * Limits PWM_SCALE_AUTO when switching back from SpreadCycle to StealthChop.
   * 
   * Can be specified as:
   * - uint8_t (0-15): Raw register value (upper 4 bits of 8-bit amplitude limit)
   *   - Lower values: Lower current jerk (smoother switching)
   *   - Higher values: Higher current jerk (faster switching)
   *   - Default: 12 (recommended, balanced)
   * - StealthChopJerkReduction enum: More intuitive (recommended)
   * 
   * @note Reduce value if experiencing current spikes during mode switching.
   * @note Only affects switching from SpreadCycle to StealthChop.
   * @note Does not limit PWM_GRAD or PWM_GRAD_AUTO offset.
   */
  uint8_t pwm_lim{12};  ///< Jerk reduction (0-15 or StealthChopJerkReduction enum, 12=balanced)
  
  /**
   * @brief Freewheeling mode when I_HOLD=0
   * 
   * Standstill option when motor current setting is zero.
   * Only available with StealthChop enabled.
   * 
   * Options:
   * - NORMAL: Normal operation (coast when ihold=0)
   * - ENABLED: Freewheeling (low resistance, easy to move)
   * - SHORT_LS: Coil short via low-side drivers (passive brake)
   * - SHORT_HS: Coil short via high-side drivers (passive brake)
   * 
   * @note Freewheeling makes motor easy to move.
   * @note Coil short options realize passive braking (energy efficient).
   * @note Enabled after TPOWERDOWN and IHOLDDELAY delay.
   */
  PWMFreewheel freewheel{PWMFreewheel::NORMAL};  ///< Freewheeling mode when I_HOLD=0

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values for automatic tuning mode:
   * - pwm_ofs: 30 (typical starting value, normal mode)
   * - pwm_grad: 0 (let automatic tuning determine)
   * - pwm_freq: 1 (~35kHz @ 12MHz, recommended)
   * - pwm_autoscale: true (automatic current scaling)
   * - pwm_autograd: true (automatic gradient adaptation)
   * - pwm_reg: 4 (balanced regulation, 2 increments per half wave)
   * - pwm_lim: 12 (balanced jerk reduction)
   * - freewheel: NORMAL (normal operation)
   */
  StealthChopConfig() = default;
  
  /**
   * @brief Helper constructor for automatic tuning mode
   * 
   * Creates a StealthChop configuration optimized for automatic tuning.
   * 
   * @param pwm_frequency PWM frequency (StealthChopPwmFreq enum or 0-3)
   * @param pwm_offset Initial PWM offset (0=special mode, 1-255=normal, 30=typical)
   * @param pwm_gradient Initial PWM gradient (0-255, 0=auto)
   * @param regulation_speed Regulation speed (StealthChopRegulationSpeed enum or 1-15, 4=balanced)
   * @param jerk_reduction Jerk reduction (StealthChopJerkReduction enum or 0-15, 12=balanced)
   */
  StealthChopConfig(uint8_t pwm_frequency, uint8_t pwm_offset = 30,
                    uint8_t pwm_gradient = 0, uint8_t regulation_speed = 4,
                    uint8_t jerk_reduction = 12)
    : pwm_ofs(pwm_offset)
    , pwm_grad(pwm_gradient)
    , pwm_freq(pwm_frequency)
    , pwm_autoscale(true)  // Enable automatic tuning
    , pwm_autograd(true)  // Enable automatic gradient
    , pwm_reg(regulation_speed)
    , pwm_lim(jerk_reduction)
    , freewheel(PWMFreewheel::NORMAL)
  {}
  
  /**
   * @brief Helper constructor using enums (most intuitive)
   * 
   * Creates a StealthChop configuration using intuitive enums.
   * 
   * @param pwm_frequency PWM frequency enum
   * @param regulation_speed Regulation speed enum
   * @param jerk_reduction Jerk reduction enum
   * @param pwm_offset Initial PWM offset (0=special mode, 1-255=normal, 30=typical)
   * @param pwm_gradient Initial PWM gradient (0-255, 0=auto)
   */
  StealthChopConfig(StealthChopPwmFreq pwm_frequency,
                    StealthChopRegulationSpeed regulation_speed = StealthChopRegulationSpeed::MODERATE,
                    StealthChopJerkReduction jerk_reduction = StealthChopJerkReduction::MODERATE,
                    uint8_t pwm_offset = 30, uint8_t pwm_gradient = 0)
    : pwm_ofs(pwm_offset)
    , pwm_grad(pwm_gradient)
    , pwm_freq(static_cast<uint8_t>(pwm_frequency))
    , pwm_autoscale(true)  // Enable automatic tuning
    , pwm_autograd(true)  // Enable automatic gradient
    , pwm_reg(static_cast<uint8_t>(regulation_speed))
    , pwm_lim(static_cast<uint8_t>(jerk_reduction))
    , freewheel(PWMFreewheel::NORMAL)
  {}
  
  /**
   * @brief Helper constructor for manual mode
   * 
   * Creates a StealthChop configuration for manual (non-automatic) mode.
   * Requires well-known motor and operating conditions.
   * 
   * @param pwm_frequency PWM frequency (StealthChopPwmFreq enum or 0-3)
   * @param pwm_offset PWM offset (calculated: 374 * R_COIL * I_COIL / V_M)
   * @param pwm_gradient PWM gradient (calculated from back EMF constant)
   */
  StealthChopConfig(uint8_t pwm_frequency, uint8_t pwm_offset, uint8_t pwm_gradient)
    : pwm_ofs(pwm_offset)
    , pwm_grad(pwm_gradient)
    , pwm_freq(pwm_frequency)
    , pwm_autoscale(false)  // Manual mode
    , pwm_autograd(false)   // Manual mode
    , pwm_reg(4)
    , pwm_lim(12)
    , freewheel(PWMFreewheel::NORMAL)
  {}
};

//===============================================================================================================
//===============================================================================================================
//                                 STALLGUARD CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================            

/**
 * @brief StallGuard2 sensitivity enumeration
 *
 * Provides intuitive sensitivity levels for StallGuard2 threshold tuning.
 * Lower SGT values = higher sensitivity (detects stalls easier, more false positives).
 * Higher SGT values = lower sensitivity (requires more torque to detect stall, fewer false positives).
 */
enum class StallGuardSensitivity : int8_t {
  VERY_HIGH = -32,   ///< Very high sensitivity (SGT = -32) - detects stalls very easily
  HIGH = -16,        ///< High sensitivity (SGT = -16) - detects stalls easily
  MODERATE = 0,      ///< Moderate sensitivity (SGT = 0) - starting value, works with most motors
  LOW = 16,          ///< Low sensitivity (SGT = 16) - requires more torque to detect stall
  VERY_LOW = 32      ///< Very low sensitivity (SGT = 32) - requires significant torque to detect stall
};

/**
 * @brief StallGuard2 configuration structure
 *
 * User-friendly configuration for StallGuard2 load measurement and stall detection.
 * StallGuard2 provides accurate measurement of motor load and can detect stalls.
 * It's used for sensorless homing, CoolStep load-adaptive current reduction, and diagnostics.
 *
 * @note StallGuard2 requires SpreadCycle mode (StealthChop must be disabled).
 * @note StallGuard2 measurement is updated with each fullstep (or every 4 fullsteps if filter enabled).
 * @note SGT threshold should be tuned for your specific motor and operating conditions.
 *
 * @see Datasheet section 13: StallGuard2 Load Measurement
 */
struct StallGuardConfig {
  /**
   * @brief StallGuard2 threshold value
   * 
   * Controls StallGuard2 sensitivity for stall detection and sets optimum measurement range.
   * 
   * Range: -64 to +63
   * - Lower values (-64 to 0): Higher sensitivity (detects stalls easier, more false positives)
   * - Higher values (0 to +63): Lower sensitivity (requires more torque to detect stall, fewer false positives)
   * - Zero (0): Starting value, works with most motors
   * 
   * Tuning: Adjust until SG_RESULT is between 0-100 at maximum load before stall.
   * 
   * @note Can be set directly as int8_t or use sensitivity enum for convenience.
   */
  int8_t threshold{0};  ///< StallGuard2 threshold (-64 to +63, 0 = starting value)
  
  /**
   * @brief Enable StallGuard2 filter
   * 
   * Filter reduces measurement rate to one measurement per electrical period (4 fullsteps).
   * Compensates for motor construction variations (e.g., misalignment of phase magnets).
   * 
   * - Enabled: More precise measurement, smoother readings (recommended for CoolStep)
   * - Disabled: Faster response, higher time resolution (recommended for sensorless homing)
   * 
   * @note Filter should be enabled when high-precision measurement is required.
   * @note Filter should be disabled for rapid response and best sensorless homing results.
   */
  bool enable_filter{false};  ///< Enable StallGuard2 filter (reduces measurement rate 4x)
  
  /**
   * @brief Lower velocity threshold for StallGuard2 operation
   * 
   * StallGuard2 is disabled below this velocity.
   * Set to match the lower limit of velocity range where StallGuard2 gives stable results.
   * 
   * @note Unit is specified by velocity_unit field.
   * @note Set to match your typical operating velocity range.
   * @note For sensorless homing, set to match search speed.
   */
  float min_velocity{0.0F};  ///< Minimum velocity for StallGuard2 (0 = no lower limit)
  
  /**
   * @brief Upper velocity threshold for StallGuard2 operation
   * 
   * StallGuard2 may not operate reliably above this velocity.
   * Set to match the upper limit of velocity range where StallGuard2 gives stable results.
   * 
   * @note Unit is specified by velocity_unit field.
   * @note Set to 0 for no upper limit (StallGuard2 active at all velocities above min_velocity).
   */
  float max_velocity{0.0F};  ///< Maximum velocity for StallGuard2 (0 = no upper limit)
  
  /**
   * @brief Unit for velocity thresholds
   * 
   * Specifies the unit used for min_velocity and max_velocity fields.
   */
  Unit velocity_unit{Unit::Steps};  ///< Unit for velocity thresholds
  
  /**
   * @brief Stop motor when stall detected
   * 
   * If true, motor stops (VACTUAL = 0) when stall is detected.
   * Motor remains stopped until RAMP_STAT.event_stop_sg flag is read.
   * 
   * @note Requires StallGuard2 to be properly tuned (threshold set correctly).
   * @note Requires min_velocity to be set (TCOOLTHRS must be configured).
   * @note Motor can be restarted by reading/writing RAMP_STAT register or disabling sg_stop.
   */
  bool stop_on_stall{false};  ///< Stop motor when stall detected

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values:
   * - threshold: 0 (starting value, works with most motors)
   * - enable_filter: false (faster response)
   * - stop_on_stall: false (don't stop on stall)
   */
  StallGuardConfig() = default;
  
  /**
   * @brief Helper constructor for quick setup
   * 
   * Creates a StallGuard2 configuration with typical settings.
   * 
   * @param sgt_threshold StallGuard2 threshold (-64 to +63, 0 = starting value)
   * @param enable_filt Enable filter for smoother readings
   * @param min_vel Minimum velocity for StallGuard2 operation
   * @param max_vel Maximum velocity for StallGuard2 operation (0 = no limit)
   * @param vel_unit Unit for velocity thresholds
   * @param stop_on_stall Stop motor when stall detected
   */
  StallGuardConfig(int8_t sgt_threshold, bool enable_filt = false,
                   float min_vel = 0.0F, float max_vel = 0.0F,
                   Unit vel_unit = Unit::Steps, bool stop_on_stall = false)
    : threshold(sgt_threshold)
    , enable_filter(enable_filt)
    , min_velocity(min_vel)
    , max_velocity(max_vel)
    , velocity_unit(vel_unit)
    , stop_on_stall(stop_on_stall)
  {}
  
  /**
   * @brief Helper constructor using sensitivity enum
   * 
   * Creates a StallGuard2 configuration using intuitive sensitivity levels.
   * 
   * @param sensitivity StallGuard2 sensitivity level (enum)
   * @param enable_filt Enable filter for smoother readings
   * @param min_vel Minimum velocity for StallGuard2 operation
   * @param max_vel Maximum velocity for StallGuard2 operation (0 = no limit)
   * @param vel_unit Unit for velocity thresholds
   * @param stop_on_stall Stop motor when stall detected
   */
  StallGuardConfig(StallGuardSensitivity sensitivity, bool enable_filt = false,
                   float min_vel = 0.0F, float max_vel = 0.0F,
                   Unit vel_unit = Unit::Steps, bool stop_on_stall = false)
    : threshold(static_cast<int8_t>(sensitivity))
    , enable_filter(enable_filt)
    , min_velocity(min_vel)
    , max_velocity(max_vel)
    , velocity_unit(vel_unit)
    , stop_on_stall(stop_on_stall)
  {}
};

//===============================================================================================================
//===============================================================================================================
//                                 RAMP CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================            

/**
 * @brief Ramp configuration structure
 *
 * Configuration parameters for the TMC5160 ramp generator.
 * The ramp generator provides two-phase acceleration and deceleration with programmable
 * start/stop velocities and transition speed (V1) for optimal motor torque utilization.
 *
 * @note All velocity and acceleration parameters support unit-aware conversion (Steps, Rad, Deg, Mm, RPM).
 * @note Parameters set to 0.0 will use driver defaults or be auto-calculated where applicable.
 * @note VSTOP must be >= VSTART to ensure successful motion termination.
 * @note D1 must not be 0 in positioning mode (defaults to 100 if not set).
 *
 * @see Datasheet section 12: Ramp Generator
 */
struct RampConfig {
  /**
   * @brief Unit for all velocity parameters (vstart, vstop, vmax, v1)
   * 
   * Specifies the unit system used for velocity values in this configuration.
   * Critical for proper conversion during initialization.
   * 
   * Supported units: Steps, Rad, Deg, Mm, RPM
   * 
   * @note Default: Unit::Steps (driver native units)
   * @note All velocity parameters (vstart, vstop, vmax, v1) use this unit
   */
  Unit velocity_unit{Unit::Steps};
  
  /**
   * @brief Unit for all acceleration parameters (amax, a1, dmax, d1)
   * 
   * Specifies the unit system used for acceleration/deceleration values in this configuration.
   * Critical for proper conversion during initialization.
   * 
   * Supported units: Steps, Rad, Deg, Mm (RPM not applicable for acceleration)
   * 
   * @note Default: Unit::Steps (driver native units)
   * @note All acceleration parameters (amax, a1, dmax, d1) use this unit
   */
  Unit acceleration_unit{Unit::Steps};
  
  // Velocity parameters (unit specified by velocity_unit field)
  float vstart{0.0f};     ///< Start velocity (unit specified by velocity_unit, 0 = can be set to zero if not used)
  float vstop{10.0f};    ///< Stop velocity (unit specified by velocity_unit, must be >= VSTART, minimum 1 recommended)
  float vmax{0.0f};       ///< Maximum velocity (unit specified by velocity_unit, 0 = must be set via SetMaxSpeed() before motion)
  float v1{0.0f};         ///< Transition velocity (unit specified by velocity_unit, switches between A1/AMAX and D1/DMAX, 0 = disabled)
  
  // Acceleration parameters (unit specified by acceleration_unit field)
  float amax{0.0f};      ///< Maximum acceleration (unit specified by acceleration_unit, used above V1, 0 = must be set via SetAcceleration() before motion)
  float a1{0.0f};        ///< First acceleration (unit specified by acceleration_unit, used between VSTART and V1, 0 = use AMAX)
  float dmax{0.0f};       ///< Maximum deceleration (unit specified by acceleration_unit, used above V1, 0 = uses AMAX value)
  float d1{100.0f};      ///< First deceleration (unit specified by acceleration_unit, used between VSTOP and V1, must not be 0 in positioning mode)
  
  // Timing parameters (in milliseconds, automatically converted to register values)
  float tpowerdown_ms{437.0f}; ///< Power down delay in milliseconds (0-5600ms at 12MHz, automatically converted)
                                ///< Time before motor power down after standstill.
                                ///< Conversion: tpowerdown_register = time_ms * fCLK / (2^18 * 1000)
                                ///< @note Actual time depends on clock frequency (fCLK). At 12MHz: ~0-5.6s range.
  float tzerowait_ms{0.0f};    ///< Zero wait time in milliseconds (0-2000ms at 12MHz, automatically converted)
                                ///< Waiting time after ramping down to zero velocity before allowing reverse motion.
                                ///< Prevents excessive jerk when changing direction (VSTART+VSTOP).
                                ///< Conversion: tzerowait_register = time_ms * fCLK / (2^18 * 1000)
                                ///< @note Actual time depends on clock frequency (fCLK). At 12MHz: ~0-2s range.

  /**
   * @brief Default constructor
   *
   * Initializes with recommended default values:
   * - velocity_unit: Steps (driver native units)
   * - acceleration_unit: Steps (driver native units)
   * - VSTART: 0 (can be zero if not used)
   * - VSTOP: 10 (minimum recommended, ensures successful motion termination)
   * - VMAX: 0 (must be set before motion)
   * - V1: 0 (disabled, uses single acceleration/deceleration)
   * - AMAX: 0 (must be set before motion)
   * - A1: 0 (uses AMAX)
   * - DMAX: 0 (uses AMAX value)
   * - D1: 100 (required for positioning mode)
   * - TPOWERDOWN: 437ms (~0.44s at 12MHz, equivalent to register value 20)
   * - TZEROWAIT: 0ms (no delay)
   */
  RampConfig() = default;
};

//===============================================================================================================
//===============================================================================================================
//                                 COOLSTEP CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================            

/**
 * @brief CoolStep current increment step width enumeration
 *
 * Defines how many current steps to increment per StallGuard2 measurement below threshold.
 * Higher values provide faster response to increasing load but may cause oscillations.
 */
enum class CoolStepIncrementStep : uint8_t {
  STEP_1 = 0,  ///< Increment by 1 step per measurement (slowest, smoothest)
  STEP_2 = 1,  ///< Increment by 2 steps per measurement
  STEP_4 = 2,  ///< Increment by 4 steps per measurement
  STEP_8 = 3   ///< Increment by 8 steps per measurement (fastest response)
};

/**
 * @brief CoolStep current decrement speed enumeration
 *
 * Defines how many StallGuard2 measurements above threshold before decrementing current.
 * Higher values provide slower current reduction, preventing rapid oscillations.
 */
enum class CoolStepDecrementSpeed : uint8_t {
  EVERY_32 = 0,  ///< Decrement every 32 measurements (slowest reduction, most stable)
  EVERY_8 = 1,   ///< Decrement every 8 measurements
  EVERY_2 = 2,    ///< Decrement every 2 measurements
  EVERY_1 = 3     ///< Decrement every measurement (fastest reduction)
};

/**
 * @brief CoolStep minimum current percentage enumeration
 *
 * Defines the minimum motor current as a percentage of IRUN when CoolStep reduces current.
 */
enum class CoolStepMinCurrent : uint8_t {
  HALF_IRUN = 0,    ///< Minimum current is 50% of IRUN (1/2)
  QUARTER_IRUN = 1  ///< Minimum current is 25% of IRUN (1/4)
};

/**
 * @brief CoolStep configuration structure
 *
 * User-friendly configuration for CoolStep automatic current reduction feature.
 * CoolStep automatically reduces motor current when load is low, saving energy and reducing heat.
 *
 * @note CoolStep requires SpreadCycle mode (StealthChop must be disabled).
 * @note CoolStep uses StallGuard2 to measure motor load, so StallGuard2 must be properly tuned.
 *
 * @see Datasheet section 14: CoolStep Operation
 */
struct CoolStepConfig {
  /**
   * @brief Lower StallGuard2 threshold for current increase
   * 
   * When SG_RESULT falls below this value, CoolStep increases current.
   * Range: 0-1023 (actual SG_RESULT values)
   * 
   * Set to 0 to disable CoolStep.
   * 
   * @note Automatically converted to SEMIN register value internally: SEMIN = lower_threshold_sg / 32
   */
  uint16_t lower_threshold_sg{0};  ///< Lower threshold in SG units (0-1023, 0 = CoolStep disabled)
  
  /**
   * @brief Upper StallGuard2 threshold for current decrease
   * 
   * When SG_RESULT is equal to or above this value enough times, CoolStep decreases current.
   * Range: 0-1023 (actual SG_RESULT values)
   * 
   * Must be greater than lower_threshold_sg for CoolStep to function properly.
   * 
   * @note Automatically converted to SEMAX register value internally based on lower_threshold_sg
   */
  uint16_t upper_threshold_sg{0};  ///< Upper threshold in SG units (0-1023)
  
  // Step size configuration (using enums for clarity)
  CoolStepIncrementStep increment_step{CoolStepIncrementStep::STEP_2};  ///< Current increment step width
  CoolStepDecrementSpeed decrement_speed{CoolStepDecrementSpeed::EVERY_8};  ///< Current decrement speed
  
  // Minimum current configuration
  CoolStepMinCurrent min_current{CoolStepMinCurrent::HALF_IRUN};  ///< Minimum current percentage
  
  // Filter configuration
  bool enable_filter{false};  ///< Enable StallGuard2 filter (reduces measurement rate by 4x)
  
  // Velocity thresholds (with unit support)
  /**
   * @brief Lower velocity threshold for CoolStep activation
   * 
   * CoolStep is disabled below this velocity.
   * Set to match the lower limit of velocity range where StallGuard2 gives stable results.
   * 
   * @note Unit is specified by velocity_unit field.
   * @note Setting equal to max_velocity disables CoolStep during acceleration/deceleration.
   */
  float min_velocity{0.0F};  ///< Minimum velocity for CoolStep (0 = disabled)
  
  /**
   * @brief Upper velocity threshold for CoolStep activation
   * 
   * CoolStep is disabled above this velocity.
   * Set to match the upper limit of velocity range where StallGuard2 gives stable results.
   * 
   * @note Unit is specified by velocity_unit field.
   */
  float max_velocity{0.0F};  ///< Maximum velocity for CoolStep (0 = no upper limit)
  
  /**
   * @brief Unit for velocity thresholds
   * 
   * Specifies the unit used for min_velocity and max_velocity fields.
   */
  Unit velocity_unit{Unit::Steps};  ///< Unit for velocity thresholds

  /**
   * @brief Default constructor
   *
   * Initializes with CoolStep disabled (semin=0).
   * All fields use default member initializers.
   */
  CoolStepConfig() = default;
  
  /**
   * @brief Helper constructor for common configuration
   * 
   * Creates a CoolStep configuration with typical settings:
   * - Lower threshold: 64 (increases current when SG < 64)
   * - Upper threshold: 256 (decreases current when SG >= 256)
   * - Moderate increment/decrement speeds
   * - 50% minimum current
   * 
   * @param lower_sg Lower SG threshold (0-1023, 0 to disable CoolStep)
   * @param upper_sg Upper SG threshold (0-1023, must be > lower_sg)
   * @param min_vel Minimum velocity for CoolStep activation
   * @param max_vel Maximum velocity for CoolStep activation
   * @param vel_unit Unit for velocity thresholds
   */
  CoolStepConfig(uint16_t lower_sg, uint16_t upper_sg, 
                 float min_vel, float max_vel, Unit vel_unit = Unit::Steps)
    : lower_threshold_sg(lower_sg)
    , upper_threshold_sg(upper_sg)
    , min_velocity(min_vel)
    , max_velocity(max_vel)
    , velocity_unit(vel_unit)
  {}
};

//===============================================================================================================
//===============================================================================================================
//                                 DCSTEP CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================            


/**
 * @brief DcStep stall detection sensitivity enumeration
 *
 * Defines the sensitivity of stall detection in DcStep mode.
 * Higher sensitivity detects stalls earlier but may trigger false positives.
 */
enum class DcStepStallSensitivity : uint8_t {
  DISABLED = 0,     ///< Stall detection disabled (dc_sg = 0)
  LOW = 1,          ///< Low sensitivity - fewer false positives (dc_sg ≈ dc_time / 20)
  MODERATE = 2,     ///< Moderate sensitivity - balanced (dc_sg ≈ dc_time / 16, recommended)
  HIGH = 3          ///< High sensitivity - detects stalls earlier (dc_sg ≈ dc_time / 12)
};

/**
 * @brief DcStep configuration structure
 *
 * User-friendly configuration for DcStep automatic commutation mode.
 * DcStep allows the motor to run near its load limit without losing steps by automatically
 * reducing velocity when overloaded. The motor operates in fullstep mode at the target velocity
 * or at reduced velocity if overloaded.
 *
 * @note DcStep requires SD_MODE=1 (external step/dir mode) or can be enabled via VDCMIN threshold.
 * @note DcStep automatically sets chopper to constant TOFF mode with slow decay only.
 * @note CHOPCONF.vhighfs and CHOPCONF.vhighchm must be set to 1 for DcStep (handled automatically).
 * @note CHOPCONF.TOFF should be >2, preferably 8-15 for DcStep operation.
 *
 * @see Datasheet section 17: DcStep
 */
struct DcStepConfig {
  /**
   * @brief Minimum velocity threshold for DcStep activation
   * 
   * Below this velocity, motor operates in normal microstep mode.
   * In DcStep operation, motor operates at minimum this velocity even when blocked.
   * 
   * Set to the lowest operating velocity where DcStep gives reliable detection.
   * 
   * @note Unit is specified by velocity_unit field.
   * @note Set to 0.0 to disable DcStep.
   */
  float min_velocity{0.0F};  ///< Minimum velocity for DcStep (0 = disabled)
  
  /**
   * @brief Unit for velocity threshold
   * 
   * Specifies the unit used for min_velocity field.
   */
  Unit velocity_unit{Unit::Steps};  ///< Unit for velocity threshold
  
  /**
   * @brief PWM on-time limit for commutation (user-friendly)
   * 
   * Controls the reference pulse width for DcStep load measurement.
   * Must be optimized for robust operation with maximum motor torque.
   * 
   * Higher value = higher torque and higher velocity capability
   * Lower value = operation down to lower velocity (as set by min_velocity)
   * 
   * Should be set slightly above effective blank time (TBL from CHOPCONF).
   * Lower limit: TBL clock cycles + n (where n = 1-100 for typical motor)
   * 
   * Range: 0.1-85.3µs (for 12MHz clock) or 0.1-42.7µs (for 24MHz clock)
   * 
   * @note Automatically converted to DC_TIME register value (0-1023 clock cycles).
   * @note If 0, DcStep uses default value based on blank time.
   */
  float pwm_on_time_us{0.0F};  ///< PWM on-time limit in microseconds (0 = auto-calculate)
  
  /**
   * @brief Stall detection sensitivity
   * 
   * Controls stall detection threshold in DcStep mode.
   * Higher sensitivity detects stalls earlier but may trigger false positives.
   * 
   * @note Automatically calculated from pwm_on_time_us if MODERATE/HIGH/LOW selected.
   * @note Set to DISABLED to disable stall detection.
   */
  DcStepStallSensitivity stall_sensitivity{DcStepStallSensitivity::MODERATE};  ///< Stall detection sensitivity
  
  /**
   * @brief Enable stop on stall
   * 
   * If true, motor stops (VACTUAL = 0) when stall is detected.
   * Motor remains stopped until RAMP_STAT.event_stop_sg flag is read.
   * 
   * @note Requires stall_sensitivity != DISABLED.
   * @note Requires StallGuard2 to be configured (TCOOLTHRS set).
   */
  bool stop_on_stall{false};  ///< Stop motor when stall detected

  /**
   * @brief Default constructor
   *
   * Initializes with DcStep disabled (min_velocity = 0).
   * All fields use default member initializers.
   */
  DcStepConfig() = default;
  
  /**
   * @brief Helper constructor for quick setup
   * 
   * Creates a DcStep configuration with typical settings:
   * - Moderate stall sensitivity
   * - Auto-calculated PWM on-time
   * 
   * @param min_vel Minimum velocity for DcStep activation
   * @param vel_unit Unit for velocity threshold
   * @param pwm_time_us PWM on-time limit in microseconds (0 = auto-calculate)
   * @param sensitivity Stall detection sensitivity
   */
  DcStepConfig(float min_vel, Unit vel_unit = Unit::Steps, 
               float pwm_time_us = 0.0F,
               DcStepStallSensitivity sensitivity = DcStepStallSensitivity::MODERATE)
    : min_velocity(min_vel)
    , velocity_unit(vel_unit)
    , pwm_on_time_us(pwm_time_us)
    , stall_sensitivity(sensitivity)
  {}
};    

//===============================================================================================================
//===============================================================================================================
//                            REFERENCE SWITCHES CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================

/**
 * @brief Reference switch active level enumeration
 *
 * Defines the electrical signal level when the switch is active/triggered.
 * Used to automatically configure polarity for proper switch detection.
 * 
 * @note Active level must always be specified (ACTIVE_LOW or ACTIVE_HIGH).
 *       Use stop_enable to control whether the switch stops the motor.
 *       This allows configuring polarity while enabling/disabling stop functionality in real-time.
 */
enum class ReferenceSwitchActiveLevel : uint8_t {
  ACTIVE_LOW,   ///< Switch is active when signal is LOW (GND)
                 ///< Typically used with pull-up resistors (switch connects to GND when active)
                 ///< Common for normally-closed switches and failsafe configurations
                 ///< Recommended for safety-critical applications (broken wire = HIGH = not active = safe)
  ACTIVE_HIGH   ///< Switch is active when signal is HIGH (VCC)
                 ///< Typically used with pull-down resistors (switch connects to VCC when active)
                 ///< Common for normally-open switches and photo interrupters
};

/**
 * @brief Position latching mode enumeration
 *
 * Defines when position should be latched (captured) on switch events.
 * Used to simplify latching configuration instead of separate boolean flags.
 */
enum class ReferenceLatchMode : uint8_t {
  DISABLED,      ///< No position latching (latch disabled)
  ACTIVE_EDGE,   ///< Latch position on active edge (switch becomes active)
                  ///< Most common for homing - captures position when switch triggers
  INACTIVE_EDGE, ///< Latch position on inactive edge (switch becomes inactive)
                  ///< Useful for capturing position when switch releases
  BOTH_EDGES     ///< Latch position on both active and inactive edges
                  ///< Captures position on both switch activation and release
};

/**
 * @brief Stop mode enumeration
 *
 * Defines how the motor stops when a reference switch is triggered.
 */
enum class ReferenceStopMode : uint8_t {
  HARD_STOP,  ///< Abrupt stop (immediate, no deceleration)
              ///< Use for emergency stops and precise homing
              ///< Motor stops exactly at switch position
  SOFT_STOP   ///< Soft stop using deceleration ramp (DMAX, V1, D1)
              ///< Use for normal operation to prevent mechanical shock
              ///< Motor decelerates smoothly to zero velocity
};

/**
 * @brief Reference switch configuration structure
 *
 * Configuration parameters for reference switches/endstops used for homing
 * and limit detection. Supports both mechanical switches and encoder N-channel
 * as a third switch option.
 *
 * @note For reliable homing, follow the complete procedure:
 * 1. Move away from switch (ensure switch is not pressed)
 * 2. Configure switches with appropriate active level and stop mode
 * 3. Start motion toward switch
 * 4. Wait for standstill after switch hit
 * 5. Calculate position offset and set home position
 *
 * @see Datasheet section 12.4: Reference Switches
 */
struct ReferenceSwitchConfig {
  // Switch active level configuration (determines polarity)
  // Active level must always be specified (ACTIVE_LOW or ACTIVE_HIGH)
  // Use stop_enable to control whether switch stops the motor (allows real-time enable/disable)
  ReferenceSwitchActiveLevel left_switch_active{ReferenceSwitchActiveLevel::ACTIVE_LOW};   ///< Left switch active level (REFL)
                                                                                            ///< ACTIVE_LOW = active when LOW (inverted polarity), ACTIVE_HIGH = active when HIGH (normal polarity)
  ReferenceSwitchActiveLevel right_switch_active{ReferenceSwitchActiveLevel::ACTIVE_LOW};  ///< Right switch active level (REFR)
                                                                                            ///< ACTIVE_LOW = active when LOW (inverted polarity), ACTIVE_HIGH = active when HIGH (normal polarity)
  
  // Motor stop enable configuration (independent of active level)
  // Allows enabling/disabling motor stop in real-time while keeping polarity configured
  bool left_switch_stop_enable{false};   ///< Enable automatic motor stop on left switch (independent of active level)
                                          ///< true = stop motor when switch is active, false = don't stop (but can still latch/read switch state)
  bool right_switch_stop_enable{false};  ///< Enable automatic motor stop on right switch (independent of active level)
                                          ///< true = stop motor when switch is active, false = don't stop (but can still latch/read switch state)
  
  // Stop configuration
  ReferenceStopMode stop_mode{ReferenceStopMode::SOFT_STOP};  ///< Stop mode (hard or soft) - only applies if stop is enabled
  bool swap_left_right{false};  ///< Swap left and right switch inputs (useful for reversed wiring)
  
  // Position latching configuration
  ReferenceLatchMode latch_left{ReferenceLatchMode::DISABLED};   ///< Left switch latching mode (must be explicitly set)
  ReferenceLatchMode latch_right{ReferenceLatchMode::DISABLED};  ///< Right switch latching mode (must be explicitly set)
  bool en_latch_encoder{false};  ///< Latch encoder position on switch event (for encoder N-channel as third switch)

  /**
   * @brief Default constructor
   *
   * Initializes with safe defaults:
   * - Active level: ACTIVE_LOW (failsafe)
   * - Stop enable: false (motor doesn't stop)
   * - Latching: DISABLED (no latching)
   * - Stop mode: SOFT_STOP
   */
  ReferenceSwitchConfig() = default;
};

//===============================================================================================================
//===============================================================================================================
//                                ENCODER FEEDBACK CONFIGURATION STRUCTURE                                          
//===============================================================================================================
//===============================================================================================================

/**
 * @brief Encoder N channel sensitivity enumeration
 *
 * Defines how the N channel event is detected (level-sensitive or edge-sensitive).
 * Similar to ReferenceLatchMode but specifically for encoder N channel edge detection.
 */
enum class EncoderNSensitivity : uint8_t {
  ACTIVE_LEVEL,  ///< N channel event is active during active N level (level-sensitive)
                  ///< Register: pos_edge=0, neg_edge=0
  RISING_EDGE,   ///< N channel is valid upon active going edge (positive edge)
                  ///< Register: pos_edge=1, neg_edge=0
  FALLING_EDGE,  ///< N channel is valid upon inactive going edge (negative edge)
                  ///< Register: pos_edge=0, neg_edge=1
  BOTH_EDGES     ///< N channel is valid upon both active and inactive going edges
                  ///< Register: pos_edge=1, neg_edge=1
};

/**
 * @brief Encoder clear mode enumeration
 *
 * Defines when and how the encoder counter is cleared on N channel events.
 */
enum class EncoderClearMode : uint8_t {
  DISABLED,    ///< No clearing or latching (clr_cont=0, clr_once=0)
               ///< N channel events are ignored
  ONCE,        ///< Latch or latch and clear X_ENC on the next N event only (clr_once=1)
               ///< Automatically disables after first N event
               ///< Useful for encoders that give N signal once per revolution
  CONTINUOUS   ///< Always latch or latch and clear X_ENC upon N event (clr_cont=1)
               ///< Continuously monitors N channel and triggers on every event
};

/**
 * @brief Encoder prescaler mode enumeration
 *
 * Defines the encoder prescaler divisor mode for ENC_CONST calculation.
 */
enum class EncoderPrescalerMode : uint8_t {
  BINARY,   ///< Binary mode - Counts ENC_CONST(fractional part) / 65536
            ///< Standard fixed-point 16.16 format
  DECIMAL   ///< Decimal mode - Counts ENC_CONST(fractional part) / 10000
            ///< Decimal representation: FACTOR.DECIMALS (e.g., 25.6 = 0x0019.0x1770)
};

/**
 * @brief Encoder configuration structure
 *
 * Configuration parameters for encoder interface operation.
 * Uses enum-based API for intuitive configuration similar to ReferenceSwitchConfig.
 *
 * @note All fields must be explicitly set. Register values are calculated automatically.
 *
 * @see Datasheet section 20: ABN Incremental Encoder Interface
 */
struct EncoderConfig {
  // N channel configuration (shares ReferenceSwitchActiveLevel enum)
  // Note: Default initialization done in constructor since enum is forward-declared
  ReferenceSwitchActiveLevel n_channel_active{ReferenceSwitchActiveLevel::ACTIVE_LOW};  ///< N channel active level (determines pol_N)
                                                  ///< ACTIVE_LOW = low active (pol_N=0), ACTIVE_HIGH = high active (pol_N=1)
  
  // A/B polarity requirements for N channel validation
  bool require_a_high{false};  ///< Require A channel HIGH for N event validation (pol_A)
                                ///< true = A must be HIGH, false = A must be LOW
  bool require_b_high{false};  ///< Require B channel HIGH for N event validation (pol_B)
                                ///< true = B must be HIGH, false = B must be LOW
  bool ignore_ab_polarity{true};  ///< Ignore A and B polarity for N channel event (ignore_AB)
                                   ///< true = ignore A/B, false = validate A/B polarity
  
  // N channel event detection
  EncoderNSensitivity n_sensitivity{EncoderNSensitivity::ACTIVE_LEVEL};  ///< N channel event sensitivity (edge/level detection)
  
  // Clear/latch mode
  EncoderClearMode clear_mode{EncoderClearMode::DISABLED};  ///< Clear mode for encoder counter on N events
  
  // Additional encoder features
  bool clear_enc_x_on_event{false};  ///< Clear encoder counter X_ENC upon N-event (clr_enc_x)
                                      ///< true = latch and clear, false = latch only
  bool latch_xactual_with_enc{false}; ///< Also latch XACTUAL position together with X_ENC (latch_x_act)
                                       ///< true = latch both, false = latch X_ENC only
  
  // Prescaler mode
  EncoderPrescalerMode prescaler_mode{EncoderPrescalerMode::BINARY};  ///< Encoder prescaler divisor mode (enc_sel_decimal)

  /**
   * @brief Default constructor
   *
   * Initializes with safe defaults:
   * - N channel: ACTIVE_HIGH, ACTIVE_LEVEL sensitivity
   * - A/B polarity: ignored
   * - Clear mode: DISABLED
   * - Prescaler: BINARY
   */
  EncoderConfig() = default;
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
