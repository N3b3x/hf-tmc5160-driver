/**
 * @file tmc51x0_config_builder.hpp
 * @brief Fluent configuration builder for TMC51x0 driver
 * 
 * Provides a fluent interface for building DriverConfig with:
 * - Units in milliamps, millivolts, milliohms, millihenries
 * - Chainable method calls for easy configuration
 * - Complete coverage of all DriverConfig fields
 * - Mimics the ESP32 test config initialization pattern
 * 
 * ## Unit Naming Convention
 * 
 * Functions follow a clear unit naming convention for safety and readability:
 * 
 * - **Unit suffixes for fixed units**: Functions that always use a specific unit
 *   have the unit suffix in the function name:
 *   - `WithMotorMa()` - rated current in milliamps
 *   - `WithRunCurrentMa()` - run current in milliamps
 *   - `WithHoldCurrentMa()` - hold current in milliamps
 *   - `WithSupplyVoltageMv()` - supply voltage in millivolts
 *   - `WithSenseResistorMohm()` - sense resistor in milliohms
 *   - `WithWindingResistanceMohm()` - winding resistance in milliohms
 *   - `WithWindingInductanceMh()` - winding inductance in millihenries
 *   - `WithMotorPowerDownDelayMs()` - always milliseconds
 *   - `WithRampPowerDownDelayMs()` - always milliseconds
 *   - `WithZeroWaitTimeMs()` - always milliseconds
 *   - `WithBbmTimeNs()` - always nanoseconds
 *   - `WithS2vsVoltageMv()` - always millivolts
 *   - `WithS2gVoltageMv()` - always millivolts
 *   - `WithMosfetMillerChargeNc()` - always nanocoulombs
 *   - `WithShortDetectionDelayUsX10()` - always 0.1µs units
 *   - `WithExternalClockHz()` - clock frequency in Hz
 * 
 * - **Unit-agnostic for flexible units**: Functions that accept different units
 *   use self-describing value types (VelocityValue, AccelerationValue):
 *   - `WithMaxSpeed(VelocityValue)` - can be Steps, RPM, RevPerSec, etc.
 *   - `WithAcceleration(AccelerationValue)` - can be Steps, RevPerSec, etc.
 *   - `WithStealthChopThreshold(VelocityValue)` - can be Steps, RPM, etc.
 * 
 * This convention ensures type safety and prevents unit conversion errors.
 * 
 * The builder follows the same pattern as ESP32 test config:
 * 1. Motor configuration (motor specs, chopper, StealthChop, StallGuard, ramp defaults)
 * 2. Board configuration (sense resistor, power stage, clock)
 * 3. Platform configuration (mechanical system type)
 * 
 * @code
 * auto config = ConfigBuilder()
 *     // Motor configuration
 *     .WithMotorMa(200, 1500)  // 200 steps, 1500mA = 1.5A rated
 *     .WithSupplyVoltageMv(24000)  // 24000mV = 24V
 *     .WithSenseResistorMohm(50)  // 50mΩ = 0.05Ω
 *     .WithRunCurrentMa(1500)  // 1500mA = 1.5A run current
 *     .WithHoldCurrentMa(750)  // 750mA = 0.75A hold current
 *     // Chopper configuration
 *     .WithChopperMode(ChopperMode::SPREAD_CYCLE)
 *     .WithChopperToff(5)
 *     .WithChopperBlankTime(ChopperBlankTime::TBL_36CLK)
 *     // StealthChop configuration
 *     .WithStealthChop(true)
 *     .WithStealthChopThreshold({800.0f, Unit::Steps})
 *     // Motion configuration
 *     .WithMaxSpeed({100.0f, Unit::RPM})
 *     .WithAcceleration({50.0f, Unit::RevPerSec})
 *     // Mechanical system
 *     .WithGearbox(5.18f)  // 5.18:1 gearbox
 *     .Build();
 * @endcode
 */

#ifndef TMC51X0_CONFIG_BUILDER_HPP
#define TMC51X0_CONFIG_BUILDER_HPP

#include <cstdint>
#include "../tmc51x0_types.hpp"

namespace tmc51x0 {

/**
 * @brief Fluent configuration builder for DriverConfig
 * 
 * Provides a user-friendly interface for building TMC51x0 configurations
 * with automatic unit conversions. All motor-specific presets are removed
 * to allow users full control over configuration.
 * 
 * The builder ensures all DriverConfig fields are properly initialized,
 * following the same pattern as ESP32 test config initialization.
 */
class ConfigBuilder {
private:
    DriverConfig config_;

public:
    /**
     * @brief Default constructor
     * 
     * Creates a builder with default configuration values.
     * All DriverConfig fields use their default constructors.
     */
    ConfigBuilder() = default;

    /**
     * @brief Construct from existing config
     * 
     * @param cfg Existing DriverConfig to start from
     */
    explicit ConfigBuilder(const DriverConfig& cfg) : config_(cfg) {}

    // ========== MOTOR SPECIFICATION ==========

    /**
     * @brief Configure motor specifications
     * 
     * Sets basic motor parameters: steps per revolution and rated current.
     * 
     * @param steps_per_rev Steps per revolution (e.g., 200 for 1.8° motors)
     * @param rated_current_ma Rated motor current in milliamps (e.g., 1500 for 1.5A)
     * @return Reference to builder for chaining
     * 
     * @code
     * builder.WithMotorMa(200, 1500);  // 200 steps, 1500mA = 1.5A rated
     * @endcode
     */
    ConfigBuilder& WithMotorMa(uint16_t steps_per_rev, uint16_t rated_current_ma) {
        config_.motor_spec.steps_per_rev = steps_per_rev;
        config_.motor_spec.rated_current_ma = rated_current_ma;
        return *this;
    }

    /**
     * @brief Set run current value
     * 
     * @param run_current_ma Run current in milliamps (0 = use rated current)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithRunCurrentMa(uint16_t run_current_ma) {
        config_.motor_spec.run_current_ma = run_current_ma;
        return *this;
    }

    /**
     * @brief Set hold current value
     * 
     * @param hold_current_ma Hold current in milliamps (0 = auto-calculate as 30% of run)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithHoldCurrentMa(uint16_t hold_current_ma) {
        config_.motor_spec.hold_current_ma = hold_current_ma;
        return *this;
    }

    /**
     * @brief Set supply voltage value
     * 
     * @param voltage_mv Supply voltage in millivolts (e.g., 24000 for 24V)
     * @return Reference to builder for chaining
     * 
     * @code
     * builder.WithSupplyVoltageMv(24000);  // 24000mV = 24V
     * @endcode
     */
    ConfigBuilder& WithSupplyVoltageMv(uint32_t voltage_mv) {
        config_.motor_spec.supply_voltage_mv = voltage_mv;
        return *this;
    }

    /**
     * @brief Set sense resistor value
     * 
     * Required for automatic current calculation.
     * 
     * @param resistance_mohm Sense resistor in milliohms (e.g., 50 for 0.05Ω)
     * @return Reference to builder for chaining
     * 
     * @code
     * builder.WithSenseResistorMohm(50);  // 50mΩ = 0.05Ω
     * @endcode
     */
    ConfigBuilder& WithSenseResistorMohm(uint32_t resistance_mohm) {
        config_.motor_spec.sense_resistor_mohm = resistance_mohm;
        return *this;
    }

    /**
     * @brief Set winding resistance value
     * 
     * Required for StealthChop lower limit calculation.
     * 
     * @param resistance_mohm Winding resistance in milliohms
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithWindingResistanceMohm(uint32_t resistance_mohm) {
        config_.motor_spec.winding_resistance_mohm = resistance_mohm;
        return *this;
    }

    /**
     * @brief Set winding inductance value
     * 
     * Used for StealthChop configuration.
     * 
     * @param inductance_mh Winding inductance in millihenries
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithWindingInductanceMh(float inductance_mh) {
        config_.motor_spec.winding_inductance_mh = inductance_mh;
        return *this;
    }

    /**
     * @brief Set motor power down delay time
     * 
     * Controls the total delay time for motor power down after standstill.
     * This is the IHOLDDELAY setting that controls the smooth transition
     * from run current to hold current.
     * 
     * @param delay_ms Total delay time in milliseconds (0 = instant)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithMotorPowerDownDelayMs(float delay_ms) {
        config_.motor_spec.iholddelay_ms = delay_ms;
        return *this;
    }

    /**
     * @brief Set steps per revolution
     * 
     * @param steps Steps per revolution (e.g., 200 for 1.8° motor)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStepsPerRevolution(uint16_t steps) {
        config_.motor_spec.steps_per_rev = steps;
        return *this;
    }

    // ========== BOARD CONFIGURATION (Power Stage) ==========

    /**
     * @brief Set MOSFET Miller charge
     * 
     * Used to calculate DRVSTRENGTH register.
     * 
     * @param charge_nc MOSFET Miller charge in nanocoulombs (0 = auto-calculate)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithMosfetMillerChargeNc(float charge_nc) {
        config_.power_stage.mosfet_miller_charge_nc = charge_nc;
        return *this;
    }

    /**
     * @brief Set Break Before Make time
     * 
     * @param time_ns BBM time in nanoseconds (0 = auto-calculate)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithBbmTimeNs(uint32_t time_ns) {
        config_.power_stage.bbm_time_ns = time_ns;
        return *this;
    }

    /**
     * @brief Set sense amplifier filter time
     * 
     * @param filter_time Sense filter time constant
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithSenseFilter(SenseFilterTime filter_time) {
        config_.power_stage.sense_filter = filter_time;
        return *this;
    }

    /**
     * @brief Set over-temperature protection level
     * 
     * @param protection Over-temperature protection level
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithOverTempProtection(OverTempProtection protection) {
        config_.power_stage.over_temp_protection = protection;
        return *this;
    }

    /**
     * @brief Set short to VS detector voltage threshold
     * 
     * @param voltage_mv Voltage threshold in millivolts (0 = auto-calculate)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithS2vsVoltageMv(uint16_t voltage_mv) {
        config_.power_stage.s2vs_voltage_mv = voltage_mv;
        return *this;
    }

    /**
     * @brief Set short to GND detector voltage threshold
     * 
     * @param voltage_mv Voltage threshold in millivolts (0 = auto-calculate)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithS2gVoltageMv(uint16_t voltage_mv) {
        config_.power_stage.s2g_voltage_mv = voltage_mv;
        return *this;
    }

    /**
     * @brief Set short detection filter bandwidth
     * 
     * @param filter Filter bandwidth (0-3, 0=lowest, 1=default)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithShortFilter(uint8_t filter) {
        config_.power_stage.shortfilter = filter;
        return *this;
    }

    /**
     * @brief Set short detection delay
     * 
     * @param delay_us_x10 Delay in 0.1µs units (0 = auto-calculate)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithShortDetectionDelayUsX10(uint8_t delay_us_x10) {
        config_.power_stage.short_detection_delay_us_x10 = delay_us_x10;
        return *this;
    }

    // ========== CHOPPER CONFIGURATION ==========

    /**
     * @brief Set chopper mode
     * 
     * @param mode Chopper mode (SPREAD_CYCLE recommended)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithChopperMode(ChopperMode mode) {
        config_.chopper.mode = mode;
        return *this;
    }

    /**
     * @brief Set chopper off-time (TOFF)
     * 
     * @param toff Off-time value (0-15, 0=driver disable, 5=typical)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithChopperToff(uint8_t toff) {
        config_.chopper.toff = toff;
        return *this;
    }

    /**
     * @brief Set chopper blank time
     * 
     * @param blank_time Blank time (TBL_36CLK typical)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithChopperBlankTime(ChopperBlankTime blank_time) {
        config_.chopper.tbl = static_cast<uint8_t>(blank_time);
        return *this;
    }

    /**
     * @brief Set hysteresis start value (SpreadCycle mode)
     * 
     * @param hstrt Hysteresis start (0-7, 4=typical)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithHysteresisStart(uint8_t hstrt) {
        config_.chopper.hstrt = hstrt;
        return *this;
    }

    /**
     * @brief Set hysteresis end value (SpreadCycle mode)
     * 
     * @param hend Hysteresis end (0-15, 0=typical)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithHysteresisEnd(uint8_t hend) {
        config_.chopper.hend = hend;
        return *this;
    }

    /**
     * @brief Set fast decay time (Classic mode only)
     * 
     * @param tfd Fast decay time (0-15, 0=slow decay only)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithFastDecayTime(uint8_t tfd) {
        config_.chopper.tfd = tfd;
        return *this;
    }

    /**
     * @brief Set passive fast decay time
     * 
     * @param tpfd Passive fast decay time (0=disabled, 1-15)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithPassiveFastDecayTime(uint8_t tpfd) {
        config_.chopper.tpfd = tpfd;
        return *this;
    }

    /**
     * @brief Set microstep resolution
     * 
     * @param mres Microstep resolution (MRES_256 typical)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithMicrostepResolution(MicrostepResolution mres) {
        config_.chopper.mres = mres;
        return *this;
    }

    /**
     * @brief Enable/disable interpolation
     * 
     * @param enable True to enable interpolation to 256 microsteps
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithInterpolation(bool enable) {
        config_.chopper.intpol = enable;
        return *this;
    }

    // ========== STEALTHCHOP CONFIGURATION ==========

    /**
     * @brief Enable/disable StealthChop
     * 
     * @param enable True to enable StealthChop, false for SpreadCycle
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStealthChop(bool enable) {
        config_.global_config.en_stealthchop_mode = enable;
        return *this;
    }

    /**
     * @brief Set StealthChop threshold velocity
     * 
     * Velocity below which StealthChop is active, above which SpreadCycle is used.
     * 
     * @param velocity Threshold velocity with unit
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStealthChopThreshold(const VelocityValue& velocity) {
        config_.stealthchop.velocity_threshold = velocity.value;
        config_.stealthchop.velocity_threshold_unit = velocity.unit;
        return *this;
    }

    /**
     * @brief Set StealthChop PWM frequency
     * 
     * @param freq PWM frequency (0-3, 1=~35kHz typical)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStealthChopPwmFreq(uint8_t freq) {
        config_.stealthchop.pwm_freq = freq;
        return *this;
    }

    /**
     * @brief Set StealthChop PWM offset
     * 
     * @param ofs PWM offset (0-255, 20-40 typical)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStealthChopPwmOfs(uint8_t ofs) {
        config_.stealthchop.pwm_ofs = ofs;
        return *this;
    }

    /**
     * @brief Enable/disable StealthChop PWM autoscale
     * 
     * @param enable True to enable automatic PWM amplitude calibration
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStealthChopAutoscale(bool enable) {
        config_.stealthchop.pwm_autoscale = enable;
        return *this;
    }

    /**
     * @brief Enable/disable StealthChop PWM autograd
     * 
     * @param enable True to enable automatic PWM gradient calibration
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStealthChopAutograd(bool enable) {
        config_.stealthchop.pwm_autograd = enable;
        return *this;
    }

    // ========== STALLGUARD CONFIGURATION ==========

    /**
     * @brief Set StallGuard2 threshold (SGT)
     * 
     * Controls StallGuard2 sensitivity for stall detection.
     * 
     * @param threshold SGT value (-64 to +63)
     *                  - Lower values: Higher sensitivity (detects stalls easier)
     *                  - Higher values: Lower sensitivity (needs more torque to detect)
     *                  - 0: Starting value, works with most motors
     * @return Reference to builder for chaining
     * 
     * @code
     * builder.WithStallGuardThreshold(10);   // Less sensitive
     * builder.WithStallGuardThreshold(-10);  // More sensitive
     * @endcode
     */
    ConfigBuilder& WithStallGuardThreshold(int8_t threshold) {
        config_.stallguard.threshold = threshold;
        return *this;
    }

    /**
     * @brief Enable/disable StallGuard2 filter
     * 
     * Filter reduces measurement rate to one per electrical period (4 fullsteps).
     * 
     * @param enable True to enable filter (smoother, recommended for CoolStep),
     *               False to disable (faster response, recommended for homing)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStallGuardFilter(bool enable) {
        config_.stallguard.enable_filter = enable;
        return *this;
    }

    /**
     * @brief Set StallGuard2 minimum velocity threshold
     * 
     * StallGuard readings are only valid above this velocity.
     * Below this speed, stall detection is unreliable.
     * 
     * @param velocity Minimum velocity value
     * @param unit Velocity unit (default: RPM)
     * @return Reference to builder for chaining
     * 
     * @code
     * builder.WithStallGuardMinVelocity(20.0f, Unit::RPM);  // 20 RPM minimum
     * @endcode
     */
    ConfigBuilder& WithStallGuardMinVelocity(float velocity, Unit unit = Unit::RPM) {
        config_.stallguard.min_velocity = velocity;
        config_.stallguard.velocity_unit = unit;
        return *this;
    }

    /**
     * @brief Set StallGuard2 maximum velocity threshold
     * 
     * StallGuard is disabled above this velocity. Set to 0 to disable limit.
     * 
     * @param velocity Maximum velocity value (0 = no limit)
     * @param unit Velocity unit (default: RPM)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStallGuardMaxVelocity(float velocity, Unit unit = Unit::RPM) {
        config_.stallguard.max_velocity = velocity;
        config_.stallguard.velocity_unit = unit;
        return *this;
    }

    /**
     * @brief Configure StallGuard2 with a complete config struct
     * 
     * Sets all StallGuard parameters at once.
     * 
     * @param sg_config Complete StallGuard configuration
     * @return Reference to builder for chaining
     * 
     * @code
     * StallGuardConfig sg{};
     * sg.threshold = -5;
     * sg.enable_filter = false;
     * sg.min_velocity = 20.0f;
     * sg.velocity_unit = Unit::RPM;
     * builder.WithStallGuard(sg);
     * @endcode
     */
    ConfigBuilder& WithStallGuard(const StallGuardConfig& sg_config) {
        config_.stallguard = sg_config;
        return *this;
    }

    // ========== GLOBAL CONFIGURATION ==========

    /**
     * @brief Enable/disable recalibration
     * 
     * @param enable True to enable recalibration
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithRecalibration(bool enable) {
        config_.global_config.recalibrate = enable;
        return *this;
    }

    /**
     * @brief Enable/disable short standstill timeout
     * 
     * @param enable True to enable short standstill timeout
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithShortStandstillTimeout(bool enable) {
        config_.global_config.en_short_standstill_timeout = enable;
        return *this;
    }

    /**
     * @brief Enable/disable StealthChop step filter
     * 
     * @param enable True to enable step filter for StealthChop
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStealthChopStepFilter(bool enable) {
        config_.global_config.en_stealthchop_step_filter = enable;
        return *this;
    }

    // ========== MOTION CONFIGURATION (Ramp) ==========

    /**
     * @brief Set maximum velocity
     * 
     * @param velocity Self-describing velocity with unit
     * @return Reference to builder for chaining
     * 
     * @code
     * builder.WithMaxSpeed({100.0f, Unit::RPM});
     * builder.WithMaxSpeed(VelocityValue::FromRPM(100.0f));
     * @endcode
     */
    ConfigBuilder& WithMaxSpeed(const VelocityValue& velocity) {
        config_.ramp_config.vmax = velocity;
        return *this;
    }

    /**
     * @brief Set maximum acceleration
     * 
     * @param acceleration Self-describing acceleration with unit
     * @return Reference to builder for chaining
     * 
     * @code
     * builder.WithAcceleration({50.0f, Unit::RevPerSec});
     * builder.WithAcceleration(AccelerationValue::FromRevPerSec(50.0f));
     * @endcode
     */
    ConfigBuilder& WithAcceleration(const AccelerationValue& acceleration) {
        config_.ramp_config.amax = acceleration;
        return *this;
    }

    /**
     * @brief Set maximum deceleration
     * 
     * @param deceleration Self-describing deceleration with unit
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithDeceleration(const AccelerationValue& deceleration) {
        config_.ramp_config.dmax = deceleration;
        return *this;
    }

    /**
     * @brief Set start velocity
     * 
     * @param velocity Self-describing velocity with unit
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStartSpeed(const VelocityValue& velocity) {
        config_.ramp_config.vstart = velocity;
        return *this;
    }

    /**
     * @brief Set stop velocity
     * 
     * @param velocity Self-describing velocity with unit
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithStopSpeed(const VelocityValue& velocity) {
        config_.ramp_config.vstop = velocity;
        return *this;
    }

    /**
     * @brief Set velocity threshold for acceleration/deceleration phase transition
     * 
     * @param velocity Self-describing velocity with unit
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithVelocityThreshold(const VelocityValue& velocity) {
        config_.ramp_config.v1 = velocity;
        return *this;
    }

    /**
     * @brief Set first acceleration phase
     * 
     * @param acceleration Self-describing acceleration with unit
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithFirstAcceleration(const AccelerationValue& acceleration) {
        config_.ramp_config.a1 = acceleration;
        return *this;
    }

    /**
     * @brief Set first deceleration phase
     * 
     * @param deceleration Self-describing deceleration with unit
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithFirstDeceleration(const AccelerationValue& deceleration) {
        config_.ramp_config.d1 = deceleration;
        return *this;
    }

    /**
     * @brief Set ramp power down delay
     * 
     * Sets the TPOWERDOWN delay - time before motor power down after standstill.
     * This is different from motor power down delay (IHOLDDELAY).
     * 
     * @param delay_ms Power down delay in milliseconds
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithRampPowerDownDelayMs(float delay_ms) {
        config_.ramp_config.tpowerdown_ms = delay_ms;
        return *this;
    }

    /**
     * @brief Set zero velocity wait time
     * 
     * @param wait_ms Wait time at zero velocity in milliseconds
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithZeroWaitTimeMs(float wait_ms) {
        config_.ramp_config.tzerowait_ms = wait_ms;
        return *this;
    }

    // ========== CLOCK CONFIGURATION ==========

    /**
     * @brief Use internal clock (12MHz)
     * 
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithInternalClock() {
        config_.external_clk_config.frequency_hz = 0;
        return *this;
    }

    /**
     * @brief Use external clock
     * 
     * @param frequency_hz External clock frequency in Hz
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithExternalClockHz(uint32_t frequency_hz) {
        config_.external_clk_config.frequency_hz = frequency_hz;
        return *this;
    }

    // ========== MECHANICAL SYSTEM ==========

    /**
     * @brief Configure direct drive system
     * 
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithDirectDrive() {
        config_.mechanical.system_type = MechanicalSystemType::DirectDrive;
        config_.mechanical.gear_ratio = 1.0f;
        config_.mechanical.lead_screw_pitch_mm = 0.0f;
        config_.mechanical.belt_pulley_teeth = 0;
        config_.mechanical.belt_pitch_mm = 0.0f;
        return *this;
    }

    /**
     * @brief Configure gearbox system
     * 
     * @param gear_ratio Gear ratio (output/input, e.g., 5.18 for 5.18:1)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithGearbox(float gear_ratio) {
        config_.mechanical.system_type = MechanicalSystemType::Gearbox;
        config_.mechanical.gear_ratio = gear_ratio;
        config_.mechanical.lead_screw_pitch_mm = 0.0f;
        config_.mechanical.belt_pulley_teeth = 0;
        config_.mechanical.belt_pitch_mm = 0.0f;
        return *this;
    }

    /**
     * @brief Configure lead screw system
     * 
     * @param pitch_mm Lead screw pitch in mm
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithLeadScrew(float pitch_mm) {
        config_.mechanical.system_type = MechanicalSystemType::LeadScrew;
        config_.mechanical.lead_screw_pitch_mm = pitch_mm;
        config_.mechanical.gear_ratio = 1.0f;
        config_.mechanical.belt_pulley_teeth = 0;
        config_.mechanical.belt_pitch_mm = 0.0f;
        return *this;
    }

    /**
     * @brief Configure belt drive system
     * 
     * @param pulley_teeth Number of pulley teeth
     * @param belt_pitch_mm Belt pitch in mm (e.g., 2.0 for GT2)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithBeltDrive(uint16_t pulley_teeth, float belt_pitch_mm) {
        config_.mechanical.system_type = MechanicalSystemType::BeltDrive;
        config_.mechanical.belt_pulley_teeth = pulley_teeth;
        config_.mechanical.belt_pitch_mm = belt_pitch_mm;
        config_.mechanical.gear_ratio = 1.0f;
        config_.mechanical.lead_screw_pitch_mm = 0.0f;
        return *this;
    }

    // ========== DIRECTION ==========

    /**
     * @brief Set motor direction
     * 
     * @param direction Motor direction (NORMAL or INVERSE)
     * @return Reference to builder for chaining
     */
    ConfigBuilder& WithDirection(MotorDirection direction) {
        config_.direction = direction;
        return *this;
    }

    // ========== BUILD ==========

    /**
     * @brief Build the final configuration
     * 
     * Returns a complete DriverConfig with all fields properly initialized.
     * The configuration follows the same pattern as ESP32 test config:
     * - Motor configuration (motor_spec, chopper, stealthchop, ramp_config, global_config, direction, mechanical gear_ratio)
     * - Board configuration (power_stage, sense_resistor_mohm, external_clk_config)
     * - Platform configuration (mechanical system_type)
     * 
     * @return The built DriverConfig
     */
    DriverConfig Build() const {
        return config_;
    }

    /**
     * @brief Get mutable reference to config for advanced customization
     * 
     * Allows direct access to the internal config for advanced use cases
     * where the fluent interface doesn't provide enough control.
     * 
     * @return Reference to internal config
     */
    DriverConfig& Config() {
        return config_;
    }

    /**
     * @brief Get const reference to config
     * 
     * @return Const reference to internal config
     */
    const DriverConfig& Config() const {
        return config_;
    }
};

} // namespace tmc51x0

#endif // TMC51X0_CONFIG_BUILDER_HPP
