/**
 * @file tmc5160.cpp
 * @brief Implementation of TMC5160 driver methods
 *
 * This file contains the template implementation of all TMC5160 driver methods.
 * It is included by tmc5160.hpp to provide header-only template instantiation.
 */

#ifndef TMC5160_IMPL
#define TMC5160_IMPL

// When included from header, use relative path; when compiled directly, use
// standard include
#ifdef TMC5160_HEADER_INCLUDED
// Already included from header - the class definition is available in the
// current context We're inside the namespace, so we can access the template
// class No need to include header or open namespace
#else
// Not included from header (shouldn't happen for template implementation)
#include "../inc/tmc5160.hpp"
#endif

#include <algorithm>
#include <cmath>

#include "../inc/tmc5160_units.hpp"
#include "../inc/tmc5160_motor_calc.hpp"

using namespace tmc5160;

// Implementation of chip communication mode control methods
template <typename CommType>
bool TMC5160<CommType>::SetChipCommMode(ChipCommMode mode) noexcept {
  const char* mode_name = (mode == ChipCommMode::SPI_INTERNAL_RAMP) ? "SPI_INTERNAL_RAMP" :
                          (mode == ChipCommMode::SPI_EXTERNAL_STEPDIR) ? "SPI_EXTERNAL_STEPDIR" :
                          (mode == ChipCommMode::UART_INTERNAL_RAMP) ? "UART_INTERNAL_RAMP" : "UNKNOWN";
  TMC5160_LOG_DEBUG(comm_, 2, "TMC5160", "SetChipCommMode(%s)", mode_name);

  // Map mode to SPI_MODE and SD_MODE pin states
  GpioSignal spi_mode_signal, sd_mode_signal;
  
  switch (mode) {
    case ChipCommMode::SPI_INTERNAL_RAMP:
      spi_mode_signal = GpioSignal::ACTIVE;  // HIGH
      sd_mode_signal = GpioSignal::INACTIVE; // LOW
      break;
    case ChipCommMode::SPI_EXTERNAL_STEPDIR:
      spi_mode_signal = GpioSignal::ACTIVE;  // HIGH
      sd_mode_signal = GpioSignal::ACTIVE;   // HIGH
      break;
    case ChipCommMode::UART_INTERNAL_RAMP:
      spi_mode_signal = GpioSignal::INACTIVE; // LOW
      sd_mode_signal = GpioSignal::INACTIVE;  // LOW
      break;
    default:
      return false;
  }
  
  // Set SPI_MODE pin
  if (!this->comm_.GpioSet(TMC5160CtrlPin::SPI_MODE, spi_mode_signal)) {
    return false;
  }
  
  // Set SD_MODE pin
  if (!this->comm_.GpioSet(TMC5160CtrlPin::SD_MODE, sd_mode_signal)) {
    return false;
  }
  
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::GetChipCommMode(ChipCommMode& mode) const noexcept {
  GpioSignal spi_mode_signal, sd_mode_signal;
  
  // Read SPI_MODE pin
  if (!this->comm_.GpioRead(TMC5160CtrlPin::SPI_MODE, spi_mode_signal)) {
    return false;
  }
  
  // Read SD_MODE pin
  if (!this->comm_.GpioRead(TMC5160CtrlPin::SD_MODE, sd_mode_signal)) {
    return false;
  }
  
  // Determine mode from pin states
  if (spi_mode_signal == GpioSignal::ACTIVE) {
    // SPI_MODE = HIGH
    if (sd_mode_signal == GpioSignal::ACTIVE) {
      mode = ChipCommMode::SPI_EXTERNAL_STEPDIR;
    } else {
      mode = ChipCommMode::SPI_INTERNAL_RAMP;
    }
  } else {
    // SPI_MODE = LOW (UART mode)
    mode = ChipCommMode::UART_INTERNAL_RAMP;
  }
  
  return true;
}

// Helper function to constrain value between min and max
template <typename T>
static constexpr T constrain(T value, T min_val, T max_val) noexcept {
  if (value < min_val) {
    return min_val;
  }
  if (value > max_val) {
    return max_val;
  }
  return value;
}

// Implementation of unit conversion helpers
template <typename CommType>
float TMC5160<CommType>::convertSpeedToSteps(float value, Unit unit) const noexcept {
  if (value == 0.0f) return 0.0f;
  
  // Calculate effective steps per revolution (full steps * microsteps * gear ratio)
  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) * 
                                  static_cast<float>(current_microsteps_) * 
                                  mechanical_system_.gear_ratio;

  switch (unit) {
    case Unit::Steps:
      return value;
    case Unit::RPM:
      // RPM * steps_per_rev / 60
      return (value * effective_steps_per_rev) / 60.0f;
    case Unit::Rad:
      // rad/s * steps_per_rev / (2*PI)
      return (value * effective_steps_per_rev) / (2.0f * 3.14159265359f);
    case Unit::Deg:
      // deg/s * steps_per_rev / 360
      return (value * effective_steps_per_rev) / 360.0f;
    case Unit::Mm:
      if (mechanical_system_.system_type == MechanicalSystemType::LeadScrew && mechanical_system_.lead_screw_pitch_mm > 0.0f) {
        // mm/s / pitch * steps_per_rev
        return (value / mechanical_system_.lead_screw_pitch_mm) * effective_steps_per_rev;
      } else if (mechanical_system_.system_type == MechanicalSystemType::BeltDrive && mechanical_system_.belt_pitch_mm > 0.0f && mechanical_system_.belt_pulley_teeth > 0) {
        // mm/s / (pitch * teeth) * steps_per_rev
        float mm_per_rev = mechanical_system_.belt_pitch_mm * static_cast<float>(mechanical_system_.belt_pulley_teeth);
        return (value / mm_per_rev) * effective_steps_per_rev;
      }
      return 0.0f; // Invalid config for mm
    default:
      return value;
  }
}

template <typename CommType>
float TMC5160<CommType>::convertAccelerationToSteps(float value, Unit unit) const noexcept {
  // Acceleration conversions are same as velocity (per second squared)
  return convertSpeedToSteps(value, unit);
}

template <typename CommType>
float TMC5160<CommType>::convertPositionToSteps(float value, Unit unit) const noexcept {
  // Position conversions logic is same as speed (time unit cancels out)
  // e.g. RPM (revs) -> Steps (revs * steps/rev)
  // rad -> Steps
  // mm -> Steps
  // Note: For RPM input as position, it implies "Revolutions".
  // Ideally we'd have a separate Unit enum for Position (Revs, Rads, Degs, Mm) vs Speed (RPM, Rad/s...)
  // But we are reusing Unit.
  // Unit::RPM for position -> treated as Revolutions
  
  if (value == 0.0f) return 0.0f;
  
  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) * 
                                  static_cast<float>(current_microsteps_) * 
                                  mechanical_system_.gear_ratio;

  switch (unit) {
    case Unit::Steps:
      return value;
    case Unit::RPM: // Treated as Revolutions
      return value * effective_steps_per_rev;
    case Unit::Rad:
      return (value * effective_steps_per_rev) / (2.0f * 3.14159265359f);
    case Unit::Deg:
      return (value * effective_steps_per_rev) / 360.0f;
    case Unit::Mm:
      if (mechanical_system_.system_type == MechanicalSystemType::LeadScrew && mechanical_system_.lead_screw_pitch_mm > 0.0f) {
        return (value / mechanical_system_.lead_screw_pitch_mm) * effective_steps_per_rev;
      } else if (mechanical_system_.system_type == MechanicalSystemType::BeltDrive && mechanical_system_.belt_pitch_mm > 0.0f && mechanical_system_.belt_pulley_teeth > 0) {
        float mm_per_rev = mechanical_system_.belt_pitch_mm * static_cast<float>(mechanical_system_.belt_pulley_teeth);
        return (value / mm_per_rev) * effective_steps_per_rev;
      }
      return 0.0f;
    default:
      return value;
  }
}

template <typename CommType>
float TMC5160<CommType>::convertStepsToUnit(int32_t steps, Unit unit) const noexcept {
  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) * 
                                  static_cast<float>(current_microsteps_) * 
                                  mechanical_system_.gear_ratio;
  
  if (effective_steps_per_rev == 0.0f) return 0.0f;
  float val = static_cast<float>(steps);

  switch (unit) {
    case Unit::Steps:
      return val;
    case Unit::RPM: // Revolutions
      return val / effective_steps_per_rev;
    case Unit::Rad:
      return (val / effective_steps_per_rev) * (2.0f * 3.14159265359f);
    case Unit::Deg:
      return (val / effective_steps_per_rev) * 360.0f;
    case Unit::Mm:
      if (mechanical_system_.system_type == MechanicalSystemType::LeadScrew) {
        return (val / effective_steps_per_rev) * mechanical_system_.lead_screw_pitch_mm;
      } else if (mechanical_system_.system_type == MechanicalSystemType::BeltDrive) {
        float mm_per_rev = mechanical_system_.belt_pitch_mm * static_cast<float>(mechanical_system_.belt_pulley_teeth);
        return (val / effective_steps_per_rev) * mm_per_rev;
      }
      return 0.0f;
    default:
      return val;
  }
}

template <typename CommType>
float TMC5160<CommType>::convertSpeedToUnit(float steps_per_sec, Unit unit) const noexcept {
  // Logic similar to steps->unit but scaling for time if needed
  // Steps/s -> RPM: (Steps/s / Steps/rev) * 60
  
  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) * 
                                  static_cast<float>(current_microsteps_) * 
                                  mechanical_system_.gear_ratio;
                                  
  if (effective_steps_per_rev == 0.0f) return 0.0f;

  switch (unit) {
    case Unit::Steps:
      return steps_per_sec;
    case Unit::RPM:
      return (steps_per_sec / effective_steps_per_rev) * 60.0f;
    case Unit::Rad:
      return (steps_per_sec / effective_steps_per_rev) * (2.0f * 3.14159265359f);
    case Unit::Deg:
      return (steps_per_sec / effective_steps_per_rev) * 360.0f;
    case Unit::Mm:
      if (mechanical_system_.system_type == MechanicalSystemType::LeadScrew) {
        return (steps_per_sec / effective_steps_per_rev) * mechanical_system_.lead_screw_pitch_mm;
      } else if (mechanical_system_.system_type == MechanicalSystemType::BeltDrive) {
        float mm_per_rev = mechanical_system_.belt_pitch_mm * static_cast<float>(mechanical_system_.belt_pulley_teeth);
        return (steps_per_sec / effective_steps_per_rev) * mm_per_rev;
      }
      return 0.0f;
    default:
      return steps_per_sec;
  }
}

template <typename CommType>
bool TMC5160<CommType>::Initialize(const DriverConfig& config) noexcept {
  TMC5160_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize(toff=%u, mres=%u)",
                     config.chopper.toff, config.chopper.mres);

  // Store physical configuration
  motor_spec_ = config.motor_spec;
  mechanical_system_ = config.mechanical;
  
  // Calculate initial microsteps from config
  // mres: 0=256, 1=128, ... 8=fullstep. microsteps = 256 >> mres
  uint8_t mres = constrain<uint8_t>(config.chopper.mres, 0U, 8U);
  current_microsteps_ = 256U >> mres;

  // Always calculate motor current settings from physical parameters
  if (motor_spec_.sense_resistor_mohm > 0 && motor_spec_.supply_voltage_mv > 0) {
    uint8_t calc_irun = 0;
    uint8_t calc_ihold = 0;
    uint16_t calc_scaler = 0;
    
    uint16_t run_current = motor_spec_.run_current_ma;
    if (run_current == 0) {
      run_current = motor_spec_.rated_current_ma;
    }
    
    if (CalculateMotorCurrent(motor_spec_, motor_spec_.sense_resistor_mohm, 
                              motor_spec_.supply_voltage_mv,
                              run_current, motor_spec_.hold_current_ma,
                              calc_irun, calc_ihold, calc_scaler)) {
      // Apply percentage adjustments before constraining
      // GLOBAL_SCALER adjustment
      if (motor_spec_.scaler_adjustment_percent != 0.0f) {
        float adjustment_factor = 1.0f + (motor_spec_.scaler_adjustment_percent / 100.0f);
        calc_scaler = static_cast<uint16_t>(std::round(static_cast<float>(calc_scaler) * adjustment_factor));
      }
      
      // IRUN adjustment
      if (motor_spec_.irun_adjustment_percent != 0.0f) {
        float adjustment_factor = 1.0f + (motor_spec_.irun_adjustment_percent / 100.0f);
        calc_irun = static_cast<uint8_t>(std::round(static_cast<float>(calc_irun) * adjustment_factor));
      }
      
      // IHOLD adjustment
      if (motor_spec_.ihold_adjustment_percent != 0.0f) {
        float adjustment_factor = 1.0f + (motor_spec_.ihold_adjustment_percent / 100.0f);
        calc_ihold = static_cast<uint8_t>(std::round(static_cast<float>(calc_ihold) * adjustment_factor));
      }
      
      // Constrain to valid ranges after adjustments
      calc_scaler = constrain<uint16_t>(calc_scaler, 32U, 256U);
      calc_irun = constrain<uint8_t>(calc_irun, 0U, 31U);
      calc_ihold = constrain<uint8_t>(calc_ihold, 0U, 31U);
      
      // Ensure IHOLD < IRUN
      if (calc_ihold >= calc_irun) {
        calc_ihold = (calc_irun > 0) ? (calc_irun - 1) : 0;
      }
      
      // Store calculated values internally (not in motor_spec_)
      calculated_irun_ = calc_irun;
      calculated_ihold_ = calc_ihold;
      calculated_global_scaler_ = calc_scaler;
      
      TMC5160_LOG_DEBUG(comm_, 1, "TMC5160", "Calculated: IRUN=%u, IHOLD=%u, GLOBAL_SCALER=%u (adjustments: scaler=%.1f%%, irun=%.1f%%, ihold=%.1f%%)",
                        calc_irun, calc_ihold, calc_scaler,
                        motor_spec_.scaler_adjustment_percent,
                        motor_spec_.irun_adjustment_percent,
                        motor_spec_.ihold_adjustment_percent);
      
      // Validate StealthChop lower limit if resistance is available
      if (motor_spec_.winding_resistance_mohm > 0) {
        uint16_t lower_limit = CalculateStealthChopLowerLimit(motor_spec_,
                                                               motor_spec_.supply_voltage_mv,
                                                               config.chopper.tbl,
                                                               config.stealthchop.pwm_freq,
                                                               f_clk_);
        if (lower_limit > 0) {
          float run_current_a = static_cast<float>(run_current) / 1000.0f;
          float lower_limit_a = static_cast<float>(lower_limit) / 1000.0f;
          if (run_current_a < lower_limit_a * 1.1f) {
            TMC5160_LOG_DEBUG(comm_, 0, "TMC5160", 
                              "WARNING: Run current (%.1fmA) may be below StealthChop lower limit (%.1fmA)",
                              run_current, lower_limit);
          }
        }
      }
    } else {
      TMC5160_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to calculate motor current settings");
      return false;
    }
  } else {
    TMC5160_LOG_DEBUG(comm_, 0, "TMC5160", 
                      "Cannot calculate: sense_resistor_mohm=%u, supply_voltage_mv=%u",
                      motor_spec_.sense_resistor_mohm, motor_spec_.supply_voltage_mv);
    return false;
  }

  // Clear reset and error flags
  GSTAT_Register gstat{};
  gstat.bits.reset = true;
  gstat.bits.drv_err = true;
  gstat.bits.uv_cp = true;
  if (!this->comm_.WriteRegister(Registers::GSTAT, gstat.value, this->GetCommAddress())) {
    return false;
  }

  // Configure power stage (calculate register values from user-friendly parameters)
  DRV_CONF_Register drv_conf{};
  
  // Calculate DRVSTRENGTH from MOSFET Miller charge
  float miller = config.power_stage.mosfet_miller_charge_nc;
  if (miller < 0.0f) {
    miller = 0.0f; // Constrain to valid range
  }
  if (miller == 0.0f) {
    // Auto-calculate from default (small MOSFET, lowest driver strength)
    miller = 10.0f;
  }
  
  uint8_t drv_strength = 0;
  // Auto-calculate from Miller charge (datasheet table 3.3)
  // Use lowest gate driver strength giving favorable switching slopes
  if (miller < 10.0f) {
    drv_strength = 0;
  } else if (miller < 20.0f) {
    drv_strength = 0; // Can use 0 or 1, prefer 0 (lowest) for best switching quality
  } else if (miller < 40.0f) {
    drv_strength = 1; // Can use 1 or 2, prefer 1 (lower)
  } else if (miller < 60.0f) {
    drv_strength = 2; // Can use 2 or 3, prefer 2 (lower)
  } else {
    drv_strength = 3; // Maximum for very large MOSFETs
  }
  drv_conf.bits.drvstrength = constrain<uint8_t>(drv_strength, 0U, 3U);
  
  // Calculate BBMTIME and BBMCLKS from nanoseconds
  uint8_t bbm_time_reg = 0;
  uint8_t bbm_clks_reg = 0;
  
  // Auto-calculate from bbm_time_ns
  uint32_t bbm_ns = config.power_stage.bbm_time_ns;
  
  if (bbm_ns == 0) {
    // Auto-calculate from MOSFET: use minimum safe time (100ns for small MOSFETs, lowest setting)
    // For larger MOSFETs with higher Miller charge, may need more time
    if (miller > 40.0f) {
      bbm_ns = 200; // Larger MOSFETs need more BBM time
    } else {
      bbm_ns = 100; // Default minimum (lowest, BBMTIME=0)
    }
  }
  
  // Ensure minimum 100ns (lowest possible setting)
  if (bbm_ns < 100) {
    bbm_ns = 100;
  }
  
  // BBMTIME calculation per datasheet:
  // Formula: time[ns] ≈ 100ns * 32 / (32 - BBMTIME)
  // Rearranged: BBMTIME = 32 - (100ns * 32 / time_ns)
  // BBMTIME range: 0-24 (bits 4..0)
  // BBMTIME=0 gives 100ns (shortest, reset default)
  // BBMTIME=16 gives 200ns
  // BBMTIME=24 gives ~375ns (longest, datasheet says 375ns, formula gives ~400ns)
  // >24 not recommended, use BBMCLKS instead
  // 
  // Datasheet hint: Choose the lowest setting safely covering the switching event.
  // Add roughly 30% of reserve to cover production stray.
  if (bbm_ns <= 200) {
    // Use BBMTIME (0-24, corresponds to 100ns to ~375ns)
    // Add 30% headroom as recommended by datasheet
    float bbm_ns_with_headroom = static_cast<float>(bbm_ns) * 1.3f;
    // Calculate BBMTIME: BBMTIME = 32 - (100 * 32 / time_ns)
    float bbm_time_float = 32.0f - (100.0f * 32.0f / bbm_ns_with_headroom);
    bbm_time_reg = static_cast<uint8_t>(std::round(bbm_time_float));
    bbm_time_reg = constrain<uint8_t>(bbm_time_reg, 0U, 24U);
    bbm_clks_reg = 0; // BBMTIME takes precedence, BBMCLKS=0 (off)
  } else {
    // Use BBMCLKS for longer times (>200ns)
    // BBMCLKS range: 0-15 (bits 11..8), typ. 83ns per clock cycle at 12MHz
    // The longer setting rules (BBMTIME vs. BBMCLKS)
    // Add 30% headroom as recommended by datasheet
    float bbm_ns_with_headroom = static_cast<float>(bbm_ns) * 1.3f;
    // Calculate clock cycles: cycles = (time_ns * f_clk) / 1e9
    // At 12MHz: 1 cycle = 83.3ns
    uint32_t clock_cycles = static_cast<uint32_t>((bbm_ns_with_headroom * static_cast<float>(f_clk_)) / 1000000000.0f);
    bbm_clks_reg = constrain<uint8_t>(static_cast<uint8_t>(clock_cycles), 0U, 15U);
    if (bbm_clks_reg == 0 && bbm_ns > 200) {
      bbm_clks_reg = 1; // Minimum 1 clock cycle if >200ns requested
    }
    bbm_time_reg = 0; // BBMCLKS takes precedence when set (longer setting rules)
  }
  
  drv_conf.bits.bbmtime = bbm_time_reg;
  drv_conf.bits.bbmclks = bbm_clks_reg;
  drv_conf.bits.otselect = static_cast<uint8_t>(config.power_stage.over_temp_protection);
  drv_conf.bits.filt_isense = static_cast<uint8_t>(config.power_stage.sense_filter);
  
  TMC5160_LOG_DEBUG(comm_, 2, "TMC5160", "Power stage: DRVSTRENGTH=%u (from %.1fnC), BBMTIME=%u, BBMCLKS=%u, FILT_ISENSE=%u",
                    drv_conf.bits.drvstrength, miller,
                    drv_conf.bits.bbmtime, drv_conf.bits.bbmclks, drv_conf.bits.filt_isense);
  
  if (!this->comm_.WriteRegister(Registers::DRV_CONF, drv_conf.value, this->GetCommAddress())) {
    return false;
  }

  // Configure global scaler (use calculated value)
  TMC5160_LOG_DEBUG(comm_, 3, "TMC5160", "Initialize: Setting GLOBAL_SCALER=%u", calculated_global_scaler_);
  if (!this->comm_.WriteRegister(Registers::GLOBAL_SCALER, calculated_global_scaler_, this->GetCommAddress())) {
    return false;
  }

  // Configure motor current (use calculated values)
  IHOLD_IRUN_Register iholdrun{};
  iholdrun.bits.ihold = calculated_ihold_;
  iholdrun.bits.irun = calculated_irun_;
  iholdrun.bits.iholddelay = 7;
  TMC5160_LOG_DEBUG(comm_, 3, "TMC5160", "Initialize: Setting IHOLD_IRUN(irun=%u, ihold=%u, iholddelay=7)",
                     iholdrun.bits.irun, iholdrun.bits.ihold);
  if (!this->comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, this->GetCommAddress())) {
    return false;
  }

  // Configure short protection (convert user-friendly values to register values)
  SHORT_CONF_Register short_conf{};
  
  // Calculate S2VS_LEVEL from voltage threshold
  uint16_t s2vs_voltage = config.power_stage.s2vs_voltage_mv;
  if (s2vs_voltage == 0) {
    s2vs_voltage = 625; // Default recommended: 625mV (S2VS_LEVEL=6)
  }
  uint8_t s2vs_level = CalculateS2VSLevel(s2vs_voltage);
  if (s2vs_level == 0 || s2vs_level < 4) {
    s2vs_level = 6; // Fallback to default
  }
  short_conf.bits.s2vs_level = constrain<uint8_t>(s2vs_level, 4U, 15U);
  
  // Calculate S2G_LEVEL from voltage threshold (consider supply voltage for VS>52V check)
  uint16_t s2g_voltage = config.power_stage.s2g_voltage_mv;
  if (s2g_voltage == 0) {
    s2g_voltage = 625; // Default recommended: 625mV (S2G_LEVEL=6)
  }
  uint8_t s2g_level = CalculateS2GLevel(s2g_voltage, motor_spec_.supply_voltage_mv);
  if (s2g_level == 0 || s2g_level < 2) {
    s2g_level = 6; // Fallback to default
  }
  short_conf.bits.s2g_level = constrain<uint8_t>(s2g_level, 2U, 15U);
  
  short_conf.bits.shortfilter =
      constrain<decltype(config.power_stage.shortfilter)>(config.power_stage.shortfilter, 0U, 3U);
  
  // Calculate shortdelay from detection delay time
  uint8_t delay_us_x10 = config.power_stage.short_detection_delay_us_x10;
  if (delay_us_x10 == 0) {
    delay_us_x10 = 8; // Default recommended: 0.8µs (typical 0.85µs, shortdelay=0)
  }
  uint8_t shortdelay = CalculateShortDelay(delay_us_x10);
  short_conf.bits.shortdelay = constrain<uint8_t>(shortdelay, 0U, 1U);
  
  if (!this->comm_.WriteRegister(Registers::SHORT_CONF, short_conf.value, this->GetCommAddress())) {
    return false;
  }

  // Configure chopper
  CHOPCONF_Register chopconf{};
  chopconf.bits.toff = constrain<decltype(config.chopper.toff)>(config.chopper.toff, 0U, 15U);
  chopconf.bits.hstrt_tfd = constrain<decltype(config.chopper.hstrt)>(config.chopper.hstrt, 0U, 7U);
  chopconf.bits.hend_offset = constrain<decltype(config.chopper.hend)>(config.chopper.hend, 0U, 15U);
  chopconf.bits.tbl = constrain<decltype(config.chopper.tbl)>(config.chopper.tbl, 0U, 3U);
  // Note: Bit 17 (vsense) is reserved per datasheet, ignoring config.chopper.vsense
  chopconf.bits.mres = constrain<decltype(config.chopper.mres)>(config.chopper.mres, 0U, 8U);
  chopconf.bits.intpol = config.chopper.intpol ? 1 : 0;
  chopconf.bits.dedge = config.chopper.dedge ? 1 : 0;
  chopconf.bits.chm = config.chopper.chm ? 1 : 0;
  if (!this->comm_.WriteRegister(Registers::CHOPCONF, chopconf.value, this->GetCommAddress())) {
    return false;
  }
  
  // Update stored microsteps
  current_microsteps_ = 256U >> chopconf.bits.mres;

  // Configure stealthChop
  PWMCONF_Register pwmconf{};
  pwmconf.bits.pwm_ofs = config.stealthchop.pwm_ofs;
  pwmconf.bits.pwm_grad = config.stealthchop.pwm_grad;
  pwmconf.bits.pwm_freq = constrain<decltype(config.stealthchop.pwm_freq)>(config.stealthchop.pwm_freq, 0U, 3U);
  pwmconf.bits.pwm_autoscale = config.stealthchop.pwm_autoscale ? 1 : 0;
  pwmconf.bits.pwm_autograd = config.stealthchop.pwm_autograd ? 1 : 0;
  pwmconf.bits.pwm_reg = constrain<decltype(config.stealthchop.pwm_reg)>(config.stealthchop.pwm_reg, 0U, 15U);
  pwmconf.bits.pwm_lim = constrain<decltype(config.stealthchop.pwm_lim)>(config.stealthchop.pwm_lim, 0U, 15U);
  pwmconf.bits.freewheel = static_cast<uint8_t>(config.stealthchop.freewheel);
  if (!this->comm_.WriteRegister(Registers::PWMCONF, pwmconf.value, this->GetCommAddress())) {
    return false;
  }

  // Set ramp mode to positioning
  TMC5160_LOG_DEBUG(comm_, 3, "TMC5160", "Initialize: Setting ramp mode to POSITIONING");
  if (!rampControl.SetRampMode(RampMode::POSITIONING)) {
    return false;
  }

  // Configure global settings (GCONF)
  GCONF_Register gconf{};
  gconf.bits.recalibrate = config.global_config.recalibrate ? 1 : 0;
  gconf.bits.faststandstill = config.global_config.faststandstill ? 1 : 0;
  gconf.bits.en_pwm_mode = config.global_config.en_pwm_mode ? 1 : 0;
  gconf.bits.multistep_filt = config.global_config.multistep_filt ? 1 : 0;
  // Use direction from config (global_config.shaft can override via ConfigureGlobalConfig if
  // needed)
  gconf.bits.shaft = (config.direction == MotorDirection::INVERSE) ? 1 : 0;
  gconf.bits.diag0_error = config.global_config.diag0_error ? 1 : 0;
  gconf.bits.diag0_otpw = config.global_config.diag0_otpw ? 1 : 0;
  gconf.bits.diag0_stall_step = config.global_config.diag0_stall_step ? 1 : 0;
  gconf.bits.diag1_stall_dir = config.global_config.diag1_stall_dir ? 1 : 0;
  gconf.bits.diag1_index = config.global_config.diag1_index ? 1 : 0;
  gconf.bits.diag1_onstate = config.global_config.diag1_onstate ? 1 : 0;
  gconf.bits.diag1_steps_skipped = config.global_config.diag1_steps_skipped ? 1 : 0;
  gconf.bits.diag0_int_pushpull = config.global_config.diag0_int_pushpull ? 1 : 0;
  gconf.bits.diag1_poscomp_pushpull = config.global_config.diag1_poscomp_pushpull ? 1 : 0;
  gconf.bits.small_hysteresis = config.global_config.small_hysteresis ? 1 : 0;
  gconf.bits.stop_enable = config.global_config.stop_enable ? 1 : 0;
  gconf.bits.direct_mode = config.global_config.direct_mode ? 1 : 0;
  gconf.bits.test_mode = config.global_config.test_mode ? 1 : 0;
  if (!this->comm_.WriteRegister(Registers::GCONF, gconf.value, this->GetCommAddress())) {
    return false;
  }

  // Set ramp parameters
  if (!rampControl.SetPowerDownDelay(config.ramp_params.tpowerdown)) {
    return false;
  }
  if (!rampControl.SetZeroWaitTime(config.ramp_params.tzerowait)) {
    return false;
  }
  if (config.ramp_params.a1 > 0.0F) {
    if (!rampControl.SetFirstAcceleration(config.ramp_params.a1)) {
      return false;
    }
  }

  // Set default ramp speeds
  if (!rampControl.SetRampSpeeds(0.0F, 0.1F, 0.0F, Unit::Steps)) {
    return false;
  }

  // Set default D1 (must not be 0 in positioning mode)
  if (!this->comm_.WriteRegister(Registers::D_1, 100, this->GetCommAddress())) {
    return false;
  }

  initialized_ = true;
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Reset() noexcept {
  GSTAT_Register gstat{};
  gstat.bits.reset = true;
  return this->comm_.WriteRegister(Registers::GSTAT, gstat.value, this->GetCommAddress());
}

template <typename CommType>
int32_t TMC5160<CommType>::speedToInternal(float speed_hz) const noexcept {
  // Datasheet formula: v[Hz] = v[5160] * (f_CLK[Hz]/2 / 2^23)
  // Rearranged: v[5160] = v[Hz] * 2^24 / f_CLK
  // Where v[Hz] is in μsteps/s (microsteps per second)
  // Input speed_hz is in steps/s, so we multiply by microstep count (256) to convert to μsteps/s
  // Final: v[5160] = (speed_hz * 256) * 2^24 / f_CLK
  if (speed_hz == 0.0F) {
    return 0;
  }
  float internal = (speed_hz * static_cast<float>(1UL << 24)) / static_cast<float>(f_clk_);
  internal *= static_cast<float>(Microsteps::USTEP_COUNT);
  return static_cast<int32_t>(internal);
}

template <typename CommType>
float TMC5160<CommType>::speedFromInternal(int32_t speed_internal) const noexcept {
  // Datasheet formula: v[Hz] = v[5160] * (f_CLK[Hz]/2 / 2^23)
  // Where v[Hz] is in μsteps/s (microsteps per second)
  // Output is in steps/s, so we divide by microstep count (256) to convert from μsteps/s
  // Final: v[steps/s] = (v[5160] * f_CLK / 2^24) / 256
  if (speed_internal == 0) {
    return 0.0F;
  }
  float speed_hz = static_cast<float>(speed_internal) * static_cast<float>(f_clk_) / static_cast<float>(1UL << 24);
  speed_hz /= static_cast<float>(Microsteps::USTEP_COUNT);
  return speed_hz;
}

template <typename CommType>
int32_t TMC5160<CommType>::accelToInternal(float accel_hz) const noexcept {
  // Datasheet formula: a[Hz/s] = a[5160] * f_CLK[Hz]^2 / (512*256) / 2^24
  // Rearranged: a[5160] = a[Hz/s] * (512*256) * 2^24 / f_CLK^2
  // Where a[Hz/s] is in μsteps/s² (microsteps per second squared)
  // Input accel_hz is in steps/s², so we multiply by microstep count (256) to convert to μsteps/s²
  // Final: a[5160] = (accel_hz * 256) * (512*256) * 2^24 / f_CLK^2
  if (accel_hz == 0.0F) {
    return 0;
  }
  float internal = accel_hz * 512.0F * 256.0F * static_cast<float>(1UL << 24) /
                   (static_cast<float>(f_clk_) * static_cast<float>(f_clk_));
  internal *= static_cast<float>(Microsteps::USTEP_COUNT);
  return static_cast<int32_t>(internal);
}

template <typename CommType>
int32_t TMC5160<CommType>::thresholdSpeedToTstep(float speed_hz) const noexcept {
  // Datasheet formula: TSTEP = f_CLK / f256STEP = f_CLK / (fSTEP*256/USC)
  // Where fSTEP is in μsteps/s, USC is microstep count (normally 256)
  // For USC=256: TSTEP = f_CLK / fSTEP
  // Input speed_hz is in steps/s, so fSTEP = speed_hz * 256 (convert to μsteps/s)
  // Final: TSTEP = f_CLK / (speed_hz * 256)
  // TSTEP is 20-bit unsigned (max value 0xFFFFF = 1048575)
  if (speed_hz == 0.0F) {
    return 0;
  }
  float tstep = static_cast<float>(f_clk_) / (speed_hz * 256.0F);
  tstep = std::max(0.0F, std::min(1048575.0F, tstep));
  return static_cast<int32_t>(tstep);
}

// RampControl implementation
template <typename CommType>
bool TMC5160<CommType>::RampControl::SetRampMode(RampMode mode) noexcept {
  auto mode_value = static_cast<uint8_t>(mode);
  const char* mode_name = (mode == RampMode::POSITIONING) ? "POSITIONING" :
                          (mode == RampMode::VELOCITY_POS) ? "VELOCITY_POS" :
                          (mode == RampMode::VELOCITY_NEG) ? "VELOCITY_NEG" : "HOLD";
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetRampMode(%s)", mode_name);
  return driver_.comm_.WriteRegister(Registers::RAMPMODE, mode_value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetTargetPosition(float value, Unit unit) noexcept {
  float steps = driver_.convertPositionToSteps(value, unit);
  return SetTargetPosition(static_cast<int32_t>(steps)); // Calls private helper
}

// Private helper implementation
template <typename CommType>
bool TMC5160<CommType>::RampControl::SetTargetPosition(int32_t position) noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetTargetPosition(%d)", position);
  return driver_.comm_.WriteRegister(Registers::XTARGET, static_cast<uint32_t>(position));
}

template <typename CommType>
float TMC5160<CommType>::RampControl::GetCurrentPosition(Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XACTUAL, value, driver_.GetCommAddress())) {
    return 0.0f;
  }
  // Sign extend from 32-bit signed
  int32_t steps = static_cast<int32_t>(value);
  return driver_.convertStepsToUnit(steps, unit);
}

template <typename CommType>
float TMC5160<CommType>::RampControl::GetTargetPosition(Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XTARGET, value, driver_.GetCommAddress())) {
    return 0.0f;
  }
  return driver_.convertStepsToUnit(static_cast<int32_t>(value), unit);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetCurrentPosition(float value, Unit unit, bool update_encoder) noexcept {
  float steps = driver_.convertPositionToSteps(value, unit);
  return SetCurrentPosition(static_cast<int32_t>(steps), update_encoder); // Calls private helper
}

// Private helper implementation
template <typename CommType>
bool TMC5160<CommType>::RampControl::SetCurrentPosition(int32_t position, bool update_encoder) noexcept {
  if (!driver_.comm_.WriteRegister(Registers::XACTUAL, static_cast<uint32_t>(position))) {
    return false;
  }
  if (update_encoder) {
    if (!driver_.comm_.WriteRegister(Registers::X_ENC, static_cast<uint32_t>(position))) {
      return false;
    }
    // Clear deviation flag
    ENC_STATUS_Register enc_status{};
    enc_status.bits.deviation_warn = true;
    driver_.comm_.WriteRegister(Registers::ENC_STATUS, enc_status.value, driver_.GetCommAddress());
  }
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetMaxSpeed(float value, Unit unit) noexcept {
  float steps_per_sec = driver_.convertSpeedToSteps(value, unit);
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetMaxSpeed(%.2f steps/s)", steps_per_sec);
  
  int32_t internal = driver_.speedToInternal(std::abs(steps_per_sec));
  internal = std::min(internal, static_cast<decltype(internal)>(0x7FFFFF)); // VMAX is 23 bits
  if (!driver_.comm_.WriteRegister(Registers::VMAX, static_cast<uint32_t>(internal))) {
    return false;
  }
  // If in velocity mode, update direction
  uint32_t rampmode = 0;
  if (driver_.comm_.ReadRegister(Registers::RAMPMODE, rampmode)) {
    if (rampmode == static_cast<uint8_t>(RampMode::VELOCITY_POS) ||
        rampmode == static_cast<uint8_t>(RampMode::VELOCITY_NEG)) {
      uint8_t new_mode =
          (steps_per_sec < 0.0F) ? static_cast<uint8_t>(RampMode::VELOCITY_NEG) : static_cast<uint8_t>(RampMode::VELOCITY_POS);
      driver_.comm_.WriteRegister(Registers::RAMPMODE, new_mode, driver_.GetCommAddress());
    }
  }
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetAcceleration(float value, Unit unit) noexcept {
  return SetAccelerations(value, value, unit);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetAccelerations(float accel_val, float decel_val, Unit unit) noexcept {
  float accel_steps = driver_.convertAccelerationToSteps(accel_val, unit);
  float decel_steps = driver_.convertAccelerationToSteps(decel_val, unit);

  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetAccelerations(accel=%.2f, decel=%.2f steps/s²)",
                     accel_steps, decel_steps);
                     
  int32_t accel_internal = driver_.accelToInternal(std::abs(accel_steps));
  int32_t decel_internal = driver_.accelToInternal(std::abs(decel_steps));
  accel_internal = std::min(accel_internal,
      static_cast<decltype(accel_internal)>(0xFFFF)); // AMAX/DMAX are 16 bits
  decel_internal = std::min(decel_internal, static_cast<decltype(decel_internal)>(0xFFFF));
  bool success = true;
  success &= driver_.comm_.WriteRegister(Registers::AMAX, static_cast<uint32_t>(accel_internal));
  success &= driver_.comm_.WriteRegister(Registers::DMAX, static_cast<uint32_t>(decel_internal));
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetDeceleration(float value, Unit unit) noexcept {
  float decel_steps = driver_.convertAccelerationToSteps(value, unit);
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetDeceleration(decel=%.2f steps/s²)", decel_steps);
  int32_t decel_internal = driver_.accelToInternal(std::abs(decel_steps));
  decel_internal = std::min(decel_internal, static_cast<decltype(decel_internal)>(0xFFFF)); // DMAX is 16 bits
  return driver_.comm_.WriteRegister(Registers::DMAX, static_cast<uint32_t>(decel_internal));
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetRampSpeeds(float start_speed, float stop_speed,
                                                   float transition_speed, Unit unit) noexcept {
  float start_steps = driver_.convertSpeedToSteps(start_speed, unit);
  float stop_steps = driver_.convertSpeedToSteps(stop_speed, unit);
  float transition_steps = driver_.convertSpeedToSteps(transition_speed, unit);
  
  int32_t vstart = driver_.speedToInternal(std::abs(start_steps));
  int32_t vstop = driver_.speedToInternal(std::abs(stop_steps));
  int32_t v1 = driver_.speedToInternal(std::abs(transition_steps));
  vstart = std::min(vstart, static_cast<decltype(vstart)>(0x3FFFF)); // VSTART is 18 bits
  vstop = std::min(vstop,
                   static_cast<decltype(vstop)>(0x3FFFF)); // VSTOP is 18 bits
  v1 = std::min(v1, static_cast<decltype(v1)>(0xFFFFF));   // V1 is 20 bits
  bool success = true;
  success &= driver_.comm_.WriteRegister(Registers::VSTART, static_cast<uint32_t>(vstart));
  success &= driver_.comm_.WriteRegister(Registers::VSTOP, static_cast<uint32_t>(vstop));
  success &= driver_.comm_.WriteRegister(Registers::V_1, static_cast<uint32_t>(v1));
  return success;
}

template <typename CommType>
float TMC5160<CommType>::RampControl::GetCurrentSpeed(Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::VACTUAL, value)) {
    return 0.0F;
  }
  // VACTUAL is 24-bit signed
  auto signed_value = static_cast<int32_t>(value);
  if (signed_value & 0x800000) {
    signed_value |= static_cast<int32_t>(0xFF000000U); // Sign extend
  }
  float steps_per_sec = driver_.speedFromInternal(signed_value);
  return driver_.convertSpeedToUnit(steps_per_sec, unit);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::IsTargetReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.position_reached != 0;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::IsTargetVelocityReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.velocity_reached != 0;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::Stop() noexcept {
  bool success = true;
  success &= driver_.comm_.WriteRegister(Registers::VSTART, 0, driver_.GetCommAddress());
  success &= driver_.comm_.WriteRegister(Registers::VMAX, 0, driver_.GetCommAddress());
  return success;
}


template <typename CommType>
bool TMC5160<CommType>::RampControl::ConfigureReferenceSwitch(const ReferenceSwitchConfig& config) noexcept {
  // Read-Modify-Write to preserve other SW_MODE bits (like sg_stop)
  uint32_t sw_mode_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_val)) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = sw_mode_val;

  sw_mode.bits.stop_l_enable = config.stop_left_enable ? 1 : 0;
  sw_mode.bits.stop_r_enable = config.stop_right_enable ? 1 : 0;
  sw_mode.bits.pol_stop_l = config.pol_stop_left ? 1 : 0;
  sw_mode.bits.pol_stop_r = config.pol_stop_right ? 1 : 0;
  sw_mode.bits.swap_lr = config.swap_left_right ? 1 : 0;
  sw_mode.bits.latch_l_active = config.latch_left_active ? 1 : 0;
  sw_mode.bits.latch_l_inactive = config.latch_left_inactive ? 1 : 0;
  sw_mode.bits.latch_r_active = config.latch_right_active ? 1 : 0;
  sw_mode.bits.latch_r_inactive = config.latch_right_inactive ? 1 : 0;
  sw_mode.bits.en_latch_encoder = config.en_latch_encoder ? 1 : 0;
  sw_mode.bits.en_softstop = config.en_softstop ? 1 : 0;
  
  return driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value, driver_.GetCommAddress());
}

template <typename CommType>
float TMC5160<CommType>::RampControl::GetLatchedPosition(Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XLATCH, value)) {
    return 0.0f;
  }
  int32_t steps = static_cast<int32_t>(value);
  return driver_.convertStepsToUnit(steps, unit);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetComparePosition(float value, Unit unit) noexcept {
  float steps = driver_.convertPositionToSteps(value, unit);
  return driver_.comm_.WriteRegister(Registers::X_COMPARE, static_cast<uint32_t>(static_cast<int32_t>(steps)));
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetPowerDownDelay(uint8_t tpowerdown) noexcept {
  return driver_.comm_.WriteRegister(Registers::TPOWERDOWN, static_cast<uint32_t>(tpowerdown));
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetZeroWaitTime(uint16_t tzerowait) noexcept {
  return driver_.comm_.WriteRegister(Registers::TZEROWAIT, static_cast<uint32_t>(tzerowait));
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetFirstAcceleration(float a1, Unit unit) noexcept {
  float a1_steps = driver_.convertAccelerationToSteps(a1, unit);
  if (a1_steps == 0.0F) {
    // Set to 0 to use AMAX for this phase
    return driver_.comm_.WriteRegister(Registers::A_1, 0, driver_.GetCommAddress());
  }
  int32_t a1_internal = driver_.accelToInternal(std::abs(a1_steps));
  a1_internal = std::min(a1_internal,
      static_cast<decltype(a1_internal)>(0xFFFF)); // A_1 is 16 bits
  return driver_.comm_.WriteRegister(Registers::A_1, static_cast<uint32_t>(a1_internal));
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetFinalDeceleration(float d1, Unit unit) noexcept {
  float d1_steps = driver_.convertAccelerationToSteps(d1, unit);
  if (d1_steps == 0.0F) {
    // Datasheet warning: "Attention: Do not set 0 in positioning mode, even if V1=0!"
    // We allow setting 0 here as it might be used in other modes or user intends it,
    // but we log a warning if logging is enabled? 
    // For now, we just pass it through, but user should be aware.
  }
  int32_t d1_internal = driver_.accelToInternal(std::abs(d1_steps));
  d1_internal = std::min(d1_internal,
      static_cast<decltype(d1_internal)>(0xFFFF)); // D_1 is 16 bits
  // Ensure value is at least 1 if user tries to set very low non-zero value that rounds to 0?
  // The register range starts at 1.
  if (d1_internal == 0 && d1_steps != 0.0F) {
      d1_internal = 1;
  }
  return driver_.comm_.WriteRegister(Registers::D_1, static_cast<uint32_t>(d1_internal));
}

// MotorControl implementation
template <typename CommType>
bool TMC5160<CommType>::MotorControl::Enable() noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::Enable()");
  
  // Enable via EN pin GPIO (EN is active LOW to enable, so set to ACTIVE/LOW to enable power stage)
  // This must be done first to enable the power stage
  driver_.comm_.GpioSet(TMC5160CtrlPin::EN, GpioSignal::ACTIVE);
  
  // Enable via CHOPCONF register (set toff > 0)
  uint32_t chopconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::CHOPCONF, chopconf_value)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = chopconf_value;
  if (chopconf.bits.toff == 0) {
    // Restore saved toff value (default to 5 if not set)
    chopconf.bits.toff = 5;
  }
  return driver_.comm_.WriteRegister(Registers::CHOPCONF, chopconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::Disable() noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::Disable()");
  
  // Disable via CHOPCONF register (set toff = 0)
  uint32_t chopconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::CHOPCONF, chopconf_value)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = chopconf_value;
  chopconf.bits.toff = 0; // Disable driver
  
  bool success = driver_.comm_.WriteRegister(Registers::CHOPCONF, chopconf.value, driver_.GetCommAddress());
  
  // Disable via EN pin GPIO (EN is active LOW to enable, so set to INACTIVE/HIGH to disable power stage)
  driver_.comm_.GpioSet(TMC5160CtrlPin::EN, GpioSignal::INACTIVE);
  
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetCurrent(uint8_t irun, uint8_t ihold) noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::SetCurrent(irun=%u, ihold=%u)", irun, ihold);
  // Read-Modify-Write to preserve iholddelay
  uint32_t ihold_irun_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::IHOLD_IRUN, ihold_irun_val)) {
    // Fallback to write-only if read fails (or first write), with default delay
    IHOLD_IRUN_Register iholdrun{};
    iholdrun.bits.irun = constrain<decltype(irun)>(irun, 0U, 31U);
    iholdrun.bits.ihold = constrain<decltype(ihold)>(ihold, 0U, 31U);
    iholdrun.bits.iholddelay = 7;
    return driver_.comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, driver_.GetCommAddress());
  }

  IHOLD_IRUN_Register iholdrun{};
  iholdrun.value = ihold_irun_val;
  iholdrun.bits.irun = constrain<decltype(irun)>(irun, 0U, 31U);
  iholdrun.bits.ihold = constrain<decltype(ihold)>(ihold, 0U, 31U);
  // iholddelay preserved from read value
  
  return driver_.comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureChopper(const ChopperConfig& config) noexcept {
  // Read-Modify-Write to preserve fields not in ChopperConfig (like diss2g, diss2vs)
  uint32_t chopconf_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::CHOPCONF, chopconf_val)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = chopconf_val;

  chopconf.bits.toff = constrain<decltype(config.toff)>(config.toff, 0U, 15U);
  chopconf.bits.hstrt_tfd = constrain<decltype(config.hstrt)>(config.hstrt, 0U, 7U);
  chopconf.bits.hend_offset = constrain<decltype(config.hend)>(config.hend, 0U, 15U);
  chopconf.bits.tbl = constrain<decltype(config.tbl)>(config.tbl, 0U, 3U);
  // Note: Bit 17 (vsense) is reserved per datasheet, ignoring config.vsense
  chopconf.bits.mres = constrain<decltype(config.mres)>(config.mres, 0U, 8U);
  chopconf.bits.intpol = config.intpol ? 1 : 0;
  chopconf.bits.dedge = config.dedge ? 1 : 0;
  chopconf.bits.chm = config.chm ? 1 : 0;
  return driver_.comm_.WriteRegister(Registers::CHOPCONF, chopconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureStealthChop(const StealthChopConfig& config) noexcept {
  PWMCONF_Register pwmconf{};
  pwmconf.bits.pwm_ofs = config.pwm_ofs;
  pwmconf.bits.pwm_grad = config.pwm_grad;
  pwmconf.bits.pwm_freq = constrain<decltype(config.pwm_freq)>(config.pwm_freq, 0U, 3U);
  pwmconf.bits.pwm_autoscale = config.pwm_autoscale ? 1 : 0;
  pwmconf.bits.pwm_autograd = config.pwm_autograd ? 1 : 0;
  pwmconf.bits.pwm_reg = constrain<decltype(config.pwm_reg)>(config.pwm_reg, 0U, 15U);
  pwmconf.bits.pwm_lim = constrain<decltype(config.pwm_lim)>(config.pwm_lim, 0U, 15U);
  pwmconf.bits.freewheel = static_cast<uint8_t>(config.freewheel);
  return driver_.comm_.WriteRegister(Registers::PWMCONF, pwmconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetModeChangeSpeeds(float pwm_thrs, float cool_thrs, float high_thrs) noexcept {
  int32_t tpwmthrs = driver_.thresholdSpeedToTstep(pwm_thrs);
  int32_t tcoolthrs = driver_.thresholdSpeedToTstep(cool_thrs);
  int32_t thigh = driver_.thresholdSpeedToTstep(high_thrs);
  tpwmthrs = std::min(tpwmthrs, static_cast<decltype(tpwmthrs)>(0xFFFFF)); // 20 bits
  tcoolthrs = std::min(tcoolthrs, static_cast<decltype(tcoolthrs)>(0xFFFFF));
  thigh = std::min(thigh, static_cast<decltype(thigh)>(0xFFFFF));
  bool success = true;
  success &= driver_.comm_.WriteRegister(Registers::TPWMTHRS, static_cast<uint32_t>(tpwmthrs));
  success &= driver_.comm_.WriteRegister(Registers::TCOOLTHRS, static_cast<uint32_t>(tcoolthrs));
  success &= driver_.comm_.WriteRegister(Registers::THIGH, static_cast<uint32_t>(thigh));
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetCoolStepThreshold(float value, Unit unit) noexcept {
  float steps_per_sec = driver_.convertSpeedToSteps(value, unit);
  int32_t tcoolthrs = driver_.thresholdSpeedToTstep(steps_per_sec);
  tcoolthrs = std::min(tcoolthrs, static_cast<decltype(tcoolthrs)>(0xFFFFF));
  return driver_.comm_.WriteRegister(Registers::TCOOLTHRS, static_cast<uint32_t>(tcoolthrs));
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetHighSpeedThreshold(float value, Unit unit) noexcept {
  float steps_per_sec = driver_.convertSpeedToSteps(value, unit);
  int32_t thigh = driver_.thresholdSpeedToTstep(steps_per_sec);
  thigh = std::min(thigh, static_cast<decltype(thigh)>(0xFFFFF));
  return driver_.comm_.WriteRegister(Registers::THIGH, static_cast<uint32_t>(thigh));
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetGlobalScaler(uint16_t scaler) noexcept {
  scaler = constrain<decltype(scaler)>(scaler, 32U, 256U);
  return driver_.comm_.WriteRegister(Registers::GLOBAL_SCALER, scaler, driver_.GetCommAddress());
}


template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureCoolStep(const CoolStepConfig& config) noexcept {
  uint32_t coolconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::COOLCONF, coolconf_value)) {
    return false;
  }
  COOLCONF_Register coolconf{};
  coolconf.value = coolconf_value;
  
  // Update CoolStep fields
  coolconf.bits.semin = constrain<decltype(config.semin)>(config.semin, 0U, 15U);
  // Bit 4 is reserved, preserve or set to 0? Assuming preserve if reading, but explicit 0 is safe
  coolconf.bits.seup = constrain<decltype(config.seup)>(config.seup, 0U, 3U);
  coolconf.bits.semax = constrain<decltype(config.semax)>(config.semax, 0U, 15U);
  coolconf.bits.sedn = constrain<decltype(config.sedn)>(config.sedn, 0U, 3U);
  coolconf.bits.seimin = config.seimin ? 1 : 0;
  // Preserve SGT (bits 22..16) and SFILT (bit 24) unless explicitly managed
  // But wait, this method is specifically for CoolStep configuration.
  // It should PROBABLY generally respect existing SGT/SFILT if they were set by StallGuard config.
  // The previous implementation zeroed them out.
  
  coolconf.bits.sfilt = config.sfilt ? 1 : 0;
  
  return driver_.comm_.WriteRegister(Registers::COOLCONF, coolconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureDcStep(const DcStepConfig& config) noexcept {
  // Convert velocity threshold to internal format
  int32_t vdc_min = 0;
  if (config.vdc_min > 0.0F) {
    vdc_min = driver_.speedToInternal(config.vdc_min);
    // VDCMIN is 23-bit register (0...2^22), but only bits 22..8 are used for comparison
    // Lower 8 bits (7..0) are ignored/unused by the hardware comparator.
    // We mask the value to 0x7FFF00 to explicitly zero out the unused bits,
    // ensuring the register value reflects the effective threshold.
    // Note: No shift is required; the register compares bits 22..8 of VDCMIN
    // against bits 22..8 of VACTUAL.
    vdc_min = vdc_min & 0x7FFF00;
  }
  bool success = driver_.comm_.WriteRegister(Registers::VDCMIN, static_cast<uint32_t>(vdc_min));
  if (!success) {
    return false;
  }
  
  // Configure DCCTRL register if dc_time or dc_sg are set
  if (config.dc_time > 0 || config.dc_sg > 0) {
    DCCTRL_Register dcctrl{};
    dcctrl.bits.dc_time = constrain<decltype(config.dc_time)>(config.dc_time, 0U, 1023U);
    dcctrl.bits.dc_sg = constrain<decltype(config.dc_sg)>(config.dc_sg, 0U, 255U);
    success &= driver_.comm_.WriteRegister(Registers::DCCTRL, dcctrl.value, driver_.GetCommAddress());
  }
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureGlobalConfig(const GlobalConfig& config) noexcept {
  GCONF_Register gconf{};
  gconf.bits.recalibrate = config.recalibrate ? 1 : 0;
  gconf.bits.faststandstill = config.faststandstill ? 1 : 0;
  gconf.bits.en_pwm_mode = config.en_pwm_mode ? 1 : 0;
  gconf.bits.multistep_filt = config.multistep_filt ? 1 : 0;
  gconf.bits.shaft = config.shaft ? 1 : 0;
  gconf.bits.diag0_error = config.diag0_error ? 1 : 0;
  gconf.bits.diag0_otpw = config.diag0_otpw ? 1 : 0;
  gconf.bits.diag0_stall_step = config.diag0_stall_step ? 1 : 0;
  gconf.bits.diag1_stall_dir = config.diag1_stall_dir ? 1 : 0;
  gconf.bits.diag1_index = config.diag1_index ? 1 : 0;
  gconf.bits.diag1_onstate = config.diag1_onstate ? 1 : 0;
  gconf.bits.diag1_steps_skipped = config.diag1_steps_skipped ? 1 : 0;
  gconf.bits.diag0_int_pushpull = config.diag0_int_pushpull ? 1 : 0;
  gconf.bits.diag1_poscomp_pushpull = config.diag1_poscomp_pushpull ? 1 : 0;
  gconf.bits.small_hysteresis = config.small_hysteresis ? 1 : 0;
  gconf.bits.stop_enable = config.stop_enable ? 1 : 0;
  gconf.bits.direct_mode = config.direct_mode ? 1 : 0;
  gconf.bits.test_mode = config.test_mode ? 1 : 0;
  return driver_.comm_.WriteRegister(Registers::GCONF, gconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::GetGlobalConfig(GlobalConfig& config) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;
  
  config.recalibrate = gconf.bits.recalibrate != 0;
  config.faststandstill = gconf.bits.faststandstill != 0;
  config.en_pwm_mode = gconf.bits.en_pwm_mode != 0;
  config.multistep_filt = gconf.bits.multistep_filt != 0;
  config.shaft = gconf.bits.shaft != 0;
  config.diag0_error = gconf.bits.diag0_error != 0;
  config.diag0_otpw = gconf.bits.diag0_otpw != 0;
  config.diag0_stall_step = gconf.bits.diag0_stall_step != 0;
  config.diag1_stall_dir = gconf.bits.diag1_stall_dir != 0;
  config.diag1_index = gconf.bits.diag1_index != 0;
  config.diag1_onstate = gconf.bits.diag1_onstate != 0;
  config.diag1_steps_skipped = gconf.bits.diag1_steps_skipped != 0;
  config.diag0_int_pushpull = gconf.bits.diag0_int_pushpull != 0;
  config.diag1_poscomp_pushpull = gconf.bits.diag1_poscomp_pushpull != 0;
  config.small_hysteresis = gconf.bits.small_hysteresis != 0;
  config.stop_enable = gconf.bits.stop_enable != 0;
  config.direct_mode = gconf.bits.direct_mode != 0;
  config.test_mode = gconf.bits.test_mode != 0;
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetStealthChopEnabled(bool enabled) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;
  gconf.bits.en_pwm_mode = enabled ? 1 : 0;
  return driver_.comm_.WriteRegister(Registers::GCONF, gconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::GetChopperConfig(ChopperConfig& config) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::CHOPCONF, value)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = value;
  
  config.toff = static_cast<uint8_t>(chopconf.bits.toff);
  config.hstrt = static_cast<uint8_t>(chopconf.bits.hstrt_tfd);
  config.hend = static_cast<uint8_t>(chopconf.bits.hend_offset);
  config.tbl = static_cast<uint8_t>(chopconf.bits.tbl);
  config.mres = static_cast<uint8_t>(chopconf.bits.mres);
  config.intpol = chopconf.bits.intpol != 0;
  config.dedge = chopconf.bits.dedge != 0;
  config.chm = chopconf.bits.chm != 0;
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::GetDiag0Config(Diag0Config& config) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;
  
  config.error = gconf.bits.diag0_error != 0;
  config.otpw = gconf.bits.diag0_otpw != 0;
  config.stall_step = gconf.bits.diag0_stall_step != 0;
  config.pushpull = gconf.bits.diag0_int_pushpull != 0;
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetDiag0Config(const Diag0Config& config) noexcept {
  // Read-Modify-Write to preserve other GCONF bits
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;
  
  // Update DIAG0 bits (5, 6, 7, 12)
  gconf.bits.diag0_error = config.error ? 1 : 0;
  gconf.bits.diag0_otpw = config.otpw ? 1 : 0;
  gconf.bits.diag0_stall_step = config.stall_step ? 1 : 0;
  gconf.bits.diag0_int_pushpull = config.pushpull ? 1 : 0;
  
  return driver_.comm_.WriteRegister(Registers::GCONF, gconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::GetDiag1Config(Diag1Config& config) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;
  
  config.stall_dir = gconf.bits.diag1_stall_dir != 0;
  config.index = gconf.bits.diag1_index != 0;
  config.onstate = gconf.bits.diag1_onstate != 0;
  config.steps_skipped = gconf.bits.diag1_steps_skipped != 0;
  config.pushpull = gconf.bits.diag1_poscomp_pushpull != 0;
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetDiag1Config(const Diag1Config& config) noexcept {
  // Read-Modify-Write to preserve other GCONF bits
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;
  
  // Update DIAG1 bits (8, 9, 10, 11, 13)
  gconf.bits.diag1_stall_dir = config.stall_dir ? 1 : 0;
  gconf.bits.diag1_index = config.index ? 1 : 0;
  gconf.bits.diag1_onstate = config.onstate ? 1 : 0;
  gconf.bits.diag1_steps_skipped = config.steps_skipped ? 1 : 0;
  gconf.bits.diag1_poscomp_pushpull = config.pushpull ? 1 : 0;
  
  return driver_.comm_.WriteRegister(Registers::GCONF, gconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetMicrostepLookupTable(uint8_t index, uint32_t value) noexcept {
  if (index > 7) {
    return false;
  }
  const uint8_t registers[] = {Registers::MSLUT_0, Registers::MSLUT_1, Registers::MSLUT_2, Registers::MSLUT_3,
                               Registers::MSLUT_4, Registers::MSLUT_5, Registers::MSLUT_6, Registers::MSLUT_7};
  return driver_.comm_.WriteRegister(registers[index], value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetMicrostepLookupTableSegmentation(uint8_t width_sel_0, uint8_t width_sel_1,
                                                                          uint8_t width_sel_2, uint8_t width_sel_3,
                                                                          uint8_t lut_seg_start1,
                                                                          uint8_t lut_seg_start2,
    uint8_t lut_seg_start3) noexcept {
  MSLUTSEL_Register mslutsel{};
  mslutsel.bits.w0 = width_sel_0 & 0x3U; // Bits 1..0
  mslutsel.bits.w1 = width_sel_1 & 0x3U; // Bits 3..2
  mslutsel.bits.w2 = width_sel_2 & 0x3U; // Bits 5..4
  mslutsel.bits.w3 = width_sel_3 & 0x3U; // Bits 7..6
  mslutsel.bits.x1 = lut_seg_start1;     // Bits 15..8
  mslutsel.bits.x2 = lut_seg_start2;     // Bits 23..16
  mslutsel.bits.x3 = lut_seg_start3;     // Bits 31..24
  return driver_.comm_.WriteRegister(Registers::MSLUTSEL, mslutsel.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetMicrostepLookupTableStart(uint16_t start_current) noexcept {
  start_current = constrain<decltype(start_current)>(start_current, 0U, 255U);
  return driver_.comm_.WriteRegister(Registers::MSLUTSTART, start_current, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetupMotorFromSpec(const MotorSpec& motor_spec,
                                                         const MechanicalSystem* mechanical_system) noexcept {
  // Calculate global scaler based on rated current
  // Typical calculation: global_scaler = (rated_current_ma * 32) / (irun * sense_resistor_current)
  // For simplicity, we'll use a basic calculation
  uint16_t global_scaler = 32;
  if (motor_spec.rated_current_ma > 0) {
    // Basic calculation: assume 1.5A max current, scale accordingly
    global_scaler = static_cast<uint16_t>(std::min(256U, std::max(32U, (motor_spec.rated_current_ma * 32U) / 1500U)));
  }

  // Set global scaler
  if (!SetGlobalScaler(global_scaler)) {
    return false;
  }

  // Calculate irun and ihold from rated current
  // irun should be between 16-31 for best performance
  // We'll use 80% of rated current for irun, 30% for ihold
  uint8_t irun = 16;
  uint8_t ihold = 0;
  if (motor_spec.rated_current_ma > 0 && global_scaler > 0) {
    // Calculate irun: target current = (irun/32) * (global_scaler/32) * sense_resistor_current
    // Simplified: irun = (target_current * 32) / (global_scaler * sense_resistor_current / 32)
    // Assuming sense resistor gives ~1.5A at irun=31, global_scaler=32
    float target_run_current = static_cast<float>(motor_spec.rated_current_ma) * 0.8F;
    auto irun_calc =
        static_cast<uint32_t>((target_run_current * 32.0F) / (static_cast<float>(global_scaler) * 0.046875F));
    irun = static_cast<uint8_t>(std::min(static_cast<uint32_t>(31U), std::max(static_cast<uint32_t>(16U), irun_calc)));
    float target_hold_current = static_cast<float>(motor_spec.rated_current_ma) * 0.3F;
    auto ihold_calc =
        static_cast<uint32_t>((target_hold_current * 32.0F) / (static_cast<float>(global_scaler) * 0.046875F));
    ihold = static_cast<uint8_t>(std::min(static_cast<uint32_t>(31U), ihold_calc));
  }

  // Set motor current
  if (!SetCurrent(irun, ihold)) {
    return false;
  }

  // Configure chopper based on inductance if available
  if (motor_spec.winding_inductance_uh > 0) {
    ChopperConfig chop_config{};
    // Higher inductance may need different settings
    // This is a simplified heuristic
    if (motor_spec.winding_inductance_uh > 3000) {
      chop_config.tbl = 3; // Longer blank time for high inductance
    }
    ConfigureChopper(chop_config);
  }

  // Note: mechanical_system is stored for unit conversions but not used here
  // as it's used by the unit conversion functions, not motor setup

  return true;
}

// Encoder implementation
template <typename CommType>
bool TMC5160<CommType>::Encoder::Configure(const EncoderConfig& config) noexcept {
  ENCMODE_Register encmode{};
  encmode.bits.pol_A = config.pol_a ? 1 : 0;
  encmode.bits.pol_B = config.pol_b ? 1 : 0;
  encmode.bits.pol_N = config.pol_n ? 1 : 0;
  encmode.bits.ignore_AB = config.ignore_ab ? 1 : 0;
  encmode.bits.clr_cont = config.clr_cont ? 1 : 0;
  encmode.bits.clr_once = config.clr_once ? 1 : 0;
  encmode.bits.sensitivity = constrain<decltype(config.sensitivity)>(config.sensitivity, 0U, 3U);
  encmode.bits.clr_enc_x = config.clr_enc_x ? 1 : 0;
  encmode.bits.latch_x_act = config.latch_x_act ? 1 : 0;
  encmode.bits.enc_sel_decimal = config.enc_sel_decimal ? 1 : 0;
  return driver_.comm_.WriteRegister(Registers::ENCMODE, encmode.value, driver_.GetCommAddress());
}

template <typename CommType>
int32_t TMC5160<CommType>::Encoder::GetPosition() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::X_ENC, value)) {
    return 0;
  }
  return static_cast<int32_t>(value);
}

template <typename CommType>
bool TMC5160<CommType>::Encoder::SetResolution(int32_t motor_steps, int32_t enc_resolution, bool inverted) noexcept {
  // Calculate factor: (motor_steps * microsteps) / enc_resolution
  float factor = static_cast<float>(motor_steps * Microsteps::USTEP_COUNT) / static_cast<float>(enc_resolution);

  // Check if binary prescaler gives exact match
  auto enc_const_binary = static_cast<int32_t>(factor * 65536.0F);
  if (enc_const_binary * enc_resolution == motor_steps * Microsteps::USTEP_COUNT * 65536) {
    // Use binary mode
    uint32_t encmode_value = 0;
    if (!driver_.comm_.ReadRegister(Registers::ENCMODE, encmode_value)) {
      return false;
    }
    ENCMODE_Register encmode{};
    encmode.value = encmode_value;
    encmode.bits.enc_sel_decimal = false;
    if (!driver_.comm_.WriteRegister(Registers::ENCMODE, encmode.value)) {
      return false;
    }
    if (inverted) {
      enc_const_binary = -enc_const_binary;
    }
    return driver_.comm_.WriteRegister(Registers::ENC_CONST, static_cast<uint32_t>(enc_const_binary));
  }
    // Use decimal mode
    uint32_t encmode_value = 0;
    if (!driver_.comm_.ReadRegister(Registers::ENCMODE, encmode_value)) {
      return false;
    }
    ENCMODE_Register encmode{};
    encmode.value = encmode_value;
    encmode.bits.enc_sel_decimal = true;
    if (!driver_.comm_.WriteRegister(Registers::ENCMODE, encmode.value)) {
      return false;
    }
    int integer_part = static_cast<int>(std::floor(factor));
  int decimal_part = static_cast<int>((factor - static_cast<float>(integer_part)) * 10000.0F);
    if (inverted) {
      integer_part = 65535 - integer_part;
      decimal_part = 10000 - decimal_part;
    }
  int32_t enc_const_decimal = (integer_part * 65536) + decimal_part;
    bool exact_match =
      ((static_cast<int32_t>(factor * 10000.0F) * enc_resolution) == (motor_steps * Microsteps::USTEP_COUNT * 10000));
  driver_.comm_.WriteRegister(Registers::ENC_CONST, static_cast<uint32_t>(enc_const_decimal));
    return exact_match;
}

template <typename CommType>
bool TMC5160<CommType>::Encoder::SetAllowedDeviation(int32_t steps) noexcept {
  int32_t deviation = steps * Microsteps::USTEP_COUNT;
  deviation = std::min(deviation, static_cast<int32_t>(0xFFFFF)); // 20 bits
  return driver_.comm_.WriteRegister(Registers::ENC_DEVIATION, static_cast<uint32_t>(deviation));
}

template <typename CommType>
bool TMC5160<CommType>::Encoder::IsDeviationDetected() noexcept {
  uint32_t enc_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::ENC_STATUS, enc_status_value)) {
    return false;
  }
  ENC_STATUS_Register enc_status{};
  enc_status.value = enc_status_value;
  return enc_status.bits.deviation_warn != 0;
}

template <typename CommType>
bool TMC5160<CommType>::Encoder::ClearDeviationFlag() noexcept {
  ENC_STATUS_Register enc_status{};
  enc_status.bits.deviation_warn = true;
  return driver_.comm_.WriteRegister(Registers::ENC_STATUS, enc_status.value, driver_.GetCommAddress());
}

template <typename CommType>
int32_t TMC5160<CommType>::Encoder::GetLatchedPosition() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::ENC_LATCH, value)) {
    return 0;
  }
  return static_cast<int32_t>(value);
}

// Diagnostics implementation
template <typename CommType>
DriverStatus TMC5160<CommType>::Diagnostics::GetStatus() noexcept {
  uint32_t gstat_value = 0;
  uint32_t drv_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GSTAT, gstat_value)) {
    return DriverStatus::OTHER_ERR;
  }
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value)) {
    return DriverStatus::OTHER_ERR;
  }

  GSTAT_Register gstat{};
  gstat.value = gstat_value;
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;

  if (gstat.bits.uv_cp) {
    return DriverStatus::CP_UV;
  }
  if (drv_status.bits.s2vsa) {
    return DriverStatus::S2VSA;
  }
  if (drv_status.bits.s2vsb) {
    return DriverStatus::S2VSB;
  }
  if (drv_status.bits.s2ga) {
    return DriverStatus::S2GA;
  }
  if (drv_status.bits.s2gb) {
    return DriverStatus::S2GB;
  }
  if (drv_status.bits.ot) {
    return DriverStatus::OT;
  }
  if (gstat.bits.drv_err) {
    return DriverStatus::OTHER_ERR;
  }
  if (drv_status.bits.otpw) {
    return DriverStatus::OTPW;
  }

  return DriverStatus::OK;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetGlobalStatus(bool& reset, bool& drv_err, bool& uv_cp) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GSTAT, value)) {
    return false;
  }
  GSTAT_Register gstat{};
  gstat.value = value;
  reset = gstat.bits.reset != 0;
  drv_err = gstat.bits.drv_err != 0;
  uv_cp = gstat.bits.uv_cp != 0;
  return true;
}

template <typename CommType>
uint16_t TMC5160<CommType>::Diagnostics::GetStallGuard() noexcept {
  uint32_t drv_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value)) {
    return 0;
  }
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;
  return static_cast<uint16_t>(drv_status.bits.sg_result);
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ConfigureStallGuard(const StallGuardConfig& config) noexcept {
  uint32_t coolconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::COOLCONF, coolconf_value)) {
    return false;
  }
  COOLCONF_Register coolconf{};
  coolconf.value = coolconf_value;
  
  // SGT is signed 7-bit (-64 to +63), constrain and mask to 7 bits (bits 22..16)
  auto sgt_signed = static_cast<int8_t>(constrain<int8_t>(config.sgt, -64, 63));
  coolconf.bits.sgt = static_cast<int32_t>(sgt_signed) & 0x7F;
  
  coolconf.bits.sfilt = config.sfilt ? 1 : 0;
  
  // Preserve CoolStep fields (semin, seup, semax, sedn, seimin)
  // They are already in coolconf.value from ReadRegister
  
  return driver_.comm_.WriteRegister(Registers::COOLCONF, coolconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::EnableStopOnStall(bool enable) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, value)) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = value;
  sw_mode.bits.sg_stop = enable ? 1 : 0;
  return driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ClearStallFlag() noexcept {
  RAMP_STAT_Register ramp_stat{};
  ramp_stat.bits.event_stop_sg = 1; // Write 1 to clear
  return driver_.comm_.WriteRegister(Registers::RAMP_STAT, ramp_stat.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::IsStallDetected() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMP_STAT, value)) {
    return false;
  }
  RAMP_STAT_Register ramp_stat{};
  ramp_stat.value = value;
  return ramp_stat.bits.event_stop_sg != 0;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetDriverStatusRegister(uint32_t& status) noexcept {
  return driver_.comm_.ReadRegister(Registers::DRV_STATUS, status, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetRampStatusRegister(uint32_t& status) noexcept {
  return driver_.comm_.ReadRegister(Registers::RAMP_STAT, status, driver_.GetCommAddress());
}

template <typename CommType>
uint32_t TMC5160<CommType>::Diagnostics::GetLostSteps() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::LOST_STEPS, value)) {
    return 0;
  }
  return value & 0xFFFFF; // LOST_STEPS is 20 bits
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::PerformSensorlessHoming(bool direction, int8_t stall_threshold, float search_speed,
                                                             int32_t& final_position, uint32_t timeout_ms) noexcept {
  // Configure StallGuard2 for homing
  StallGuardConfig sg_config{};
  sg_config.sgt = stall_threshold;
  sg_config.sfilt = true; // Enable filter for stable readings
  if (!ConfigureStallGuard(sg_config)) {
    return false;
  }

  // Enable StallGuard2 stop in SW_MODE
  uint32_t sw_mode_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_value)) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = sw_mode_value;
  sw_mode.bits.sg_stop = true;     // Enable stop on stall
  sw_mode.bits.en_softstop = false; // Use hard stop for precise homing (per datasheet 12.4)
  if (!driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value)) {
    return false;
  }

  // Clear any existing stall flags
  uint32_t ramp_stat_dummy = 0;
  driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat_dummy);
  // Write back 1 to event_stop_sg to clear it
  RAMP_STAT_Register clear_stat{};
  clear_stat.bits.event_stop_sg = 1;
  driver_.comm_.WriteRegister(Registers::RAMP_STAT, clear_stat.value);

  // Set velocity mode and start movement
  RampMode mode = direction ? RampMode::VELOCITY_POS : RampMode::VELOCITY_NEG;
  if (!driver_.rampControl.SetRampMode(mode)) {
    return false;
  }
  
  // CRITICAL: Set acceleration/deceleration BEFORE setting speeds
  // Velocity mode requires AMAX > 0 to actually accelerate from VSTART to VMAX
  // Use reasonable acceleration: reach VMAX in ~0.1 seconds
  float acceleration = std::max(search_speed * 10.0f, 50000.0f); // At least 50k steps/s²
  if (!driver_.rampControl.SetAcceleration(acceleration)) {
    return false;
  }
  if (!driver_.rampControl.SetDeceleration(acceleration)) {
    return false;
  }
  
  // Set VSTART > 0 to actually start motion in velocity mode
  // VSTART should be reasonable but less than VMAX - use 10% of search speed or minimum 1000 steps/s
  float vstart_speed = std::max(search_speed * 0.1f, 1000.0f);
  if (!driver_.rampControl.SetRampSpeeds(vstart_speed, 100.0f, 0.0f, Unit::Steps)) {
    return false;
  }
  if (!driver_.rampControl.SetMaxSpeed(search_speed)) {
    return false;
  }

  // Wait for stall event
  bool stalled = false;
  // Simple polling loop with crude timeout using rough cycle estimates
  // Assuming ~10us per register read interaction on SPI
  uint32_t loops = timeout_ms * 100; 
  
  for (uint32_t i = 0; i < loops; i++) {
    uint32_t ramp_stat = 0;
    if (driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
      RAMP_STAT_Register status{};
      status.value = ramp_stat;
      
      // Check for stall stop event or velocity zero (stop executed)
      if (status.bits.event_stop_sg || (status.bits.vzero && status.bits.status_sg)) {
        stalled = true;
        break;
      }
    }
    // Small delay to relieve bus
    // driver_.comm_.DelayUs(100); // Requires DelayUs in Comm interface, assuming it exists
  }

  // Stop motor (ensure VMAX=0)
  driver_.rampControl.Stop();

  // Disable StallGuard2 stop to prevent accidental stops later
  sw_mode.bits.sg_stop = false;
  driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value);

  // Read final position
  final_position = driver_.rampControl.GetCurrentPosition();

  // Clear the stall event flag again to be clean
  driver_.comm_.WriteRegister(Registers::RAMP_STAT, clear_stat.value);

  return stalled;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::PerformSwitchHoming(bool direction, float search_speed, float switch_speed,
                                                         int32_t& final_position, bool use_left_switch, uint32_t timeout_ms) noexcept {
  // 1. Activate position latching and motor stop upon switch event
  SW_MODE_Register sw_mode{};
  // Read current SW_MODE to preserve other settings (like softstop)
  uint32_t sw_mode_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_val)) {
    return false;
  }
  sw_mode.value = sw_mode_val;
  
  // Configure latching and stop enable based on switch choice
  if (use_left_switch) {
    sw_mode.bits.latch_l_active = true;
    sw_mode.bits.stop_l_enable = true;
  } else {
    sw_mode.bits.latch_r_active = true;
    sw_mode.bits.stop_r_enable = true;
  }
  // Use hard stop for precise homing (soft stop can overshoot) - datasheet recommends hard stop for StallGuard,
  // but for switches hard stop ensures we don't crash if switch is a hard limit.
  // However, datasheet 12.4 says "Or motor can be softly decelerated...".
  // Let's assume hard stop for safety during homing search.
  sw_mode.bits.en_softstop = false; 
  
  if (!driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value)) {
    return false;
  }

  // 2. Start motion ramp into direction of switch
  RampMode mode = direction ? RampMode::VELOCITY_POS : RampMode::VELOCITY_NEG;
  if (!driver_.rampControl.SetRampMode(mode)) {
    return false;
  }
  if (!driver_.rampControl.SetMaxSpeed(search_speed)) {
    return false;
  }

  // 3. Wait for switch hit (motor stops automatically)
  bool switch_hit = false;
  uint32_t loops = timeout_ms * 100;
  for (uint32_t i = 0; i < loops; i++) {
    uint32_t ramp_stat = 0;
    if (driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
      RAMP_STAT_Register status{};
      status.value = ramp_stat;
      
      // Check for specific stop event
      if ((use_left_switch && status.bits.event_stop_l) || 
          (!use_left_switch && status.bits.event_stop_r)) {
        switch_hit = true;
        break;
      }
      // Also check vzero if we missed the event flag but stopped
      if (status.bits.vzero && ((use_left_switch && status.bits.status_stop_l) || (!use_left_switch && status.bits.status_stop_r))) {
        switch_hit = true;
        break;
      }
    }
    // driver_.comm_.DelayUs(100);
  }

  // 4. Stop motor command (VMAX=0) to ensure it stays stopped
  driver_.rampControl.Stop();

  if (switch_hit) {
    // 5. Read latched position
    // "Latching of... XACTUAL to XLATCH upon a switch event gives a precise snapshot"
    // However, datasheet 12.4 "Implementing a homing procedure" step 5 says:
    // "Switch to hold mode... calculate difference between latched and actual... or when using hard stop XACTUAL stops exactly at home position"
    // Since we used hard stop, XACTUAL should be valid.
    // Let's read XLATCH just in case user wants it, but we return current XACTUAL as "final position".
    // Actually, let's just return the current position where it stopped.
    final_position = driver_.rampControl.GetCurrentPosition();
  }

  // Disable stop function to allow moving away
  if (use_left_switch) {
    sw_mode.bits.stop_l_enable = false;
  } else {
    sw_mode.bits.stop_r_enable = false;
  }
  driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value);

  return switch_hit;
}

// Communication implementation
template <typename CommType>
bool TMC5160<CommType>::Communication::ConfigureSlaveAddress(uint8_t slave_address, uint8_t send_delay) noexcept {
  SLAVECONF_Register slaveconf{};
  slaveconf.bits.slaveaddr = slave_address & 0x7F;
  slaveconf.bits.senddelay = constrain<uint8_t>(send_delay, 0U, 15U);

  if (!driver_.comm_.WriteRegister(Registers::SLAVECONF, slaveconf.value)) {
    return false;
  }

  // Store send delay locally (slave address is same as uart_node_address)
  driver_.send_delay_ = constrain<uint8_t>(send_delay, 0U, 15U);
  
  // Update UART node address (slave address and UART node address are the same)
  driver_.uart_node_address_ = slave_address & 0xFF;

  // Update UART interface slave address if using UART
  if (driver_.comm_.GetMode() == CommMode::UART) {
    // Cast to UART interface and update address
    // Note: This requires the interface to be UartCommInterface
    // The user should also call SetSlaveAddress on the interface directly
  }

  return true;
}


// Protection implementation
template <typename CommType>
bool TMC5160<CommType>::Protection::ConfigureShortProtection(const PowerStageParameters& config) noexcept {
  // Convert user-friendly values to register values
  uint8_t s2vs_level = CalculateS2VSLevel(config.s2vs_voltage_mv);
  if (s2vs_level == 0 || s2vs_level < 4) {
    s2vs_level = 6; // Fallback to default
  }
  
  // Get supply voltage from motor spec for S2G calculation
  uint32_t supply_voltage = 0;
  // Try to get from stored motor_spec_ if available, otherwise use 0 (will use VS<50V defaults)
  if (driver_.motor_spec_.supply_voltage_mv > 0) {
    supply_voltage = driver_.motor_spec_.supply_voltage_mv;
  }
  
  uint8_t s2g_level = CalculateS2GLevel(config.s2g_voltage_mv, supply_voltage);
  if (s2g_level == 0 || s2g_level < 2) {
    s2g_level = 6; // Fallback to default
  }
  
  uint8_t shortdelay = CalculateShortDelay(config.short_detection_delay_us_x10);
  
  return SetShortProtectionLevels(s2vs_level, s2g_level, config.shortfilter, shortdelay);
}

template <typename CommType>
bool TMC5160<CommType>::Protection::SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level, uint8_t shortfilter,
    uint8_t shortdelay) noexcept {
  SHORT_CONF_Register short_conf{};
  short_conf.bits.s2vs_level = constrain<decltype(s2vs_level)>(s2vs_level, 4U, 15U);
  short_conf.bits.s2g_level = constrain<decltype(s2g_level)>(s2g_level, 2U, 15U);
  short_conf.bits.shortfilter = constrain<uint8_t>(shortfilter, 0U, 3U);
  short_conf.bits.shortdelay = constrain<uint8_t>(shortdelay, 0U, 1U);
  return driver_.comm_.WriteRegister(Registers::SHORT_CONF, short_conf.value, driver_.GetCommAddress());
}

// Diagnostics read-only register implementations
template <typename CommType>
uint32_t TMC5160<CommType>::Diagnostics::GetTimeBetweenMicrosteps() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::TSTEP, value)) {
    return 0;
  }
  return value; // TSTEP is 20 bits, but we return full 32-bit value
}

template <typename CommType>
uint16_t TMC5160<CommType>::Diagnostics::GetMicrostepCounter() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::MSCNT, value)) {
    return 0;
  }
  return static_cast<uint16_t>(value & 0x3FF); // MSCNT is 10 bits
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetMicrostepCurrent(int16_t& phase_a, int16_t& phase_b) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::MSCURACT, value)) {
    return false;
  }
  MSCURACT_Register mscuract{};
  mscuract.value = value;
  
  // CUR_B is bits 8..0 (signed 9-bit)
  auto cur_b_raw = static_cast<int16_t>(mscuract.bits.cur_b);
  if (cur_b_raw & 0x0100) {                     // Check sign bit (bit 8)
    cur_b_raw |= static_cast<int16_t>(0xFE00U); // Sign extend to 16-bit
  }
  phase_b = cur_b_raw;
  
  // CUR_A is bits 24..16 (signed 9-bit)
  auto cur_a_raw = static_cast<int16_t>(mscuract.bits.cur_a);
  if (cur_a_raw & 0x0100) {                     // Check sign bit (bit 8)
    cur_a_raw |= static_cast<int16_t>(0xFE00U); // Sign extend to 16-bit
  }
  phase_a = cur_a_raw;
  
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetPwmScale(uint8_t& pwm_scale_sum, int16_t& pwm_scale_auto) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::PWM_SCALE, value)) {
    return false;
  }
  PWM_SCALE_Register pwm_scale{};
  pwm_scale.value = value;
  pwm_scale_sum = static_cast<uint8_t>(pwm_scale.bits.pwm_scale_sum);
  // PWM_SCALE_AUTO is signed 9-bit (bits 24..16)
  auto auto_raw = static_cast<int16_t>(pwm_scale.bits.pwm_scale_auto);
  if (auto_raw & 0x0100) {                     // Check sign bit (bit 8 of the 9-bit field)
    auto_raw |= static_cast<int16_t>(0xFE00U); // Sign extend to 16-bit
  }
  pwm_scale_auto = auto_raw;
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetPwmAuto(uint8_t& pwm_ofs_auto, uint8_t& pwm_grad_auto) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::PWM_AUTO, value)) {
    return false;
  }
  PWM_AUTO_Register pwm_auto{};
  pwm_auto.value = value;
  pwm_ofs_auto = static_cast<uint8_t>(pwm_auto.bits.pwm_ofs_auto);
  pwm_grad_auto = static_cast<uint8_t>(pwm_auto.bits.pwm_grad_auto);
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ReadInputStatus(InputStatus& input_status) noexcept {
  uint32_t io_pins = 0;
  if (!ReadGpioPins(io_pins)) {
    return false;
  }
  
  IOIN_Register ioin{};
  ioin.value = io_pins;
  
  input_status.refl_step = ioin.bits.refl_step != 0;
  input_status.refr_dir = ioin.bits.refr_dir != 0;
  input_status.encb_dcen_cfg4 = ioin.bits.encb_dcen_cfg4 != 0;
  input_status.enca_dcin_cfg5 = ioin.bits.enca_dcin_cfg5 != 0;
  input_status.drv_enn = ioin.bits.drv_enn != 0;
  input_status.enc_n_dco_cfg6 = ioin.bits.enc_n_dco_cfg6 != 0;
  input_status.sd_mode = ioin.bits.sd_mode != 0;
  input_status.swcomp_in = ioin.bits.swcomp_in != 0;
  input_status.version = static_cast<uint8_t>(ioin.bits.version);
  
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ReadIcVersion(uint8_t& version) noexcept {
  uint32_t io_pins = 0;
  if (!ReadGpioPins(io_pins)) {
    return false;
  }
  IOIN_Register ioin{};
  ioin.value = io_pins;
  version = static_cast<uint8_t>(ioin.bits.version);
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ReadGpioPins(uint32_t& io_pins) noexcept {
  return driver_.comm_.ReadRegister(Registers::IO_INPUT_OUTPUT, io_pins, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ReadFactoryConfig(uint8_t& fclktrim) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::FACTORY_CONF, value)) {
    return false;
  }
  // FACTORY_CONF contains FCLKTRIM in bits 0-4
  fclktrim = static_cast<uint8_t>(value & 0x1F);
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::SetSdoCfg0Polarity(bool polarity) noexcept {
  // Register 0x04 is dual purpose:
  // - Read: IOIN (Input states)
  // - Write: OUTPUT (Output configuration)
  //
  // Bit 0 of OUTPUT register controls the polarity of SDO_CFG0 pin in UART mode.
  // 0: Normal / Active High (Logic 1 = High Voltage)
  // 1: Inverted / Active Low (Logic 1 = Low Voltage) - Default reset value
  //
  // NOTE: Writing to 0x04 DOES NOT overwrite input flags (as they are read-only at this address).
  // Writing only affects the output configuration latch.
  
  uint32_t value = polarity ? 1 : 0;
  return driver_.comm_.WriteRegister(Registers::IO_INPUT_OUTPUT, value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ReadOtpConfig(uint8_t& otp_fclktrim, bool& otp_s2_level, bool& otp_bbm,
                                                   bool& otp_tbl) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::OTP_READ, value)) {
    return false;
  }
  OTP_READ_Register otp_read{};
  otp_read.value = value;
  otp_fclktrim = static_cast<uint8_t>(otp_read.bits.otp_fclktrim);
  otp_s2_level = otp_read.bits.otp_S2_level != 0;
  otp_bbm = otp_read.bits.otp_bbm != 0;
  otp_tbl = otp_read.bits.otp_tbl != 0;
  return true;
}

template <typename CommType>
uint8_t TMC5160<CommType>::Diagnostics::GetUartTransmissionCount() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::IFCNT, value)) {
    return 0;
  }
  return static_cast<uint8_t>(value & 0xFF); // IFCNT is 8 bits
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ReadOffsetCalibration(uint8_t& phase_a, uint8_t& phase_b) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::OFFSET_READ, value)) {
    return false;
  }
  OFFSET_READ_Register offset_read{};
  offset_read.value = value;
  phase_a = static_cast<uint8_t>(offset_read.bits.phase_a);
  phase_b = static_cast<uint8_t>(offset_read.bits.phase_b);
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::VerifySetup() noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "--- TMC5160 Setup Verification ---");

  // 1. Check IC Version
  uint8_t version = 0;
  if (ReadIcVersion(version)) {
    if (version == 0x30) {
      TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "IC Version: 0x30 (Matches TMC5160)");
    } else {
      TMC5160_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "IC Version MISMATCH: 0x%02X (Expected 0x30)", version);
      // If version is 0x00 or 0xFF, it's likely a communication error
      if (version == 0x00 || version == 0xFF) {
        TMC5160_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "CRITICAL: Bus communication likely failed!");
        return false;
      }
    }
  } else {
    TMC5160_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "Failed to read IC Version!");
    return false;
  }

  // 2. Check Input Pins (Reg 0x04)
  InputStatus inputs;
  if (ReadInputStatus(inputs)) {
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "--- Input Pins (IOIN 0x04) ---");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "REFL_STEP:      %s", inputs.refl_step ? "HIGH" : "LOW");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "REFR_DIR:       %s", inputs.refr_dir ? "HIGH" : "LOW");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "ENCB_DCEN_CFG4: %s", inputs.encb_dcen_cfg4 ? "HIGH" : "LOW");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "ENCA_DCIN_CFG5: %s", inputs.enca_dcin_cfg5 ? "HIGH" : "LOW");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "DRV_ENN:        %s (Active LOW)", inputs.drv_enn ? "HIGH (Disabled)" : "LOW (Enabled)");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "ENC_N_DCO_CFG6: %s", inputs.enc_n_dco_cfg6 ? "HIGH" : "LOW");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "SD_MODE:        %s %s", inputs.sd_mode ? "HIGH" : "LOW", inputs.sd_mode ? "(External Step/Dir)" : "(Internal Ramp)");
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "SWCOMP_IN:      %s", inputs.swcomp_in ? "HIGH" : "LOW");
    
    // Warn about configuration mismatches
    if (inputs.drv_enn) {
       TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "NOTE: DRV_ENN is HIGH. Driver power stage is currently DISABLED.");
    }
  } else {
    TMC5160_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "Failed to read Input Status!");
  }

  TMC5160_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "----------------------------------");
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::TuneStallGuard(float target_velocity, int8_t& final_sgt, int8_t min_sgt, int8_t max_sgt, 
                                                    float acceleration, float min_velocity, float max_velocity, 
                                                    Unit velocity_unit) noexcept {
  // Convert inputs to steps
  float target_v_steps = driver_.convertSpeedToSteps(target_velocity, velocity_unit);
  float min_v_steps = driver_.convertSpeedToSteps(min_velocity, velocity_unit);
  float max_v_steps = driver_.convertSpeedToSteps(max_velocity, velocity_unit);
  float accel_steps = driver_.convertAccelerationToSteps(acceleration, velocity_unit);

  TMC5160_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", "Starting SGT tuning. Target Vel=%.2f steps/s", target_v_steps);

  // 0. CRITICAL: Ensure stop-on-stall is DISABLED
  if (!EnableStopOnStall(false)) {
    return false;
  }

  // Clear any previous stop events/flags
  ClearStallFlag();
  driver_.comm_.DelayMs(10); // Allow flags to clear

  // 1. Start with SGT=0 (datasheet recommendation) or min_sgt if higher
  // Starting too low (e.g., -10) causes immediate false stalls
  int8_t current_sgt = (min_sgt < 0) ? 0 : min_sgt;
  if (current_sgt != min_sgt) {
    TMC5160_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", "Adjusted starting SGT from %d to %d (avoiding false stalls)", min_sgt, current_sgt);
  }
  
  // Enable velocity mode for continuous motion during tuning
  if (!driver_.rampControl.SetRampMode(RampMode::VELOCITY_POS)) {
    return false;
  }
  
  // Set explicit acceleration
  if (!driver_.rampControl.SetAccelerations(accel_steps, accel_steps, Unit::Steps)) {
    return false;
  }
  
  // Start motion
  if (!driver_.rampControl.SetMaxSpeed(target_v_steps, Unit::Steps)) {
    return false;
  }
  
  // Wait for motor to reach speed
  bool velocity_reached = false;
  for (int i = 0; i < 500; i++) { // 500 * 10ms = 5000ms timeout
    if (driver_.rampControl.IsTargetVelocityReached()) {
        velocity_reached = true;
        break;
    }
    // Also check speed manually
    float current_speed = driver_.rampControl.GetCurrentSpeed(Unit::Steps);
    if (std::abs(current_speed - target_v_steps) < 100.0f) {
        velocity_reached = true;
        break;
    }
    driver_.comm_.DelayMs(10);
  }
  
  if (!velocity_reached) {
      TMC5160_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", "Warning: Target velocity not reached before tuning loop (V=%.1f)", 
                        driver_.rampControl.GetCurrentSpeed(Unit::Steps));
  }

  // 2. Tuning loop
  bool tuned = false;
  while (current_sgt <= max_sgt) {
    // Update SGT
    StallGuardConfig sg_config{};
    sg_config.sgt = current_sgt;
    sg_config.sfilt = false; // Disable filter during tuning
    if (!ConfigureStallGuard(sg_config)) {
      driver_.rampControl.Stop();
      return false;
    }
    
    driver_.comm_.DelayMs(10);
    
    // Sample SG_RESULT multiple times
    bool stall_indicated = false;
    for (int i = 0; i < 4; i++) {
      uint16_t sg_val = GetStallGuard();
      if (sg_val == 0) {
        stall_indicated = true;
        break; 
      }
      driver_.comm_.DelayMs(1);
    }
    
    if (stall_indicated) {
      // Check if motor stopped unexpected
      float vact = driver_.rampControl.GetCurrentSpeed(Unit::Steps);
      if (std::abs(vact) < 10.0f) {
          TMC5160_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", "Motor stopped during tuning! (V=%.1f). Checking sg_stop...", vact);
          // Ensure sg_stop is disabled and restart
          EnableStopOnStall(false);
          ClearStallFlag();
          driver_.rampControl.SetMaxSpeed(target_v_steps, Unit::Steps);
          driver_.comm_.DelayMs(200);
      }
      
      TMC5160_LOG_DEBUG(driver_.comm_, 2, "TuneStallGuard", "SGT %d too low (SG=0), increasing...", current_sgt);
      current_sgt++;
    } else {
      // Stable non-zero reading found at target velocity
      TMC5160_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", "SGT %d stable at target. Verifying range...", current_sgt);
      
      // 3. Verify at Min/Max velocities if requested
      bool range_ok = true;
      
      // Check Min Velocity
      if (min_v_steps > 0.0f) {
        driver_.rampControl.SetMaxSpeed(min_v_steps, Unit::Steps);
        // Wait to reach speed
        driver_.comm_.DelayMs(500); // Rough delay for decel
        
        // Check for stalls at low speed
        for (int i = 0; i < 4; i++) {
           if (GetStallGuard() == 0) {
             TMC5160_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", "Stall detected at MIN velocity with SGT %d", current_sgt);
             range_ok = false;
             break;
           }
           driver_.comm_.DelayMs(5);
        }
      }
      
      // Check Max Velocity (only if min check passed)
      if (range_ok && max_v_steps > 0.0f) {
        driver_.rampControl.SetMaxSpeed(max_v_steps, Unit::Steps);
        // Wait to reach speed
        driver_.comm_.DelayMs(500); // Rough delay for accel
        
        // Check for stalls at high speed
        for (int i = 0; i < 4; i++) {
           if (GetStallGuard() == 0) {
             TMC5160_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", "Stall detected at MAX velocity with SGT %d", current_sgt);
             range_ok = false;
             break;
           }
           driver_.comm_.DelayMs(5);
        }
      }
      
      if (range_ok) {
      final_sgt = current_sgt;
      tuned = true;
      break;
      } else {
        // If range check failed, increase SGT (make less sensitive) and retry
        current_sgt++;
        // Restore target velocity for next iteration baseline
        driver_.rampControl.SetMaxSpeed(target_v_steps, Unit::Steps);
        driver_.comm_.DelayMs(200);
      }
    }
  }
  
  // Stop motor
  driver_.rampControl.Stop();
  
  if (!tuned) {
    TMC5160_LOG_DEBUG(driver_.comm_, 0, "TuneStallGuard", "Tuning failed. Reached max SGT without stable signal.");
    return false;
  }
  
  return true;
}

// UartConfig implementation
template <typename CommType>
bool TMC5160<CommType>::UartConfig::ConfigureSlave(uint8_t slave_address, uint8_t send_delay) noexcept {
  // During sequential programming, the device is accessible at address 0
  // (first device: NAI=GND, subsequent devices: previous NAO=LOW)
  // We need to use address 0 to communicate, not the target address
  uint8_t current_accessible_address = 0;

  SLAVECONF_Register slaveconf{};
  slaveconf.bits.slaveaddr = slave_address & 0xFF; // Address range is 0-254 (8-bit)
  slaveconf.bits.senddelay = constrain<decltype(send_delay)>(send_delay, 0U, 15U);

  // Write to SLAVECONF using current accessible address (0)
  bool success = driver_->comm_.WriteRegister(Registers::SLAVECONF, slaveconf.value, current_accessible_address);

  // Only update the driver's node address and send delay after successful programming
  if (success) {
    driver_->uart_node_address_ = slave_address & 0xFF; // Address range is 0-254 (slave address == UART node address)
    driver_->send_delay_ = constrain<decltype(send_delay)>(send_delay, 0U, 15U); // Store send delay locally
  }

  return success;
}

#endif // TMC5160_IMPL
