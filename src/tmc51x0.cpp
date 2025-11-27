/**
 * @file tmc51x0.cpp
 * @brief Implementation of TMC51x0 driver methods (TMC5130 & TMC5160)
 *
 * This file contains the template implementation of all TMC51x0 driver methods.
 * It is included by tmc51x0.hpp to provide header-only template instantiation.
 * Supports both TMC5130 and TMC5160 chips.
 */

#ifndef TMC51X0_IMPL
#define TMC51X0_IMPL

// When included from header, use relative path; when compiled directly, use
// standard include
#ifdef TMC51X0_HEADER_INCLUDED
// Already included from header - the class definition is available in the
// current context We're inside the namespace, so we can access the template
// class No need to include header or open namespace
#else
// Not included from header (shouldn't happen for template implementation)
#include "../inc/tmc51x0.hpp"
#endif

#include <algorithm>
#include <cmath>

#include "../inc/tmc51x0_motor_calc.hpp"
#include "../inc/tmc51x0_units.hpp"

using namespace tmc51x0;

// Implementation of operating mode control methods
template <typename CommType>
bool TMC51x0<CommType>::Communication::SetOperatingMode(ChipCommMode mode) noexcept {
  const char* mode_name = (mode == ChipCommMode::SPI_INTERNAL_RAMP)             ? "SPI_INTERNAL_RAMP"
                          : (mode == ChipCommMode::SPI_EXTERNAL_STEPDIR)        ? "SPI_EXTERNAL_STEPDIR"
                          : (mode == ChipCommMode::UART_INTERNAL_RAMP)          ? "UART_INTERNAL_RAMP"
                          : (mode == ChipCommMode::STANDALONE_EXTERNAL_STEPDIR) ? "STANDALONE_EXTERNAL_STEPDIR"
                                                                                : "UNKNOWN";
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "Communication::SetOperatingMode(%s)", mode_name);

  // Map mode to SPI_MODE and SD_MODE pin states
  GpioSignal spi_mode_signal, sd_mode_signal;

  switch (mode) {
  case ChipCommMode::SPI_INTERNAL_RAMP:
    spi_mode_signal = GpioSignal::ACTIVE;  // HIGH
    sd_mode_signal = GpioSignal::INACTIVE; // LOW
    break;
  case ChipCommMode::SPI_EXTERNAL_STEPDIR:
    spi_mode_signal = GpioSignal::ACTIVE; // HIGH
    sd_mode_signal = GpioSignal::ACTIVE;  // HIGH
    break;
  case ChipCommMode::UART_INTERNAL_RAMP:
    spi_mode_signal = GpioSignal::INACTIVE; // LOW
    sd_mode_signal = GpioSignal::INACTIVE;  // LOW
    break;
  case ChipCommMode::STANDALONE_EXTERNAL_STEPDIR:
    spi_mode_signal = GpioSignal::INACTIVE; // LOW
    sd_mode_signal = GpioSignal::ACTIVE;    // HIGH
    break;
  default:
    return false;
  }

  // Set SPI_MODE pin
  if (!driver_.comm_.GpioSet(TMC51x0CtrlPin::SPI_MODE, spi_mode_signal)) {
    return false;
  }

  // Set SD_MODE pin
  if (!driver_.comm_.GpioSet(TMC51x0CtrlPin::SD_MODE, sd_mode_signal)) {
    return false;
  }

  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Communication::GetOperatingMode(ChipCommMode& mode) const noexcept {
  GpioSignal spi_mode_signal, sd_mode_signal;

  // Read SPI_MODE pin
  if (!driver_.comm_.GpioRead(TMC51x0CtrlPin::SPI_MODE, spi_mode_signal)) {
    return false;
  }

  // Read SD_MODE pin
  if (!driver_.comm_.GpioRead(TMC51x0CtrlPin::SD_MODE, sd_mode_signal)) {
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
    // SPI_MODE = LOW
    if (sd_mode_signal == GpioSignal::ACTIVE) {
      mode = ChipCommMode::STANDALONE_EXTERNAL_STEPDIR;
    } else {
      mode = ChipCommMode::UART_INTERNAL_RAMP;
    }
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
float TMC51x0<CommType>::convertSpeedToSteps(float value, Unit unit) const noexcept {
  if (value == 0.0f)
    return 0.0f;

  // Calculate effective steps per revolution (full steps * microsteps * gear ratio)
  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) *
                                  static_cast<float>(current_microsteps_) * mechanical_system_.gear_ratio;

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
    if (mechanical_system_.system_type == MechanicalSystemType::LeadScrew &&
        mechanical_system_.lead_screw_pitch_mm > 0.0f) {
      // mm/s / pitch * steps_per_rev
      return (value / mechanical_system_.lead_screw_pitch_mm) * effective_steps_per_rev;
    } else if (mechanical_system_.system_type == MechanicalSystemType::BeltDrive &&
               mechanical_system_.belt_pitch_mm > 0.0f && mechanical_system_.belt_pulley_teeth > 0) {
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
float TMC51x0<CommType>::convertAccelerationToSteps(float value, Unit unit) const noexcept {
  // Acceleration conversions are same as velocity (per second squared)
  return convertSpeedToSteps(value, unit);
}

template <typename CommType>
float TMC51x0<CommType>::convertPositionToSteps(float value, Unit unit) const noexcept {
  // Position conversions logic is same as speed (time unit cancels out)
  // e.g. RPM (revs) -> Steps (revs * steps/rev)
  // rad -> Steps
  // mm -> Steps
  // Note: For RPM input as position, it implies "Revolutions".
  // Ideally we'd have a separate Unit enum for Position (Revs, Rads, Degs, Mm) vs Speed (RPM, Rad/s...)
  // But we are reusing Unit.
  // Unit::RPM for position -> treated as Revolutions

  if (value == 0.0f)
    return 0.0f;

  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) *
                                  static_cast<float>(current_microsteps_) * mechanical_system_.gear_ratio;

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
    if (mechanical_system_.system_type == MechanicalSystemType::LeadScrew &&
        mechanical_system_.lead_screw_pitch_mm > 0.0f) {
      return (value / mechanical_system_.lead_screw_pitch_mm) * effective_steps_per_rev;
    } else if (mechanical_system_.system_type == MechanicalSystemType::BeltDrive &&
               mechanical_system_.belt_pitch_mm > 0.0f && mechanical_system_.belt_pulley_teeth > 0) {
      float mm_per_rev = mechanical_system_.belt_pitch_mm * static_cast<float>(mechanical_system_.belt_pulley_teeth);
      return (value / mm_per_rev) * effective_steps_per_rev;
    }
    return 0.0f;
  default:
    return value;
  }
}

template <typename CommType>
float TMC51x0<CommType>::convertStepsToUnit(int32_t steps, Unit unit) const noexcept {
  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) *
                                  static_cast<float>(current_microsteps_) * mechanical_system_.gear_ratio;

  if (effective_steps_per_rev == 0.0f)
    return 0.0f;
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
float TMC51x0<CommType>::convertSpeedToUnit(float steps_per_sec, Unit unit) const noexcept {
  // Logic similar to steps->unit but scaling for time if needed
  // Steps/s -> RPM: (Steps/s / Steps/rev) * 60

  float effective_steps_per_rev = static_cast<float>(motor_spec_.steps_per_rev) *
                                  static_cast<float>(current_microsteps_) * mechanical_system_.gear_ratio;

  if (effective_steps_per_rev == 0.0f)
    return 0.0f;

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
bool TMC51x0<CommType>::Initialize(const DriverConfig& config) noexcept {
  TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize(toff=%u, mres=%u)", config.chopper.toff,
                    static_cast<uint8_t>(config.chopper.mres));

  // Store physical configuration
  motor_spec_ = config.motor_spec;
  mechanical_system_ = config.mechanical;

  // Configure clock source
  if (!communication.SetClkFreq(config.external_clk_config)) {
    return false;
  }

  // Calculate initial microsteps from config
  // mres: 0=256, 1=128, ... 8=fullstep. microsteps = 256 >> mres
  uint8_t mres = constrain<uint8_t>(static_cast<uint8_t>(config.chopper.mres), 0U, 8U);
  current_microsteps_ = 256U >> mres;

  // Configure motor current from motor specifications
  if (!motorControl.ConfigureMotorCurrent(motor_spec_)) {
    return false;
  }

  // Validate StealthChop lower limit if resistance is available
  if (motor_spec_.winding_resistance_mohm > 0) {
    uint16_t run_current = motor_spec_.run_current_ma;
    if (run_current == 0) {
      run_current = motor_spec_.rated_current_ma;
    }
    if (run_current > 0) {
      uint16_t lower_limit = CalculateStealthChopLowerLimit(motor_spec_, motor_spec_.supply_voltage_mv,
                                                            config.chopper.tbl, config.stealthchop.pwm_freq, f_clk_);
      if (lower_limit > 0) {
        float run_current_a = static_cast<float>(run_current) / 1000.0f;
        float lower_limit_a = static_cast<float>(lower_limit) / 1000.0f;
        if (run_current_a < lower_limit_a * 1.1f) {
          TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160",
                            "WARNING: Run current (%.1fmA) may be below StealthChop lower limit (%.1fmA)", run_current,
                            lower_limit);
        }
      }
    }
  }

  // Clear reset and error flags
  GSTAT_Register gstat{};
  gstat.bits.reset = true;
  gstat.bits.drv_err = true;
  gstat.bits.uv_cp = true;
  if (!this->comm_.WriteRegister(Registers::GSTAT, gstat.value, this->GetCommAddress())) {
    return false;
  }

  // Configure power stage
  if (!motorControl.ConfigurePowerStage(config.power_stage)) {
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
  if (!motorControl.ConfigureChopper(config.chopper)) {
    return false;
  }

  // Configure StealthChop
  if (!motorControl.ConfigureStealthChop(config.stealthchop)) {
    return false;
  }

  // Set ramp mode to positioning
  TMC51X0_LOG_DEBUG(comm_, 3, "TMC5160", "Initialize: Setting ramp mode to POSITIONING");
  if (!rampControl.SetRampMode(RampMode::POSITIONING)) {
    return false;
  }

  // Detect chip version (read from IOIN register)
  uint8_t chip_version = 0;
  if (diagnostics.ReadIcVersion(chip_version)) {
    chip_version_ = chip_version;
    if (chip_version == ChipVersion::TMC5130) {
      TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Detected TMC5130 chip (version 0x%02X)", chip_version);
    } else if (chip_version == ChipVersion::TMC5160) {
      TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Detected TMC5160 chip (version 0x%02X)", chip_version);
    } else {
      TMC51X0_LOG_DEBUG(comm_, 1, "TMC5160", "Unknown chip version: 0x%02X", chip_version);
    }
  } else {
    TMC51X0_LOG_DEBUG(comm_, 1, "TMC5160", "Failed to read chip version, assuming TMC5160");
    chip_version_ = ChipVersion::TMC5160; // Default to TMC5160
  }

  // Configure global settings (GCONF)
  // Read-Modify-Write to preserve reserved bits (bits 18-31)
  uint32_t gconf_value = 0;
  if (!this->comm_.ReadRegister(Registers::GCONF, gconf_value, this->GetCommAddress())) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = gconf_value;

  gconf.bits.recalibrate = config.global_config.recalibrate ? 1 : 0;
  gconf.bits.faststandstill = config.global_config.en_short_standstill_timeout ? 1 : 0;
  gconf.bits.en_pwm_mode = config.global_config.en_stealthchop_mode ? 1 : 0;
  gconf.bits.multistep_filt = config.global_config.en_stealthchop_step_filter ? 1 : 0;
  // Use direction from config (global_config.invert_direction can override via ConfigureGlobalConfig if
  // needed)
  gconf.bits.shaft = (config.direction == MotorDirection::INVERSE) ? 1 : 0;
  gconf.bits.diag0_error = config.global_config.diag0.error ? 1 : 0;
  gconf.bits.diag0_otpw = config.global_config.diag0.otpw ? 1 : 0;
  gconf.bits.diag0_stall_step = config.global_config.diag0.stall_step ? 1 : 0;
  gconf.bits.diag1_stall_dir = config.global_config.diag1.stall_dir ? 1 : 0;
  gconf.bits.diag1_index = config.global_config.diag1.index ? 1 : 0;
  gconf.bits.diag1_onstate = config.global_config.diag1.onstate ? 1 : 0;
  gconf.bits.diag1_steps_skipped = config.global_config.diag1.steps_skipped ? 1 : 0;
  gconf.bits.diag0_int_pushpull = config.global_config.diag0.pushpull ? 1 : 0;
  gconf.bits.diag1_poscomp_pushpull = config.global_config.diag1.pushpull ? 1 : 0;
  gconf.bits.small_hysteresis = config.global_config.en_small_step_frequency_hysteresis ? 1 : 0;
  gconf.bits.stop_enable = config.global_config.enca_dcin_sequencer_stop ? 1 : 0;
  gconf.bits.direct_mode = config.global_config.direct_mode ? 1 : 0;
  gconf.bits.test_mode = 0; // Always disabled (factory test mode, not for user)
  // Reserved bits (18-31) are preserved from read value

  if (!this->comm_.WriteRegister(Registers::GCONF, gconf.value, this->GetCommAddress())) {
    return false;
  }

  // Configure ramp generator
  if (!rampControl.ConfigureRamp(config.ramp_config)) {
    return false;
  }

  // Configure reference switches (defaults are safe/disabled)
  TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize: Configuring reference switches");
  if (!rampControl.ConfigureReferenceSwitch(config.reference_switch_config)) {
    TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to configure reference switches");
    return false;
  }

  // Configure encoder (defaults are safe/disabled)
  TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize: Configuring encoder");
  if (!encoder.Configure(config.encoder_config)) {
    TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to configure encoder");
    return false;
  }

  // Set encoder resolution if encoder pulses per rev is configured
  if (config.encoder_config.pulses_per_rev > 0) {
    // Use motor output steps (accounting for gearbox if present)
    // motor_spec.steps_per_rev is motor steps, mechanical.gear_ratio accounts for gearbox
    int32_t motor_output_steps =
        static_cast<int32_t>(static_cast<float>(motor_spec_.steps_per_rev) * mechanical_system_.gear_ratio);

    TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160",
                      "Initialize: Setting encoder resolution (motor_steps=%ld, enc_pulses=%u, invert=%s)",
                      motor_output_steps, config.encoder_config.pulses_per_rev,
                      config.encoder_config.invert_direction ? "true" : "false");
    if (!encoder.SetResolution(motor_output_steps, static_cast<int32_t>(config.encoder_config.pulses_per_rev),
                               config.encoder_config.invert_direction)) {
      TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to set encoder resolution");
      return false;
    }
  }

  // Configure UART node address if set (only used in UART mode)
  if (config.uart_config.node_address > 0 || config.uart_config.send_delay > 0) {
    TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize: Configuring UART node address (address=%u, send_delay=%u)",
                      config.uart_config.node_address, config.uart_config.send_delay);
    if (!communication.ConfigureUartNodeAddress(config.uart_config.node_address, config.uart_config.send_delay)) {
      TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to configure UART node address");
      return false;
    }
  }

  // Configure StallGuard2 (defaults are safe/disabled)
  TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize: Configuring StallGuard2");
  if (!diagnostics.ConfigureStallGuard(config.stallguard)) {
    TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to configure StallGuard2");
    return false;
  }

  // Configure CoolStep (defaults are safe/disabled)
  TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize: Configuring CoolStep");
  if (!motorControl.ConfigureCoolStep(config.coolstep)) {
    TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to configure CoolStep");
    return false;
  }

  // Configure DcStep (defaults are safe/disabled)
  TMC51X0_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize: Configuring DcStep");
  if (!motorControl.ConfigureDcStep(config.dcstep)) {
    TMC51X0_LOG_DEBUG(comm_, 0, "TMC5160", "Failed to configure DcStep");
    return false;
  }

  // Store configuration (will be updated on all runtime changes via Configure* methods)
  driver_config_ = config;

  initialized_ = true;
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Reset() noexcept {
  GSTAT_Register gstat{};
  gstat.bits.reset = true;
  return this->comm_.WriteRegister(Registers::GSTAT, gstat.value, this->GetCommAddress());
}

template <typename CommType>
int32_t TMC51x0<CommType>::speedToInternal(float speed_hz) const noexcept {
  // Datasheet formula: v[Hz] = v[5160] * (f_CLK[Hz]/2 / 2^23)
  // Rearranged: v[5160] = v[Hz] * 2^24 / f_CLK
  // Where v[Hz] is in μsteps/s (microsteps per second)
  // Input speed_hz is in steps/s, so we multiply by microstep count (USC) to convert to μsteps/s
  // Final: v[5160] = (speed_hz * USC) * 2^24 / f_CLK
  // Note: USC (microstep count) is tracked in current_microsteps_ and can vary (256, 128, 64, etc.)
  if (speed_hz == 0.0F) {
    return 0;
  }
  float internal = (speed_hz * static_cast<float>(1UL << 24)) / static_cast<float>(f_clk_);
  internal *= static_cast<float>(current_microsteps_);
  return static_cast<int32_t>(internal);
}

template <typename CommType>
float TMC51x0<CommType>::speedFromInternal(int32_t speed_internal) const noexcept {
  // Datasheet formula: v[Hz] = v[5160] * (f_CLK[Hz]/2 / 2^23)
  // Where v[Hz] is in μsteps/s (microsteps per second)
  // Output is in steps/s, so we divide by microstep count (USC) to convert from μsteps/s
  // Final: v[steps/s] = (v[5160] * f_CLK / 2^24) / USC
  // Note: USC (microstep count) is tracked in current_microsteps_ and can vary (256, 128, 64, etc.)
  if (speed_internal == 0) {
    return 0.0F;
  }
  float speed_hz = static_cast<float>(speed_internal) * static_cast<float>(f_clk_) / static_cast<float>(1UL << 24);
  speed_hz /= static_cast<float>(current_microsteps_);
  return speed_hz;
}

template <typename CommType>
int32_t TMC51x0<CommType>::accelToInternal(float accel_hz) const noexcept {
  // Datasheet formula: a[Hz/s] = a[5160] * f_CLK[Hz]^2 / (512*256) / 2^24
  // Rearranged: a[5160] = a[Hz/s] * (512*256) * 2^24 / f_CLK^2
  // Where a[Hz/s] is in μsteps/s² (microsteps per second squared)
  // Input accel_hz is in steps/s², so we multiply by microstep count (USC) to convert to μsteps/s²
  // Final: a[5160] = (accel_hz * USC) * (512*256) * 2^24 / f_CLK^2
  // Note: USC (microstep count) is tracked in current_microsteps_ and can vary (256, 128, 64, etc.)
  // Note: The 256 in (512*256) is a fixed constant from the datasheet formula, not the microstep count
  if (accel_hz == 0.0F) {
    return 0;
  }
  float internal = accel_hz * 512.0F * 256.0F * static_cast<float>(1UL << 24) /
                   (static_cast<float>(f_clk_) * static_cast<float>(f_clk_));
  internal *= static_cast<float>(current_microsteps_);
  return static_cast<int32_t>(internal);
}

template <typename CommType>
int32_t TMC51x0<CommType>::thresholdSpeedToTstep(float speed_hz) const noexcept {
  // Datasheet formula: TSTEP = f_CLK / f256STEP = f_CLK / (fSTEP*256/USC)
  // Where fSTEP is in μsteps/s, USC is microstep count (can vary: 256, 128, 64, etc.)
  // Input speed_hz is in steps/s, so fSTEP = speed_hz * USC (convert to μsteps/s)
  // Final: TSTEP = f_CLK / (speed_hz * USC)
  // Note: USC (microstep count) is tracked in current_microsteps_ and can vary
  // TSTEP is 20-bit unsigned (max value 0xFFFFF = 1048575)
  if (speed_hz == 0.0F) {
    return 0;
  }
  float tstep = static_cast<float>(f_clk_) / (speed_hz * static_cast<float>(current_microsteps_));
  tstep = std::max(0.0F, std::min(1048575.0F, tstep));
  return static_cast<int32_t>(tstep);
}

// RampControl implementation
template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetRampMode(RampMode mode) noexcept {
  auto mode_value = static_cast<uint8_t>(mode);
  const char* mode_name = (mode == RampMode::POSITIONING)    ? "POSITIONING"
                          : (mode == RampMode::VELOCITY_POS) ? "VELOCITY_POS"
                          : (mode == RampMode::VELOCITY_NEG) ? "VELOCITY_NEG"
                                                             : "HOLD";
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetRampMode(%s)", mode_name);

  // Read-Modify-Write to preserve reserved bits (bits 2-31)
  uint32_t rampmode_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMPMODE, rampmode_value, driver_.GetCommAddress())) {
    return false;
  }
  // Clear mode bits (0-1) and set new mode, preserve reserved bits (2-31)
  rampmode_value = (rampmode_value & 0xFFFFFFFCU) | (mode_value & 0x03U);
  return driver_.comm_.WriteRegister(Registers::RAMPMODE, rampmode_value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetRampMode(RampMode& mode) noexcept {
  uint32_t rampmode_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMPMODE, rampmode_value, driver_.GetCommAddress())) {
    return false;
  }
  uint8_t mode_bits = static_cast<uint8_t>(rampmode_value & 0x03U);
  mode = static_cast<RampMode>(mode_bits);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::IsPositionReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_.diagnostics.GetRampStatusRegister(ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.position_reached != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::IsVelocityReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_.diagnostics.GetRampStatusRegister(ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.velocity_reached != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::IsStandstill() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_.diagnostics.GetRampStatusRegister(ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.vzero != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetReferenceSwitchStatus(bool& left_active, bool& right_active, bool& left_enabled,
                                                              bool& right_enabled) noexcept {
  // Get switch active status from RAMP_STAT
  uint32_t ramp_stat = 0;
  if (!driver_.diagnostics.GetRampStatusRegister(ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  left_active = status.bits.status_stop_l != 0;
  right_active = status.bits.status_stop_r != 0;

  // Get switch enabled status from SW_MODE
  uint32_t sw_mode_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_value, driver_.GetCommAddress())) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = sw_mode_value;
  left_enabled = sw_mode.bits.stop_l_enable != 0;
  right_enabled = sw_mode.bits.stop_r_enable != 0;

  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetXCompare(float position, Unit unit) noexcept {
  float steps = driver_.convertPositionToSteps(position, unit);
  int32_t x_compare = static_cast<int32_t>(std::round(steps));

  bool success =
      driver_.comm_.WriteRegister(Registers::X_COMPARE, static_cast<uint32_t>(x_compare), driver_.GetCommAddress());
  if (success) {
    driver_.write_only_regs_.x_compare = static_cast<uint32_t>(x_compare);
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetXCompare(float& position, Unit unit) const noexcept {
  int32_t steps = static_cast<int32_t>(driver_.write_only_regs_.x_compare);
  position = driver_.convertStepsToUnit(steps, unit);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetTargetPosition(float value, Unit unit) noexcept {
  float steps = driver_.convertPositionToSteps(value, unit);
  return SetTargetPosition(static_cast<int32_t>(steps)); // Calls private helper
}

// Private helper implementation
template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetTargetPosition(int32_t position) noexcept {
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetTargetPosition(%d)", position);
  return driver_.comm_.WriteRegister(Registers::XTARGET, static_cast<uint32_t>(position));
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetCurrentPosition(float& position, Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XACTUAL, value, driver_.GetCommAddress())) {
    return false;
  }
  // Sign extend from 32-bit signed
  int32_t steps = static_cast<int32_t>(value);
  position = driver_.convertStepsToUnit(steps, unit);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetTargetPosition(float& position, Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XTARGET, value, driver_.GetCommAddress())) {
    return false;
  }
  position = driver_.convertStepsToUnit(static_cast<int32_t>(value), unit);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetCurrentPosition(float value, Unit unit, bool update_encoder) noexcept {
  float steps = driver_.convertPositionToSteps(value, unit);
  return SetCurrentPosition(static_cast<int32_t>(steps), update_encoder); // Calls private helper
}

// Private helper implementation
template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetCurrentPosition(int32_t position, bool update_encoder) noexcept {
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
bool TMC51x0<CommType>::RampControl::SetMaxSpeed(float value, Unit unit) noexcept {
  float steps_per_sec = driver_.convertSpeedToSteps(value, unit);
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetMaxSpeed(%.2f steps/s)", steps_per_sec);

  int32_t internal = driver_.speedToInternal(std::abs(steps_per_sec));
  internal = std::min(internal, static_cast<decltype(internal)>(0x7FFFFF)); // VMAX is 23 bits
  bool success = driver_.comm_.WriteRegister(Registers::VMAX, static_cast<uint32_t>(internal));
  if (success) {
    driver_.write_only_regs_.vmax = static_cast<uint32_t>(internal);
    // Update driver config if in steps unit
    if (unit == Unit::Steps) {
      driver_.driver_config_.ramp_config.vmax = value;
    }
  }
  if (!success) {
    return false;
  }
  // If in velocity mode, update direction
  uint32_t rampmode = 0;
  if (driver_.comm_.ReadRegister(Registers::RAMPMODE, rampmode, driver_.GetCommAddress())) {
    if ((rampmode & 0x03U) == static_cast<uint8_t>(RampMode::VELOCITY_POS) ||
        (rampmode & 0x03U) == static_cast<uint8_t>(RampMode::VELOCITY_NEG)) {
      uint8_t new_mode = (steps_per_sec < 0.0F) ? static_cast<uint8_t>(RampMode::VELOCITY_NEG)
                                                : static_cast<uint8_t>(RampMode::VELOCITY_POS);
      // Read-Modify-Write to preserve reserved bits (bits 2-31)
      rampmode = (rampmode & 0xFFFFFFFCU) | (new_mode & 0x03U);
      driver_.comm_.WriteRegister(Registers::RAMPMODE, rampmode, driver_.GetCommAddress());
    }
  }
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetAcceleration(float value, Unit unit) noexcept {
  return SetAccelerations(value, value, unit);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetAccelerations(float accel_val, float decel_val, Unit unit) noexcept {
  float accel_steps = driver_.convertAccelerationToSteps(accel_val, unit);
  float decel_steps = driver_.convertAccelerationToSteps(decel_val, unit);

  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetAccelerations(accel=%.2f, decel=%.2f steps/s²)",
                    accel_steps, decel_steps);

  int32_t accel_internal = driver_.accelToInternal(std::abs(accel_steps));
  int32_t decel_internal = driver_.accelToInternal(std::abs(decel_steps));
  accel_internal = std::min(accel_internal,
                            static_cast<decltype(accel_internal)>(0xFFFF)); // AMAX/DMAX are 16 bits
  decel_internal = std::min(decel_internal, static_cast<decltype(decel_internal)>(0xFFFF));
  bool success = true;
  success &= driver_.comm_.WriteRegister(Registers::AMAX, static_cast<uint32_t>(accel_internal));
  if (success) {
    driver_.write_only_regs_.amax = static_cast<uint32_t>(accel_internal);
  }
  success &= driver_.comm_.WriteRegister(Registers::DMAX, static_cast<uint32_t>(decel_internal));
  if (success) {
    driver_.write_only_regs_.dmax = static_cast<uint32_t>(decel_internal);
    // Update driver config if in steps unit
    if (unit == Unit::Steps) {
      driver_.driver_config_.ramp_config.amax = accel_val;
      driver_.driver_config_.ramp_config.dmax = decel_val;
    }
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetDeceleration(float value, Unit unit) noexcept {
  float decel_steps = driver_.convertAccelerationToSteps(value, unit);
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetDeceleration(decel=%.2f steps/s²)", decel_steps);
  int32_t decel_internal = driver_.accelToInternal(std::abs(decel_steps));
  decel_internal = std::min(decel_internal, static_cast<decltype(decel_internal)>(0xFFFF)); // DMAX is 16 bits
  return driver_.comm_.WriteRegister(Registers::DMAX, static_cast<uint32_t>(decel_internal));
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetRampSpeeds(float start_speed, float stop_speed, float transition_speed,
                                                   Unit unit) noexcept {
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
  if (success) {
    driver_.write_only_regs_.vstart = static_cast<uint32_t>(vstart);
  }
  success &= driver_.comm_.WriteRegister(Registers::VSTOP, static_cast<uint32_t>(vstop));
  if (success) {
    driver_.write_only_regs_.vstop = static_cast<uint32_t>(vstop);
  }
  success &= driver_.comm_.WriteRegister(Registers::V_1, static_cast<uint32_t>(v1));
  if (success) {
    driver_.write_only_regs_.v_1 = static_cast<uint32_t>(v1);
    // Update driver config if in steps unit
    if (unit == Unit::Steps) {
      driver_.driver_config_.ramp_config.vstart = start_speed;
      driver_.driver_config_.ramp_config.vstop = stop_speed;
      driver_.driver_config_.ramp_config.v1 = transition_speed;
    }
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetCurrentSpeed(float& speed, Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::VACTUAL, value)) {
    return false;
  }
  // VACTUAL is 24-bit signed
  auto signed_value = static_cast<int32_t>(value);
  if (signed_value & 0x800000) {
    signed_value |= static_cast<int32_t>(0xFF000000U); // Sign extend
  }
  float steps_per_sec = driver_.speedFromInternal(signed_value);
  speed = driver_.convertSpeedToUnit(steps_per_sec, unit);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::IsTargetReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.position_reached != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::IsTargetVelocityReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.velocity_reached != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::Stop() noexcept {
  bool success = true;
  success &= driver_.comm_.WriteRegister(Registers::VSTART, 0, driver_.GetCommAddress());
  success &= driver_.comm_.WriteRegister(Registers::VMAX, 0, driver_.GetCommAddress());
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::ConfigureReferenceSwitch(const ReferenceSwitchConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.reference_switch_config = config;

  // Read-Modify-Write to preserve other SW_MODE bits (like sg_stop)
  uint32_t sw_mode_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_val)) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = sw_mode_val;

  // Use stop enable flags (independent of active level)
  // Allows enabling/disabling motor stop in real-time while keeping polarity configured
  sw_mode.bits.stop_l_enable = config.left_switch_stop_enable ? 1 : 0;
  sw_mode.bits.stop_r_enable = config.right_switch_stop_enable ? 1 : 0;

  // Compute polarity from active level (ACTIVE_LOW = inverted polarity = true, ACTIVE_HIGH = normal = false)
  sw_mode.bits.pol_stop_l = (config.left_switch_active == ReferenceSwitchActiveLevel::ACTIVE_LOW) ? 1 : 0;
  sw_mode.bits.pol_stop_r = (config.right_switch_active == ReferenceSwitchActiveLevel::ACTIVE_LOW) ? 1 : 0;

  sw_mode.bits.swap_lr = config.swap_left_right ? 1 : 0;

  // Compute latching flags from enum
  sw_mode.bits.latch_l_active =
      (config.latch_left == ReferenceLatchMode::ACTIVE_EDGE || config.latch_left == ReferenceLatchMode::BOTH_EDGES) ? 1
                                                                                                                    : 0;
  sw_mode.bits.latch_l_inactive =
      (config.latch_left == ReferenceLatchMode::INACTIVE_EDGE || config.latch_left == ReferenceLatchMode::BOTH_EDGES)
          ? 1
          : 0;
  sw_mode.bits.latch_r_active =
      (config.latch_right == ReferenceLatchMode::ACTIVE_EDGE || config.latch_right == ReferenceLatchMode::BOTH_EDGES)
          ? 1
          : 0;
  sw_mode.bits.latch_r_inactive =
      (config.latch_right == ReferenceLatchMode::INACTIVE_EDGE || config.latch_right == ReferenceLatchMode::BOTH_EDGES)
          ? 1
          : 0;

  sw_mode.bits.en_latch_encoder = config.en_latch_encoder ? 1 : 0;
  sw_mode.bits.en_softstop = (config.stop_mode == ReferenceStopMode::SOFT_STOP) ? 1 : 0;

  return driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetReferenceSwitchConfig(ReferenceSwitchConfig& config) noexcept {
  uint32_t sw_mode_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_val)) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = sw_mode_val;

  // Read stop enable flags
  config.left_switch_stop_enable = (sw_mode.bits.stop_l_enable != 0);
  config.right_switch_stop_enable = (sw_mode.bits.stop_r_enable != 0);

  // Read polarity and convert to active level
  config.left_switch_active =
      (sw_mode.bits.pol_stop_l != 0) ? ReferenceSwitchActiveLevel::ACTIVE_LOW : ReferenceSwitchActiveLevel::ACTIVE_HIGH;
  config.right_switch_active =
      (sw_mode.bits.pol_stop_r != 0) ? ReferenceSwitchActiveLevel::ACTIVE_LOW : ReferenceSwitchActiveLevel::ACTIVE_HIGH;

  // Read swap
  config.swap_left_right = (sw_mode.bits.swap_lr != 0);

  // Read latching modes
  bool latch_l_active = (sw_mode.bits.latch_l_active != 0);
  bool latch_l_inactive = (sw_mode.bits.latch_l_inactive != 0);
  if (latch_l_active && latch_l_inactive) {
    config.latch_left = ReferenceLatchMode::BOTH_EDGES;
  } else if (latch_l_active) {
    config.latch_left = ReferenceLatchMode::ACTIVE_EDGE;
  } else if (latch_l_inactive) {
    config.latch_left = ReferenceLatchMode::INACTIVE_EDGE;
  } else {
    config.latch_left = ReferenceLatchMode::DISABLED;
  }

  bool latch_r_active = (sw_mode.bits.latch_r_active != 0);
  bool latch_r_inactive = (sw_mode.bits.latch_r_inactive != 0);
  if (latch_r_active && latch_r_inactive) {
    config.latch_right = ReferenceLatchMode::BOTH_EDGES;
  } else if (latch_r_active) {
    config.latch_right = ReferenceLatchMode::ACTIVE_EDGE;
  } else if (latch_r_inactive) {
    config.latch_right = ReferenceLatchMode::INACTIVE_EDGE;
  } else {
    config.latch_right = ReferenceLatchMode::DISABLED;
  }

  // Read encoder latching
  config.en_latch_encoder = (sw_mode.bits.en_latch_encoder != 0);

  // Read stop mode
  config.stop_mode = (sw_mode.bits.en_softstop != 0) ? ReferenceStopMode::SOFT_STOP : ReferenceStopMode::HARD_STOP;

  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetLeftSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept {
  ReferenceSwitchConfig config{};
  if (!GetReferenceSwitchConfig(config)) {
    return false;
  }
  config.left_switch_active = active_level;
  return ConfigureReferenceSwitch(config);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetRightSwitchActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept {
  ReferenceSwitchConfig config{};
  if (!GetReferenceSwitchConfig(config)) {
    return false;
  }
  config.right_switch_active = active_level;
  return ConfigureReferenceSwitch(config);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetLeftSwitchStopEnable(bool enable) noexcept {
  ReferenceSwitchConfig config{};
  if (!GetReferenceSwitchConfig(config)) {
    return false;
  }
  config.left_switch_stop_enable = enable;
  return ConfigureReferenceSwitch(config);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetRightSwitchStopEnable(bool enable) noexcept {
  ReferenceSwitchConfig config{};
  if (!GetReferenceSwitchConfig(config)) {
    return false;
  }
  config.right_switch_stop_enable = enable;
  return ConfigureReferenceSwitch(config);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetLeftSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept {
  ReferenceSwitchConfig config{};
  if (!GetReferenceSwitchConfig(config)) {
    return false;
  }
  config.latch_left = latch_mode;
  return ConfigureReferenceSwitch(config);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetRightSwitchLatchMode(ReferenceLatchMode latch_mode) noexcept {
  ReferenceSwitchConfig config{};
  if (!GetReferenceSwitchConfig(config)) {
    return false;
  }
  config.latch_right = latch_mode;
  return ConfigureReferenceSwitch(config);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetStopMode(ReferenceStopMode stop_mode) noexcept {
  ReferenceSwitchConfig config{};
  if (!GetReferenceSwitchConfig(config)) {
    return false;
  }
  config.stop_mode = stop_mode;
  return ConfigureReferenceSwitch(config);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::GetLatchedPosition(float& position, Unit unit) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XLATCH, value)) {
    return false;
  }
  int32_t steps = static_cast<int32_t>(value);
  position = driver_.convertStepsToUnit(steps, unit);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetComparePosition(float value, Unit unit) noexcept {
  float steps = driver_.convertPositionToSteps(value, unit);
  return driver_.comm_.WriteRegister(Registers::X_COMPARE, static_cast<uint32_t>(static_cast<int32_t>(steps)));
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetPowerDownDelay(uint8_t tpowerdown) noexcept {
  bool success = driver_.comm_.WriteRegister(Registers::TPOWERDOWN, static_cast<uint32_t>(tpowerdown));
  if (success) {
    driver_.write_only_regs_.tpowerdown = tpowerdown;
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetPowerDownDelayMs(float delay_ms) noexcept {
  // TPOWERDOWN: time_ms = tpowerdown * 2^18 / fCLK * 1000
  // Rearranged: tpowerdown = time_ms * fCLK / (2^18 * 1000)
  float tpowerdown_reg = (delay_ms * static_cast<float>(driver_.f_clk_)) / (262144.0f * 1000.0f);
  uint8_t tpowerdown = constrain<uint8_t>(static_cast<uint8_t>(std::round(tpowerdown_reg)), 0U, 255U);
  return SetPowerDownDelay(tpowerdown);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetZeroWaitTime(uint16_t tzerowait) noexcept {
  bool success = driver_.comm_.WriteRegister(Registers::TZEROWAIT, static_cast<uint32_t>(tzerowait));
  if (success) {
    driver_.write_only_regs_.tzerowait = tzerowait;
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetZeroWaitTimeMs(float delay_ms) noexcept {
  // TZEROWAIT: time_ms = tzerowait * 2^18 / fCLK * 1000
  // Rearranged: tzerowait = time_ms * fCLK / (2^18 * 1000)
  float tzerowait_reg = (delay_ms * static_cast<float>(driver_.f_clk_)) / (262144.0f * 1000.0f);
  uint16_t tzerowait = constrain<uint16_t>(static_cast<uint16_t>(std::round(tzerowait_reg)), 0U, 65535U);
  return SetZeroWaitTime(tzerowait);
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::ConfigureRamp(const RampConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.ramp_config = config;

  // Set timing parameters first (convert from milliseconds to register values)
  if (config.tpowerdown_ms >= 0.0f) {
    float tpowerdown_reg = (config.tpowerdown_ms * static_cast<float>(driver_.f_clk_)) / (262144.0f * 1000.0f);
    uint8_t tpowerdown = constrain<uint8_t>(static_cast<uint8_t>(std::round(tpowerdown_reg)), 0U, 255U);
    if (!SetPowerDownDelay(tpowerdown)) {
      return false;
    }
  }

  if (config.tzerowait_ms >= 0.0f) {
    float tzerowait_reg = (config.tzerowait_ms * static_cast<float>(driver_.f_clk_)) / (262144.0f * 1000.0f);
    uint16_t tzerowait = constrain<uint16_t>(static_cast<uint16_t>(std::round(tzerowait_reg)), 0U, 65535U);
    if (!SetZeroWaitTime(tzerowait)) {
      return false;
    }
  }

  // Get unit specifications for proper conversion
  Unit velocity_unit = config.velocity_unit;
  Unit acceleration_unit = config.acceleration_unit;

  // Set velocity parameters (VSTART, VSTOP, V1) using specified unit
  float vstart = config.vstart;
  float vstop = config.vstop;
  float v1 = config.v1;

  // Ensure VSTOP >= VSTART (datasheet requirement)
  if (vstop < vstart && vstart > 0.0f) {
    vstop = vstart; // Use VSTART if VSTOP is less
  } else if (vstop == 0.0f && vstart == 0.0f) {
    vstop = 10.0f; // Default minimum VSTOP
  }

  if (!SetRampSpeeds(vstart, vstop, v1, velocity_unit)) {
    return false;
  }

  // Set maximum velocity if configured (VMAX) using specified unit
  if (config.vmax > 0.0f) {
    if (!SetMaxSpeed(config.vmax, velocity_unit)) {
      return false;
    }
  }

  // Set acceleration parameters using specified unit
  if (config.amax > 0.0f) {
    float dmax = (config.dmax > 0.0f) ? config.dmax : config.amax;
    if (!SetAccelerations(config.amax, dmax, acceleration_unit)) {
      return false;
    }
  }

  // A1 (first acceleration, used between VSTART and V1) using specified unit
  if (config.a1 > 0.0f) {
    if (!SetFirstAcceleration(config.a1, acceleration_unit)) {
      return false;
    }
  }

  // D1 (first deceleration, used between VSTOP and V1, must not be 0 in positioning mode) using specified unit
  if (config.d1 > 0.0f) {
    if (!SetFinalDeceleration(config.d1, acceleration_unit)) {
      return false;
    }
  } else {
    // Set default D1 if not configured (required for positioning mode)
    if (!SetFinalDeceleration(100.0f, Unit::Steps)) {
      return false;
    }
  }

  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetFirstAcceleration(float a1, Unit unit) noexcept {
  float a1_steps = driver_.convertAccelerationToSteps(a1, unit);
  uint32_t a1_value = 0;
  if (a1_steps == 0.0F) {
    // Set to 0 to use AMAX for this phase
    a1_value = 0;
  } else {
    int32_t a1_internal = driver_.accelToInternal(std::abs(a1_steps));
    a1_internal = std::min(a1_internal, static_cast<decltype(a1_internal)>(0xFFFF)); // A_1 is 16 bits
    a1_value = static_cast<uint32_t>(a1_internal);
  }
  bool success = driver_.comm_.WriteRegister(Registers::A_1, a1_value, driver_.GetCommAddress());
  if (success) {
    driver_.write_only_regs_.a_1 = a1_value;
    // Update driver config if in steps unit
    if (unit == Unit::Steps) {
      driver_.driver_config_.ramp_config.a1 = a1;
    }
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::RampControl::SetFinalDeceleration(float d1, Unit unit) noexcept {
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
bool TMC51x0<CommType>::MotorControl::Enable() noexcept {
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::Enable()");

  // Enable via EN pin GPIO (EN is active LOW to enable, so set to ACTIVE/LOW to enable power stage)
  // This must be done first to enable the power stage
  driver_.comm_.GpioSet(TMC51x0CtrlPin::EN, GpioSignal::ACTIVE);

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
bool TMC51x0<CommType>::MotorControl::Disable() noexcept {
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::Disable()");

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
  driver_.comm_.GpioSet(TMC51x0CtrlPin::EN, GpioSignal::INACTIVE);

  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetCurrent(uint8_t irun, uint8_t ihold) noexcept {
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::SetCurrent(irun=%u, ihold=%u)", irun, ihold);
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
bool TMC51x0<CommType>::MotorControl::ConfigureChopper(const ChopperConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.chopper = config;

  // Read-Modify-Write to preserve any fields not explicitly set in ChopperConfig
  uint32_t chopconf_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::CHOPCONF, chopconf_val)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = chopconf_val;

  // Common fields
  chopconf.bits.toff = constrain<decltype(config.toff)>(config.toff, 0U, 15U);
  chopconf.bits.tbl = constrain<decltype(config.tbl)>(config.tbl, 0U, 3U);
  chopconf.bits.tpfd = constrain<decltype(config.tpfd)>(config.tpfd, 0U, 15U);
  chopconf.bits.mres = constrain<uint8_t>(static_cast<uint8_t>(config.mres), 0U, 8U);
  chopconf.bits.intpol = config.intpol ? 1 : 0;
  chopconf.bits.dedge = config.dedge ? 1 : 0;
  chopconf.bits.vhighfs = config.vhighfs ? 1 : 0;
  chopconf.bits.vhighchm = config.vhighchm ? 1 : 0;
  chopconf.bits.diss2g = config.diss2g ? 1 : 0;
  chopconf.bits.diss2vs = config.diss2vs ? 1 : 0;
  // Note: Bit 17 is reserved per datasheet and is not used

  // Mode-specific fields
  bool is_classic_mode = (config.mode == ChopperMode::CLASSIC);
  chopconf.bits.chm = is_classic_mode ? 1 : 0;

  if (is_classic_mode) {
    // Classic mode: hstrt_tfd = TFD[2:0], hend_offset = OFFSET, tfd_3 = TFD[3], disfdcc
    uint8_t tfd_constrained = constrain<decltype(config.tfd)>(config.tfd, 0U, 15U);
    chopconf.bits.hstrt_tfd = tfd_constrained & 0x07;                                   // Bits 2:0
    chopconf.bits.tfd_3 = (tfd_constrained >> 3) & 0x01;                                // Bit 3
    chopconf.bits.hend_offset = constrain<decltype(config.hend)>(config.hend, 0U, 15U); // OFFSET
    chopconf.bits.disfdcc = config.disfdcc ? 1 : 0;
  } else {
    // SpreadCycle mode: hstrt_tfd = HSTRT, hend_offset = HEND
    chopconf.bits.hstrt_tfd = constrain<decltype(config.hstrt)>(config.hstrt, 0U, 7U);
    chopconf.bits.hend_offset = constrain<decltype(config.hend)>(config.hend, 0U, 15U);
    chopconf.bits.tfd_3 = 0;   // Reserved in SpreadCycle mode
    chopconf.bits.disfdcc = 0; // Not used in SpreadCycle mode
  }

  bool success = driver_.comm_.WriteRegister(Registers::CHOPCONF, chopconf.value, driver_.GetCommAddress());

  // Update stored microsteps
  if (success) {
    uint8_t mres = constrain<uint8_t>(static_cast<uint8_t>(config.mres), 0U, 8U);
    driver_.current_microsteps_ = 256U >> mres;
  }

  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::ConfigureStealthChop(const StealthChopConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.stealthchop = config;

  PWMCONF_Register pwmconf{};
  pwmconf.bits.pwm_ofs = config.pwm_ofs;
  pwmconf.bits.pwm_grad = config.pwm_grad;
  pwmconf.bits.pwm_freq = constrain<decltype(config.pwm_freq)>(config.pwm_freq, 0U, 3U);
  pwmconf.bits.pwm_autoscale = config.pwm_autoscale ? 1 : 0;
  pwmconf.bits.pwm_autograd = config.pwm_autograd ? 1 : 0;
  pwmconf.bits.pwm_reg = constrain<decltype(config.pwm_reg)>(config.pwm_reg, 0U, 15U);
  pwmconf.bits.pwm_lim = constrain<decltype(config.pwm_lim)>(config.pwm_lim, 0U, 15U);
  pwmconf.bits.freewheel = static_cast<uint8_t>(config.freewheel);
  bool success = driver_.comm_.WriteRegister(Registers::PWMCONF, pwmconf.value, driver_.GetCommAddress());
  if (success) {
    driver_.write_only_regs_.pwmconf = pwmconf.value;
  }

  // Configure StealthChop velocity threshold (TPWMTHRS) if set
  if (success && config.velocity_threshold > 0.0F) {
    float steps_per_sec = driver_.convertSpeedToSteps(config.velocity_threshold, config.velocity_threshold_unit);
    int32_t tpwmthrs = driver_.thresholdSpeedToTstep(steps_per_sec);
    tpwmthrs = std::min(tpwmthrs, static_cast<decltype(tpwmthrs)>(0xFFFFF)); // TPWMTHRS is 20 bits
    TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "Setting TPWMTHRS=%ld (%.2f %s)", tpwmthrs,
                      config.velocity_threshold,
                      (config.velocity_threshold_unit == Unit::Steps) ? "steps/s" : "units/s");
    success =
        driver_.comm_.WriteRegister(Registers::TPWMTHRS, static_cast<uint32_t>(tpwmthrs), driver_.GetCommAddress());
    if (success) {
      driver_.write_only_regs_.tpwmthrs = static_cast<uint32_t>(tpwmthrs);
    }
  }

  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::ConfigurePowerStage(const PowerStageParameters& config) noexcept {
  // Update driver config
  driver_.driver_config_.power_stage = config;

  // Configure power stage (calculate register values from user-friendly parameters)
  DRV_CONF_Register drv_conf{};

  // Calculate DRVSTRENGTH from MOSFET Miller charge
  float miller = config.mosfet_miller_charge_nc;
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
  uint32_t bbm_ns = config.bbm_time_ns;

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
    uint32_t clock_cycles =
        static_cast<uint32_t>((bbm_ns_with_headroom * static_cast<float>(driver_.f_clk_)) / 1000000000.0f);
    bbm_clks_reg = constrain<uint8_t>(static_cast<uint8_t>(clock_cycles), 0U, 15U);
    if (bbm_clks_reg == 0 && bbm_ns > 200) {
      bbm_clks_reg = 1; // Minimum 1 clock cycle if >200ns requested
    }
    bbm_time_reg = 0; // BBMCLKS takes precedence when set (longer setting rules)
  }

  drv_conf.bits.bbmtime = bbm_time_reg;
  drv_conf.bits.bbmclks = bbm_clks_reg;
  drv_conf.bits.otselect = static_cast<uint8_t>(config.over_temp_protection);
  drv_conf.bits.filt_isense = static_cast<uint8_t>(config.sense_filter);

  TMC51X0_LOG_DEBUG(
      driver_.comm_, 2, "TMC5160", "Power stage: DRVSTRENGTH=%u (from %.1fnC), BBMTIME=%u, BBMCLKS=%u, FILT_ISENSE=%u",
      drv_conf.bits.drvstrength, miller, drv_conf.bits.bbmtime, drv_conf.bits.bbmclks, drv_conf.bits.filt_isense);

  bool success = driver_.comm_.WriteRegister(Registers::DRV_CONF, drv_conf.value, driver_.GetCommAddress());
  if (success) {
    // Track write-only register
    driver_.write_only_regs_.drv_conf = drv_conf.value;
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::ConfigureMotorCurrent(const MotorSpec& motor_spec) noexcept {
  // Validate inputs
  if (motor_spec.sense_resistor_mohm == 0 || motor_spec.supply_voltage_mv == 0) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "TMC5160",
                      "Cannot calculate motor current: sense_resistor_mohm=%u, supply_voltage_mv=%u",
                      motor_spec.sense_resistor_mohm, motor_spec.supply_voltage_mv);
    return false;
  }

  // Calculate motor current settings from physical parameters
  uint8_t calc_irun = 0;
  uint8_t calc_ihold = 0;
  uint16_t calc_scaler = 0;

  uint16_t run_current = motor_spec.run_current_ma;
  if (run_current == 0) {
    run_current = motor_spec.rated_current_ma;
  }
  if (run_current == 0) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "TMC5160", "Failed to calculate motor current: no current specified");
    return false;
  }

  if (!CalculateMotorCurrent(motor_spec, motor_spec.sense_resistor_mohm, motor_spec.supply_voltage_mv, run_current,
                             motor_spec.hold_current_ma, calc_irun, calc_ihold, calc_scaler)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "TMC5160", "Failed to calculate motor current settings");
    return false;
  }

  // Apply percentage adjustments before constraining
  // GLOBAL_SCALER adjustment
  if (motor_spec.scaler_adjustment_percent != 0.0f) {
    float adjustment_factor = 1.0f + (motor_spec.scaler_adjustment_percent / 100.0f);
    calc_scaler = static_cast<uint16_t>(std::round(static_cast<float>(calc_scaler) * adjustment_factor));
  }

  // IRUN adjustment
  if (motor_spec.irun_adjustment_percent != 0.0f) {
    float adjustment_factor = 1.0f + (motor_spec.irun_adjustment_percent / 100.0f);
    calc_irun = static_cast<uint8_t>(std::round(static_cast<float>(calc_irun) * adjustment_factor));
  }

  // IHOLD adjustment
  if (motor_spec.ihold_adjustment_percent != 0.0f) {
    float adjustment_factor = 1.0f + (motor_spec.ihold_adjustment_percent / 100.0f);
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

  // Store calculated values internally
  driver_.calculated_irun_ = calc_irun;
  driver_.calculated_ihold_ = calc_ihold;
  driver_.calculated_global_scaler_ = calc_scaler;

  TMC51X0_LOG_DEBUG(
      driver_.comm_, 1, "TMC5160",
      "Calculated: IRUN=%u, IHOLD=%u, GLOBAL_SCALER=%u (adjustments: scaler=%.1f%%, irun=%.1f%%, ihold=%.1f%%)",
      calc_irun, calc_ihold, calc_scaler, motor_spec.scaler_adjustment_percent, motor_spec.irun_adjustment_percent,
      motor_spec.ihold_adjustment_percent);

  // Configure global scaler (TMC5160 only, TMC5130 doesn't have this register)
  if (driver_.chip_version_ != ChipVersion::TMC5130) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 3, "TMC5160", "Setting GLOBAL_SCALER=%u", calc_scaler);
    if (!driver_.comm_.WriteRegister(Registers::GLOBAL_SCALER, calc_scaler, driver_.GetCommAddress())) {
      return false;
    }
  } else {
    // TMC5130: Skip GLOBAL_SCALER, use calculated IRUN directly
    TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "TMC5130: Skipping GLOBAL_SCALER (not supported)");
  }

  // Configure motor current (use calculated values)
  // Calculate IHOLDDELAY from total delay time, clock frequency, and current step count
  // IHOLDDELAY is delay PER current reduction step
  // Total delay = (IRUN - IHOLD) × IHOLDDELAY × (2^18 / f_clk)
  // Rearranged: IHOLDDELAY = (total_delay_ms / (IRUN - IHOLD)) × f_clk / (1000 × 2^18)
  uint8_t iholddelay_value = 0;
  if (motor_spec.iholddelay_ms > 0.0f) {
    uint8_t current_steps = (calc_irun > calc_ihold) ? (calc_irun - calc_ihold) : 0;

    if (current_steps > 0) {
      // Calculate per-step delay from total delay
      float per_step_delay_ms = motor_spec.iholddelay_ms / static_cast<float>(current_steps);
      // Calculate IHOLDDELAY register value: delay_value = (per_step_delay_ms * f_clk) / (1000 * 2^18)
      // Where 2^18 = 262144
      float delay_clocks = (per_step_delay_ms * static_cast<float>(driver_.f_clk_)) / (1000.0f * 262144.0f);
      iholddelay_value = constrain<uint8_t>(static_cast<uint8_t>(std::round(delay_clocks)), 0U, 15U);

      // Calculate actual total delay for logging
      float actual_total_delay_ms = static_cast<float>(current_steps) * static_cast<float>(iholddelay_value) *
                                    (262144.0f * 1000.0f / static_cast<float>(driver_.f_clk_));
      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160",
                        "IHOLDDELAY calculation: desired_total=%.2f ms, steps=%u, per_step=%.2f ms, IHOLDDELAY=%u, "
                        "actual_total=%.2f ms",
                        motor_spec.iholddelay_ms, current_steps, per_step_delay_ms, iholddelay_value,
                        actual_total_delay_ms);
    } else {
      // IRUN == IHOLD, no current reduction steps, delay is always 0
      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160",
                        "IHOLDDELAY ignored (IRUN=%u == IHOLD=%u, no current reduction steps)", calc_irun, calc_ihold);
    }
  }
  // else: iholddelay_value = 0 (instant power down)

  IHOLD_IRUN_Register iholdrun{};
  iholdrun.bits.ihold = calc_ihold;
  iholdrun.bits.irun = calc_irun;
  iholdrun.bits.iholddelay = iholddelay_value;
  TMC51X0_LOG_DEBUG(driver_.comm_, 3, "TMC5160", "Setting IHOLD_IRUN(irun=%u, ihold=%u, iholddelay=%u)",
                    iholdrun.bits.irun, iholdrun.bits.ihold, iholdrun.bits.iholddelay);

  bool success = true;
  // Configure global scaler (TMC5160 only, TMC5130 doesn't have this register)
  if (driver_.chip_version_ != ChipVersion::TMC5130) {
    success = driver_.comm_.WriteRegister(Registers::GLOBAL_SCALER, calc_scaler, driver_.GetCommAddress());
    if (success) {
      driver_.write_only_regs_.global_scaler = calc_scaler;
    }
  } else {
    // TMC5130: Skip GLOBAL_SCALER, use calculated IRUN directly
    TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "TMC5130: Skipping GLOBAL_SCALER (not supported)");
  }
  if (success) {
    success = driver_.comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, driver_.GetCommAddress());
    if (success) {
      driver_.write_only_regs_.ihold_irun = iholdrun.value;
    }
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetModeChangeSpeeds(float pwm_thrs, float cool_thrs, float high_thrs) noexcept {
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
bool TMC51x0<CommType>::MotorControl::SetCoolStepThreshold(float value, Unit unit) noexcept {
  float steps_per_sec = driver_.convertSpeedToSteps(value, unit);
  int32_t tcoolthrs = driver_.thresholdSpeedToTstep(steps_per_sec);
  tcoolthrs = std::min(tcoolthrs, static_cast<decltype(tcoolthrs)>(0xFFFFF));
  return driver_.comm_.WriteRegister(Registers::TCOOLTHRS, static_cast<uint32_t>(tcoolthrs));
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetHighSpeedThreshold(float value, Unit unit) noexcept {
  float steps_per_sec = driver_.convertSpeedToSteps(value, unit);
  int32_t thigh = driver_.thresholdSpeedToTstep(steps_per_sec);
  thigh = std::min(thigh, static_cast<decltype(thigh)>(0xFFFFF));
  return driver_.comm_.WriteRegister(Registers::THIGH, static_cast<uint32_t>(thigh));
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetStealthChopVelocityThreshold(float value, Unit unit) noexcept {
  if (value <= 0.0F) {
    // Setting to 0 disables the threshold (StealthChop always used if enabled)
    return driver_.comm_.WriteRegister(Registers::TPWMTHRS, 0, driver_.GetCommAddress());
  }
  float steps_per_sec = driver_.convertSpeedToSteps(value, unit);
  int32_t tpwmthrs = driver_.thresholdSpeedToTstep(steps_per_sec);
  tpwmthrs = std::min(tpwmthrs, static_cast<decltype(tpwmthrs)>(0xFFFFF)); // TPWMTHRS is 20 bits
  return driver_.comm_.WriteRegister(Registers::TPWMTHRS, static_cast<uint32_t>(tpwmthrs), driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetGlobalScaler(uint16_t scaler) noexcept {
  // TMC5130 doesn't support GLOBAL_SCALER register
  if (driver_.chip_version_ == ChipVersion::TMC5130) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TMC5160", "TMC5130: GLOBAL_SCALER not supported, skipping");
    return true; // Return success but don't write
  }
  scaler = constrain<decltype(scaler)>(scaler, 32U, 256U);
  return driver_.comm_.WriteRegister(Registers::GLOBAL_SCALER, scaler, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::ConfigureCoolStep(const CoolStepConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.coolstep = config;

  uint32_t coolconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::COOLCONF, coolconf_value)) {
    return false;
  }
  COOLCONF_Register coolconf{};
  coolconf.value = coolconf_value;

  // Preserve SGT (StallGuard2 threshold) - it's configured separately via StallGuardConfig
  // Only update CoolStep-specific fields

  // Calculate SEMIN and SEMAX from user-friendly thresholds
  uint8_t semin = 0;
  uint8_t semax = 0;

  if (config.lower_threshold_sg > 0) {
    // Convert SG threshold to SEMIN: threshold = SEMIN * 32
    // SEMIN = threshold / 32, clamped to 0-15
    semin = static_cast<uint8_t>(config.lower_threshold_sg / 32U);
    semin = constrain<decltype(semin)>(semin, 0U, 15U);

    // Calculate SEMAX from upper threshold
    if (config.upper_threshold_sg > 0) {
      // Upper threshold = (SEMIN + SEMAX + 1) * 32
      // SEMAX = (upper_threshold / 32) - SEMIN - 1
      uint16_t upper_semin_equiv = static_cast<uint16_t>(config.upper_threshold_sg / 32U);
      if (upper_semin_equiv > semin) {
        semax = static_cast<uint8_t>(upper_semin_equiv - semin - 1U);
        semax = constrain<decltype(semax)>(semax, 0U, 15U);
      } else {
        // Upper threshold must be higher than lower threshold
        // If invalid, set semax to 0 (will use minimum hysteresis)
        semax = 0;
      }
    } else {
      // If upper threshold not specified, use default hysteresis
      // Default: semax = 5 (provides reasonable hysteresis gap)
      semax = 5;
    }
  }
  // If lower_threshold_sg = 0, semin = 0 (CoolStep disabled)

  // Update CoolStep register fields
  coolconf.bits.semin = semin; // 0 = CoolStep disabled
  coolconf.bits.semax = semax;

  // Convert increment step enum to register value
  coolconf.bits.seup = static_cast<uint8_t>(config.increment_step);

  // Convert decrement speed enum to register value
  coolconf.bits.sedn = static_cast<uint8_t>(config.decrement_speed);

  // Convert minimum current enum to register value
  coolconf.bits.seimin = (config.min_current == CoolStepMinCurrent::QUARTER_IRUN) ? 1 : 0;

  // Update filter enable
  coolconf.bits.sfilt = config.enable_filter ? 1 : 0;

  // Write COOLCONF register
  bool success = driver_.comm_.WriteRegister(Registers::COOLCONF, coolconf.value, driver_.GetCommAddress());
  if (success) {
    driver_.write_only_regs_.coolconf = coolconf.value;
  }

  // Configure velocity thresholds if provided
  if (success && config.min_velocity > 0.0F) {
    float steps_per_sec = driver_.convertSpeedToSteps(config.min_velocity, config.velocity_unit);
    int32_t tcoolthrs = driver_.thresholdSpeedToTstep(steps_per_sec);
    tcoolthrs = std::min(tcoolthrs, static_cast<decltype(tcoolthrs)>(0xFFFFF)); // 20 bits
    success &= driver_.comm_.WriteRegister(Registers::TCOOLTHRS, static_cast<uint32_t>(tcoolthrs));
    if (success) {
      driver_.write_only_regs_.tcoolthrs = static_cast<uint32_t>(tcoolthrs);
    }
  }

  if (success && config.max_velocity > 0.0F) {
    float steps_per_sec = driver_.convertSpeedToSteps(config.max_velocity, config.velocity_unit);
    int32_t thigh = driver_.thresholdSpeedToTstep(steps_per_sec);
    thigh = std::min(thigh, static_cast<decltype(thigh)>(0xFFFFF)); // 20 bits
    success &= driver_.comm_.WriteRegister(Registers::THIGH, static_cast<uint32_t>(thigh));
    if (success) {
      driver_.write_only_regs_.thigh = static_cast<uint32_t>(thigh);
    }
  }

  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::ConfigureDcStep(const DcStepConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.dcstep = config;

  // Convert velocity threshold to internal format with unit support
  int32_t vdc_min = 0;
  if (config.min_velocity > 0.0F) {
    float steps_per_sec = driver_.convertSpeedToSteps(config.min_velocity, config.velocity_unit);
    vdc_min = driver_.speedToInternal(steps_per_sec);
    // VDCMIN is 23-bit register (0...2^22), but only bits 22..8 are used for comparison
    // Lower 8 bits (7..0) are ignored/unused by the hardware comparator.
    // We mask the value to 0x7FFF00 to explicitly zero out the unused bits,
    // ensuring the register value reflects the effective threshold.
    // Note: No shift is required; the register compares bits 22..8 of VDCMIN
    // against bits 22..8 of VACTUAL.
    vdc_min = vdc_min & 0x7FFF00;
  }
  bool success = driver_.comm_.WriteRegister(Registers::VDCMIN, static_cast<uint32_t>(vdc_min));
  if (success) {
    driver_.write_only_regs_.vdcmin = static_cast<uint32_t>(vdc_min);
  }
  if (!success) {
    return false;
  }

  // Calculate DC_TIME from PWM on-time (microseconds) or auto-calculate from blank time
  uint16_t dc_time = 0;

  if (config.pwm_on_time_us > 0.0F) {
    // Convert microseconds to clock cycles: cycles = (time_us * f_clk) / 1e6
    float clock_cycles = (config.pwm_on_time_us * static_cast<float>(driver_.f_clk_)) / 1000000.0F;
    dc_time = static_cast<uint16_t>(std::round(clock_cycles));
    dc_time = constrain<decltype(dc_time)>(dc_time, 0U, 1023U);
  } else {
    // Auto-calculate from blank time (TBL) if not specified
    // Read current CHOPCONF to get TBL value
    uint32_t chopconf_value = 0;
    if (driver_.comm_.ReadRegister(Registers::CHOPCONF, chopconf_value)) {
      CHOPCONF_Register chopconf{};
      chopconf.value = chopconf_value;

      // TBL values: 0=16 clocks, 1=24 clocks, 2=36 clocks, 3=54 clocks
      uint8_t tbl_clocks[] = {16, 24, 36, 54};
      uint8_t tbl_index = constrain<uint8_t>(chopconf.bits.tbl, 0U, 3U);
      uint8_t blank_time_clocks = tbl_clocks[tbl_index];

      // DC_TIME should be set slightly above blank time
      // Datasheet: Lower limit = TBL + n (where n = 1-100 for typical motor)
      // Use n = 20 as a reasonable default (provides margin without being too conservative)
      dc_time = blank_time_clocks + 20;
      dc_time = constrain<decltype(dc_time)>(dc_time, 0U, 1023U);
    } else {
      // If can't read CHOPCONF, use conservative default (36 clocks + 20 = 56)
      dc_time = 56;
    }
  }

  // Calculate DC_SG from DC_TIME based on sensitivity
  uint8_t dc_sg = 0;
  if (config.stall_sensitivity != DcStepStallSensitivity::DISABLED && dc_time > 0) {
    // DC_SG should be set slightly higher than DC_TIME/16
    // Different sensitivity levels use different multipliers
    float dc_sg_float = 0.0F;
    switch (config.stall_sensitivity) {
    case DcStepStallSensitivity::LOW:
      dc_sg_float = static_cast<float>(dc_time) / 20.0F; // Less sensitive
      break;
    case DcStepStallSensitivity::MODERATE:
      dc_sg_float = static_cast<float>(dc_time) / 16.0F; // Recommended (datasheet default)
      break;
    case DcStepStallSensitivity::HIGH:
      dc_sg_float = static_cast<float>(dc_time) / 12.0F; // More sensitive
      break;
    default:
      dc_sg_float = 0.0F;
      break;
    }
    dc_sg = static_cast<uint8_t>(std::round(dc_sg_float));
    // Ensure minimum value of 1 if sensitivity is enabled
    if (dc_sg == 0 && config.stall_sensitivity != DcStepStallSensitivity::DISABLED) {
      dc_sg = 1;
    }
    dc_sg = constrain<decltype(dc_sg)>(dc_sg, 0U, 255U);
  }

  // Configure DCCTRL register
  DCCTRL_Register dcctrl{};
  dcctrl.bits.dc_time = dc_time;
  dcctrl.bits.dc_sg = dc_sg;
  bool dcctrl_success = driver_.comm_.WriteRegister(Registers::DCCTRL, dcctrl.value, driver_.GetCommAddress());
  if (dcctrl_success) {
    driver_.write_only_regs_.dcctrl = dcctrl.value;
  }
  success &= dcctrl_success;

  // Configure stop on stall if requested
  if (success && config.stop_on_stall && config.stall_sensitivity != DcStepStallSensitivity::DISABLED) {
    // Read current SW_MODE register
    uint32_t sw_mode_value = 0;
    if (driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_value)) {
      SW_MODE_Register sw_mode{};
      sw_mode.value = sw_mode_value;
      sw_mode.bits.sg_stop = 1; // Enable stop on stall
      success &= driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value, driver_.GetCommAddress());
    }
  }

  // Ensure CHOPCONF.vhighfs and CHOPCONF.vhighchm are set for DcStep
  // These flags switch to fullstepping when VDCMIN is exceeded
  // Note: These can also be set via ChopperConfig.vhighfs and ChopperConfig.vhighchm
  if (success && config.min_velocity > 0.0F) {
    uint32_t chopconf_value = 0;
    if (driver_.comm_.ReadRegister(Registers::CHOPCONF, chopconf_value)) {
      CHOPCONF_Register chopconf{};
      chopconf.value = chopconf_value;
      chopconf.bits.vhighfs = 1;  // Enable fullstepping at high velocity
      chopconf.bits.vhighchm = 1; // Enable chopper mode switching at high velocity
      success &= driver_.comm_.WriteRegister(Registers::CHOPCONF, chopconf.value, driver_.GetCommAddress());
    }
  }

  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::ConfigureGlobalConfig(const GlobalConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.global_config = config;

  // Read-Modify-Write to preserve reserved bits (bits 18-31)
  uint32_t gconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, gconf_value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = gconf_value;

  // Update only the configurable bits, preserving reserved bits
  gconf.bits.recalibrate = config.recalibrate ? 1 : 0;
  gconf.bits.faststandstill = config.en_short_standstill_timeout ? 1 : 0;
  gconf.bits.en_pwm_mode = config.en_stealthchop_mode ? 1 : 0;
  gconf.bits.multistep_filt = config.en_stealthchop_step_filter ? 1 : 0;
  gconf.bits.shaft = config.invert_direction ? 1 : 0;
  gconf.bits.diag0_error = config.diag0.error ? 1 : 0;
  gconf.bits.diag0_otpw = config.diag0.otpw ? 1 : 0;
  gconf.bits.diag0_stall_step = config.diag0.stall_step ? 1 : 0;
  gconf.bits.diag1_stall_dir = config.diag1.stall_dir ? 1 : 0;
  gconf.bits.diag1_index = config.diag1.index ? 1 : 0;
  gconf.bits.diag1_onstate = config.diag1.onstate ? 1 : 0;
  gconf.bits.diag1_steps_skipped = config.diag1.steps_skipped ? 1 : 0;
  gconf.bits.diag0_int_pushpull = config.diag0.pushpull ? 1 : 0;
  gconf.bits.diag1_poscomp_pushpull = config.diag1.pushpull ? 1 : 0;
  gconf.bits.small_hysteresis = config.en_small_step_frequency_hysteresis ? 1 : 0;
  gconf.bits.stop_enable = config.enca_dcin_sequencer_stop ? 1 : 0;
  gconf.bits.direct_mode = config.direct_mode ? 1 : 0;
  gconf.bits.test_mode = 0; // Always disabled (factory test mode, not for user)
  // Reserved bits (18-31) are preserved from read value

  return driver_.comm_.WriteRegister(Registers::GCONF, gconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::GetGlobalConfig(GlobalConfig& config) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;

  config.recalibrate = gconf.bits.recalibrate != 0;
  config.en_short_standstill_timeout = gconf.bits.faststandstill != 0;
  config.en_stealthchop_mode = gconf.bits.en_pwm_mode != 0;
  config.en_stealthchop_step_filter = gconf.bits.multistep_filt != 0;
  config.invert_direction = gconf.bits.shaft != 0;
  config.diag0.error = gconf.bits.diag0_error != 0;
  config.diag0.otpw = gconf.bits.diag0_otpw != 0;
  config.diag0.stall_step = gconf.bits.diag0_stall_step != 0;
  config.diag0.pushpull = gconf.bits.diag0_int_pushpull != 0;
  config.diag1.stall_dir = gconf.bits.diag1_stall_dir != 0;
  config.diag1.index = gconf.bits.diag1_index != 0;
  config.diag1.onstate = gconf.bits.diag1_onstate != 0;
  config.diag1.steps_skipped = gconf.bits.diag1_steps_skipped != 0;
  config.diag1.pushpull = gconf.bits.diag1_poscomp_pushpull != 0;
  config.en_small_step_frequency_hysteresis = gconf.bits.small_hysteresis != 0;
  config.enca_dcin_sequencer_stop = gconf.bits.stop_enable != 0;
  config.direct_mode = gconf.bits.direct_mode != 0;
  // test_mode is not exposed to user (factory use only)
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetStealthChopEnabled(bool enabled) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = value;
  gconf.bits.en_pwm_mode = enabled ? 1 : 0; // Register bit name is en_pwm_mode, but it controls StealthChop
  return driver_.comm_.WriteRegister(Registers::GCONF, gconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::IsEnabled() noexcept {
  ChopperConfig config{};
  if (!GetChopperConfig(config)) {
    return false;
  }
  return config.toff > 0; // Motor is enabled if toff > 0
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::IsStealthChopEnabled() noexcept {
  uint32_t gconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, gconf_value, driver_.GetCommAddress())) {
    return false;
  }
  GCONF_Register gconf{};
  gconf.value = gconf_value;
  return gconf.bits.en_pwm_mode != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::IsStealthChopCalibrated() noexcept {
  uint8_t pwm_scale_sum = 0;
  int16_t pwm_scale_auto = 0;
  // Note: GetPwmScale is declared in MotorControl but implemented in Diagnostics
  // Using Diagnostics implementation for now
  if (!driver_.diagnostics.GetPwmScale(pwm_scale_sum, pwm_scale_auto)) {
    return false;
  }
  // StealthChop is calibrated if pwm_scale_auto is non-zero
  // pwm_scale_auto is a 9-bit signed value
  // Sign extend 9-bit to 16-bit for proper comparison
  if (pwm_scale_auto & 0x100) {
    pwm_scale_auto |= 0xFE00;
  }
  // Consider calibrated if value is not 0 and not in the very small range (-10 to 10)
  return (pwm_scale_auto != 0) && !(pwm_scale_auto > -10 && pwm_scale_auto < 10);
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::GetChopperConfig(ChopperConfig& config) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::CHOPCONF, value)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = value;

  // Common fields
  config.toff = static_cast<uint8_t>(chopconf.bits.toff);
  config.tbl = static_cast<uint8_t>(chopconf.bits.tbl);
  config.tpfd = static_cast<uint8_t>(chopconf.bits.tpfd);
  config.mres = static_cast<MicrostepResolution>(chopconf.bits.mres);
  config.intpol = chopconf.bits.intpol != 0;
  config.dedge = chopconf.bits.dedge != 0;
  config.vhighfs = chopconf.bits.vhighfs != 0;
  config.vhighchm = chopconf.bits.vhighchm != 0;
  config.diss2g = chopconf.bits.diss2g != 0;
  config.diss2vs = chopconf.bits.diss2vs != 0;

  // Mode-specific fields
  bool is_classic_mode = (chopconf.bits.chm != 0);
  config.mode = is_classic_mode ? ChopperMode::CLASSIC : ChopperMode::SPREAD_CYCLE;

  if (is_classic_mode) {
    // Classic mode: reconstruct TFD from hstrt_tfd and tfd_3
    config.tfd = static_cast<uint8_t>(chopconf.bits.hstrt_tfd | (chopconf.bits.tfd_3 << 3));
    config.hend = static_cast<uint8_t>(chopconf.bits.hend_offset); // OFFSET
    config.disfdcc = chopconf.bits.disfdcc != 0;
    config.hstrt = 0; // Not used in Classic mode
  } else {
    // SpreadCycle mode
    config.hstrt = static_cast<uint8_t>(chopconf.bits.hstrt_tfd);
    config.hend = static_cast<uint8_t>(chopconf.bits.hend_offset);
    config.tfd = 0;         // Not used in SpreadCycle mode
    config.disfdcc = false; // Not used in SpreadCycle mode
  }

  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::GetDiag0Config(Diag0Config& config) noexcept {
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
bool TMC51x0<CommType>::MotorControl::SetDiag0Config(const Diag0Config& config) noexcept {
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
bool TMC51x0<CommType>::MotorControl::GetDiag1Config(Diag1Config& config) noexcept {
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
bool TMC51x0<CommType>::MotorControl::SetDiag1Config(const Diag1Config& config) noexcept {
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
bool TMC51x0<CommType>::MotorControl::SetCoilCurrents(int16_t coil_a, int16_t coil_b) noexcept {
  // Constrain to valid 9-bit signed range (-256 to +255)
  // Datasheet recommends ±248 for safe operation in SpreadCycle mode
  coil_a = constrain<int16_t>(coil_a, -256, 255);
  coil_b = constrain<int16_t>(coil_b, -256, 255);

  // Pack coil currents into XTARGET register:
  // - Bits 8..0: Coil A current (signed 9-bit)
  // - Bits 24..16: Coil B current (signed 9-bit)
  // Sign extend 9-bit values to 32-bit signed, then mask to 9 bits
  uint32_t xtarget = 0;

  // Coil A: bits 8..0 (signed 9-bit)
  uint32_t coil_a_unsigned = static_cast<uint32_t>(coil_a) & 0x1FFU; // Mask to 9 bits
  xtarget |= coil_a_unsigned;

  // Coil B: bits 24..16 (signed 9-bit)
  uint32_t coil_b_unsigned = static_cast<uint32_t>(coil_b) & 0x1FFU; // Mask to 9 bits
  xtarget |= (coil_b_unsigned << 16);

  return driver_.comm_.WriteRegister(Registers::XTARGET, xtarget, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetIholdDelayMs(float total_delay_ms) noexcept {
  if (total_delay_ms <= 0.0f) {
    // Setting to 0 or negative = instant power down (IHOLDDELAY = 0)
    // Read-Modify-Write to preserve IRUN and IHOLD
    uint32_t ihold_irun_val = 0;
    if (!driver_.comm_.ReadRegister(Registers::IHOLD_IRUN, ihold_irun_val, driver_.GetCommAddress())) {
      return false;
    }
    IHOLD_IRUN_Register iholdrun{};
    iholdrun.value = ihold_irun_val;
    iholdrun.bits.iholddelay = 0;
    return driver_.comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, driver_.GetCommAddress());
  }

  // Read current IRUN and IHOLD to calculate current reduction steps
  uint32_t ihold_irun_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::IHOLD_IRUN, ihold_irun_val, driver_.GetCommAddress())) {
    return false;
  }
  IHOLD_IRUN_Register iholdrun{};
  iholdrun.value = ihold_irun_val;

  uint8_t current_irun = iholdrun.bits.irun;
  uint8_t current_ihold = iholdrun.bits.ihold;

  // Calculate number of current reduction steps
  uint8_t current_steps = (current_irun > current_ihold) ? (current_irun - current_ihold) : 0;

  if (current_steps == 0) {
    // IRUN == IHOLD, no current reduction steps, delay is always 0
    iholdrun.bits.iholddelay = 0;
    return driver_.comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, driver_.GetCommAddress());
  }

  // Calculate per-step delay from total delay
  float per_step_delay_ms = total_delay_ms / static_cast<float>(current_steps);

  // Calculate IHOLDDELAY register value: delay_value = (per_step_delay_ms * f_clk) / (1000 * 2^18)
  // Where 2^18 = 262144
  float delay_clocks = (per_step_delay_ms * static_cast<float>(driver_.f_clk_)) / (1000.0f * 262144.0f);
  uint8_t iholddelay_value = constrain<uint8_t>(static_cast<uint8_t>(std::round(delay_clocks)), 0U, 15U);

  // Update register
  iholdrun.bits.iholddelay = iholddelay_value;
  return driver_.comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetMicrostepLookupTable(uint8_t index, uint32_t value) noexcept {
  if (index > 7) {
    return false;
  }
  const uint8_t registers[] = {Registers::MSLUT_0, Registers::MSLUT_1, Registers::MSLUT_2, Registers::MSLUT_3,
                               Registers::MSLUT_4, Registers::MSLUT_5, Registers::MSLUT_6, Registers::MSLUT_7};
  return driver_.comm_.WriteRegister(registers[index], value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetMicrostepLookupTableSegmentation(uint8_t width_sel_0, uint8_t width_sel_1,
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
bool TMC51x0<CommType>::MotorControl::SetMicrostepLookupTableStart(uint16_t start_current) noexcept {
  start_current = constrain<decltype(start_current)>(start_current, 0U, 255U);
  return driver_.comm_.WriteRegister(Registers::MSLUTSTART, start_current, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::MotorControl::SetupMotorFromSpec(const MotorSpec& motor_spec,
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
bool TMC51x0<CommType>::Encoder::Configure(const EncoderConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.encoder_config = config;

  // Read-Modify-Write to preserve reserved bits (bits 11-31)
  uint32_t encmode_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::ENCMODE, encmode_value)) {
    return false;
  }
  ENCMODE_Register encmode{};
  encmode.value = encmode_value;

  // N channel active level (shares ReferenceSwitchActiveLevel enum)
  encmode.bits.pol_N = (config.n_channel_active == ReferenceSwitchActiveLevel::ACTIVE_HIGH) ? 1 : 0;

  // A/B polarity requirements for N channel validation
  encmode.bits.pol_A = config.require_a_high ? 1 : 0;
  encmode.bits.pol_B = config.require_b_high ? 1 : 0;
  encmode.bits.ignore_AB = config.ignore_ab_polarity ? 1 : 0;

  // Clear mode (clr_cont and clr_once are mutually exclusive)
  encmode.bits.clr_cont = (config.clear_mode == EncoderClearMode::CONTINUOUS) ? 1 : 0;
  encmode.bits.clr_once = (config.clear_mode == EncoderClearMode::ONCE) ? 1 : 0;

  // N channel sensitivity (edge/level detection)
  // Register uses pos_edge (bit 6) and neg_edge (bit 7) as separate bits
  switch (config.n_sensitivity) {
  case EncoderNSensitivity::ACTIVE_LEVEL:
    encmode.bits.pos_edge = 0;
    encmode.bits.neg_edge = 0;
    break;
  case EncoderNSensitivity::RISING_EDGE:
    encmode.bits.pos_edge = 1;
    encmode.bits.neg_edge = 0;
    break;
  case EncoderNSensitivity::FALLING_EDGE:
    encmode.bits.pos_edge = 0;
    encmode.bits.neg_edge = 1;
    break;
  case EncoderNSensitivity::BOTH_EDGES:
    encmode.bits.pos_edge = 1;
    encmode.bits.neg_edge = 1;
    break;
  }

  // Additional encoder features
  encmode.bits.clr_enc_x = config.clear_enc_x_on_event ? 1 : 0;
  encmode.bits.latch_x_act = config.latch_xactual_with_enc ? 1 : 0;

  // Prescaler mode
  encmode.bits.enc_sel_decimal = (config.prescaler_mode == EncoderPrescalerMode::DECIMAL) ? 1 : 0;
  // Reserved bits (11-31) are preserved from read value

  bool success = driver_.comm_.WriteRegister(Registers::ENCMODE, encmode.value, driver_.GetCommAddress());

  // Set encoder deviation if configured (must be done after Configure to ensure microsteps are set)
  if (success && config.allowed_deviation_steps > 0) {
    success = SetAllowedDeviation(config.allowed_deviation_steps);
  }

  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::GetEncoderConfig(EncoderConfig& config) noexcept {
  uint32_t encmode_val = 0;
  if (!driver_.comm_.ReadRegister(Registers::ENCMODE, encmode_val)) {
    return false;
  }
  ENCMODE_Register encmode{};
  encmode.value = encmode_val;

  // Read N channel active level
  config.n_channel_active =
      (encmode.bits.pol_N != 0) ? ReferenceSwitchActiveLevel::ACTIVE_HIGH : ReferenceSwitchActiveLevel::ACTIVE_LOW;

  // Read A/B polarity requirements
  config.require_a_high = (encmode.bits.pol_A != 0);
  config.require_b_high = (encmode.bits.pol_B != 0);
  config.ignore_ab_polarity = (encmode.bits.ignore_AB != 0);

  // Read clear mode (clr_cont and clr_once are mutually exclusive)
  if (encmode.bits.clr_cont != 0) {
    config.clear_mode = EncoderClearMode::CONTINUOUS;
  } else if (encmode.bits.clr_once != 0) {
    config.clear_mode = EncoderClearMode::ONCE;
  } else {
    config.clear_mode = EncoderClearMode::DISABLED;
  }

  // Read N channel sensitivity (pos_edge=bit6, neg_edge=bit7)
  if (encmode.bits.pos_edge == 0 && encmode.bits.neg_edge == 0) {
    config.n_sensitivity = EncoderNSensitivity::ACTIVE_LEVEL;
  } else if (encmode.bits.pos_edge == 1 && encmode.bits.neg_edge == 0) {
    config.n_sensitivity = EncoderNSensitivity::RISING_EDGE;
  } else if (encmode.bits.pos_edge == 0 && encmode.bits.neg_edge == 1) {
    config.n_sensitivity = EncoderNSensitivity::FALLING_EDGE;
  } else { // pos_edge == 1 && neg_edge == 1
    config.n_sensitivity = EncoderNSensitivity::BOTH_EDGES;
  }

  // Read additional features
  config.clear_enc_x_on_event = (encmode.bits.clr_enc_x != 0);
  config.latch_xactual_with_enc = (encmode.bits.latch_x_act != 0);

  // Read prescaler mode
  config.prescaler_mode =
      (encmode.bits.enc_sel_decimal != 0) ? EncoderPrescalerMode::DECIMAL : EncoderPrescalerMode::BINARY;

  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::SetNChannelActiveLevel(ReferenceSwitchActiveLevel active_level) noexcept {
  EncoderConfig config{};
  if (!GetEncoderConfig(config)) {
    return false;
  }
  config.n_channel_active = active_level;
  return Configure(config);
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::SetNChannelSensitivity(EncoderNSensitivity sensitivity) noexcept {
  EncoderConfig config{};
  if (!GetEncoderConfig(config)) {
    return false;
  }
  config.n_sensitivity = sensitivity;
  return Configure(config);
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::SetClearMode(EncoderClearMode clear_mode) noexcept {
  EncoderConfig config{};
  if (!GetEncoderConfig(config)) {
    return false;
  }
  config.clear_mode = clear_mode;
  return Configure(config);
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::SetPrescalerMode(EncoderPrescalerMode prescaler_mode) noexcept {
  EncoderConfig config{};
  if (!GetEncoderConfig(config)) {
    return false;
  }
  config.prescaler_mode = prescaler_mode;
  return Configure(config);
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::GetPosition(int32_t& position) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::X_ENC, value)) {
    return false;
  }
  position = static_cast<int32_t>(value);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::SetResolution(int32_t motor_steps, int32_t enc_resolution, bool inverted) noexcept {
  // Calculate factor: (motor_steps * microsteps) / enc_resolution
  // Use current microstep setting (may vary: 256, 128, 64, etc.)
  float factor = static_cast<float>(motor_steps * driver_.current_microsteps_) / static_cast<float>(enc_resolution);

  // Check if binary prescaler gives exact match
  auto enc_const_binary = static_cast<int32_t>(factor * 65536.0F);
  if (enc_const_binary * enc_resolution == motor_steps * driver_.current_microsteps_ * 65536) {
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
    uint32_t enc_const_value = static_cast<uint32_t>(enc_const_binary);
    bool success = driver_.comm_.WriteRegister(Registers::ENC_CONST, enc_const_value);
    if (success) {
      driver_.write_only_regs_.enc_const = enc_const_value;
    }
    return success;
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
  bool exact_match = ((static_cast<int32_t>(factor * 10000.0F) * enc_resolution) ==
                      (motor_steps * driver_.current_microsteps_ * 10000));
  uint32_t enc_const_value = static_cast<uint32_t>(enc_const_decimal);
  bool success = driver_.comm_.WriteRegister(Registers::ENC_CONST, enc_const_value);
  if (success) {
    driver_.write_only_regs_.enc_const = enc_const_value;
  }
  return exact_match && success;
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::SetAllowedDeviation(int32_t steps) noexcept {
  // Convert steps to microsteps using current microstep setting
  int32_t deviation = steps * driver_.current_microsteps_;
  deviation = std::min(deviation, static_cast<int32_t>(0xFFFFF)); // 20 bits
  uint32_t deviation_value = static_cast<uint32_t>(deviation);
  bool success = driver_.comm_.WriteRegister(Registers::ENC_DEVIATION, deviation_value);
  if (success) {
    driver_.write_only_regs_.enc_deviation = deviation_value;
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::IsDeviationDetected() noexcept {
  uint32_t enc_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::ENC_STATUS, enc_status_value)) {
    return false;
  }
  ENC_STATUS_Register enc_status{};
  enc_status.value = enc_status_value;
  return enc_status.bits.deviation_warn != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::ClearDeviationFlag() noexcept {
  ENC_STATUS_Register enc_status{};
  enc_status.bits.deviation_warn = true;
  return driver_.comm_.WriteRegister(Registers::ENC_STATUS, enc_status.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::Encoder::GetLatchedPosition(int32_t& position) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::ENC_LATCH, value)) {
    return false;
  }
  position = static_cast<int32_t>(value);
  return true;
}

// Diagnostics implementation
template <typename CommType>
DriverStatus TMC51x0<CommType>::Diagnostics::GetStatus() noexcept {
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
bool TMC51x0<CommType>::Diagnostics::GetGlobalStatus(bool& reset, bool& drv_err, bool& uv_cp) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::GetStallGuard(uint16_t& value) noexcept {
  uint32_t drv_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value, driver_.GetCommAddress())) {
    return false;
  }
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;
  value = static_cast<uint16_t>(drv_status.bits.sg_result);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::GetStallGuardResult(uint16_t& sg_result) noexcept {
  uint32_t drv_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value, driver_.GetCommAddress())) {
    return false;
  }
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;
  sg_result = static_cast<uint16_t>(drv_status.bits.sg_result);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::ConfigureStallGuard(const StallGuardConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.stallguard = config;

  uint32_t coolconf_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::COOLCONF, coolconf_value)) {
    return false;
  }
  COOLCONF_Register coolconf{};
  coolconf.value = coolconf_value;

  // SGT is signed 7-bit (-64 to +63), constrain and mask to 7 bits (bits 22..16)
  auto sgt_signed = static_cast<int8_t>(constrain<int8_t>(config.threshold, -64, 63));
  coolconf.bits.sgt = static_cast<int32_t>(sgt_signed) & 0x7F;

  // Update filter enable
  coolconf.bits.sfilt = config.enable_filter ? 1 : 0;

  // Preserve CoolStep fields (semin, seup, semax, sedn, seimin)
  // They are already in coolconf.value from ReadRegister

  // Write COOLCONF register
  bool success = driver_.comm_.WriteRegister(Registers::COOLCONF, coolconf.value, driver_.GetCommAddress());

  // Configure velocity thresholds if provided
  if (success && config.min_velocity > 0.0F) {
    float steps_per_sec = driver_.convertSpeedToSteps(config.min_velocity, config.velocity_unit);
    int32_t tcoolthrs = driver_.thresholdSpeedToTstep(steps_per_sec);
    tcoolthrs = std::min(tcoolthrs, static_cast<decltype(tcoolthrs)>(0xFFFFF)); // 20 bits
    success &= driver_.comm_.WriteRegister(Registers::TCOOLTHRS, static_cast<uint32_t>(tcoolthrs));
  }

  if (success && config.max_velocity > 0.0F) {
    float steps_per_sec = driver_.convertSpeedToSteps(config.max_velocity, config.velocity_unit);
    int32_t thigh = driver_.thresholdSpeedToTstep(steps_per_sec);
    thigh = std::min(thigh, static_cast<decltype(thigh)>(0xFFFFF)); // 20 bits
    success &= driver_.comm_.WriteRegister(Registers::THIGH, static_cast<uint32_t>(thigh));
  }

  // Configure stop on stall if requested
  if (success && config.stop_on_stall) {
    uint32_t sw_mode_value = 0;
    if (driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_value)) {
      SW_MODE_Register sw_mode{};
      sw_mode.value = sw_mode_value;
      sw_mode.bits.sg_stop = 1; // Enable stop on stall
      success &= driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value, driver_.GetCommAddress());
    }
  }

  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::EnableStopOnStall(bool enable) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::IsStopOnStallEnabled() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, value, driver_.GetCommAddress())) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = value;
  return sw_mode.bits.sg_stop != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::SetSoftStop(bool enable) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, value, driver_.GetCommAddress())) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = value;
  sw_mode.bits.en_softstop = enable ? 1 : 0;
  return driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::IsSoftStopEnabled() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, value, driver_.GetCommAddress())) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = value;
  return sw_mode.bits.en_softstop != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::ClearStallFlag() noexcept {
  RAMP_STAT_Register ramp_stat{};
  ramp_stat.bits.event_stop_sg = 1; // Write 1 to clear
  return driver_.comm_.WriteRegister(Registers::RAMP_STAT, ramp_stat.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::IsStallDetected() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMP_STAT, value)) {
    return false;
  }
  RAMP_STAT_Register ramp_stat{};
  ramp_stat.value = value;
  return ramp_stat.bits.event_stop_sg != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::GetDriverStatusRegister(uint32_t& status) noexcept {
  return driver_.comm_.ReadRegister(Registers::DRV_STATUS, status, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::IsOpenLoadA() noexcept {
  uint32_t drv_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value, driver_.GetCommAddress())) {
    return false;
  }
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;
  return drv_status.bits.ola != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::IsOpenLoadB() noexcept {
  uint32_t drv_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value, driver_.GetCommAddress())) {
    return false;
  }
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;
  return drv_status.bits.olb != 0;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::CheckOpenLoad(bool& phase_a, bool& phase_b) noexcept {
  uint32_t drv_status_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value, driver_.GetCommAddress())) {
    return false;
  }
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;
  phase_a = drv_status.bits.ola != 0;
  phase_b = drv_status.bits.olb != 0;
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::GetRampStatusRegister(uint32_t& status) noexcept {
  return driver_.comm_.ReadRegister(Registers::RAMP_STAT, status, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::ClearRampStatus(uint32_t bits_to_clear) noexcept {
  // RAMP_STAT is read-write-clear: writing 1 to a bit clears it
  return driver_.comm_.WriteRegister(Registers::RAMP_STAT, bits_to_clear, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::SetTcoolthrs(float threshold, Unit unit) noexcept {
  if (threshold < 0.0F) {
    return false;
  }

  float steps_per_sec = driver_.convertSpeedToSteps(threshold, unit);
  int32_t tcoolthrs = driver_.thresholdSpeedToTstep(steps_per_sec);
  tcoolthrs = std::min(tcoolthrs, static_cast<int32_t>(0xFFFFF)); // 20 bits

  bool success =
      driver_.comm_.WriteRegister(Registers::TCOOLTHRS, static_cast<uint32_t>(tcoolthrs), driver_.GetCommAddress());
  if (success) {
    driver_.write_only_regs_.tcoolthrs = static_cast<uint32_t>(tcoolthrs);
  }
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::GetTcoolthrs(float& threshold, Unit unit) const noexcept {
  uint32_t tcoolthrs = driver_.write_only_regs_.tcoolthrs;
  // Convert TSTEP value back to speed: fSTEP = f_CLK / (TSTEP * USC)
  // TCOOLTHRS is in TSTEP units (20 bits), where TSTEP = f_CLK / (fSTEP * USC)
  // So fSTEP = f_CLK / (TCOOLTHRS * USC)
  // For threshold, we use the same conversion as thresholdSpeedToTstep but in reverse
  if (tcoolthrs == 0) {
    threshold = 0.0f; // 0 means disabled (infinite threshold)
  } else {
    float f_clk = static_cast<float>(driver_.f_clk_);
    float current_microsteps = static_cast<float>(driver_.current_microsteps_);
    float steps_per_sec = f_clk / (static_cast<float>(tcoolthrs) * current_microsteps);
    threshold = driver_.convertSpeedToUnit(steps_per_sec, unit);
  }
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::GetLostSteps(uint32_t& steps) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::LOST_STEPS, value)) {
    return false;
  }
  steps = value & 0xFFFFF; // LOST_STEPS is 20 bits
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Homing::CacheCurrentSettings() noexcept {
  cache_.is_valid = false;
  
  // Cache StealthChop state
  cache_.cached_stealthchop_enabled = driver_.motorControl.IsStealthChopEnabled();
  cache_.stealthchop_was_modified = false; // Will be set if we modify it
  
  // Cache SW_MODE register
  uint32_t sw_mode_value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, sw_mode_value)) {
    return false;
  }
  cache_.cached_sw_mode.value = sw_mode_value;
  cache_.sw_mode_was_modified = false; // Will be set if we modify it
  
  // Cache ramp settings - read from registers
  uint32_t ramp_mode_val = 0;
  if (driver_.comm_.ReadRegister(Registers::RAMPMODE, ramp_mode_val)) {
    cache_.cached_ramp_mode = static_cast<RampMode>(ramp_mode_val);
  }
  
  // Read VMAX, AMAX, DMAX, VSTART, VSTOP from registers
  uint32_t vmax_val = 0;
  if (driver_.comm_.ReadRegister(Registers::VMAX, vmax_val)) {
    cache_.cached_max_speed = driver_.speedFromInternal(static_cast<int32_t>(vmax_val));
  }
  
  uint32_t amax_val = 0;
  if (driver_.comm_.ReadRegister(Registers::AMAX, amax_val)) {
    // Convert from internal units (Hz) to steps/s²
    // Internal: accel = (2^24) / (256 * accel_internal)
    // Reverse: accel_internal = (2^24) / (256 * accel)
    // So: accel = (2^24) / (256 * accel_internal)
    if (amax_val > 0) {
      cache_.cached_acceleration = static_cast<float>(16777216.0 / (256.0 * static_cast<double>(amax_val)));
    } else {
      cache_.cached_acceleration = 0.0f;
    }
  }
  
  uint32_t dmax_val = 0;
  if (driver_.comm_.ReadRegister(Registers::DMAX, dmax_val)) {
    // Convert from internal units (Hz) to steps/s²
    if (dmax_val > 0) {
      cache_.cached_deceleration = static_cast<float>(16777216.0 / (256.0 * static_cast<double>(dmax_val)));
    } else {
      cache_.cached_deceleration = 0.0f;
    }
  }
  
  uint32_t vstart_val = 0;
  if (driver_.comm_.ReadRegister(Registers::VSTART, vstart_val)) {
    cache_.cached_vstart = driver_.speedFromInternal(static_cast<int32_t>(vstart_val));
  }
  
  uint32_t vstop_val = 0;
  if (driver_.comm_.ReadRegister(Registers::VSTOP, vstop_val)) {
    cache_.cached_vstop = driver_.speedFromInternal(static_cast<int32_t>(vstop_val));
  }
  
  cache_.ramp_settings_were_modified = false; // Will be set if we modify them
  cache_.is_valid = true;
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Homing::RestoreCachedSettings() noexcept {
  if (!cache_.is_valid) {
    return false;
  }
  
  bool success = true;
  
  // Restore StealthChop state (if it was modified)
  if (cache_.stealthchop_was_modified) {
    success &= driver_.motorControl.SetStealthChopEnabled(cache_.cached_stealthchop_enabled);
  }
  
  // Restore SW_MODE register (if it was modified)
  if (cache_.sw_mode_was_modified) {
    success &= driver_.comm_.WriteRegister(Registers::SW_MODE, cache_.cached_sw_mode.value);
  }
  
  // Restore ramp settings (if they were modified)
  if (cache_.ramp_settings_were_modified) {
    success &= driver_.rampControl.SetRampMode(cache_.cached_ramp_mode);
    success &= driver_.rampControl.SetMaxSpeed(cache_.cached_max_speed);
    success &= driver_.rampControl.SetAcceleration(cache_.cached_acceleration);
    success &= driver_.rampControl.SetDeceleration(cache_.cached_deceleration);
    success &= driver_.rampControl.SetRampSpeeds(cache_.cached_vstart, cache_.cached_vstop, 0.0f, Unit::Steps);
  }
  
  return success;
}

template <typename CommType>
bool TMC51x0<CommType>::Homing::EnsureSpreadCycleForStallGuard() noexcept {
  // Check if StealthChop is enabled
  if (driver_.motorControl.IsStealthChopEnabled()) {
    // Cache the state and disable StealthChop (StallGuard requires SpreadCycle)
    cache_.cached_stealthchop_enabled = true;
    cache_.stealthchop_was_modified = true;
    return driver_.motorControl.SetStealthChopEnabled(false);
  }
  return true; // Already in SpreadCycle mode
}

template <typename CommType>
bool TMC51x0<CommType>::Homing::PerformSensorlessHoming(bool direction, float search_speed,
                                                         int32_t& final_position, uint32_t timeout_ms) noexcept {
  // Cache current settings
  if (!CacheCurrentSettings()) {
    return false;
  }
  
  // Ensure SpreadCycle mode for StallGuard (disable StealthChop if enabled)
  if (!EnsureSpreadCycleForStallGuard()) {
    return false;
  }
  
  // Use existing StallGuard configuration (SGT threshold from motor config)
  // Do NOT reconfigure StallGuard - use the existing SGT value
  
  // Enable StallGuard2 stop in SW_MODE
  SW_MODE_Register sw_mode{};
  sw_mode.value = cache_.cached_sw_mode.value;
  sw_mode.bits.sg_stop = true;      // Enable stop on stall
  sw_mode.bits.en_softstop = false; // Use hard stop for precise homing (per datasheet 12.4)
  if (!driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value)) {
    RestoreCachedSettings();
    return false;
  }
  cache_.sw_mode_was_modified = true;

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
    RestoreCachedSettings();
    return false;
  }
  cache_.ramp_settings_were_modified = true;

  // CRITICAL: Set acceleration/deceleration BEFORE setting speeds
  // Velocity mode requires AMAX > 0 to actually accelerate from VSTART to VMAX
  // Use reasonable acceleration: reach VMAX in ~0.1 seconds
  float acceleration = std::max(search_speed * 10.0f, 50000.0f); // At least 50k steps/s²
  if (!driver_.rampControl.SetAcceleration(acceleration)) {
    RestoreCachedSettings();
    return false;
  }
  if (!driver_.rampControl.SetDeceleration(acceleration)) {
    RestoreCachedSettings();
    return false;
  }

  // Set VSTART > 0 to actually start motion in velocity mode
  // VSTART should be reasonable but less than VMAX - use 10% of search speed or minimum 1000 steps/s
  float vstart_speed = std::max(search_speed * 0.1f, 1000.0f);
  if (!driver_.rampControl.SetRampSpeeds(vstart_speed, 100.0f, 0.0f, Unit::Steps)) {
    RestoreCachedSettings();
    return false;
  }
  if (!driver_.rampControl.SetMaxSpeed(search_speed)) {
    RestoreCachedSettings();
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
  float final_pos_float = 0.0f;
  if (driver_.rampControl.GetCurrentPosition(final_pos_float, Unit::Steps)) {
    final_position = static_cast<int32_t>(final_pos_float);
  }

  // Clear the stall event flag again to be clean
  driver_.comm_.WriteRegister(Registers::RAMP_STAT, clear_stat.value);

  // Restore cached settings before returning
  RestoreCachedSettings();

  return stalled;
}

template <typename CommType>
bool TMC51x0<CommType>::Homing::PerformSwitchHoming(bool direction, float search_speed, float switch_speed,
                                                      int32_t& final_position, bool use_left_switch,
                                                      uint32_t timeout_ms) noexcept {
  // Cache current settings
  if (!CacheCurrentSettings()) {
    return false;
  }
  
  // Complete homing procedure per datasheet section 12.4:
  // Step 1: Make sure switch is not pressed (move away from switch)
  // This is user responsibility - we assume switch is not pressed at start

  // Step 2: Activate position latching and motor stop upon switch event
  SW_MODE_Register sw_mode{};
  sw_mode.value = cache_.cached_sw_mode.value;

  // Configure latching and stop enable based on switch choice
  // Note: This assumes switch is already configured via ConfigureReferenceSwitch
  // We're just enabling latching and stop for the homing procedure
  if (use_left_switch) {
    sw_mode.bits.latch_l_active = true; // Latch on active edge for homing
    sw_mode.bits.stop_l_enable = true;  // Enable stop on left switch
  } else {
    sw_mode.bits.latch_r_active = true; // Latch on active edge for homing
    sw_mode.bits.stop_r_enable = true;  // Enable stop on right switch
  }
  // Use hard stop for precise homing (per datasheet 12.4 recommendation)
  // Hard stop ensures motor stops exactly at switch position (no overshoot)
  sw_mode.bits.en_softstop = false;

  if (!driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value)) {
    RestoreCachedSettings();
    return false;
  }
  cache_.sw_mode_was_modified = true;

  // Step 3: Start motion ramp into direction of switch
  // Move to a more negative position for left switch, more positive for right switch
  RampMode mode = direction ? RampMode::VELOCITY_POS : RampMode::VELOCITY_NEG;
  if (!driver_.rampControl.SetRampMode(mode)) {
    RestoreCachedSettings();
    return false;
  }
  cache_.ramp_settings_were_modified = true;

  // Set acceleration before speed (required for velocity mode)
  float acceleration = std::max(search_speed * 10.0f, 50000.0f); // At least 50k steps/s²
  if (!driver_.rampControl.SetAcceleration(acceleration)) {
    RestoreCachedSettings();
    return false;
  }
  if (!driver_.rampControl.SetDeceleration(acceleration)) {
    RestoreCachedSettings();
    return false;
  }

  if (!driver_.rampControl.SetMaxSpeed(search_speed)) {
    RestoreCachedSettings();
    return false;
  }

  // Step 3 (continued): Wait for switch hit (motor stops automatically)
  bool switch_hit = false;
  uint32_t loops = timeout_ms * 100; // 10ms per loop
  for (uint32_t i = 0; i < loops; i++) {
    uint32_t ramp_stat = 0;
    if (driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
      RAMP_STAT_Register status{};
      status.value = ramp_stat;

      // Check for specific stop event
      if ((use_left_switch && status.bits.event_stop_l) || (!use_left_switch && status.bits.event_stop_r)) {
        switch_hit = true;
        break;
      }
      // Also check vzero if we missed the event flag but stopped
      if (status.bits.vzero &&
          ((use_left_switch && status.bits.status_stop_l) || (!use_left_switch && status.bits.status_stop_r))) {
        switch_hit = true;
        break;
      }
    }
    // Small delay for polling (10ms)
    driver_.comm_.DelayMs(10);
  }

  // Ensure motor is stopped
  driver_.rampControl.Stop();

  if (!switch_hit) {
    // Timeout - disable stop function and return
    if (use_left_switch) {
      sw_mode.bits.stop_l_enable = false;
    } else {
      sw_mode.bits.stop_r_enable = false;
    }
    driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value);
    RestoreCachedSettings();
    return false;
  }

  // Step 4: Wait until motor is in standstill (poll VACTUAL or check vzero/standstill flag)
  // Per datasheet: "Wait until the motor is in standstill again by polling the actual velocity
  // VACTUAL or checking vzero or the standstill flag"
  uint32_t standstill_loops = 0;
  const uint32_t max_standstill_loops = 1000; // 10 seconds max wait
  bool in_standstill = false;

  while (standstill_loops < max_standstill_loops) {
    uint32_t ramp_stat = 0;
    if (driver_.comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
      RAMP_STAT_Register status{};
      status.value = ramp_stat;

      // Check vzero flag (velocity reached zero)
      if (status.bits.vzero) {
        // Also verify VACTUAL is near zero
        float vactual = 0.0f;
        if (driver_.rampControl.GetCurrentSpeed(vactual, Unit::Steps) &&
            std::abs(vactual) < 1.0f) { // Less than 1 step/s
          in_standstill = true;
          break;
        }
      }
    }
    standstill_loops++;
    // Small delay for polling (10ms)
    driver_.comm_.DelayMs(10);
  }

  if (!in_standstill) {
    // Motor didn't reach standstill - disable stop and return error
    if (use_left_switch) {
      sw_mode.bits.stop_l_enable = false;
    } else {
      sw_mode.bits.stop_r_enable = false;
    }
    driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value);
    RestoreCachedSettings();
    return false;
  }

  // Step 5: Switch to hold mode and calculate difference between latched and actual position
  // Per datasheet: "Switch the ramp generator to hold mode and calculate the difference
  // between the latched position and the actual position. For StallGuard based homing or
  // when using hard stop, XACTUAL stops exactly at the home position, so there is no difference (0)."

  // Switch to hold mode
  if (!driver_.rampControl.SetRampMode(RampMode::HOLD)) {
    RestoreCachedSettings();
    return false;
  }

  // Read latched position (XLATCH) - captured at switch hit
  float latched_pos_float = 0.0f;
  int32_t latched_position = 0;
  if (driver_.rampControl.GetLatchedPosition(latched_pos_float, Unit::Steps)) {
    latched_position = static_cast<int32_t>(latched_pos_float);
  }

  // Read actual position (XACTUAL) - current motor position
  float actual_pos_float = 0.0f;
  int32_t actual_position = 0;
  if (driver_.rampControl.GetCurrentPosition(actual_pos_float, Unit::Steps)) {
    actual_position = static_cast<int32_t>(actual_pos_float);
  }

  // Calculate difference
  // With hard stop, XACTUAL stops exactly at switch position, so XLATCH ≈ XACTUAL
  // The difference represents any offset between latched and actual position
  int32_t position_difference = latched_position - actual_position;

  // Step 6: Write the calculated difference into the actual position register
  // Per datasheet: "Write the calculated difference into the actual position register.
  // Now, homing is finished. A move to position 0 will bring back the motor exactly to the switching point."
  //
  // To make the switch position = home (0):
  // - Current: XACTUAL = switch_position, XLATCH = switch_position (captured)
  // - Goal: When motor is at switch, XACTUAL = 0
  // - Solution: Set XACTUAL = XACTUAL - latched_position
  //   This makes: new_XACTUAL = switch_position - switch_position = 0
  //   And latched position becomes: XLATCH - (switch_position - switch_position) = 0
  //
  // With hard stop, difference should be ~0, so this is effectively setting XACTUAL = 0
  int32_t new_position = actual_position - latched_position;
  if (!driver_.rampControl.SetCurrentPosition(static_cast<float>(new_position), Unit::Steps, false)) {
    RestoreCachedSettings();
    return false;
  }

  // Return final position (should be 0 after homing, representing home position)
  float final_pos_float = 0.0f;
  if (driver_.rampControl.GetCurrentPosition(final_pos_float, Unit::Steps)) {
    final_position = static_cast<int32_t>(final_pos_float);
  }

  // Disable stop function to allow moving away from switch
  if (use_left_switch) {
    sw_mode.bits.stop_l_enable = false;
    sw_mode.bits.latch_l_active = false;
  } else {
    sw_mode.bits.stop_r_enable = false;
    sw_mode.bits.latch_r_active = false;
  }
  driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value);

  // Restore cached settings before returning
  RestoreCachedSettings();

  return true;
}

// Communication implementation
template <typename CommType>
bool TMC51x0<CommType>::Communication::ConfigureUartNodeAddress(uint8_t node_address, uint8_t send_delay) noexcept {
  SLAVECONF_Register slaveconf{};
  slaveconf.bits.slaveaddr = node_address & 0xFF; // Address range is 0-254 (8-bit)
  slaveconf.bits.senddelay = constrain<uint8_t>(send_delay, 0U, 15U);

  bool success = driver_.comm_.WriteRegister(Registers::SLAVECONF, slaveconf.value, driver_.GetCommAddress());
  if (success) {
    driver_.write_only_regs_.slaveconf = slaveconf.value;

    // Store send delay locally (SLAVECONF register is write-only)
    driver_.send_delay_ = constrain<uint8_t>(send_delay, 0U, 15U);

    // Update UART node address (node address and slave address are the same)
    driver_.uart_node_address_ = node_address & 0xFF;

    // Update driver config
    driver_.driver_config_.uart_config.node_address = node_address & 0xFF;
    driver_.driver_config_.uart_config.send_delay = constrain<uint8_t>(send_delay, 0U, 15U);
  }

  return success;

  // Update UART interface slave address if using UART
  if (driver_.comm_.GetMode() == CommMode::UART) {
    // Cast to UART interface and update address
    // Note: This requires the interface to be UartCommInterface
    // The user should also call SetUartNodeAddress on the interface directly if needed
  }

  return true;
}

// Protection implementation
template <typename CommType>
bool TMC51x0<CommType>::Protection::ConfigureShortProtection(const PowerStageParameters& config) noexcept {
  // Update driver config (short protection is part of power_stage)
  driver_.driver_config_.power_stage = config;

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
bool TMC51x0<CommType>::Protection::SetShortProtectionLevels(uint8_t s2vs_level, uint8_t s2g_level, uint8_t shortfilter,
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
bool TMC51x0<CommType>::Diagnostics::GetTimeBetweenMicrosteps(uint32_t& time) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::TSTEP, value)) {
    return false;
  }
  time = value; // TSTEP is 20 bits, but we return full 32-bit value
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::GetMicrostepCounter(uint16_t& counter) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::MSCNT, value)) {
    return false;
  }
  counter = static_cast<uint16_t>(value & 0x3FF); // MSCNT is 10 bits
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::GetMicrostepCurrent(int16_t& phase_a, int16_t& phase_b) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::GetPwmScale(uint8_t& pwm_scale_sum, int16_t& pwm_scale_auto) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::GetPwmAuto(uint8_t& pwm_ofs_auto, uint8_t& pwm_grad_auto) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::ReadInputStatus(InputStatus& input_status) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::ReadIcVersion(uint8_t& version) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::ReadGpioPins(uint32_t& io_pins) noexcept {
  return driver_.comm_.ReadRegister(Registers::IO_INPUT_OUTPUT, io_pins, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC51x0<CommType>::Communication::SetClkFreq(const ExternalClockConfig& config) noexcept {
  // Update driver config
  driver_.driver_config_.external_clk_config = config;

  // Calculate internal clock frequency (f_clk_) from external clock configuration
  // If external_clk_config.frequency_hz > 0: use external clock at that frequency
  // If external_clk_config.frequency_hz == 0: use internal clock (12 MHz default)
  if (config.frequency_hz > 0) {
    driver_.f_clk_ = config.frequency_hz;
    // Validate external clock frequency range
    if (driver_.f_clk_ < ClockFreq::MIN_F_CLK || driver_.f_clk_ > ClockFreq::MAX_F_CLK) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 0, "TMC5160",
                        "Invalid external clock frequency: %u Hz (valid range: %u-%u Hz). Using internal 12 MHz clock.",
                        driver_.f_clk_, ClockFreq::MIN_F_CLK, ClockFreq::MAX_F_CLK);
      driver_.f_clk_ = ClockFreq::DEFAULT_F_CLK; // Fallback to internal clock
    }
  } else {
    // Use internal clock (default: 12 MHz)
    driver_.f_clk_ = ClockFreq::DEFAULT_F_CLK;
  }

  // Configure clock source on CLK pin (optional)
  // If SetClkFreq() is not implemented, returns false and assumes internal clock (CLK tied to GND)
  // If SetClkFreq() is implemented:
  //   - Pass frequency > 0 to use external clock at that frequency
  //   - Pass 0 to explicitly use internal clock (CLK pin set to GND)
  // f_clk_ is used for all timing calculations regardless of clock source
  uint32_t clk_freq_to_set = (config.frequency_hz > 0) ? config.frequency_hz : 0;
  if (!driver_.comm_.SetClkFreq(clk_freq_to_set)) {
    if (clk_freq_to_set == 0) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160",
                        "Communication::SetClkFreq: Using internal oscillator (frequency_hz=0, CLK pin should be tied "
                        "to GND, f_clk=%u Hz)",
                        driver_.f_clk_);
    } else {
      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160",
                        "Communication::SetClkFreq: Using internal oscillator (clock control not supported, CLK pin "
                        "tied to GND, f_clk=%u Hz for calculations)",
                        driver_.f_clk_);
      // Clock control not supported, but we still use the configured frequency for calculations
      // If external clock was requested but not supported, we fall back to internal
      if (config.frequency_hz > 0) {
        TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TMC5160",
                          "WARNING: External clock requested (%u Hz) but not supported, using internal 12 MHz clock",
                          config.frequency_hz);
        driver_.f_clk_ = ClockFreq::DEFAULT_F_CLK; // Fallback to internal clock frequency
      }
    }
  } else {
    if (clk_freq_to_set == 0) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TMC5160",
                        "Communication::SetClkFreq: Internal clock enabled (CLK pin set to GND, f_clk=%u Hz)",
                        driver_.f_clk_);
    } else {
      TMC51X0_LOG_DEBUG(
          driver_.comm_, 2, "TMC5160",
          "Communication::SetClkFreq: External clock set to %u Hz on CLK pin (f_clk=%u Hz for calculations)",
          clk_freq_to_set, driver_.f_clk_);
    }
  }

  return true;
}

template <typename CommType>
std::string TMC51x0<CommType>::GetDriverConfigString() const noexcept {
  std::string info;
  info.reserve(2048); // Pre-allocate for efficiency

  info += "=== TMC5160 Driver Debug Information ===\n\n";

  // Initialization status
  info += "Initialization: ";
  info += initialized_ ? "Initialized\n" : "Not Initialized\n";
  info += "Clock Frequency: " + std::to_string(f_clk_) + " Hz\n";
  info += "Daisy Chain Position: " + std::to_string(daisy_chain_position_) + "\n";
  info += "UART Node Address: " + std::to_string(uart_node_address_) + "\n\n";

  // Motor specifications
  info += "--- Motor Specifications ---\n";
  info += "Steps per Rev: " + std::to_string(motor_spec_.steps_per_rev) + "\n";
  info += "Rated Current: " + std::to_string(motor_spec_.rated_current_ma) + " mA\n";
  info += "Run Current: " + std::to_string(motor_spec_.run_current_ma) + " mA\n";
  info += "Hold Current: " + std::to_string(motor_spec_.hold_current_ma) + " mA\n";
  info += "Sense Resistor: " + std::to_string(motor_spec_.sense_resistor_mohm) + " mOhm\n";
  info += "Supply Voltage: " + std::to_string(motor_spec_.supply_voltage_mv) + " mV\n";
  info += "Calculated IRUN: " + std::to_string(calculated_irun_) + "\n";
  info += "Calculated IHOLD: " + std::to_string(calculated_ihold_) + "\n";
  info += "Calculated GLOBAL_SCALER: " + std::to_string(calculated_global_scaler_) + "\n";
  info += "Current Microsteps: " + std::to_string(current_microsteps_) + "\n\n";

  // Mechanical system
  info += "--- Mechanical System ---\n";
  info += "Gear Ratio: " + std::to_string(mechanical_system_.gear_ratio) + "\n";
  info += "System Type: ";
  switch (mechanical_system_.system_type) {
  case MechanicalSystemType::DirectDrive:
    info += "DirectDrive\n";
    break;
  case MechanicalSystemType::LeadScrew:
    info += "LeadScrew\n";
    break;
  case MechanicalSystemType::BeltDrive:
    info += "BeltDrive\n";
    break;
  case MechanicalSystemType::Gearbox:
    info += "Gearbox\n";
    break;
  }
  info += "Lead Screw Pitch: " + std::to_string(mechanical_system_.lead_screw_pitch_mm) + " mm\n";
  info += "\n";

  // Chopper configuration
  info += "--- Chopper Configuration ---\n";
  info +=
      "Mode: " + std::string((driver_config_.chopper.mode == ChopperMode::SPREAD_CYCLE) ? "SpreadCycle" : "Classic") +
      "\n";
  info += "TOFF: " + std::to_string(driver_config_.chopper.toff) + "\n";
  info += "TBL: " + std::to_string(driver_config_.chopper.tbl) + "\n";
  info += "MRES: " + std::to_string(static_cast<uint8_t>(driver_config_.chopper.mres)) + "\n";
  info += "Interpolation: " + std::string(driver_config_.chopper.intpol ? "Enabled" : "Disabled") + "\n";
  info += "\n";

  // Ramp configuration
  info += "--- Ramp Configuration ---\n";
  info += "VSTART: " + std::to_string(driver_config_.ramp_config.vstart) + "\n";
  info += "VSTOP: " + std::to_string(driver_config_.ramp_config.vstop) + "\n";
  info += "VMAX: " + std::to_string(driver_config_.ramp_config.vmax) + "\n";
  info += "V1: " + std::to_string(driver_config_.ramp_config.v1) + "\n";
  info += "AMAX: " + std::to_string(driver_config_.ramp_config.amax) + "\n";
  info += "DMAX: " + std::to_string(driver_config_.ramp_config.dmax) + "\n";
  info += "A1: " + std::to_string(driver_config_.ramp_config.a1) + "\n";
  info += "D1: " + std::to_string(driver_config_.ramp_config.d1) + "\n";
  info += "\n";

  // Write-only register values
  info += "--- Write-Only Register Values ---\n";
  info += "X_COMPARE: 0x" + std::to_string(write_only_regs_.x_compare) + "\n";
  info += "SHORT_CONF: 0x" + std::to_string(write_only_regs_.short_conf) + "\n";
  info += "DRV_CONF: 0x" + std::to_string(write_only_regs_.drv_conf) + "\n";
  info += "GLOBAL_SCALER: " + std::to_string(write_only_regs_.global_scaler) + "\n";
  info += "IHOLD_IRUN: 0x" + std::to_string(write_only_regs_.ihold_irun) + "\n";
  info += "TPOWERDOWN: " + std::to_string(write_only_regs_.tpowerdown) + "\n";
  info += "TPWMTHRS: " + std::to_string(write_only_regs_.tpwmthrs) + "\n";
  info += "TCOOLTHRS: " + std::to_string(write_only_regs_.tcoolthrs) + "\n";
  info += "THIGH: " + std::to_string(write_only_regs_.thigh) + "\n";
  info += "VSTART: " + std::to_string(write_only_regs_.vstart) + "\n";
  info += "A_1: " + std::to_string(write_only_regs_.a_1) + "\n";
  info += "V_1: " + std::to_string(write_only_regs_.v_1) + "\n";
  info += "AMAX: " + std::to_string(write_only_regs_.amax) + "\n";
  info += "VMAX: " + std::to_string(write_only_regs_.vmax) + "\n";
  info += "DMAX: " + std::to_string(write_only_regs_.dmax) + "\n";
  info += "D_1: " + std::to_string(write_only_regs_.d_1) + "\n";
  info += "VSTOP: " + std::to_string(write_only_regs_.vstop) + "\n";
  info += "TZEROWAIT: " + std::to_string(write_only_regs_.tzerowait) + "\n";
  info += "VDCMIN: " + std::to_string(write_only_regs_.vdcmin) + "\n";
  info += "ENC_CONST: 0x" + std::to_string(write_only_regs_.enc_const) + "\n";
  info += "ENC_DEVIATION: " + std::to_string(write_only_regs_.enc_deviation) + "\n";
  info += "COOLCONF: 0x" + std::to_string(write_only_regs_.coolconf) + "\n";
  info += "DCCTRL: 0x" + std::to_string(write_only_regs_.dcctrl) + "\n";
  info += "PWMCONF: 0x" + std::to_string(write_only_regs_.pwmconf) + "\n";
  info += "SLAVECONF: 0x" + std::to_string(write_only_regs_.slaveconf) + "\n";
  info += "\n";

  return info;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::ReadFactoryConfig(uint8_t& fclktrim) noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::FACTORY_CONF, value)) {
    return false;
  }
  // FACTORY_CONF contains FCLKTRIM in bits 0-4
  fclktrim = static_cast<uint8_t>(value & 0x1F);
  return true;
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::SetSdoCfg0Polarity(bool polarity) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::ReadOtpConfig(uint8_t& otp_fclktrim, bool& otp_s2_level, bool& otp_bbm,
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
uint8_t TMC51x0<CommType>::Diagnostics::GetUartTransmissionCount() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::IFCNT, value)) {
    return 0;
  }
  return static_cast<uint8_t>(value & 0xFF); // IFCNT is 8 bits
}

template <typename CommType>
bool TMC51x0<CommType>::Diagnostics::ReadOffsetCalibration(uint8_t& phase_a, uint8_t& phase_b) noexcept {
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
bool TMC51x0<CommType>::Diagnostics::VerifySetup() noexcept {
  TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "--- TMC5160 Setup Verification ---");

  // 1. Check IC Version
  uint8_t version = 0;
  if (ReadIcVersion(version)) {
    if (version == 0x30) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "IC Version: 0x30 (Matches TMC5160)");
    } else {
      TMC51X0_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "IC Version MISMATCH: 0x%02X (Expected 0x30)", version);
      // If version is 0x00 or 0xFF, it's likely a communication error
      if (version == 0x00 || version == 0xFF) {
        TMC51X0_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "CRITICAL: Bus communication likely failed!");
        return false;
      }
    }
  } else {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "Failed to read IC Version!");
    return false;
  }

  // 2. Check Input Pins (Reg 0x04)
  InputStatus inputs;
  if (ReadInputStatus(inputs)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "--- Input Pins (IOIN 0x04) ---");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "REFL_STEP:      %s", inputs.refl_step ? "HIGH" : "LOW");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "REFR_DIR:       %s", inputs.refr_dir ? "HIGH" : "LOW");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "ENCB_DCEN_CFG4: %s", inputs.encb_dcen_cfg4 ? "HIGH" : "LOW");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "ENCA_DCIN_CFG5: %s", inputs.enca_dcin_cfg5 ? "HIGH" : "LOW");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "DRV_ENN:        %s (Active LOW)",
                      inputs.drv_enn ? "HIGH (Disabled)" : "LOW (Enabled)");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "ENC_N_DCO_CFG6: %s", inputs.enc_n_dco_cfg6 ? "HIGH" : "LOW");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "SD_MODE:        %s %s", inputs.sd_mode ? "HIGH" : "LOW",
                      inputs.sd_mode ? "(External Step/Dir)" : "(Internal Ramp)");
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "SWCOMP_IN:      %s", inputs.swcomp_in ? "HIGH" : "LOW");

    // Warn about configuration mismatches
    if (inputs.drv_enn) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup",
                        "NOTE: DRV_ENN is HIGH. Driver power stage is currently DISABLED.");
    }
  } else {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "VerifySetup", "Failed to read Input Status!");
  }

  TMC51X0_LOG_DEBUG(driver_.comm_, 1, "VerifySetup", "----------------------------------");
  return true;
}

// Helper function to test if a velocity works with a given SGT
template <typename CommType>
bool TestVelocityWithSGT(TMC51x0<CommType>& driver, float velocity_steps, int8_t sgt,
                                 uint16_t& sg_result_out, int sample_count = 8) {
  // Configure SGT
  StallGuardConfig sg_config{};
  sg_config.threshold = sgt;
  sg_config.enable_filter = false;
  if (!driver.diagnostics.ConfigureStallGuard(sg_config)) {
    return false;
  }
  driver.GetComm().DelayMs(10);

  // Set velocity
  if (!driver.rampControl.SetMaxSpeed(velocity_steps, Unit::Steps)) {
    return false;
  }

  // Wait for velocity to stabilize
  for (int i = 0; i < 100; i++) { // 100 * 10ms = 1000ms timeout
    float current_speed = 0.0f;
    if (driver.rampControl.GetCurrentSpeed(current_speed, Unit::Steps) &&
        std::abs(current_speed - velocity_steps) < 50.0f) {
      break;
    }
    driver.GetComm().DelayMs(10);
  }

  // Sample SG_RESULT
  uint16_t sg_sum = 0;
  uint16_t sg_count = 0;
  bool stall_detected = false;

  for (int i = 0; i < sample_count; i++) {
    uint16_t sg_val = 0;
    if (driver.diagnostics.GetStallGuard(sg_val)) {
      if (sg_val == 0) {
        stall_detected = true;
        break;
      }
      sg_sum += sg_val;
      sg_count++;
    }
    driver.GetComm().DelayMs(5);
  }

  if (stall_detected) {
    sg_result_out = 0;
    return false;
  }

  sg_result_out = (sg_count > 0) ? (sg_sum / sg_count) : 0;
  return true;
}

// Helper function to find working velocity range for a given SGT
template <typename CommType>
bool FindWorkingVelocityRange(TMC51x0<CommType>& driver, int8_t sgt, float start_velocity,
                                     float direction, float& found_velocity, uint16_t& sg_result_out,
                                     float min_velocity = 100.0f, float max_velocity = 100000.0f) {
  float test_velocity = start_velocity;
  const float step_size = (direction > 0) ? 1000.0f : -1000.0f; // 1000 steps/s steps
  const int max_iterations = 50;

  for (int i = 0; i < max_iterations; i++) {
    // Clamp to reasonable range
    if (test_velocity < min_velocity) {
      test_velocity = min_velocity;
    }
    if (test_velocity > max_velocity) {
      test_velocity = max_velocity;
    }

    if (TestVelocityWithSGT(driver, test_velocity, sgt, sg_result_out)) {
      found_velocity = test_velocity;
      return true;
    }

    test_velocity += step_size;

    // Check if we've gone out of bounds
    if ((direction > 0 && test_velocity > max_velocity) ||
        (direction < 0 && test_velocity < min_velocity)) {
      break;
    }
  }

  return false;
}

template <typename CommType>
bool TMC51x0<CommType>::Tuning::TuneStallGuard(float target_velocity, StallGuardTuningResult& result,
                                                    int8_t min_sgt, int8_t max_sgt, float acceleration,
                                                    float min_velocity, float max_velocity,
                                                    Unit velocity_unit) noexcept {
  // Initialize result
  result = StallGuardTuningResult{};

  // Convert inputs to steps
  float target_v_steps = driver_.convertSpeedToSteps(target_velocity, velocity_unit);
  float min_v_steps = driver_.convertSpeedToSteps(min_velocity, velocity_unit);
  float max_v_steps = driver_.convertSpeedToSteps(max_velocity, velocity_unit);
  float accel_steps = driver_.convertAccelerationToSteps(acceleration, velocity_unit);

  TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                    "Starting comprehensive SGT tuning. Target=%.2f, Min=%.2f, Max=%.2f steps/s",
                    target_v_steps, min_v_steps, max_v_steps);

  // 0. CRITICAL: Ensure stop-on-stall is DISABLED
  if (!driver_.diagnostics.EnableStopOnStall(false)) {
    return false;
  }

  // Clear any previous stop events/flags
  driver_.diagnostics.ClearStallFlag();
  driver_.comm_.DelayMs(10);

  // 1. Start with SGT=0 (datasheet recommendation) or min_sgt if higher
  int8_t current_sgt = (min_sgt < 0) ? 0 : min_sgt;
  if (current_sgt != min_sgt) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                      "Adjusted starting SGT from %d to %d (avoiding false stalls)", min_sgt, current_sgt);
  }

  // Enable velocity mode for continuous motion during tuning
  if (!driver_.rampControl.SetRampMode(RampMode::VELOCITY_POS)) {
    return false;
  }

  // Set explicit acceleration
  if (!driver_.rampControl.SetAccelerations(accel_steps, accel_steps, Unit::Steps)) {
    return false;
  }

  // Start motion at target velocity
  if (!driver_.rampControl.SetMaxSpeed(target_v_steps, Unit::Steps)) {
    return false;
  }

  // Wait for motor to reach target speed
  bool velocity_reached = false;
  for (int i = 0; i < 500; i++) { // 500 * 10ms = 5000ms timeout
    if (driver_.rampControl.IsTargetVelocityReached()) {
      velocity_reached = true;
      break;
    }
    float current_speed = 0.0f;
    if (driver_.rampControl.GetCurrentSpeed(current_speed, Unit::Steps) &&
        std::abs(current_speed - target_v_steps) < 100.0f) {
      velocity_reached = true;
      break;
    }
    driver_.comm_.DelayMs(10);
  }

  if (!velocity_reached) {
    float current_speed_debug = 0.0f;
    driver_.rampControl.GetCurrentSpeed(current_speed_debug, Unit::Steps);
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard",
                      "Warning: Target velocity not reached before tuning (V=%.1f)", current_speed_debug);
  }

  // 2. PRIMARY GOAL: Find optimal SGT at target velocity (most important)
  TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                    "Phase 1: Finding optimal SGT at target velocity %.2f steps/s", target_v_steps);

  // First, find the range of working SGT values
  int8_t min_working_sgt = -1;
  int8_t max_working_sgt = -1;
  int8_t best_sgt = -1;
  uint16_t best_sg_result = 0;
  const uint16_t target_sg_result = 200; // Target SG_RESULT for optimal sensitivity (100-500 range is good)
  uint16_t best_sg_diff = 1023; // Difference from target

  // Scan through SGT range to find all working values
  while (current_sgt <= max_sgt) {
    // Update SGT
    StallGuardConfig sg_config{};
    sg_config.threshold = current_sgt;
    sg_config.enable_filter = false; // Disable filter during tuning
    if (!driver_.diagnostics.ConfigureStallGuard(sg_config)) {
      driver_.rampControl.Stop();
      return false;
    }

    driver_.comm_.DelayMs(10);

    // Sample SG_RESULT multiple times at target velocity
    bool stall_indicated = false;
    uint16_t sg_sum = 0;
    uint16_t sg_count = 0;

    for (int i = 0; i < 8; i++) {
      uint16_t sg_val = 0;
      if (driver_.diagnostics.GetStallGuard(sg_val)) {
        if (sg_val == 0) {
          stall_indicated = true;
          break;
        }
        sg_sum += sg_val;
        sg_count++;
      }
      driver_.comm_.DelayMs(5);
    }

    if (stall_indicated) {
      // Check if motor stopped unexpectedly
      float vact = 0.0f;
      if (driver_.rampControl.GetCurrentSpeed(vact, Unit::Steps) && std::abs(vact) < 10.0f) {
        TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard",
                          "Motor stopped during tuning! (V=%.1f). Restarting...", vact);
        driver_.diagnostics.EnableStopOnStall(false);
        driver_.diagnostics.ClearStallFlag();
        driver_.rampControl.SetMaxSpeed(target_v_steps, Unit::Steps);
        driver_.comm_.DelayMs(200);
      }

      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TuneStallGuard", 
                        "SGT %d too low at target velocity (SG=0), increasing...", current_sgt);
      current_sgt++;
    } else {
      // This SGT works - record it
      uint16_t avg_sg_result = (sg_count > 0) ? (sg_sum / sg_count) : 0;
      
      if (min_working_sgt == -1) {
        min_working_sgt = current_sgt; // First working SGT
      }
      max_working_sgt = current_sgt; // Last working SGT so far

      // Calculate how close this SG_RESULT is to our target (200 is ideal for good sensitivity)
      uint16_t sg_diff = (avg_sg_result > target_sg_result) ? 
                         (avg_sg_result - target_sg_result) : 
                         (target_sg_result - avg_sg_result);

      // Prefer SG_RESULT in range 100-500 (good sensitivity range)
      // Closer to 200 is better, but anything in 100-500 is acceptable
      if (avg_sg_result >= 100 && avg_sg_result <= 500) {
        if (sg_diff < best_sg_diff) {
          best_sgt = current_sgt;
          best_sg_result = avg_sg_result;
          best_sg_diff = sg_diff;
        }
      } else if (best_sgt == -1) {
        // If no SGT in ideal range yet, use this one as fallback
        best_sgt = current_sgt;
        best_sg_result = avg_sg_result;
        best_sg_diff = sg_diff;
      }

      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "TuneStallGuard", 
                        "SGT %d works: SG_RESULT=%u (diff from target=%u)", 
                        current_sgt, avg_sg_result, sg_diff);

      current_sgt++;
    }
  }

  // Determine optimal SGT
  if (min_working_sgt == -1) {
    // No working SGT found
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "TuneStallGuard", 
                      "Failed to find any working SGT at target velocity. Reached max SGT %d.", max_sgt);
    driver_.rampControl.Stop();
    return false;
  }

  // If we found a best SGT (in ideal range), use it; otherwise use middle of working range
  if (best_sgt != -1) {
    result.optimal_sgt = best_sgt;
    result.target_velocity_sg_result = best_sg_result;
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                      "Optimal SGT found: %d (SG_RESULT=%u, in ideal range 100-500)", 
                      best_sgt, best_sg_result);
  } else {
    // Use middle of working range for stability
    result.optimal_sgt = (min_working_sgt + max_working_sgt) / 2;
    // Re-test to get actual SG_RESULT at optimal SGT
    StallGuardConfig sg_config{};
    sg_config.threshold = result.optimal_sgt;
    sg_config.enable_filter = false;
    driver_.diagnostics.ConfigureStallGuard(sg_config);
    driver_.comm_.DelayMs(10);
    
    uint16_t sg_sum = 0;
    uint16_t sg_count = 0;
    for (int i = 0; i < 8; i++) {
      uint16_t sg_val = 0;
      if (driver_.diagnostics.GetStallGuard(sg_val) && sg_val > 0) {
        sg_sum += sg_val;
        sg_count++;
      }
      driver_.comm_.DelayMs(5);
    }
    result.target_velocity_sg_result = (sg_count > 0) ? (sg_sum / sg_count) : 0;
    
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                      "Optimal SGT found: %d (middle of working range %d-%d, SG_RESULT=%u)", 
                      result.optimal_sgt, min_working_sgt, max_working_sgt, result.target_velocity_sg_result);
  }

  result.tuning_success = true;

  // 3. SECONDARY GOAL: Verify optimal SGT works at min/max velocities (if specified)
  if (min_v_steps > 0.0f || max_v_steps > 0.0f) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                      "Phase 2: Verifying optimal SGT %d at min/max velocities", result.optimal_sgt);

    // Test at min velocity
    if (min_v_steps > 0.0f) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                        "Testing min velocity %.2f steps/s with SGT %d", min_v_steps, result.optimal_sgt);

      if (TestVelocityWithSGT(driver_, min_v_steps, result.optimal_sgt, result.min_velocity_sg_result)) {
        result.min_velocity_success = true;
        result.min_velocity_sgt = result.optimal_sgt;
        TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                          "Min velocity works! SG_RESULT=%u", result.min_velocity_sg_result);
      } else {
        TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                          "Min velocity %.2f does NOT work with optimal SGT %d. Finding working min velocity...",
                          min_v_steps, result.optimal_sgt);

        // Try to find a working min velocity (search downward from target)
        float found_min_vel = 0.0f;
        if (FindWorkingVelocityRange(driver_, result.optimal_sgt, target_v_steps, -1.0f, 
                                     found_min_vel, result.min_velocity_sg_result, 100.0f, target_v_steps)) {
          result.actual_min_velocity = driver_.convertSpeedToUnit(found_min_vel, velocity_unit);
          TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                            "Found working min velocity: %.2f (SG_RESULT=%u)", 
                            result.actual_min_velocity, result.min_velocity_sg_result);
        } else {
          TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                            "Could not find working min velocity with optimal SGT");
        }
      }
    }

    // Test at max velocity
    if (max_v_steps > 0.0f) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                        "Testing max velocity %.2f steps/s with SGT %d", max_v_steps, result.optimal_sgt);

      if (TestVelocityWithSGT(driver_, max_v_steps, result.optimal_sgt, result.max_velocity_sg_result)) {
        result.max_velocity_success = true;
        result.max_velocity_sgt = result.optimal_sgt;
        TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                          "Max velocity works! SG_RESULT=%u", result.max_velocity_sg_result);
      } else {
        TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                          "Max velocity %.2f does NOT work with optimal SGT %d. Finding working max velocity...",
                          max_v_steps, result.optimal_sgt);

        // Try to find a working max velocity (search upward from target)
        float found_max_vel = 0.0f;
        if (FindWorkingVelocityRange(driver_, result.optimal_sgt, target_v_steps, 1.0f, 
                                     found_max_vel, result.max_velocity_sg_result, target_v_steps, 100000.0f)) {
          result.actual_max_velocity = driver_.convertSpeedToUnit(found_max_vel, velocity_unit);
          TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                            "Found working max velocity: %.2f (SG_RESULT=%u)", 
                            result.actual_max_velocity, result.max_velocity_sg_result);
        } else {
          TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                            "Could not find working max velocity with optimal SGT");
        }
      }
    }
  }

  // Stop motor
  driver_.rampControl.Stop();

  TMC51X0_LOG_DEBUG(driver_.comm_, 1, "TuneStallGuard", 
                    "Tuning complete. Optimal SGT=%d, Min success=%d, Max success=%d",
                    result.optimal_sgt, result.min_velocity_success, result.max_velocity_success);

  return true;
}

// Legacy overload for backward compatibility
template <typename CommType>
bool TMC51x0<CommType>::Tuning::TuneStallGuard(float target_velocity, int8_t& final_sgt, int8_t min_sgt,
                                                    int8_t max_sgt, float acceleration, float min_velocity,
                                                    float max_velocity, Unit velocity_unit) noexcept {
  // Call the new comprehensive function and extract just the optimal SGT
  StallGuardTuningResult result;
  bool success = TuneStallGuard(target_velocity, result, min_sgt, max_sgt, acceleration,
                                  min_velocity, max_velocity, velocity_unit);
  
  if (success) {
    final_sgt = result.optimal_sgt;
  }
  
  return success;
}

// AutoTuneStallGuard implementation
template <typename CommType>
bool TMC51x0<CommType>::Tuning::AutoTuneStallGuard(float target_velocity, StallGuardTuningResult& result,
                                                     int8_t min_sgt, int8_t max_sgt, float acceleration,
                                                     float min_velocity, float max_velocity, Unit velocity_unit,
                                                     uint16_t safe_current_margin_mA) noexcept {
  // Initialize result
  result = StallGuardTuningResult{};

  TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                    "Starting comprehensive StallGuard tuning with current margin=%u mA",
                    safe_current_margin_mA);

  // ========================================================================
  // STEP 1: Save current motor settings
  // ========================================================================
  struct SavedSettings {
    uint8_t saved_irun{0};
    uint8_t saved_ihold{0};
    uint16_t saved_global_scaler{0};
    CoolStepConfig saved_coolstep{};
    StallGuardConfig saved_stallguard{};
    bool settings_saved{false};
  } saved;

  // Read current IRUN and IHOLD
  uint32_t ihold_irun_val = 0;
  if (driver_.comm_.ReadRegister(Registers::IHOLD_IRUN, ihold_irun_val, driver_.GetCommAddress())) {
    IHOLD_IRUN_Register iholdrun{};
    iholdrun.value = ihold_irun_val;
    saved.saved_irun = iholdrun.bits.irun;
    saved.saved_ihold = iholdrun.bits.ihold;
  } else {
    // Fallback: use calculated values if available
    saved.saved_irun = driver_.calculated_irun_;
    saved.saved_ihold = driver_.calculated_ihold_;
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                      "Could not read IHOLD_IRUN, using calculated values: IRUN=%u, IHOLD=%u",
                      saved.saved_irun, saved.saved_ihold);
  }

  // Read current GLOBAL_SCALER (TMC5160 only)
  if (driver_.chip_version_ != ChipVersion::TMC5130) {
    uint32_t global_scaler_val = 0;
    if (driver_.comm_.ReadRegister(Registers::GLOBAL_SCALER, global_scaler_val, driver_.GetCommAddress())) {
      saved.saved_global_scaler = static_cast<uint16_t>(global_scaler_val & 0xFF);
      if (saved.saved_global_scaler == 0) {
        saved.saved_global_scaler = 256; // 0 means 256
      }
    } else {
      saved.saved_global_scaler = driver_.calculated_global_scaler_;
      TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                        "Could not read GLOBAL_SCALER, using calculated value: %u",
                        saved.saved_global_scaler);
    }
  }

  // Save current CoolStep config
  saved.saved_coolstep = driver_.driver_config_.coolstep;

  // Save current StallGuard config
  saved.saved_stallguard = driver_.driver_config_.stallguard;

  saved.settings_saved = true;
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "AutoTuneStallGuard",
                    "Saved settings: IRUN=%u, IHOLD=%u, GLOBAL_SCALER=%u",
                    saved.saved_irun, saved.saved_ihold, saved.saved_global_scaler);

  // ========================================================================
  // STEP 2: Apply safe current margin if specified
  // ========================================================================
  bool current_was_reduced = false;
  if (safe_current_margin_mA > 0) {
    // Calculate current RMS from saved settings
    // I_RMS = (GLOBAL_SCALER/256) * ((IRUN+1)/32) * (VFS/RSENSE) * (1/√2)
    constexpr float VFS = 0.325F;
    constexpr float SQRT2 = 1.41421356237F;
    float rsense_ohm = static_cast<float>(driver_.motor_spec_.sense_resistor_mohm) / 1000.0F;

    if (rsense_ohm > 0.0F && driver_.motor_spec_.sense_resistor_mohm > 0) {
      float current_rms_a = (static_cast<float>(saved.saved_global_scaler) / 256.0F) *
                            (static_cast<float>(saved.saved_irun + 1) / 32.0F) * (VFS / rsense_ohm) / SQRT2;
      uint16_t current_rms_ma = static_cast<uint16_t>(current_rms_a * 1000.0F);

      // Calculate new current with margin
      uint16_t new_current_ma = (current_rms_ma > safe_current_margin_mA) ?
                                 (current_rms_ma - safe_current_margin_mA) : 0;

      // Ensure minimum current (at least 100mA or 20% of original, whichever is higher)
      uint16_t min_current_ma = std::max(static_cast<uint16_t>(100U),
                                          static_cast<uint16_t>(current_rms_ma * 0.2F));
      if (new_current_ma < min_current_ma) {
        new_current_ma = min_current_ma;
        TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                          "Current margin would reduce current too low, using minimum: %u mA",
                          new_current_ma);
      }

      if (new_current_ma > 0 && new_current_ma < current_rms_ma) {
        // Calculate new IRUN and GLOBAL_SCALER from reduced current
        uint8_t new_irun = 0;
        uint8_t new_ihold = 0;
        uint16_t new_scaler = 0;

        // Calculate hold current proportionally from original hold current
        // First calculate original hold current RMS
        float original_hold_rms_a = (static_cast<float>(saved.saved_global_scaler) / 256.0F) *
                                    (static_cast<float>(saved.saved_ihold + 1) / 32.0F) * (VFS / rsense_ohm) / SQRT2;
        uint16_t original_hold_ma = static_cast<uint16_t>(original_hold_rms_a * 1000.0F);
        
        // Scale hold current proportionally to run current reduction
        float current_ratio = static_cast<float>(new_current_ma) / static_cast<float>(current_rms_ma);
        uint16_t new_hold_current_ma = static_cast<uint16_t>(static_cast<float>(original_hold_ma) * current_ratio);

        if (CalculateMotorCurrent(driver_.motor_spec_, driver_.motor_spec_.sense_resistor_mohm,
                                   driver_.motor_spec_.supply_voltage_mv, new_current_ma, new_hold_current_ma,
                                   new_irun, new_ihold, new_scaler)) {
          // Ensure minimum IRUN=8 for StealthChop compatibility
          if (new_irun < 8) {
            new_irun = 8;
            // Recalculate scaler for IRUN=8
            float run_current_a = static_cast<float>(new_current_ma) / 1000.0F;
            float scaler_float = (run_current_a * 256.0F * 32.0F) / (9.0F * (VFS / rsense_ohm) / SQRT2);
            new_scaler = static_cast<uint16_t>(std::round(scaler_float));
            new_scaler = std::max<uint16_t>(new_scaler, 32U);
            new_scaler = std::min<uint16_t>(new_scaler, 256U);
          }

          // Apply new current settings
          if (driver_.chip_version_ != ChipVersion::TMC5130) {
            if (!driver_.motorControl.SetGlobalScaler(new_scaler)) {
              TMC51X0_LOG_DEBUG(driver_.comm_, 0, "AutoTuneStallGuard",
                                "Failed to set GLOBAL_SCALER for current margin");
              // Continue anyway - might still work
            }
          }

          if (!driver_.motorControl.SetCurrent(new_irun, new_ihold)) {
            TMC51X0_LOG_DEBUG(driver_.comm_, 0, "AutoTuneStallGuard",
                              "Failed to set current for current margin");
            // Continue anyway - might still work
          } else {
            current_was_reduced = true;
            TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                              "Reduced current: %u mA -> %u mA (IRUN: %u->%u, SCALER: %u->%u)",
                              current_rms_ma, new_current_ma, saved.saved_irun, new_irun,
                              saved.saved_global_scaler, new_scaler);
          }
        } else {
          TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                            "Could not calculate new current settings, using original current");
        }
      }
    } else {
      TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                        "Cannot apply current margin: sense resistor not configured");
    }
  }

  // ========================================================================
  // STEP 3: Disable CoolStep (SGMIN=0)
  // ========================================================================
  CoolStepConfig coolstep_disabled{};
  coolstep_disabled.lower_threshold_sg = 0; // Disable CoolStep
  coolstep_disabled.enable_filter = false;   // Disable filter during tuning
  if (!driver_.motorControl.ConfigureCoolStep(coolstep_disabled)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "AutoTuneStallGuard",
                      "Failed to disable CoolStep");
    // Continue anyway
  } else {
    TMC51X0_LOG_DEBUG(driver_.comm_, 2, "AutoTuneStallGuard", "CoolStep disabled for tuning");
  }

  // ========================================================================
  // STEP 4: Explicitly disable StallGuard filter (SFILT=0) for tuning
  // ========================================================================
  // Configure StallGuard with filter disabled (TuneStallGuard will also do this, but
  // we do it here to ensure it's set before any tuning operations)
  StallGuardConfig sg_config_no_filter{};
  sg_config_no_filter.threshold = 0; // Temporary, will be set during tuning
  sg_config_no_filter.enable_filter = false; // Disable filter for immediate response
  if (!driver_.diagnostics.ConfigureStallGuard(sg_config_no_filter)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                      "Failed to configure StallGuard filter (non-critical)");
    // Continue anyway - TuneStallGuard will set it
  }

  // ========================================================================
  // STEP 5: Disable stop-on-stall and clear stall flags
  // ========================================================================
  if (!driver_.diagnostics.EnableStopOnStall(false)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "AutoTuneStallGuard",
                      "Failed to disable stop-on-stall");
    // Continue anyway
  }
  driver_.diagnostics.ClearStallFlag();
  driver_.comm_.DelayMs(10);

  // ========================================================================
  // STEP 6: Perform comprehensive SGT tuning using existing function
  // ========================================================================
  // Adjust min_sgt to start at 0 if negative (to avoid false stalls)
  int8_t adjusted_min_sgt = (min_sgt < 0) ? 0 : min_sgt;
  if (adjusted_min_sgt != min_sgt) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                      "Adjusted min_sgt from %d to %d (avoiding false stalls)", min_sgt, adjusted_min_sgt);
  }

  bool tuning_success = TuneStallGuard(target_velocity, result, adjusted_min_sgt, max_sgt,
                                       acceleration, min_velocity, max_velocity, velocity_unit);

  // ========================================================================
  // STEP 7: Restore all saved settings
  // ========================================================================
  bool restore_success = true;

  // Restore motor current if it was reduced
  if (current_was_reduced && saved.settings_saved) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 2, "AutoTuneStallGuard",
                      "Restoring motor current: IRUN=%u, IHOLD=%u, GLOBAL_SCALER=%u",
                      saved.saved_irun, saved.saved_ihold, saved.saved_global_scaler);

    if (driver_.chip_version_ != ChipVersion::TMC5130) {
      if (!driver_.motorControl.SetGlobalScaler(saved.saved_global_scaler)) {
        TMC51X0_LOG_DEBUG(driver_.comm_, 0, "AutoTuneStallGuard",
                          "Failed to restore GLOBAL_SCALER");
        restore_success = false;
      }
    }

    if (!driver_.motorControl.SetCurrent(saved.saved_irun, saved.saved_ihold)) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 0, "AutoTuneStallGuard",
                        "Failed to restore motor current");
      restore_success = false;
    } else {
      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "AutoTuneStallGuard",
                        "Motor current restored successfully");
    }
  }

  // Restore CoolStep configuration
  if (saved.settings_saved) {
    if (!driver_.motorControl.ConfigureCoolStep(saved.saved_coolstep)) {
      TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                        "Failed to restore CoolStep configuration (non-critical)");
      // Non-critical, continue
    } else {
      TMC51X0_LOG_DEBUG(driver_.comm_, 2, "AutoTuneStallGuard",
                        "CoolStep configuration restored");
    }
  }

  // Note: StallGuard configuration (SGT) is intentionally NOT restored
  // because the optimal SGT found during tuning should remain active.
  // The user can configure it explicitly if needed.

  if (!restore_success) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "AutoTuneStallGuard",
                      "Warning: Some settings could not be restored");
  }

  TMC51X0_LOG_DEBUG(driver_.comm_, 1, "AutoTuneStallGuard",
                    "Tuning complete. Success=%d, Optimal SGT=%d",
                    tuning_success, result.optimal_sgt);

  return tuning_success;
}

// UartConfig implementation
template <typename CommType>
bool TMC51x0<CommType>::UartConfig::ConfigureUartNodeAddress(uint8_t node_address, uint8_t send_delay) noexcept {
  // During sequential programming, the device is accessible at address 0
  // (first device: NAI=GND, subsequent devices: previous NAO=LOW)
  // We need to use address 0 to communicate, not the target address
  uint8_t current_accessible_address = 0;

  SLAVECONF_Register slaveconf{};
  slaveconf.bits.slaveaddr = node_address & 0xFF; // Address range is 0-254 (8-bit)
  slaveconf.bits.senddelay = constrain<decltype(send_delay)>(send_delay, 0U, 15U);

  // Write to SLAVECONF using current accessible address (0)
  bool success = driver_->comm_.WriteRegister(Registers::SLAVECONF, slaveconf.value, current_accessible_address);

  // Only update the driver's node address and send delay after successful programming
  if (success) {
    driver_->uart_node_address_ = node_address & 0xFF;                           // Address range is 0-254
    driver_->send_delay_ = constrain<decltype(send_delay)>(send_delay, 0U, 15U); // Store send delay locally
  }

  return success;
}

//================================================================================
//                                    PRINTER METHODS
//================================================================================

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintRegisterField(const char* name, uint32_t value, const char* format) noexcept {
  char format_str[64];
  snprintf(format_str, sizeof(format_str), "  %%s: %s", format);
  TMC51X0_LOG_DEBUG(driver_.comm_, 3, "Printer", format_str, name, value);
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintGconf() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GCONF, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading GCONF");
    return;
  }
  
  GCONF_Register gconf{};
  gconf.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "GCONF Register: 0x%08X", gconf.value);
  PrintRegisterField("recalibrate", gconf.bits.recalibrate, "%u");
  PrintRegisterField("faststandstill", gconf.bits.faststandstill, "%u");
  PrintRegisterField("en_pwm_mode (StealthChop)", gconf.bits.en_pwm_mode, "%u");
  PrintRegisterField("multistep_filt", gconf.bits.multistep_filt, "%u");
  PrintRegisterField("shaft", gconf.bits.shaft, "%u");
  PrintRegisterField("diag0_error", gconf.bits.diag0_error, "%u");
  PrintRegisterField("diag0_otpw", gconf.bits.diag0_otpw, "%u");
  PrintRegisterField("diag0_stall_step", gconf.bits.diag0_stall_step, "%u");
  PrintRegisterField("diag1_stall_dir", gconf.bits.diag1_stall_dir, "%u");
  PrintRegisterField("diag1_index", gconf.bits.diag1_index, "%u");
  PrintRegisterField("diag1_onstate", gconf.bits.diag1_onstate, "%u");
  PrintRegisterField("diag1_steps_skipped", gconf.bits.diag1_steps_skipped, "%u");
  PrintRegisterField("diag0_int_pushpull", gconf.bits.diag0_int_pushpull, "%u");
  PrintRegisterField("diag1_poscomp_pushpull", gconf.bits.diag1_poscomp_pushpull, "%u");
  PrintRegisterField("small_hysteresis", gconf.bits.small_hysteresis, "%u");
  PrintRegisterField("stop_enable", gconf.bits.stop_enable, "%u");
  PrintRegisterField("direct_mode", gconf.bits.direct_mode, "%u");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintGstat() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::GSTAT, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading GSTAT");
    return;
  }
  
  GSTAT_Register gstat{};
  gstat.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "GSTAT Register: 0x%08X", gstat.value);
  PrintRegisterField("reset", gstat.bits.reset, "%u");
  PrintRegisterField("drv_err", gstat.bits.drv_err, "%u");
  PrintRegisterField("uv_cp", gstat.bits.uv_cp, "%u");
  
  // Clear flags by writing 1 to them
  if (gstat.value != 0) {
    driver_.comm_.WriteRegister(Registers::GSTAT, gstat.value);
  }
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintRampStat() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::RAMP_STAT, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading RAMP_STAT");
    return;
  }
  
  RAMP_STAT_Register ramp_stat{};
  ramp_stat.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "RAMP_STAT Register: 0x%08X", ramp_stat.value);
  PrintRegisterField("status_stop_l", ramp_stat.bits.status_stop_l, "%u");
  PrintRegisterField("status_stop_r", ramp_stat.bits.status_stop_r, "%u");
  PrintRegisterField("status_latch_l", ramp_stat.bits.status_latch_l, "%u");
  PrintRegisterField("status_latch_r", ramp_stat.bits.status_latch_r, "%u");
  PrintRegisterField("event_stop_sg", ramp_stat.bits.event_stop_sg, "%u");
  PrintRegisterField("event_pos_reached", ramp_stat.bits.event_pos_reached, "%u");
  PrintRegisterField("velocity_reached", ramp_stat.bits.velocity_reached, "%u");
  PrintRegisterField("position_reached", ramp_stat.bits.position_reached, "%u");
  PrintRegisterField("vzero", ramp_stat.bits.vzero, "%u");
  PrintRegisterField("t_zerowait_active", ramp_stat.bits.t_zerowait_active, "%u");
  PrintRegisterField("second_move", ramp_stat.bits.second_move, "%u");
  PrintRegisterField("status_sg", ramp_stat.bits.status_sg, "%u");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintDrvStatus() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::DRV_STATUS, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading DRV_STATUS");
    return;
  }
  
  DRV_STATUS_Register drv_status{};
  drv_status.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "DRV_STATUS Register: 0x%08X", drv_status.value);
  PrintRegisterField("stst", drv_status.bits.stst, "%u");
  PrintRegisterField("olb", drv_status.bits.olb, "%u");
  PrintRegisterField("ola", drv_status.bits.ola, "%u");
  PrintRegisterField("s2gb", drv_status.bits.s2gb, "%u");
  PrintRegisterField("s2ga", drv_status.bits.s2ga, "%u");
  PrintRegisterField("otpw", drv_status.bits.otpw, "%u");
  PrintRegisterField("ot", drv_status.bits.ot, "%u");
  PrintRegisterField("stallguard", drv_status.bits.stallguard, "%u");
  PrintRegisterField("cs_actual", drv_status.bits.cs_actual, "%u");
  PrintRegisterField("fsactive", drv_status.bits.fsactive, "%u");
  PrintRegisterField("stealth", drv_status.bits.stealth, "%u");
  PrintRegisterField("sg_result", drv_status.bits.sg_result, "%u");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintChopconf() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::CHOPCONF, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading CHOPCONF");
    return;
  }
  
  CHOPCONF_Register chopconf{};
  chopconf.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "CHOPCONF Register: 0x%08X", chopconf.value);
  PrintRegisterField("toff", chopconf.bits.toff, "%u");
  PrintRegisterField("hstrt_tfd", chopconf.bits.hstrt_tfd, "%u");
  PrintRegisterField("hend_offset", chopconf.bits.hend_offset, "%u");
  PrintRegisterField("tfd_3", chopconf.bits.tfd_3, "%u");
  PrintRegisterField("disfdcc", chopconf.bits.disfdcc, "%u");
  PrintRegisterField("chm", chopconf.bits.chm, "%u");
  PrintRegisterField("tbl", chopconf.bits.tbl, "%u");
  PrintRegisterField("vhighfs", chopconf.bits.vhighfs, "%u");
  PrintRegisterField("vhighchm", chopconf.bits.vhighchm, "%u");
  PrintRegisterField("tpfd", chopconf.bits.tpfd, "%u");
  PrintRegisterField("mres", chopconf.bits.mres, "%u");
  PrintRegisterField("intpol", chopconf.bits.intpol, "%u");
  PrintRegisterField("dedge", chopconf.bits.dedge, "%u");
  PrintRegisterField("diss2g", chopconf.bits.diss2g, "%u");
  PrintRegisterField("diss2vs", chopconf.bits.diss2vs, "%u");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintPwmconf() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::PWMCONF, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading PWMCONF");
    return;
  }
  
  PWMCONF_Register pwmconf{};
  pwmconf.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "PWMCONF Register: 0x%08X", pwmconf.value);
  PrintRegisterField("pwm_ofs", pwmconf.bits.pwm_ofs, "%u");
  PrintRegisterField("pwm_grad", pwmconf.bits.pwm_grad, "%u");
  PrintRegisterField("pwm_freq", pwmconf.bits.pwm_freq, "%u");
  PrintRegisterField("pwm_autoscale", pwmconf.bits.pwm_autoscale, "%u");
  PrintRegisterField("pwm_autograd", pwmconf.bits.pwm_autograd, "%u");
  PrintRegisterField("freewheel", pwmconf.bits.freewheel, "%u");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintPwmScale() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::PWM_SCALE, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading PWM_SCALE");
    return;
  }
  
  PWM_SCALE_Register pwm_scale{};
  pwm_scale.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "PWM_SCALE Register: 0x%08X", pwm_scale.value);
  PrintRegisterField("pwm_scale_sum", pwm_scale.bits.pwm_scale_sum, "%u");
  PrintRegisterField("pwm_scale_auto", pwm_scale.bits.pwm_scale_auto, "%d");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintSwMode() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::SW_MODE, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading SW_MODE");
    return;
  }
  
  SW_MODE_Register sw_mode{};
  sw_mode.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "SW_MODE Register: 0x%08X", sw_mode.value);
  PrintRegisterField("stop_l_enable", sw_mode.bits.stop_l_enable, "%u");
  PrintRegisterField("stop_r_enable", sw_mode.bits.stop_r_enable, "%u");
  PrintRegisterField("pol_stop_l", sw_mode.bits.pol_stop_l, "%u");
  PrintRegisterField("pol_stop_r", sw_mode.bits.pol_stop_r, "%u");
  PrintRegisterField("swap_lr", sw_mode.bits.swap_lr, "%u");
  PrintRegisterField("latch_l_active", sw_mode.bits.latch_l_active, "%u");
  PrintRegisterField("latch_l_inactive", sw_mode.bits.latch_l_inactive, "%u");
  PrintRegisterField("latch_r_active", sw_mode.bits.latch_r_active, "%u");
  PrintRegisterField("latch_r_inactive", sw_mode.bits.latch_r_inactive, "%u");
  PrintRegisterField("en_latch_encoder", sw_mode.bits.en_latch_encoder, "%u");
  PrintRegisterField("sg_stop", sw_mode.bits.sg_stop, "%u");
  PrintRegisterField("en_softstop", sw_mode.bits.en_softstop, "%u");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintIoin() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::IO_INPUT_OUTPUT, value)) {
    TMC51X0_LOG_DEBUG(driver_.comm_, 0, "Printer", "Error reading IOIN");
    return;
  }
  
  IOIN_Register ioin{};
  ioin.value = value;
  
  TMC51X0_LOG_DEBUG(driver_.comm_, 2, "Printer", "IOIN Register: 0x%08X", ioin.value);
  PrintRegisterField("refl_step", ioin.bits.refl_step, "%u");
  PrintRegisterField("refr_dir", ioin.bits.refr_dir, "%u");
  PrintRegisterField("encb_dcen_cfg4", ioin.bits.encb_dcen_cfg4, "%u");
  PrintRegisterField("enca_dcin_cfg5", ioin.bits.enca_dcin_cfg5, "%u");
  PrintRegisterField("drv_enn", ioin.bits.drv_enn, "%u");
  PrintRegisterField("enc_n_dco_cfg6", ioin.bits.enc_n_dco_cfg6, "%u");
  PrintRegisterField("sd_mode", ioin.bits.sd_mode, "%u");
  PrintRegisterField("swcomp_in", ioin.bits.swcomp_in, "%u");
  PrintRegisterField("version", ioin.bits.version, "0x%02X");
}

template <typename CommType>
void TMC51x0<CommType>::Printer::PrintAll() noexcept {
  PrintGconf();
  PrintGstat();
  PrintRampStat();
  PrintDrvStatus();
  PrintChopconf();
  PrintPwmconf();
  PrintPwmScale();
  PrintSwMode();
  PrintIoin();
}

#endif // TMC51X0_IMPL
