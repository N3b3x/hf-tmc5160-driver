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

template <typename CommType>
bool TMC5160<CommType>::Initialize(const DriverConfig& config) noexcept {
  TMC5160_LOG_DEBUG(comm_, 2, "TMC5160", "Initialize(irun=%u, ihold=%u, global_scaler=%u, toff=%u, mres=%u)",
                     config.motor.irun, config.motor.ihold, config.motor.global_scaler,
                     config.chopper.toff, config.chopper.mres);

  // Clear reset and error flags
  GSTAT_Register gstat{};
  gstat.bits.reset = true;
  gstat.bits.drv_err = true;
  gstat.bits.uv_cp = true;
  if (!this->comm_.WriteRegister(Registers::GSTAT, gstat.value, this->GetCommAddress())) {
    return false;
  }

  // Configure power stage
  DRV_CONF_Register drv_conf{};
  drv_conf.bits.drvstrength =
      constrain<decltype(config.power_stage.drv_strength)>(config.power_stage.drv_strength, 0U, 3U);
  drv_conf.bits.bbmtime = constrain<decltype(config.power_stage.bbm_time)>(config.power_stage.bbm_time, 0U, 24U);
  drv_conf.bits.bbmclks = constrain<decltype(config.power_stage.bbm_clks)>(config.power_stage.bbm_clks, 0U, 15U);
  drv_conf.bits.otselect = constrain<decltype(config.power_stage.otselect)>(config.power_stage.otselect, 0U, 3U);
  drv_conf.bits.filt_isense =
      constrain<decltype(config.power_stage.filt_isense)>(config.power_stage.filt_isense, 0U, 3U);
  if (!this->comm_.WriteRegister(Registers::DRV_CONF, drv_conf.value, this->GetCommAddress())) {
    return false;
  }

  // Configure global scaler
  auto scaler = constrain<decltype(config.motor.global_scaler)>(config.motor.global_scaler, 32U, 256U);
  TMC5160_LOG_DEBUG(comm_, 3, "TMC5160", "Initialize: Setting GLOBAL_SCALER=%u", scaler);
  if (!this->comm_.WriteRegister(Registers::GLOBAL_SCALER, scaler, this->GetCommAddress())) {
    return false;
  }

  // Configure motor current
  IHOLD_IRUN_Register iholdrun{};
  iholdrun.bits.ihold = constrain<decltype(config.motor.ihold)>(config.motor.ihold, 0U, 31U);
  iholdrun.bits.irun = constrain<decltype(config.motor.irun)>(config.motor.irun, 0U, 31U);
  iholdrun.bits.iholddelay = 7;
  TMC5160_LOG_DEBUG(comm_, 3, "TMC5160", "Initialize: Setting IHOLD_IRUN(irun=%u, ihold=%u, iholddelay=7)",
                     iholdrun.bits.irun, iholdrun.bits.ihold);
  if (!this->comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, this->GetCommAddress())) {
    return false;
  }

  // Configure short protection
  SHORT_CONF_Register short_conf{};
  short_conf.bits.s2vs_level =
      constrain<decltype(config.short_protection.s2vs_level)>(config.short_protection.s2vs_level, 4U, 15U);
  short_conf.bits.s2g_level =
      constrain<decltype(config.short_protection.s2g_level)>(config.short_protection.s2g_level, 2U, 15U);
  short_conf.bits.shortfilter =
      constrain<decltype(config.short_protection.shortfilter)>(config.short_protection.shortfilter, 0U, 3U);
  short_conf.bits.shortdelay =
      constrain<decltype(config.short_protection.shortdelay)>(config.short_protection.shortdelay, 0U, 1U);
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
  if (!rampControl.SetRampSpeeds(0.0F, 0.1F, 0.0F)) {
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
bool TMC5160<CommType>::RampControl::SetTargetPosition(int32_t position) noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetTargetPosition(%d)", position);
  return driver_.comm_.WriteRegister(Registers::XTARGET, static_cast<uint32_t>(position));
}

template <typename CommType>
int32_t TMC5160<CommType>::RampControl::GetCurrentPosition() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XACTUAL, value, driver_.GetCommAddress())) {
    return 0;
  }
  // Sign extend from 32-bit signed
  return static_cast<int32_t>(value);
}

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
bool TMC5160<CommType>::RampControl::SetMaxSpeed(float speed) noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetMaxSpeed(%.2f steps/s)", speed);
  int32_t internal = driver_.speedToInternal(std::abs(speed));
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
          (speed < 0.0F) ? static_cast<uint8_t>(RampMode::VELOCITY_NEG) : static_cast<uint8_t>(RampMode::VELOCITY_POS);
      driver_.comm_.WriteRegister(Registers::RAMPMODE, new_mode, driver_.GetCommAddress());
    }
  }
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetAcceleration(float acceleration) noexcept {
  return SetAccelerations(acceleration, acceleration);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetAccelerations(float acceleration, float deceleration) noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "RampControl::SetAccelerations(accel=%.2f, decel=%.2f steps/s²)",
                     acceleration, deceleration);
  int32_t accel_internal = driver_.accelToInternal(std::abs(acceleration));
  int32_t decel_internal = driver_.accelToInternal(std::abs(deceleration));
  accel_internal = std::min(accel_internal,
      static_cast<decltype(accel_internal)>(0xFFFF)); // AMAX/DMAX are 16 bits
  decel_internal = std::min(decel_internal, static_cast<decltype(decel_internal)>(0xFFFF));
  bool success = true;
  success &= driver_.comm_.WriteRegister(Registers::AMAX, static_cast<uint32_t>(accel_internal));
  success &= driver_.comm_.WriteRegister(Registers::DMAX, static_cast<uint32_t>(decel_internal));
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetRampSpeeds(float start_speed, float stop_speed,
                                                   float transition_speed) noexcept {
  int32_t vstart = driver_.speedToInternal(std::abs(start_speed));
  int32_t vstop = driver_.speedToInternal(std::abs(stop_speed));
  int32_t v1 = driver_.speedToInternal(std::abs(transition_speed));
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
float TMC5160<CommType>::RampControl::GetCurrentSpeed() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::VACTUAL, value)) {
    return 0.0F;
  }
  // VACTUAL is 24-bit signed
  auto signed_value = static_cast<int32_t>(value);
  if (signed_value & 0x800000) {
    signed_value |= static_cast<int32_t>(0xFF000000U); // Sign extend
  }
  return driver_.speedFromInternal(signed_value);
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
bool TMC5160<CommType>::RampControl::SetTargetPositionMm(float position_mm, uint16_t steps_per_rev,
    float lead_screw_pitch_mm) noexcept {
  int32_t steps = MmToSteps(position_mm, steps_per_rev, lead_screw_pitch_mm);
  return SetTargetPosition(steps);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetMaxSpeedRpm(float rpm, uint16_t steps_per_rev) noexcept {
  float steps_per_sec = RpmToStepsPerSec(rpm, steps_per_rev);
  return SetMaxSpeed(steps_per_sec);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::ConfigureReferenceSwitch(const ReferenceSwitchConfig& config) noexcept {
  SW_MODE_Register sw_mode{};
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
int32_t TMC5160<CommType>::RampControl::GetLatchedPosition() noexcept {
  uint32_t value = 0;
  if (!driver_.comm_.ReadRegister(Registers::XLATCH, value)) {
    return 0;
  }
  return static_cast<int32_t>(value);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetComparePosition(int32_t position) noexcept {
  return driver_.comm_.WriteRegister(Registers::X_COMPARE, static_cast<uint32_t>(position));
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
bool TMC5160<CommType>::RampControl::SetFirstAcceleration(float a1) noexcept {
  if (a1 == 0.0F) {
    // Set to 0 to use AMAX for this phase
    return driver_.comm_.WriteRegister(Registers::A_1, 0, driver_.GetCommAddress());
  }
  int32_t a1_internal = driver_.accelToInternal(std::abs(a1));
  a1_internal = std::min(a1_internal,
      static_cast<decltype(a1_internal)>(0xFFFF)); // A_1 is 16 bits
  return driver_.comm_.WriteRegister(Registers::A_1, static_cast<uint32_t>(a1_internal));
}

// MotorControl implementation
template <typename CommType>
bool TMC5160<CommType>::MotorControl::Enable() noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::Enable()");
  
  // Enable via EN pin GPIO (EN is active HIGH to disable, so set to INACTIVE/LOW to enable)
  // This must be done first to enable the power stage
  driver_.comm_.GpioSet(TMC5160CtrlPin::EN, GpioSignal::INACTIVE);
  
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
  
  // Disable via EN pin GPIO (EN is active HIGH to disable, so set to ACTIVE/HIGH to disable)
  driver_.comm_.GpioSet(TMC5160CtrlPin::EN, GpioSignal::ACTIVE);
  
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetCurrent(uint8_t irun, uint8_t ihold) noexcept {
  TMC5160_LOG_DEBUG(driver_.comm_, 2, "TMC5160", "MotorControl::SetCurrent(irun=%u, ihold=%u)", irun, ihold);
  IHOLD_IRUN_Register iholdrun{};
  iholdrun.bits.irun = constrain<decltype(irun)>(irun, 0U, 31U);
  iholdrun.bits.ihold = constrain<decltype(ihold)>(ihold, 0U, 31U);
  iholdrun.bits.iholddelay = 7;
  return driver_.comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureChopper(const ChopperConfig& config) noexcept {
  CHOPCONF_Register chopconf{};
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
bool TMC5160<CommType>::MotorControl::SetGlobalScaler(uint16_t scaler) noexcept {
  scaler = constrain<decltype(scaler)>(scaler, 32U, 256U);
  return driver_.comm_.WriteRegister(Registers::GLOBAL_SCALER, scaler, driver_.GetCommAddress());
}


template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureCoolStep(const CoolStepConfig& config) noexcept {
  COOLCONF_Register coolconf{};
  // Initialize all fields explicitly (reserved bits will be 0 from initialization)
  coolconf.bits.semin = constrain<decltype(config.semin)>(config.semin, 0U, 15U);
  coolconf.bits.reserved1 = 0; // Bit 4: Reserved, set to 0
  coolconf.bits.seup = constrain<decltype(config.seup)>(config.seup, 0U, 3U);
  coolconf.bits.reserved2 = 0; // Bit 7: Reserved, set to 0
  coolconf.bits.semax = constrain<decltype(config.semax)>(config.semax, 0U, 15U);
  coolconf.bits.reserved3 = 0; // Bit 12: Reserved, set to 0
  coolconf.bits.sedn = constrain<decltype(config.sedn)>(config.sedn, 0U, 3U);
  coolconf.bits.seimin = config.seimin ? 1 : 0;
  coolconf.bits.sgt = 0;       // Bits 22..16: Default to 0 (starting value for most motors)
  coolconf.bits.reserved4 = 0; // Bit 23: Reserved, set to 0
  coolconf.bits.sfilt = config.sfilt ? 1 : 0;
  coolconf.bits.reserved5 = 0; // Bits 31..25: Reserved, set to 0
  // Note: sgt can be configured separately via ConfigureStallGuard() if needed
  return driver_.comm_.WriteRegister(Registers::COOLCONF, coolconf.value, driver_.GetCommAddress());
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureDcStep(const DcStepConfig& config) noexcept {
  // Convert velocity threshold to internal format
  int32_t vdc_min = 0;
  if (config.vdc_min > 0.0F) {
    vdc_min = driver_.speedToInternal(config.vdc_min);
    // VDCMIN is 23-bit register, but only bits 22..8 are used (bits 7..0 ignored)
    // Mask to 0x7FFF00 to ensure bits 22..8 are set correctly, bits 7..0 are zero
    vdc_min = std::min(vdc_min, static_cast<decltype(vdc_min)>(0x7FFF));
    vdc_min = (vdc_min << 8) & 0x7FFF00; // Shift to bits 22..8, clear bits 7..0
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
  COOLCONF_Register coolconf{};
  // Initialize all fields explicitly (reserved bits set to 0)
  coolconf.bits.semin = constrain<decltype(config.semin)>(config.semin, 0U, 15U);
  coolconf.bits.reserved1 = 0; // Bit 4: Reserved, set to 0
  coolconf.bits.seup = constrain<decltype(config.seup)>(config.seup, 0U, 3U);
  coolconf.bits.reserved2 = 0; // Bit 7: Reserved, set to 0
  coolconf.bits.semax = constrain<decltype(config.semax)>(config.semax, 0U, 15U);
  coolconf.bits.reserved3 = 0; // Bit 12: Reserved, set to 0
  coolconf.bits.sedn = constrain<decltype(config.sedn)>(config.sedn, 0U, 3U);
  coolconf.bits.seimin = config.seimin ? 1 : 0;
  // SGT is signed 7-bit (-64 to +63), constrain and mask to 7 bits (bits 22..16)
  // Zero (0) is the starting value working with most motors
  auto sgt_signed = static_cast<int8_t>(constrain<int8_t>(config.sgt, -64, 63));
  coolconf.bits.sgt = static_cast<int32_t>(sgt_signed) & 0x7F;
  coolconf.bits.reserved4 = 0; // Bit 23: Reserved, set to 0
  coolconf.bits.sfilt = config.sfilt ? 1 : 0;
  coolconf.bits.reserved5 = 0; // Bits 31..25: Reserved, set to 0
  return driver_.comm_.WriteRegister(Registers::COOLCONF, coolconf.value, driver_.GetCommAddress());
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
                                                             int32_t& final_position) noexcept {
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
  sw_mode.bits.en_softstop = true; // Use soft stop
  if (!driver_.comm_.WriteRegister(Registers::SW_MODE, sw_mode.value)) {
    return false;
  }

  // Set velocity mode and start movement
  RampMode mode = direction ? RampMode::VELOCITY_POS : RampMode::VELOCITY_NEG;
  if (!driver_.rampControl.SetRampMode(mode)) {
    return false;
  }
  if (!driver_.rampControl.SetMaxSpeed(search_speed)) {
    return false;
  }

  // Wait for stall (this is a simplified implementation)
  // In a real implementation, you would poll GetStallGuard() or check RAMP_STAT
  // For now, we'll just set the speed and let the hardware handle it
  // The user should check IsTargetReached() or monitor stall status

  // Read final position
  final_position = driver_.rampControl.GetCurrentPosition();

  // Stop motor
  driver_.rampControl.Stop();

  return true;
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
bool TMC5160<CommType>::Protection::ConfigureShortProtection(const ShortProtectionConfig& config) noexcept {
  return SetShortProtectionLevels(config.s2vs_level, config.s2g_level, config.shortfilter, config.shortdelay);
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
