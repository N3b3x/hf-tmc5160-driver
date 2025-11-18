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

// Helper function to constrain value between min and max
template <typename T>
static constexpr T constrain(T value, T min_val, T max_val) noexcept {
  return (value < min_val) ? min_val : ((value > max_val) ? max_val : value);
}

template <typename CommType>
bool TMC5160<CommType>::Initialize(const DriverConfig &config) noexcept {
  // Clear reset and error flags
  GSTAT_Register gstat{};
  gstat.bits.reset = true;
  gstat.bits.drv_err = true;
  gstat.bits.uv_cp = true;
  if (!comm_.WriteRegister(Registers::GSTAT, gstat.value)) {
    return false;
  }

  // Configure power stage
  DRV_CONF_Register drv_conf{};
  drv_conf.bits.drvstrength =
      constrain<decltype(config.power_stage.drv_strength)>(
          config.power_stage.drv_strength, 0U, 3U);
  drv_conf.bits.bbmtime = constrain<decltype(config.power_stage.bbm_time)>(
      config.power_stage.bbm_time, 0U, 24U);
  drv_conf.bits.bbmclks = constrain<decltype(config.power_stage.bbm_clks)>(
      config.power_stage.bbm_clks, 0U, 15U);
  if (!comm_.WriteRegister(Registers::DRV_CONF, drv_conf.value)) {
    return false;
  }

  // Configure global scaler
  uint16_t scaler = constrain<decltype(config.motor.global_scaler)>(
      config.motor.global_scaler, 32U, 256U);
  if (!comm_.WriteRegister(Registers::GLOBAL_SCALER, scaler)) {
    return false;
  }

  // Configure motor current
  IHOLD_IRUN_Register iholdrun{};
  iholdrun.bits.ihold =
      constrain<decltype(config.motor.ihold)>(config.motor.ihold, 0U, 31U);
  iholdrun.bits.irun =
      constrain<decltype(config.motor.irun)>(config.motor.irun, 0U, 31U);
  iholdrun.bits.iholddelay = 7;
  if (!comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value)) {
    return false;
  }

  // Configure short protection
  SHORT_CONF_Register short_conf{};
  short_conf.bits.s2vs_level =
      constrain<decltype(config.short_protection.s2vs_level)>(
          config.short_protection.s2vs_level, 4U, 15U);
  short_conf.bits.s2g_level =
      constrain<decltype(config.short_protection.s2g_level)>(
          config.short_protection.s2g_level, 2U, 15U);
  short_conf.bits.shortfilter =
      constrain<decltype(config.short_protection.shortfilter)>(
          config.short_protection.shortfilter, 0U, 3U);
  short_conf.bits.shortdelay =
      constrain<decltype(config.short_protection.shortdelay)>(
          config.short_protection.shortdelay, 0U, 1U);
  if (!comm_.WriteRegister(Registers::SHORT_CONF, short_conf.value)) {
    return false;
  }

  // Configure chopper
  CHOPCONF_Register chopconf{};
  chopconf.bits.toff =
      constrain<decltype(config.chopper.toff)>(config.chopper.toff, 0U, 15U);
  chopconf.bits.hstrt_tfd =
      constrain<decltype(config.chopper.hstrt)>(config.chopper.hstrt, 0U, 7U);
  chopconf.bits.hend_offset =
      constrain<decltype(config.chopper.hend)>(config.chopper.hend, 0U, 15U);
  chopconf.bits.tbl =
      constrain<decltype(config.chopper.tbl)>(config.chopper.tbl, 0U, 3U);
  chopconf.bits.vsense = config.chopper.vsense ? 1 : 0;
  chopconf.bits.mres =
      constrain<decltype(config.chopper.mres)>(config.chopper.mres, 0U, 8U);
  chopconf.bits.intpol = config.chopper.intpol ? 1 : 0;
  chopconf.bits.dedge = config.chopper.dedge ? 1 : 0;
  chopconf.bits.chm = config.chopper.chm ? 1 : 0;
  if (!comm_.WriteRegister(Registers::CHOPCONF, chopconf.value)) {
    return false;
  }

  // Configure stealthChop
  PWMCONF_Register pwmconf{};
  pwmconf.bits.pwm_ofs = config.stealthchop.pwm_ofs;
  pwmconf.bits.pwm_grad = config.stealthchop.pwm_grad;
  pwmconf.bits.pwm_freq = constrain<decltype(config.stealthchop.pwm_freq)>(
      config.stealthchop.pwm_freq, 0U, 3U);
  pwmconf.bits.pwm_autoscale = config.stealthchop.pwm_autoscale ? 1 : 0;
  pwmconf.bits.pwm_autograd = config.stealthchop.pwm_autograd ? 1 : 0;
  pwmconf.bits.pwm_reg = constrain<decltype(config.stealthchop.pwm_reg)>(
      config.stealthchop.pwm_reg, 0U, 15U);
  pwmconf.bits.pwm_lim = constrain<decltype(config.stealthchop.pwm_lim)>(
      config.stealthchop.pwm_lim, 0U, 15U);
  if (!comm_.WriteRegister(Registers::PWMCONF, pwmconf.value)) {
    return false;
  }

  // Set ramp mode to positioning
  if (!rampControl.SetRampMode(RampMode::POSITIONING)) {
    return false;
  }

  // Configure global settings
  GCONF_Register gconf{};
  gconf.bits.en_pwm_mode = true; // Enable stealthChop
  gconf.bits.shaft = (config.direction == MotorDirection::INVERSE) ? 1 : 0;
  if (!comm_.WriteRegister(Registers::GCONF, gconf.value)) {
    return false;
  }

  // Set default ramp speeds
  if (!rampControl.SetRampSpeeds(0.0f, 0.1f, 0.0f)) {
    return false;
  }

  // Set default D1 (must not be 0 in positioning mode)
  if (!comm_.WriteRegister(Registers::D_1, 100)) {
    return false;
  }

  initialized_ = true;
  return true;
}

template <typename CommType> bool TMC5160<CommType>::Reset() noexcept {
  GSTAT_Register gstat{};
  gstat.bits.reset = true;
  return comm_.WriteRegister(Registers::GSTAT, gstat.value);
}

template <typename CommType>
int32_t TMC5160<CommType>::speedToInternal(float speed_hz) const noexcept {
  // v[Hz] = v[5160] * (f_CLK[Hz]/2 / 2^23)
  // v[5160] = v[Hz] / (f_CLK[Hz]/2 / 2^23)
  // v[5160] = v[Hz] * 2^24 / f_CLK
  // Then multiply by microstep count (256)
  if (speed_hz == 0.0f) {
    return 0;
  }
  float internal =
      (speed_hz * static_cast<float>(1UL << 24)) / static_cast<float>(f_clk_);
  internal *= static_cast<float>(Microsteps::USTEP_COUNT);
  return static_cast<int32_t>(internal);
}

template <typename CommType>
float TMC5160<CommType>::speedFromInternal(
    int32_t speed_internal) const noexcept {
  // v[Hz] = v[5160] * (f_CLK[Hz]/2 / 2^23) / microstep_count
  if (speed_internal == 0) {
    return 0.0f;
  }
  float speed_hz = static_cast<float>(speed_internal) *
                   static_cast<float>(f_clk_) / static_cast<float>(1UL << 24);
  speed_hz /= static_cast<float>(Microsteps::USTEP_COUNT);
  return speed_hz;
}

template <typename CommType>
int32_t TMC5160<CommType>::accelToInternal(float accel_hz) const noexcept {
  // a[Hz/s] = a[5160] * f_CLK[Hz]^2 / (512*256) / 2^24
  // a[5160] = a[Hz/s] * (512*256) * 2^24 / f_CLK^2
  // Then multiply by microstep count (256)
  if (accel_hz == 0.0f) {
    return 0;
  }
  float internal = accel_hz * 512.0f * 256.0f * static_cast<float>(1UL << 24) /
                   (static_cast<float>(f_clk_) * static_cast<float>(f_clk_));
  internal *= static_cast<float>(Microsteps::USTEP_COUNT);
  return static_cast<int32_t>(internal);
}

template <typename CommType>
int32_t
TMC5160<CommType>::thresholdSpeedToTstep(float speed_hz) const noexcept {
  // TSTEP = f_CLK / (speed * 256)
  if (speed_hz == 0.0f) {
    return 0;
  }
  float tstep = static_cast<float>(f_clk_) / (speed_hz * 256.0f);
  tstep = std::max(0.0f, std::min(1048575.0f, tstep));
  return static_cast<int32_t>(tstep);
}

// RampControl implementation
template <typename CommType>
bool TMC5160<CommType>::RampControl::SetRampMode(RampMode mode) noexcept {
  uint8_t mode_value = static_cast<uint8_t>(mode);
  return driver_->comm_.WriteRegister(Registers::RAMPMODE, mode_value);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetTargetPosition(
    int32_t position) noexcept {
  return driver_->comm_.WriteRegister(Registers::XTARGET,
                                      static_cast<uint32_t>(position));
}

template <typename CommType>
int32_t TMC5160<CommType>::RampControl::GetCurrentPosition() noexcept {
  uint32_t value = 0;
  if (!driver_->comm_.ReadRegister(Registers::XACTUAL, value)) {
    return 0;
  }
  // Sign extend from 32-bit signed
  return static_cast<int32_t>(value);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetCurrentPosition(
    int32_t position, bool update_encoder) noexcept {
  if (!driver_->comm_.WriteRegister(Registers::XACTUAL,
                                    static_cast<uint32_t>(position))) {
    return false;
  }
  if (update_encoder) {
    if (!driver_->comm_.WriteRegister(Registers::X_ENC,
                                      static_cast<uint32_t>(position))) {
      return false;
    }
    // Clear deviation flag
    ENC_STATUS_Register enc_status{};
    enc_status.bits.deviation_warn = true;
    driver_->comm_.WriteRegister(Registers::ENC_STATUS, enc_status.value);
  }
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetMaxSpeed(float speed) noexcept {
  int32_t internal = driver_->speedToInternal(std::abs(speed));
  internal = std::min(
      internal, static_cast<decltype(internal)>(0x7FFFFF)); // VMAX is 23 bits
  if (!driver_->comm_.WriteRegister(Registers::VMAX,
                                    static_cast<uint32_t>(internal))) {
    return false;
  }
  // If in velocity mode, update direction
  uint32_t rampmode = 0;
  if (driver_->comm_.ReadRegister(Registers::RAMPMODE, rampmode)) {
    if (rampmode == static_cast<uint8_t>(RampMode::VELOCITY_POS) ||
        rampmode == static_cast<uint8_t>(RampMode::VELOCITY_NEG)) {
      uint8_t new_mode = (speed < 0.0f)
                             ? static_cast<uint8_t>(RampMode::VELOCITY_NEG)
                             : static_cast<uint8_t>(RampMode::VELOCITY_POS);
      driver_->comm_.WriteRegister(Registers::RAMPMODE, new_mode);
    }
  }
  return true;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetAcceleration(
    float acceleration) noexcept {
  return SetAccelerations(acceleration, acceleration);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetAccelerations(
    float acceleration, float deceleration) noexcept {
  int32_t accel_internal = driver_->accelToInternal(std::abs(acceleration));
  int32_t decel_internal = driver_->accelToInternal(std::abs(deceleration));
  accel_internal = std::min(
      accel_internal,
      static_cast<decltype(accel_internal)>(0xFFFF)); // AMAX/DMAX are 16 bits
  decel_internal =
      std::min(decel_internal, static_cast<decltype(decel_internal)>(0xFFFF));
  bool success = true;
  success &= driver_->comm_.WriteRegister(
      Registers::AMAX, static_cast<uint32_t>(accel_internal));
  success &= driver_->comm_.WriteRegister(
      Registers::DMAX, static_cast<uint32_t>(decel_internal));
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetRampSpeeds(
    float start_speed, float stop_speed, float transition_speed) noexcept {
  int32_t vstart = driver_->speedToInternal(std::abs(start_speed));
  int32_t vstop = driver_->speedToInternal(std::abs(stop_speed));
  int32_t v1 = driver_->speedToInternal(std::abs(transition_speed));
  vstart = std::min(
      vstart, static_cast<decltype(vstart)>(0x3FFFF)); // VSTART is 18 bits
  vstop = std::min(vstop,
                   static_cast<decltype(vstop)>(0x3FFFF)); // VSTOP is 18 bits
  v1 = std::min(v1, static_cast<decltype(v1)>(0xFFFFF));   // V1 is 20 bits
  bool success = true;
  success &= driver_->comm_.WriteRegister(Registers::VSTART,
                                          static_cast<uint32_t>(vstart));
  success &= driver_->comm_.WriteRegister(Registers::VSTOP,
                                          static_cast<uint32_t>(vstop));
  success &=
      driver_->comm_.WriteRegister(Registers::V_1, static_cast<uint32_t>(v1));
  return success;
}

template <typename CommType>
float TMC5160<CommType>::RampControl::GetCurrentSpeed() noexcept {
  uint32_t value = 0;
  if (!driver_->comm_.ReadRegister(Registers::VACTUAL, value)) {
    return 0.0f;
  }
  // VACTUAL is 24-bit signed
  int32_t signed_value = static_cast<int32_t>(value);
  if (signed_value & 0x800000) {
    signed_value |= 0xFF000000; // Sign extend
  }
  return driver_->speedFromInternal(signed_value);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::IsTargetReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_->comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.position_reached != 0;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::IsTargetVelocityReached() noexcept {
  uint32_t ramp_stat = 0;
  if (!driver_->comm_.ReadRegister(Registers::RAMP_STAT, ramp_stat)) {
    return false;
  }
  RAMP_STAT_Register status{};
  status.value = ramp_stat;
  return status.bits.velocity_reached != 0;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::Stop() noexcept {
  bool success = true;
  success &= driver_->comm_.WriteRegister(Registers::VSTART, 0);
  success &= driver_->comm_.WriteRegister(Registers::VMAX, 0);
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetTargetPositionMm(
    float position_mm, uint16_t steps_per_rev,
    float lead_screw_pitch_mm) noexcept {
  int32_t steps = MmToSteps(position_mm, steps_per_rev, lead_screw_pitch_mm);
  return SetTargetPosition(steps);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetMaxSpeedRpm(float rpm,
                                                     uint16_t steps_per_rev) noexcept {
  float steps_per_sec = RpmToStepsPerSec(rpm, steps_per_rev);
  return SetMaxSpeed(steps_per_sec);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::ConfigureReferenceSwitch(
    const ReferenceSwitchConfig &config) noexcept {
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
  return driver_->comm_.WriteRegister(Registers::SW_MODE, sw_mode.value);
}

template <typename CommType>
int32_t TMC5160<CommType>::RampControl::GetLatchedPosition() noexcept {
  uint32_t value = 0;
  if (!driver_->comm_.ReadRegister(Registers::XLATCH, value)) {
    return 0;
  }
  return static_cast<int32_t>(value);
}

template <typename CommType>
bool TMC5160<CommType>::RampControl::SetComparePosition(
    int32_t position) noexcept {
  return driver_->comm_.WriteRegister(Registers::X_COMPARE,
                                      static_cast<uint32_t>(position));
}

// MotorControl implementation
template <typename CommType>
bool TMC5160<CommType>::MotorControl::Enable() noexcept {
  uint32_t chopconf_value = 0;
  if (!driver_->comm_.ReadRegister(Registers::CHOPCONF, chopconf_value)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = chopconf_value;
  if (chopconf.bits.toff == 0) {
    // Restore saved toff value (default to 5 if not set)
    chopconf.bits.toff = 5;
  }
  return driver_->comm_.WriteRegister(Registers::CHOPCONF, chopconf.value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::Disable() noexcept {
  uint32_t chopconf_value = 0;
  if (!driver_->comm_.ReadRegister(Registers::CHOPCONF, chopconf_value)) {
    return false;
  }
  CHOPCONF_Register chopconf{};
  chopconf.value = chopconf_value;
  chopconf.bits.toff = 0; // Disable driver
  return driver_->comm_.WriteRegister(Registers::CHOPCONF, chopconf.value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetCurrent(uint8_t irun,
                                                 uint8_t ihold) noexcept {
  IHOLD_IRUN_Register iholdrun{};
  iholdrun.bits.irun = constrain<decltype(irun)>(irun, 0U, 31U);
  iholdrun.bits.ihold = constrain<decltype(ihold)>(ihold, 0U, 31U);
  iholdrun.bits.iholddelay = 7;
  return driver_->comm_.WriteRegister(Registers::IHOLD_IRUN, iholdrun.value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureChopper(
    const ChopperConfig &config) noexcept {
  CHOPCONF_Register chopconf{};
  chopconf.bits.toff = constrain<decltype(config.toff)>(config.toff, 0U, 15U);
  chopconf.bits.hstrt_tfd =
      constrain<decltype(config.hstrt)>(config.hstrt, 0U, 7U);
  chopconf.bits.hend_offset =
      constrain<decltype(config.hend)>(config.hend, 0U, 15U);
  chopconf.bits.tbl = constrain<decltype(config.tbl)>(config.tbl, 0U, 3U);
  chopconf.bits.vsense = config.vsense ? 1 : 0;
  chopconf.bits.mres = constrain<decltype(config.mres)>(config.mres, 0U, 8U);
  chopconf.bits.intpol = config.intpol ? 1 : 0;
  chopconf.bits.dedge = config.dedge ? 1 : 0;
  chopconf.bits.chm = config.chm ? 1 : 0;
  return driver_->comm_.WriteRegister(Registers::CHOPCONF, chopconf.value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureStealthChop(
    const StealthChopConfig &config) noexcept {
  PWMCONF_Register pwmconf{};
  pwmconf.bits.pwm_ofs = config.pwm_ofs;
  pwmconf.bits.pwm_grad = config.pwm_grad;
  pwmconf.bits.pwm_freq =
      constrain<decltype(config.pwm_freq)>(config.pwm_freq, 0U, 3U);
  pwmconf.bits.pwm_autoscale = config.pwm_autoscale ? 1 : 0;
  pwmconf.bits.pwm_autograd = config.pwm_autograd ? 1 : 0;
  pwmconf.bits.pwm_reg =
      constrain<decltype(config.pwm_reg)>(config.pwm_reg, 0U, 15U);
  pwmconf.bits.pwm_lim =
      constrain<decltype(config.pwm_lim)>(config.pwm_lim, 0U, 15U);
  return driver_->comm_.WriteRegister(Registers::PWMCONF, pwmconf.value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetModeChangeSpeeds(
    float pwm_thrs, float cool_thrs, float high_thrs) noexcept {
  int32_t tpwmthrs = driver_->thresholdSpeedToTstep(pwm_thrs);
  int32_t tcoolthrs = driver_->thresholdSpeedToTstep(cool_thrs);
  int32_t thigh = driver_->thresholdSpeedToTstep(high_thrs);
  tpwmthrs =
      std::min(tpwmthrs, static_cast<decltype(tpwmthrs)>(0xFFFFF)); // 20 bits
  tcoolthrs = std::min(tcoolthrs, static_cast<decltype(tcoolthrs)>(0xFFFFF));
  thigh = std::min(thigh, static_cast<decltype(thigh)>(0xFFFFF));
  bool success = true;
  success &= driver_->comm_.WriteRegister(Registers::TPWMTHRS,
                                          static_cast<uint32_t>(tpwmthrs));
  success &= driver_->comm_.WriteRegister(Registers::TCOOLTHRS,
                                          static_cast<uint32_t>(tcoolthrs));
  success &= driver_->comm_.WriteRegister(Registers::THIGH,
                                          static_cast<uint32_t>(thigh));
  return success;
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetGlobalScaler(
    uint16_t scaler) noexcept {
  scaler = constrain<decltype(scaler)>(scaler, 32U, 256U);
  return driver_->comm_.WriteRegister(Registers::GLOBAL_SCALER, scaler);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetFreewheelingMode(
    PWMFreewheel mode) noexcept {
  uint32_t pwmconf_value = 0;
  if (!driver_->comm_.ReadRegister(Registers::PWMCONF, pwmconf_value)) {
    return false;
  }
  PWMCONF_Register pwmconf{};
  pwmconf.value = pwmconf_value;
  pwmconf.bits.freewheel = static_cast<uint8_t>(mode);
  return driver_->comm_.WriteRegister(Registers::PWMCONF, pwmconf.value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureCoolStep(
    const CoolStepConfig &config) noexcept {
  COOLCONF_Register coolconf{};
  coolconf.bits.semin =
      constrain<decltype(config.semin)>(config.semin, 0U, 15U);
  coolconf.bits.seup =
      constrain<decltype(config.seup)>(config.seup, 0U, 3U);
  coolconf.bits.semax =
      constrain<decltype(config.semax)>(config.semax, 0U, 15U);
  coolconf.bits.sedn =
      constrain<decltype(config.sedn)>(config.sedn, 0U, 3U);
  coolconf.bits.seimin = config.seimin ? 1 : 0;
  coolconf.bits.sfilt = config.sfilt ? 1 : 0;
  // Note: sgt is configured via ConfigureStallGuard, not here
  return driver_->comm_.WriteRegister(Registers::COOLCONF, coolconf.value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::ConfigureDcStep(
    const DcStepConfig &config) noexcept {
  // Convert velocity threshold to internal format
  int32_t vdc_min = 0;
  if (config.vdc_min > 0.0f) {
    vdc_min = driver_->speedToInternal(config.vdc_min);
    vdc_min = std::min(vdc_min, static_cast<decltype(vdc_min)>(0xFFFFF));
  }
  return driver_->comm_.WriteRegister(Registers::VDCMIN,
                                       static_cast<uint32_t>(vdc_min));
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetMicrostepLookupTable(
    uint8_t index, uint32_t value) noexcept {
  if (index > 7) {
    return false;
  }
  const uint8_t registers[] = {Registers::MSLUT_0, Registers::MSLUT_1,
                                Registers::MSLUT_2, Registers::MSLUT_3,
                                Registers::MSLUT_4, Registers::MSLUT_5,
                                Registers::MSLUT_6, Registers::MSLUT_7};
  return driver_->comm_.WriteRegister(registers[index], value);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetMicrostepLookupTableSegmentation(
    uint8_t width_sel_0, uint8_t width_sel_1, uint8_t width_sel_2,
    uint8_t width_sel_3, uint8_t lut_seg_start1, uint8_t lut_seg_start2,
    uint8_t lut_seg_start3) noexcept {
  // Note: MSLUTSEL register structure needs to be defined in registers.hpp
  // For now, we'll construct it manually
  uint32_t mslutsel = 0;
  mslutsel |= (static_cast<uint32_t>(width_sel_0) & 0x3U) << 0;
  mslutsel |= (static_cast<uint32_t>(width_sel_1) & 0x3U) << 2;
  mslutsel |= (static_cast<uint32_t>(width_sel_2) & 0x3U) << 4;
  mslutsel |= (static_cast<uint32_t>(width_sel_3) & 0x3U) << 6;
  mslutsel |= (static_cast<uint32_t>(lut_seg_start1) & 0xFFU) << 8;
  mslutsel |= (static_cast<uint32_t>(lut_seg_start2) & 0xFFU) << 16;
  mslutsel |= (static_cast<uint32_t>(lut_seg_start3) & 0xFFU) << 24;
  return driver_->comm_.WriteRegister(Registers::MSLUTSEL, mslutsel);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetMicrostepLookupTableStart(
    uint16_t start_current) noexcept {
  start_current = constrain<decltype(start_current)>(start_current, 0U, 255U);
  return driver_->comm_.WriteRegister(Registers::MSLUTSTART, start_current);
}

template <typename CommType>
bool TMC5160<CommType>::MotorControl::SetupMotorFromSpec(
    const MotorSpec &motor_spec,
    const MechanicalSystem *mechanical_system) noexcept {
  // Calculate global scaler based on rated current
  // Typical calculation: global_scaler = (rated_current_ma * 32) / (irun * sense_resistor_current)
  // For simplicity, we'll use a basic calculation
  uint16_t global_scaler = 32;
  if (motor_spec.rated_current_ma > 0) {
    // Basic calculation: assume 1.5A max current, scale accordingly
    global_scaler = static_cast<uint16_t>(
        std::min(256U, std::max(32U, (motor_spec.rated_current_ma * 32U) / 1500U)));
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
    float target_run_current = motor_spec.rated_current_ma * 0.8f;
    uint32_t irun_calc = static_cast<uint32_t>(
        (target_run_current * 32.0f) /
        (static_cast<float>(global_scaler) * 0.046875f));
    irun = static_cast<uint8_t>(
        std::min(static_cast<uint32_t>(31U),
                 std::max(static_cast<uint32_t>(16U), irun_calc)));
    float target_hold_current = motor_spec.rated_current_ma * 0.3f;
    uint32_t ihold_calc = static_cast<uint32_t>(
        (target_hold_current * 32.0f) /
        (static_cast<float>(global_scaler) * 0.046875f));
    ihold = static_cast<uint8_t>(
        std::min(static_cast<uint32_t>(31U), ihold_calc));
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
bool TMC5160<CommType>::Encoder::Configure(
    const EncoderConfig &config) noexcept {
  ENCMODE_Register encmode{};
  encmode.bits.pol_A = config.pol_a ? 1 : 0;
  encmode.bits.pol_B = config.pol_b ? 1 : 0;
  encmode.bits.pol_N = config.pol_n ? 1 : 0;
  encmode.bits.ignore_AB = config.ignore_ab ? 1 : 0;
  encmode.bits.clr_cont = config.clr_cont ? 1 : 0;
  encmode.bits.clr_once = config.clr_once ? 1 : 0;
  encmode.bits.sensitivity =
      constrain<decltype(config.sensitivity)>(config.sensitivity, 0U, 3U);
  encmode.bits.clr_enc_x = config.clr_enc_x ? 1 : 0;
  encmode.bits.latch_x_act = config.latch_x_act ? 1 : 0;
  encmode.bits.enc_sel_decimal = config.enc_sel_decimal ? 1 : 0;
  return driver_->comm_.WriteRegister(Registers::ENCMODE, encmode.value);
}

template <typename CommType>
int32_t TMC5160<CommType>::Encoder::GetPosition() noexcept {
  uint32_t value = 0;
  if (!driver_->comm_.ReadRegister(Registers::X_ENC, value)) {
    return 0;
  }
  return static_cast<int32_t>(value);
}

template <typename CommType>
bool TMC5160<CommType>::Encoder::SetResolution(int32_t motor_steps,
                                               int32_t enc_resolution,
                                               bool inverted) noexcept {
  // Calculate factor: (motor_steps * microsteps) / enc_resolution
  float factor = static_cast<float>(motor_steps * Microsteps::USTEP_COUNT) /
                 static_cast<float>(enc_resolution);

  // Check if binary prescaler gives exact match
  int32_t enc_const_binary = static_cast<int32_t>(factor * 65536.0f);
  if (enc_const_binary * enc_resolution ==
      motor_steps * Microsteps::USTEP_COUNT * 65536) {
    // Use binary mode
    uint32_t encmode_value = 0;
    if (!driver_->comm_.ReadRegister(Registers::ENCMODE, encmode_value)) {
      return false;
    }
    ENCMODE_Register encmode{};
    encmode.value = encmode_value;
    encmode.bits.enc_sel_decimal = false;
    if (!driver_->comm_.WriteRegister(Registers::ENCMODE, encmode.value)) {
      return false;
    }
    if (inverted) {
      enc_const_binary = -enc_const_binary;
    }
    return driver_->comm_.WriteRegister(
        Registers::ENC_CONST, static_cast<uint32_t>(enc_const_binary));
  } else {
    // Use decimal mode
    uint32_t encmode_value = 0;
    if (!driver_->comm_.ReadRegister(Registers::ENCMODE, encmode_value)) {
      return false;
    }
    ENCMODE_Register encmode{};
    encmode.value = encmode_value;
    encmode.bits.enc_sel_decimal = true;
    if (!driver_->comm_.WriteRegister(Registers::ENCMODE, encmode.value)) {
      return false;
    }
    int integer_part = static_cast<int>(std::floor(factor));
    int decimal_part = static_cast<int>(
        (factor - static_cast<float>(integer_part)) * 10000.0f);
    if (inverted) {
      integer_part = 65535 - integer_part;
      decimal_part = 10000 - decimal_part;
    }
    int32_t enc_const_decimal = integer_part * 65536 + decimal_part;
    bool exact_match =
        (static_cast<int32_t>(factor * 10000.0f) * enc_resolution ==
         motor_steps * Microsteps::USTEP_COUNT * 10000);
    driver_->comm_.WriteRegister(Registers::ENC_CONST,
                                 static_cast<uint32_t>(enc_const_decimal));
    return exact_match;
  }
}

template <typename CommType>
bool TMC5160<CommType>::Encoder::SetAllowedDeviation(int32_t steps) noexcept {
  int32_t deviation = steps * Microsteps::USTEP_COUNT;
  deviation = std::min(deviation, static_cast<int32_t>(0xFFFFF)); // 20 bits
  return driver_->comm_.WriteRegister(Registers::ENC_DEVIATION,
                                      static_cast<uint32_t>(deviation));
}

template <typename CommType>
bool TMC5160<CommType>::Encoder::IsDeviationDetected() noexcept {
  uint32_t enc_status_value = 0;
  if (!driver_->comm_.ReadRegister(Registers::ENC_STATUS, enc_status_value)) {
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
  return driver_->comm_.WriteRegister(Registers::ENC_STATUS, enc_status.value);
}

// Diagnostics implementation
template <typename CommType>
DriverStatus TMC5160<CommType>::Diagnostics::GetStatus() noexcept {
  uint32_t gstat_value = 0;
  uint32_t drv_status_value = 0;
  if (!driver_->comm_.ReadRegister(Registers::GSTAT, gstat_value)) {
    return DriverStatus::OTHER_ERR;
  }
  if (!driver_->comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value)) {
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
  if (!driver_->comm_.ReadRegister(Registers::DRV_STATUS, drv_status_value)) {
    return 0;
  }
  DRV_STATUS_Register drv_status{};
  drv_status.value = drv_status_value;
  return static_cast<uint16_t>(drv_status.bits.sg_result);
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::ConfigureStallGuard(
    const StallGuardConfig &config) noexcept {
  COOLCONF_Register coolconf{};
  coolconf.bits.semin =
      constrain<decltype(config.semin)>(config.semin, 0U, 15U);
  coolconf.bits.semax =
      constrain<decltype(config.semax)>(config.semax, 0U, 15U);
  coolconf.bits.seup = constrain<decltype(config.seup)>(config.seup, 0U, 3U);
  coolconf.bits.sedn = constrain<decltype(config.sedn)>(config.sedn, 0U, 3U);
  coolconf.bits.seimin = config.seimin ? 1 : 0;
  coolconf.bits.sfilt = config.sfilt ? 1 : 0;
  // SGT is 7-bit signed (-64 to 63)
  int8_t sgt_signed =
      static_cast<int8_t>(constrain<int8_t>(config.sgt, -64, 63));
  coolconf.bits.sgt = static_cast<uint8_t>(sgt_signed) & 0x7F;
  return driver_->comm_.WriteRegister(Registers::COOLCONF, coolconf.value);
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetDriverStatusRegister(
    uint32_t &status) noexcept {
  return driver_->comm_.ReadRegister(Registers::DRV_STATUS, status);
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::GetRampStatusRegister(
    uint32_t &status) noexcept {
  return driver_->comm_.ReadRegister(Registers::RAMP_STAT, status);
}

template <typename CommType>
uint32_t TMC5160<CommType>::Diagnostics::GetLostSteps() noexcept {
  uint32_t value = 0;
  if (!driver_->comm_.ReadRegister(Registers::LOST_STEPS, value)) {
    return 0;
  }
  return value;
}

template <typename CommType>
bool TMC5160<CommType>::Diagnostics::PerformSensorlessHoming(
    bool direction, int8_t stall_threshold, float search_speed,
    int32_t &final_position) noexcept {
  // Configure StallGuard2 for homing
  StallGuardConfig sg_config{};
  sg_config.sgt = stall_threshold;
  sg_config.sfilt = true; // Enable filter for stable readings
  if (!ConfigureStallGuard(sg_config)) {
    return false;
  }

  // Enable StallGuard2 stop in SW_MODE
  uint32_t sw_mode_value = 0;
  if (!driver_->comm_.ReadRegister(Registers::SW_MODE, sw_mode_value)) {
    return false;
  }
  SW_MODE_Register sw_mode{};
  sw_mode.value = sw_mode_value;
  sw_mode.bits.sg_stop = true; // Enable stop on stall
  sw_mode.bits.en_softstop = true; // Use soft stop
  if (!driver_->comm_.WriteRegister(Registers::SW_MODE, sw_mode.value)) {
    return false;
  }

  // Set velocity mode and start movement
  RampMode mode = direction ? RampMode::VELOCITY_POS : RampMode::VELOCITY_NEG;
  if (!driver_->rampControl.SetRampMode(mode)) {
    return false;
  }
  if (!driver_->rampControl.SetMaxSpeed(search_speed)) {
    return false;
  }

  // Wait for stall (this is a simplified implementation)
  // In a real implementation, you would poll GetStallGuard() or check RAMP_STAT
  // For now, we'll just set the speed and let the hardware handle it
  // The user should check IsTargetReached() or monitor stall status

  // Read final position
  final_position = driver_->rampControl.GetCurrentPosition();

  // Stop motor
  driver_->rampControl.Stop();

  return true;
}

// Protection implementation
template <typename CommType>
bool TMC5160<CommType>::Protection::ConfigureShortProtection(
    const ShortProtectionConfig &config) noexcept {
  return SetShortProtectionLevels(config.s2vs_level, config.s2g_level,
                                  config.shortfilter, config.shortdelay);
}

template <typename CommType>
bool TMC5160<CommType>::Protection::SetShortProtectionLevels(
    uint8_t s2vs_level, uint8_t s2g_level, uint8_t shortfilter,
    uint8_t shortdelay) noexcept {
  SHORT_CONF_Register short_conf{};
  short_conf.bits.s2vs_level =
      constrain<decltype(s2vs_level)>(s2vs_level, 4U, 15U);
  short_conf.bits.s2g_level =
      constrain<decltype(s2g_level)>(s2g_level, 2U, 15U);
  short_conf.bits.shortfilter = constrain<uint8_t>(shortfilter, 0U, 3U);
  short_conf.bits.shortdelay = constrain<uint8_t>(shortdelay, 0U, 1U);
  return driver_->comm_.WriteRegister(Registers::SHORT_CONF, short_conf.value);
}

#endif // TMC5160_IMPL
