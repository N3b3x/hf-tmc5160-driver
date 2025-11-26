/**
 * @file tmc51x0_motor_calc.hpp
 * @brief Motor current calculation functions for TMC51x0 (TMC5130 & TMC5160)
 *
 * This file provides functions to automatically calculate IRUN, IHOLD, and GLOBAL_SCALER
 * based on motor specifications, sense resistor, and supply voltage using datasheet equations.
 * Supports both TMC5130 and TMC5160 chips.
 *
 * @defgroup TMC51X0_MotorCalc Motor Current Calculation
 * @brief Functions for calculating motor current settings
 */

#ifndef TMC51X0_MOTOR_CALC_HPP
#define TMC51X0_MOTOR_CALC_HPP

#include "tmc51x0_types.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace tmc51x0 {

// Datasheet constants (shared across calculation functions)
namespace MotorCalcConstants {
constexpr float VFS = 0.325F;           // Typical full-scale voltage (V)
constexpr float SQRT2 = 1.41421356237F; // √2
} // namespace MotorCalcConstants

/**
 * @brief Calculate motor current settings from physical parameters
 *
 * Calculates IRUN, IHOLD, and GLOBAL_SCALER based on:
 * - Desired run current (typically motor rated current)
 * - Sense resistor value
 * - Supply voltage (for StealthChop lower limit calculation)
 * - Motor winding resistance (for StealthChop lower limit)
 *
 * @param motor_spec Motor specifications including rated current, resistance, etc.
 * @param sense_resistor_mohm Sense resistor value in milliohms (e.g., 50 for 0.05Ω)
 * @param supply_voltage_mv Supply voltage in millivolts (e.g., 24000 for 24V)
 * @param run_current_ma Desired run current in milliamps (default: motor rated current)
 * @param hold_current_ma Desired hold current in milliamps (default: 30% of run current)
 * @param irun Reference to store calculated IRUN value (0-31)
 * @param ihold Reference to store calculated IHOLD value (0-31)
 * @param global_scaler Reference to store calculated GLOBAL_SCALER value (32-256)
 * @return true if calculation successful, false if parameters invalid
 *
 * @note Based on datasheet equation:
 *       I_RMS = (GLOBAL_SCALER/256) * ((CS+1)/32) * (VFS/RSENSE) * (1/√2)
 *       Where VFS ≈ 0.325V (typical full-scale voltage)
 *
 * @note For best precision, IRUN will be constrained to 16-31 range
 * @note For automatic tuning compatibility, IRUN will be at least 8
 */
inline bool CalculateMotorCurrent(const MotorSpec& motor_spec, uint32_t sense_resistor_mohm, uint32_t supply_voltage_mv,
                                  uint16_t run_current_ma, uint16_t hold_current_ma, uint8_t& irun, uint8_t& ihold,
                                  uint16_t& global_scaler) noexcept {
  // Validate inputs
  if (sense_resistor_mohm == 0 || supply_voltage_mv == 0) {
    return false;
  }

  // Use motor rated current if run_current_ma is 0
  if (run_current_ma == 0) {
    run_current_ma = motor_spec.rated_current_ma;
  }
  if (run_current_ma == 0) {
    return false; // No current specified
  }

  // Default hold current to 30% of run current if not specified
  if (hold_current_ma == 0) {
    float hold_current_float = run_current_ma * 0.3F;
    hold_current_ma = static_cast<uint16_t>(hold_current_float);
  }

  // Use shared constants
  constexpr float VFS = MotorCalcConstants::VFS;
  constexpr float SQRT2 = MotorCalcConstants::SQRT2;

  // Convert sense resistor from mΩ to Ω
  float rsense_ohm = static_cast<float>(sense_resistor_mohm) / 1000.0F;

  // Calculate maximum possible RMS current at full scale (GLOBAL_SCALER=256, CS=31)
  // I_RMS_max = (256/256) * ((31+1)/32) * (VFS/RSENSE) * (1/√2)
  //           = 1.0 * 1.0 * (VFS/RSENSE) * (1/√2)
  float i_rms_max = (VFS / rsense_ohm) / SQRT2;

  // Check if desired current exceeds maximum possible
  float run_current_a = static_cast<float>(run_current_ma) / 1000.0F;
  if (run_current_a > i_rms_max * 1.1F) { // Allow 10% tolerance
    return false;                          // Desired current too high for sense resistor
  }

  // Strategy: Use IRUN in optimal range (16-31) and fine-tune with GLOBAL_SCALER
  // Start with IRUN=31 (maximum CS) and calculate required GLOBAL_SCALER
  // Then adjust IRUN down if GLOBAL_SCALER would be too low

  // Calculate required GLOBAL_SCALER for IRUN=31
  // I_RMS = (GLOBAL_SCALER/256) * ((31+1)/32) * (VFS/RSENSE) * (1/√2)
  // Rearranged: GLOBAL_SCALER = I_RMS * 256 * 32 / ((31+1) * (VFS/RSENSE) * (1/√2))
  float global_scaler_float = (run_current_a * 256.0F * 32.0F) / (32.0F * (VFS / rsense_ohm) / SQRT2);

  // Constrain GLOBAL_SCALER to valid range (32-256, where 0 = 256)
  auto calculated_scaler = static_cast<uint16_t>(std::round(global_scaler_float));
  calculated_scaler = std::max<uint16_t>(calculated_scaler, 32);
  calculated_scaler = std::min<uint16_t>(calculated_scaler, 256);

  // If GLOBAL_SCALER is at maximum (256), we can reduce IRUN for better precision
  // Try to find optimal IRUN in range 16-31
  uint8_t optimal_irun = 31;
  uint16_t optimal_scaler = calculated_scaler;

  if (calculated_scaler >= 200) {
    // Try reducing IRUN to get GLOBAL_SCALER in better range (128-200)
    for (uint8_t test_irun = 30; test_irun >= 16; --test_irun) {
      float test_scaler_float = (run_current_a * 256.0F * 32.0F) / (static_cast<float>(test_irun + 1) * (VFS / rsense_ohm) / SQRT2);
      auto test_scaler = static_cast<uint16_t>(std::round(test_scaler_float));

      if (test_scaler >= 32 && test_scaler <= 200) {
        optimal_irun = test_irun;
        optimal_scaler = test_scaler;
        break;
      }
    }
  }

  // Ensure IRUN meets minimum for automatic tuning (IRUN ≥ 8)
  if (optimal_irun < 8) {
    optimal_irun = 8;
    // Recalculate scaler for IRUN=8
    float scaler_float = (run_current_a * 256.0F * 32.0F) / (9.0F * (VFS / rsense_ohm) / SQRT2);
    optimal_scaler = static_cast<uint16_t>(std::round(scaler_float));
    optimal_scaler = std::max<uint16_t>(optimal_scaler, 32);
    optimal_scaler = std::min<uint16_t>(optimal_scaler, 256);
  }

  // Calculate IHOLD using same method
  float hold_current_a = static_cast<float>(hold_current_ma) / 1000.0F;

  // Calculate required GLOBAL_SCALER for IHOLD (use same scaler as IRUN)
  // I_RMS = (GLOBAL_SCALER/256) * ((IHOLD+1)/32) * (VFS/RSENSE) * (1/√2)
  // Rearranged: IHOLD = (I_RMS * 256 * 32) / (GLOBAL_SCALER * (VFS/RSENSE) * (1/√2)) - 1
  float ihold_float =
      ((hold_current_a * 256.0F * 32.0F) / (static_cast<float>(optimal_scaler) * (VFS / rsense_ohm) / SQRT2)) - 1.0F;

  auto calculated_ihold = static_cast<uint8_t>(std::round(ihold_float));

  // Constrain IHOLD to valid range (0-31) and ensure it's less than IRUN
  calculated_ihold = std::min<uint8_t>(calculated_ihold, 31);
  if (calculated_ihold >= optimal_irun) {
    calculated_ihold = (optimal_irun > 0) ? (optimal_irun - 1) : 0;
  }

  irun = optimal_irun;
  ihold = calculated_ihold;
  global_scaler = optimal_scaler;

  return true;
}

/**
 * @brief Calculate StealthChop lower current limit
 *
 * Calculates the minimum motor current that can be regulated in StealthChop mode
 * based on blank time, PWM frequency, supply voltage, and motor resistance.
 *
 * @param motor_spec Motor specifications including winding resistance
 * @param supply_voltage_mv Supply voltage in millivolts
 * @param tbl Blank time setting (0-3, corresponds to 16, 24, 36, 54 clock cycles)
 * @param pwm_freq PWM frequency setting (0-3)
 * @param f_clk Clock frequency in Hz (default: 12 MHz)
 * @return Lower current limit in milliamps, or 0 if calculation not possible
 *
 * @note Based on datasheet equation:
 *       I_Lower_Limit = t_BLANK * f_PWM * V_M / R_COIL
 *       Where t_BLANK depends on TBL setting
 */
inline uint16_t CalculateStealthChopLowerLimit(const MotorSpec& motor_spec, uint32_t supply_voltage_mv, uint8_t tbl,
                                               uint8_t pwm_freq, uint32_t f_clk = 12000000U) noexcept {
  if (motor_spec.winding_resistance_mohm == 0) {
    return 0; // Cannot calculate without resistance
  }

  // Blank time in clock cycles based on TBL setting
  constexpr std::array<uint8_t, 4> blank_times = {16, 24, 36, 54};
  const uint8_t tbl_index = std::min(tbl, static_cast<uint8_t>(3));
  uint8_t t_blank = blank_times[tbl_index];

  // PWM frequency divider based on PWM_FREQ setting
  constexpr std::array<uint32_t, 4> pwm_divisors = {1024, 683, 512, 410};
  const uint8_t pwm_freq_index = std::min(pwm_freq, static_cast<uint8_t>(3));
  uint32_t pwm_divisor = pwm_divisors[pwm_freq_index];

  // Calculate PWM frequency: f_PWM = 2 / (pwm_divisor * t_CLK)
  // Actually: f_PWM = 2 / (pwm_divisor * (1/f_CLK)) = 2 * f_CLK / pwm_divisor
  float f_pwm = (2.0F * static_cast<float>(f_clk)) / static_cast<float>(pwm_divisor);

  // Convert to Hz: f_PWM = 2 / (pwm_divisor * t_CLK) where t_CLK = 1/f_CLK
  // So: f_PWM = 2 * f_CLK / pwm_divisor (already calculated above)

  // Convert resistance from mΩ to Ω
  float r_coil = static_cast<float>(motor_spec.winding_resistance_mohm) / 1000.0F;

  // Convert supply voltage from mV to V
  float v_m = static_cast<float>(supply_voltage_mv) / 1000.0F;

  // Calculate lower limit: I = t_BLANK * f_PWM * V_M / R_COIL
  // t_BLANK is in clock cycles, so we need: t_BLANK / f_CLK (time in seconds)
  float t_blank_sec = static_cast<float>(t_blank) / static_cast<float>(f_clk);
  float i_lower = t_blank_sec * f_pwm * v_m / r_coil;

  // Convert to milliamps
  return static_cast<uint16_t>(i_lower * 1000.0F);
}

/**
 * @brief Calculate maximum RMS current for a given sense resistor
 *
 * @param sense_resistor_mohm Sense resistor value in milliohms
 * @return Maximum RMS current in milliamps at full scale (GLOBAL_SCALER=256, CS=31)
 */
inline uint16_t CalculateMaxCurrentForSenseResistor(uint32_t sense_resistor_mohm) noexcept {
  if (sense_resistor_mohm == 0) {
    return 0;
  }

  // Use shared constants
  constexpr float VFS = MotorCalcConstants::VFS;
  constexpr float SQRT2 = MotorCalcConstants::SQRT2;

  float rsense_ohm = static_cast<float>(sense_resistor_mohm) / 1000.0F;
  float i_rms_max = (VFS / rsense_ohm) / SQRT2;

  return static_cast<uint16_t>(i_rms_max * 1000.0F);
}

/**
 * @brief Calculate S2VS_LEVEL register value from voltage threshold
 *
 * Converts user-friendly voltage threshold (mV) to S2VS_LEVEL register value (4-15).
 * Uses datasheet typical values for interpolation.
 *
 * @param voltage_mv Voltage threshold in millivolts (0 = auto-calculate to 625mV)
 * @return S2VS_LEVEL register value (4-15), or 0 if voltage out of range
 *
 * Datasheet typical values:
 * - S2VS_LEVEL=6: 625mV
 * - S2VS_LEVEL=15: 1560mV
 *
 * @note Linear interpolation between known points
 */
inline uint8_t CalculateS2VSLevel(uint16_t voltage_mv) noexcept {
  if (voltage_mv == 0) {
    return 6; // Default recommended value
  }

  // Constrain to valid range
  if (voltage_mv < 400 || voltage_mv > 2000) {
    return 0; // Invalid
  }

  // Datasheet typical values for interpolation
  // S2VS_LEVEL=6: 625mV, S2VS_LEVEL=15: 1560mV
  // Linear relationship: level = 6 + (voltage - 625) * (15-6) / (1560-625)
  // Simplified: level = 6 + (voltage - 625) * 9 / 935

  if (voltage_mv <= 625) {
    // Below or at level 6 threshold - use linear interpolation from level 4 (approx 400mV) to level 6
    // Level 4-6 range: approximate 400-625mV
    if (voltage_mv < 400) {
      return 4; // Minimum
    }
    float level = 4.0F + ((static_cast<float>(voltage_mv - 400) / 225.0F) * 2.0F); // 4 to 6
    return static_cast<uint8_t>(std::round(level));
  }
  // Above level 6 threshold - interpolate from level 6 to level 15
  float level = 6.0F + ((static_cast<float>(voltage_mv - 625) / 935.0F) * 9.0F); // 6 to 15
  auto calculated = static_cast<uint8_t>(std::round(level));
  return std::min(static_cast<uint8_t>(15), std::max(static_cast<uint8_t>(4), calculated));
}

/**
 * @brief Calculate S2G_LEVEL register value from voltage threshold
 *
 * Converts user-friendly voltage threshold (mV) to S2G_LEVEL register value (2-15).
 * Uses datasheet typical values for interpolation.
 *
 * @param voltage_mv Voltage threshold in millivolts (0 = auto-calculate to 625mV)
 * @param supply_voltage_mv Supply voltage in millivolts (for VS-dependent calculation)
 * @return S2G_LEVEL register value (2-15), or 0 if voltage out of range
 *
 * Datasheet typical values (VS-dependent):
 * - S2G_LEVEL=6 (VS<50V): 625mV
 * - S2G_LEVEL=15 (VS<52V): 1560mV
 * - S2G_LEVEL=15 (VS<55V): 850mV (minimum to prevent false triggers)
 *
 * @note For VS>52V, minimum recommended is 1200mV (S2G_LEVEL=12)
 */
inline uint8_t CalculateS2GLevel(uint16_t voltage_mv, uint32_t supply_voltage_mv = 0) noexcept {
  if (voltage_mv == 0) {
    return 6; // Default recommended value
  }

  // Constrain to valid range
  if (voltage_mv < 400 || voltage_mv > 2000) {
    return 0; // Invalid
  }

  // For VS>52V, enforce minimum 1200mV (S2G_LEVEL=12) to prevent false triggers
  if (supply_voltage_mv > 52000 && voltage_mv < 1200) {
    voltage_mv = 1200; // Enforce minimum
  }

  // Datasheet typical values for interpolation
  // S2G_LEVEL=6: 625mV, S2G_LEVEL=15: 1560mV (VS<52V) or 850mV (VS<55V)
  // Use VS<52V values for interpolation (more conservative)

  if (voltage_mv <= 625) {
    // Below or at level 6 threshold - use linear interpolation from level 2 (approx 400mV) to level 6
    if (voltage_mv < 400) {
      return 2; // Minimum
    }
    float level = 2.0F + ((static_cast<float>(voltage_mv - 400) / 225.0F) * 4.0F); // 2 to 6
    return static_cast<uint8_t>(std::round(level));
  }
  // Above level 6 threshold - interpolate from level 6 to level 15
  float level = 6.0F + ((static_cast<float>(voltage_mv - 625) / 935.0F) * 9.0F); // 6 to 15
  auto calculated = static_cast<uint8_t>(std::round(level));
  return std::min(static_cast<uint8_t>(15), std::max(static_cast<uint8_t>(2), calculated));
}

/**
 * @brief Calculate shortdelay register bit from detection delay time
 *
 * Converts user-friendly delay time (µs in 0.1µs units) to shortdelay register bit (0-1).
 *
 * @param delay_us_x10 Detection delay in 0.1µs units (0 = auto-calculate to 8.5 = 0.85µs)
 * @return shortdelay register bit (0 or 1), or 0 if delay out of range
 *
 * Datasheet timing (typical values):
 * - shortdelay=0: 0.5-0.85-1.1µs (normal)
 * - shortdelay=1: 1.1-1.6-2.2µs (high delay)
 *
 * @note Threshold at ~1.0µs: below uses shortdelay=0, above uses shortdelay=1
 */
inline uint8_t CalculateShortDelay(uint8_t delay_us_x10) noexcept {
  if (delay_us_x10 == 0) {
    return 0; // Default recommended value (0.85µs = shortdelay=0)
  }

  // Constrain to valid range (5-25 = 0.5-2.5µs)
  if (delay_us_x10 < 5) {
    delay_us_x10 = 5; // Minimum 0.5µs
  } else if (delay_us_x10 > 25) {
    delay_us_x10 = 25; // Maximum 2.5µs
  }

  // Threshold at ~1.0µs (10 in 0.1µs units)
  // Below 1.0µs: use shortdelay=0 (normal)
  // At or above 1.0µs: use shortdelay=1 (high delay)
  return (delay_us_x10 >= 10) ? 1 : 0;
}

} // namespace tmc51x0

#endif // TMC51X0_MOTOR_CALC_HPP
