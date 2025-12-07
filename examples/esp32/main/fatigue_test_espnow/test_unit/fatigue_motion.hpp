/**
 * @file fatigue_motion.hpp
 * @brief Unified fatigue test motion controller
 * 
 * Extracted and improved from fatigue_test_encoder.cpp and fatigue_test_stallguard.cpp
 * Provides sinusoidal back-and-forth motion between bounds for fatigue testing.
 */

#pragma once

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include <cstdint>
#include <cstdbool>
#include <cmath>

// Forward declarations
class Esp32TmcMutex;
class TmcMutexGuard;

namespace FatigueTest {

enum class AngleUnit { DEGREES, RADIANS };

/**
 * @brief Unified fatigue test motion controller
 * 
 * Provides pure sinusoidal back-and-forth motion between bounds for fatigue testing.
 * Supports global bounds (hardware limits) and local bounds (oscillation range).
 */
class FatigueTestMotion {
public:
    struct Status {
        bool running;
        bool bounded;
        float frequency_hz;
        float min_degrees_from_center;
        float max_degrees_from_center;
        uint32_t current_cycles;
        uint32_t target_cycles;
        uint32_t dwell_min_ms;
        uint32_t dwell_max_ms;
        float global_min_degrees;
        float global_max_degrees;
    };

    FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver) noexcept;
    ~FatigueTestMotion() noexcept;

    void SetGlobalBounds(float min_bound_degrees, float max_bound_degrees) noexcept;
    void GetGlobalBoundsDegrees(float& min_degrees, float& max_degrees) const noexcept;
    void SetUnbounded(float current_position_degrees, float default_range_degrees = 175.0f) noexcept;
    
    // Local bounds and motion parameters
    bool SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center) noexcept;
    void GetLocalBoundsFromCenterDegrees(float& min_degrees, float& max_degrees) const noexcept;
    bool SetFrequency(float frequency_hz) noexcept;
    float GetFrequency() const noexcept;
    bool SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms) noexcept;
    void GetDwellTimes(uint32_t& dwell_at_min_ms, uint32_t& dwell_at_max_ms) const noexcept;
    
    // Cycle control
    bool SetTargetCycles(uint32_t cycles) noexcept;
    uint32_t GetCurrentCycles() const noexcept;
    uint32_t GetTargetCycles() const noexcept;
    bool IsCycleComplete() const noexcept;
    void ResetCycles() noexcept;
    
    // Motion control
    bool Start() noexcept;
    void Stop() noexcept;
    bool IsRunning() const noexcept;
    void Update() noexcept;
    
    // Status
    Status GetStatus() const noexcept;
    float GetEstimatedFrequency() const noexcept;
    bool IsBounded() const noexcept;

private:
    tmc51x0::TMC51x0<Esp32SPI>* driver_;
    
    // Bounds (all in degrees)
    float global_min_bound_;
    float global_max_bound_;
    float local_min_bound_;
    float local_max_bound_;
    float home_position_;
    bool bounded_;
    
    // Motion parameters
    float amplitude_;
    float frequency_hz_;
    uint32_t dwell_at_min_ms_;
    uint32_t dwell_at_max_ms_;
    bool running_;
    uint32_t start_time_us_;
    float phase_offset_;
    
    // Cycle tracking
    uint32_t target_cycles_;
    uint32_t current_cycles_;
    bool cycle_complete_;
    bool last_was_negative_;
    bool cycle_started_;
    float last_target_relative_;
    
    // State machine
    enum class MotionState { MOVING_TO_MIN, MOVING_TO_MAX, DWELL_AT_MIN, DWELL_AT_MAX, STOPPED };
    MotionState state_;
    uint32_t dwell_start_time_ms_;
    bool sinusoidal_mode_;
    
    // Trajectory parameters (all in higher-level units)
    float calculated_vmax_rpm_;      // Maximum velocity in RPM
    float calculated_amax_rev_s2_;   // Maximum acceleration in rev/s²
    float estimated_frequency_hz_;   // Estimated actual frequency
    
    // Thread safety
    mutable Esp32TmcMutex mutex_;
    
    // Internal methods
    void RecalculateTrajectory() noexcept;
    void ClipLocalBoundsToGlobal() noexcept;
    void UpdateSinuousMotion() noexcept;
};

} // namespace FatigueTest
