/**
 * @file fatigue_motion.hpp
 * @brief Unified fatigue test motion controller
 * 
 * Extracted and improved from fatigue_test_encoder.cpp and fatigue_test_stallguard.cpp
 * Provides sinusoidal back-and-forth motion between bounds for fatigue testing.
 */

#pragma once

#include "tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc_mutex.hpp"  // Required for Esp32TmcMutex member variable
#include <cstdint>
#include <cstdbool>
#include <cmath>

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

    /**
     * @brief Construct a motion controller bound to a TMC51x0 driver instance.
     *
     * @param driver Pointer to an initialized driver (must remain valid for the
     * lifetime of this object).
     *
     * @note This class uses internal locking (`Esp32TmcMutex`) to serialize access.
     */
    FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver) noexcept;

    /**
     * @brief Destructor.
     *
     * @note Does not implicitly stop the motor; callers should call Stop() as part
     * of application shutdown/state transitions.
     */
    ~FatigueTestMotion() noexcept;

    /**
     * @brief Set global (hardware) bounds in degrees.
     *
     * @details
     * These bounds represent the full allowed travel range and are typically
     * produced by bounds finding.
     *
     * @param min_bound_degrees Minimum bound (degrees).
     * @param max_bound_degrees Maximum bound (degrees).
     */
    void SetGlobalBounds(float min_bound_degrees, float max_bound_degrees) noexcept;

    /**
     * @brief Read current global bounds (degrees).
     * @param min_degrees Output min bound.
     * @param max_degrees Output max bound.
     */
    void GetGlobalBoundsDegrees(float& min_degrees, float& max_degrees) const noexcept;

    /**
     * @brief Mark the system as unbounded and establish a default travel window.
     *
     * @details
     * Used when no mechanical stops are detected. Sets a default global range
     * around the current position and establishes home/zero at the current position.
     *
     * @param current_position_degrees Current position (degrees).
     * @param default_range_degrees Default travel range to assume (degrees).
     */
    void SetUnbounded(float current_position_degrees, float default_range_degrees = 175.0f) noexcept;
    
    // Local bounds and motion parameters
    /**
     * @brief Set local oscillation bounds (degrees) relative to the center.
     *
     * @details
     * Local bounds are clipped to global bounds (if bounded) and used as the
     * sinusoidal target range.
     *
     * @param min_degrees_from_center Minimum local bound.
     * @param max_degrees_from_center Maximum local bound.
     * @return true if accepted; false if rejected.
     */
    bool SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center) noexcept;

    /**
     * @brief Read local bounds (degrees).
     * @param min_degrees Output min local bound.
     * @param max_degrees Output max local bound.
     */
    void GetLocalBoundsFromCenterDegrees(float& min_degrees, float& max_degrees) const noexcept;

    /**
     * @brief Set target oscillation frequency (Hz).
     * @param frequency_hz Frequency in Hz.
     * @return true if accepted; false otherwise.
     */
    bool SetFrequency(float frequency_hz) noexcept;

    /**
     * @brief Get configured frequency (Hz).
     */
    float GetFrequency() const noexcept;

    /**
     * @brief Set dwell times at local bounds.
     * @param dwell_at_min_ms Dwell at min (ms).
     * @param dwell_at_max_ms Dwell at max (ms).
     * @return true if accepted; false otherwise.
     */
    bool SetDwellTimes(uint32_t dwell_at_min_ms, uint32_t dwell_at_max_ms) noexcept;

    /**
     * @brief Get dwell times at local bounds.
     * @param dwell_at_min_ms Output dwell at min (ms).
     * @param dwell_at_max_ms Output dwell at max (ms).
     */
    void GetDwellTimes(uint32_t& dwell_at_min_ms, uint32_t& dwell_at_max_ms) const noexcept;
    
    // Cycle control
    /**
     * @brief Set target cycle count (0 = infinite).
     * @param cycles Target cycles.
     * @return true if accepted.
     */
    bool SetTargetCycles(uint32_t cycles) noexcept;

    /**
     * @brief Get current completed cycles.
     */
    uint32_t GetCurrentCycles() const noexcept;

    /**
     * @brief Get configured target cycles.
     */
    uint32_t GetTargetCycles() const noexcept;

    /**
     * @brief Check whether the configured target cycle count has been reached.
     */
    bool IsCycleComplete() const noexcept;

    /**
     * @brief Reset cycle counter and completion flags.
     */
    void ResetCycles() noexcept;
    
    // Motion control
    /**
     * @brief Start active motion.
     *
     * @details
     * Requires local bounds to be set. Configures the driver for positioning mode.
     *
     * @return true on success; false on failure (e.g., bounds not set, driver not standstill).
     */
    bool Start() noexcept;

    /**
     * @brief Stop active motion and command driver stop.
     */
    void Stop() noexcept;

    /**
     * @brief Whether the motion controller is currently active.
     */
    bool IsRunning() const noexcept;

    /**
     * @brief Periodic update tick.
     *
     * @details Intended to be called from a periodic task. Generates and applies
     * updated target positions and handles dwell/cycle completion logic.
     */
    void Update() noexcept;
    
    // Status
    /**
     * @brief Snapshot current controller status.
     */
    Status GetStatus() const noexcept;

    /**
     * @brief Get estimated realized frequency (Hz) based on dwell + trajectory.
     */
    float GetEstimatedFrequency() const noexcept;

    /**
     * @brief Whether the controller is in bounded mode (mechanical stops found).
     */
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

// Include implementation
// Note: esp32_tmc_mutex.hpp is already included above (required for Esp32TmcMutex member variable)
#ifndef FATIGUE_MOTION_HEADER_INCLUDED
#define FATIGUE_MOTION_HEADER_INCLUDED
// NOLINTNEXTLINE(bugprone-suspicious-include) - Intentional: implementation file
#include "fatigue_motion_impl.hpp"
#undef FATIGUE_MOTION_HEADER_INCLUDED
#endif // FATIGUE_MOTION_HEADER_INCLUDED
