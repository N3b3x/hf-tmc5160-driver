/**
 * @file fatigue_motion.hpp
 * @brief Unified fatigue test motion controller
 * 
 * Provides point-to-point back-and-forth motion between bounds for fatigue testing.
 * Uses direct VMAX/AMAX control like bounds_finding_test.cpp for smooth, predictable motion.
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
 * Provides point-to-point back-and-forth motion between bounds for fatigue testing.
 * Supports global bounds (hardware limits) and local bounds (oscillation range).
 */
class FatigueTestMotion {
public:
    struct Status {
        bool running;
        bool bounded;
        float vmax_rpm;                   // User-set max velocity (RPM)
        float amax_rev_s2;                // User-set acceleration (rev/s²)
        float estimated_frequency_hz;     // Derived cycle frequency (informational)
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
     * @param driver_mutex Reference to a mutex that protects ALL driver SPI access.
     *        This same mutex must be used by any code that directly accesses the driver.
     *
     * @note Thread safety: The provided mutex is used to serialize all SPI transactions.
     *       Callers MUST use the same mutex for any direct driver access outside this class.
     */
    FatigueTestMotion(tmc51x0::TMC51x0<Esp32SPI>* driver, Esp32TmcMutex& driver_mutex) noexcept;

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
     * Local bounds are clipped to global bounds (if bounded) and define the
     * oscillation range. An optional edge_backoff_deg parameter specifies
     * how far inside the mechanical bounds to stay during oscillation.
     *
     * @param min_degrees_from_center Minimum local bound.
     * @param max_degrees_from_center Maximum local bound.
     * @param edge_backoff_deg How far inside bounds to stay (default 3.5°, like bounds_finding_test).
     * @return true if accepted; false if rejected.
     */
    bool SetLocalBoundsFromCenterDegrees(float min_degrees_from_center, float max_degrees_from_center, 
                                         float edge_backoff_deg = 3.5f) noexcept;

    /**
     * @brief Read local bounds (degrees).
     * @param min_degrees Output min local bound.
     * @param max_degrees Output max local bound.
     */
    void GetLocalBoundsFromCenterDegrees(float& min_degrees, float& max_degrees) const noexcept;

    /**
     * @brief Set maximum velocity for oscillation (direct TMC5160 VMAX control).
     * @param vmax_rpm Maximum velocity in RPM.
     * @return true if accepted (value clamped to safe limits).
     */
    bool SetMaxVelocity(float vmax_rpm) noexcept;

    /**
     * @brief Get configured maximum velocity (RPM).
     */
    float GetMaxVelocity() const noexcept;

    /**
     * @brief Set acceleration for oscillation (direct TMC5160 AMAX control).
     * @param amax_rev_s2 Acceleration in rev/s².
     * @return true if accepted (value clamped to safe limits).
     */
    bool SetAcceleration(float amax_rev_s2) noexcept;

    /**
     * @brief Get configured acceleration (rev/s²).
     */
    float GetAcceleration() const noexcept;

    /**
     * @brief Get estimated cycle frequency based on current velocity/accel/distance.
     * @return Estimated frequency in Hz (informational only).
     * @note This is derived from the motion parameters, not a setpoint.
     */
    float GetEstimatedCycleFrequency() const noexcept;

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
     * @brief Pause active motion (stops movement but keeps motor energized for resume).
     * @details Sets ramp mode to HOLD to maintain position. Motion can be resumed with Start().
     */
    void Pause() noexcept;

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
    
    // Motion parameters (directly user-controlled)
    float amplitude_;
    float vmax_rpm_;                 // User-set max velocity (RPM) - direct to TMC5160
    float amax_rev_s2_;              // User-set acceleration (rev/s²) - direct to TMC5160
    uint32_t dwell_at_min_ms_;
    uint32_t dwell_at_max_ms_;
    bool running_;
    uint64_t start_time_us_;
    
    // Cycle tracking
    uint32_t target_cycles_;
    uint32_t current_cycles_;
    bool cycle_complete_;
    
    // State machine for point-to-point motion
    enum class MotionState { MOVING_TO_MIN, MOVING_TO_MAX, DWELL_AT_MIN, DWELL_AT_MAX, PAUSED, STOPPED };
    MotionState state_;
    uint32_t dwell_start_time_ms_;
    
    // Derived parameters (calculated from user inputs)
    float estimated_frequency_hz_;   // Estimated cycle frequency based on vmax/amax/distance
    
    // Thread safety - reference to external mutex that protects ALL driver SPI access
    Esp32TmcMutex& driver_mutex_;
    
    // Internal methods
    void RecalculateEstimatedFrequency() noexcept;
    void ClipLocalBoundsToGlobal() noexcept;
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
