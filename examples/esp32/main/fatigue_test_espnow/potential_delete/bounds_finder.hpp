/**
 * @file bounds_finder.hpp
 * @brief Abstract interface for bounds finding strategies
 * 
 * Provides a unified interface for different bounds detection methods:
 * - StallGuard2 (sensorless)
 * - Encoder-based (position monitoring)
 */

#pragma once

#include "tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include <cstdint>
#include <memory>

namespace FatigueTest {

/**
 * @brief Bounds finding result
 * 
 * All bounds are stored in degrees relative to the current zero/home position.
 * The driver handles all step conversions internally.
 */
struct BoundsResult {
    bool success;
    float min_bound;  // Minimum bound in degrees
    float max_bound;  // Maximum bound in degrees
    bool bounded;     // Whether mechanical stops were found
    bool cancelled;   // Whether the operation was cancelled
    
    BoundsResult() : success(false), min_bound(0.0f), max_bound(0.0f), bounded(false), cancelled(false) {}
    BoundsResult(bool s, float min, float max, bool b, bool c = false)
        : success(s), min_bound(min), max_bound(max), bounded(b), cancelled(c) {}
};

/**
 * @brief Runtime configuration for bounds finding
 * 
 * When values are 0.0f, test config defaults are used.
 * This allows backward compatibility and sensible defaults.
 */
struct BoundsFinderConfig {
    float search_velocity_rpm = 0.0f;           // 0 = use test config default (BOUNDS_SEARCH_SPEED_RPM)
    float min_velocity_rpm = 0.0f;              // 0 = use test config default (MIN_VELOCITY_RPM)
    float current_factor = 0.0f;                // 0 = use test config default  
    float search_accel_rev_s2 = 0.0f;           // 0 = use test config default
};

/**
 * @brief Abstract bounds finder interface
 */
class IBoundsFinder {
public:
    virtual ~IBoundsFinder() = default;
    
    /**
     * @brief Find motor bounds in both directions
     * @param driver TMC51x0 driver instance
     * @param config Optional runtime configuration (nullptr = use test config defaults)
     * @return BoundsResult with found bounds in degrees
     */
    virtual BoundsResult FindBounds(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        const BoundsFinderConfig* config = nullptr,  // nullptr = use test config defaults
        const volatile bool* cancel = nullptr        // optional cancellation flag (true = cancel)
    ) = 0;
    
    /**
     * @brief Get method name for logging
     */
    virtual const char* GetMethodName() const = 0;
};

// Factory functions - template-based to automatically select test config based on test rig
template<tmc51x0_test_config::TestRigType test_rig>
std::unique_ptr<IBoundsFinder> CreateStallGuardBoundsFinder();

template<tmc51x0_test_config::TestRigType test_rig>
std::unique_ptr<IBoundsFinder> CreateEncoderBoundsFinder();

} // namespace FatigueTest
