/**
 * @file bounds_finder.hpp
 * @brief Abstract interface for bounds finding strategies
 * 
 * Provides a unified interface for different bounds detection methods:
 * - StallGuard2 (sensorless)
 * - Encoder-based (position monitoring)
 */

#pragma once

#include "../../../inc/tmc51x0.hpp"
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
    
    BoundsResult() : success(false), min_bound(0.0f), max_bound(0.0f), bounded(false) {}
    BoundsResult(bool s, float min, float max, bool b) 
        : success(s), min_bound(min), max_bound(max), bounded(b) {}
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
     * @return BoundsResult with found bounds in degrees
     */
    virtual BoundsResult FindBounds(
        tmc51x0::TMC51x0<Esp32SPI>& driver
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
