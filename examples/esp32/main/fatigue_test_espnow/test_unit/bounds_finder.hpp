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
#include <cstdint>
#include <memory>

namespace FatigueTest {

/**
 * @brief Bounds finding result
 */
struct BoundsResult {
    bool success;
    int32_t min_bound;  // Minimum bound in steps
    int32_t max_bound;  // Maximum bound in steps
    bool bounded;       // Whether mechanical stops were found
    
    BoundsResult() : success(false), min_bound(0), max_bound(0), bounded(false) {}
    BoundsResult(bool s, int32_t min, int32_t max, bool b) 
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
     * @param steps_per_rev Steps per revolution for unit conversion
     * @return BoundsResult with found bounds
     */
    virtual BoundsResult FindBounds(
        tmc51x0::TMC51x0<Esp32SPI>& driver,
        uint16_t steps_per_rev
    ) = 0;
    
    /**
     * @brief Get method name for logging
     */
    virtual const char* GetMethodName() const = 0;
};

// Factory functions
std::unique_ptr<IBoundsFinder> CreateStallGuardBoundsFinder();
std::unique_ptr<IBoundsFinder> CreateEncoderBoundsFinder();

} // namespace FatigueTest
