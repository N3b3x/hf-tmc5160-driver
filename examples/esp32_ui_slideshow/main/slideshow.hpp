/**
 * @file slideshow.hpp
 * @brief Main slideshow logic and state machine
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <vector>
#include <string>

// Forward declarations
class Adafruit_IL0373;

namespace Slideshow {

/**
 * @brief Slideshow states
 */
enum class State {
    INIT,           // Initializing
    SCANNING,       // Scanning SD card for images
    DISPLAYING,     // Displaying current image
    AUTO_ADVANCE,   // Auto-advancing through images
    ERROR,          // Error state
    SLEEPING        // Deep sleep (inactivity)
};

/**
 * @brief Initialize slideshow system
 * @return true if successful
 */
bool init();

/**
 * @brief Main slideshow task
 * @param arg Task argument (unused)
 */
void task(void* arg);

/**
 * @brief Get current state
 * @return Current slideshow state
 */
State getState();

/**
 * @brief Get current image index
 * @return Current image index (0-based)
 */
size_t getCurrentImageIndex();

/**
 * @brief Get total number of images
 * @return Total image count
 */
size_t getImageCount();

} // namespace Slideshow

