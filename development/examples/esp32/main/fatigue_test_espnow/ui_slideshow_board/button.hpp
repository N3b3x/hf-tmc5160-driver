/**
 * @file button.hpp
 * @brief Button handling for slideshow board
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

enum class SlideshowButtonId {
    UP,      // Previous image
    SELECT,  // Toggle auto-advance / Favorite
    DOWN     // Next image
};

struct SlideshowButtonEvent {
    SlideshowButtonId id;
    bool pressed;  // true on press, false on release
};

namespace SlideshowButtons {

bool init(QueueHandle_t evt_queue);
void configure_wakeup();  // for deep sleep wake

} // namespace SlideshowButtons

