/**
 * @file button.hpp
 * @brief Button handling for UI board
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

enum class ButtonId {
    UP,        // Legacy: for e-ink slideshow
    SELECT,    // Legacy: for e-ink slideshow
    DOWN,      // Legacy: for e-ink slideshow
    BACK,      // NEW: Navigate back
    CONFIRM    // NEW: Critical action confirmation
};

struct ButtonEvent {
    ButtonId id;
};

namespace Buttons {

bool init(QueueHandle_t evt_queue);
void configure_wakeup();  // for deep sleep wake

} // namespace Buttons
