/**
 * @file ui.hpp
 * @brief UI state machine for e-ink display
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "../espnow_protocol.hpp"
#include "button.hpp"

// UI states
enum class UiState {
    MAIN,
    SETTINGS_MENU,          // Settings menu list
    SETTINGS_EDIT_CYCLES,   // Editing cycle amount
    SETTINGS_EDIT_TIME,     // Editing time per cycle
    SETTINGS_EDIT_DWELL,    // Editing dwell time
    SETTINGS_EDIT_METHOD,   // Editing bounds method
    SETTINGS_EDIT_ORIENT,   // Editing orientation
    CONFIRM_STOP,
    CONFIRM_POPUP,          // Generic confirmation popup
    ERROR_SCREEN,
    RUNNING,
    PAUSED,
    COMPLETE
};

enum class UiEventType {
    BTN,
    PROTO,
    TIMER_INACTIVITY
};

struct UiEvent {
    UiEventType type;
    union {
        ButtonEvent btn;
        ProtoEvent  proto;
    } data;
};

namespace UI {

void init(QueueHandle_t ui_queue, Settings* settings, uint32_t* inactivity_ticks_ptr);
void task(void*);

} // namespace UI
