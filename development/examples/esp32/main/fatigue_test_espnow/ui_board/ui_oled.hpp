/**
 * @file ui_oled.hpp
 * @brief UI implementation for OLED display with rotary encoder
 * 
 * Replaces e-ink display UI with:
 * - 1.3" SH1106 OLED display (128x64, I2C)
 * - EC11 rotary encoder for navigation
 * - BACK and CONFIRM buttons for actions
 * 
 * Maintains compatibility with existing Settings structure and ESP-NOW protocol.
 */

#pragma once

#include "ui.hpp"  // Reuse UiState, UiEvent, etc.
#include "oled_menu.hpp"
#include "../../components/Adafruit_SH1106_ESPIDF/Adafruit_SH1106.h"
#include "../../components/EC11_Encoder/inc/ec11_encoder.hpp"
#include "config.hpp"
#include "settings.hpp"
#include "button.hpp"
#include "espnow_protocol.hpp"

namespace UI_OLED {

void init(QueueHandle_t ui_queue, Settings* settings, uint32_t* inactivity_ticks_ptr);
void task(void* arg);

} // namespace UI_OLED

