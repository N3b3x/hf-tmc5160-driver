/**
 * @file espnow_protocol.hpp
 * @brief ESP-NOW protocol API for UI board
 */

#pragma once

#include "../espnow_protocol.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace EspNowProto {

bool init(QueueHandle_t event_queue);
bool send_config_request();
bool send_config_set(const Settings& s);
bool send_start();
bool send_pause();
bool send_resume();
bool send_stop();

} // namespace EspNowProto
