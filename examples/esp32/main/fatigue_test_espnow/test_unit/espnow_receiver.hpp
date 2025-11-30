/**
 * @file espnow_receiver.hpp
 * @brief ESP-NOW receiver for test unit
 */

#pragma once

#include "../espnow_protocol.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace EspNowReceiver {

bool init(QueueHandle_t event_queue);
bool send_config_response(const Settings& s);
bool send_config_ack(bool ok, uint8_t err_code = 0);
bool send_start_ack();
bool send_pause_ack();
bool send_resume_ack();
bool send_stop_ack();
bool send_status_update(uint32_t cycle, TestState state, uint8_t err_code = 0);
bool send_error(uint8_t err_code, uint32_t at_cycle);
bool send_test_complete();

} // namespace EspNowReceiver
