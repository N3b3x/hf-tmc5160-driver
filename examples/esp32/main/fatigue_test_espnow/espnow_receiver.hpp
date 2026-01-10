/**
 * @file espnow_receiver.hpp
 * @brief ESP-NOW receiver for test unit with secure pairing support
 * 
 * Features:
 * - Pre-configured MAC address support (backward compatibility)
 * - Secure pairing with HMAC mutual authentication
 * - NVS-based approved peer storage
 * - Message validation against approved peer list
 */

#pragma once

#include "espnow_protocol.hpp"
#include "espnow_security.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace EspNowReceiver {

/**
 * @brief Initialize ESP-NOW (WiFi STA + ESP-NOW) and start the receive task.
 *
 * @details
 * - Initializes NVS/WiFi/ESP-NOW.
 * - Registers RX/TX callbacks.
 * - Creates an internal queue for raw RX frames and starts an internal task that
 *   parses/validates packets and pushes `ProtoEvent` messages to `event_queue`.
 *
 * @param event_queue FreeRTOS queue that receives parsed `ProtoEvent` values.
 * @return true on successful initialization; false otherwise.
 */
bool init(QueueHandle_t event_queue);

/**
 * @brief Send CONFIG_RESPONSE containing the current settings.
 * @param s Current settings (test unit portion is serialized to wire).
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_config_response(const Settings& s);

/**
 * @brief Send CONFIG_ACK for a CONFIG_SET request.
 * @param ok true if configuration was applied; false if rejected.
 * @param err_code Optional error code (meaning is application-defined).
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_config_ack(bool ok, uint8_t err_code = 0);

/**
 * @brief Send START_ACK (acknowledge START receipt).
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_start_ack();

/**
 * @brief Send PAUSE_ACK (acknowledge PAUSE receipt).
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_pause_ack();

/**
 * @brief Send RESUME_ACK (acknowledge RESUME receipt).
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_resume_ack();

/**
 * @brief Send STOP_ACK (acknowledge STOP receipt).
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_stop_ack();

/**
 * @brief Send STATUS_UPDATE with cycle count and state.
 * @param cycle Current cycle count.
 * @param state Current protocol state.
 * @param err_code Optional error code (used when state == ERROR).
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_status_update(uint32_t cycle, TestState state, uint8_t err_code = 0);

/**
 * @brief Send ERROR message.
 * @param err_code Application-defined error code.
 * @param at_cycle Cycle count at which the error was observed.
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_error(uint8_t err_code, uint32_t at_cycle);

/**
 * @brief Send TEST_COMPLETE message.
 * @return true if the packet was queued for transmission; false otherwise.
 */
bool send_test_complete();

// ============================================================================
// PAIRING FUNCTIONS
// ============================================================================

/**
 * @brief Enter pairing mode for the specified duration.
 * 
 * While in pairing mode, the test unit will respond to PairingRequest
 * messages from remote controllers. After the timeout expires, pairing
 * mode is automatically disabled.
 * 
 * @param timeout_sec Duration of pairing mode in seconds (default: 30)
 */
void enter_pairing_mode(uint32_t timeout_sec = PAIRING_MODE_TIMEOUT_SEC);

/**
 * @brief Exit pairing mode immediately.
 */
void exit_pairing_mode();

/**
 * @brief Check if device is currently in pairing mode.
 * @return true if in pairing mode
 */
bool is_in_pairing_mode();

/**
 * @brief Get access to the security settings for peer management.
 * 
 * Use this with PeerStore functions to manage approved peers.
 * 
 * @return Reference to internal SecuritySettings
 */
SecuritySettings& get_security_settings();

/**
 * @brief Manually add a peer as approved (bypasses pairing).
 * 
 * Useful for adding pre-configured peers or debugging.
 * 
 * @param mac Peer's MAC address
 * @param type Peer's device type
 * @param name Human-readable name
 * @return true if added successfully
 */
bool add_approved_peer(const uint8_t mac[6], DeviceType type, const char* name);

/**
 * @brief Remove a peer from the approved list.
 * 
 * @param mac Peer's MAC address
 * @return true if removed
 */
bool remove_approved_peer(const uint8_t mac[6]);

/**
 * @brief Get the number of approved peers.
 * @return Number of approved peers (including pre-configured)
 */
size_t get_approved_peer_count();

} // namespace EspNowReceiver
