/**
 * @file espnow_protocol.hpp
 * @brief ESP-NOW communication protocol for fatigue tester
 * 
 * Defines message types, packet structures, and protocol handlers
 * for communication between UI board (remote controller) and test unit.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_idf_pedantic_compat.hpp"
#include "esp_log.h"

// ------------- ESPNOW CONFIG -------------

// Sync byte at start of every message
static constexpr uint8_t ESPNOW_SYNC_BYTE = 0xAA;

// Max payload bytes in our custom packet
static constexpr size_t ESPNOW_MAX_PAYLOAD = 48;

// WiFi/ESP-NOW channel
static constexpr uint8_t WIFI_CHANNEL = 1;

// Remote controller (UI board) MAC address (STA interface).
// If set non-zero, the test unit can send responses immediately (without waiting
// to "learn" the sender MAC from the first inbound ESPNOW message).
//
// Update this when the remote controller prints its MAC, e.g.:
//   Remote Controller MAC (STA): 9C:9E:6E:77:24:F8
static constexpr uint8_t UI_BOARD_MAC_[6] = { 0x9C, 0x9E, 0x6E, 0x77, 0x24, 0xF8 };

// ------------- MESSAGE TYPES -------------

/**
 * @brief Wire-level message type identifiers.
 *
 * @details
 * These values are serialized into `EspNowHeader::type` and must match on both
 * the test unit and the remote controller.
 */
enum class MsgType : uint8_t {
    CONFIG_REQUEST = 1,
    CONFIG_RESPONSE,
    CONFIG_SET,
    CONFIG_ACK,
    START,
    START_ACK,
    PAUSE,
    PAUSE_ACK,
    RESUME,
    RESUME_ACK,
    STOP,
    STOP_ACK,
    STATUS_UPDATE,
    ERROR,
    TEST_COMPLETE,
    ERROR_CLEAR = 16   // Error clear message
};

/**
 * @brief Protocol-visible test states encoded in `STATUS_UPDATE`.
 */
enum class TestState : uint8_t {
    IDLE = 0,
    RUNNING,
    PAUSED,
    COMPLETED,
    ERROR
};

// ------------- PACKET STRUCTURES -------------

/**
 * @brief ESP-NOW packet header (wire format).
 *
 * @details
 * This header is followed by `len` bytes of payload and then a 16-bit CRC.
 * CRC is computed over `hdr + payload[0..len-1]` (CRC field excluded).
 */
#pragma pack(push, 1)
struct EspNowHeader {
    uint8_t sync;   // always ESPNOW_SYNC_BYTE
    uint8_t type;   // MsgType
    uint8_t id;     // sequence ID
    uint8_t len;    // payload length (0..ESPNOW_MAX_PAYLOAD)
};

/**
 * @brief Full packet representation (header + max payload + CRC field).
 *
 * @note On the wire, payload length is `hdr.len` and CRC is located at
 * offset `sizeof(EspNowHeader) + hdr.len`. Do not assume the `crc` field in this
 * struct aligns with the received buffer for shorter payloads.
 */
struct EspNowPacket {
    EspNowHeader hdr;
    uint8_t      payload[ESPNOW_MAX_PAYLOAD];
    uint16_t     crc;  // CRC over [hdr + payload[0..len-1]]
};
#pragma pack(pop)

/**
 * @brief Payload for CONFIG_SET / CONFIG_RESPONSE.
 */
#pragma pack(push, 1)
struct ConfigPayload {
    uint32_t cycle_amount;
    uint32_t time_per_cycle_sec;
    uint32_t dwell_time_sec;
    uint8_t  bounds_method;      // 0 = stallguard, 1 = encoder
    float    bounds_search_velocity_rpm;       // New - search speed during bounds finding
    float    stallguard_min_velocity_rpm;      // New - minimum velocity threshold for StallGuard2
    float    stall_detection_current_factor;  // New - current reduction factor
    float    bounds_search_accel_rev_s2;       // New - acceleration during bounds finding
};

/**
 * @brief Payload for CONFIG_ACK.
 */
struct ConfigAckPayload {
    uint8_t ok;        // 1 = success, 0 = failure
    uint8_t err_code;  // optional
};

/**
 * @brief Payload for STATUS_UPDATE.
 */
struct StatusPayload {
    uint32_t cycle_number;
    uint8_t  state;      // TestState
    uint8_t  err_code;   // if state == ERROR
};

/**
 * @brief Payload for ERROR.
 */
struct ErrorPayload {
    uint8_t  err_code;
    uint32_t at_cycle;
};
#pragma pack(pop)

// ------------- SETTINGS STRUCTURE -------------

/**
 * @brief Test unit settings - synchronized with test machine via ESP-NOW
 * These settings control the fatigue test behavior
 */
struct TestUnitSettings {
    uint32_t cycle_amount   = 1000;
    uint32_t time_per_cycle = 1;    // seconds
    uint32_t dwell_time     = 1;    // seconds
    bool     bounds_method_stallguard = true; // true = stallguard, false = encoder
    
    // Stall detection configuration (configurable via remote controller)
    float    bounds_search_velocity_rpm = 0.0f;          // 0 = use test config default (BOUNDS_SEARCH_SPEED_RPM)
    float    stallguard_min_velocity_rpm = 0.0f;          // 0 = use test config default (MIN_VELOCITY_RPM/TCOOLTHRS)
    float    stall_detection_current_factor = 0.0f;      // 0 = use test config default
    float    bounds_search_accel_rev_s2 = 0.0f;          // 0 = use test config default
};

/**
 * @brief UI board settings - stored locally, never synchronized with test unit
 * These settings control the UI board's display and behavior
 */
struct UISettings {
    bool orientation_flipped = false;
    // Future UI settings can be added here (e.g., brightness, contrast, etc.)
};

/**
 * @brief Complete settings structure containing both test unit and UI settings
 * This is the main settings structure used throughout the application
 */
struct Settings {
    TestUnitSettings test_unit;  // Test machine settings (synced via ESP-NOW)
    UISettings        ui;         // UI board settings (local only)
};

// ------------- EVENTS -------------

/**
 * @brief Higher-level events emitted by the protocol layer.
 *
 * @details
 * On the test-unit side, these events represent commands received from the UI board.
 * On either side, these can also represent parsed status/response semantics.
 */
enum class ProtoEventType {
    // Incoming commands from UI board (test unit side)
    CONFIG_REQUEST,
    CONFIG_SET,
    START,
    PAUSE,
    RESUME,
    STOP,
    // Status/response events (both sides)
    CONFIG_UPDATED,
    CONFIG_APPLY_OK,
    CONFIG_APPLY_FAIL,
    STARTED,
    PAUSED,
    RESUMED,
    STOPPED,
    STATUS,
    ERROR_EVENT,
    TEST_COMPLETED
};

/**
 * @brief Protocol event structure pushed through FreeRTOS queues.
 *
 * @details The active union member depends on `type`.
 */
struct ProtoEvent {
    ProtoEventType type;
    union {
        TestUnitSettings config;  // Only test unit settings in protocol events (includes new float fields)
        struct { uint32_t cycle; TestState state; uint8_t err_code; } status;
        struct { uint8_t err_code; uint32_t at_cycle; } error;
    } data;
};

// ------------- CRC16-CCITT FUNCTION -------------

/**
 * @brief Compute CRC16-CCITT (poly 0x1021, init 0xFFFF).
 *
 * @param data Data buffer.
 * @param len Number of bytes to include.
 * @return Computed CRC16 value.
 */
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}
