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
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"

// ------------- ESPNOW CONFIG -------------

// Sync byte at start of every message
static constexpr uint8_t ESPNOW_SYNC_BYTE = 0xAA;

// Max payload bytes in our custom packet
static constexpr size_t ESPNOW_MAX_PAYLOAD = 48;

// WiFi/ESP-NOW channel
static constexpr uint8_t WIFI_CHANNEL = 1;

// ------------- MESSAGE TYPES -------------

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
    TEST_COMPLETE
};

// Status/state encoded in STATUS_UPDATE payload
enum class TestState : uint8_t {
    IDLE = 0,
    RUNNING,
    PAUSED,
    COMPLETED,
    ERROR
};

// ------------- PACKET STRUCTURES -------------

// Packet header with sync + type + id + length
#pragma pack(push, 1)
struct EspNowHeader {
    uint8_t sync;   // always ESPNOW_SYNC_BYTE
    uint8_t type;   // MsgType
    uint8_t id;     // sequence ID
    uint8_t len;    // payload length (0..ESPNOW_MAX_PAYLOAD)
};

// Full packet
struct EspNowPacket {
    EspNowHeader hdr;
    uint8_t      payload[ESPNOW_MAX_PAYLOAD];
    uint16_t     crc;  // CRC over [hdr + payload[0..len-1]]
};
#pragma pack(pop)

// Specific payload layouts
#pragma pack(push, 1)
struct ConfigPayload {
    uint32_t cycle_amount;
    uint32_t time_per_cycle_sec;
    uint32_t dwell_time_sec;
    uint8_t  bounds_method;      // 0 = stallguard, 1 = encoder
};

struct ConfigAckPayload {
    uint8_t ok;        // 1 = success, 0 = failure
    uint8_t err_code;  // optional
};

struct StatusPayload {
    uint32_t cycle_number;
    uint8_t  state;      // TestState
    uint8_t  err_code;   // if state == ERROR
};

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

// Events delivered to higher layers (UI)
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

struct ProtoEvent {
    ProtoEventType type;
    union {
        TestUnitSettings config;  // Only test unit settings in protocol events
        struct { uint32_t cycle; TestState state; uint8_t err_code; } status;
        struct { uint8_t err_code; uint32_t at_cycle; } error;
    } data;
};

// ------------- CRC16-CCITT FUNCTION -------------

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
