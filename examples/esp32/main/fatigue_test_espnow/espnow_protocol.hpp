/**
 * @file espnow_protocol.hpp
 * @brief ESP-NOW communication protocol for fatigue tester
 * 
 * Protocol compatible with esp32_remote_controller.
 * Uses 6-byte header with version and device_id fields.
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

// Include security definitions for pairing support
#include "espnow_security.hpp"

// ------------- ESPNOW CONFIG -------------

// Sync byte at start of every message
static constexpr uint8_t ESPNOW_SYNC_BYTE = 0xAA;

// Protocol version (must match remote controller)
static constexpr uint8_t ESPNOW_PROTOCOL_VERSION = 1;

// Device ID for this fatigue tester (must match remote controller device registry)
static constexpr uint8_t DEVICE_ID_FATIGUE_TESTER = 1;

// Max payload bytes in our custom packet
static constexpr size_t ESPNOW_MAX_PAYLOAD = 200;

// WiFi/ESP-NOW channel
static constexpr uint8_t WIFI_CHANNEL = 1;

// Remote controller (UI board) MAC address (STA interface).
// If set non-zero, the test unit can send responses immediately (without waiting
// to "learn" the sender MAC from the first inbound ESPNOW message).
//
// Update this when the remote controller prints its MAC, e.g.:
//   Remote Controller MAC (STA): B0:81:84:96:90:10
static constexpr uint8_t UI_BOARD_MAC[6] = { 0xB0, 0x81, 0x84, 0x96, 0x90, 0x10 };

// ------------- MESSAGE TYPES -------------

/**
 * @brief Wire-level message type identifiers.
 *
 * @details
 * These values are serialized into `EspNowHeader::type` and must match
 * the remote controller's espnow::MsgType enum exactly.
 * 
 * Per coding standards: PascalCase for enum values (state/error types).
 */
enum class MsgType : uint8_t {
    DeviceDiscovery = 1,
    DeviceInfo,
    ConfigRequest,     // = 3
    ConfigResponse,    // = 4
    ConfigSet,         // = 5
    ConfigAck,         // = 6
    Command,           // = 7
    CommandAck,        // = 8
    StatusUpdate,      // = 9
    Error,             // = 10
    ErrorClear,        // = 11
    TestComplete,      // = 12
    
        // Fatigue-test extensions
        BoundsResult      = 13,
    
    // Security / Pairing messages (20-29 range)
    PairingRequest  = 20,   ///< Initiate pairing (broadcast)
    PairingResponse = 21,   ///< Response with HMAC proof
    PairingConfirm  = 22,   ///< Final confirmation
    PairingReject   = 23,   ///< Explicit rejection
    Unpair          = 24,   ///< Remove a paired device
};

/**
 * @brief Protocol-visible test states encoded in `STATUS_UPDATE`.
 * 
 * Per coding standards: PascalCase for enum values (state types).
 */
enum class TestState : uint8_t {
    Idle = 0,
    Running,
    Paused,
    Completed,
    Error
};

/**
 * @brief Command IDs for COMMAND message type.
 * 
 * Per coding standards: PascalCase for enum values.
 */
enum class CommandId : uint8_t {
    Start = 1,
    Pause = 2,
    Resume = 3,
    Stop = 4,
    // Dedicated bounds-finding command (independent of starting the test).
    RunBoundsFinding = 5,
};

    /**
     * @brief Payload for BOUNDS_RESULT.
     *
     * @details
     * Sends the bounds relative to the established center/home (degrees).
     * `min_degrees_from_center` is typically negative and `max_degrees_from_center`
     * is typically positive.
     */
    #pragma pack(push, 1)
    struct BoundsResultPayload {
        uint8_t ok;        ///< 1=success (bounded or intentionally unbounded default); 0=failure
        uint8_t bounded;   ///< 1=mechanical stops detected; 0=unbounded default window used
        uint8_t cancelled; ///< 1=cancelled by user/STOP/PAUSE
        uint8_t reserved;
        float   min_degrees_from_center;
        float   max_degrees_from_center;
        float   global_min_degrees;
        float   global_max_degrees;
    };
    #pragma pack(pop)

// ------------- PACKET STRUCTURES -------------

/**
 * @brief ESP-NOW packet header (wire format) - 6 bytes.
 *
 * @details
 * This header matches the remote controller's protocol:
 * - sync: sync byte (0xAA)
 * - version: protocol version
 * - device_id: device type ID
 * - type: MsgType
 * - id: sequence ID
 * - len: payload length
 * 
 * CRC is computed over `hdr + payload[0..len-1]` (CRC field excluded).
 */
#pragma pack(push, 1)
struct EspNowHeader {
    uint8_t sync;       // always ESPNOW_SYNC_BYTE
    uint8_t version;    // protocol version
    uint8_t device_id;  // device type ID
    uint8_t type;       // MsgType
    uint8_t id;         // sequence ID
    uint8_t len;        // payload length (0..ESPNOW_MAX_PAYLOAD)
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
 * 
 * @note Must match remote controller's FatigueTestConfigPayload structure.
 * Extended fields are optional - older remote controllers may send only base bytes.
 * 
 * PROTOCOL V2 CHANGE: Replaced time_per_cycle_sec with oscillation_vmax_rpm and
 * oscillation_amax_rev_s2 for direct TMC5160 ramp control.
 */
#pragma pack(push, 1)
struct ConfigPayload {
    // Base fields (17 bytes) - required, always present
    uint32_t cycle_amount;                     // Target number of cycles (0 = infinite)
    float    oscillation_vmax_rpm;             // Max oscillation velocity (RPM) - directly to TMC5160 VMAX
    float    oscillation_amax_rev_s2;          // Oscillation acceleration (rev/s²) - directly to TMC5160 AMAX
    uint32_t dwell_time_ms;                    // Dwell time at endpoints (milliseconds)
    uint8_t  bounds_method;                    // 0 = stallguard, 1 = encoder
    
    // Extended fields (16 bytes) - optional, for bounds finding configuration
    float    bounds_search_velocity_rpm;       // Search speed during bounds finding (RPM)
    float    stallguard_min_velocity_rpm;      // Minimum velocity threshold for StallGuard2 (RPM)
    float    stall_detection_current_factor;   // Current reduction factor (0.0-1.0)
    float    bounds_search_accel_rev_s2;       // Acceleration during bounds finding (rev/s²)

    // Extended v2 field (optional)
    // StallGuard threshold (SGT). Valid range is typically [-64, 63].
    // 127 means "use test config default".
    int8_t   stallguard_sgt;
};

/**
 * @brief Payload for CONFIG_ACK.
 */
struct ConfigAckPayload {
    uint8_t ok;        // 1 = success, 0 = failure
    uint8_t err_code;  // optional error code
};

/**
 * @brief Payload for COMMAND message.
 */
struct CommandPayload {
    uint8_t command_id;  // CommandId enum value
};

/**
 * @brief Payload for STATUS_UPDATE.
 */
struct StatusPayload {
    uint32_t cycle_number;
    uint8_t  state;      // TestState enum value
    uint8_t  err_code;   // error code if state == Error
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
 * @brief Test unit settings - synchronized with test machine via ESP-NOW.
 * 
 * These settings control the fatigue test behavior.
 * 
 * PROTOCOL V2: Uses direct velocity/acceleration control instead of cycle time.
 */
struct TestUnitSettings {
    uint32_t cycle_amount = 1000;                     // Target cycles (0 = infinite)
    float    oscillation_vmax_rpm = 60.0f;            // Max velocity during oscillation (RPM)
    float    oscillation_amax_rev_s2 = 10.0f;         // Acceleration during oscillation (rev/s²)
    uint32_t dwell_time_ms = 1000;                    // Dwell at endpoints (ms)
    bool     bounds_method_stallguard = true;         // true = stallguard, false = encoder
    
    // Extended configuration for bounds finding (configurable via remote controller)
    float    bounds_search_velocity_rpm = 0.0f;       // 0 = use test config default
    float    stallguard_min_velocity_rpm = 0.0f;      // 0 = use test config default
    float    stall_detection_current_factor = 0.0f;   // 0 = use test config default
    float    bounds_search_accel_rev_s2 = 0.0f;       // 0 = use test config default

    // StallGuard threshold (SGT).
    int8_t   stallguard_sgt = 127;                    // 127 = use test config default
};

/**
 * @brief UI board settings - stored locally, never synchronized with test unit.
 * 
 * These settings control the UI board's display and behavior.
 */
struct UISettings {
    bool orientation_flipped = false;
    // Future UI settings can be added here (e.g., brightness, contrast, etc.)
};

/**
 * @brief Complete settings structure containing both test unit and UI settings.
 * 
 * This is the main settings structure used throughout the application.
 */
struct Settings {
    TestUnitSettings test_unit;  // Test machine settings (synced via ESP-NOW)
    UISettings       ui;         // UI board settings (local only)
};

// ------------- EVENTS -------------

/**
 * @brief Higher-level events emitted by the protocol layer.
 *
 * @details
 * On the test-unit side, these events represent commands received from the UI board.
 * On either side, these can also represent parsed status/response semantics.
 * 
 * Per coding standards: PascalCase for enum values (event types).
 */
enum class ProtoEventType {
    // Incoming commands from UI board (test unit side)
    ConfigRequest,
    ConfigSet,
    CommandStart,
    CommandPause,
    CommandResume,
    CommandStop,
        CommandRunBoundsFinding,
    // Status/response events (both sides)
    ConfigUpdated,
    ConfigApplyOk,
    ConfigApplyFail,
    Started,
    Paused,
    Resumed,
    Stopped,
    Status,
    ErrorEvent,
    TestCompleted,
    
    // Pairing events
    PairingRequest,     ///< Incoming pairing request (test unit receives)
    PairingComplete,    ///< Pairing completed successfully
    PairingFailed,      ///< Pairing failed (rejected or timeout)
    PeerUnpaired,       ///< A peer was unpaired
};

/**
 * @brief Protocol event structure pushed through FreeRTOS queues.
 *
 * @details The active union member depends on `type`.
 */
struct ProtoEvent {
    ProtoEventType type;
    union {
        TestUnitSettings config;  // Config data (includes extended float fields)
        struct { uint32_t cycle; TestState state; uint8_t err_code; } status;
        struct { uint8_t err_code; uint32_t at_cycle; } error;
        struct { 
            uint8_t peer_mac[6];
            uint8_t device_type;
            char    device_name[MAX_DEVICE_NAME_LEN];
        } pairing;
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
inline uint16_t Crc16Ccitt(const uint8_t* data, size_t len) noexcept
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
