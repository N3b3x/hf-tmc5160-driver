# ESP-NOW Protocol Specification

## Overview

The ESP-NOW protocol provides wireless communication between the remote controller (UI board) and the fatigue test unit. It uses a custom packet format with CRC16-CCITT error detection.

## Protocol Version

**Current Version**: 1

The protocol header includes a version field for future compatibility.

## Packet Format

### Header Structure (6 bytes)

```
┌─────────────────────────────────────────────────────────────────┐
│                    EspNowHeader (6 bytes)                       │
├────────┬─────────┬───────────┬────────┬────────┬────────────────┤
│  Sync  │ Version │ Device ID │  Type  │   ID   │    Length      │
│  (1B)  │  (1B)   │   (1B)    │  (1B)  │  (1B)  │     (1B)       │
│  0xAA  │    1    │     1     │ MsgType│ SeqNum │  0-200 bytes   │
└────────┴─────────┴───────────┴────────┴────────┴────────────────┘
```

**Field Descriptions**:
- **Sync**: Always `0xAA` (sync byte for packet detection)
- **Version**: Protocol version (currently `1`)
- **Device ID**: Device type identifier (0 = broadcast, 1 = Fatigue Tester)
- **Type**: Message type (see MsgType enum below)
- **ID**: Sequence ID (increments per message, wraps at 255)
- **Length**: Payload length in bytes (0-200)

### Full Packet Structure

```
┌─────────────────────────────────────────────────────────────────┐
│ EspNowHeader (6 bytes)                                          │
├─────────────────────────────────────────────────────────────────┤
│ Payload (0-200 bytes, variable length per hdr.len)              │
├─────────────────────────────────────────────────────────────────┤
│ CRC16 (2 bytes, CRC16-CCITT over header + payload)              │
└─────────────────────────────────────────────────────────────────┘
```

**Total Packet Size**: 8-208 bytes (6 header + 0-200 payload + 2 CRC)

**Important**: The CRC is placed immediately after the payload, NOT at a fixed offset.
For a 29-byte payload, CRC is at offset 35 (6 + 29).

## Message Types (MsgType Enum)

```cpp
enum class MsgType : uint8_t {
    DeviceDiscovery = 1,   // Discover devices on network
    DeviceInfo      = 2,   // Device information response
    ConfigRequest   = 3,   // Request current configuration
    ConfigResponse  = 4,   // Send current configuration
    ConfigSet       = 5,   // Set new configuration
    ConfigAck       = 6,   // Acknowledge config set
    Command         = 7,   // Send command (Start/Pause/Resume/Stop)
    CommandAck      = 8,   // Acknowledge command receipt
    StatusUpdate    = 9,   // Periodic status update
    Error           = 10,  // Error notification
    ErrorClear      = 11,  // Clear error state
    TestComplete    = 12   // Test completion notification
};
```

### Message Summary

| Type | Value | Direction | Payload | Description |
|------|-------|-----------|---------|-------------|
| `DeviceDiscovery` | 1 | Controller → Unit | None | Discover devices |
| `DeviceInfo` | 2 | Unit → Controller | Device info | Device information |
| `ConfigRequest` | 3 | Controller → Unit | None | Request configuration |
| `ConfigResponse` | 4 | Unit → Controller | ConfigPayload (29B) | Current configuration |
| `ConfigSet` | 5 | Controller → Unit | ConfigPayload (29B) | Set configuration |
| `ConfigAck` | 6 | Unit → Controller | ConfigAckPayload (2B) | Acknowledge config |
| `Command` | 7 | Controller → Unit | CommandPayload (1B+) | Start/Pause/Resume/Stop |
| `CommandAck` | 8 | Unit → Controller | None | Acknowledge command |
| `StatusUpdate` | 9 | Unit → Controller | StatusPayload (6B) | Periodic status |
| `Error` | 10 | Unit → Controller | ErrorPayload (5B) | Error notification |
| `ErrorClear` | 11 | Unit → Controller | None | Error cleared |
| `TestComplete` | 12 | Unit → Controller | None | Test completed |

## Payload Structures

### ConfigPayload (29 bytes)

Used in `ConfigSet` and `ConfigResponse` messages.

```cpp
#pragma pack(push, 1)
struct ConfigPayload {
    // Base fields (13 bytes) - required, always present
    uint32_t cycle_amount;                    // Target number of cycles
    uint32_t time_per_cycle_sec;              // Time per cycle (seconds)
    uint32_t dwell_time_sec;                  // Dwell time at bounds (seconds)
    uint8_t  bounds_method;                   // 0 = StallGuard, 1 = Encoder
    
    // Extended fields (16 bytes) - optional advanced configuration
    float    bounds_search_velocity_rpm;      // Search speed (RPM), 0 = use default
    float    stallguard_min_velocity_rpm;     // SG2 min velocity (RPM), 0 = use default
    float    stall_detection_current_factor;  // Current reduction (0.0-1.0), 0 = use default
    float    bounds_search_accel_rev_s2;      // Search acceleration (rev/s²), 0 = use default
};
#pragma pack(pop)
```

**Extended Field Behavior**:
- Value of `0.0f` means "use test unit's default from TestConfig"
- Non-zero values override the test unit's defaults
- Backward compatible: older controllers can send only 13 bytes (base fields)

**Byte Layout**:
```
Offset  Size  Field
------  ----  -----
0       4     cycle_amount
4       4     time_per_cycle_sec
8       4     dwell_time_sec
12      1     bounds_method
13      4     bounds_search_velocity_rpm (float)
17      4     stallguard_min_velocity_rpm (float)
21      4     stall_detection_current_factor (float)
25      4     bounds_search_accel_rev_s2 (float)
------  ----
Total:  29 bytes
```

### CommandPayload (1+ bytes)

Used in `Command` messages.

```cpp
struct CommandPayload {
    uint8_t command_id;  // CommandId enum value
    // Additional command-specific data may follow
};
```

**Command IDs**:
```cpp
enum class CommandId : uint8_t {
    Start  = 1,  // Start fatigue test
    Pause  = 2,  // Pause test
    Resume = 3,  // Resume test
    Stop   = 4   // Stop test
};
```

### ConfigAckPayload (2 bytes)

Used in `ConfigAck` messages.

```cpp
struct ConfigAckPayload {
    uint8_t ok;        // 1 = success, 0 = failure
    uint8_t err_code;  // Error code if ok == 0
};
```

### StatusPayload (6 bytes)

Used in `StatusUpdate` messages.

```cpp
struct StatusPayload {
    uint32_t cycle_number;  // Current cycle count
    uint8_t  state;         // TestState enum value
    uint8_t  err_code;      // Error code if state == Error
};
```

**Byte Layout**:
```
Offset  Size  Field
------  ----  -----
0       4     cycle_number
4       1     state
5       1     err_code
------  ----
Total:  6 bytes
```

### ErrorPayload (5 bytes)

Used in `Error` messages.

```cpp
struct ErrorPayload {
    uint8_t  err_code;   // Error code
    uint32_t at_cycle;   // Cycle number when error occurred
};
```

## Test States

```cpp
enum class TestState : uint8_t {
    Idle      = 0,  // Not running, ready for commands
    Running   = 1,  // Test in progress (includes bounds finding)
    Paused    = 2,  // Test paused, motor de-energized
    Completed = 3,  // Test completed successfully
    Error     = 4   // Error state
};
```

## Error Codes

| Code | Description |
|------|-------------|
| 0 | Success / No error |
| 1 | Bounds not found |
| 2 | Start failed |
| 3 | Configuration error |
| 4 | Motion control error |
| 5 | Communication error |

## CRC16-CCITT Calculation

The CRC is calculated over the header (6 bytes) + payload (0-200 bytes).
The CRC field itself is NOT included in the calculation.

**Parameters**:
- Polynomial: `0x1021` (CRC16-CCITT)
- Initial Value: `0xFFFF`

**Algorithm**:
```cpp
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
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
```

**CRC Placement**:
The CRC is placed immediately after the payload in the transmitted packet:
- Offset = `sizeof(EspNowHeader) + payload_len` = `6 + len`
- For a ConfigPayload (29 bytes): CRC at offset 35

## Communication Flow

### Initialization Sequence

```
Remote Controller                       Test Unit
      │                                     │
      │──── CONFIG_REQUEST (8 bytes) ──────►│
      │     [hdr:6][crc:2]                  │
      │                                     │
      │◄── CONFIG_RESPONSE (37 bytes) ──────│
      │    [hdr:6][ConfigPayload:29][crc:2] │
      │                                     │
```

### Configuration Update Sequence

```
Remote Controller                       Test Unit
      │                                     │
      │──── CONFIG_SET (37 bytes) ─────────►│
      │     [hdr:6][ConfigPayload:29][crc:2]│
      │                                     │
      │◄── CONFIG_ACK (10 bytes) ───────────│
      │    [hdr:6][ConfigAckPayload:2][crc:2]│
      │                                     │
```

### Test Execution Sequence

```
Remote Controller                       Test Unit
      │                                     │
      │──── Command:Start (9 bytes) ───────►│
      │     [hdr:6][cmd_id:1][crc:2]        │
      │                                     │
      │◄── COMMAND_ACK (8 bytes) ───────────│
      │    [hdr:6][crc:2]                   │
      │                                     │
      │◄── STATUS_UPDATE (14 bytes, 1Hz) ───│  (periodic while running)
      │    [hdr:6][StatusPayload:6][crc:2]  │
      │                                     │
      │──── Command:Pause (9 bytes) ───────►│
      │                                     │
      │◄── COMMAND_ACK ─────────────────────│
      │                                     │
      │──── Command:Resume (9 bytes) ──────►│
      │                                     │
      │◄── COMMAND_ACK ─────────────────────│
      │                                     │
      │◄── STATUS_UPDATE (14 bytes, 1Hz) ───│  (periodic continues)
      │                                     │
      │──── Command:Stop (9 bytes) ────────►│
      │                                     │
      │◄── COMMAND_ACK ─────────────────────│
      │                                     │
      │◄── TEST_COMPLETE (8 bytes) ─────────│  (on normal completion)
      │    [hdr:6][crc:2]                   │
      │                                     │
```

### Status Update Timing

The test unit sends `STATUS_UPDATE` messages:
- **Periodic**: Every 1 second while state is `Running` or `BoundsFinding`
- **Immediate**: On START, PAUSE, RESUME, STOP command acknowledgment
- **On Completion**: Final status before `TEST_COMPLETE`

## WiFi Configuration

**Default WiFi Channel**: 1

Both devices must use the same WiFi channel for ESP-NOW communication.
Configured in both projects' `config.hpp` / `espnow_protocol.hpp`.

## MAC Address Configuration

### Remote Controller → Test Unit

The remote controller must be configured with the test unit's MAC address:

```cpp
// In remote controller's config.hpp
static constexpr uint8_t TEST_UNIT_MAC_[6] = { 0x24, 0x6F, 0x28, 0xXX, 0xXX, 0xXX };
```

### Test Unit → Remote Controller (Optional)

The test unit can optionally be pre-configured with the remote controller's MAC:

```cpp
// In test unit's espnow_protocol.hpp
static constexpr uint8_t UI_BOARD_MAC[6] = { 0x9C, 0x9E, 0x6E, 0xXX, 0xXX, 0xXX };
```

If not pre-configured (all zeros), the test unit learns the MAC from the first received packet.

## Sequence ID Management

- Sequence IDs increment for each sent message (per device)
- Used for tracking message order and detecting duplicates
- Wraps around at 255 (uint8_t)
- Not currently used for retry logic

## Reliability Features

| Feature | Implementation |
|---------|----------------|
| **CRC Validation** | All packets validated with CRC16-CCITT |
| **Sync Byte** | All packets must start with 0xAA |
| **Version Check** | Protocol version must match |
| **Length Validation** | Payload length checked against expected |
| **Retry Logic** | Application layer (manual via UI) |

## Performance Characteristics

| Metric | Value |
|--------|-------|
| **Latency** | < 10ms typical (ESP-NOW is low-latency) |
| **Status Update Rate** | 1 Hz (every 1000ms) |
| **Range** | ~100-200m line-of-sight (environment dependent) |
| **Max Payload** | 200 bytes |
| **Max Packet** | 208 bytes (6 + 200 + 2) |

## Backward Compatibility

The protocol supports backward compatibility:

1. **Base ConfigPayload (13 bytes)**: Older remote controllers can send only base fields
2. **Extended ConfigPayload (29 bytes)**: Newer controllers include extended float fields
3. **Extended fields = 0.0f**: Interpreted as "use test unit defaults"

The test unit checks `payload_len` to determine which format was received.

## Implementation Files

| Component | File | Description |
|-----------|------|-------------|
| **Test Unit Protocol** | `espnow_protocol.hpp` | Protocol definitions |
| **Test Unit Receiver** | `espnow_receiver.cpp` | Receive/send implementation |
| **Test Unit Main** | `main.cpp` | Event handling |
| **Remote Controller Protocol** | `protocol/espnow_protocol.hpp` | Protocol definitions |
| **Remote Controller Impl** | `protocol/espnow_protocol.cpp` | Send/receive implementation |
| **Device Payloads** | `protocol/device_protocols.hpp` | Payload structures |
| **Fatigue Tester UI** | `devices/fatigue_tester.cpp` | UI event handling |
