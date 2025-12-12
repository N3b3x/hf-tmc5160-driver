# ESP-NOW Protocol Specification

## Overview

The ESP-NOW protocol provides wireless communication between the remote controller and the fatigue test unit. It uses a custom packet format with CRC16-CCITT error detection.

## Protocol Version

**Current Version**: 1

The protocol header includes a version field for future compatibility.

## Packet Format

### Header Structure

```
┌─────────────────────────────────────────────────┐
│              EspNowHeader (6 bytes)            │
├──────┬──────┬──────────┬──────┬──────┬─────────┤
│ Sync │ Ver  │ DeviceID │ Type │  ID  │ Length │
│(1B)  │(1B)  │   (1B)   │(1B)  │(1B)  │  (1B)  │
└──────┴──────┴──────────┴──────┴──────┴─────────┘
```

**Field Descriptions**:
- **Sync**: Always `0xAA` (sync byte)
- **Ver**: Protocol version (currently `1`)
- **DeviceID**: Device type identifier (0 = broadcast, 1 = Fatigue Tester, etc.)
- **Type**: Message type (see MsgType enum)
- **ID**: Sequence ID (increments per message)
- **Length**: Payload length (0-48 bytes)

### Full Packet Structure

```
┌─────────────────────────────────────────────────────┐
│ EspNowHeader (6 bytes)                             │
├─────────────────────────────────────────────────────┤
│ Payload (0-48 bytes, variable length)               │
├─────────────────────────────────────────────────────┤
│ CRC16 (2 bytes, CRC16-CCITT over header + payload)  │
└─────────────────────────────────────────────────────┘
```

**Total Packet Size**: 8-56 bytes (6 header + 0-48 payload + 2 CRC)

## Message Types

### Control Messages

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| `CONFIG_REQUEST` | 1 | Controller → Unit | Request current configuration |
| `CONFIG_RESPONSE` | 2 | Unit → Controller | Send current configuration |
| `CONFIG_SET` | 3 | Controller → Unit | Set new configuration |
| `CONFIG_ACK` | 4 | Unit → Controller | Acknowledge config set |
| `START` | 5 | Controller → Unit | Start fatigue test |
| `START_ACK` | 6 | Unit → Controller | Acknowledge start |
| `PAUSE` | 7 | Controller → Unit | Pause test |
| `PAUSE_ACK` | 8 | Unit → Controller | Acknowledge pause |
| `RESUME` | 9 | Controller → Unit | Resume test |
| `RESUME_ACK` | 10 | Unit → Controller | Acknowledge resume |
| `STOP` | 11 | Controller → Unit | Stop test |
| `STOP_ACK` | 12 | Unit → Controller | Acknowledge stop |

### Status Messages

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| `STATUS_UPDATE` | 13 | Unit → Controller | Periodic status update |
| `ERROR` | 14 | Unit → Controller | Error notification |
| `TEST_COMPLETE` | 15 | Unit → Controller | Test completion notification |

## Payload Structures

### ConfigPayload (16 bytes)

```cpp
struct ConfigPayload {
    uint32_t cycle_amount;        // Target number of cycles
    uint32_t time_per_cycle_sec;  // Time per cycle (seconds)
    uint32_t dwell_time_sec;      // Dwell time at bounds (seconds)
    uint8_t  bounds_method;       // 0 = StallGuard, 1 = Encoder
};
```

**Used in**: `CONFIG_RESPONSE`, `CONFIG_SET`

### ConfigAckPayload (2 bytes)

```cpp
struct ConfigAckPayload {
    uint8_t ok;        // 1 = success, 0 = failure
    uint8_t err_code;  // Error code if ok == 0
};
```

**Used in**: `CONFIG_ACK`

### StatusPayload (6 bytes)

```cpp
struct StatusPayload {
    uint32_t cycle_number;  // Current cycle count
    uint8_t  state;         // TestState enum
    uint8_t  err_code;      // Error code if state == ERROR
};
```

**Used in**: `STATUS_UPDATE`

### ErrorPayload (5 bytes)

```cpp
struct ErrorPayload {
    uint8_t  err_code;   // Error code
    uint32_t at_cycle;   // Cycle number when error occurred
};
```

**Used in**: `ERROR`

## Test States

```cpp
enum class TestState : uint8_t {
    IDLE = 0,      // Not running
    RUNNING = 1,   // Test in progress
    PAUSED = 2,    // Test paused
    COMPLETED = 3, // Test completed successfully
    ERROR = 4      // Error state
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

The CRC is calculated over the header (6 bytes) + payload (0-48 bytes).

**Polynomial**: 0x1021 (CRC16-CCITT)
**Initial Value**: 0xFFFF

**Algorithm**:
```cpp
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
```

## Communication Flow

### Initialization Sequence

```
Controller                          Test Unit
    │                                  │
    │─── CONFIG_REQUEST ──────────────▶│
    │                                  │
    │◀── CONFIG_RESPONSE ──────────────│
    │                                  │
```

### Test Execution Sequence

```
Controller                          Test Unit
    │                                  │
    │─── CONFIG_SET ──────────────────▶│
    │                                  │
    │◀── CONFIG_ACK ───────────────────│
    │                                  │
    │─── START ────────────────────────▶│
    │                                  │
    │◀── START_ACK ───────────────────│
    │                                  │
    │◀── STATUS_UPDATE (periodic) ────│
    │                                  │
    │─── PAUSE ───────────────────────▶│
    │                                  │
    │◀── PAUSE_ACK ────────────────────│
    │                                  │
    │─── RESUME ───────────────────────▶│
    │                                  │
    │◀── RESUME_ACK ───────────────────│
    │                                  │
    │─── STOP ─────────────────────────▶│
    │                                  │
    │◀── STOP_ACK ─────────────────────│
    │                                  │
    │◀── TEST_COMPLETE ────────────────│
    │                                  │
```

## WiFi Channel

**Default Channel**: 1

Both devices must use the same WiFi channel for ESP-NOW communication.

## MAC Address Configuration

The remote controller must be configured with the test unit's MAC address. This is set in the controller's `config.hpp`:

```cpp
static constexpr uint8_t TEST_UNIT_MAC_[6] = { 0x24, 0x6F, 0x28, 0x00, 0x00, 0x01 };
```

## Sequence ID Management

- Sequence IDs increment for each sent message
- Used for tracking message order and detecting duplicates
- Wraps around at 255 (uint8_t)

## Reliability

- **CRC Validation**: All packets validated with CRC16-CCITT
- **Sync Byte**: All packets must start with 0xAA
- **Version Check**: Protocol version must match
- **Retry Logic**: Implemented at application layer (if needed)

## Performance

- **Latency**: < 10ms typical (ESP-NOW is low-latency)
- **Throughput**: Sufficient for control messages (not high-bandwidth)
- **Range**: ~100-200m line-of-sight (depends on environment)

## Future Enhancements

- Protocol version negotiation
- Acknowledgment retry mechanism
- Broadcast support for multiple units
- Encryption support

