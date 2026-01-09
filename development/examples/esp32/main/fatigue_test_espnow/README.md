# Fatigue Test with ESP-NOW Communication

This directory contains the fatigue test unit that receives commands via ESP-NOW and performs fatigue testing with dual bounds detection methods.

## Overview

The **Test Unit** is a unified fatigue tester with dual communication:
- Receives commands from remote controller via ESP-NOW
- Accepts direct commands via UART (for debugging/development)
- Performs bounds finding using selectable method (StallGuard2 or encoder-based)
- Runs sinusoidal fatigue test motion
- Sends status updates back to remote controller (1 Hz while running)
- **Unified Implementation**: Uses the driver library's built-in homing/bounds subsystem

**Note**: The remote controller UI has been moved to a standalone project: `examples/esp32_remote_controller/`

## Directory Structure

```
fatigue_test_espnow/
├── docs/
│   ├── ARCHITECTURE.md          # System architecture documentation
│   ├── HARDWARE_SETUP.md        # Hardware setup guide
│   └── PROTOCOL.md              # ESP-NOW protocol specification
├── espnow_protocol.hpp          # ESP-NOW protocol definitions
├── espnow_receiver.hpp/cpp      # ESP-NOW receiver implementation
├── main.cpp                     # Main application (unified implementation)
├── espnow_protocol_test_unit.cpp # Protocol test unit (no motor control)
├── fatigue_motion.hpp           # Unified motion controller interface
└── fatigue_motion_impl.hpp      # Motion controller implementation
```

## ESP-NOW Protocol

The protocol uses a 6-byte header with sync byte (0xAA), version, device ID, and CRC16-CCITT for error detection.

### Message Types

| Type | Description |
|------|-------------|
| `ConfigRequest` / `ConfigResponse` | Request/send current settings |
| `ConfigSet` / `ConfigAck` | Set new configuration |
| `Command` | Start/Pause/Resume/Stop commands |
| `CommandAck` | Command acknowledgment |
| `StatusUpdate` | Periodic status updates (1 Hz) |
| `Error` | Error notification |
| `TestComplete` | Test completion notification |

### Settings Structure

```cpp
struct TestUnitSettings {
    // Base fields (always synchronized)
    uint32_t cycle_amount;              // Target number of cycles
    uint32_t time_per_cycle;            // Time per cycle in seconds
    uint32_t dwell_time;                // Dwell time at bounds in seconds
    bool     bounds_method_stallguard;  // true = StallGuard2, false = encoder
    
    // Extended fields (0.0f = use test config defaults)
    float    bounds_search_velocity_rpm;       // Search speed during bounds finding (RPM)
    float    stallguard_min_velocity_rpm;      // Min velocity for StallGuard2 (RPM)
    float    stall_detection_current_factor;   // Current reduction factor (0.0-1.0)
    float    bounds_search_accel_rev_s2;       // Search acceleration (rev/s²)
};
```

**Extended Field Behavior**:
- Value of `0.0f` → uses test unit's TestConfig default
- Non-zero value → overrides the default
- Backward compatible with older remote controllers (base fields only)

### Remote Controller Settings Menu

The remote controller provides a scrollable settings menu:

| Setting | Range | Step | Description |
|---------|-------|------|-------------|
| Cycles | 1-100,000 | 100 | Target cycle count |
| Time/Cycle | 1-3600 s | 1 | Oscillation period |
| Dwell Time | 0-60 s | 1 | Pause at bounds |
| Bounds Mode | SG/ENC | Toggle | Detection method |
| Search Speed | 0-300 RPM | 5 | Bounds search speed (0=AUTO) |
| SG Min Vel | 0-100 RPM | 1 | StallGuard min velocity (0=AUTO) |
| Curr Factor | 0.0-1.0 | 0.05 | Current reduction (0=AUTO) |
| Search Accel | 0-20 rev/s² | 0.5 | Search acceleration (0=AUTO) |
| Error Severity | 1-3 | 1 | Min error level to display |
| Flip Screen | NORM/FLIP | Toggle | Display orientation |

## Configuration

### Test Unit Configuration

The test unit uses the same test rig configuration as other fatigue test examples. Edit `main.cpp`:

```cpp
// Test rig selection
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
```

### MAC Address Configuration

1. **Flash test unit** and note its MAC address from serial output:
   ```
   ESP-NOW Device MAC Address: XX:XX:XX:XX:XX:XX
   ```

2. **Update remote controller** (`config.hpp`):
   ```cpp
   static constexpr uint8_t TEST_UNIT_MAC_[6] = { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX };
   ```

3. **(Optional)** Update test unit with controller MAC for immediate responses:
   ```cpp
   // In espnow_protocol.hpp
   static constexpr uint8_t UI_BOARD_MAC[6] = { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX };
   ```

## Building

### Full Test Unit (with Motor Control)

```bash
cd examples/esp32
./scripts/build_app.sh fatigue_test_espnow_unit Release
./scripts/flash_app.sh flash_monitor fatigue_test_espnow_unit Release
```

### Protocol Test Unit (No Motor Control)

For testing ESP-NOW protocol communication without hardware dependencies:

```bash
cd examples/esp32
./scripts/build_app.sh espnow_protocol_test_unit Release
```

The protocol test unit (`espnow_protocol_test_unit.cpp`) is a minimal implementation that:
- Receives all ESP-NOW protocol commands
- Responds with appropriate protocol messages (ACKs, status updates)
- Simulates test state changes (IDLE → RUNNING → PAUSED → COMPLETED)
- Does NOT initialize motor driver or motion controller
- Useful for validating ESP-NOW communication without motor hardware

## Usage

1. **Flash the test unit** with the firmware
2. **Configure MAC address** in the remote controller
3. **Power on test unit first**, then remote controller
4. **Remote controller requests config** from test unit on startup
5. **Use remote controller** to configure and run tests

## Features

### Test Unit
- **Dual Communication**: ESP-NOW (wireless) + UART (direct serial)
- **Dual Bounds Detection**: StallGuard2 or encoder-based (selectable)
- **Library-Based Homing**: Uses `driver.homing.FindBounds()` for robust bounds detection
- **Sinusoidal Motion**: Smooth oscillation between detected bounds
- **Configurable Parameters**: All settings adjustable via remote controller
- **Status Updates**: 1 Hz updates while running, immediate on state changes
- **Error Reporting**: Error codes and cycle counts sent to remote

### Communication Flow

```
Remote Controller                       Test Unit
      │                                     │
      │──── CONFIG_REQUEST ────────────────►│
      │◄── CONFIG_RESPONSE (29-byte payload)│
      │                                     │
      │     [User adjusts settings]         │
      │                                     │
      │──── CONFIG_SET (29-byte payload) ──►│
      │◄── CONFIG_ACK ──────────────────────│
      │                                     │
      │──── Command:Start ─────────────────►│
      │◄── COMMAND_ACK ─────────────────────│
      │                                     │
      │◄── STATUS_UPDATE (1 Hz) ────────────│  ← cycles, state
      │◄── STATUS_UPDATE ───────────────────│
      │...                                  │
      │                                     │
      │──── Command:Stop ──────────────────►│
      │◄── COMMAND_ACK ─────────────────────│
      │◄── TEST_COMPLETE ───────────────────│
      │                                     │
```

## Protocol Details

### Packet Format

```
┌─────────────────────────────────────────────────────────────────┐
│ Header (6 bytes): [Sync:0xAA][Ver:1][DevID][Type][ID][Length]   │
├─────────────────────────────────────────────────────────────────┤
│ Payload (0-200 bytes, per Length field)                         │
├─────────────────────────────────────────────────────────────────┤
│ CRC16 (2 bytes, CRC16-CCITT over header + payload)              │
└─────────────────────────────────────────────────────────────────┘
```

### Error Codes

| Code | Description |
|------|-------------|
| 0 | Success |
| 1 | Bounds not found |
| 2 | Start failed |
| 3 | Configuration error |
| 4 | Motion control error |
| 5 | Communication error |

## Bounds Finding

The test unit uses the **driver library built-in homing/bounds subsystem**:

```cpp
// StallGuard2 method
g_driver->homing.FindBounds(Homing::BoundsMethod::StallGuard, options, home_config, cancel_cb);

// Encoder method
g_driver->homing.FindBounds(Homing::BoundsMethod::Encoder, options, home_config, cancel_cb);
```

**Features**:
- Automatic backoff from detected bounds
- Configurable search speed and acceleration
- Timeout protection
- Cancel callback for user-initiated stop
- Home placement at center of detected bounds

## UART Command Interface

The test unit supports UART commands for debugging:

| Command | Description |
|---------|-------------|
| `-f <freq>` | Set frequency in Hz |
| `-d <min> <max>` | Set dwell times in ms |
| `-b <min> <max>` | Set angle bounds in degrees |
| `-c <count>` | Set target cycle count (0 = infinite) |
| `-a <action>` | start, stop, or reset |
| `-s` | Show current status |
| `-h` | Show help message |

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No communication | Check MAC addresses match in both projects |
| CRC errors | Verify WiFi channel is 1 in both projects |
| Bounds finding fails | Check motor/encoder connections, adjust search speed |
| UART not working | Check baud rate (115200 default) |
| Settings not saving | Ensure both exit paths trigger save (BACK button or "Back" menu) |

## Documentation

Comprehensive documentation is available:

- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System architecture and design patterns
- **[docs/PROTOCOL.md](docs/PROTOCOL.md)** - Complete ESP-NOW protocol specification
- **[docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md)** - Hardware setup and pin configuration

## Implementation Details

### Task Architecture

| Task | Priority | Period | Description |
|------|----------|--------|-------------|
| `espnow_command_task` | 5 | Event-driven | Process ESP-NOW commands |
| `motion_control_task` | 5 | 10ms | Motion update loop |
| `status_update_task` | 3 | 1000ms | Send status to remote |
| `uart_command_task` | 3 | 50ms | Process UART commands |
| `bounds_finding_task` | 4 | Dynamic | Created on demand |

### State Machine

```
                    ┌─────────────┐
                    │    IDLE     │◄──────────────────┐
                    └──────┬──────┘                   │
                           │ START                    │ STOP/Complete
                           ▼                          │
                    ┌─────────────┐                   │
                    │   BOUNDS    │                   │
                    │   FINDING   │                   │
                    └──────┬──────┘                   │
                           │ Bounds found             │
                           ▼                          │
            ┌─────►┌─────────────┐                    │
            │      │   RUNNING   │────────────────────┤
            │      └──────┬──────┘                    │
    RESUME  │             │ PAUSE                     │
            │             ▼                           │
            │      ┌─────────────┐                    │
            └──────│   PAUSED    │────────────────────┘
                   └─────────────┘         STOP
```

## Future Enhancements

- Real-time parameter adjustment via ESP-NOW
- Data logging to SD card or flash
- Multiple test unit support (broadcast commands)
- Web interface for configuration
- OTA firmware updates
