# Fatigue Test with ESP-NOW Communication

This directory contains the fatigue test unit that receives commands via ESP-NOW and performs fatigue testing with dual bounds detection methods.

## Overview

The **Test Unit** is a unified fatigue tester with dual communication:
- Receives commands from remote controller via ESP-NOW
- Accepts direct commands via UART (for debugging/development)
- Performs bounds finding using selectable method (StallGuard2 or encoder-based)
- Runs sinusoidal fatigue test motion
- Sends status updates back to remote controller
- **Unified Implementation**: Combines `fatigue_test_encoder.cpp` and `fatigue_test_stallguard.cpp` with proper C++ abstractions

**Note**: The remote controller UI has been moved to a standalone project: `examples/esp32_remote_controller/`

## Directory Structure

```
fatigue_test_espnow/
├── espnow_protocol.hpp          # Shared ESP-NOW protocol definitions
├── main.cpp                     # Main application (unified implementation)
├── espnow_protocol_test_unit.cpp # Protocol test unit (no motor control)
├── espnow_receiver.hpp/cpp      # ESP-NOW receiver implementation
├── bounds_finder.hpp            # Abstract bounds finder interface
├── bounds_finder_stallguard.cpp # StallGuard2 implementation
├── bounds_finder_encoder.cpp    # Encoder-based implementation
├── fatigue_motion.hpp           # Unified motion controller interface
└── fatigue_motion_impl.hpp      # Motion controller implementation
```

## ESP-NOW Protocol

The protocol uses a sync byte (0xAA) and CRC16-CCITT for error detection.

### Message Types

- `CONFIG_REQUEST` / `CONFIG_RESPONSE`: Request/send current settings
- `CONFIG_SET` / `CONFIG_ACK`: Set new configuration
- `START` / `START_ACK`: Start fatigue test
- `PAUSE` / `PAUSE_ACK`: Pause test
- `RESUME` / `RESUME_ACK`: Resume test
- `STOP` / `STOP_ACK`: Stop test
- `STATUS_UPDATE`: Periodic status updates (cycle count, state)
- `ERROR`: Error notification
- `TEST_COMPLETE`: Test completion notification

### Settings Structure

```cpp
struct Settings {
    uint32_t cycle_amount;        // Target number of cycles
    uint32_t time_per_cycle;      // Time per cycle in seconds
    uint32_t dwell_time;          // Dwell time at bounds in seconds
    bool bounds_method_stallguard; // true = StallGuard2, false = encoder
};
```

## Configuration

### Test Unit Configuration

The test unit uses the same test rig configuration as other fatigue test examples. Edit `main.cpp`:

```cpp
// Test rig selection
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
```

## Building

### Full Test Unit (with Motor Control)

```bash
cd examples/esp32
./scripts/build_app.sh fatigue_test_espnow_unit Release
```

### Protocol Test Unit (No Motor Control)

For testing ESP-NOW protocol communication without hardware dependencies:

```bash
cd examples/esp32
./scripts/build_app.sh espnow_protocol_test_unit Release
```

The protocol test unit (`espnow_protocol_test_unit.cpp`) is a minimal implementation that:
- Receives all ESP-NOW protocol commands (CONFIG_REQUEST, CONFIG_SET, START, PAUSE, RESUME, STOP)
- Responds with appropriate protocol messages (ACKs, status updates, etc.)
- Simulates test state changes (IDLE → RUNNING → PAUSED → COMPLETED)
- Does NOT initialize motor driver, bounds finder, or motion controller
- Useful for validating ESP-NOW communication without requiring motor hardware

**Use Cases:**
- Testing ESP-NOW protocol implementation
- Validating remote controller communication
- Debugging protocol issues without motor hardware
- Development and CI/CD testing

## Usage

1. **Flash the test unit** with the firmware
2. **Configure MAC address** in the remote controller (see `examples/esp32_remote_controller/`)
3. **Power on test unit first**, then remote controller
4. **Remote controller will request config** from test unit on startup
5. **Use remote controller** to start, pause, resume, or stop tests

## Features

### Test Unit
- **Dual Communication**: ESP-NOW (wireless) + UART (direct serial)
- **Dual Bounds Detection**: Selectable StallGuard2 or encoder-based
- **Unified Motion Controller**: Extracted from `fatigue_test_encoder.cpp` and `fatigue_test_stallguard.cpp`
- Sinusoidal fatigue test motion
- Cycle counting
- Status updates sent to remote controller
- Error handling and reporting
- **Proper C++ Abstractions**: Interface-based design for extensibility

## Protocol Details

### Packet Format

```
[Sync Byte: 0xAA][Type][ID][Length][Payload...][CRC16]
```

- Sync Byte: Always 0xAA
- Type: Message type (MsgType enum)
- ID: Sequence ID (increments)
- Length: Payload length (0-48 bytes)
- Payload: Message-specific data
- CRC16: CRC16-CCITT over header + payload

### Error Codes

- 0: Success
- 1: Bounds not found
- 2: Start failed
- 3: Configuration error

## Bounds Finding

The test unit includes **full implementations** of both bounds finding methods:

- **Encoder-based** (`bounds_finder_encoder.cpp`): Complete implementation extracted from `fatigue_test_encoder.cpp`
  - Monitors encoder position changes
  - Detects stalls when encoder stops moving while motor is commanded to move
  - Handles false stall detection with movement thresholds

- **StallGuard2** (`bounds_finder_stallguard.cpp`): Complete implementation extracted from `fatigue_test_stallguard.cpp`
  - Uses TMC51x0 StallGuard2 sensorless detection
  - Configures SGT threshold for homing
  - Handles false stall detection with movement thresholds

Both implementations follow the `IBoundsFinder` interface, allowing easy switching between methods. The method is selected via the `bounds_method_stallguard` setting from the remote controller.

## UART Command Interface

The test unit supports the same UART commands as the standalone fatigue test examples:

- `-f <freq>` / `--freq <freq>`: Set frequency in Hz
- `-d <min> <max>` / `--dwell <min> <max>`: Set dwell times in ms
- `-b <min> <max>` / `--bounds <min> <max>`: Set angle bounds in degrees
- `-c <count>` / `--cycles <count>`: Set target cycle count (0 = infinite)
- `-a <action>` / `--action <action>`: start, stop, or reset
- `-s` / `--status`: Show current status
- `-h` / `--help`: Show help message

This allows direct control and debugging via serial terminal while ESP-NOW handles remote control.

## Troubleshooting

1. **No communication**: Check MAC addresses are correct in remote controller
2. **CRC errors**: Check WiFi channel matches (default: channel 1)
3. **Bounds finding fails**: Check motor and encoder connections
4. **UART commands not working**: Check baud rate (115200 default)

## Implementation Details

### Unified Architecture

The test unit implementation combines:

1. **Bounds Finding Abstraction**:
   - `IBoundsFinder` interface defines the contract
   - `StallGuardBoundsFinder` and `EncoderBoundsFinder` implement the interface
   - Factory functions create instances: `CreateStallGuardBoundsFinder()`, `CreateEncoderBoundsFinder()`

2. **Motion Controller**:
   - Unified `FatigueTestMotion` class extracted from existing implementations
   - Supports both sinusoidal and ramp-based motion
   - Thread-safe with RAII mutex guards
   - Cycle counting with center-crossing detection

3. **Command Interface**:
   - `UartCommandParser` handles UART commands
   - ESP-NOW commands processed via `EspNowReceiver`
   - Both interfaces update the same `FatigueTestMotion` instance

## Documentation

Comprehensive documentation is available:

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System architecture, component overview, and design patterns
- **[PROTOCOL.md](PROTOCOL.md)** - Complete ESP-NOW protocol specification
- **[HARDWARE_SETUP.md](HARDWARE_SETUP.md)** - Hardware setup guide, pin configuration, and troubleshooting

## Future Enhancements

- Real-time parameter adjustment via ESP-NOW
- Data logging to SD card or flash
- Multiple test unit support (broadcast commands)
- Web interface for configuration
