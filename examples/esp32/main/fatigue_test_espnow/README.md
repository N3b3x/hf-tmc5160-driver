# Fatigue Test with ESP-NOW Communication

This directory contains a complete fatigue testing system with wireless remote control via ESP-NOW.

## Overview

The system consists of two ESP32 devices:

1. **UI Board** (`ui_board/`): Remote controller with e-ink display and buttons
   - Sends commands to test unit via ESP-NOW
   - Displays status and settings on e-ink display
   - Supports deep sleep wake from buttons
   - Inactivity timeout for power saving

2. **Test Unit** (`test_unit/`): Unified fatigue tester with dual communication
   - Receives commands from UI board via ESP-NOW
   - Accepts direct commands via UART (for debugging/development)
   - Performs bounds finding using selectable method (StallGuard2 or encoder-based)
   - Runs sinusoidal fatigue test motion
   - Sends status updates back to UI board
   - **Unified Implementation**: Combines `fatigue_test_encoder.cpp` and `fatigue_test_stallguard.cpp` with proper C++ abstractions

## Directory Structure

```
fatigue_test_espnow/
├── espnow_protocol.hpp          # Shared ESP-NOW protocol definitions
├── ui_board/                     # Remote controller code
│   ├── main.cpp                 # Main application
│   ├── config.hpp               # GPIO and configuration
│   ├── button.hpp/cpp           # Button handling
│   ├── settings.hpp/cpp         # Settings storage (NVS)
│   ├── espnow_protocol.hpp/cpp  # ESP-NOW protocol implementation
│   └── ui.hpp/cpp               # UI state machine (e-ink display)
└── test_unit/                    # Fatigue tester code
    ├── main.cpp                 # Main application (unified implementation)
    ├── espnow_receiver.hpp/cpp  # ESP-NOW receiver implementation
    ├── bounds_finder.hpp        # Abstract bounds finder interface
    ├── bounds_finder_stallguard.cpp  # StallGuard2 implementation
    ├── bounds_finder_encoder.cpp     # Encoder-based implementation
    └── fatigue_motion.hpp       # Unified motion controller interface
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
    bool orientation_flipped;      // Display orientation
    bool bounds_method_stallguard; // true = StallGuard2, false = encoder
};
```

## Configuration

### UI Board Configuration

Edit `ui_board/config.hpp`:

```cpp
// Button GPIOs (must be RTC-capable for deep sleep wake)
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_4;
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_6;

// E-ink display pins (adjust for your hardware)
static constexpr int EINK_DC_PIN   = 10;
static constexpr int EINK_RESET_PIN = 11;
static constexpr int EINK_CS_PIN    = 9;
static constexpr int EINK_BUSY_PIN  = 12;

// Test unit MAC address (set to your test unit's MAC)
static constexpr uint8_t TEST_UNIT_MAC[6] = { 0x24, 0x6F, 0x28, 0x00, 0x00, 0x01 };
```

### Test Unit Configuration

The test unit uses the same test rig configuration as other fatigue test examples. Edit `test_unit/main.cpp`:

```cpp
// Test rig selection
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
```

## Building

### UI Board

```bash
cd examples/esp32
./scripts/build_app.sh fatigue_test_espnow_ui Release
```

### Test Unit

The unified test unit is built from `fatigue_test_espnow_unit.cpp` in the main directory:

```bash
cd examples/esp32
./scripts/build_app.sh fatigue_test_espnow_unit Release
```

**Note**: The main application file (`fatigue_test_espnow_unit.cpp`) includes the full implementation combining:
- Bounds finding from both `fatigue_test_encoder.cpp` and `fatigue_test_stallguard.cpp`
- UART command interface (same commands as standalone examples)
- ESP-NOW communication with UI board
- Unified `FatigueTestMotion` class

The implementation uses proper C++ abstractions:
- `IBoundsFinder` interface for bounds detection strategies
- Factory functions for creating bounds finders
- Unified motion controller extracted from existing implementations

## Usage

1. **Flash both devices** with their respective firmware
2. **Configure MAC addresses**: Set the test unit's MAC in `ui_board/config.hpp`
3. **Power on test unit first**, then UI board
4. **UI board will request config** from test unit on startup
5. **Use buttons** on UI board to:
   - UP: Start test
   - SELECT: Settings menu / Pause/Resume
   - DOWN: Stop test (with confirmation)

## Features

### UI Board
- E-ink display for status and settings
- Three-button navigation
- Deep sleep with button wake
- Inactivity timeout (60 seconds default)
- Settings stored in NVS

### Test Unit
- **Dual Communication**: ESP-NOW (wireless) + UART (direct serial)
- **Dual Bounds Detection**: Selectable StallGuard2 or encoder-based
- **Unified Motion Controller**: Extracted from `fatigue_test_encoder.cpp` and `fatigue_test_stallguard.cpp`
- Sinusoidal fatigue test motion
- Cycle counting
- Status updates sent to UI board
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

## Integration Notes

### E-ink Display

The UI code includes **full, production-ready** e-ink display integration for the **2.9" ThinkInk FeatherWing**:

- **Vertical Orientation**: Optimized for portrait mode (128x296 pixels)
- **All Drawing Functions Implemented**:
  - `draw_main_screen()`: Main UI with settings summary and controls
  - `draw_settings_screen()`: Settings display with all parameters
  - `draw_error_screen()`: Error display with slow blinking (e-ink optimized)
  - `draw_complete_screen()`: Test completion screen
  - `update_status_footer()`: Dynamic footer with cycle progress

- **E-ink Optimizations**:
  - Slow refresh rates (2+ seconds for blinking)
  - Partial update support (when available)
  - Proper rotation handling (portrait mode)
  - Compact layout for vertical display

The display is initialized with proper dimensions and rotation in `UI::init()`.

### Bounds Finding

The unified test unit includes **full implementations** of both bounds finding methods:

- **Encoder-based** (`bounds_finder_encoder.cpp`): Complete implementation extracted from `fatigue_test_encoder.cpp`
  - Monitors encoder position changes
  - Detects stalls when encoder stops moving while motor is commanded to move
  - Handles false stall detection with movement thresholds

- **StallGuard2** (`bounds_finder_stallguard.cpp`): Complete implementation extracted from `fatigue_test_stallguard.cpp`
  - Uses TMC51x0 StallGuard2 sensorless detection
  - Configures SGT threshold for homing
  - Handles false stall detection with movement thresholds

Both implementations follow the `IBoundsFinder` interface, allowing easy switching between methods. The method is selected via the `bounds_method_stallguard` setting in the UI.

### UART Command Interface

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

1. **No communication**: Check MAC addresses are correct
2. **CRC errors**: Check WiFi channel matches (default: channel 1)
3. **Buttons not working**: Verify GPIO pins and RTC capability
4. **Display not updating**: Implement e-ink drawing functions
5. **Bounds finding fails**: Check motor and encoder connections

## Implementation Details

### Unified Architecture

The test unit implementation (`fatigue_test_espnow_unit.cpp`) combines:

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

### File Organization

- **Main Application**: `fatigue_test_espnow_unit.cpp` (in `main/` directory)
- **Bounds Finders**: `test_unit/bounds_finder_*.cpp` (implementations)
- **Motion Controller**: Extracted inline or in separate header
- **UI Board**: Complete implementation in `ui_board/` directory

## Future Enhancements

- Settings editing UI (nested menu with value editing)
- Real-time parameter adjustment via ESP-NOW
- Data logging to SD card or flash
- Multiple test unit support (broadcast commands)
- Web interface for configuration
