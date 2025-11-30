# Fatigue Test with ESP-NOW Communication

This directory contains a complete fatigue testing system with wireless remote control via ESP-NOW.

## Overview

The system consists of two ESP32 devices:

1. **UI Board** (`ui_board/`): Remote controller with e-ink display and buttons
   - Sends commands to test unit via ESP-NOW
   - Displays status and settings on e-ink display
   - Supports deep sleep wake from buttons
   - Inactivity timeout for power saving

2. **Test Unit** (`test_unit/`): Fatigue tester that receives commands
   - Receives commands from UI board via ESP-NOW
   - Performs bounds finding (StallGuard2 or encoder-based)
   - Runs sinusoidal fatigue test motion
   - Sends status updates back to UI board

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
    ├── main.cpp                 # Main application
    └── espnow_receiver.hpp/cpp  # ESP-NOW receiver implementation
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

```bash
cd examples/esp32
./scripts/build_app.sh fatigue_test_espnow_unit Release
```

Note: You'll need to add these app types to your build system.

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
- ESP-NOW command reception
- Bounds finding (StallGuard2 or encoder-based)
- Sinusoidal fatigue test motion
- Cycle counting
- Status updates sent to UI board
- Error handling and reporting

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

The UI code includes stubs for e-ink display integration. To integrate with Adafruit ThinkInk:

1. Include Adafruit libraries in `ui.cpp`
2. Implement drawing functions:
   - `draw_main_screen()`
   - `draw_settings_screen()`
   - `draw_error_screen()`
   - `draw_complete_screen()`
   - `update_status_footer()`

### Bounds Finding

The test unit includes simplified bounds finding. For full implementation:

- **Encoder-based**: See `fatigue_test_encoder.cpp` for complete encoder-based bounds finding
- **StallGuard2**: See `fatigue_test_stallguard.cpp` for complete StallGuard2-based bounds finding

The current implementation uses simplified bounds finding. You can replace the `find_bounds_encoder()` and `find_bounds_stallguard()` functions with the full implementations from the respective example files.

## Troubleshooting

1. **No communication**: Check MAC addresses are correct
2. **CRC errors**: Check WiFi channel matches (default: channel 1)
3. **Buttons not working**: Verify GPIO pins and RTC capability
4. **Display not updating**: Implement e-ink drawing functions
5. **Bounds finding fails**: Check motor and encoder connections

## Future Enhancements

- Full e-ink display integration
- Settings editing UI
- Real-time parameter adjustment
- Data logging
- Multiple test unit support
