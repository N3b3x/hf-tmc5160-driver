# Fatigue Test ESP-NOW Unit - Architecture Documentation

## Overview

The Fatigue Test ESP-NOW Unit is a unified fatigue testing system that combines motor control, bounds detection, and wireless communication. It supports dual communication interfaces (ESP-NOW and UART) and dual bounds detection methods (StallGuard2 and encoder-based).

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Fatigue Test Unit                         │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │  ESP-NOW     │    │    UART      │    │   Motion    │  │
│  │  Receiver    │───▶│   Command    │───▶│  Controller │  │
│  │              │    │   Parser     │    │             │  │
│  └──────────────┘    └──────────────┘    └──────┬───────┘  │
│         │                  │                      │          │
│         └──────────────────┴──────────────────────┘          │
│                            │                                  │
│                   ┌────────▼────────┐                         │
│                   │  Settings Store │                         │
│                   └─────────────────┘                         │
│                            │                                  │
│         ┌──────────────────┴──────────────────┐              │
│         │                                      │              │
│  ┌──────▼────────┐                    ┌───────▼──────┐      │
│  │   Bounds      │                    │   TMC51x0    │      │
│  │   Finder      │                    │   Driver     │      │
│  │               │                    │              │      │
│  │ ┌──────────┐  │                    └───────┬──────┘      │
│  │ │StallGuard│  │                            │              │
│  │ └──────────┘  │                            │              │
│  │ ┌──────────┐  │                    ┌───────▼──────┐      │
│  │ │ Encoder  │  │                    │     Motor     │      │
│  │ └──────────┘  │                    │   Hardware   │      │
│  └───────────────┘                    └───────────────┘      │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Component Overview

### 1. ESP-NOW Communication Layer

**Files**: `espnow_protocol.hpp`, `espnow_receiver.hpp/cpp`

- **Purpose**: Wireless communication with remote controller
- **Protocol**: Custom ESP-NOW protocol with CRC16-CCITT error detection
- **Features**:
  - Sync byte validation (0xAA)
  - Sequence ID tracking
  - CRC16-CCITT checksum
  - Queue-based event delivery

**Key Classes/Functions**:
- `EspNowReceiver::init()` - Initialize ESP-NOW receiver
- `EspNowReceiver::send_*()` - Send various message types
- `ProtoEvent` - Event structure for higher layers

### 2. UART Command Interface

**Location**: `main.cpp` (UartCommandParser)

- **Purpose**: Direct serial control for debugging and development
- **Protocol**: Text-based command parser
- **Commands**: See README.md for full command list

### 3. Bounds Detection System

**Files**: `bounds_finder.hpp`, `bounds_finder_stallguard.cpp`, `bounds_finder_encoder.cpp`

- **Purpose**: Detect physical limits of motion
- **Architecture**: Interface-based design with multiple implementations

**Interface**:
```cpp
class IBoundsFinder {
    virtual bool FindBounds(int32_t& min_pos, int32_t& max_pos) = 0;
    virtual void Reset() = 0;
};
```

**Implementations**:
- **StallGuardBoundsFinder**: Uses TMC51x0 StallGuard2 sensorless detection
- **EncoderBoundsFinder**: Monitors encoder position changes

### 4. Motion Controller

**Files**: `fatigue_motion.hpp`, `fatigue_motion_impl.hpp`

- **Purpose**: Control sinusoidal back-and-forth motion
- **Features**:
  - Sinusoidal motion generation
  - Cycle counting (center-crossing detection)
  - Dwell time at bounds
  - Thread-safe operation (RAII mutex guards)

**Key Methods**:
- `SetFrequency()` - Set motion frequency
- `SetTargetCycles()` - Set target cycle count
- `SetDwellTimes()` - Set dwell time at bounds
- `Start()` / `Pause()` / `Resume()` / `Stop()` - Control motion

### 5. Settings Management

**Location**: `espnow_protocol.hpp` (Settings structure)

- **Purpose**: Store and synchronize test parameters
- **Storage**: NVS (Non-Volatile Storage) for persistence
- **Synchronization**: ESP-NOW protocol for remote updates

**Settings Structure**:
```cpp
struct TestUnitSettings {
    uint32_t cycle_amount;        // Target cycles
    uint32_t time_per_cycle;      // Seconds per cycle
    uint32_t dwell_time;          // Dwell time at bounds
    bool bounds_method_stallguard; // Detection method
};
```

## Task Architecture

The system uses FreeRTOS tasks for concurrent operation:

1. **espnow_command_task**: Processes ESP-NOW commands
2. **motion_control_task**: Controls motor motion
3. **status_update_task**: Sends periodic status updates
4. **UART task**: Handles serial commands (inline in main)

## Data Flow

### Command Flow (ESP-NOW)

```
Remote Controller
    │
    │ ESP-NOW Packet
    ▼
ESP-NOW Receiver (ISR)
    │
    │ ProtoEvent (Queue)
    ▼
espnow_command_task
    │
    │ Update Settings / Command
    ▼
Motion Controller
    │
    │ Motor Commands
    ▼
TMC51x0 Driver
    │
    │ SPI Commands
    ▼
Motor Hardware
```

### Status Flow

```
Motion Controller
    │
    │ Status Data
    ▼
status_update_task
    │
    │ StatusUpdate Message
    ▼
ESP-NOW Sender
    │
    │ ESP-NOW Packet
    ▼
Remote Controller
```

## Thread Safety

- **Mutex Protection**: TMC driver access protected by `Esp32TmcMutex`
- **Queue Communication**: Inter-task communication via FreeRTOS queues
- **RAII Guards**: Automatic mutex locking/unlocking

## Error Handling

- **Protocol Errors**: CRC validation, sync byte checking
- **Bounds Finding Errors**: Timeout detection, false stall prevention
- **Motion Errors**: State validation, bounds checking
- **Communication Errors**: Retry logic, error reporting

## Memory Management

- **Stack Allocation**: Task stacks sized appropriately
- **Heap Allocation**: Display buffers, queues
- **Static Allocation**: Global state, settings

## Performance Considerations

- **Motion Control**: High-frequency updates (100Hz+)
- **Status Updates**: Lower frequency (1-10Hz)
- **ESP-NOW**: Event-driven, low latency
- **UART**: Blocking, but low priority

## Extension Points

1. **New Bounds Finder**: Implement `IBoundsFinder` interface
2. **New Motion Patterns**: Extend `FatigueTestMotion` class
3. **New Commands**: Add to UART parser and ESP-NOW protocol
4. **New Settings**: Extend `TestUnitSettings` structure

