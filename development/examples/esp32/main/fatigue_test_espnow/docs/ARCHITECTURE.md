# Fatigue Test ESP-NOW Unit - Architecture Documentation

## Overview

The Fatigue Test ESP-NOW Unit is a unified fatigue testing system that combines motor control, bounds detection, and wireless communication. It supports dual communication interfaces (ESP-NOW and UART) and dual bounds detection methods (StallGuard2 and encoder-based).

## System Architecture

```
┌───────────────────────────────────────────────────────────────────────────┐
│                         Fatigue Test Unit                                 │
├───────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌─────────────────┐       ┌─────────────────┐       ┌─────────────────┐  │
│  │   ESP-NOW       │       │   FreeRTOS      │       │   Motion        │  │
│  │   Receiver      │──────▶│   Event Queue   │──────▶│   Controller    │  │
│  │   (ISR-safe)    │       │                 │       │ (Point-to-Point)│  │
│  └─────────────────┘       └─────────────────┘       └────────┬────────┘  │
│           │                                                    │          │
│           │                ┌─────────────────┐                 │          │
│           │                │  Global State   │                 │          │
│           └───────────────▶│  & Settings     │◀────────────────┘          │
│                            └────────┬────────┘                            │
│                                     │                                     │
│        ┌────────────────────────────┼────────────────────────────┐        │
│        │                            │                            │        │
│        ▼                            ▼                            ▼        │
│  ┌───────────────┐          ┌───────────────┐          ┌───────────────┐  │
│  │    UART       │          │   Bounds      │          │   TMC51x0     │  │
│  │   Command     │          │   Finding     │          │   Driver      │  │
│  │   Parser      │          │   (Library)   │          │   (SPI)       │  │
│  └───────────────┘          └───────────────┘          └───────┬───────┘  │
│                                     │                          │          │
│                                     │                          │          │
│                             ┌───────▼───────┐          ┌───────▼───────┐  │
│                             │   driver.     │          │    Motor      │  │
│                             │   homing      │          │   Hardware    │  │
│                             │   FindBounds  │          │               │  │
│                             └───────────────┘          └───────────────┘  │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
```

## Component Overview

### 1. ESP-NOW Communication Layer

**Files**: `espnow_protocol.hpp`, `espnow_receiver.hpp/cpp`

- **Purpose**: Wireless communication with remote controller
- **Protocol Version**: 1
- **Header Size**: 6 bytes (sync, version, device_id, type, id, len)
- **Max Payload**: 200 bytes
- **Error Detection**: CRC16-CCITT

**Features**:
- Sync byte validation (0xAA)
- Protocol version checking
- Sequence ID tracking
- CRC16-CCITT checksum
- Queue-based event delivery (ISR-safe)
- MAC address learning (optional pre-configuration)

**Key Functions**:
```cpp
EspNowReceiver::init(QueueHandle_t event_queue);
EspNowReceiver::send_config_response(const Settings& s);
EspNowReceiver::send_config_ack(bool ok, uint8_t err_code);
EspNowReceiver::send_status_update(uint32_t cycle, TestState state);
EspNowReceiver::send_error(uint8_t err_code, uint32_t at_cycle);
EspNowReceiver::send_test_complete();
```

### 2. UART Command Interface

**Location**: `main.cpp` (UartCommandParser)

- **Purpose**: Direct serial control for debugging and development
- **Protocol**: Text-based command parser
- **Baud Rate**: 115200
- **Commands**: `-f`, `-d`, `-b`, `-c`, `-a`, `-s`, `-h`

### 3. Bounds Detection System

**Implementation**: TMC51x0 driver library built-in homing subsystem

- **Purpose**: Detect physical limits of motion and establish coordinate frame
- **API**: `g_driver->homing.FindBounds(method, options, home_config, cancel_cb)`

**Supported Methods**:
| Method | Description |
|--------|-------------|
| `StallGuard` | Sensorless stall detection via TMC51x0 StallGuard2 |
| `Encoder` | Encoder-based bounds detection |
| `Switch` | Limit switch detection (available in library) |

**BoundsOptions Structure** (passed to FindBounds):
```cpp
struct BoundsOptions {
    float search_speed;           // Search velocity (RPM)
    float search_span;            // Max search distance (degrees)
    float backoff_distance;       // Backoff from detected bound (degrees)
    uint32_t timeout_ms;          // Search timeout
    float search_accel;           // Search acceleration (rev/s²)
    float search_decel;           // Search deceleration (rev/s²)
    float current_reduction_factor; // Current reduction during SG search
    StallGuardConfig* stallguard_override; // Optional SG configuration
    Unit speed_unit;              // Unit for speed values
    Unit position_unit;           // Unit for position values
    Unit accel_unit;              // Unit for acceleration values
};
```

### 4. Motion Controller

**Files**: `fatigue_motion.hpp`, `fatigue_motion_impl.hpp`

- **Purpose**: Generate point-to-point back-and-forth motion between bounds
- **Features**:
  - Point-to-point position control (direct VMAX/AMAX)
  - Cycle counting (center-crossing detection)
  - Configurable dwell time at bounds
  - Thread-safe operation (RAII mutex guards)
  - Support for both bounded and unbounded operation

**Key Methods**:
```cpp
void SetFrequency(float freq_hz);
void SetTargetCycles(uint32_t cycles);
void SetDwellTimes(uint32_t min_ms, uint32_t max_ms);
void SetGlobalBounds(float min_deg, float max_deg);
void SetLocalBoundsFromCenterDegrees(float range);
void Start();
void Pause();
void Resume();
void Stop();
FatigueStatus GetStatus() const;
```

### 5. Settings Management

**Location**: `espnow_protocol.hpp`

Settings are stored in memory and synchronized via ESP-NOW. Extended fields support backward compatibility.

**TestUnitSettings Structure**:
```cpp
struct TestUnitSettings {
    // Base fields (always synchronized)
    uint32_t cycle_amount   = 1000;   // Target cycles
        float    oscillation_vmax_rpm = 60.0f;    // Max velocity during oscillation (RPM)
        float    oscillation_amax_rev_s2 = 10.0f; // Acceleration during oscillation (rev/s²)
        uint32_t dwell_time_ms = 1000;            // Dwell time at endpoints (milliseconds)
    bool bounds_method_stallguard = true; // Detection method
    
    // Extended fields (0.0f = use TestConfig defaults)
    float bounds_search_velocity_rpm = 0.0f;       // Search speed (RPM)
    float stallguard_min_velocity_rpm = 0.0f;      // SG min velocity (RPM)
        int8_t stallguard_sgt = 127;                   // StallGuard threshold [-64..63], 127=default
    float stall_detection_current_factor = 0.0f;   // Current reduction (0.0-1.0)
    float bounds_search_accel_rev_s2 = 0.0f;       // Search acceleration (rev/s²)
};
```

**Extended Field Behavior**:
- Value of `0.0f` → test unit uses TestConfig defaults
- Non-zero value → overrides the default
- Ensures backward compatibility with older remote controllers

## Task Architecture

The system uses FreeRTOS tasks for concurrent operation:

| Task | Priority | Stack | Period | Description |
|------|----------|-------|--------|-------------|
| `espnow_command_task` | 5 | 4KB | Event-driven | Process ESP-NOW commands |
| `motion_control_task` | 5 | 8KB | 10ms | Motor motion updates |
| `status_update_task` | 3 | 4KB | 1000ms | Send status to remote |
| `uart_command_task` | 3 | 4KB | 50ms | Process UART commands |
| `bounds_finding_task` | 4 | 8KB | Dynamic | Created on START, deleted on completion |

### Task Communication

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ ESP-NOW ISR     │────▶│  Raw RX Queue   │────▶│ espnow_recv_task│
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
                                                         ▼
                                               ┌─────────────────┐
                                               │ ProtoEvent Queue│
                                               └────────┬────────┘
                                                        │
                        ┌───────────────────────────────┼──────────────────┐
                        │                               │                  │
                        ▼                               ▼                  ▼
               ┌─────────────────┐           ┌─────────────────┐  ┌─────────────┐
               │espnow_cmd_task  │           │bounds_find_task │  │motion_ctrl  │
               │(commands)       │           │(on-demand)      │  │task         │
               └─────────────────┘           └─────────────────┘  └─────────────┘
```

## State Machine

```
                        ┌─────────────────┐
                        │      IDLE       │◀─────────────────────────────────┐
                        │  (motor off)    │                                  │
                        └────────┬────────┘                                  │
                                 │                                           │
                                 │ START command                             │
                                 ▼                                           │
                        ┌─────────────────┐                                  │
                        │ BOUNDS_FINDING  │                                  │
                        │ (task running)  │                                  │
                        └────────┬────────┘                                  │
                                 │                                           │
                    ┌────────────┼────────────┐                              │
                    │            │            │                              │
                    ▼            ▼            ▼                              │
           ┌──────────────┐  ┌──────────┐  ┌──────────┐                      │
           │ Bounds found │  │ Cancelled│  │ Timeout/ │                      │
           │              │  │ (PAUSE)  │  │  Error   │                      │
           └──────┬───────┘  └────┬─────┘  └────┬─────┘                      │
                  │               │             │                            │
                  │               │             └────────────────────────────┤
                  │               │                                          │
                  ▼               ▼                                          │
         ┌─────────────────┐   ┌─────────────────┐                           │
         │    RUNNING      │   │     PAUSED      │                           │
         │(point-to-point) │◀─▶│  (motor off)    │───────────────────────────┤
         └────────┬────────┘   └─────────────────┘        STOP               │
                  │                    ▲                                     │
                  │ PAUSE              │ RESUME                              │
                  └────────────────────┘                                     │
                  │                                                          │
                  │ Target cycles reached                                    │
                  ▼                                                          │
         ┌─────────────────┐                                                 │
         │   COMPLETED     │─────────────────────────────────────────────────┘
         │ (TEST_COMPLETE) │
         └─────────────────┘
```

## Data Flow

### Command Flow (ESP-NOW → Motor)

```
Remote Controller
        │
        │ ESP-NOW Packet [hdr:6][payload:N][crc:2]
        ▼
ESP-NOW Receiver (ISR) ─── espnowRecvCb()
        │
        │ RawMsg (data + len)
        ▼
s_raw_recv_queue (FreeRTOS)
        │
        ▼
recvTask() ─── handlePacket()
        │  • CRC validation
        │  • Header parsing
        │  • Payload extraction
        ▼
ProtoEvent {type, data}
        │
        ▼
g_espnowQueue (FreeRTOS)
        │
        ▼
espnow_command_task()
        │  • ConfigSet → update g_settings
        │  • CommandStart → RequestStart()
        │  • CommandPause → g_cancel_bounds, Stop()
        │  • etc.
        ▼
bounds_finding_task() or motion_control_task()
        │
        │ g_driver->homing.FindBounds() or g_motion->Update()
        ▼
TMC51x0 Driver (SPI)
        │
        ▼
Motor Hardware
```

### Status Flow (Motor → Remote)

```
motion_control_task() or bounds_finding_task()
        │
        │ Motion status (cycles, position, state)
        ▼
g_motion->GetStatus()
        │
        ▼
status_update_task() (every 1000ms if RUNNING/BOUNDS_FINDING)
        │
        │ EspNowReceiver::send_status_update(cycle, state)
        ▼
sendPacketToUi(MsgType::StatusUpdate, StatusPayload)
        │  • Build header
        │  • Copy payload
        │  • Calculate CRC
        │  • Place CRC at correct offset
        ▼
esp_now_send(s_ui_board_mac, buffer, len)
        │
        │ ESP-NOW Packet [hdr:6][StatusPayload:6][crc:2]
        ▼
Remote Controller
```

## Thread Safety

### Mutex Protection
- **TMC Driver Access**: Protected by `Esp32TmcMutex` for all SPI operations
- **Motion Controller**: Internal mutex for state access

### Queue Communication
- **ProtoEvent Queue**: ESP-NOW events to command task
- **Raw RX Queue**: ISR to receive task (ISR-safe xQueueSendFromISR)

### RAII Guards
```cpp
// Automatic mutex locking/unlocking
{
    std::lock_guard<std::mutex> lock(motion_mutex_);
    // Safe access to motion state
}
```

## Error Handling

| Error Type | Detection | Response |
|------------|-----------|----------|
| Protocol CRC | CRC mismatch | Packet dropped, log warning |
| Protocol Version | Version check | Packet dropped, log warning |
| Bounds Finding Timeout | Timer expiration | Error state, report to remote |
| Bounds Not Found | Library result | Error state, report to remote |
| Motion Error | State validation | Error state, motor disabled |

## Memory Management

| Type | Usage | Notes |
|------|-------|-------|
| **Stack** | Task stacks (4-8KB each) | Sized for worst-case recursion |
| **Heap** | FreeRTOS queues | Created at init, fixed size |
| **Static** | Global state, settings, driver | Single instances |
| **BSS** | Uninitialized globals | Zero-initialized at startup |

## Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Motion Update Rate | 100 Hz | 10ms period in motion_control_task |
| Status Update Rate | 1 Hz | 1000ms period in status_update_task |
| ESP-NOW Latency | < 10ms | Event-driven, low latency |
| Command Response | < 50ms | Including bounds check |
| Bounds Finding | 5-30s | Depends on search speed/span |

## Extension Points

1. **New Motion Patterns**: Extend `FatigueTestMotion` class
   - Add new motion profile generators
   - Implement custom cycle counting logic

2. **New Commands**: Extend protocol
   - Add to `MsgType` enum in both projects
   - Implement handler in `espnow_command_task`

3. **New Settings**: Extend `TestUnitSettings`
   - Add field to struct
   - Update CONFIG_SET handler
   - Update remote controller payload

4. **New Status Fields**: Extend `StatusPayload`
   - Add fields to struct
   - Update send_status_update()
   - Update remote controller handler

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| ESP-IDF | 5.x | FreeRTOS, WiFi, ESP-NOW |
| TMC51x0 Driver Library | Latest | Motor control, homing |
| FreeRTOS | (ESP-IDF) | Task management, queues |
