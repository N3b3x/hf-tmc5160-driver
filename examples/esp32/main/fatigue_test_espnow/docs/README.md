# Fatigue Test ESP-NOW System Documentation

## Overview

This directory contains the fatigue test unit firmware that communicates with a remote controller via ESP-NOW. The system implements secure mutual authentication for device pairing and efficient bounds caching for improved test workflow.

## Documentation Index

| Document | Description |
|----------|-------------|
| [PAIRING_PROTOCOL.md](PAIRING_PROTOCOL.md) | Secure ESP-NOW pairing with HMAC-SHA256 authentication |
| [BOUNDS_CACHING.md](BOUNDS_CACHING.md) | Bounds finding cache system for efficient test workflow |

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          REMOTE CONTROLLER                                   │
│  ┌─────────────┐  ┌─────────────────┐  ┌────────────────┐                   │
│  │   Display   │  │  Encoder/Buttons │  │  ESP-NOW TX/RX │                   │
│  └─────────────┘  └─────────────────┘  └───────┬────────┘                   │
│                                                │                             │
│                     Pairing Manager            │                             │
│                     Peer Store (NVS)           │                             │
└────────────────────────────────────────────────┼─────────────────────────────┘
                                                 │
                                                 │ ESP-NOW (2.4 GHz)
                                                 │ - Config messages
                                                 │ - Commands
                                                 │ - Status updates
                                                 │ - Pairing protocol
                                                 │
┌────────────────────────────────────────────────┼─────────────────────────────┐
│                          FATIGUE TEST UNIT     │                             │
│                                                │                             │
│  ┌────────────────┐  ┌─────────────────────────▼──────────────────────────┐  │
│  │  UART Console  │  │              ESP-NOW Receiver                      │  │
│  │  - commands    │  │  - Pairing responder                               │  │
│  │  - status      │  │  - Message validation                              │  │
│  │  - config      │  │  - Peer store (NVS)                                │  │
│  └───────┬────────┘  └────────────────────────────────────────────────────┘  │
│          │                                                                   │
│          │           ┌────────────────────────────────────────────────────┐  │
│          └──────────►│              Motion Controller                     │  │
│                      │  - Bounds finding (StallGuard/Encoder)             │  │
│                      │  - Point-to-point motion                           │  │
│                      │  - Cycle counting                                  │  │
│                      │  - Bounds cache (2 min validity)                   │  │
│                      └────────────────────────────────────────────────────┘  │
│                                        │                                     │
│                                        ▼                                     │
│                      ┌────────────────────────────────────────────────────┐  │
│                      │              TMC5160 Driver                        │  │
│                      │  - SPI communication                               │  │
│                      │  - StallGuard2                                     │  │
│                      │  - Ramp generator                                  │  │
│                      └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Quick Start

### 1. Hardware Setup

Connect the TMC5160 driver to the ESP32 using the configured SPI pins (see `esp32_tmc51x0_test_config.hpp`).

### 2. First-Time Pairing

**On Fatigue Test Unit (UART):**
```
> pair
╔══════════════════════════════════════════════════════════════════════════════╗
║ Pairing mode enabled for 30 seconds.                                        ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

**On Remote Controller:**
1. Navigate to Settings → Pair Device
2. Press to initiate pairing
3. Wait for "Pairing Successful" message

### 3. Running a Test

**Quick workflow:**
```bash
# Configure test parameters
> set -f 0.5 -c 1000 -b -60 60

# Start test (auto-finds bounds)
> start

# Monitor status
> status
```

**Optimized workflow (with bounds cache):**
```bash
# Pre-find bounds
> bounds

# Adjust settings as needed
> set -f 0.7

# Start immediately (skips bounds finding)
> start
```

## UART Commands

| Command | Description |
|---------|-------------|
| `set [options]` | Configure test parameters |
| `start` | Start fatigue test |
| `stop` | Stop fatigue test |
| `pause` | Pause (motor de-energized) |
| `resume` | Resume from pause |
| `bounds` | Run bounds finding independently |
| `reset` | Reset cycle counter |
| `status` | Show current status |
| `pair` | Enter pairing mode (30 sec) |
| `help [cmd]` | Show help |

### SET Options

| Option | Description | Range |
|--------|-------------|-------|
| `-f, --frequency <Hz>` | Motion frequency | 0.01 - 10.0 |
| `-d, --dwell <min> <max>` | Dwell times (ms) | 0 - 60000 |
| `-b, --bounds <min> <max>` | Angle bounds (°) | -180 to 180 |
| `-c, --cycles <count>` | Target cycles | 0 = infinite |

## Security

### Pairing Secret Configuration

The pairing secret is **not hardcoded** in the source code. It's injected at build time for security.

**Quick Setup:**
```bash
# 1. Generate a secret
openssl rand -hex 16

# 2. Copy the template (in each project directory)
cp secrets.template.yml secrets.local.yml

# 3. Edit secrets.local.yml with your secret
# espnow_pairing_secret: "your_32_char_hex_secret"

# 4. Build (secret is automatically loaded)
./scripts/build_app.sh fatigue_test_espnow_unit Release
```

**Alternative Methods:**
```bash
# Via command line
./scripts/build_app.sh fatigue_test_espnow_unit Release --secret abc123...

# Via environment variable
export ESPNOW_PAIRING_SECRET="abc123..."
./scripts/build_app.sh fatigue_test_espnow_unit Release
```

**IMPORTANT:** Both devices must use the same secret!

### Approved Peers

- Stored in NVS (survives reboot)
- Maximum 4 peers per device
- Pre-configured MAC addresses automatically approved

## Troubleshooting

### "Not in pairing mode" error

1. Run `pair` on the fatigue test unit first
2. Then initiate pairing from the remote within 30 seconds

### "HMAC verification failed"

Devices have different `PAIRING_SECRET`. Ensure both are compiled with identical `espnow_security.hpp`.

### Bounds finding fails

1. Check mechanical limits are reachable
2. Verify StallGuard threshold is appropriate
3. Try encoder-based bounds finding

### Motor overheats

1. Reduce test frequency
2. Add dwell time between directions
3. Check bounds cache timeout settings

## File Structure

```
fatigue_test_espnow/
├── main.cpp                  # Application entry point, UART command handler
├── espnow_receiver.cpp/hpp   # ESP-NOW communication, pairing responder
├── espnow_protocol.hpp       # Protocol message definitions
├── espnow_security.hpp       # Security constants, HMAC functions
├── espnow_peer_store.cpp/hpp # NVS-based peer storage
├── fatigue_motion.cpp/hpp    # Motion controller, bounds finding
└── docs/
    ├── README.md             # This file
    ├── PAIRING_PROTOCOL.md   # Pairing protocol specification
    └── BOUNDS_CACHING.md     # Bounds cache system documentation
```

## Related Documentation

- TMC5160 Datasheet: `datasheet/TMC5160A_datasheet_rev1.18.pdf`
- Driver API: `docs/api_reference.md`
- Hardware Setup: `docs/hardware_setup.md`

