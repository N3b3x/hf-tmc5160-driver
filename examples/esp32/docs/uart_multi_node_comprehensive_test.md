---
layout: default
title: "UART Communication Test"
description: "UART single-wire communication testing for TMC51x0 (single and multi-node)"
nav_order: 7
parent: "ESP32 Examples"
permalink: /docs/esp32-examples/uart-communication-test/
---

# UART Communication Test Suite

## Overview

The `uart_multi_node_comprehensive_test.cpp` tests TMC51x0 UART single-wire communication with 1 or more drivers. It exercises the full communication stack -- from raw register access through motor control -- over UART instead of SPI.

## Purpose

This test suite validates:
- ESP32 UART interface initialization and baud rate sync with the TMC5160
- Register read/write communication (IOIN, GCONF)
- Node address configuration (NODECONF register)
- Send delay configuration for multi-node systems
- Full motor control (ramp, enable, positioning) over UART

## Test Categories

### 1. Basic UART Communication

Initializes the driver, reads IOIN (chip version) and GCONF to verify the UART link, CRC, and frame structure all work.

**What it proves:** The `Esp32UART` class correctly implements `UartSend()` / `UartReceive()`, the TMC5160 auto-detects the baud rate from the sync frame, and CRC8 checksums are valid in both directions.

### 2. Node Address Configuration

Writes a new address (e.g. 42) via `ConfigureUartNodeAddress()`, then reads IOIN on that address to confirm the chip responds. Restores address 0 afterward.

**What it proves:** The NODECONF register write works, and the driver correctly tracks the address in software so subsequent reads use the new address.

### 3. Send Delay Configuration

Iterates through SENDDELAY values (0, 1, 2, 4, 8) and reads IOIN after each to verify communication still works. SENDDELAY controls how many bit-times the TMC5160 waits before replying.

**What it proves:** The receive timeout in `Esp32UART::UartReceive()` handles varying reply delays correctly. For multi-node systems, SENDDELAY >= 2 is required.

### 4. Motor Control via UART

Initializes ramp parameters, enables the motor, commands a 45-degree move, and polls `IsTargetReached()` until the motor arrives. This proves the full driver API works over UART.

**What it proves:** All driver subsystems (motorControl, rampControl) function correctly over UART, not just raw register access.

## Hardware Configuration

### Single-Driver Setup (default: `TEST_NODE_COUNT = 1`)

```
ESP32                TMC5160
GPIO 17 (TX) ------> SWN (pin 26 / DIAG0_SWN)
GPIO 16 (RX) <------ SWP (pin 27 / DIAG1_SWP)
GPIO 11      ------> DRV_ENN (enable)
GND          ------> SD_MODE (pin 21) -- LOW for UART mode
GND          ------> SPI_MODE (pin 22) -- LOW for UART mode
```

### Multi-Driver UART Bus

All chips share the same TX/RX lines. NAI/NAO pins form an addressing chain:

```
ESP32          Chip 1                Chip 2
TX  ---------- SWN ------------------- SWN
RX  ---------- SWP ------------------- SWP
GND ---------- NAI (SDI_CFG1)
               NAO (SDO_CFG0) -------- NAI (SDI_CFG1)
```

## Pin Configuration

Default pins (edit in the source file):

| Signal | GPIO | Description |
|--------|------|-------------|
| UART TX | 17 | ESP32 TX -> TMC5160 SWN |
| UART RX | 16 | ESP32 RX <- TMC5160 SWP |
| EN | 11 | Motor enable (active LOW) |
| SD_MODE | GND | Must be LOW for UART mode |
| SPI_MODE | GND | Must be LOW for UART mode |

## Test Configuration

Adjust these constants in the source file:

```cpp
static constexpr uint8_t TEST_NODE_COUNT = 1;   // Number of TMC nodes on the bus
static constexpr uart_port_t UART_PORT = UART_NUM_1;
static constexpr uint32_t UART_BAUD = 115200;   // TMC5160 auto-detects
static constexpr int UART_TX_PIN = 17;
static constexpr int UART_RX_PIN = 16;
static constexpr int EN_PIN      = 11;
```

Test sections can be individually enabled/disabled:

```cpp
static constexpr bool ENABLE_BASIC_COMM_TESTS = true;
static constexpr bool ENABLE_NODE_ADDRESS_TESTS = true;
static constexpr bool ENABLE_SEND_DELAY_TESTS = true;
static constexpr bool ENABLE_MOTOR_CONTROL_TESTS = true;
```

## Code Structure

### `Esp32UART` Communication Interface

Defined in `test_config/esp32_tmc51x0_bus.hpp`. Inherits from `tmc51x0::UartCommInterface<Esp32UART>` (CRTP) and implements:

| Method | Description |
|--------|-------------|
| `Initialize()` | Configures ESP-IDF UART driver, sets pins, flushes stale data |
| `UartSend()` | Writes frame bytes, waits for TX FIFO drain, flushes echo |
| `UartReceive()` | Reads reply bytes with 100 ms timeout |
| `GpioSet()` / `GpioRead()` | Same pin abstraction as `Esp32SPI` |
| `DelayMs()` / `DelayUs()` | FreeRTOS and ROM delays |

### Test Flow

1. `create_uart_drivers()` -- Creates `Esp32UART` + N driver instances
2. Each test initializes the driver with `DriverConfig` from the test rig config
3. Tests exercise register access, node addressing, or motor control
4. Results reported via the test framework

## Expected Behavior

### Typical Output (single node)

```
I UART_MultiNode_Test: ==============================================================
I UART_MultiNode_Test:   ESP32 TMC51x0 UART Communication Test Suite
I UART_MultiNode_Test: ==============================================================
I UART_MultiNode_Test: Node count: 1 | UART port: 1 | Baud: 115200 | TX=17 RX=16

I UART_MultiNode_Test: Driver 0: Initialized successfully
I UART_MultiNode_Test: Driver 0: GCONF = 0x00000004
I UART_MultiNode_Test: Driver 0: IOIN = 0x30XXXXXX (chip version: 0x30)
I UART_MultiNode_Test: [PASS] Basic UART Communication

I UART_MultiNode_Test: Node address updated to 42 (send delay: 2)
I UART_MultiNode_Test: Successfully read IOIN on address 42
I UART_MultiNode_Test: [PASS] Node Address Configuration

I UART_MultiNode_Test: SENDDELAY=0: read OK
I UART_MultiNode_Test: SENDDELAY=2: read OK
I UART_MultiNode_Test: SENDDELAY=8: read OK
I UART_MultiNode_Test: [PASS] Send Delay Configuration

I UART_MultiNode_Test: Driver 0: Moving to 45.0 degrees...
I UART_MultiNode_Test: All 1 driver(s) reached target position
I UART_MultiNode_Test: [PASS] Motor Control via UART
```

## Troubleshooting

### No Response from TMC5160

| Check | Fix |
|-------|-----|
| Mode pins | SD_MODE and SPI_MODE must both be LOW (GND) |
| TX/RX wiring | TX -> SWN (pin 26), RX <- SWP (pin 27) |
| Baud rate | Try 9600 or 57600 if 115200 doesn't work |
| Power | VM supply must be present for UART to work |
| UART port | Avoid `UART_NUM_0` (console); use `UART_NUM_1` or `UART_NUM_2` |

### CRC Errors or Garbled Data

- Check for loose connections on TX/RX
- Ensure no other device is driving the UART lines
- Reduce baud rate (TMC5160 minimum is ~9000 baud at 20 MHz clock)

### Motor Doesn't Move

- Verify EN pin is wired and motor supply (VM) is present
- Check IRUN/IHOLD settings match your sense resistors
- Read DRV_STATUS for fault flags (`drv_err`, `uv_cp`, `ot`)

## Related Documentation

- [SPI Daisy Chain Test](spi_daisy_chain_comprehensive_test.md) -- SPI multi-motor equivalent
- [Multi-Chip Communication](../../../docs/special_features_multi_chip.md) -- Detailed UART/SPI daisy chain guide
- [Communication Interface](../../../docs/special_features_communication_interface.md) -- UART protocol details
- [Platform Integration](../../../docs/platform_integration.md) -- Implementing `Esp32UART`

---

**Navigation:** [<- SPI Daisy Chain Test](spi_daisy_chain_comprehensive_test.md) | [Back to Index](index.md)
