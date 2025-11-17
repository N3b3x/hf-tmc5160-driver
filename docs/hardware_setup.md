---
layout: default
title: "🔌 Hardware Setup"
description: "Hardware wiring and connection guide for the TMC5160"
nav_order: 3
parent: "📚 Documentation"
permalink: /docs/hardware_setup/
---

# Hardware Setup

This guide covers the physical connections and hardware requirements for the TMC5160 stepper motor driver chip.

## Pin Connections

### SPI Interface Connections

```
MCU              TMC5160
─────────────────────────
3.3V      ────── VDD
GND       ────── GND
SCK       ────── SCK
MOSI      ────── SDI
MISO      ────── SDO
CS        ────── CSN
EN        ────── EN (optional, can be tied high)
DIR       ────── DIR (for step/dir mode)
STEP      ────── STEP (for step/dir mode)
```

### UART Interface Connections

```
MCU              TMC5160
─────────────────────────
3.3V      ────── VDD
GND       ────── GND
TX        ────── UART_RXD
RX        ────── UART_TXD
TX_EN     ────── UART_TXEN (optional, for transceiver)
```

### Motor Connections

```
TMC5160          Stepper Motor
──────────────────────────────
1B        ────── Motor Coil B+
1A        ────── Motor Coil B-
2B        ────── Motor Coil A+
2A        ────── Motor Coil A-
```

### Pin Descriptions

| Pin | Name | Description | Required |
|-----|------|-------------|----------|
| VDD | Power | 3.3V power supply | Yes |
| GND | Ground | Ground reference | Yes |
| SCK | Clock | SPI clock line | Yes (SPI) |
| SDI | Data In | SPI data input (MOSI) | Yes (SPI) |
| SDO | Data Out | SPI data output (MISO) | Yes (SPI) |
| CSN | Chip Select | SPI chip select (active low) | Yes (SPI) |
| UART_RXD | UART RX | UART receive data | Yes (UART) |
| UART_TXD | UART TX | UART transmit data | Yes (UART) |
| EN | Enable | Driver enable (active low) | Optional |
| DIR | Direction | Step/dir direction input | Optional |
| STEP | Step | Step pulse input | Optional |

## Power Requirements

- **Motor Supply Voltage**: 8V to 60V DC
- **Logic Supply Voltage**: 3.3V (VDD)
- **Current Consumption**: ~20mA typical (logic), motor current depends on load
- **Power Supply**: Separate motor supply recommended with proper decoupling

## Bus Configuration

### SPI Configuration

- **Mode**: SPI Mode 3 (CPOL=1, CPHA=1)
- **Speed**: Up to 4 MHz (recommended)
- **Bit Order**: MSB first
- **CS Polarity**: Active low

### UART Configuration

- **Baud Rate**: Typically 500000 bps (configurable)
- **Data Format**: 8 data bits, 1 stop bit, no parity
- **Slave Address**: 0-127 (7-bit address)

## Physical Layout Recommendations

- Keep SPI/UART traces short (< 10cm recommended)
- Use ground plane for noise reduction
- Place decoupling capacitors (100nF ceramic + 10µF electrolytic) close to VDD pin
- Use separate ground planes for motor power and logic if possible
- Route motor power traces away from sensitive logic signals
- Use twisted pair cables for motor connections

## Example Wiring Diagram

```
                    TMC5160
                    ┌─────────┐
        3.3V ───────┤ VDD     │
        GND  ───────┤ GND     │
        SCK  ───────┤ SCK     │
        MOSI ───────┤ SDI     │
        MISO ───────┤ SDO     │
        CS   ───────┤ CSN     │
        EN   ───────┤ EN      │
                    └─────────┘
```

## Next Steps

- Verify connections with a multimeter
- Proceed to [Quick Start](quickstart.md) to test the connection
- Review [Platform Integration](platform_integration.md) for software setup

---

**Navigation**
⬅️ [Quick Start](quickstart.md) | [Next: Platform Integration ➡️](platform_integration.md) | [Back to Index](index.md)

