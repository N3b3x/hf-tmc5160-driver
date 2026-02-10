# Communication Interface

The TMC51x0 driver (TMC5130 & TMC5160) provides a flexible and robust communication interface layer that supports both **SPI** (Serial Peripheral Interface) and **UART** (Single Wire Interface) protocols. This abstraction allows the core driver logic to remain platform-agnostic while ensuring strict compliance with the TMC51x0 datasheet specifications.

## Overview

The communication layer is implemented using a **CRTP (Curiously Recurring Template Pattern)** base class `CommInterface`, which defines the standard API for register access (`ReadRegister`, `WriteRegister`) and GPIO control. Derived classes (`SpiCommInterface`, `UartCommInterface`) implement the specific protocol details.

| Feature | SPI Interface | UART Interface |
| :--- | :--- | :--- |
| **Type** | Synchronous, 4-wire | Asynchronous, Single Wire (Half-Duplex) |
| **Speed** | High (up to fCLK/2) | Medium (9600 baud to fCLK/16) |
| **Topology** | Point-to-Point or Daisy Chain | Multi-Drop Bus (up to 255 nodes) |
| **Addressing** | Chip Select (CSN) or Daisy Chain Position | 8-bit Node Address |
| **Reliability** | Standard SPI | CRC8 Checksum protected |

---

## SPI Interface

The SPI interface is the standard high-speed communication method for the TMC51x0 (TMC5130 & TMC5160). It uses a **40-bit (5-byte)** datagram format.

### Datagram Structure

Communication is always performed in 40-bit blocks. The MSB is transmitted first.

```ascii
  MSB                                                                   LSB
  Bit 39 ... ... ... ... ... ... ... ... ... ... ... ... ... ... ... Bit 0
  +-----------------------+-----------------------------------------------+
  |    Address Byte       |                  Data Word                    |
  +---+-------------------+-----------------------------------------------+
  |RW | 7-Bit Register    |                 32-Bit Data                   |
  |   |    Address        |             (MSB ...... LSB)                  |
  +---+-------------------+-----------------------------------------------+
  | 7 | 6 . . . . . . 0   | 31 . . . . . . . . . . . . . . . . . . . .  0 |
  +---+-------------------+-----------------------------------------------+
```

*   **RW (Read/Write)**: `0` for Read, `1` for Write.
*   **Address**: 7-bit register address (0x00 to 0x7F).
*   **Data**: 32-bit data payload.

### Read/Write Operations

*   **Write Access**:
    1.  Master sends: `[ 1 | Address ] + [ 32-bit Value ]` (40 bits total).
    2.  TMC51x0 responds: `[ SPI_STATUS ] + [ Previous Data ]`.
    3.  Status flags (e.g., Driver Error, Reset) are returned in the first byte.

*   **Read Access (Pipelined)**:
    1.  **Request**: Master sends `[ 0 | Address ] + [ Dummy Data ]`.
    2.  **Response**: TMC5160 responds with `[ SPI_STATUS ] + [ Dummy/Previous Data ]`.
    3.  **Retrieval**: To get the requested data, the master must perform a **second** transaction (Read or Write). The data requested in the *first* transaction is returned in the *second* transaction.
    *   *Note: The driver handles this pipelining automatically in `ReadRegister`.*

### Daisy Chaining

The driver supports controlling multiple TMC5160 chips in a single SPI chain. The command for a specific chip is shifted through preceding chips.

**Setup:**
*   Connect SDO of Chip 1 -> SDI of Chip 2, etc.
*   Share SCK and CSN lines.

**Protocol:**
To address device `K` in a chain of `N` devices:
1.  Send `(K)` padding datagrams (dummy data) to shift the command to the target.
2.  Send the 40-bit Command.
3.  Send `(N-1-K)` padding datagrams to fill the chain.

This ensures the command latches into the correct device when CSN goes high.

---

## UART Single Wire Interface

The UART interface allows controlling multiple devices using a single wire (plus ground) or RS485 transceiver. It uses a packet-based protocol with CRC8 protection.

### Write Access Datagram (64 bits / 8 bytes)

Used to write data to a register.

```ascii
  Byte 0       Byte 1       Byte 2       Byte 3 .. 6      Byte 7
+------------+------------+------------+----------------+------------+
| Sync + Rsv | Node Addr  | RW + Addr  |  32-Bit Data   |    CRC8    |
+------------+------------+------------+----------------+------------+
| 0x05 (0xA) | 0 ... 254  | 1 | RegAddr|  MSB ... LSB   |  Checksum  |
+------------+------------+------------+----------------+------------+
```

*   **Sync**: `0x05` (Corresponds to `1010` binary pattern on wire LSB-first).
*   **Node Addr**: 8-bit unique address for the target device.
*   **RW + Addr**: Bit 7 is `1` (Write), Bits 0-6 are register address.
*   **CRC8**: Calculated over Bytes 0 to 6 using polynomial `0x07`.

### Read Request Datagram (32 bits / 4 bytes)

Used to request data from a register.

```ascii
  Byte 0       Byte 1       Byte 2       Byte 3
+------------+------------+------------+------------+
| Sync + Rsv | Node Addr  | RW + Addr  |    CRC8    |
+------------+------------+------------+------------+
| 0x05 (0xA) | 0 ... 254  | 0 | RegAddr|  Checksum  |
+------------+------------+------------+------------+
```

*   **RW + Addr**: Bit 7 is `0` (Read).
*   **CRC8**: Calculated over Bytes 0 to 2.

### Read Reply Datagram (64 bits / 8 bytes)

The device response to a Read Request.

```ascii
  Byte 0       Byte 1       Byte 2       Byte 3 .. 6      Byte 7
+------------+------------+------------+----------------+------------+
| Sync + Rsv | Master Addr| Reg Addr   |  32-Bit Data   |    CRC8    |
+------------+------------+------------+----------------+------------+
| 0x05 (0xA) |    0xFF    |    0x00    |  MSB ... LSB   |  Checksum  |
+------------+------------+------------+----------------+------------+
```

*   **Master Addr**: Always `0xFF`.
*   **Reg Addr**: Always `0x00` in reply.
*   **Data**: The requested register value.

---

## Implementation Details

### CRC8 Calculation
The UART interface uses the standard CRC8-ATM polynomial:
\[ CRC = x^8 + x^2 + x^1 + x^0 \]
(Polynomial: `0x07`, Initial Value: `0x00`)

### Addressing
*   **SPI**:Addressed by `daisy_chain_position` (0 = first device).
*   **UART**: Addressed by `node_address` (0-254).

## Usage Example

```cpp
// 1. Define your specific Communication Interface
class MySPI : public tmc51x0::SpiCommInterface<MySPI> {
public:
    // Implement required low-level transfer
    bool SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) {
        // Hardware SPI call...
        return true;
    }
};

// 2. Instantiate Interface and Driver
MySPI spi_interface;
tmc51x0::TMC51x0<MySPI> motor_driver(spi_interface);

// 3. Initialize
tmc51x0::DriverConfig config;
motor_driver.Initialize(config);

// 4. Move Motor
motor_driver.rampControl.SetTargetPosition(51200);
```

---

**Navigation**
[← Multi-Chip Communication](special_features_multi_chip.md) | [Back to Index](index.md) | [Next: GPIO Pin Configuration →](gpio_pin_configuration.md)

