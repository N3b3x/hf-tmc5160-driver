---
layout: default
title: "🔗 Multi-Chip Communication"
description: "SPI daisy chaining and UART multi-node addressing for TMC5160"
nav_order: 12
parent: "📚 Documentation"
permalink: /docs/special_features_multi_chip/
---

# Multi-Chip Communication

The TMC5160 driver supports multiple chips on a single communication bus through two methods:

1. **SPI Daisy Chaining**: Multiple TMC5160 chips can be connected on a single SPI bus using chip select (CSN) pins or extended SPI transfers.
2. **UART Multi-Node Addressing**: Up to 255 TMC5160 chips can be addressed on a single UART bus using slave addresses and NAI/NAO pin daisy chaining.

## Overview

When multiple TMC5160 chips are used in a system, each chip must be individually addressable. The driver provides methods for:

- **SPI**: Chip selection via CSN pins or extended transfers (>40 bits)
- **UART**: Slave address configuration via SLAVECONF register and NAI/NAO pin control

## SPI Daisy Chaining

### Hardware Setup

For SPI daisy chaining, you have two options:

#### Option 1: Multiple CSN Pins (Recommended)

Each TMC5160 chip has its own CSN (chip select) pin connected to the MCU:

```
MCU              TMC5160 #1          TMC5160 #2          TMC5160 #3
─────────────────────────────────────────────────────────────────────
SCK       ────── SCK                ────── SCK          ────── SCK
MOSI      ────── SDI                ────── SDI          ────── SDI
MISO      ────── SDO                ────── SDO          ────── SDO
CSN1      ────── CSN
CSN2                                ────── CSN
CSN3                                                    ────── CSN
```

### Software Implementation

Override `SetChipSelect()` in your SPI communication interface for multi-chip setups:

```cpp
class MyMultiChipSPI : public tmc5160::SpiCommInterface<MyMultiChipSPI> {
private:
  gpio_num_t csn_pins_[3]; // CSN pins for 3 chips

public:
  bool SetChipSelect(uint8_t csn_pin_index, bool active) noexcept override {
    if (csn_pin_index >= 3) {
      return false;
    }
    gpio_set_level(csn_pins_[csn_pin_index], active ? 0 : 1); // Active low
    return true;
  }

  // ... other required methods ...
};

// Usage
MyMultiChipSPI spi(/* ... */);
tmc5160::TMC5160 driver1(spi);
tmc5160::TMC5160 driver2(spi);
tmc5160::TMC5160 driver3(spi);

// Read from chip 1 (CSN index 0)
uint32_t value;
driver1.GetComm().ReadRegister(0x00, value, 0);

// Read from chip 2 (CSN index 1)
driver2.GetComm().ReadRegister(0x00, value, 1);

// Read from chip 3 (CSN index 2)
driver3.GetComm().ReadRegister(0x00, value, 2);
```

## UART Multi-Node Addressing

### Hardware Setup

For UART multi-node addressing, connect chips in a daisy chain using NAI/NAO pins:

```
MCU              TMC5160 #1          TMC5160 #2          TMC5160 #3
─────────────────────────────────────────────────────────────────────
TXD       ────── UART_RXD     ────── UART_RXD     ────── UART_RXD
RXD       ────── UART_TXD     ────── UART_TXD     ────── UART_TXD
NAI       ────── NAI (GND)
NAO       ────── NAI
                  NAO       ────── NAI
                              NAO       ────── NAI
```

### Sequential Addressing Procedure

To program addresses for multiple chips:

1. **First chip**: NAI tied to GND → responds to address 0
2. **Program first chip**: Set SLAVECONF.slaveaddr = 1, set NAO low
3. **Second chip**: Now responds to address 1
4. **Program second chip**: Set SLAVECONF.slaveaddr = 2, set NAO low
5. **Continue** for remaining chips

### Software Implementation

#### Configure Slave Address

Use the `Communication` subsystem to configure slave addresses:

```cpp
tmc5160::TMC5160 driver(uart_comm);

// Configure slave address for this chip
// send_delay should be >1 when multiple slaves are present
driver.communication.ConfigureSlaveAddress(1, 2); // Address 1, delay 2 bit times

// Get current slave address
uint8_t addr = driver.communication.GetSlaveAddress();
```

#### NAI/NAO Pin Control

Override `SetNaiPin()` and `GetNaoPin()` in your UART interface:

```cpp
class MyUART : public tmc5160::UartCommInterface<MyUART> {
private:
  gpio_num_t nai_pin_;
  gpio_num_t nao_pin_;

public:
  bool SetNaiPin(bool active) noexcept override {
    gpio_set_level(nai_pin_, active ? 1 : 0);
    return true;
  }

  bool GetNaoPin(bool &active) noexcept override {
    int level = gpio_get_level(nao_pin_);
    active = (level == 1);
    return true;
  }

  // ... other required methods ...
};
```

#### Sequential Address Programming Example

```cpp
void programSlaveAddresses() {
  MyUART uart(/* ... */);
  
  // First chip: NAI tied to GND (address 0)
  uart.SetNaiPin(false); // Set NAI low
  uart.SetSlaveAddress(0);
  
  tmc5160::TMC5160 driver1(uart);
  driver1.communication.ConfigureSlaveAddress(1, 2); // Program to address 1
  
  // Set NAO low to enable next chip
  bool nao_state;
  uart.GetNaoPin(nao_state);
  if (nao_state) {
    // Read NAO from chip 1, connect to NAI of chip 2
    // In hardware: chip1.NAO -> chip2.NAI
    // Software: Set chip 2's NAI based on chip 1's NAO
  }
  
  // Second chip: Now responds to address 1
  uart.SetSlaveAddress(1);
  tmc5160::TMC5160 driver2(uart);
  driver2.communication.ConfigureSlaveAddress(2, 2); // Program to address 2
  
  // Continue for remaining chips...
}
```

## Communication Interface Methods

### SPI Methods

| Method | Description |
|--------|-------------|
| `SetChipSelect(csn_pin_index, active)` | Set CSN pin state for chip selection |
| `ReadRegister(address, value, csn_pin_index)` | Read register with optional chip selection |
| `WriteRegister(address, value, csn_pin_index)` | Write register with optional chip selection |

### UART Methods

| Method | Description |
|--------|-------------|
| `SetSlaveAddress(address)` | Set 7-bit slave address (0-127) |
| `GetSlaveAddress()` | Get current slave address |
| `SetNaiPin(active)` | Set NAI pin state for daisy chaining |
| `GetNaoPin(active)` | Read NAO pin state |
| `ConfigureSlaveAddress(address, send_delay)` | Configure SLAVECONF register |

## Best Practices

1. **SPI Multi-Chip Setup**:
   - Use separate CSN pins for each chip for better performance and reliability
   - Ensure CSN timing requirements are met (minimum 2*tclk + 10ns)
   - Always deassert CSN after transactions to avoid bus conflicts

2. **UART Multi-Node**:
   - Set `send_delay` > 1 when multiple slaves are present
   - Program addresses sequentially starting from address 0
   - Verify addresses after programming

3. **Error Handling**:
   - Always check return values from communication methods
   - Implement timeout handling for multi-chip operations
   - Verify chip selection before register access

## Limitations

- **SPI**: Each chip requires a separate CSN pin on the MCU
- **UART**: Maximum 255 nodes (7-bit address space)
- **Timing**: Ensure proper timing between chip selection and data transfer
- **Hardware**: NAI/NAO pins require external connections for UART daisy chaining

---

**Navigation**
⬅️ [Previous: Advanced Configuration](special_features_advanced_configuration.md) | [Next: Troubleshooting ➡️](troubleshooting.md) | [Docs Hub 📚](index.md)

