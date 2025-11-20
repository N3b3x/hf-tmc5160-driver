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

## Architecture Overview

The TMC5160 driver architecture supports multiple chips on a single communication bus. The key architectural principle is:

> **Daisy-chain position is a property of the `TMC5160` driver instance, not the communication interface.**

This allows multiple `TMC5160` instances to share the same `SpiCommInterface` instance while each maintaining its own position in the daisy chain.

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Single SPI Bus                           │
│  (One SpiCommInterface instance shared by all drivers)      │
└─────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
┌───────▼──────┐   ┌────────▼────────┐   ┌──────▼──────┐
│ TMC5160 #1   │   │  TMC5160 #2     │   │ TMC5160 #3  │
│              │   │                 │   │             │
│ position = 0 │   │  position = 1   │   │ position = 2│
│              │   │                 │   │             │
│ daisy_chain_ │   │  daisy_chain_   │   │ daisy_chain_│
│ position_    │   │  position_      │   │ position_   │
└───────┬──────┘   └────────┬────────┘   └──────┬──────┘
        │                   │                   │
        └───────────────────┼───────────────────┘
                            │
                    All pass their position
                    when calling ReadRegister/
                    WriteRegister
```

### Key Concepts

- **One `SpiCommInterface` instance** can be shared by multiple `TMC5160` drivers
- **Each `TMC5160` instance** has its own `daisy_chain_position_` (set in constructor)
- **Daisy-chain position** determines how much padding is added to reach the target chip
- **CSN (chip select)** is shared by all chips (tied together) and handled automatically

### Daisy-Chain Position

- **Position 0**: First chip (no padding needed)
- **Position 1**: Second chip (5 bytes padding)
- **Position 2**: Third chip (10 bytes padding)
- **Position N**: (N * 5 bytes padding)

## SPI Daisy Chaining

### Hardware Setup

For SPI daisy chaining, connect multiple TMC5160 chips on a single SPI bus:

```

               +-------------+       +-------------+       +-------------+
               | MCU / Host  |       | TMC5160 #1  |       | TMC5160 #2  |
               | Controller  |       |   (First)   |  ...  |   (Last)    |
               +-------------+       +-------------+       +-------------+
                                        (SDI/SDO are the data pins)
                                                                
SCK (Clock)    +----------------------> SCK (Input) -----> SCK (Input)
               |
CSN (Select)   +----------------------> CSN (Input) -----> CSN (Input)
               |
MOSI (Master   +----------------------> SDI (Input)
Out)           |                                |
               |                                +-----------------> SDO (Output)
               |                                                    |
               |                                                    v
               |                                                  SDI (Input)
               |                                                    |
               |                                                    +-----------------> SDO (Output)
               |                                                                        |
MISO (Master   <------------------------------------------------------------------------+
In)
```

**Key Points:**
- All chips share the same CSN (chip select) - tied together
- All chips share SCK and MOSI (SDI)
- MISO (SDO) is daisy-chained: Chip 1 SDO → Chip 2 SDI → Chip 2 SDO → Chip 3 SDI → Chip 3 SDO → MCU MISO
- CSN is handled automatically by the SPI hardware peripheral or your `SpiTransfer()` implementation

### Software Implementation

**Single SPI Bus, Multiple Drivers:**

```cpp
// Create one SPI communication interface (shared by all chips)
Esp32SPI spi(SPI2_HOST, 
             GPIO_NUM_23, // MOSI
             GPIO_NUM_19, // MISO
             GPIO_NUM_18, // SCLK
             GPIO_NUM_5,  // CS (shared by all chips)
             GPIO_NUM_2,  // EN
             GPIO_NUM_4,  // DIR
             GPIO_NUM_15, // STEP
             4000000);    // 4 MHz SPI clock

// Initialize SPI interface
spi.Initialize();

// Create multiple TMC5160 instances, each with its own daisy-chain position
// Position 0 = first chip, Position 1 = second chip, etc.
tmc5160::TMC5160 driver1(spi, 12'000'000, 0); // First chip (position 0)
tmc5160::TMC5160 driver2(spi, 12'000'000, 1); // Second chip (position 1)
tmc5160::TMC5160 driver3(spi, 12'000'000, 2); // Third chip (position 2)

// Configure each driver
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;
cfg.motor.ihold = 10;

driver1.Initialize(cfg);
driver2.Initialize(cfg);
driver3.Initialize(cfg);

// Each driver automatically uses its own daisy-chain position
// when accessing registers - no manual position setting needed!
driver1.rampControl.SetTargetPosition(1000); // Accesses chip 0
driver2.rampControl.SetTargetPosition(2000); // Accesses chip 1
driver3.rampControl.SetTargetPosition(3000); // Accesses chip 2

// Read from each chip
uint32_t value1, value2, value3;
driver1.GetComm().ReadRegister(0x00, value1); // Reads from chip 0
driver2.GetComm().ReadRegister(0x00, value2); // Reads from chip 1
driver3.GetComm().ReadRegister(0x00, value3); // Reads from chip 2
```

**Setting Daisy-Chain Position at Runtime:**

```cpp
// Create driver with default position (0)
tmc5160::TMC5160 driver(spi);

// Change position at runtime if needed
driver.SetDaisyChainPosition(2); // Now addresses chip at position 2

// Get current position
uint8_t pos = driver.GetDaisyChainPosition(); // Returns 2
```

**How Daisy-Chain Padding Works:**

1. Each `TMC5160` instance stores its own `daisy_chain_position_` (set in constructor or via `SetDaisyChainPosition()`)
   - **Important**: Position is tracked by the `TMC5160` class, NOT by the communication interface
2. When `ReadRegister()` or `WriteRegister()` is called, the driver automatically passes its `daisy_chain_position_` as a parameter to the communication interface methods
3. The `SpiCommInterface` uses the position parameter along with the total chain length (set via `SetDaisyChainLength()`) to calculate the correct transfer size and padding:
   - Position 0: 5 bytes (command only)
   - Position 1: 10 bytes (command + 5 bytes padding)
   - Position 2: 15 bytes (command + 10 bytes padding)
4. The command is placed at bytes 0-4, padding (zeros) from byte 5 onwards
5. The response from chip N is extracted from bytes `(N * 5)` to `(N * 5 + 4)`

**Transfer Size Calculation:**

For a chip at position N:
- **Command**: 5 bytes (40 bits) at offset 0
- **Padding**: N * 5 bytes (N * 40 bits) starting at offset 5
- **Total**: (N + 1) * 5 bytes
- **Response**: Extracted from bytes (N * 5) to (N * 5 + 4)

**Important Note on Response Ordering:**
- Responses come back in **REVERSE order** (last device first, first device last)
- Per datasheet: To read from device k in a chain of n devices, send 40·(n-k+1) dummy bits total
- Our implementation uses (k+1)*5 bytes when we only know k (device position), not n (total chain length)
- The response from device k is correctly extracted from the last 5 bytes of the transfer (offset k*5 bytes)

**Example Transfer Sizes:**

| Position | Total Bytes | Command | Padding | Response Offset |
|----------|-------------|---------|---------|-----------------|
| 0        | 5           | 5       | 0       | 0               |
| 1        | 10          | 5       | 5       | 5               |
| 2        | 15          | 5       | 10      | 10              |
| 3        | 20          | 5       | 15      | 15              |

### Using TMC5160DaisyChain for Multiple Devices

For managing multiple devices efficiently, use the `TMC5160DaisyChain` class:

```cpp
// Create daisy-chain manager with 3 onboard devices
tmc5160::TMC5160DaisyChain<MySPI, 5> chain(spiComm, 3, 12'000'000);

// Initialize all devices
tmc5160::DriverConfig cfg{};
chain.InitializeAll(cfg);

// Create user-friendly aliases for device indices
auto& x_axis = chain[0];  // Position 0 = X-axis motor
auto& y_axis = chain[1];  // Position 1 = Y-axis motor
auto& z_axis = chain[2];  // Position 2 = Z-axis motor

// Access devices using aliases
x_axis.rampControl.SetTargetPosition(1000);
y_axis.rampControl.SetMaxSpeed(500.0f);
z_axis.motorControl.Enable();

// Add extra device at runtime if needed
if (chain.AddDevice(3)) {
  chain[3].Initialize(cfg);
}
```

**Note:** The `TMC5160DaisyChain` class automatically handles proper chain length
configuration and sequential positioning. Individual `TMC5160` instances can also
be created manually, but users must ensure positions match the physical chain order.

### Auto-Detecting Chain Length

The `SpiCommInterface` provides an `AutoDetectChainLength()` method that can automatically
detect the number of devices in the daisy chain by sending a unique command that loops back:

```cpp
// Auto-detect chain length (probes up to 8 devices by default)
uint8_t detected = spi.AutoDetectChainLength();
if (detected > 0) {
  // Chain length automatically set, can also use return value
  ESP_LOGI(TAG, "Detected %u devices in chain", detected);
} else {
  // Detection failed or single chip
  ESP_LOGI(TAG, "Single chip or detection failed");
}

// Or specify max devices to probe
uint8_t detected = spi.AutoDetectChainLength(16); // Probe up to 16 devices
```

**How It Works:**
1. Sends a unique read command (register 0x73) to position `(max_devices+1)` (beyond last device)
2. Sends enough padding to allow command to loop through all devices and back
3. Searches received data for the exact command pattern
4. When found at offset `(n+1)*5` bytes, chain length = `n`
5. Automatically sets the chain length via `SetDaisyChainLength()`

**Important Notes:**
- Requires devices to be powered (but don't need to be initialized)
- Uses a safe read operation that doesn't modify device state
- Requires exact 5-byte pattern match to confirm loopback
- Returns 0 if detection fails (assumes single chip)

## UART Multi-Node Addressing

**Important**: UART multi-node addressing is **NOT** daisy-chaining like SPI. It uses a shared UART bus with node addresses (0-254) programmed via SLAVECONF register. Multiple TMC5160 instances share one `UartCommInterface` on the same UART bus. **Per datasheet procedure, devices are programmed backwards starting from address 254** (254, 253, 252, ...).

### UART Mode Requirements

For UART multi-node addressing, the TMC5160 must be configured in UART mode:

- **SD_MODE** (pin 21): Must be tied **LOW** (0)
- **SPI_MODE** (pin 22): Must be tied **LOW** (0)
- When both are LOW, UART operation is enabled

### Pin Functions in UART Mode

In UART mode (SD_MODE=0, SPI_MODE=0), certain pins take on special functions for sequential addressing:

| Pin Name | Pin Number | Function in UART Mode | Description |
|----------|------------|----------------------|-------------|
| **SDI_CFG1** | 15 | **NAI** (Next Address Input) | Input pin for address selection. When active, slave address increments by one. |
| **SDO_CFG0** | 16 | **NAO** (Next Address Output) | Output pin that connects to the next chip's NAI. Set low after programming to enable next chip. |
| **DIAG0_SWN** | 26 | **SWION** (Single Wire I/O Negative) | Single wire interface I/O (negative) for UART communication |
| **DIAG1_SWP** | 27 | **SWIOP** (Single Wire I/O Positive) | Single wire interface I/O (positive) for UART communication |

**Note**: The UART communication uses the standard UART_TXD and UART_RXD pins, while NAI/NAO pins are used for addressing multiple nodes.

### Hardware Setup

For UART multi-node addressing, connect chips in a daisy chain using NAI/NAO pins:

```
               +-------------+       +-------------+       +-------------+
               | MCU / Host  |       | TMC5160 #1  |       | TMC5160 #2  |  ...
               | Controller  |       | (Address 0) |       | (Address 1) |
               +-------------+       +-------------+       +-------------+
---------------------------------------------------------------------------------
                       (UART Bus - Connected in Parallel)

TXD (Transmit) +----------------------> UART_RXD (Input) -----> UART_RXD (Input)
               |
RXD (Receive)  <---------------------- UART_TXD (Output) <----- UART_TXD (Output)

---------------------------------------------------------------------------------
                       (Node Address Chain - Connected Serially)

               +---> NAI (Input)
MCU GND        |
(or Logic 0)   |    (NAI of the first driver sets the base address, Address 0)
               |
               +--- NAI (GND)
                        |
                        +----------------> NAO (Output)
                                             |
                                             v
                                           NAI (Input)
                                             |
                                             +----------------> NAO (Output)
                                                                  |
                                                                  v
                                                                NAI (Input)
                                                                (This NAI sets Address 2)
```

**Key Points:**
- All chips share the same UART bus (TXD/RXD)
- First chip's NAI is tied to GND (address 0)
- Each chip's NAO connects to the next chip's NAI
- SD_MODE and SPI_MODE must both be LOW for UART mode

### Addressing Schemes

The TMC5160 supports two addressing schemes for UART daisy chaining:

#### Scheme 1: Simple Addressing (1-2 Nodes)

For systems with only one or two TMC5160 devices:

- Set the NAI pins of both devices to different levels
- No sequential programming required
- Suitable for simple two-chip systems

#### Scheme 2: Sequential Addressing (Up to 255 Nodes)

For systems with more than two devices (up to 255 nodes):

- Use the sequential addressing procedure described below
- Each device must be programmed sequentially
- **Per datasheet Figure 5.1, devices are programmed backwards from address 254** (254, 253, 252, ...)
- First device (logical index 0) is programmed to address 254
- Second device (logical index 1) is programmed to address 253
- Continue: device at logical index i is programmed to address (254 - i)

### Sequential Addressing Procedure

To program addresses for multiple chips (up to 255 nodes), follow the datasheet procedure (Figure 5.1):

**Initial State at Power-Up:**
- First chip: NAI tied to GND → responds to address 0
- All other chips: NAI connected to previous chip's NAO (HIGH) → initially respond to address 1

**Programming Sequence (Per Datasheet):**

1. **Addressing Phase 1**: First chip responds to address 0 (NAI = GND)
   - Program first chip (logical index 0) to address **254**
   - Set SLAVECONF.slaveaddr = 254
   - **After programming, the chip's NAO automatically goes LOW** → next chip becomes accessible at address 0

2. **Addressing Phase 2**: Second chip now responds to address 0 (because previous chip's NAO is LOW)
   - Program second chip (logical index 1) to address **253**
   - Set SLAVECONF.slaveaddr = 253
   - After programming, NAO goes LOW → next chip becomes accessible at address 0

3. **Addressing Phase 3+**: Continue for remaining chips
   - Each chip is programmed sequentially, accessible at address 0
   - Device at logical index i is programmed to address **(254 - i)**
   - After programming, NAO goes LOW automatically → next chip becomes accessible at address 0
   - Continue until all chips are programmed

**Critical Steps:**
- Tie the NAI pin of the first TMC5160 to GND
- Interconnect NAO output of each chip to the next chip's NAI pin
- **Program devices backwards from address 254** (254, 253, 252, ...) per datasheet
- After programming each chip, its NAO output automatically goes LOW to enable the next chip
- The next chip becomes accessible at address 0 only after the previous chip's NAO is LOW

**Note**: The logical device indices (0, 1, 2, ...) used in software correspond to physical addresses (254, 253, 252, ...) programmed into the chips. This follows the datasheet procedure for addressing up to 255 nodes.

### Software Implementation

#### Architecture

- **One UartCommInterface**: Shared by all TMC5160 instances on the same UART bus
- **Multiple TMC5160 Instances**: One per device, each with its own programmed node address
- **Node Address Storage**: Each `TMC5160` instance stores its own `uart_node_address_` (like `daisy_chain_position_` for SPI)
- **ReadRegister/WriteRegister**: Accept `node_address` parameter (UART uses this, SPI uses `daisy_chain_position`)
- **Logical vs Physical Addressing**: Devices are accessed via logical indices (0, 1, 2, ...) but programmed with physical addresses (254, 253, 252, ...) per datasheet

#### Configure Node Address (Manual Method)

For manual programming without the manager class:

```cpp
// Create shared UART communication interface
MyUART uart_comm(/* ... */);

// Create TMC5160 instances (initial address 0, will be programmed)
tmc5160::TMC5160 driver0(uart_comm, 12'000'000, 0, 0); // Logical index 0
tmc5160::TMC5160 driver1(uart_comm, 12'000'000, 0, 0); // Logical index 1
tmc5160::TMC5160 driver2(uart_comm, 12'000'000, 0, 0); // Logical index 2

// Program devices sequentially per datasheet (backwards from 254)
// First chip: accessible at address 0, program to address 254
driver0.uartConfig.ConfigureSlave(254, 2); // Address 254, delay 2 bit times
driver0.SetUartNodeAddress(254);
// After programming, NAO goes LOW → next chip accessible at address 0

// Second chip: accessible at address 0, program to address 253
driver1.uartConfig.ConfigureSlave(253, 2); // Address 253, delay 2 bit times
driver1.SetUartNodeAddress(253);
// After programming, NAO goes LOW → next chip accessible at address 0

// Third chip: accessible at address 0, program to address 252
driver2.uartConfig.ConfigureSlave(252, 2); // Address 252, delay 2 bit times
driver2.SetUartNodeAddress(252);

// Get current node address
uint8_t addr = driver0.GetUartNodeAddress(); // Returns 254
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

#### Using TMC5160MultiNode Manager Class (Recommended)

The easiest way to manage multiple UART nodes is using the `TMC5160MultiNode` class:

```cpp
#include "tmc5160_multi_node.hpp"

// Create shared UART communication interface
MyUART uart_comm(/* ... constructor args ... */);

// Create multi-node manager with 3 onboard devices, capacity for 5 total
// Onboard devices are created with initial address 0 (will be programmed)
tmc5160::TMC5160MultiNode<MyUART, 5> nodes(uart_comm, 3, 12'000'000);

// Program all devices sequentially (required at startup)
// This programs devices to addresses 254, 253, 252 per datasheet procedure
// Device at logical index 0 → address 254
// Device at logical index 1 → address 253
// Device at logical index 2 → address 252
if (!nodes.ProgramSequentially()) {
  // Error handling
  return;
}

// Initialize all devices
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;
cfg.motor.ihold = 10;
nodes.InitializeAll(cfg);

// Create user-friendly aliases using logical indices
auto& x_axis = nodes[0];  // Logical index 0, programmed to address 254
auto& y_axis = nodes[1];  // Logical index 1, programmed to address 253
auto& z_axis = nodes[2];  // Logical index 2, programmed to address 252

// Use devices (access via logical indices, communication uses programmed addresses)
x_axis.rampControl.SetTargetPosition(1000);
y_axis.rampControl.SetMaxSpeed(500.0f);
z_axis.motorControl.Enable();

// Check actual programmed addresses
uint8_t x_addr = x_axis.GetUartNodeAddress(); // Returns 254
uint8_t y_addr = y_axis.GetUartNodeAddress(); // Returns 253
uint8_t z_addr = z_axis.GetUartNodeAddress(); // Returns 252
```

#### Manual Sequential Programming Example

For manual control without the manager class:

```cpp
#include "tmc5160.hpp"

void programAllChipsSequentially(MyUART& uart, uint8_t num_chips) {
  // Create TMC5160 instances (initial address 0, will be programmed)
  // All instances share the same UartCommInterface
  std::vector<tmc5160::TMC5160<MyUART>> drivers;
  for (uint8_t i = 0; i < num_chips; i++) {
    drivers.emplace_back(uart, 12'000'000, 0, 0); // f_clk, daisy_pos=0, initial uart_addr=0
  }
  
  // Program each device sequentially per datasheet (backwards from 254)
  // First chip: NAI is tied to GND (hardware) → responds to address 0
  for (uint8_t i = 0; i < num_chips; i++) {
    // Calculate target address: 254 - i (per datasheet)
    uint8_t target_address = 254 - i;
    
    // Program device to target address
    // send_delay should be >= 2 for multi-node systems
    if (!drivers[i].uartConfig.ConfigureSlave(target_address, 2)) {
      // Error: failed to program chip
      break;
    }
    
    // Update the driver's node address
    drivers[i].SetUartNodeAddress(target_address);
    
    // After programming, chip's NAO goes LOW automatically to enable next chip
    // Next chip becomes accessible at address 0
    // Small delay to ensure NAO settles
    uart.DelayMs(10);
    
    // Verify address
    uint8_t verify_addr = drivers[i].GetUartNodeAddress();
    if (verify_addr != target_address) {
      // Error: address verification failed
      break;
    }
  }
}
```

**Important Notes:**
- The first chip's NAI must be tied to GND in hardware
- Each chip's NAO connects to the next chip's NAI in hardware
- After programming each chip, its NAO automatically goes LOW to enable the next chip
- Set `send_delay` to at least 2 for multi-node systems (via `ConfigureSlaveAddress()`)
- Program chips sequentially - you cannot skip ahead
- Verify each address after programming

## Communication Interface Methods

### SPI Methods

| Method | Description |
|--------|-------------|
| `ReadRegister(address, value, daisy_chain_position = 0)` | Read register with optional daisy-chain position parameter |
| `WriteRegister(address, value, daisy_chain_position = 0)` | Write register with optional daisy-chain position parameter |
| `SetDaisyChainLength(total_length)` | Set total number of devices in chain (for proper response extraction) |
| `GetDaisyChainLength()` | Get current chain length setting |
| `AutoDetectChainLength(max_devices = 8)` | Auto-detect chain length by sending command that loops back |

**Note**: The `daisy_chain_position` parameter in `ReadRegister()` and `WriteRegister()` is provided by the `TMC5160` class automatically. The `SpiCommInterface` does NOT track individual device positions - only the total chain length.

### UART Methods

| Method | Description |
|--------|-------------|
| `ReadRegister(address, value, node_address = 0)` | Read register with node address parameter |
| `WriteRegister(address, value, node_address = 0)` | Write register with node address parameter |
| `SetNaiPin(active)` | Set NAI pin state for sequential addressing |
| `GetNaoPin(active)` | Read NAO pin state |

**Note**: The `node_address` parameter in `ReadRegister()` and `WriteRegister()` is provided by the `TMC5160` class automatically via `GetCommAddress()`. The `UartCommInterface` does NOT store node addresses - multiple `TMC5160` instances share one `UartCommInterface` on the same UART bus.

### TMC5160 Class Methods (UART)

| Method | Description |
|--------|-------------|
| `SetUartNodeAddress(address)` | Set UART node address (0-127) for this instance |
| `GetUartNodeAddress()` | Get current UART node address |
| `uartConfig.ConfigureSlave(address, send_delay)` | Configure SLAVECONF register with node address |

### TMC5160MultiNode Manager Class

| Method | Description |
|--------|-------------|
| `ProgramSequentially(send_delay = 2)` | Program all active devices sequentially using NAI/NAO. Programs devices to addresses (254, 253, 252, ...) per datasheet. |
| `ProgramDevice(index, send_delay = 2)` | Program a single device at logical index to address (254 - index) |
| `AddDevice(index)` | Add an extra device at specified logical index |
| `RemoveDevice(index)` | Remove an extra device at specified logical index |
| `operator[](index)` | Access individual TMC5160 driver by logical index (0, 1, 2, ...). Actual programmed addresses are (254, 253, 252, ...). |
| `InitializeAll(config)` | Initialize all active devices with same configuration |

## Best Practices

1. **SPI Daisy-Chain Setup**:
   - All chips share the same CSN (tied together) - handled automatically by SPI hardware
   - Ensure CSN timing requirements are met (minimum 2*tclk + 10ns)
   - CSN control is handled in your `SpiTransfer()` implementation
   - Use `TMC5160DaisyChain` class for managing multiple devices efficiently
   - Set chain length using `SetDaisyChainLength()` for proper response extraction
   - Optionally use `AutoDetectChainLength()` to automatically detect chain length

2. **UART Multi-Node**:
   - **Mode Configuration**: Ensure SD_MODE=0 and SPI_MODE=0 for UART operation
   - **Hardware Setup**: First chip's NAI must be tied to GND; chain NAO→NAI for subsequent chips
   - **Shared Interface**: Multiple `TMC5160` instances share one `UartCommInterface` on the same UART bus
   - **Node Address Storage**: Each `TMC5160` instance stores its own `uart_node_address_` (like `daisy_chain_position_` for SPI)
   - **Send Delay**: Set `send_delay` >= 2 when multiple nodes are present (via `uartConfig.ConfigureSlave()`)
   - **Sequential Programming**: Program addresses backwards from 254 (254, 253, 252, ...) per datasheet procedure (use `TMC5160MultiNode::ProgramSequentially()`)
   - **Logical vs Physical**: Devices accessed via logical indices (0, 1, 2, ...) but programmed with physical addresses (254, 253, 252, ...)
   - **NAO Control**: After programming each chip, its NAO automatically goes LOW to enable the next chip
   - **Verification**: Always verify addresses after programming
   - **Pin Mapping**: Remember that in UART mode, SDI_CFG1→NAI, SDO_CFG0→NAO, DIAG0_SWN→SWION, DIAG1_SWP→SWIOP

3. **Error Handling**:
   - Always check return values from communication methods
   - Implement timeout handling for multi-chip operations
   - Verify chip selection before register access

## Limitations

- **SPI Daisy-Chain**: 
  - All chips share the same CSN (tied together)
  - Transfer size for chip at position N is `(N + 1) * 5` bytes
  
- **UART Multi-Node**:
  - Maximum 255 nodes (8-bit address space, 0-254)
  - Requires SD_MODE=0 and SPI_MODE=0 for UART operation
  - Sequential programming required - cannot program chips out of order
  - Devices programmed backwards from address 254 (254, 253, 252, ...) per datasheet
  - Logical indices (0, 1, 2, ...) map to physical addresses (254, 253, 252, ...)
  - NAI/NAO pins must be properly connected (first chip NAI to GND, chain NAO→NAI)
  - Send delay must be >= 2 for multi-node systems
  
- **Timing**: 
  - Ensure proper timing for CSN and SPI transactions
  - UART baud rate: minimum 9000 baud, maximum fCLK/16
  
- **Hardware**: 
  - SPI: MISO (SDO) must be daisy-chained
  - UART: NAI/NAO pins required for daisy chaining (SDI_CFG1→NAI, SDO_CFG0→NAO)

---

**Navigation**
⬅️ [Previous: Advanced Configuration](special_features_advanced_configuration.md) | [Next: Troubleshooting ➡️](troubleshooting.md) | [Docs Hub 📚](index.md)

