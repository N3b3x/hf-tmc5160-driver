# UART Multi-Node Comprehensive Test

## Overview

The `uart_multi_node_comprehensive_test.cpp` provides comprehensive testing for TMC5160 UART multi-node features including node addressing, slave address configuration, send delay configuration, and multi-node coordination.

## Purpose

This test suite is ideal for:
- Validating UART multi-node configuration
- Testing node addressing
- Verifying slave address assignment
- Testing send delay configuration
- Validating multi-node coordination

## ⚠️ Important Warning

**MULTI-MOTOR HARDWARE REQUIRED**

This test suite requires **multiple TMC5160 drivers** connected in a UART network. **DO NOT run these tests on a single-motor setup.**

## Test Categories

### 1. Node Addressing Tests

Tests UART node addressing:
- Node address configuration
- NAI/NAO pin management
- Address assignment
- Address verification

### 2. Slave Address Tests

Tests slave address configuration:
- Slave address setting
- Address reading
- Address verification

### 3. Send Delay Tests

Tests send delay configuration:
- Send delay setting
- Delay timing
- Delay verification

### 4. Multi-Node Coordination Tests

Tests multi-node coordination:
- Simultaneous node control
- Coordinated operations
- Multi-node status reading

## Hardware Requirements

- ESP32 development board
- **2+ TMC5160 stepper motor drivers** (connected via UART)
- Stepper motors connected to each TMC5160
- UART connection: All chips share TXD/RXD
- NAI/NAO pins for addressing: First chip NAI to GND, chain NAO→NAI
- Mode pins: SD_MODE=0 (GND), SPI_MODE=0 (GND) for UART mode

## UART Multi-Node Wiring

### UART Connection

All chips share UART signals:
- **TXD**: Shared transmit (MCU TX → all chips RX)
- **RXD**: Shared receive (MCU RX ← all chips TX)
- **TXEN**: Optional, for RS485 transceiver

### NAI/NAO Addressing Chain

NAI/NAO pins form a daisy chain for addressing:
- **Chip 1**: NAI → GND (address 0)
- **Chip 1 NAO** → **Chip 2 NAI** (address 1)
- **Chip 2 NAO** → **Chip 3 NAI** (address 2)
- And so on...

### Example Wiring (2 Nodes)

```
MCU          Chip 1          Chip 2
TXD ──────── RXD ──────────── RXD
RXD ──────── TXD ──────────── TXD
GND ──────── NAI
             NAO ──────────── NAI
```

### Example Wiring (3 Nodes)

```
MCU          Chip 1          Chip 2          Chip 3
TXD ──────── RXD ──────────── RXD ──────────── RXD
RXD ──────── TXD ──────────── TXD ──────────── TXD
GND ──────── NAI
             NAO ──────────── NAI
                              NAO ──────────── NAI
```

## Pin Configuration

Default pin configuration:

- **UART**: TX=17, RX=16, TXEN=4 (optional, for RS485)
- **NAI/NAO**: GPIO pins for addressing (configured via code)
- **Control**: EN, DIR, STEP (can be shared or separate)
- **Mode Pins**: SD_MODE=0 (GND), SPI_MODE=0 (GND) for UART mode

## Test Configuration

Tests can be enabled/disabled:

```cpp
static constexpr bool ENABLE_NODE_ADDRESSING_TESTS = true;
static constexpr bool ENABLE_SLAVE_ADDRESS_TESTS = true;
static constexpr bool ENABLE_SEND_DELAY_TESTS = true;
static constexpr bool ENABLE_MULTI_NODE_COORDINATION_TESTS = true;
```

### Node Count Configuration

Set the number of nodes:

```cpp
static constexpr uint8_t TEST_NODE_COUNT = 2; // Number of nodes
```

## UART Multi-Node Operation

### Node Addressing

Each node in the UART network has:
- **Hardware Address**: Determined by NAI/NAO chain position (0, 1, 2, ...)
- **Slave Address**: Software-configurable address (default matches hardware address)

### Communication Protocol

UART multi-node uses:
- **Broadcast**: Address 0xFF (all nodes respond)
- **Specific Address**: Address 0-254 (only matching node responds)
- **Send Delay**: Delay between transmissions for proper addressing

### Sequential Programming

For initial configuration, use sequential programming:
- Configure nodes one at a time
- Use NAI/NAO pins to select node
- Set slave address and other parameters

## Detailed Test Descriptions

### Node Addressing Tests

#### Test: Node Address Configuration
- **Purpose**: Verify node addresses can be configured
- **Steps**:
  1. Configure NAI/NAO pins
  2. Set node addresses
  3. Verify addresses assigned correctly
- **Expected**: Node addresses configured correctly

#### Test: Address Verification
- **Purpose**: Verify addresses are correct
- **Steps**:
  1. Read address from each node
  2. Verify addresses match expected
  3. Test addressing works
- **Expected**: Addresses verified correctly

### Slave Address Tests

#### Test: Slave Address Configuration
- **Purpose**: Verify slave addresses can be set
- **Steps**:
  1. Set slave address for each node
  2. Read back addresses
  3. Verify addresses match
- **Expected**: Slave addresses set correctly

### Send Delay Tests

#### Test: Send Delay Configuration
- **Purpose**: Verify send delay can be configured
- **Steps**:
  1. Set send delay
  2. Verify delay configured
  3. Test delay timing
- **Expected**: Send delay configured correctly

### Multi-Node Coordination Tests

#### Test: Simultaneous Node Control
- **Purpose**: Verify multiple nodes can be controlled
- **Steps**:
  1. Send commands to multiple nodes
  2. Verify all nodes respond
  3. Check node independence
- **Expected**: All nodes controlled independently

## Code Structure

### Creating UART Multi-Node Interface

```cpp
// Create UART interface
Esp32UART uart(UART_NUM_1, tx_pin, rx_pin, baud_rate);

// Create multi-node helper
TMC5160MultiNode<Esp32UART> multi_node(uart, node_count);

// Access nodes by address
multi_node.GetNode(0)->rampControl.SetTargetPosition(1000);
multi_node.GetNode(1)->rampControl.SetTargetPosition(2000);
```

### Sequential Programming

For initial setup:

```cpp
// Configure node 0
SetNAIPin(0, LOW);  // Select node 0
SetNAIPin(1, HIGH);
node0->SetSlaveAddress(0);

// Configure node 1
SetNAIPin(0, HIGH);
SetNAIPin(1, LOW);  // Select node 1
node1->SetSlaveAddress(1);
```

## Expected Behavior

### Test Execution

1. **Initialization**: UART interface and multi-node setup
2. **Addressing Tests**: Node addressing configuration
3. **Slave Address Tests**: Slave address configuration
4. **Send Delay Tests**: Send delay configuration
5. **Coordination Tests**: Multi-node coordination
6. **Summary**: Test results displayed

### Typical Output

```
I (1234) UART_MultiNode_Test: ╔══════════════════════════════════════════════════════════════════════════════╗
I (1235) UART_MultiNode_Test: ║        UART Multi-Node Comprehensive Test Suite                              ║
I (1236) UART_MultiNode_Test: ╚══════════════════════════════════════════════════════════════════════════════╝
I (1237) UART_MultiNode_Test: ⚠️ MULTI-MOTOR HARDWARE REQUIRED
I (1238) UART_MultiNode_Test: [PASS] Node Addressing: Node addresses configured
I (1239) UART_MultiNode_Test: [PASS] Slave Address: Slave addresses set correctly
...
```

## Troubleshooting

### Nodes Not Responding

**Symptoms**: Can't communicate with nodes

**Solutions**:
1. Verify UART wiring (TXD/RXD shared correctly)
2. Check baud rate matches all nodes
3. Verify mode pins (SD_MODE=0, SPI_MODE=0)
4. Check NAI/NAO addressing chain
5. Verify slave addresses are set correctly

### Wrong Node Responding

**Symptoms**: Commands go to wrong node

**Solutions**:
1. Verify NAI/NAO addressing chain
2. Check slave addresses match expected
3. Verify send delay is configured
4. Check for addressing conflicts

### Addressing Not Working

**Symptoms**: Can't address individual nodes

**Solutions**:
1. Verify NAI/NAO pin configuration
2. Check addressing chain wiring
3. Verify slave addresses are unique
4. Check send delay timing
5. Test with broadcast address first

## Related Documentation

- [SPI Daisy Chain Test](spi_daisy_chain_comprehensive_test.md) - SPI multi-motor setup
- [Special Features: Multi-Chip](../../../docs/special_features_multi_chip.md) - Detailed multi-node guide
- [Special Features: Communication Interface](../../../docs/special_features_communication_interface.md) - UART protocol details

