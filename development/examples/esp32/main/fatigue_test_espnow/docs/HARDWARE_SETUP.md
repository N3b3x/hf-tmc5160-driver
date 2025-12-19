# Hardware Setup Guide - Fatigue Test ESP-NOW Unit

## Overview

This guide covers the hardware setup for the Fatigue Test ESP-NOW Unit, including motor connections, encoder setup, and ESP32 configuration.

## Hardware Requirements

### Required Components

1. **ESP32 Development Board** (ESP32-C6 recommended)
2. **TMC5160 Stepper Motor Driver** (SPI interface)
3. **Stepper Motor** (compatible with TMC5160)
4. **ABN Encoder** (optional, for encoder-based bounds detection)
5. **Reference Switches** (optional, for homing)
6. **Power Supply** (appropriate for motor and ESP32)

## Pin Configuration

### SPI Interface (TMC5160)

The TMC5160 communicates via SPI. Default pin configuration (from test config):

- **MOSI**: GPIO 23
- **MISO**: GPIO 19
- **SCK**: GPIO 18
- **CS**: GPIO 5

**Note**: Pin configuration is managed by `tmc51x0_test_config::GetDefaultPinConfig()`. Modify test config if needed.

### UART Interface (Debug/Control)

- **TX**: GPIO 1 (UART0)
- **RX**: GPIO 3 (UART0)
- **Baud Rate**: 115200

### ESP-NOW

- **WiFi Channel**: 1 (configured in code)
- **MAC Address**: Auto-detected, can be read from serial output

## Motor Configuration

### Test Rig Selection

The system uses a test rig configuration system. Edit `main.cpp`:

```cpp
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
```

### Motor Parameters

Motor parameters are configured via `tmc51x0_test_config::ConfigureDriverFromTestRig()`. Key parameters:

- **Sense Resistor**: Typically 50mΩ
- **Supply Voltage**: Motor supply voltage (e.g., 24V)
- **Rated Current**: Motor rated current
- **Microsteps**: Typically 256 microsteps

### Motor Wiring

1. **Power Supply**:
   - Connect motor power supply to TMC5160 VM/GND
   - Ensure adequate current capacity
   - Use appropriate voltage for your motor

2. **Motor Phases**:
   - Connect motor phases to TMC5160 motor outputs
   - Verify phase wiring (swap if motor runs backwards)

3. **TMC5160 Configuration**:
   - SPI interface to ESP32
   - Enable pin (if used)
   - DIAG pin (for diagnostics)

## Encoder Setup (Optional)

### ABN Encoder Wiring

If using encoder-based bounds detection:

- **A Phase**: Connect to encoder A input
- **B Phase**: Connect to encoder B input
- **Index**: Optional, for homing

**Note**: Encoder configuration is managed by test rig config:
- `GetTestRigEncoderConfig<SELECTED_TEST_RIG>()`
- `GetTestRigEncoderPulsesPerRev<SELECTED_TEST_RIG>()`
- `GetTestRigEncoderInvertDirection<SELECTED_TEST_RIG>()`

### Encoder Requirements

- **Resolution**: Sufficient for position monitoring
- **Interface**: Quadrature encoder (A/B phases)
- **Mounting**: Properly aligned with motor shaft

## Reference Switches (Optional)

For homing and position reference:

- **Min Switch**: Physical limit switch at minimum position
- **Max Switch**: Physical limit switch at maximum position

**Note**: Current implementation uses bounds finding (StallGuard or encoder) rather than limit switches.

## Power Supply

### Requirements

1. **Motor Power**:
   - Voltage: As specified for your motor (typically 12V-48V)
   - Current: Sufficient for motor operation (check motor specs)
   - Regulation: Stable voltage under load

2. **ESP32 Power**:
   - Voltage: 3.3V or 5V (depending on board)
   - Current: ~200-500mA typical
   - Can share supply with TMC5160 logic supply

### Power Sequencing

1. Power on motor supply first
2. Power on ESP32
3. Allow initialization time (~1-2 seconds)

## Mechanical Setup

### Motor Mounting

- Secure motor to test fixture
- Ensure proper alignment
- Check for mechanical binding

### Load Configuration

- Attach test load to motor shaft
- Verify load is within motor specifications
- Check for excessive friction or binding

### Bounds Detection

- **StallGuard Method**: No additional hardware needed
- **Encoder Method**: Requires encoder properly mounted

## Initial Testing

### 1. Serial Connection

Connect to ESP32 via USB/UART:

```bash
# Find serial port
ls /dev/ttyUSB*  # Linux
ls /dev/tty.*    # macOS

# Connect with serial monitor
screen /dev/ttyUSB0 115200
```

### 2. Verify Initialization

Look for startup messages:
```
╔══════════════════════════════════════════════════════════════════════════════╗
║         Fatigue Test Unit with ESP-NOW Communication                         ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

### 3. Test UART Commands

Try basic commands:
```
-s          # Show status
-h          # Show help
```

### 4. Test ESP-NOW Communication

1. Power on test unit
2. Note MAC address from serial output
3. Configure MAC in remote controller
4. Power on remote controller
5. Verify communication

## Troubleshooting

### Motor Not Moving

1. **Check Power Supply**:
   - Verify motor power is connected
   - Check voltage levels
   - Verify current capacity

2. **Check SPI Communication**:
   - Verify SPI wiring
   - Check CS pin connection
   - Verify SPI clock speed

3. **Check Motor Wiring**:
   - Verify phase connections
   - Check for shorts
   - Try swapping phases

4. **Check TMC5160 Configuration**:
   - Verify driver is enabled
   - Check current settings
   - Verify microstep configuration

### ESP-NOW Not Working

1. **Check MAC Address**:
   - Verify MAC is correct in remote controller
   - Check both devices are on same WiFi channel

2. **Check Range**:
   - Ensure devices are within range (~100-200m)
   - Check for interference

3. **Check Serial Output**:
   - Look for ESP-NOW initialization messages
   - Check for error messages

### Bounds Finding Fails

1. **StallGuard Method**:
   - Check SGT threshold setting
   - Verify motor can actually stall
   - Check for false stall detection

2. **Encoder Method**:
   - Verify encoder wiring
   - Check encoder pulses per revolution setting
   - Verify encoder is properly mounted

### Serial Communication Issues

1. **Check Baud Rate**: Must be 115200
2. **Check Wiring**: Verify TX/RX connections
3. **Check USB Driver**: Ensure drivers are installed

## Safety Considerations

1. **Motor Power**: Use appropriate safety measures for high-voltage motor supplies
2. **Mechanical Safety**: Ensure proper mechanical stops to prevent over-travel
3. **Emergency Stop**: Consider adding emergency stop functionality
4. **Current Limits**: Set appropriate current limits in TMC5160 configuration

## Calibration

### StallGuard Calibration

If using StallGuard method:

1. Run stallguard tuning tool (separate example)
2. Determine optimal SGT threshold
3. Configure in test rig settings

### Encoder Calibration

If using encoder method:

1. Verify encoder pulses per revolution
2. Check encoder direction (invert if needed)
3. Verify encoder mounting alignment

## Maintenance

1. **Regular Checks**:
   - Motor connections
   - Encoder alignment (if used)
   - Power supply stability

2. **Software Updates**:
   - Keep firmware updated
   - Check for configuration changes

3. **Mechanical Maintenance**:
   - Lubricate moving parts
   - Check for wear
   - Verify alignment

