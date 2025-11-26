---
layout: default
title: "🐛 Troubleshooting"
description: "Common issues and solutions for the TMC5160 driver"
nav_order: 8
parent: "📚 Documentation"
permalink: /docs/troubleshooting/
---

# Troubleshooting

This guide helps you diagnose and resolve common issues when using the TMC5160 driver.

## Common Error Messages

### Error: "Motor doesn't move"

**Symptoms:**
- Driver initializes successfully
- No motor movement
- No error status reported

**Causes:**
- Motor not enabled
- Enable pin not configured correctly
- Motor current too low
- Chopper disabled (toff = 0)

**Solutions:**
1. Verify `driver.motorControl.Enable()` is called
2. Check enable pin wiring and active level configuration
3. Increase motor current: Set `cfg.motor_spec.rated_current_ma` to appropriate value (e.g., 2000 for 2A). IRUN is automatically calculated.
4. Verify chopper is enabled: `cfg.chopper.toff > 0`

### Error: "Communication timeout"

**Symptoms:**
- Register reads/writes fail
- Driver initialization fails

**Causes:**
- Incorrect SPI/UART wiring
- Wrong SPI mode or baud rate
- CS pin not configured correctly
- Clock frequency too high

**Solutions:**
1. Verify SPI mode is 3 (CPOL=1, CPHA=1)
2. Check CS pin is pulled high when not in use
3. Reduce SPI clock frequency to 4 MHz or lower
4. Verify UART baud rate matches (typically 500000 bps)
5. Check signal integrity with oscilloscope

## Hardware Issues

### Device Not Detected

**Symptoms:**
- Initialization fails
- No response from device
- Register reads return 0xFFFFFFFF or 0x00000000

**Checklist:**
- [ ] Verify power supply voltage is correct (3.3V for VDD, 8-60V for motor)
- [ ] Check all connections are secure
- [ ] Verify SPI CS line is correct and active low
- [ ] Check UART node address matches configuration
- [ ] Use oscilloscope/logic analyzer to verify bus activity
- [ ] Verify motor supply is connected (driver needs motor supply even if motor not connected)

### Communication Errors

**Symptoms:**
- Timeout errors
- CRC/checksum errors (UART)
- Incorrect register values

**Solutions:**
- Check bus speed is within IC specifications (SPI: max 4 MHz, UART: typically 500kbps)
- Verify signal integrity (noise, reflections, crosstalk)
- Ensure proper bus termination
- Check for loose connections
- Verify CS timing (minimum CS high time: 2*tclk + 10ns)

### Motor Issues

**Symptoms:**
- Motor doesn't move
- Motor moves erratically
- Motor makes noise but doesn't move
- Motor loses steps

**Solutions:**
- **Doesn't move**: Check enable pin, verify motor current > 0, check chopper enabled
- **Erratic movement**: Check motor wiring, verify current settings, check for short circuits
- **Noise but no movement**: Verify motor supply voltage, check motor connections, verify current settings
- **Loses steps**: Increase motor current, reduce acceleration, check mechanical load

## Software Issues

### Compilation Errors

**Error: "No matching function"**

**Solution:**
- Ensure you've implemented all required communication interface methods
- Check method signatures match the interface definition exactly
- Verify template parameter is correct: `TMC5160<MySPI>`

**Error: "Undefined reference"**

**Solution:**
- Driver is header-only, no linking needed
- Check include paths are correct
- Verify C++17 standard is enabled: `-std=c++17`

**Error: "Incomplete type"**

**Solution:**
- Include all necessary headers: `tmc5160.hpp` includes everything needed
- Check for circular dependencies

### Runtime Errors

**Initialization Fails**

**Checklist:**
- [ ] Communication interface is properly initialized
- [ ] Hardware connections are correct
- [ ] Configuration parameters are valid (currents 0-31, etc.)
- [ ] Device is powered and ready
- [ ] SPI/UART is configured correctly

**Unexpected Behavior**

**Checklist:**
- [ ] Verify configuration matches your use case
- [ ] Check for timing issues (delays between operations)
- [ ] Review error handling code
- [ ] Verify register values match expected values
- [ ] Check driver status: `driver.diagnostics.GetStatus()`

## Debugging Tips

### Enable Debug Output

```cpp
// Debug logging is enabled by default
// To disable (reduce code size):
#define TMC5160_DISABLE_DEBUG_LOGGING
#include "inc/tmc5160.hpp"
```

### Use a Logic Analyzer

For bus communication issues, a logic analyzer can help:
- Verify correct protocol timing
- Check for signal integrity issues
- Validate data being sent/received
- Verify CS timing

### Check Register Values

Read back registers to verify configuration:

```cpp
uint32_t gconf = 0;
driver.GetComm().ReadRegister(Registers::GCONF, gconf);
// Verify expected values
```

### Monitor Driver Status

```cpp
DriverStatus status = driver.diagnostics.GetStatus();
if (status != DriverStatus::OK) {
    // Handle error condition
}
```

## FAQ

### Q: What is the recommended motor current setting?

**A:** Set `motor_spec.rated_current_ma` to your motor's rated current (e.g., 2000 for 2A). The driver automatically calculates IRUN and IHOLD. IRUN will be between 16-31 for optimal performance, and IHOLD will be set to approximately 30% of run current (or as specified in `motor_spec.hold_current_ma`).

### Q: How do I choose between stealthChop and spreadCycle?

**A:** Use stealthChop for silent operation at low speeds. Use spreadCycle for high torque at higher speeds. Set mode
change speeds to switch automatically.

### Q: What microstep resolution should I use?

**A:** Common values:
- `mres = MRES_16`: 16 microsteps (good balance)
- `mres = MRES_32`: 32 microsteps (smoother, lower torque)
- `mres = MRES_256` : 256 microsteps (smoothest motion, lowest torque)
- `mres = FULLSTEP`: Full step (maximum torque)

### Q: How do I tune StallGuard2?

**A:** Start with `sgt = 0` and adjust based on your motor:
- Lower values = more sensitive (detects stall earlier)
- Higher values = less sensitive
- Monitor SG value during normal operation and set threshold below that value

### Q: Why is my motor noisy?

**A:** Possible causes:
- Using spreadCycle mode (normal chopper noise)
- Low microstep resolution
- Incorrect chopper settings
- Try stealthChop mode for silent operation

## Getting More Help

If you're still experiencing issues:

1. Check the [API Reference](api_reference.md) for method details
2. Review [Examples](examples.md) for working code
3. Search existing issues on GitHub
4. Open a new issue with:
   - Description of the problem
   - Steps to reproduce
   - Hardware setup details
   - Error messages/logs
   - Register values (if applicable)

---

**Navigation**
⬅️ [Examples](examples.md) | [Back to Index](index.md)

