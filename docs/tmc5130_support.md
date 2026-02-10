---
layout: default
title: "🔧 TMC5130 Support"
description: "Automatic detection and support for TMC5130 and TMC5160 chips"
nav_order: 12
parent: "📚 Documentation"
permalink: /docs/tmc5130_support/
---

# TMC5130 Support

The driver automatically detects and supports both TMC5130 and TMC5160 chips.

## Automatic Detection

The driver detects the chip version during `Initialize()`:
- **TMC5130**: Version 0x11
- **TMC5160**: Version 0x30

The chip version is read from the IOIN register and stored internally. You can check which chip was detected using:

```cpp
uint8_t version = driver.status.GetChipVersion();
if (version == ChipVersion::TMC5130) {
    // TMC5130-specific logic if needed
} else if (version == ChipVersion::TMC5160) {
    // TMC5160-specific logic if needed
}
```

## Differences Handled Automatically

The driver automatically handles differences between TMC5130 and TMC5160:

1. **GLOBAL_SCALER**: TMC5130 doesn't have this register - driver skips it automatically
2. **Current Limits**: TMC5130 has internal MOSFETs (1.64A max), TMC5160 uses external MOSFETs
3. **Version Detection**: Use `driver.status.GetChipVersion()` to check detected version

## Usage

No special configuration needed - the driver handles differences automatically:

```cpp
// Initialize driver (automatically detects chip version)
tmc51x0::DriverConfig cfg;
// ... configure cfg ...

auto init_result = driver.Initialize(cfg);
if (!init_result) {
    printf("Initialization error: %s\n", init_result.ErrorMessage());
    // Handle error
}

// Check which chip was detected
uint8_t version = driver.status.GetChipVersion();
if (version == ChipVersion::TMC5130) {
    printf("Detected TMC5130\n");
} else if (version == ChipVersion::TMC5160) {
    printf("Detected TMC5160\n");
}
```

## Technical Details

### GLOBAL_SCALER Register

The TMC5130 doesn't have the GLOBAL_SCALER register that exists in TMC5160. The driver automatically:

- Skips writing to GLOBAL_SCALER during `SetMotorCurrent()` for TMC5130
- Uses calculated IRUN values directly (without global scaler adjustment)
- Logs a debug message when GLOBAL_SCALER is skipped

### Current Limits

- **TMC5130**: Internal MOSFETs with 1.64A maximum current
- **TMC5160**: External MOSFETs (current limit depends on external components)

The driver doesn't enforce these limits automatically - ensure your motor current settings are appropriate for your chip.

## Compatibility

The driver is fully compatible with both chips. All features work the same way, with the exception of GLOBAL_SCALER which is automatically skipped for TMC5130.

---

**Navigation**
[← Register Access](special_features_register_access.md) | [Back to Index](index.md) | [Next: Troubleshooting →](troubleshooting.md)

