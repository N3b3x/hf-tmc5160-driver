---
layout: default
title: "📚 Documentation"
description: "Complete documentation for the HardFOC TMC51x0 Driver (TMC5130 & TMC5160)"
nav_order: 2
parent: "HardFOC TMC51x0 Driver (TMC5130 & TMC5160)"
permalink: /docs/
has_children: true
---

# HF-TMC51x0 Documentation (TMC5130 & TMC5160)

Welcome! This directory contains step-by-step guides for installing, building, and using the **HF-TMC51x0** library, which supports both TMC5130 and TMC5160 chips.

## 📚 Documentation Structure

### **Getting Started**

1. **[🛠️ Installation](installation.md)** – Prerequisites and how to obtain the source
2. **[⚡ Quick Start](quickstart.md)** – Minimal working example to get you running
3. **[🔌 Hardware Setup](hardware_setup.md)** – Wiring diagrams and pin connections

### **Integration**

4. **[🔧 Platform Integration](platform_integration.md)** – Implement the CRTP communication interface for your platform
5. **[⚙️ Configuration](configuration.md)** – Configuration options and settings

### **Reference**

6. **[📖 API Reference](api_reference.md)** – Complete API documentation
7. **[💡 Examples](examples.md)** – Detailed example walkthroughs

### **Advanced Features**

8. **[📐 Unit Conversions](special_features_unit_conversions.md)** – Converting between physical units and driver steps
9. **[⚙️ Motor Setup from Specifications](special_features_motor_setup.md)** – High-level motor configuration from physical parameters
10. **[🏠 Sensorless Homing](special_features_sensorless_homing.md)** – Homing without endstops using StallGuard2
11. **[🎯 Automatic Tuning](special_features_advanced_configuration.md#automatic-tuning-with-comprehensive-velocity-range-analysis)** – Intelligent StallGuard2 threshold optimization
12. **[🔧 Advanced Configuration](special_features_advanced_configuration.md)** – CoolStep, dcStep, freewheeling, and more
13. **[🔗 Multi-Chip Communication](special_features_multi_chip.md)** – SPI daisy chaining and UART multi-node addressing
14. **[📡 Communication Interface](special_features_communication_interface.md)** – Low-level SPI/UART communication details
15. **[🔌 GPIO Pin Configuration](gpio_pin_configuration.md)** – Pin mapping, active levels, and mode pin setup
16. **[📝 Register Access](special_features_register_access.md)** – Direct register read/write and initialization flow
17. **[🔄 TMC5130 Support](tmc5130_support.md)** – Differences between TMC5130 and TMC5160

### **Troubleshooting**

18. **[🐛 Troubleshooting](troubleshooting.md)** – Common issues and solutions

---

## 🚀 Quick Start Path

**New to TMC51x0?** Follow this recommended path:

1. Start with **[Installation](installation.md)** to prepare your environment
2. Follow **[Hardware Setup](hardware_setup.md)** to wire your hardware
3. Read **[Quick Start](quickstart.md)** for a minimal working example and `Result<T>` error handling
4. Check **[Platform Integration](platform_integration.md)** to implement the SPI/UART interface
5. Explore **[Examples](examples.md)** for progressively harder walkthroughs with explanations
6. Refer to **[Troubleshooting](troubleshooting.md)** when things don't work as expected

---

## 💡 Need Help?

- **🐛 Found a bug?** Check the [Troubleshooting](troubleshooting.md) guide
- **❓ Have questions?** Review the [API Reference](api_reference.md)
- **📝 Want to contribute?** See the contributing guidelines in the main README

---

**Navigation**
[Next: Installation ->](installation.md)

