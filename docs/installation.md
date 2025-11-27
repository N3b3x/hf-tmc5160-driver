---
layout: default
title: "🛠️ Installation"
description: "Installation and build instructions for the TMC51x0 driver (TMC5130 & TMC5160)"
nav_order: 1
parent: "📚 Documentation"
permalink: /docs/installation/
---

# Installation

This guide covers how to obtain, build, and verify the TMC51x0 driver (TMC5130 & TMC5160) library.

## Prerequisites

Before installing the driver, ensure you have:

- **C++17 Compiler**: GCC 7+, Clang 5+, or MSVC 2017+
- **Build System**: CMake 3.15+ or ESP-IDF (for ESP32 examples)
- **Platform SDK**: ESP-IDF, STM32 HAL, or Arduino (depending on your platform)

## Obtaining the Source

### Option 1: Git Clone

```bash
git clone https://github.com/N3b3x/hf-tmc5160-driver.git
cd hf-tmc5160-driver
```

### Option 2: Copy Files

Copy the following files into your project:

```
inc/
  ├── tmc51x0.hpp
  ├── tmc51x0_comm_interface.hpp
  ├── tmc51x0_registers.hpp
  ├── tmc51x0_types.hpp
```

The driver is header-only (template-based), so no separate source files are needed.

## Building the Library

### Using ESP-IDF Component

For ESP32 projects, add the driver as a component:

1. Copy the driver to your `components/` directory:
   ```bash
   cp -r hf-tmc5160-driver components/hf_tmc51x0
   ```

2. Include in your `CMakeLists.txt`:
   ```cmake
   idf_component_register(
       INCLUDE_DIRS "inc"
       REQUIRES driver
   )
   ```

3. Use in your code:
   ```cpp
   #include "tmc51x0.hpp"
   ```

### Using CMake

```cmake
add_subdirectory(hf-tmc5160-driver)
target_link_libraries(your_target PRIVATE hf_tmc51x0)
```

### Header-Only Usage

Since the driver is header-only, you can simply include it:

```cpp
#include "inc/tmc51x0.hpp"
```

Make sure your compiler has C++17 support enabled.

## Verification

To verify the installation:

1. Include the header in a test file:
   ```cpp
   #include "inc/tmc51x0.hpp"
   ```

2. Compile a simple test:
   ```bash
   g++ -std=c++17 -I inc/ -c test.cpp -o test.o
   ```

3. If compilation succeeds, the library is properly installed.

## Next Steps

- Follow the [Quick Start](quickstart.md) guide to create your first application
- Review [Hardware Setup](hardware_setup.md) for wiring instructions
- Check [Platform Integration](platform_integration.md) to implement the communication interface

---

**Navigation**
⬅️ [Back to Index](index.md) | [Next: Quick Start ➡️](quickstart.md)

