---
layout: default
title: "⚙️ CMake Integration"
description: "How to integrate the TMC51x0 driver into your CMake project"
nav_order: 5
parent: "📚 Documentation"
permalink: /docs/cmake_integration/
---

# TMC51x0 — CMake Integration Guide

> **Build contract architecture and templates:** See the main [Documentation](index.md) for integration details.

## Quick Start (Generic CMake)

```cmake
add_subdirectory(external/hf-tmc5160-driver)
target_link_libraries(my_app PRIVATE hf::tmc51x0)
```

## ESP-IDF Integration

Component wrapper: `examples/esp32/components/hf_tmc51x0/`

```cmake
idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES hf_tmc51x0 driver
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
```

## Build Variables

| Variable | Value |
|----------|-------|
| `HF_TMC51X0_TARGET_NAME` | `hf_tmc51x0` |
| `HF_TMC51X0_VERSION` | Current `MAJOR.MINOR.PATCH` |
| `HF_TMC51X0_SOURCE_FILES` | `""` (header-only) |
| `HF_TMC51X0_IDF_REQUIRES` | `driver` |

---

**Navigation**
⬅️ [Back to Documentation Index](index.md)
