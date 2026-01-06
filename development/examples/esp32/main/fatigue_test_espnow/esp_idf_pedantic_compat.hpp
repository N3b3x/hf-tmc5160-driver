/**
 * @file esp_idf_pedantic_compat.hpp
 * @brief Compatibility include wrapper for ESP-IDF headers when building with -Wpedantic in C++.
 *
 * @details
 * ESP-IDF is primarily a C codebase and some of its public headers legitimately use
 * C extensions such as:
 * - zero-length arrays (`uint8_t payload[0]`)
 * - flexible array members (`uint8_t ssi[]`)
 *
 * When those headers are included from C++ translation units with `-Wpedantic`,
 * GCC emits warnings like:
 * - "ISO C++ forbids zero-size array"
 * - "ISO C++ forbids flexible array member"
 *
 * Those warnings are not actionable from the application side without patching ESP-IDF.
 * This wrapper locally suppresses `-Wpedantic` while including the affected headers,
 * allowing us to keep `-Wpedantic` enabled for *our* code.
 */

#pragma once

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include "esp_now.h"
#include "esp_wifi.h"

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif


