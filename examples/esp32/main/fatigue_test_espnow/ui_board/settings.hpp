/**
 * @file settings.hpp
 * @brief Settings storage for UI board
 */

#pragma once

#include "../espnow_protocol.hpp"

namespace SettingsStore {

void init(Settings& s);
void save(const Settings& s);

} // namespace SettingsStore
