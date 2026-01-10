/**
 * @file espnow_peer_store.hpp
 * @brief NVS-based storage for approved ESP-NOW peers
 * 
 * Manages the list of approved (paired) peers stored in Non-Volatile Storage.
 * Provides functions to add, remove, and query approved peers.
 */

#pragma once

#include "espnow_security.hpp"
#include <cstdint>

namespace PeerStore {

/**
 * @brief Initialize peer storage and load approved peers from NVS.
 * 
 * @param sec Security settings structure to populate
 * @param preconfigured_mac Optional pre-configured MAC from compile-time config.
 *                          If non-zero, this MAC is always trusted (backward compatibility).
 * @param preconfigured_type DeviceType of the pre-configured MAC
 * @param preconfigured_name Name for the pre-configured peer
 */
void Init(SecuritySettings& sec, 
          const uint8_t* preconfigured_mac = nullptr,
          DeviceType preconfigured_type = DeviceType::Unknown,
          const char* preconfigured_name = nullptr) noexcept;

/**
 * @brief Add a new approved peer to storage.
 * 
 * If the peer already exists (by MAC), updates the existing entry.
 * 
 * @param sec Security settings structure
 * @param mac Peer's MAC address
 * @param type Peer's device type
 * @param name Human-readable name for the peer
 * @return true if added/updated successfully, false if no room
 */
bool AddPeer(SecuritySettings& sec, const uint8_t mac[6], 
             DeviceType type, const char* name) noexcept;

/**
 * @brief Remove a peer by MAC address.
 * 
 * @param sec Security settings structure
 * @param mac MAC address of peer to remove
 * @return true if removed, false if not found
 */
bool RemovePeer(SecuritySettings& sec, const uint8_t mac[6]) noexcept;

/**
 * @brief Check if a MAC address is in the approved peer list.
 * 
 * @param sec Security settings structure
 * @param mac MAC address to check
 * @return true if peer is approved
 */
bool IsPeerApproved(const SecuritySettings& sec, const uint8_t mac[6]) noexcept;

/**
 * @brief Get peer information by MAC address.
 * 
 * @param sec Security settings structure
 * @param mac MAC address to look up
 * @return Pointer to peer info, or nullptr if not found
 */
const ApprovedPeer* GetPeer(const SecuritySettings& sec, const uint8_t mac[6]) noexcept;

/**
 * @brief Get the first valid peer MAC of a specific device type.
 * 
 * Useful for getting the default target MAC when sending messages.
 * 
 * @param sec Security settings structure
 * @param type Device type to find
 * @param mac_out Output buffer for MAC address (6 bytes)
 * @return true if found, false if no peer of that type exists
 */
bool GetFirstPeerOfType(const SecuritySettings& sec, DeviceType type, 
                        uint8_t mac_out[6]) noexcept;

/**
 * @brief Save the current peer list to NVS.
 * 
 * Called automatically by AddPeer/RemovePeer, but can be called
 * manually if needed.
 * 
 * @param sec Security settings structure
 */
void Save(const SecuritySettings& sec) noexcept;

/**
 * @brief Get the number of valid approved peers.
 * 
 * @param sec Security settings structure
 * @return Number of approved peers (0 to MAX_APPROVED_PEERS)
 */
size_t GetPeerCount(const SecuritySettings& sec) noexcept;

/**
 * @brief Clear all approved peers (factory reset).
 * 
 * @param sec Security settings structure
 */
void ClearAll(SecuritySettings& sec) noexcept;

/**
 * @brief Log all approved peers (for debugging).
 * 
 * @param sec Security settings structure
 */
void LogPeers(const SecuritySettings& sec) noexcept;

} // namespace PeerStore

