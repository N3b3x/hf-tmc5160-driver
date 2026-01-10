/**
 * @file espnow_peer_store.cpp
 * @brief NVS-based storage implementation for approved ESP-NOW peers
 */

#include "espnow_peer_store.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_crc.h"
#include <cstring>

static const char* TAG = "PeerStore";

namespace {

/// NVS namespace for peer storage
const char* NVS_NAMESPACE = "espnow_peers";

/// NVS key for peer blob
const char* KEY_PEERS = "peers";

/// NVS key for CRC
const char* KEY_CRC = "peers_crc";

/// Pre-configured peer (from compile-time config) - always trusted
ApprovedPeer s_preconfigured_peer{};
bool s_has_preconfigured = false;

} // namespace

void PeerStore::Init(SecuritySettings& sec,
                     const uint8_t* preconfigured_mac,
                     DeviceType preconfigured_type,
                     const char* preconfigured_name) noexcept
{
    // Clear the structure
    std::memset(&sec, 0, sizeof(sec));
    for (auto& peer : sec.approved_peers) {
        peer.valid = false;
    }
    
    // Store pre-configured MAC for backward compatibility
    if (preconfigured_mac != nullptr && !IsZeroMac(preconfigured_mac)) {
        s_has_preconfigured = true;
        std::memcpy(s_preconfigured_peer.mac, preconfigured_mac, 6);
        s_preconfigured_peer.device_type = static_cast<uint8_t>(preconfigured_type);
        if (preconfigured_name) {
            strncpy(s_preconfigured_peer.name, preconfigured_name, 
                    sizeof(s_preconfigured_peer.name) - 1);
        } else {
            strncpy(s_preconfigured_peer.name, "Pre-configured", 
                    sizeof(s_preconfigured_peer.name) - 1);
        }
        s_preconfigured_peer.valid = true;
        
        ESP_LOGI(TAG, "Pre-configured peer registered: %02X:%02X:%02X:%02X:%02X:%02X (%s)",
                 preconfigured_mac[0], preconfigured_mac[1], preconfigured_mac[2],
                 preconfigured_mac[3], preconfigured_mac[4], preconfigured_mac[5],
                 s_preconfigured_peer.name);
    }
    
    // Open NVS
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return;
    }
    
    // Try to read existing peer data
    size_t required_size = sizeof(SecuritySettings);
    size_t blob_size = 0;
    
    err = nvs_get_blob(h, KEY_PEERS, nullptr, &blob_size);
    
    if (err == ESP_OK && blob_size == required_size) {
        SecuritySettings loaded{};
        err = nvs_get_blob(h, KEY_PEERS, &loaded, &blob_size);
        
        if (err == ESP_OK) {
            // Verify CRC
            uint32_t stored_crc = 0;
            if (nvs_get_u32(h, KEY_CRC, &stored_crc) == ESP_OK) {
                uint32_t calc_crc = esp_crc32_le(0, (const uint8_t*)&loaded, 
                                                  sizeof(SecuritySettings));
                if (calc_crc == stored_crc) {
                    // CRC matches, use loaded data
                    sec = loaded;
                    ESP_LOGI(TAG, "Loaded peer list from NVS (CRC: 0x%08lx)", stored_crc);
                } else {
                    ESP_LOGW(TAG, "Peer list CRC mismatch, starting fresh");
                }
            } else {
                ESP_LOGW(TAG, "Peer list CRC missing, starting fresh");
            }
        }
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved peer list found, starting fresh");
    } else {
        ESP_LOGW(TAG, "Peer list size mismatch or error, starting fresh");
    }
    
    nvs_close(h);
    
    // Log current state
    LogPeers(sec);
}

bool PeerStore::AddPeer(SecuritySettings& sec, const uint8_t mac[6],
                        DeviceType type, const char* name) noexcept
{
    if (IsZeroMac(mac)) {
        ESP_LOGW(TAG, "Cannot add zero MAC address");
        return false;
    }
    
    // Check if already exists - update if so
    for (auto& peer : sec.approved_peers) {
        if (peer.valid && MacEquals(peer.mac, mac)) {
            peer.device_type = static_cast<uint8_t>(type);
            if (name) {
                strncpy(peer.name, name, sizeof(peer.name) - 1);
                peer.name[sizeof(peer.name) - 1] = '\0';
            }
            ESP_LOGI(TAG, "Updated peer: %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            Save(sec);
            return true;
        }
    }
    
    // Find empty slot
    for (auto& peer : sec.approved_peers) {
        if (!peer.valid) {
            std::memcpy(peer.mac, mac, 6);
            peer.device_type = static_cast<uint8_t>(type);
            peer.paired_timestamp = 0;  // Could add real timestamp if needed
            peer.valid = true;
            if (name) {
                strncpy(peer.name, name, sizeof(peer.name) - 1);
                peer.name[sizeof(peer.name) - 1] = '\0';
            } else {
                strncpy(peer.name, "Unknown", sizeof(peer.name) - 1);
            }
            
            ESP_LOGI(TAG, "Added new peer: %02X:%02X:%02X:%02X:%02X:%02X (%s)",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], peer.name);
            Save(sec);
            return true;
        }
    }
    
    ESP_LOGW(TAG, "No room for new peer (max %zu)", MAX_APPROVED_PEERS);
    return false;
}

bool PeerStore::RemovePeer(SecuritySettings& sec, const uint8_t mac[6]) noexcept
{
    for (auto& peer : sec.approved_peers) {
        if (peer.valid && MacEquals(peer.mac, mac)) {
            ESP_LOGI(TAG, "Removed peer: %02X:%02X:%02X:%02X:%02X:%02X (%s)",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], peer.name);
            peer.valid = false;
            std::memset(&peer, 0, sizeof(peer));
            Save(sec);
            return true;
        }
    }
    return false;
}

bool PeerStore::IsPeerApproved(const SecuritySettings& sec, const uint8_t mac[6]) noexcept
{
    if (IsZeroMac(mac)) {
        return false;
    }
    
    // Check pre-configured peer first (backward compatibility)
    if (s_has_preconfigured && MacEquals(s_preconfigured_peer.mac, mac)) {
        return true;
    }
    
    // Check dynamically paired peers
    for (const auto& peer : sec.approved_peers) {
        if (peer.valid && MacEquals(peer.mac, mac)) {
            return true;
        }
    }
    
    return false;
}

const ApprovedPeer* PeerStore::GetPeer(const SecuritySettings& sec, 
                                        const uint8_t mac[6]) noexcept
{
    // Check pre-configured peer first
    if (s_has_preconfigured && MacEquals(s_preconfigured_peer.mac, mac)) {
        return &s_preconfigured_peer;
    }
    
    for (const auto& peer : sec.approved_peers) {
        if (peer.valid && MacEquals(peer.mac, mac)) {
            return &peer;
        }
    }
    return nullptr;
}

bool PeerStore::GetFirstPeerOfType(const SecuritySettings& sec, DeviceType type,
                                    uint8_t mac_out[6]) noexcept
{
    uint8_t type_val = static_cast<uint8_t>(type);
    
    // Check pre-configured peer first
    if (s_has_preconfigured && s_preconfigured_peer.device_type == type_val) {
        std::memcpy(mac_out, s_preconfigured_peer.mac, 6);
        return true;
    }
    
    for (const auto& peer : sec.approved_peers) {
        if (peer.valid && peer.device_type == type_val) {
            std::memcpy(mac_out, peer.mac, 6);
            return true;
        }
    }
    return false;
}

void PeerStore::Save(const SecuritySettings& sec) noexcept
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for write: %s", esp_err_to_name(err));
        return;
    }
    
    // Calculate CRC
    uint32_t crc = esp_crc32_le(0, (const uint8_t*)&sec, sizeof(SecuritySettings));
    
    // Write blob and CRC
    err = nvs_set_blob(h, KEY_PEERS, &sec, sizeof(SecuritySettings));
    if (err == ESP_OK) {
        err = nvs_set_u32(h, KEY_CRC, crc);
    }
    
    if (err == ESP_OK) {
        err = nvs_commit(h);
        if (err == ESP_OK) {
            ESP_LOGD(TAG, "Peer list saved (CRC: 0x%08lx)", crc);
        } else {
            ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "NVS write failed: %s", esp_err_to_name(err));
    }
    
    nvs_close(h);
}

size_t PeerStore::GetPeerCount(const SecuritySettings& sec) noexcept
{
    size_t count = 0;
    
    // Count pre-configured if present
    if (s_has_preconfigured) {
        ++count;
    }
    
    for (const auto& peer : sec.approved_peers) {
        if (peer.valid) {
            ++count;
        }
    }
    return count;
}

void PeerStore::ClearAll(SecuritySettings& sec) noexcept
{
    ESP_LOGW(TAG, "Clearing all approved peers");
    for (auto& peer : sec.approved_peers) {
        peer.valid = false;
        std::memset(&peer, 0, sizeof(peer));
    }
    Save(sec);
}

void PeerStore::LogPeers(const SecuritySettings& sec) noexcept
{
    size_t count = GetPeerCount(sec);
    ESP_LOGI(TAG, "Approved peers: %zu", count);
    
    if (s_has_preconfigured) {
        ESP_LOGI(TAG, "  [PRE] %02X:%02X:%02X:%02X:%02X:%02X (%s, type=%u)",
                 s_preconfigured_peer.mac[0], s_preconfigured_peer.mac[1],
                 s_preconfigured_peer.mac[2], s_preconfigured_peer.mac[3],
                 s_preconfigured_peer.mac[4], s_preconfigured_peer.mac[5],
                 s_preconfigured_peer.name, s_preconfigured_peer.device_type);
    }
    
    for (size_t i = 0; i < MAX_APPROVED_PEERS; ++i) {
        const auto& peer = sec.approved_peers[i];
        if (peer.valid) {
            ESP_LOGI(TAG, "  [%zu] %02X:%02X:%02X:%02X:%02X:%02X (%s, type=%u)",
                     i, peer.mac[0], peer.mac[1], peer.mac[2],
                     peer.mac[3], peer.mac[4], peer.mac[5],
                     peer.name, peer.device_type);
        }
    }
}

