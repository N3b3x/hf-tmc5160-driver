/**
 * @file espnow_security.hpp
 * @brief ESP-NOW security and pairing protocol definitions
 * 
 * This header defines the secure pairing mechanism for ESP-NOW communication.
 * Both the remote controller and test unit must share this file for protocol
 * compatibility.
 * 
 * Security Model:
 * - Pre-shared pairing secret compiled into both devices
 * - Challenge-response HMAC for mutual authentication
 * - Explicit pairing mode required (device must be put in pairing mode)
 * - Approved peers stored in NVS for persistence
 * - All non-pairing messages validated against approved peer list
 * 
 * Backward Compatibility:
 * - Pre-configured MAC addresses (hardcoded) are always trusted
 * - Pairing is an additional feature, not a replacement
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "esp_random.h"
#include "mbedtls/md.h"

// ============================================================================
// PAIRING SECRET CONFIGURATION
// ============================================================================
//
// The pairing secret is injected at build time via CMake.
// Configuration sources (in priority order):
//   1. --secret command line argument to build_app.sh
//   2. ESPNOW_PAIRING_SECRET environment variable
//   3. secrets.local.yml file (gitignored)
//   4. Auto-generate for DEBUG builds (with warning)
//   5. Build error for RELEASE builds
//
// See secrets.template.yml for configuration instructions.
// ============================================================================

#ifndef ESPNOW_PAIRING_SECRET_HEX
    #if defined(NDEBUG)
        // RELEASE build without secret - fail with helpful message
        #error "ESPNOW_PAIRING_SECRET not configured for RELEASE build. " \
               "Copy secrets.template.yml to secrets.local.yml and add your secret. " \
               "Generate with: openssl rand -hex 16"
    #else
        // DEBUG build - use placeholder with warning
        #warning "Using auto-generated pairing secret for DEBUG build. NOT SECURE for production!"
        #define ESPNOW_PAIRING_SECRET_HEX "00000000deadbeefcafebabedeadbeef"
    #endif
#endif

// Validate secret length at compile time
static_assert(sizeof(ESPNOW_PAIRING_SECRET_HEX) == 33, 
              "ESPNOW_PAIRING_SECRET_HEX must be exactly 32 hex characters");

/**
 * @brief Compile-time hex string to byte array parser.
 * 
 * Converts a 32-character hex string to 16 bytes at compile time.
 */
namespace PairingSecretParser {
    /// Convert a single hex character to its numeric value (0-15)
    constexpr uint8_t HexCharToNibble(char c) noexcept {
        return (c >= '0' && c <= '9') ? static_cast<uint8_t>(c - '0') :
               (c >= 'a' && c <= 'f') ? static_cast<uint8_t>(c - 'a' + 10) :
               (c >= 'A' && c <= 'F') ? static_cast<uint8_t>(c - 'A' + 10) : 0;
    }
    
    /// Convert two hex characters at position i*2 to a single byte
    constexpr uint8_t HexByte(const char* s, size_t i) noexcept {
        return static_cast<uint8_t>(
            (HexCharToNibble(s[i * 2]) << 4) | HexCharToNibble(s[i * 2 + 1])
        );
    }
} // namespace PairingSecretParser

/**
 * @brief Pre-shared pairing secret (16 bytes).
 * 
 * This secret is parsed from ESPNOW_PAIRING_SECRET_HEX at compile time.
 * All devices in the same deployment must use the same secret.
 * 
 * @note Configure via secrets.local.yml or environment variable.
 * @see secrets.template.yml for configuration instructions.
 */
static constexpr uint8_t PAIRING_SECRET[16] = {
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 0),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 1),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 2),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 3),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 4),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 5),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 6),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 7),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 8),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 9),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 10),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 11),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 12),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 13),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 14),
    PairingSecretParser::HexByte(ESPNOW_PAIRING_SECRET_HEX, 15)
};

/// Challenge nonce size in bytes
static constexpr size_t CHALLENGE_SIZE = 8;

/// HMAC output size (truncated SHA-256)
static constexpr size_t HMAC_SIZE = 16;

/// Maximum number of approved peers to store in NVS
static constexpr size_t MAX_APPROVED_PEERS = 4;

/// Maximum device name length
static constexpr size_t MAX_DEVICE_NAME_LEN = 16;

/// Pairing mode timeout in seconds (default)
static constexpr uint32_t PAIRING_MODE_TIMEOUT_SEC = 30;

/// Pairing response timeout in milliseconds
static constexpr uint32_t PAIRING_RESPONSE_TIMEOUT_MS = 10000;

/// Broadcast MAC address for discovery
static constexpr uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ============================================================================
// DEVICE TYPES
// ============================================================================

/**
 * @brief Device type identifiers for mutual verification.
 * 
 * During pairing, devices verify they're connecting to the expected type.
 */
enum class DeviceType : uint8_t {
    Unknown          = 0,
    RemoteController = 1,   ///< UI board / M5Dial remote controller
    FatigueTester    = 2,   ///< Fatigue test unit
    // Future device types can be added here
};

// ============================================================================
// PAIRING MESSAGE TYPES (extend existing MsgType enum)
// ============================================================================

/**
 * @brief Security/Pairing message type values.
 * 
 * These should be added to the MsgType enum in espnow_protocol.hpp
 * Using values 20-29 to avoid collision with existing message types.
 */
namespace PairingMsgType {
    static constexpr uint8_t PairingRequest  = 20;  ///< Initiate pairing (broadcast)
    static constexpr uint8_t PairingResponse = 21;  ///< Response with HMAC proof
    static constexpr uint8_t PairingConfirm  = 22;  ///< Final confirmation
    static constexpr uint8_t PairingReject   = 23;  ///< Explicit rejection
    static constexpr uint8_t Unpair          = 24;  ///< Remove a paired device
}

// ============================================================================
// PAIRING REJECTION REASONS
// ============================================================================

enum class PairingRejectReason : uint8_t {
    NotInPairingMode = 0,   ///< Device not in pairing mode
    WrongDeviceType  = 1,   ///< Requester looking for wrong device type
    HmacFailed       = 2,   ///< HMAC verification failed
    AlreadyPaired    = 3,   ///< Already paired (peer list full)
    ProtocolMismatch = 4,   ///< Protocol version mismatch
};

// ============================================================================
// PAIRING MESSAGE STRUCTURES
// ============================================================================

#pragma pack(push, 1)

/**
 * @brief Pairing request payload - sent by initiator (broadcast).
 * 
 * The initiator generates a random challenge. The responder must prove
 * knowledge of the shared secret by computing HMAC(secret, challenge).
 */
struct PairingRequestPayload {
    uint8_t  requester_mac[6];       ///< Requester's MAC address
    uint8_t  device_type;            ///< DeviceType - what I am
    uint8_t  expected_peer_type;     ///< DeviceType - what I'm looking for
    uint8_t  challenge[CHALLENGE_SIZE];  ///< Random nonce for verification
    uint8_t  protocol_version;       ///< Protocol version for compatibility
};

/**
 * @brief Pairing response payload - sent by responder (unicast).
 * 
 * The responder proves identity via HMAC and issues a counter-challenge
 * for mutual authentication.
 */
struct PairingResponsePayload {
    uint8_t  responder_mac[6];       ///< Responder's MAC address
    uint8_t  device_type;            ///< DeviceType - what I am
    uint8_t  challenge[CHALLENGE_SIZE];     ///< My challenge for mutual auth
    uint8_t  hmac_response[HMAC_SIZE];      ///< HMAC(secret, requester_challenge)
    char     device_name[MAX_DEVICE_NAME_LEN]; ///< Human-readable name
};

/**
 * @brief Pairing confirmation payload - sent by initiator to complete.
 * 
 * Completes mutual authentication by proving the initiator also knows
 * the shared secret.
 */
struct PairingConfirmPayload {
    uint8_t  confirmer_mac[6];       ///< Confirmer's MAC address
    uint8_t  hmac_response[HMAC_SIZE];      ///< HMAC(secret, responder_challenge)
    uint8_t  success;                ///< 1 = success, 0 = verification failed
};

/**
 * @brief Pairing rejection payload.
 */
struct PairingRejectPayload {
    uint8_t  rejecter_mac[6];        ///< Rejecter's MAC address
    uint8_t  reason;                 ///< PairingRejectReason
};

#pragma pack(pop)

// ============================================================================
// APPROVED PEER STORAGE STRUCTURES
// ============================================================================

/**
 * @brief Information about an approved (paired) peer.
 */
struct ApprovedPeer {
    uint8_t  mac[6];                         ///< Peer's MAC address
    uint8_t  device_type;                    ///< DeviceType enum value
    char     name[MAX_DEVICE_NAME_LEN];      ///< Friendly name
    uint32_t paired_timestamp;               ///< Unix timestamp when paired (or 0)
    bool     valid;                          ///< true if this slot is in use
};

/**
 * @brief Security settings stored in NVS.
 */
struct SecuritySettings {
    ApprovedPeer approved_peers[MAX_APPROVED_PEERS];
};

// ============================================================================
// HMAC COMPUTATION FUNCTIONS
// ============================================================================

/**
 * @brief Compute HMAC-SHA256, truncated to HMAC_SIZE bytes.
 * 
 * @param challenge The challenge nonce to authenticate
 * @param challenge_len Length of challenge in bytes
 * @param out Output buffer (must be at least HMAC_SIZE bytes)
 */
inline void ComputePairingHmac(const uint8_t* challenge, size_t challenge_len,
                               uint8_t out[HMAC_SIZE]) noexcept
{
    uint8_t full_hmac[32];  // SHA-256 produces 32 bytes
    
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    mbedtls_md_hmac_starts(&ctx, PAIRING_SECRET, sizeof(PAIRING_SECRET));
    mbedtls_md_hmac_update(&ctx, challenge, challenge_len);
    mbedtls_md_hmac_finish(&ctx, full_hmac);
    mbedtls_md_free(&ctx);
    
    // Truncate to HMAC_SIZE bytes for space efficiency
    std::memcpy(out, full_hmac, HMAC_SIZE);
}

/**
 * @brief Verify an HMAC response using constant-time comparison.
 * 
 * @param challenge The challenge that was sent
 * @param challenge_len Length of challenge in bytes
 * @param received_hmac The HMAC response to verify
 * @return true if HMAC matches (peer knows the secret)
 */
inline bool VerifyPairingHmac(const uint8_t* challenge, size_t challenge_len,
                              const uint8_t received_hmac[HMAC_SIZE]) noexcept
{
    uint8_t expected[HMAC_SIZE];
    ComputePairingHmac(challenge, challenge_len, expected);
    
    // Constant-time comparison to prevent timing attacks
    uint8_t diff = 0;
    for (size_t i = 0; i < HMAC_SIZE; ++i) {
        diff |= (expected[i] ^ received_hmac[i]);
    }
    return diff == 0;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Generate a random challenge nonce.
 * 
 * @param out Output buffer (must be at least CHALLENGE_SIZE bytes)
 */
inline void GenerateChallenge(uint8_t out[CHALLENGE_SIZE]) noexcept
{
    esp_fill_random(out, CHALLENGE_SIZE);
}

/**
 * @brief Check if a MAC address is all zeros.
 */
inline bool IsZeroMac(const uint8_t mac[6]) noexcept
{
    for (int i = 0; i < 6; ++i) {
        if (mac[i] != 0) return false;
    }
    return true;
}

/**
 * @brief Check if a MAC address is the broadcast address.
 */
inline bool IsBroadcastMac(const uint8_t mac[6]) noexcept
{
    return std::memcmp(mac, BROADCAST_MAC, 6) == 0;
}

/**
 * @brief Compare two MAC addresses.
 */
inline bool MacEquals(const uint8_t a[6], const uint8_t b[6]) noexcept
{
    return std::memcmp(a, b, 6) == 0;
}

/**
 * @brief Format MAC address to string buffer (needs 18 bytes: XX:XX:XX:XX:XX:XX\0)
 */
inline void FormatMac(const uint8_t mac[6], char* out, size_t out_size) noexcept
{
    if (out_size >= 18) {
        snprintf(out, out_size, "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

