# ESP-NOW Secure Pairing Protocol

## Overview

This document describes the secure pairing protocol used between the Remote Controller and Fatigue Test Unit for ESP-NOW communication. The protocol provides mutual authentication using HMAC-SHA256 challenge-response, ensuring both devices share a pre-configured secret before establishing communication.

## Security Goals

1. **Mutual Authentication**: Both devices prove knowledge of the shared secret
2. **Replay Attack Prevention**: Random challenges prevent message replay
3. **Device Type Verification**: Controllers only pair with testers, and vice versa
4. **Explicit User Action**: Pairing requires manual action on both devices
5. **Persistence**: Approved peers survive device reboots
6. **Backward Compatibility**: Pre-configured MAC addresses work without pairing

## Protocol Participants

| Role | Device | Description |
|------|--------|-------------|
| **Initiator** | Remote Controller | Starts pairing, sends challenge, verifies response |
| **Responder** | Fatigue Test Unit | Receives request (when in pairing mode), proves identity |

## Shared Secret Configuration

Both devices must be compiled with the same 16-byte pairing secret. The secret is injected at build time and **never hardcoded in the source code**.

### Configuration Methods (Priority Order)

| Method | Usage | Best For |
|--------|-------|----------|
| `--secret` argument | `./build_app.sh app Release --secret <hex>` | CI/CD pipelines |
| Environment variable | `ESPNOW_PAIRING_SECRET=<hex> ./build_app.sh` | Development sessions |
| `secrets.local.yml` | Copy from template, add secret | Local development |

### Quick Setup

```bash
# 1. Generate a secret
openssl rand -hex 16

# 2. Copy the template
cp secrets.template.yml secrets.local.yml

# 3. Edit secrets.local.yml with your secret
espnow_pairing_secret: "your_32_char_hex_secret"

# 4. Build (secret is automatically loaded)
./scripts/build_app.sh fatigue_test_espnow_unit Release
```

### Build Behavior

| Build Type | Without Secret | With Secret |
|------------|----------------|-------------|
| **Debug** | Uses placeholder (warning shown) | Uses configured secret |
| **Release** | Compile error with instructions | Uses configured secret |

The secret is parsed at compile time into the `PAIRING_SECRET` byte array for HMAC-SHA256 computation.

---

## Message Types

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| `PairingRequest` | 20 | Controller → Broadcast | Initiate pairing |
| `PairingResponse` | 21 | Tester → Controller | Respond with HMAC proof |
| `PairingConfirm` | 22 | Controller → Tester | Confirm mutual authentication |
| `PairingReject` | 23 | Tester → Controller | Reject pairing request |
| `Unpair` | 24 | Either direction | Remove a paired device |

---

## Message Structures

### PairingRequest (Controller → Broadcast)

```cpp
struct PairingRequestPayload {
    uint8_t  requester_mac[6];       // Controller's MAC address
    uint8_t  device_type;            // DeviceType::RemoteController (1)
    uint8_t  expected_peer_type;     // DeviceType::FatigueTester (2)
    uint8_t  challenge[8];           // Random 8-byte nonce
    uint8_t  protocol_version;       // Must be 1
};
```

**Size**: 18 bytes

### PairingResponse (Tester → Controller)

```cpp
struct PairingResponsePayload {
    uint8_t  responder_mac[6];       // Tester's MAC address
    uint8_t  device_type;            // DeviceType::FatigueTester (2)
    uint8_t  challenge[8];           // Counter-challenge for mutual auth
    uint8_t  hmac_response[16];      // HMAC(secret, requester_challenge)
    char     device_name[16];        // Human-readable name
};
```

**Size**: 47 bytes

### PairingConfirm (Controller → Tester)

```cpp
struct PairingConfirmPayload {
    uint8_t  confirmer_mac[6];       // Controller's MAC address
    uint8_t  hmac_response[16];      // HMAC(secret, responder_challenge)
    uint8_t  success;                // 1 = success, 0 = failure
};
```

**Size**: 23 bytes

### PairingReject (Tester → Controller)

```cpp
struct PairingRejectPayload {
    uint8_t  rejecter_mac[6];        // Tester's MAC address
    uint8_t  reason;                 // Rejection reason code
};
```

**Rejection Reasons**:
| Code | Reason |
|------|--------|
| 0 | Not in pairing mode |
| 1 | Wrong device type |
| 2 | HMAC verification failed |
| 3 | Peer list full |
| 4 | Protocol version mismatch |

---

## Protocol Flow

### Successful Pairing Sequence

```
┌───────────────────┐                              ┌────────────────────┐
│ Remote Controller │                              │ Fatigue Test Unit  │
│   (Initiator)     │                              │    (Responder)     │
└─────────┬─────────┘                              └──────────┬─────────┘
          │                                                   │
          │  User: Selects "Pair Device" in UI menu           │
          │◄─────────────────────────────────────────         │
          │                                                   │
          │                           User: Types "pair" in UART terminal
          │                         ──────────────────────────►│
          │                                                   │
          │              [Tester enters PAIRING MODE for 30s] │
          │                                                   │
          │  1. Generate random 8-byte challenge              │
          │  ───────────────────────────────────              │
          │                                                   │
          │  2. PAIRING_REQUEST (broadcast)                   │
          │  ─────────────────────────────────────────────────►│
          │  - requester_mac = Controller MAC                 │
          │  - device_type = RemoteController (1)             │
          │  - expected_peer_type = FatigueTester (2)         │
          │  - challenge = [8 random bytes]                   │
          │  - protocol_version = 1                           │
          │                                                   │
          │                  [Tester validates request]       │
          │                  - In pairing mode? ✓             │
          │                  - Expected type matches? ✓       │
          │                  - Protocol version matches? ✓    │
          │                                                   │
          │                  [Tester computes HMAC response]  │
          │                  hmac = HMAC-SHA256(secret, challenge)
          │                                                   │
          │  3. PAIRING_RESPONSE (unicast to controller)      │
          │◄─────────────────────────────────────────────────│
          │  - responder_mac = Tester MAC                     │
          │  - device_type = FatigueTester (2)                │
          │  - challenge = [8 new random bytes]               │
          │  - hmac_response = HMAC of controller's challenge │
          │  - device_name = "Fatigue Tester"                 │
          │                                                   │
          │  [Controller verifies HMAC]                       │
          │  expected = HMAC-SHA256(secret, my_challenge)     │
          │  if (expected == received) → Tester is authentic  │
          │                                                   │
          │  [Controller computes HMAC for tester's challenge]│
          │  my_hmac = HMAC-SHA256(secret, tester_challenge)  │
          │                                                   │
          │  4. PAIRING_CONFIRM (unicast to tester)           │
          │  ─────────────────────────────────────────────────►│
          │  - confirmer_mac = Controller MAC                 │
          │  - hmac_response = HMAC of tester's challenge     │
          │  - success = 1                                    │
          │                                                   │
          │                  [Tester verifies HMAC]           │
          │                  expected = HMAC-SHA256(secret, my_challenge)
          │                  if (expected == received) → Controller authentic
          │                                                   │
          │  [Controller adds Tester to approved list]        │
          │                  [Tester adds Controller to approved list]
          │                                                   │
          │  ═══════════ PAIRING COMPLETE ═══════════         │
          │                                                   │
          │  [Both save peer lists to NVS]                    │
          │                                                   │
          ▼                                                   ▼
```

### Failed Pairing: Not in Pairing Mode

```
┌───────────────────┐                              ┌────────────────────┐
│ Remote Controller │                              │ Fatigue Test Unit  │
└─────────┬─────────┘                              └──────────┬─────────┘
          │                                                   │
          │  PAIRING_REQUEST (broadcast)                      │
          │  ─────────────────────────────────────────────────►│
          │                                                   │
          │                  [Tester checks: In pairing mode?]│
          │                  Answer: NO                       │
          │                                                   │
          │  PAIRING_REJECT                                   │
          │◄─────────────────────────────────────────────────│
          │  - reason = 0 (Not in pairing mode)               │
          │                                                   │
          │  [Controller shows error in UI]                   │
          ▼                                                   ▼
```

### Failed Pairing: HMAC Verification Failed

```
┌───────────────────┐                              ┌────────────────────┐
│ Remote Controller │                              │ Fatigue Test Unit  │
│ (Wrong Secret)    │                              │ (Correct Secret)   │
└─────────┬─────────┘                              └──────────┬─────────┘
          │                                                   │
          │  PAIRING_REQUEST                                  │
          │  ─────────────────────────────────────────────────►│
          │                                                   │
          │  PAIRING_RESPONSE                                 │
          │◄─────────────────────────────────────────────────│
          │                                                   │
          │  [Controller computes HMAC with wrong secret]     │
          │  [HMAC doesn't match response → REJECT]           │
          │                                                   │
          │  [Controller does NOT send PAIRING_CONFIRM]       │
          │  [Pairing fails with "Unauthorized device" error] │
          │                                                   │
          ▼                                                   ▼
```

---

## HMAC Computation

HMAC is computed using HMAC-SHA256 with the shared secret as the key:

```cpp
void ComputePairingHmac(const uint8_t* challenge, size_t challenge_len,
                        uint8_t out[16]) noexcept
{
    uint8_t full_hmac[32];  // SHA-256 output
    
    mbedtls_md_hmac(
        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
        PAIRING_SECRET, 16,      // Key
        challenge, challenge_len, // Message
        full_hmac                 // Output
    );
    
    // Truncate to 16 bytes
    memcpy(out, full_hmac, 16);
}
```

**Constant-Time Comparison**: HMAC verification uses constant-time comparison to prevent timing attacks:

```cpp
bool VerifyPairingHmac(const uint8_t* challenge, size_t len,
                       const uint8_t received[16]) noexcept
{
    uint8_t expected[16];
    ComputePairingHmac(challenge, len, expected);
    
    uint8_t diff = 0;
    for (size_t i = 0; i < 16; ++i) {
        diff |= (expected[i] ^ received[i]);
    }
    return diff == 0;  // Constant-time comparison
}
```

---

## State Machines

### Fatigue Test Unit (Responder)

```
                    ┌─────────┐
                    │  IDLE   │◄────────────────────────────┐
                    └────┬────┘                             │
                         │ "pair" UART command              │
                         ▼                                  │
                ┌────────────────┐                          │
         ┌──────│ PAIRING_MODE   │──────┐                   │
         │      │ (30s timeout)  │      │                   │
         │      └───────┬────────┘      │                   │
         │              │               │                   │
   timeout expires      │ PairingRequest received           │
         │              ▼               │                   │
         │   ┌──────────────────┐       │                   │
         │   │ AWAITING_CONFIRM │       │                   │
         │   │  (5s timeout)    │       │                   │
         │   └────────┬─────────┘       │                   │
         │            │                 │                   │
         │  PairingConfirm received     │                   │
         │            │                 │                   │
         │            ▼                 │                   │
         │   ┌──────────────────┐       │                   │
         │   │ VERIFY HMAC      │       │                   │
         │   └────────┬─────────┘       │                   │
         │            │                 │                   │
         │    ┌───────┴───────┐         │                   │
         │    │               │         │                   │
         │  PASS            FAIL        │                   │
         │    │               │         │                   │
         │    ▼               │         │                   │
         │  ADD TO            │         │                   │
         │  APPROVED          │         │                   │
         │  LIST              │         │                   │
         │    │               │         │                   │
         └────┴───────────────┴─────────┴───────────────────┘
```

### Remote Controller (Initiator)

```
                    ┌─────────┐
                    │  IDLE   │◄────────────────────────────┐
                    └────┬────┘                             │
                         │ User selects "Pair"              │
                         ▼                                  │
              ┌──────────────────────┐                      │
              │ WAITING_FOR_RESPONSE │                      │
              │   (10s timeout)      │──────┐               │
              └──────────┬───────────┘      │               │
                         │                  │               │
           PairingResponse received    timeout expires      │
                         │                  │               │
                         ▼                  │               │
              ┌──────────────────────┐      │               │
              │   VERIFY HMAC        │      │               │
              └──────────┬───────────┘      │               │
                         │                  │               │
                 ┌───────┴───────┐          │               │
                 │               │          │               │
               PASS            FAIL         │               │
                 │               │          │               │
                 ▼               ▼          ▼               │
         ┌─────────────┐   ┌─────────┐ ┌─────────┐          │
         │SEND CONFIRM │   │ FAILED  │ │ FAILED  │          │
         └──────┬──────┘   └────┬────┘ └────┬────┘          │
                │               │           │               │
                ▼               │           │               │
         ┌─────────────┐        │           │               │
         │ ADD TO LIST │        │           │               │
         └──────┬──────┘        │           │               │
                │               │           │               │
                ▼               │           │               │
         ┌─────────────┐        │           │               │
         │  COMPLETE   │        │           │               │
         └──────┬──────┘        │           │               │
                │               │           │               │
                └───────────────┴───────────┴───────────────┘
```

---

## NVS Storage

### Storage Format

Approved peers are stored in NVS under namespace `espnow_peers`:

| Key | Type | Description |
|-----|------|-------------|
| `peers` | Blob | Array of `ApprovedPeer` structures |
| `peers_crc` | u32 | CRC32 for data integrity |

### ApprovedPeer Structure

```cpp
struct ApprovedPeer {
    uint8_t  mac[6];             // Peer's MAC address
    uint8_t  device_type;        // DeviceType enum value
    char     name[16];           // Human-readable name
    uint32_t paired_timestamp;   // When paired (or 0)
    bool     valid;              // Slot in use?
};
```

Maximum peers: 4 per device

### Pre-Configured Peer

For backward compatibility, the compile-time MAC address (`TEST_UNIT_MAC_` or `UI_BOARD_MAC`) is always trusted and does not consume an NVS slot.

---

## Message Validation

### Security Gate

Every received message (except pairing messages) passes through this validation:

```cpp
bool ValidateMessageSource(const uint8_t* sender_mac, MsgType type) {
    // Pairing messages bypass validation
    if (type == PairingRequest || type == PairingResponse ||
        type == PairingConfirm || type == PairingReject) {
        return true;
    }
    
    // All other messages must be from approved peers
    return PeerStore::IsPeerApproved(security_settings, sender_mac);
}
```

**Rejected messages are silently dropped** to avoid giving attackers information about why their messages were rejected.

---

## API Reference

### Fatigue Test Unit (espnow_receiver.hpp)

```cpp
namespace EspNowReceiver {
    // Enter pairing mode for specified duration (default 30s)
    void enter_pairing_mode(uint32_t timeout_sec = 30);
    
    // Exit pairing mode early
    void exit_pairing_mode();
    
    // Check if currently in pairing mode
    bool is_in_pairing_mode();
    
    // Access security settings for peer management
    SecuritySettings& get_security_settings();
    
    // Manually add/remove approved peers
    bool add_approved_peer(const uint8_t mac[6], DeviceType type, const char* name);
    bool remove_approved_peer(const uint8_t mac[6]);
    
    // Get peer count
    size_t get_approved_peer_count();
}
```

### Remote Controller (espnow_protocol.hpp)

```cpp
namespace espnow {
    // Start pairing (broadcasts discovery)
    bool StartPairing() noexcept;
    
    // Cancel pairing attempt
    void CancelPairing() noexcept;
    
    // Get current pairing state
    PairingState GetPairingState() noexcept;
    
    // Peer management
    bool IsPeerApproved(const uint8_t mac[6]) noexcept;
    bool AddApprovedPeer(const uint8_t mac[6], DeviceType type, const char* name) noexcept;
    bool RemoveApprovedPeer(const uint8_t mac[6]) noexcept;
    size_t GetApprovedPeerCount() noexcept;
    
    // Get target device MAC for sending
    bool GetTargetDeviceMac(uint8_t mac_out[6]) noexcept;
}
```

---

## UART Commands (Fatigue Test Unit)

| Command | Description |
|---------|-------------|
| `pair` | Enter pairing mode for 30 seconds |

**Example Session**:
```
> pair
╔══════════════════════════════════════════════════════════════════════════════╗
║                              PAIRING MODE                                     ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ Pairing mode enabled for 30 seconds.                                         ║
║ Start pairing from your remote controller now.                               ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ Current approved peers:                                        1             ║
╚══════════════════════════════════════════════════════════════════════════════╝

[After successful pairing]
╔══════════════════════════════════════════════════════════════════════════════╗
║ PAIRING SUCCESSFUL!                                                           ║
║ Remote controller: 9C:9E:6E:77:24:F8                                         ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

---

## Security Considerations

### Threat Model

| Threat | Mitigation |
|--------|-----------|
| Unauthorized device sends commands | All messages validated against approved peer list |
| Rogue device tries to pair | HMAC verification ensures shared secret knowledge |
| Replay attack | Random 8-byte challenge in each pairing attempt |
| Timing attack on HMAC | Constant-time comparison |
| Eavesdropping during pairing | HMAC protects secret (only challenge/response visible) |
| Physical access to device | Pairing mode requires explicit action |

### Recommendations

1. **Change the default secret** before production deployment
2. **Limit pairing mode duration** (30 seconds is reasonable)
3. **Review approved peer list** periodically
4. **Consider enabling ESP-NOW encryption** for encrypted data transmission

---

## Troubleshooting

### Pairing Fails with "Not in pairing mode"

**Cause**: Fatigue test unit is not in pairing mode when request is sent.

**Solution**: 
1. On the fatigue test unit, type `pair` in UART terminal
2. Within 30 seconds, start pairing from the remote controller

### Pairing Fails with "HMAC verification failed"

**Cause**: Devices have different `PAIRING_SECRET` values.

**Solution**: Ensure both devices are compiled with identical `espnow_security.hpp` files.

### Device Not Responding to Commands After Pairing

**Cause**: Message validation is rejecting messages from the peer.

**Solution**: 
1. Check that pairing completed successfully on both sides
2. Verify peer was added to approved list: `get_approved_peer_count()`
3. Check NVS storage is working (no corruption)

### Pairing Timeout

**Cause**: Response not received within 10 seconds.

**Solution**:
1. Ensure devices are on same WiFi channel
2. Check for RF interference
3. Verify test unit is powered and in pairing mode

