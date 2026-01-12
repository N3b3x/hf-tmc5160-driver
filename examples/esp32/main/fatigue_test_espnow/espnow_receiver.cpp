/**
 * @file espnow_receiver.cpp
 * @brief ESP-NOW receiver implementation for test unit with secure pairing
 * 
 * Protocol compatible with esp32_remote_controller (6-byte header).
 * 
 * Features:
 * - Pre-configured MAC address support (backward compatibility)
 * - Secure pairing with HMAC mutual authentication
 * - NVS-based approved peer storage
 * - Message validation against approved peer list
 */

#include "espnow_receiver.hpp"
#include "espnow_protocol.hpp"
#include "espnow_peer_store.hpp"
#include <cstring>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_idf_pedantic_compat.hpp"
#include "esp_log.h"

static const char* TAG = "EspNowRx";

// ============================================================================
// MODULE STATE
// ============================================================================

static QueueHandle_t s_proto_event_queue = nullptr;
static uint8_t s_next_msg_id = 1;

/// Security settings with approved peer list
static SecuritySettings s_security{};

/// Active UI board MAC for replies (most recent approved sender)
static uint8_t s_active_ui_board_mac[6] = {0};

/// Receive queue from ISR to this module
static QueueHandle_t s_raw_recv_queue = nullptr;

// ============================================================================
// PAIRING STATE MACHINE
// ============================================================================

/// Pairing mode state
static volatile bool s_pairing_mode = false;
static volatile TickType_t s_pairing_timeout_tick = 0;

/// Pending pairing state (waiting for confirm after sending response)
static bool s_awaiting_pairing_confirm = false;
static uint8_t s_pending_requester_mac[6] = {0};
static uint8_t s_my_challenge[CHALLENGE_SIZE] = {0};
static TickType_t s_pairing_confirm_timeout = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static bool isZeroMac(const uint8_t mac[6])
{
    for (int i = 0; i < 6; ++i) {
        if (mac[i] != 0) return false;
    }
    return true;
}

static bool macEq(const uint8_t a[6], const uint8_t b[6])
{
    return std::memcmp(a, b, 6) == 0;
}

static void tryAddEspNowPeer(const uint8_t mac[6])
{
    if (isZeroMac(mac)) return;

    esp_now_peer_info_t peer{};
    std::memset(&peer, 0, sizeof(peer));
    std::memcpy(peer.peer_addr, mac, 6);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Failed to add peer %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], esp_err_to_name(err));
    }
}

static void setActiveUiMac(const uint8_t mac[6])
{
    if (!isZeroMac(mac)) {
        std::memcpy(s_active_ui_board_mac, mac, 6);
    }
}

// ============================================================================
// RAW MESSAGE STRUCTURE
// ============================================================================

struct RawMsg {
    uint8_t data[sizeof(EspNowPacket)];
    int     len;
    uint8_t src_mac[6];
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void espnowRecvCb(const esp_now_recv_info_t* info, const uint8_t* data, int len);
static void espnowSendCb(const wifi_tx_info_t* info, esp_now_send_status_t status);
static void recvTask(void*);
static void handlePacket(const RawMsg& msg, const EspNowPacket& pkt, uint16_t recv_crc);
static void handlePairingRequest(const uint8_t* src_mac, const EspNowPacket& pkt);
static void handlePairingConfirm(const uint8_t* src_mac, const EspNowPacket& pkt);
static bool sendPacketTo(const uint8_t* dst_mac, MsgType type, const void* payload, uint8_t payload_len);

// ============================================================================
// INITIALIZATION
// ============================================================================

bool EspNowReceiver::init(QueueHandle_t event_queue)
{
    s_proto_event_queue = event_queue;

    // Initialize NVS (required for WiFi and peer storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize peer storage with pre-configured MAC (backward compatibility)
    PeerStore::Init(s_security, UI_BOARD_MAC, DeviceType::RemoteController, "Pre-configured");

    // Init WiFi in STA mode
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Get and print MAC address for pairing
    uint8_t mac_addr[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac_addr));
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "Fatigue Test Unit MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "Configure this MAC in remote controller, or use secure pairing");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════════════════════");

    // Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnowRecvCb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnowSendCb));

    // Add pre-configured peer if valid
    if (!isZeroMac(UI_BOARD_MAC)) {
        tryAddEspNowPeer(UI_BOARD_MAC);
        setActiveUiMac(UI_BOARD_MAC);
        ESP_LOGI(TAG, "Pre-configured UI board MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 UI_BOARD_MAC[0], UI_BOARD_MAC[1], UI_BOARD_MAC[2],
                 UI_BOARD_MAC[3], UI_BOARD_MAC[4], UI_BOARD_MAC[5]);
    }

    // Add any previously paired peers as ESP-NOW peers
    for (size_t i = 0; i < MAX_APPROVED_PEERS; ++i) {
        const auto& peer = s_security.approved_peers[i];
        if (peer.valid && !isZeroMac(peer.mac)) {
            tryAddEspNowPeer(peer.mac);
            ESP_LOGI(TAG, "Restored paired peer: %02X:%02X:%02X:%02X:%02X:%02X (%s)",
                     peer.mac[0], peer.mac[1], peer.mac[2],
                     peer.mac[3], peer.mac[4], peer.mac[5], peer.name);
        }
    }

    s_raw_recv_queue = xQueueCreate(10, sizeof(RawMsg));
    xTaskCreate(recvTask, "espnow_recv_task", 4096, nullptr, 5, nullptr);

    ESP_LOGI(TAG, "ESP-NOW receiver initialized (protocol v%u, device_id=%u)",
             ESPNOW_PROTOCOL_VERSION, DEVICE_ID_FATIGUE_TESTER);
    ESP_LOGI(TAG, "Approved peers: %zu", PeerStore::GetPeerCount(s_security));
    
    return true;
}

// ============================================================================
// PACKET SEND HELPERS
// ============================================================================

static bool sendPacketTo(const uint8_t* dst_mac, MsgType type, const void* payload, uint8_t payload_len)
{
    if (payload_len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload too big: %d", payload_len);
        return false;
    }

    if (dst_mac == nullptr || isZeroMac(dst_mac)) {
        ESP_LOGW(TAG, "Invalid destination MAC");
        return false;
    }

    EspNowPacket pkt{};
    pkt.hdr.sync = ESPNOW_SYNC_BYTE;
    pkt.hdr.version = ESPNOW_PROTOCOL_VERSION;
    pkt.hdr.device_id = DEVICE_ID_FATIGUE_TESTER;
    pkt.hdr.type = static_cast<uint8_t>(type);
    pkt.hdr.id = s_next_msg_id++;
    pkt.hdr.len = payload_len;

    if (payload_len && payload) {
        std::memcpy(pkt.payload, payload, payload_len);
    }

    // Calculate CRC over header + payload
    size_t crc_len = sizeof(pkt.hdr) + payload_len;
    uint16_t crc = Crc16Ccitt(reinterpret_cast<const uint8_t*>(&pkt.hdr), crc_len);
    
    // Build linear buffer: header + payload + CRC
    uint8_t send_buf[sizeof(EspNowHeader) + ESPNOW_MAX_PAYLOAD + sizeof(uint16_t)];
    std::memcpy(send_buf, &pkt.hdr, sizeof(pkt.hdr));
    if (payload_len > 0) {
        std::memcpy(send_buf + sizeof(pkt.hdr), pkt.payload, payload_len);
    }
    std::memcpy(send_buf + sizeof(pkt.hdr) + payload_len, &crc, sizeof(uint16_t));
    
    size_t total_len = crc_len + sizeof(uint16_t);
    esp_err_t err = esp_now_send(dst_mac, send_buf, total_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send error: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGD(TAG, "TX: type=%u, id=%u, payload_len=%u, total=%zu",
             static_cast<unsigned>(type), pkt.hdr.id, payload_len, total_len);
    return true;
}

static bool sendPacketToUi(MsgType type, const void* payload, uint8_t payload_len)
{
    const uint8_t* dst_mac = nullptr;
    
    if (!isZeroMac(s_active_ui_board_mac)) {
        dst_mac = s_active_ui_board_mac;
    }

    if (dst_mac == nullptr) {
        ESP_LOGW(TAG, "UI board MAC not known yet, cannot send");
        return false;
    }

    return sendPacketTo(dst_mac, type, payload, payload_len);
}

// ============================================================================
// PUBLIC SEND FUNCTIONS
// ============================================================================

bool EspNowReceiver::send_config_response(const Settings& s)
{
    ConfigPayload p{};
    p.cycle_amount = s.test_unit.cycle_amount;
    p.oscillation_vmax_rpm = s.test_unit.oscillation_vmax_rpm;
    p.oscillation_amax_rev_s2 = s.test_unit.oscillation_amax_rev_s2;
    p.dwell_time_ms = s.test_unit.dwell_time_ms;
    p.bounds_method = s.test_unit.bounds_method_stallguard ? 0 : 1;
    p.bounds_search_velocity_rpm = s.test_unit.bounds_search_velocity_rpm;
    p.stallguard_min_velocity_rpm = s.test_unit.stallguard_min_velocity_rpm;
    p.stall_detection_current_factor = s.test_unit.stall_detection_current_factor;
    p.bounds_search_accel_rev_s2 = s.test_unit.bounds_search_accel_rev_s2;
    p.stallguard_sgt = s.test_unit.stallguard_sgt;
    return sendPacketToUi(MsgType::ConfigResponse, &p, sizeof(p));
}

bool EspNowReceiver::send_config_ack(bool ok, uint8_t err_code)
{
    ConfigAckPayload p{};
    p.ok = ok ? 1 : 0;
    p.err_code = err_code;
    return sendPacketToUi(MsgType::ConfigAck, &p, sizeof(p));
}

bool EspNowReceiver::send_start_ack()
{
    return sendPacketToUi(MsgType::CommandAck, nullptr, 0);
}

bool EspNowReceiver::send_pause_ack()
{
    return sendPacketToUi(MsgType::CommandAck, nullptr, 0);
}

bool EspNowReceiver::send_resume_ack()
{
    return sendPacketToUi(MsgType::CommandAck, nullptr, 0);
}

bool EspNowReceiver::send_stop_ack()
{
    return sendPacketToUi(MsgType::CommandAck, nullptr, 0);
}

bool EspNowReceiver::send_status_update(uint32_t cycle, TestState state, uint8_t err_code, uint8_t bounds_valid)
{
    StatusPayload p{};
    p.cycle_number = cycle;
    p.state = static_cast<uint8_t>(state);
    p.err_code = err_code;
    p.bounds_valid = bounds_valid;
    return sendPacketToUi(MsgType::StatusUpdate, &p, sizeof(p));
}

bool EspNowReceiver::send_error(uint8_t err_code, uint32_t at_cycle)
{
    ErrorPayload p{};
    p.err_code = err_code;
    p.at_cycle = at_cycle;
    return sendPacketToUi(MsgType::Error, &p, sizeof(p));
}

bool EspNowReceiver::send_test_complete()
{
    return sendPacketToUi(MsgType::TestComplete, nullptr, 0);
}

bool EspNowReceiver::send_bounds_result(uint8_t ok,
                                        uint8_t bounded,
                                        uint8_t cancelled,
                                        float min_deg_from_center,
                                        float max_deg_from_center,
                                        float global_min_deg,
                                        float global_max_deg)
{
    BoundsResultPayload p{};
    p.ok = ok;
    p.bounded = bounded;
    p.cancelled = cancelled;
    p.reserved = 0;
    p.min_degrees_from_center = min_deg_from_center;
    p.max_degrees_from_center = max_deg_from_center;
    p.global_min_degrees = global_min_deg;
    p.global_max_degrees = global_max_deg;
    return sendPacketToUi(MsgType::BoundsResult, &p, sizeof(p));
}

// ============================================================================
// PAIRING FUNCTIONS
// ============================================================================

void EspNowReceiver::enter_pairing_mode(uint32_t timeout_sec)
{
    s_pairing_mode = true;
    s_pairing_timeout_tick = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_sec * 1000);
    s_awaiting_pairing_confirm = false;
    
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║ PAIRING MODE ENABLED for %lu seconds                                          ║", timeout_sec);
    ESP_LOGI(TAG, "║ Waiting for remote controller pairing request...                              ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════════════════════════════╝");
}

void EspNowReceiver::exit_pairing_mode()
{
    if (s_pairing_mode) {
        s_pairing_mode = false;
        s_awaiting_pairing_confirm = false;
        ESP_LOGI(TAG, "Pairing mode disabled");
    }
}

bool EspNowReceiver::is_in_pairing_mode()
{
    if (!s_pairing_mode) return false;
    
    if (xTaskGetTickCount() > s_pairing_timeout_tick) {
        s_pairing_mode = false;
        s_awaiting_pairing_confirm = false;
        ESP_LOGI(TAG, "Pairing mode timed out");
        return false;
    }
    return true;
}

SecuritySettings& EspNowReceiver::get_security_settings()
{
    return s_security;
}

bool EspNowReceiver::add_approved_peer(const uint8_t mac[6], DeviceType type, const char* name)
{
    bool result = PeerStore::AddPeer(s_security, mac, type, name);
    if (result) {
        tryAddEspNowPeer(mac);
    }
    return result;
}

bool EspNowReceiver::remove_approved_peer(const uint8_t mac[6])
{
    return PeerStore::RemovePeer(s_security, mac);
}

size_t EspNowReceiver::get_approved_peer_count()
{
    return PeerStore::GetPeerCount(s_security);
}

// ============================================================================
// PAIRING MESSAGE HANDLERS
// ============================================================================

static void handlePairingRequest(const uint8_t* src_mac, const EspNowPacket& pkt)
{
    if (pkt.hdr.len < sizeof(PairingRequestPayload)) {
        ESP_LOGW(TAG, "PairingRequest too short: %u bytes", pkt.hdr.len);
        return;
    }

    PairingRequestPayload req;
    std::memcpy(&req, pkt.payload, sizeof(req));

    ESP_LOGI(TAG, "Received pairing request from %02X:%02X:%02X:%02X:%02X:%02X",
             src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);

    // SECURITY CHECK 1: Must be in pairing mode
    if (!EspNowReceiver::is_in_pairing_mode()) {
        ESP_LOGW(TAG, "Rejected pairing: not in pairing mode");
        
        PairingRejectPayload reject{};
        esp_wifi_get_mac(WIFI_IF_STA, reject.rejecter_mac);
        reject.reason = static_cast<uint8_t>(PairingRejectReason::NotInPairingMode);
        
        tryAddEspNowPeer(src_mac);
        sendPacketTo(src_mac, MsgType::PairingReject, &reject, sizeof(reject));
        return;
    }

    // SECURITY CHECK 2: Verify they're looking for our device type
    if (req.expected_peer_type != static_cast<uint8_t>(DeviceType::FatigueTester)) {
        ESP_LOGW(TAG, "Rejected pairing: looking for type %u, I am FatigueTester",
                 req.expected_peer_type);
        
        PairingRejectPayload reject{};
        esp_wifi_get_mac(WIFI_IF_STA, reject.rejecter_mac);
        reject.reason = static_cast<uint8_t>(PairingRejectReason::WrongDeviceType);
        
        tryAddEspNowPeer(src_mac);
        sendPacketTo(src_mac, MsgType::PairingReject, &reject, sizeof(reject));
        return;
    }

    // SECURITY CHECK 3: Verify they claim to be a remote controller
    if (req.device_type != static_cast<uint8_t>(DeviceType::RemoteController)) {
        ESP_LOGW(TAG, "Rejected pairing: requester claims type %u, expected RemoteController",
                 req.device_type);
        
        PairingRejectPayload reject{};
        esp_wifi_get_mac(WIFI_IF_STA, reject.rejecter_mac);
        reject.reason = static_cast<uint8_t>(PairingRejectReason::WrongDeviceType);
        
        tryAddEspNowPeer(src_mac);
        sendPacketTo(src_mac, MsgType::PairingReject, &reject, sizeof(reject));
        return;
    }

    // SECURITY CHECK 4: Protocol version
    if (req.protocol_version != ESPNOW_PROTOCOL_VERSION) {
        ESP_LOGW(TAG, "Rejected pairing: protocol version %u, expected %u",
                 req.protocol_version, ESPNOW_PROTOCOL_VERSION);
        
        PairingRejectPayload reject{};
        esp_wifi_get_mac(WIFI_IF_STA, reject.rejecter_mac);
        reject.reason = static_cast<uint8_t>(PairingRejectReason::ProtocolMismatch);
        
        tryAddEspNowPeer(src_mac);
        sendPacketTo(src_mac, MsgType::PairingReject, &reject, sizeof(reject));
        return;
    }

    // Store pending pairing state
    std::memcpy(s_pending_requester_mac, req.requester_mac, 6);
    
    // Generate our challenge for mutual authentication
    GenerateChallenge(s_my_challenge);

    // Compute HMAC response to prove we know the secret
    uint8_t hmac_response[HMAC_SIZE];
    ComputePairingHmac(req.challenge, CHALLENGE_SIZE, hmac_response);

    // Build response
    PairingResponsePayload resp{};
    esp_wifi_get_mac(WIFI_IF_STA, resp.responder_mac);
    resp.device_type = static_cast<uint8_t>(DeviceType::FatigueTester);
    std::memcpy(resp.challenge, s_my_challenge, CHALLENGE_SIZE);
    std::memcpy(resp.hmac_response, hmac_response, HMAC_SIZE);
    strncpy(resp.device_name, "Fatigue Tester", sizeof(resp.device_name) - 1);

    // Add requester as ESP-NOW peer for sending response
    tryAddEspNowPeer(req.requester_mac);

    // Send response
    if (sendPacketTo(req.requester_mac, MsgType::PairingResponse, &resp, sizeof(resp))) {
        s_awaiting_pairing_confirm = true;
        s_pairing_confirm_timeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
        ESP_LOGI(TAG, "Sent pairing response, awaiting confirm...");
    } else {
        ESP_LOGE(TAG, "Failed to send pairing response");
    }
}

static void handlePairingConfirm(const uint8_t* src_mac, const EspNowPacket& pkt)
{
    if (!s_awaiting_pairing_confirm) {
        ESP_LOGW(TAG, "Unexpected PairingConfirm (not awaiting)");
        return;
    }

    if (!macEq(src_mac, s_pending_requester_mac)) {
        ESP_LOGW(TAG, "PairingConfirm from unexpected MAC");
        return;
    }

    if (xTaskGetTickCount() > s_pairing_confirm_timeout) {
        ESP_LOGW(TAG, "PairingConfirm timed out");
        s_awaiting_pairing_confirm = false;
        return;
    }

    if (pkt.hdr.len < sizeof(PairingConfirmPayload)) {
        ESP_LOGW(TAG, "PairingConfirm too short: %u bytes", pkt.hdr.len);
        s_awaiting_pairing_confirm = false;
        return;
    }

    PairingConfirmPayload confirm;
    std::memcpy(&confirm, pkt.payload, sizeof(confirm));

    // SECURITY CHECK: Verify their HMAC (mutual authentication)
    if (!VerifyPairingHmac(s_my_challenge, CHALLENGE_SIZE, confirm.hmac_response)) {
        ESP_LOGE(TAG, "Pairing HMAC verification FAILED - unauthorized device!");
        s_awaiting_pairing_confirm = false;
        s_pairing_mode = false;
        
        // Don't add as approved peer
        return;
    }

    if (!confirm.success) {
        ESP_LOGW(TAG, "Remote controller rejected pairing");
        s_awaiting_pairing_confirm = false;
        return;
    }

    // SUCCESS! Add to approved peers
    bool added = PeerStore::AddPeer(s_security, src_mac, 
                                    DeviceType::RemoteController, "Remote Ctrl");
    
    s_awaiting_pairing_confirm = false;
    s_pairing_mode = false;

    if (added) {
        setActiveUiMac(src_mac);
        
        ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║ PAIRING SUCCESSFUL!                                                           ║");
        ESP_LOGI(TAG, "║ Remote controller: %02X:%02X:%02X:%02X:%02X:%02X                                        ║",
                 src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
        ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════════════════════════════╝");

        // Notify application layer
        if (s_proto_event_queue) {
            ProtoEvent ev{};
            ev.type = ProtoEventType::PairingComplete;
            std::memcpy(ev.data.pairing.peer_mac, src_mac, 6);
            ev.data.pairing.device_type = static_cast<uint8_t>(DeviceType::RemoteController);
            strncpy(ev.data.pairing.device_name, "Remote Ctrl", sizeof(ev.data.pairing.device_name) - 1);
            xQueueSend(s_proto_event_queue, &ev, 0);
        }
    } else {
        ESP_LOGE(TAG, "Failed to add peer to approved list (full?)");
    }
}

// ============================================================================
// CALLBACKS & RECEIVE TASK
// ============================================================================

static void espnowSendCb(const wifi_tx_info_t* info, esp_now_send_status_t status)
{
    (void)info;
    ESP_LOGD(TAG, "ESP-NOW send status=%s",
             status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

static void espnowRecvCb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    // Minimum packet size: header (6 bytes) + CRC (2 bytes) = 8 bytes
    if (len < 8 || len > static_cast<int>(sizeof(EspNowPacket))) {
        ESP_LOGW(TAG, "RX callback: Invalid length %d", len);
        return;
    }

    RawMsg msg{};
    msg.len = len;
    std::memcpy(msg.data, data, len);
    std::memcpy(msg.src_mac, info->src_addr, 6);

    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_raw_recv_queue, &msg, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

static void handlePacket(const RawMsg& msg, const EspNowPacket& pkt, uint16_t recv_crc)
{
    // Validate header
    if (pkt.hdr.sync != ESPNOW_SYNC_BYTE) {
        ESP_LOGW(TAG, "Bad SYNC 0x%02X", pkt.hdr.sync);
        return;
    }
    if (pkt.hdr.version != ESPNOW_PROTOCOL_VERSION) {
        ESP_LOGW(TAG, "Unsupported protocol version: %d (expected %d)",
                 pkt.hdr.version, ESPNOW_PROTOCOL_VERSION);
        return;
    }
    if (pkt.hdr.len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "Bad LEN %d", pkt.hdr.len);
        return;
    }

    // Verify CRC
    size_t crc_len = sizeof(pkt.hdr) + pkt.hdr.len;
    uint16_t calc = Crc16Ccitt(reinterpret_cast<const uint8_t*>(&pkt.hdr), crc_len);
    if (calc != recv_crc) {
        ESP_LOGW(TAG, "CRC mismatch (calc=0x%04X recv=0x%04X)", calc, recv_crc);
        return;
    }

    MsgType type = static_cast<MsgType>(pkt.hdr.type);

    ESP_LOGD(TAG, "RX valid: type=%u from %02X:%02X:%02X:%02X:%02X:%02X",
             pkt.hdr.type, msg.src_mac[0], msg.src_mac[1], msg.src_mac[2],
             msg.src_mac[3], msg.src_mac[4], msg.src_mac[5]);

    // Handle pairing messages (exempt from peer validation)
    if (type == MsgType::PairingRequest) {
        handlePairingRequest(msg.src_mac, pkt);
        return;
    }
    if (type == MsgType::PairingConfirm) {
        handlePairingConfirm(msg.src_mac, pkt);
        return;
    }

    // SECURITY GATE: All other messages must come from approved peers
    if (!PeerStore::IsPeerApproved(s_security, msg.src_mac)) {
        ESP_LOGW(TAG, "Rejected message from unapproved peer: %02X:%02X:%02X:%02X:%02X:%02X",
                 msg.src_mac[0], msg.src_mac[1], msg.src_mac[2],
                 msg.src_mac[3], msg.src_mac[4], msg.src_mac[5]);
        return;
    }

    // Update active UI MAC for replies (from approved sender)
    setActiveUiMac(msg.src_mac);
    tryAddEspNowPeer(msg.src_mac);

    // Process regular messages
    ProtoEvent ev{};

    switch (type) {
        case MsgType::ConfigRequest:
            ev.type = ProtoEventType::ConfigRequest;
            break;

        case MsgType::ConfigSet:
            {
                // PROTOCOL V2 layout:
                // Bytes 0-3: cycle_amount (4)
                // Bytes 4-7: oscillation_vmax_rpm (4)
                // Bytes 8-11: oscillation_amax_rev_s2 (4)
                // Bytes 12-15: dwell_time_ms (4)
                // Byte 16: bounds_method (1)
                // -- Base size = 17 --
                // Bytes 17-20: bounds_search_velocity_rpm (4)
                // Bytes 21-24: stallguard_min_velocity_rpm (4)
                // Bytes 25-28: stall_detection_current_factor (4)
                // Bytes 29-32: bounds_search_accel_rev_s2 (4)
                // -- Extended V1 size = 33 --
                // Byte 33: stallguard_sgt (1)
                // -- Extended V2 size = 34 --
                constexpr size_t kBaseSize = 17;
                constexpr size_t kExtendedV1Size = 17 + (4 * 4); // 33 bytes
                constexpr size_t kExtendedV2Size = kExtendedV1Size + 1; // 34 bytes

                if (pkt.hdr.len >= kBaseSize) {
                    ev.type = ProtoEventType::ConfigSet;
                    
                    // Base fields
                    std::memcpy(&ev.data.config.cycle_amount, pkt.payload + 0, 4);
                    std::memcpy(&ev.data.config.oscillation_vmax_rpm, pkt.payload + 4, 4);
                    std::memcpy(&ev.data.config.oscillation_amax_rev_s2, pkt.payload + 8, 4);
                    std::memcpy(&ev.data.config.dwell_time_ms, pkt.payload + 12, 4);
                    ev.data.config.bounds_method_stallguard = (pkt.payload[16] == 0);

                    // Default extended fields
                    ev.data.config.bounds_search_velocity_rpm = 0.0f;
                    ev.data.config.stallguard_min_velocity_rpm = 0.0f;
                    ev.data.config.stall_detection_current_factor = 0.0f;
                    ev.data.config.bounds_search_accel_rev_s2 = 0.0f;
                    ev.data.config.stallguard_sgt = 127;

                    // Extended V1 fields (bounds finding config)
                    if (pkt.hdr.len >= kExtendedV1Size) {
                        std::memcpy(&ev.data.config.bounds_search_velocity_rpm, pkt.payload + 17, 4);
                        std::memcpy(&ev.data.config.stallguard_min_velocity_rpm, pkt.payload + 21, 4);
                        std::memcpy(&ev.data.config.stall_detection_current_factor, pkt.payload + 25, 4);
                        std::memcpy(&ev.data.config.bounds_search_accel_rev_s2, pkt.payload + 29, 4);
                    }

                    // Extended V2 field (SGT)
                    if (pkt.hdr.len >= kExtendedV2Size) {
                        std::memcpy(&ev.data.config.stallguard_sgt, pkt.payload + 33, 1);
                    }
                } else {
                    ESP_LOGW(TAG, "CONFIG_SET payload too short: %u bytes (need >= %zu)", pkt.hdr.len, kBaseSize);
                    return;
                }
            }
            break;

        case MsgType::Command:
            if (pkt.hdr.len >= 1) {
                uint8_t cmd_id = pkt.payload[0];
                switch (static_cast<CommandId>(cmd_id)) {
                    case CommandId::Start:
                        ev.type = ProtoEventType::CommandStart;
                        break;
                    case CommandId::Pause:
                        ev.type = ProtoEventType::CommandPause;
                        break;
                    case CommandId::Resume:
                        ev.type = ProtoEventType::CommandResume;
                        break;
                    case CommandId::Stop:
                        ev.type = ProtoEventType::CommandStop;
            break;
                    case CommandId::RunBoundsFinding:
                        ev.type = ProtoEventType::CommandRunBoundsFinding;
            break;
                    default:
                        ESP_LOGW(TAG, "Unknown command ID: %u", cmd_id);
                        return;
        }
            } else {
                ESP_LOGW(TAG, "COMMAND without payload");
                return;
            }
            break;

        default:
            ESP_LOGW(TAG, "Unhandled msg type %u", static_cast<unsigned>(pkt.hdr.type));
            return;
    }

    if (s_proto_event_queue) {
        xQueueSend(s_proto_event_queue, &ev, 0);
    }
}

static void recvTask(void* arg)
{
    (void)arg;
    RawMsg msg{};
    
    while (true) {
        if (xQueueReceive(s_raw_recv_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.len < static_cast<int>(sizeof(EspNowHeader) + 2)) {
                ESP_LOGW(TAG, "Packet too short: %d bytes", msg.len);
                continue;
            }
            
            // Parse header
            EspNowHeader hdr{};
            std::memcpy(&hdr, msg.data, sizeof(hdr));
            
            // Verify length
            size_t expected_len = sizeof(hdr) + hdr.len + sizeof(uint16_t);
            if (msg.len < static_cast<int>(expected_len)) {
                ESP_LOGW(TAG, "Packet too short: got %d, need %zu", msg.len, expected_len);
                continue;
            }
            
            // Build packet structure
            EspNowPacket pkt{};
            std::memcpy(&pkt.hdr, msg.data, sizeof(hdr));
            if (hdr.len > 0) {
                std::memcpy(pkt.payload, msg.data + sizeof(hdr), hdr.len);
            }
            
            // Extract CRC from correct offset
            size_t crc_offset = sizeof(hdr) + hdr.len;
            uint16_t recv_crc = 0;
            std::memcpy(&recv_crc, msg.data + crc_offset, sizeof(uint16_t));
            
            handlePacket(msg, pkt, recv_crc);
        }
    }
}
