/**
 * @file espnow_receiver.cpp
 * @brief ESP-NOW receiver implementation for test unit
 * 
 * Protocol compatible with esp32_remote_controller (6-byte header).
 */

#include "espnow_receiver.hpp"
#include "espnow_protocol.hpp"
#include <cstring>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_idf_pedantic_compat.hpp"
#include "esp_log.h"

static const char* TAG = "EspNowRx";

static QueueHandle_t s_proto_event_queue = nullptr;
static uint8_t s_next_msg_id = 1;
static uint8_t s_ui_board_mac[6] = {0};

// Receive queue from ISR to this module
static QueueHandle_t s_raw_recv_queue = nullptr;

struct RawMsg {
    uint8_t data[sizeof(EspNowPacket)];
    int     len;
    uint8_t src_mac[6];
};

// Forward declarations
static void espnowRecvCb(const esp_now_recv_info_t* info, const uint8_t* data, int len);
static void espnowSendCb(const wifi_tx_info_t* info, esp_now_send_status_t status);
static void recvTask(void*);

// -------- ESPNOW INIT --------

bool EspNowReceiver::init(QueueHandle_t event_queue)
{
    s_proto_event_queue = event_queue;

    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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
    ESP_LOGI(TAG, "ESP-NOW Device MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "Use this MAC address to configure the remote controller for pairing");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════════════════════");

    // Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnowRecvCb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnowSendCb));

    // Pre-configure the UI board MAC (if provided) so we can send immediately.
    // If the UI board MAC is left as all zeros, we fall back to learning it from
    // the first received ESPNOW packet.
    bool ui_mac_configured = false;
    {
        uint8_t zero[6] = {0};
        if (std::memcmp(UI_BOARD_MAC, zero, 6) != 0) {
            std::memcpy(s_ui_board_mac, UI_BOARD_MAC, 6);
            ui_mac_configured = true;
            ESP_LOGI(TAG, "Using configured UI board MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     s_ui_board_mac[0], s_ui_board_mac[1], s_ui_board_mac[2],
                     s_ui_board_mac[3], s_ui_board_mac[4], s_ui_board_mac[5]);

            esp_now_peer_info_t peer{};
            std::memset(&peer, 0, sizeof(peer));
            std::memcpy(peer.peer_addr, s_ui_board_mac, 6);
            peer.ifidx = WIFI_IF_STA;
            peer.channel = WIFI_CHANNEL;
            peer.encrypt = false;
            esp_err_t add_err = esp_now_add_peer(&peer);
            if (add_err != ESP_OK && add_err != ESP_ERR_ESPNOW_EXIST) {
                ESP_LOGW(TAG, "Failed to add UI board peer (err=%s). Will still try to learn via RX.",
                         esp_err_to_name(add_err));
                // Clear so send path falls back to learning.
                std::memset(s_ui_board_mac, 0, sizeof(s_ui_board_mac));
                ui_mac_configured = false;
            }
        }
    }

    s_raw_recv_queue = xQueueCreate(10, sizeof(RawMsg));
    xTaskCreate(recvTask, "espnow_recv_task", 4096, nullptr, 5, nullptr);

    ESP_LOGI(TAG, "ESP-NOW receiver initialized (protocol v%u, device_id=%u)",
             ESPNOW_PROTOCOL_VERSION, DEVICE_ID_FATIGUE_TESTER);
    if (!ui_mac_configured) {
        ESP_LOGI(TAG, "UI board MAC not pre-configured; will learn from first received packet.");
    }
    return true;
}

// -------- PACKET BUILD / SEND HELPERS --------

static bool sendPacketToUi(MsgType type, const void* payload, uint8_t payload_len)
{
    if (payload_len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload too big: %d", payload_len);
        return false;
    }

    if (s_ui_board_mac[0] == 0 && s_ui_board_mac[1] == 0) {
        ESP_LOGW(TAG, "UI board MAC not known yet, cannot send");
        return false;
    }

    EspNowPacket pkt{};
    pkt.hdr.sync = ESPNOW_SYNC_BYTE;
    pkt.hdr.version = ESPNOW_PROTOCOL_VERSION;
    pkt.hdr.device_id = DEVICE_ID_FATIGUE_TESTER;
    pkt.hdr.type = static_cast<uint8_t>(type);
    pkt.hdr.id   = s_next_msg_id++;
    pkt.hdr.len  = payload_len;

    if (payload_len && payload) {
        std::memcpy(pkt.payload, payload, payload_len);
    }

    // Calculate CRC over header + payload (NOT including CRC field itself)
    size_t crc_len = sizeof(pkt.hdr) + payload_len;
    uint16_t crc = Crc16Ccitt(reinterpret_cast<const uint8_t*>(&pkt.hdr), crc_len);

    size_t total_len = crc_len + sizeof(uint16_t);
    
    // Construct packet buffer: header + payload + CRC
    // We can't send directly from &pkt because the CRC field is at the wrong offset in the structure
    uint8_t send_buf[sizeof(EspNowHeader) + ESPNOW_MAX_PAYLOAD + sizeof(uint16_t)];
    std::memcpy(send_buf, &pkt.hdr, sizeof(pkt.hdr));
    if (payload_len > 0) {
        std::memcpy(send_buf + sizeof(pkt.hdr), pkt.payload, payload_len);
    }
    std::memcpy(send_buf + sizeof(pkt.hdr) + payload_len, &crc, sizeof(uint16_t));
    
    esp_err_t err = esp_now_send(s_ui_board_mac, send_buf, total_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send error: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGD(TAG, "TX: type=%u, id=%u, payload_len=%u, total=%zu",
             static_cast<unsigned>(type), pkt.hdr.id, payload_len, total_len);
    return true;
}

// -------- PUBLIC SEND FUNCTIONS --------

bool EspNowReceiver::send_config_response(const Settings& s)
{
    ConfigPayload p{};
    p.cycle_amount       = s.test_unit.cycle_amount;
    p.time_per_cycle_sec = s.test_unit.time_per_cycle;
    p.dwell_time_sec     = s.test_unit.dwell_time;
    p.bounds_method      = s.test_unit.bounds_method_stallguard ? 0 : 1;
    p.bounds_search_velocity_rpm = s.test_unit.bounds_search_velocity_rpm;
    p.stallguard_min_velocity_rpm = s.test_unit.stallguard_min_velocity_rpm;
    p.stall_detection_current_factor = s.test_unit.stall_detection_current_factor;
    p.bounds_search_accel_rev_s2 = s.test_unit.bounds_search_accel_rev_s2;
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
    // Send COMMAND_ACK (no payload needed, or could echo the command_id)
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

bool EspNowReceiver::send_status_update(uint32_t cycle, TestState state, uint8_t err_code)
{
    StatusPayload p{};
    p.cycle_number = cycle;
    p.state = static_cast<uint8_t>(state);
    p.err_code = err_code;
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

// -------- CALLBACKS & RECV TASK --------

static void espnowSendCb(const wifi_tx_info_t* info, esp_now_send_status_t status)
{
    (void)info; // Unused in ESP-IDF v5.5
    ESP_LOGD(TAG, "ESP-NOW send status=%s",
             status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

static void espnowRecvCb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    // Minimum packet size: header (6 bytes) + CRC (2 bytes) = 8 bytes
    if (len < 8 || len > static_cast<int>(sizeof(EspNowPacket))) {
        ESP_LOGW(TAG, "RX callback: Invalid length %d (min=8, max=%zu)", len, sizeof(EspNowPacket));
        return;
    }

    // Store UI board MAC on first message
    if (s_ui_board_mac[0] == 0 && s_ui_board_mac[1] == 0) {
        std::memcpy(s_ui_board_mac, info->src_addr, 6);
        ESP_LOGI(TAG, "UI board MAC learned: %02X:%02X:%02X:%02X:%02X:%02X",
                 s_ui_board_mac[0], s_ui_board_mac[1], s_ui_board_mac[2],
                 s_ui_board_mac[3], s_ui_board_mac[4], s_ui_board_mac[5]);
        
        // Add as peer for sending responses
        esp_now_peer_info_t peer{};
        std::memset(&peer, 0, sizeof(peer));
        std::memcpy(peer.peer_addr, s_ui_board_mac, 6);
        peer.ifidx = WIFI_IF_STA;
        peer.channel = WIFI_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    RawMsg msg{};
    msg.len = len;
    std::memcpy(msg.data, data, len);
    std::memcpy(msg.src_mac, info->src_addr, 6);

    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_raw_recv_queue, &msg, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

static void handlePacket(const EspNowPacket& pkt, uint16_t recv_crc_from_data)
{
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
    // Calculate CRC over header + payload (NOT including CRC field itself)
    size_t crc_len = sizeof(pkt.hdr) + pkt.hdr.len;
    uint16_t calc = Crc16Ccitt(reinterpret_cast<const uint8_t*>(&pkt.hdr), crc_len);
    
    if (calc != recv_crc_from_data) {
        ESP_LOGW(TAG, "CRC mismatch (calc=0x%04X recv=0x%04X) - type=%u, id=%u, payload_len=%u", 
                 calc, recv_crc_from_data, pkt.hdr.type, pkt.hdr.id, pkt.hdr.len);
        return;
    }

    MsgType type = static_cast<MsgType>(pkt.hdr.type);
    ProtoEvent ev{};

    ESP_LOGD(TAG, "RX valid: type=%u, device_id=%u, id=%u, len=%u",
             pkt.hdr.type, pkt.hdr.device_id, pkt.hdr.id, pkt.hdr.len);

    switch (type) {
        case MsgType::ConfigRequest: {
            ev.type = ProtoEventType::ConfigRequest;
            break;
        }
        case MsgType::ConfigSet: {
            if (pkt.hdr.len >= sizeof(ConfigPayload)) {
            ConfigPayload p{};
            std::memcpy(&p, pkt.payload, sizeof(p));
                ev.type = ProtoEventType::ConfigSet;
            ev.data.config.cycle_amount   = p.cycle_amount;
            ev.data.config.time_per_cycle = p.time_per_cycle_sec;
            ev.data.config.dwell_time     = p.dwell_time_sec;
            ev.data.config.bounds_method_stallguard = (p.bounds_method == 0);
            ev.data.config.bounds_search_velocity_rpm = p.bounds_search_velocity_rpm;
            ev.data.config.stallguard_min_velocity_rpm = p.stallguard_min_velocity_rpm;
            ev.data.config.stall_detection_current_factor = p.stall_detection_current_factor;
            ev.data.config.bounds_search_accel_rev_s2 = p.bounds_search_accel_rev_s2;
            } else if (pkt.hdr.len >= 13) {
                // Minimal config from older remote controller (without float fields)
                // Support backward compatibility with remote controllers that don't have extended fields
                ev.type = ProtoEventType::ConfigSet;
                std::memcpy(&ev.data.config.cycle_amount, pkt.payload, 4);
                std::memcpy(&ev.data.config.time_per_cycle, pkt.payload + 4, 4);
                std::memcpy(&ev.data.config.dwell_time, pkt.payload + 8, 4);
                ev.data.config.bounds_method_stallguard = (pkt.payload[12] == 0);
                // Extended fields remain at defaults (0.0f)
                ev.data.config.bounds_search_velocity_rpm = 0.0f;
                ev.data.config.stallguard_min_velocity_rpm = 0.0f;
                ev.data.config.stall_detection_current_factor = 0.0f;
                ev.data.config.bounds_search_accel_rev_s2 = 0.0f;
            } else {
                ESP_LOGW(TAG, "CONFIG_SET payload too short: %u bytes", pkt.hdr.len);
                return;
            }
            break;
        }
        case MsgType::Command: {
            // COMMAND message contains a command_id in the payload
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
                    default:
                        ESP_LOGW(TAG, "Unknown command ID: %u", cmd_id);
                        return;
        }
            } else {
                ESP_LOGW(TAG, "COMMAND without payload");
                return;
            }
            break;
        }
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
                ESP_LOGW(TAG, "Packet too short: %d bytes (min %zu)", msg.len, sizeof(EspNowHeader) + 2);
                continue;
            }
            
            // Parse header first to get payload length
            EspNowHeader hdr{};
            std::memcpy(&hdr, msg.data, sizeof(hdr));
            
            // Verify we have enough data for header + payload + CRC
            size_t expected_len = sizeof(hdr) + hdr.len + sizeof(uint16_t);
            if (msg.len < static_cast<int>(expected_len)) {
                ESP_LOGW(TAG, "Packet too short: got %d, need %zu (hdr=%zu, payload=%u, crc=%zu)", 
                         msg.len, expected_len, sizeof(hdr), hdr.len, sizeof(uint16_t));
                continue;
            }
            
            // Copy header and payload into packet structure
            EspNowPacket pkt{};
            std::memcpy(&pkt.hdr, msg.data, sizeof(hdr));
            if (hdr.len > 0) {
                std::memcpy(pkt.payload, msg.data + sizeof(hdr), hdr.len);
            }
            
            // Read CRC from the correct offset in received data (NOT from structure!)
            // CRC is at offset: sizeof(header) + actual_payload_len
            size_t crc_offset = sizeof(hdr) + hdr.len;
            uint16_t recv_crc = 0;
            std::memcpy(&recv_crc, msg.data + crc_offset, sizeof(uint16_t));
            
            ESP_LOGD(TAG, "RX: msg.len=%d, type=%u, device_id=%u, id=%u, payload_len=%u", 
                     msg.len, hdr.type, hdr.device_id, hdr.id, hdr.len);
            
            handlePacket(pkt, recv_crc);
        }
    }
}
