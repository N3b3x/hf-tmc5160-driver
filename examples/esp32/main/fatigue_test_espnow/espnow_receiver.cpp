/**
 * @file espnow_receiver.cpp
 * @brief ESP-NOW receiver implementation for test unit
 */

#include "espnow_receiver.hpp"
#include "espnow_protocol.hpp"
#include <cstring>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_idf_pedantic_compat.hpp"
#include "esp_log.h"

static const char* TAG = "EspNowRx";

static QueueHandle_t s_protoEventQueue = nullptr;
static uint8_t s_nextMsgId = 1;
static uint8_t s_uiBoardMac[6] = {0};

// recv queue from ISR to this module
static QueueHandle_t s_rawRecvQueue = nullptr;

struct RawMsg {
    uint8_t data[sizeof(EspNowPacket)];
    int     len;
    uint8_t src_mac[6];
};

// Forward declarations
static void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len);
static void espnow_send_cb(const wifi_tx_info_t* info, esp_now_send_status_t status);
static void recv_task(void*);

// -------- ESPNOW INIT --------

bool EspNowReceiver::init(QueueHandle_t event_queue)
{
    s_protoEventQueue = event_queue;

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
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // Pre-configure the UI board MAC (if provided) so we can send immediately.
    // If the UI board MAC is left as all zeros, we fall back to learning it from
    // the first received ESPNOW packet.
    bool ui_mac_configured = false;
    {
        uint8_t zero[6] = {0};
        if (std::memcmp(UI_BOARD_MAC_, zero, 6) != 0) {
            std::memcpy(s_uiBoardMac, UI_BOARD_MAC_, 6);
            ui_mac_configured = true;
            ESP_LOGI(TAG, "Using configured UI board MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     s_uiBoardMac[0], s_uiBoardMac[1], s_uiBoardMac[2],
                     s_uiBoardMac[3], s_uiBoardMac[4], s_uiBoardMac[5]);

            esp_now_peer_info_t peer{};
            std::memset(&peer, 0, sizeof(peer));
            std::memcpy(peer.peer_addr, s_uiBoardMac, 6);
            peer.ifidx = WIFI_IF_STA;
            peer.channel = WIFI_CHANNEL;
            peer.encrypt = false;
            esp_err_t add_err = esp_now_add_peer(&peer);
            if (add_err != ESP_OK && add_err != ESP_ERR_ESPNOW_EXIST) {
                ESP_LOGW(TAG, "Failed to add UI board peer (err=%s). Will still try to learn via RX.",
                         esp_err_to_name(add_err));
                // Clear so send path falls back to learning.
                std::memset(s_uiBoardMac, 0, sizeof(s_uiBoardMac));
                ui_mac_configured = false;
            }
        }
    }

    s_rawRecvQueue = xQueueCreate(10, sizeof(RawMsg));
    xTaskCreate(recv_task, "espnow_recv_task", 4096, nullptr, 5, nullptr);

    ESP_LOGI(TAG, "ESP-NOW receiver initialized");
    if (!ui_mac_configured) {
        ESP_LOGI(TAG, "UI board MAC not pre-configured; will learn from first received packet.");
    }
    return true;
}

// -------- PACKET BUILD / SEND HELPERS --------

static bool send_packet_to_ui(MsgType type, const void* payload, uint8_t payload_len)
{
    if (payload_len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload too big: %d", payload_len);
        return false;
    }

    if (s_uiBoardMac[0] == 0 && s_uiBoardMac[1] == 0) {
        ESP_LOGW(TAG, "UI board MAC not known yet, cannot send");
        return false;
    }

    EspNowPacket pkt{};
    pkt.hdr.sync = ESPNOW_SYNC_BYTE;
    pkt.hdr.type = static_cast<uint8_t>(type);
    pkt.hdr.id   = s_nextMsgId++;
    pkt.hdr.len  = payload_len;

    if (payload_len && payload) {
        std::memcpy(pkt.payload, payload, payload_len);
    }

    // Calculate CRC over header + payload (NOT including CRC field itself)
    size_t crc_len = sizeof(pkt.hdr) + payload_len;
    uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt.hdr), crc_len);

    size_t total_len = crc_len + sizeof(uint16_t);
    
    // Construct packet buffer: header + payload + CRC
    // We can't send directly from &pkt because the CRC field is at the wrong offset in the structure
    uint8_t send_buf[sizeof(EspNowHeader) + ESPNOW_MAX_PAYLOAD + sizeof(uint16_t)];
    std::memcpy(send_buf, &pkt.hdr, sizeof(pkt.hdr));
    if (payload_len > 0) {
        std::memcpy(send_buf + sizeof(pkt.hdr), pkt.payload, payload_len);
    }
    std::memcpy(send_buf + sizeof(pkt.hdr) + payload_len, &crc, sizeof(uint16_t));
    
    esp_err_t err = esp_now_send(s_uiBoardMac, send_buf, total_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send error: %s", esp_err_to_name(err));
        return false;
    }
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
    return send_packet_to_ui(MsgType::CONFIG_RESPONSE, &p, sizeof(p));
}

bool EspNowReceiver::send_config_ack(bool ok, uint8_t err_code)
{
    ConfigAckPayload p{};
    p.ok = ok ? 1 : 0;
    p.err_code = err_code;
    return send_packet_to_ui(MsgType::CONFIG_ACK, &p, sizeof(p));
}

bool EspNowReceiver::send_start_ack()
{
    return send_packet_to_ui(MsgType::START_ACK, nullptr, 0);
}

bool EspNowReceiver::send_pause_ack()
{
    return send_packet_to_ui(MsgType::PAUSE_ACK, nullptr, 0);
}

bool EspNowReceiver::send_resume_ack()
{
    return send_packet_to_ui(MsgType::RESUME_ACK, nullptr, 0);
}

bool EspNowReceiver::send_stop_ack()
{
    return send_packet_to_ui(MsgType::STOP_ACK, nullptr, 0);
}

bool EspNowReceiver::send_status_update(uint32_t cycle, TestState state, uint8_t err_code)
{
    StatusPayload p{};
    p.cycle_number = cycle;
    p.state = static_cast<uint8_t>(state);
    p.err_code = err_code;
    return send_packet_to_ui(MsgType::STATUS_UPDATE, &p, sizeof(p));
}

bool EspNowReceiver::send_error(uint8_t err_code, uint32_t at_cycle)
{
    ErrorPayload p{};
    p.err_code = err_code;
    p.at_cycle = at_cycle;
    return send_packet_to_ui(MsgType::ERROR, &p, sizeof(p));
}

bool EspNowReceiver::send_test_complete()
{
    return send_packet_to_ui(MsgType::TEST_COMPLETE, nullptr, 0);
}

// -------- CALLBACKS & RECV TASK --------

static void espnow_send_cb(const wifi_tx_info_t* info, esp_now_send_status_t status)
{
    (void)info; // Unused in ESP-IDF v5.5
    ESP_LOGD(TAG, "ESP-NOW send status=%s",
             status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

static void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    // Minimum packet size: header (4 bytes) + CRC (2 bytes) = 6 bytes
    if (len < 6 || len > (int)sizeof(EspNowPacket)) {
        ESP_LOGW(TAG, "RX callback: Invalid length %d (min=6, max=%zu)", len, sizeof(EspNowPacket));
        return;
    }
    
    // Debug: Print raw received data and length
    ESP_LOGW(TAG, "RX callback: len=%d bytes", len);
    ESP_LOGW(TAG, "RX callback: first 8 bytes: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", 
             data[0], data[1], data[2], data[3], data[4], data[5], len > 6 ? data[6] : 0, len > 7 ? data[7] : 0);
    if (len >= 6) {
        ESP_LOGW(TAG, "RX callback: CRC bytes [4]=0x%02X [5]=0x%02X", data[4], data[5]);
    }

    // Store UI board MAC on first message
    if (s_uiBoardMac[0] == 0 && s_uiBoardMac[1] == 0) {
        std::memcpy(s_uiBoardMac, info->src_addr, 6);
        ESP_LOGI(TAG, "UI board MAC learned: %02X:%02X:%02X:%02X:%02X:%02X",
                 s_uiBoardMac[0], s_uiBoardMac[1], s_uiBoardMac[2],
                 s_uiBoardMac[3], s_uiBoardMac[4], s_uiBoardMac[5]);
        
        // Add as peer for sending responses
        esp_now_peer_info_t peer{};
        std::memset(&peer, 0, sizeof(peer));
        std::memcpy(peer.peer_addr, s_uiBoardMac, 6);
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
    xQueueSendFromISR(s_rawRecvQueue, &msg, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

static void handle_packet(const EspNowPacket& pkt, uint16_t recv_crc_from_data)
{
    if (pkt.hdr.sync != ESPNOW_SYNC_BYTE) {
        ESP_LOGW(TAG, "Bad SYNC 0x%02X", pkt.hdr.sync);
        return;
    }
    if (pkt.hdr.len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "Bad LEN %d", pkt.hdr.len);
        return;
    }
    // Calculate CRC over header + payload (NOT including CRC field itself)
    size_t crc_len = sizeof(pkt.hdr) + pkt.hdr.len;
    uint16_t calc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt.hdr), crc_len);
    
    // Debug: Print CRC calculation details
    ESP_LOGD(TAG, "CRC check: calc_len=%zu, calc=0x%04X, recv=0x%04X", crc_len, calc, recv_crc_from_data);
    
    if (calc != recv_crc_from_data) {
        ESP_LOGW(TAG, "CRC mismatch (calc=0x%04X recv=0x%04X) - type=%u, id=%u, payload_len=%u", 
                 calc, recv_crc_from_data, pkt.hdr.type, pkt.hdr.id, pkt.hdr.len);
        // Print raw packet bytes for debugging
        ESP_LOGW(TAG, "Raw packet bytes (first 32):");
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt.hdr);
        for (int i = 0; i < 32 && i < (int)(sizeof(pkt.hdr) + pkt.hdr.len + sizeof(uint16_t)); i++) {
            ESP_LOGW(TAG, "  [%d] = 0x%02X", i, raw[i]);
        }
        return;
    }

    MsgType type = static_cast<MsgType>(pkt.hdr.type);
    ProtoEvent ev{};

    switch (type) {
        case MsgType::CONFIG_REQUEST: {
            ev.type = ProtoEventType::CONFIG_REQUEST;
            break;
        }
        case MsgType::CONFIG_SET: {
            ConfigPayload p{};
            std::memcpy(&p, pkt.payload, sizeof(p));
            ev.type = ProtoEventType::CONFIG_SET;
            ev.data.config.cycle_amount   = p.cycle_amount;
            ev.data.config.time_per_cycle = p.time_per_cycle_sec;
            ev.data.config.dwell_time     = p.dwell_time_sec;
            ev.data.config.bounds_method_stallguard = (p.bounds_method == 0);
            ev.data.config.bounds_search_velocity_rpm = p.bounds_search_velocity_rpm;
            ev.data.config.stallguard_min_velocity_rpm = p.stallguard_min_velocity_rpm;
            ev.data.config.stall_detection_current_factor = p.stall_detection_current_factor;
            ev.data.config.bounds_search_accel_rev_s2 = p.bounds_search_accel_rev_s2;
            break;
        }
        case MsgType::START: {
            ev.type = ProtoEventType::START;
            break;
        }
        case MsgType::PAUSE: {
            ev.type = ProtoEventType::PAUSE;
            break;
        }
        case MsgType::RESUME: {
            ev.type = ProtoEventType::RESUME;
            break;
        }
        case MsgType::STOP: {
            ev.type = ProtoEventType::STOP;
            break;
        }
        default:
            ESP_LOGW(TAG, "Unhandled msg type %u", (unsigned)pkt.hdr.type);
            return;
    }

    if (s_protoEventQueue) {
        xQueueSend(s_protoEventQueue, &ev, 0);
    }
}

static void recv_task(void* arg)
{
    RawMsg msg{};
    while (true) {
        if (xQueueReceive(s_rawRecvQueue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.len < (int)sizeof(EspNowHeader) + 2) {
                ESP_LOGW(TAG, "Packet too short: %d bytes (min %zu)", msg.len, sizeof(EspNowHeader) + 2);
                continue;
            }
            
            // Parse header first to get payload length
            EspNowHeader hdr{};
            std::memcpy(&hdr, msg.data, sizeof(hdr));
            
            // Verify we have enough data for header + payload + CRC
            size_t expected_len = sizeof(hdr) + hdr.len + sizeof(uint16_t);
            if (msg.len < (int)expected_len) {
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
            
            // Debug: Print raw received data and packet details
            ESP_LOGW(TAG, "RX: msg.len=%d, type=%u, id=%u, payload_len=%u, crc_offset=%zu", 
                     msg.len, hdr.type, hdr.id, hdr.len, crc_offset);
            ESP_LOGW(TAG, "Raw received data (first %d bytes):", msg.len > 16 ? 16 : msg.len);
            for (int i = 0; i < (msg.len > 16 ? 16 : msg.len); i++) {
                ESP_LOGW(TAG, "  msg.data[%d] = 0x%02X", i, msg.data[i]);
            }
            if (msg.len >= (int)(crc_offset + sizeof(uint16_t))) {
                ESP_LOGW(TAG, "CRC bytes at offset %zu: 0x%02X 0x%02X, recv_crc=0x%04X",
                         crc_offset, msg.data[crc_offset], msg.data[crc_offset+1], recv_crc);
            } else {
                ESP_LOGW(TAG, "ERROR: Packet too short for CRC! msg.len=%d, need=%zu", 
                         msg.len, crc_offset + sizeof(uint16_t));
            }
            if (hdr.len > 0) {
                ESP_LOGD(TAG, "Payload (first %d bytes):", hdr.len > 16 ? 16 : hdr.len);
                for (int i = 0; i < (hdr.len > 16 ? 16 : hdr.len); i++) {
                    ESP_LOGD(TAG, "  [%d] = 0x%02X", i, pkt.payload[i]);
                }
            }
            
            handle_packet(pkt, recv_crc);
        }
    }
}
