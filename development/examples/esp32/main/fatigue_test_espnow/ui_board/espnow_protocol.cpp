/**
 * @file espnow_protocol.cpp
 * @brief ESP-NOW protocol implementation for UI board
 */

#include "espnow_protocol.hpp"
#include "config.hpp"
#include <cstring>
#include "esp_system.h"

static const char* TAG = "EspNowProto";

static QueueHandle_t s_protoEventQueue = nullptr;
static uint8_t s_nextMsgId = 1;

// recv queue from ISR to this module
static QueueHandle_t s_rawRecvQueue = nullptr;

struct RawMsg {
    uint8_t data[sizeof(EspNowPacket)];
    int     len;
};

// Forward declarations
static void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len);
static void espnow_send_cb(const wifi_tx_info_t* info, esp_now_send_status_t status);
static void recv_task(void*);

// -------- ESPNOW INIT --------

// -------- PACKET BUILD / SEND HELPERS --------

static bool send_packet(MsgType type, const void* payload, uint8_t payload_len)
{
    if (payload_len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload too big: %d", payload_len);
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

    size_t crc_len = sizeof(pkt.hdr) + payload_len;
    pkt.crc = crc16_ccitt(reinterpret_cast<uint8_t*>(&pkt), crc_len);

    size_t total_len = crc_len + sizeof(pkt.crc);
    esp_err_t err = esp_now_send(TEST_UNIT_MAC, reinterpret_cast<uint8_t*>(&pkt), total_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send error: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

// -------- PUBLIC SEND FUNCTIONS --------
// (Moved to namespace EspNowProto below)

// -------- CALLBACKS & RECV TASK --------

static void espnow_send_cb(const wifi_tx_info_t* info, esp_now_send_status_t status)
{
    (void)info; // Unused in ESP-IDF v5.5
    ESP_LOGD(TAG, "ESP-NOW send status=%s",
             status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

static void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    if (len <= 0 || len > (int)sizeof(EspNowPacket)) return;

    RawMsg msg{};
    msg.len = len;
    std::memcpy(msg.data, data, len);

    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_rawRecvQueue, &msg, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

static void handle_packet(const EspNowPacket& pkt)
{
    if (pkt.hdr.sync != ESPNOW_SYNC_BYTE) {
        ESP_LOGW(TAG, "Bad SYNC 0x%02X", pkt.hdr.sync);
        return;
    }
    if (pkt.hdr.len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "Bad LEN %d", pkt.hdr.len);
        return;
    }
    size_t crc_len = sizeof(pkt.hdr) + pkt.hdr.len;
    uint16_t calc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt), crc_len);
    if (calc != pkt.crc) {
        ESP_LOGW(TAG, "CRC mismatch (calc=0x%04X recv=0x%04X)", calc, pkt.crc);
        return;
    }

    MsgType type = static_cast<MsgType>(pkt.hdr.type);
    ProtoEvent ev{};

    switch (type) {
        case MsgType::CONFIG_RESPONSE: {
            ConfigPayload p{};
            std::memcpy(&p, pkt.payload, sizeof(p));
            ev.type = ProtoEventType::CONFIG_UPDATED;
            ev.data.config.cycle_amount   = p.cycle_amount;
            ev.data.config.time_per_cycle = p.time_per_cycle_sec;
            ev.data.config.dwell_time     = p.dwell_time_sec;
            ev.data.config.orientation_flipped = (p.orientation_flipped != 0);
            ev.data.config.bounds_method_stallguard = (p.bounds_method == 0);
            break;
        }
        case MsgType::CONFIG_ACK: {
            ConfigAckPayload p{};
            std::memcpy(&p, pkt.payload, sizeof(p));
            ev.type = p.ok ? ProtoEventType::CONFIG_APPLY_OK : ProtoEventType::CONFIG_APPLY_FAIL;
            break;
        }
        case MsgType::START_ACK: {
            ev.type = ProtoEventType::STARTED;
            break;
        }
        case MsgType::PAUSE_ACK: {
            ev.type = ProtoEventType::PAUSED;
            break;
        }
        case MsgType::RESUME_ACK: {
            ev.type = ProtoEventType::RESUMED;
            break;
        }
        case MsgType::STOP_ACK: {
            ev.type = ProtoEventType::STOPPED;
            break;
        }
        case MsgType::STATUS_UPDATE: {
            StatusPayload p{};
            std::memcpy(&p, pkt.payload, sizeof(p));
            ev.type = ProtoEventType::STATUS;
            ev.data.status.cycle   = p.cycle_number;
            ev.data.status.state   = static_cast<TestState>(p.state);
            ev.data.status.err_code= p.err_code;
            if (ev.data.status.state == TestState::COMPLETED)
                ev.type = ProtoEventType::TEST_COMPLETED;
            if (ev.data.status.state == TestState::ERROR)
                ev.type = ProtoEventType::ERROR_EVENT;
            break;
        }
        case MsgType::ERROR: {
            ErrorPayload p{};
            std::memcpy(&p, pkt.payload, sizeof(p));
            ev.type = ProtoEventType::ERROR_EVENT;
            ev.data.error.err_code = p.err_code;
            ev.data.error.at_cycle = p.at_cycle;
            break;
        }
        case MsgType::TEST_COMPLETE: {
            ev.type = ProtoEventType::TEST_COMPLETED;
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
            EspNowPacket pkt{};
            if (msg.len < (int)sizeof(EspNowHeader) + 2) continue;
            std::memcpy(&pkt, msg.data, msg.len);
            handle_packet(pkt);
        }
    }
}

// -------- NAMESPACE API --------

namespace EspNowProto {

bool init(QueueHandle_t event_queue)
{
    s_protoEventQueue = event_queue;

    // Init WiFi in STA mode
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    esp_now_peer_info_t peer{};
    std::memset(&peer, 0, sizeof(peer));
    std::memcpy(peer.peer_addr, TEST_UNIT_MAC, 6);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    s_rawRecvQueue = xQueueCreate(10, sizeof(RawMsg));
    xTaskCreate(recv_task, "espnow_recv_task", 4096, nullptr, 5, nullptr);

    ESP_LOGI(TAG, "ESP-NOW protocol initialized");
    return true;
}

bool send_config_request()
{
    return send_packet(MsgType::CONFIG_REQUEST, nullptr, 0);
}

bool send_config_set(const Settings& s)
{
    ConfigPayload p{};
    p.cycle_amount       = s.cycle_amount;
    p.time_per_cycle_sec = s.time_per_cycle;
    p.dwell_time_sec     = s.dwell_time;
    p.orientation_flipped= s.orientation_flipped ? 1 : 0;
    p.bounds_method      = s.bounds_method_stallguard ? 0 : 1;
    return send_packet(MsgType::CONFIG_SET, &p, sizeof(p));
}

bool send_start()
{
    return send_packet(MsgType::START, nullptr, 0);
}

bool send_pause()
{
    return send_packet(MsgType::PAUSE, nullptr, 0);
}

bool send_resume()
{
    return send_packet(MsgType::RESUME, nullptr, 0);
}

bool send_stop()
{
    return send_packet(MsgType::STOP, nullptr, 0);
}

} // namespace EspNowProto
