#include "MqttTransport.h"
#include "Log.h"

#include "mqtt_client.h"

#include <cstring>

struct MqttTransport::Impl {
    esp_mqtt_client_handle_t client = nullptr;
    esp_mqtt_client_config_t cfg = {};
    bool started = false;
    bool connected = false;

    ConnectCallback connectCb = nullptr;
    DisconnectCallback disconnectCb = nullptr;
    SubscribeCallback subscribeCb = nullptr;
    MessageCallback messageCb = nullptr;

    char topicBuf[192] = {};

    static void eventHandler(void* handlerArgs, esp_event_base_t base,
                             int32_t eventId, void* eventData) {
        (void)base;

        auto* self = static_cast<Impl*>(handlerArgs);
        auto* event = static_cast<esp_mqtt_event_handle_t>(eventData);
        if (self == nullptr || event == nullptr) return;

        switch ((esp_mqtt_event_id_t)eventId) {
            case MQTT_EVENT_BEFORE_CONNECT:
                if (self->client != nullptr) {
                    esp_err_t err = esp_mqtt_set_config(self->client, &self->cfg);
                    if (err != ESP_OK) {
                        LOG_W("MQTT: set_config fallo (%d)", (int)err);
                    }
                }
                break;

            case MQTT_EVENT_CONNECTED:
                self->connected = true;
                if (self->connectCb != nullptr) {
                    self->connectCb(event->session_present != 0);
                }
                break;

            case MQTT_EVENT_DISCONNECTED:
                self->connected = false;
                if (self->disconnectCb != nullptr) {
                    self->disconnectCb();
                }
                break;

            case MQTT_EVENT_SUBSCRIBED:
                if (self->subscribeCb != nullptr) {
                    self->subscribeCb((uint16_t)event->msg_id, 0);
                }
                break;

            case MQTT_EVENT_DATA: {
                if (event->current_data_offset == 0) {
                    size_t topicLen = event->topic_len > 0 ? (size_t)event->topic_len : 0;
                    if (topicLen >= sizeof(self->topicBuf)) {
                        topicLen = sizeof(self->topicBuf) - 1;
                    }
                    if (event->topic != nullptr && topicLen > 0) {
                        memcpy(self->topicBuf, event->topic, topicLen);
                    }
                    self->topicBuf[topicLen] = '\0';
                }

                if (self->messageCb != nullptr) {
                    const size_t len = event->data_len > 0 ? (size_t)event->data_len : 0;
                    const size_t index = event->current_data_offset > 0
                                             ? (size_t)event->current_data_offset
                                             : 0;
                    const size_t total = event->total_data_len > 0
                                             ? (size_t)event->total_data_len
                                             : len;
                    self->messageCb(self->topicBuf, event->data, len, index, total);
                }
                break;
            }

            case MQTT_EVENT_ERROR:
                if (event->error_handle != nullptr) {
                    LOG_E("MQTT: error type=%d conn=%d tls_err=0x%X verify=0x%X",
                          (int)event->error_handle->error_type,
                          (int)event->error_handle->connect_return_code,
                          (unsigned)event->error_handle->esp_tls_last_esp_err,
                          (unsigned)event->error_handle->esp_tls_cert_verify_flags);
                }
                if (!self->connected && self->disconnectCb != nullptr) {
                    self->disconnectCb();
                }
                break;

            default:
                break;
        }
    }
};

MqttTransport::MqttTransport() : m_impl(new Impl()) {}

MqttTransport::~MqttTransport() {
    if (m_impl == nullptr) return;
    if (m_impl->client != nullptr) {
        esp_mqtt_client_stop(m_impl->client);
        esp_mqtt_client_destroy(m_impl->client);
    }
    delete m_impl;
}

bool MqttTransport::begin(const char* host, uint16_t port,
                          const char* clientId,
                          const char* username,
                          const char* password,
                          const TlsConfig* tls) {
    if (m_impl == nullptr || m_impl->client != nullptr) return false;

    esp_mqtt_client_config_t& cfg = m_impl->cfg;

    cfg.host = host;
    cfg.port = port;
    cfg.client_id = clientId;

    cfg.username = (username != nullptr && username[0] != '\0') ? username : nullptr;
    cfg.password = (cfg.username != nullptr) ? password : nullptr;

    cfg.keepalive = 15;
    cfg.disable_clean_session = 0;
    cfg.disable_auto_reconnect = true;
    cfg.protocol_ver = MQTT_PROTOCOL_V_3_1_1;
    cfg.task_stack = 8192;
    cfg.buffer_size = 2048;
    cfg.out_buffer_size = 2048;
    cfg.network_timeout_ms = 2000;   // < TWDT (5 s): acota bloqueos de red

    if (tls != nullptr) {
        cfg.transport = MQTT_TRANSPORT_OVER_SSL;
        cfg.cert_pem = tls->caPem;
        cfg.cert_len = tls->caLen;
        cfg.client_cert_pem = tls->clientCertPem;
        cfg.client_cert_len = tls->clientCertLen;
        cfg.client_key_pem = tls->clientKeyPem;
        cfg.client_key_len = tls->clientKeyLen;
    } else {
        cfg.transport = MQTT_TRANSPORT_OVER_TCP;
    }

    m_impl->client = esp_mqtt_client_init(&cfg);
    if (m_impl->client == nullptr) {
        LOG_E("MQTT: esp_mqtt_client_init fallo");
        return false;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        m_impl->client, MQTT_EVENT_ANY, &Impl::eventHandler, m_impl);
    if (err != ESP_OK) {
        LOG_E("MQTT: register_event fallo (%d)", (int)err);
        esp_mqtt_client_destroy(m_impl->client);
        m_impl->client = nullptr;
        return false;
    }

    return true;
}

void MqttTransport::onConnect(ConnectCallback cb) {
    if (m_impl != nullptr) m_impl->connectCb = cb;
}

void MqttTransport::onDisconnect(DisconnectCallback cb) {
    if (m_impl != nullptr) m_impl->disconnectCb = cb;
}

void MqttTransport::onSubscribe(SubscribeCallback cb) {
    if (m_impl != nullptr) m_impl->subscribeCb = cb;
}

void MqttTransport::onMessage(MessageCallback cb) {
    if (m_impl != nullptr) m_impl->messageCb = cb;
}

bool MqttTransport::start() {
    if (m_impl == nullptr || m_impl->client == nullptr) return false;
    if (m_impl->started) return true;

    esp_err_t err = esp_mqtt_client_start(m_impl->client);
    if (err != ESP_OK) {
        LOG_E("MQTT: start fallo (%d)", (int)err);
        return false;
    }

    m_impl->started = true;
    return true;
}

bool MqttTransport::connect() {
    if (m_impl == nullptr || m_impl->client == nullptr) return false;
    if (!m_impl->started) return start();

    esp_err_t err = esp_mqtt_client_reconnect(m_impl->client);
    if (err != ESP_OK) {
        LOG_W("MQTT: reconnect fallo (%d)", (int)err);
        return false;
    }

    return true;
}

void MqttTransport::disconnect() {
    if (m_impl == nullptr || m_impl->client == nullptr) return;
    esp_mqtt_client_disconnect(m_impl->client);
}

bool MqttTransport::isConnected() const {
    return m_impl != nullptr && m_impl->connected;
}

uint16_t MqttTransport::publish(const char* topic, uint8_t qos, bool retain,
                                const char* data, size_t len) {
    if (m_impl == nullptr || m_impl->client == nullptr) return 0;

    int rc = esp_mqtt_client_publish(m_impl->client, topic, data, (int)len,
                                     (int)qos, retain ? 1 : 0);
    if (rc < 0) return 0;
    return rc == 0 ? 1 : (uint16_t)rc;
}

uint16_t MqttTransport::subscribe(const char* topic, uint8_t qos) {
    if (m_impl == nullptr || m_impl->client == nullptr) return 0;

    int rc = esp_mqtt_client_subscribe(m_impl->client, topic, (int)qos);
    return rc < 0 ? 0 : (uint16_t)rc;
}

void MqttTransport::setWill(const char* topic, uint8_t qos, bool retain,
                            const char* data, size_t len) {
    if (m_impl == nullptr) return;

    esp_mqtt_client_config_t& cfg = m_impl->cfg;
    cfg.lwt_topic = topic;
    cfg.lwt_msg = data;
    cfg.lwt_msg_len = (int)len;
    cfg.lwt_qos = (int)qos;
    cfg.lwt_retain = retain ? 1 : 0;

    if (m_impl->client != nullptr) {
        esp_err_t err = esp_mqtt_set_config(m_impl->client, &cfg);
        if (err != ESP_OK) {
            LOG_W("MQTT: set_config fallo (%d)", (int)err);
        }
    }
}
