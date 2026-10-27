#ifndef MQTT_TRANSPORT_H
#define MQTT_TRANSPORT_H

#include <cstddef>
#include <cstdint>

class MqttTransport {
public:
    using ConnectCallback = void (*)(bool sessionPresent);
    using DisconnectCallback = void (*)();
    using SubscribeCallback = void (*)(uint16_t packetId, uint8_t qos);
    using MessageCallback = void (*)(const char* topic, const char* payload,
                                     size_t len, size_t index, size_t total);

    struct TlsConfig {
        const char* caPem;
        size_t caLen;
        const char* clientCertPem;
        size_t clientCertLen;
        const char* clientKeyPem;
        size_t clientKeyLen;
    };

    MqttTransport();
    ~MqttTransport();

    MqttTransport(const MqttTransport&) = delete;
    MqttTransport& operator=(const MqttTransport&) = delete;

    bool begin(const char* host, uint16_t port,
               const char* clientId,
               const char* username,
               const char* password,
               const TlsConfig* tls = nullptr);

    void onConnect(ConnectCallback cb);
    void onDisconnect(DisconnectCallback cb);
    void onSubscribe(SubscribeCallback cb);
    void onMessage(MessageCallback cb);

    bool start();
    bool connect();
    void disconnect();
    bool isConnected() const;

    uint16_t publish(const char* topic, uint8_t qos, bool retain,
                     const char* data, size_t len);

    uint16_t subscribe(const char* topic, uint8_t qos);

    void setWill(const char* topic, uint8_t qos, bool retain,
                 const char* data, size_t len);

private:
    struct Impl;
    Impl* m_impl;
};

#endif // MQTT_TRANSPORT_H
