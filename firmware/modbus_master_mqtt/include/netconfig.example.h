#ifndef NETCONFIG_H
#define NETCONFIG_H

// Copia este archivo a include/netconfig.h y completa tus valores.
// include/netconfig.h esta gitignored: nunca commitees credenciales reales.

#define WIFI_SSID   "your-ssid"
#define WIFI_PASS   "your-password"

// 1883 sin TLS es solo para una LAN de confianza.
// Fuera de la LAN usa MQTT sobre TLS (normalmente 8883), credenciales y ACLs.
// MqttTransport tiene soporte TLS (TlsConfig), pero mqtt_setup() no lo cablea
// todavia: este proyecto usa TCP plano por defecto.
#define MQTT_HOST   "192.168.1.10"
#define MQTT_PORT   1883
#define MQTT_USER   ""
#define MQTT_PASS   ""

#define MQTT_CLIENT "nemo-medidor1"

#define SPB_GROUP_ID  "your-group"
#define SPB_EDGE_NODE "your-edge-node"

// En una LAN aislada, SPB_NTP1 puede ser la IP de un servidor NTP local.
#define SPB_NTP1      "pool.ntp.org"
#define SPB_NTP2      "time.nist.gov"

#endif // NETCONFIG_H
