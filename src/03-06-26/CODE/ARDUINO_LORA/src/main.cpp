#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// --- Credenciales LoRaWAN (ABP - Activation By Personalization) ---
// Selección automática basada en el tipo de Arduino

#if defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32u4__) || defined(ARDUINO_AVR_MEGA) || defined(__AVR_ATmega2560__)
    // Credenciales para Arduino Leonardo o Mega
    static const u1_t APPSKEY[16] = { 0xA3, 0xC5, 0xB2, 0x08, 0x4F, 0xE7, 0xD2, 0x94, 0x6B, 0x81, 0xF9, 0xA1, 0x25, 0x3E, 0xD0, 0x47 };
    static const u1_t NWKSKEY[16] = { 0xE5, 0xC7, 0xB4, 0xF9, 0x3A, 0xD6, 0xC2, 0x8E, 0x7F, 0x81, 0x20, 0xA9, 0x64, 0xC5, 0xB3, 0x7F };
    static const u4_t DEVADDR = 0x260CA111;
    #define DEVICE_TYPE "Leonardo/Mega"
#elif defined(ARDUINO_AVR_UNO) || defined(__AVR_ATmega328P__)
    // Credenciales para Arduino Uno
    static const u1_t APPSKEY[16] = { 0x8F, 0x19, 0xE7, 0x3A, 0xCB, 0x50, 0x74, 0x61, 0x92, 0xA4, 0xDF, 0x26, 0xB3, 0xE1, 0x8C, 0x59 };
    static const u1_t NWKSKEY[16] = { 0xC4, 0xD8, 0xE9, 0x1F, 0xB2, 0x7A, 0x08, 0xF3, 0xE1, 0x56, 0x2A, 0xC4, 0x8B, 0x5F, 0x70, 0xDD };
    static const u4_t DEVADDR = 0x260CA112;
    #define DEVICE_TYPE "Uno"
#else
    #error "Tipo de Arduino no soportado. Solo se admite Arduino Uno, Leonardo o Mega."
#endif

// Estas funciones son requeridas por la librería LMIC, pero para ABP se pueden dejar vacías.
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// --- Configuración de Pines para Dragino LoRa/GPS Shield v1.3 en Arduino UNO ---
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7}, 
};

// --- Phantom payload (formato nuevo: per-sensor Len Bytes, bloques de 4 bytes) ---
// Wire format: [ID_MSG][TIMESTAMP 4B BE][ACTIVATE][LEN_BYTES...][DATA_BLOCKS...]
const byte phantom_payload[] = {
    0x26,                           // ID_MSG
    0x67, 0x1B, 0x77, 0x40,         // Timestamp (hardcoded, big-endian)
    0xFF,                           // Activate byte: 8 sensores activos (bits 0-7)
    // Len Bytes (1 por sensor, LSB→MSB order)
    0x01,                           // BATERIA (bit 0): 1 muestra x 4 bytes
    0x01,                           // VOLTAJE (bit 1)
    0x01,                           // CORRIENTE (bit 2)
    0x01,                           // EXT+0 Presión (bit 3)
    0x01,                           // EXT+1 mV (bit 4)
    0x01,                           // EXT+2 Lux (bit 5)
    0x01,                           // EXT+3 Potencia (bit 6)
    0x01,                           // EXT+4 FP (bit 7)
    // Data blocks (4 bytes each, float32 big-endian)
    0x42, 0xC8, 0x00, 0x00,         // BATERIA: 100.0 kWh
    0x43, 0x7A, 0x00, 0x00,         // VOLTAJE: 250.0 V
    0x40, 0xA0, 0x00, 0x00,         // CORRIENTE: 5.0 A
    0x42, 0xC8, 0x00, 0x00,         // EXT+0: 100.0 (presión)
    0x44, 0x7A, 0x00, 0x00,         // EXT+1: 1000.0 (mV)
    0x44, 0x7A, 0x00, 0x00,         // EXT+2: 1000.0 (lux)
    0x43, 0x48, 0x00, 0x00,         // EXT+3: 200.0 W
    0x3F, 0x80, 0x00, 0x00,         // EXT+4: 1.0 (factor de potencia)
};

// --- Variables para el control de tiempo y envío ---
unsigned long lastSendTime = 0;
const unsigned long intervalMs = 30000;  // 30 segundos

// Función para manejar los eventos de la librería LMIC
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (Paquete enviado)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("ACK Recibido"));
            if (LMIC.dataLen) {
              Serial.println(F("Datos recibidos!"));
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.print(F("Evento desconocido: "));
            Serial.println((unsigned) ev);
            break;
    }
}

// Función para poner en cola un paquete para su envío
void do_send(const byte* payload, size_t size) {
    // Revisar si el transmisor está ocupado
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, no se puede enviar. Reintentando..."));
    } else {
        // Preparar el paquete para transmitir en el puerto 69 (como en el código original)
        LMIC_setTxData2(69, (uint8_t*)payload, size, 0);
        Serial.print(F("Paquete de "));
        Serial.print(size);
        Serial.println(F(" bytes encolado para envío."));
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println(F("Iniciando Nodo LoRaWAN ABP..."));

    // LMIC init
    os_init();
    // Resetear el estado de MAC.
    LMIC_reset();

    // Configurar la sesión ABP con las credenciales
    LMIC_setSession (0x1, DEVADDR, (u1_t*)NWKSKEY, (u1_t*)APPSKEY);

    // Seleccionar la sub-banda correcta para tu región (US915)
    // El canal 7 es una elección común en la sub-banda 2.
    // Si usas TTN, esto es manejado por el network server.
    #if defined(CFG_us915)
    LMIC_selectSubBand(7); // Para TTN en US915, la sub-banda 1 (canales 8-15) es común.
    #endif
    
    // Deshabilitar Link Check para ABP (no es necesario y ahorra memoria/energía)
    LMIC_setLinkCheckMode(0);

    // TTN utiliza DR_SF7 para las respuestas en RX2.
    LMIC.dn2Dr = DR_SF9;

    // Configurar potencia de transmisión (14dBm es un valor seguro para muchas regiones)
    LMIC_setDrTxpow(DR_SF7,14);

    Serial.println(F("Nodo configurado. Iniciando bucle de envío."));
}

void loop() {
    unsigned long currentTime = millis();

    // El motor de LMIC debe correr constantemente
    os_runloop_once();

    // Solo intentar enviar si el stack no está ya ocupado
    if (!(LMIC.opmode & OP_TXRXPEND)) {
        if (currentTime - lastSendTime >= intervalMs) {
            lastSendTime = currentTime;
            Serial.println("\nEnviando payload fantasma...");
            do_send(phantom_payload, sizeof(phantom_payload));
        }
    }
}