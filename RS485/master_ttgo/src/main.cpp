#include <Arduino.h>
#include <SPI.h>
#include <hal/hal.h>
#include <lmic.h>

// ==================== LORA CONFIG ====================
// Deshabilitar ventana RX de recepción (si no se esperan downlinks)
#define DISABLE_INVERT_IQ_ON_RX 1
#define DISABLE_RX 1
#define CFG_sx1272_radio 1

// Funciones de configuración LoRa (placeholders)
void os_getArtEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevKey(u1_t *buf) { memset(buf, 0, 16); }

// Claves de red y dispositivo (reemplaza con tus valores reales)
static u1_t NWKSKEY[16] = {0x49, 0x78, 0xCB, 0x8E, 0x7F, 0xFB, 0xD4, 0x6B,
                           0xC5, 0x70, 0xFE, 0x11, 0xF1, 0x7F, 0xA5, 0x6E};
static u1_t APPSKEY[16] = {0x53, 0xC0, 0x20, 0x84, 0x14, 0x86, 0x26, 0x39,
                           0x81, 0xFA, 0x77, 0x35, 0x5D, 0x27, 0x87, 0x62};
static const u4_t DEVADDR = 0x260CB229;

// Configuración de pines LoRa
const lmic_pinmap lmic_pins = {.nss = 18,
                               .rxtx = LMIC_UNUSED_PIN,
                               .rst = LMIC_UNUSED_PIN,
                               .dio = {26, 33, 32}};

// Payload máximo para DR3
constexpr size_t LORA_PAYLOAD_MAX = 220;

// Estructura para fragmentos de datos LoRa
struct Fragmento {
    uint8_t data[LORA_PAYLOAD_MAX];
    size_t len;
};

// Cola para mensajes binarios LoRa
QueueHandle_t queueFragmentos;

// Semáforo para controlar el fin de un ciclo de envío
SemaphoreHandle_t semaforoEnvioCompleto;

// ==================== LORA CALLBACKS ====================
// Callback de eventos LoRa
void onEvent(ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        Serial.println("[LORA] TX completo.");
        // Libera el semáforo para indicar que el ciclo de transmisión ha terminado.
        xSemaphoreGive(semaforoEnvioCompleto);
        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println("[LORA] ACK recibido.");
        }
    }
}

// ==================== FUNCIONES LORA ====================
// Inicialización de LoRa
void initLoRa() {
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // 1% de tolerancia

    // Configuración específica para ABP y US915
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
    LMIC_selectSubBand(7); // Asegúrate que esta es la sub-banda correcta para tu gateway
    LMIC_setDrTxpow(US915_DR_SF7, 20); // DR3 = SF7BW125
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
}

// ==================== TAREA LORA (OPTIMIZADA) ====================
// Tarea dedicada para el envío de datos LoRaWAN
void tareaLoRa(void *pvParameters) {
    Fragmento frag;
    while (true) {
        if (xQueueReceive(queueFragmentos, &frag, portMAX_DELAY) == pdTRUE) {
            // Espera semáforo antes de enviar
            xSemaphoreTake(semaforoEnvioCompleto, portMAX_DELAY);
            LMIC_setTxData2(1, frag.data, frag.len, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // sólo lógica propia, no runloop
    }
}

// Tarea dedicada solo al runloop
void tareaRunLoop(void *pvParameters) {
    while (true) {
        os_runloop_once();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ==================== SETUP Y LOOP ====================
void setup() {
    Serial.begin(115200); // 115200 es más estándar y estable que 921600
    while (!Serial); // Espera a que el puerto serie esté listo
    delay(1000);
    Serial.println("Iniciando sistema...");

    // Inicialización SPI
    SPI.begin();

    // Creación de cola para fragmentos
    queueFragmentos = xQueueCreate(10, sizeof(Fragmento));

    // Creación de semáforo binario para señalizar el fin de la transmisión
    semaforoEnvioCompleto = xSemaphoreCreateBinary();
    xSemaphoreGive(semaforoEnvioCompleto); // Lo dejamos disponible para el primer envío

    // Inicialización de LoRa
    initLoRa();

    xTaskCreatePinnedToCore(tareaRunLoop, "RunLoop", 2048, NULL, 2, NULL, 1);

    // Creación de tarea LoRa en el núcleo 1
    xTaskCreatePinnedToCore(tareaLoRa, "LoRaTask", 2048, NULL, 5, NULL, 1); // 4KB y prioridad 5

    // Ejemplo: Enviar un paquete de prueba después de 5 segundos
    delay(5000);
    Fragmento testFrag;
    const char* mensaje = "Hola LoRa , 128qedsdsadasdasdsadsadsadsadsadsafsdfsdffs!";
    testFrag.len = strlen(mensaje);
    memcpy(testFrag.data, mensaje, testFrag.len);
    if (xQueueSend(queueFragmentos, &testFrag, pdMS_TO_TICKS(100)) == pdPASS) {
        Serial.println("Fragmento de prueba enviado a la cola.");
    }
}

void loop() {
    // El loop principal puede quedar vacío o usarse para tareas de baja prioridad.
    // Es mejor que la tarea del loop no se elimine y simplemente ceda el control.
    vTaskDelay(pdMS_TO_TICKS(1000));
}