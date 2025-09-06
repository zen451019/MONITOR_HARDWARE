/**********************************************************************************************************************
 * @file        main.cpp
 * @author      Manuel Felipe Ospina
 * @date        2025-09-03
 * @version     1.0
 *
 * @brief       Firmware for NEMO (Node for Electromechanical Monitoring & Operation).
 * @details     This firmware runs on an ESP32 and is designed for the acquisition, processing,
 * and transmission of analog signals (current/voltage). It features an OLED display
 * for real-time visualization and uses LoRaWAN for data uplink.
 *
 * --------------------------------------------------------------------------------------------------------------------
 *
 * @copyright   Copyright (c) 2025, EMASA. All rights reserved.
 *
 * @license     Pending
 *
 **********************************************************************************************************************
 *
 * System Architecture:
 * --------------------
 * The system uses FreeRTOS to manage concurrent tasks for data acquisition, processing, and communication.
 * - An ISR (`onADCTimer`) samples ADC pins at a high frequency (`FS_HZ`) and stores data in a circular FIFO buffer.
 * - `TaskProcesamiento`: Periodically calculates RMS values from the FIFO, applies an adaptive EMA filter,
 * and sends the results to a processing queue.
 * - `TaskRegistroResultados`: Collects processed results into blocks. Once a block is full or the system
 * is disabled, it encodes the data into a bit-packed LoRaWAN payload.
 * - `tareaLoRa`: Manages the LoRaWAN communication, sending payloads from a queue and handling TX/RX events.
 * - `TaskBatteryLevel`: Periodically measures the battery voltage.
 * - `TaskDisplay`: Manages the OLED screen, showing a boot sequence and a log of recent LoRaWAN transmissions.
 * - `TaskMonitorPin`: Controls the system's global enable/disable state based on a physical pin.
 *
 * Main Features:
 * --------------
 * - Multi-channel ADC sampling using a hardware timer ISR.
 * - Efficient RMS calculation with a running circular buffer.
 * - Adaptive Exponential Moving Average (EMA) filtering for signal smoothing.
 * - Data blocking and bit-packed payload encoding for efficient LoRaWAN transmission.
 * - LoRaWAN uplink configured for US915 (Sub-band 7, DR3).
 * - Battery level monitoring.
 * - System ON/OFF control via `MONITOR_PIN`.
 * - OLED display with boot logo and event history.
 *
 * Hardware Configuration:
 * -----------------------
 * - MCU:          ESP32
 * - Display:      128x64 OLED (I2C Addr: 0x3C)
 * - LoRa Module:  (LMIC compatible)
 * - NSS:        GPIO 18
 * - RST:        GPIO 14
 * - DIO{0,1,2}: GPIO {26, 33, 32}
 * - Analog Inputs:
 * - Voltage 1:  GPIO 36
 * - Voltage 2:  GPIO 39
 * - Voltage 3:  GPIO 34
 * - Current 1:  GPIO 25
 * - Control & Monitoring:
 * - System EN/DIS: GPIO 35 (MONITOR_PIN)
 * - Battery ADC:   GPIO 14 (BATTERY_PIN) *Note: Same as LoRa RST.*
 *
 * Dependencies:
 * -------------
 * - Arduino Core for ESP32
 * - FreeRTOS (ESP-IDF)
 * - ESP32AnalogRead library
 * - MCCI LoRaWAN LMIC library
 * - Adafruit GFX and Adafruit SSD1306 libraries
 *
 * Revision History:
 * -----------------
 * Version |    Date    |     Author      |              Description
 * --------|------------|-----------------|----------------------------------------------------
 * 1.0     | 2025-09-03 | Manuel F. Ospina| Changed LoRa message transmission to a unified
 *         |            |                 | message model, replacing previous fragmented system.
 * --------|------------|-----------------|----------------------------------------------------
 * 1.1     | 2025-09-05 | Manuel F. Ospina| Redesigned payload format into a unified, scalable
 *         |            |                 | and modular message model for LoRa transmission.
 * --------|------------|-----------------|----------------------------------------------------
 * 
 *********************************************************************************************************************/

#include <Arduino.h>

// Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/TomThumb.h>   // fuente ultra pequeña

const unsigned char epd_bitmap_ [] PROGMEM = {
  // 'Asset 3, 20x20px
    0x00, 0x00, 0x00, 0x07, 0xfe, 0x00, 0x1f, 0xff, 0x80, 0x3f, 0xff, 0xc0, 0x7e, 0x07, 0xe0, 0x78, 
    0x01, 0xf0, 0x71, 0xf8, 0xe0, 0x03, 0xfc, 0x00, 0x07, 0xff, 0x00, 0x0f, 0xff, 0x00, 0x0f, 0x0f, 
    0x00, 0x06, 0x02, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x60, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ADC, math, RTOS, LoRa
#include <ESP32AnalogRead.h>
#include <math.h>
#include <vector>
#include <stdint.h>
//Deshabilitar ventana RX de recepción
#define DISABLE_INVERT_IQ_ON_RX 1
#define DISABLE_RX 1

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ==================== DEFINES / PARÁMETROS ====================
// Parámetros del display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1   // El pin reset no está conectado en el TTGO
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MONITOR_PIN 35
volatile bool sistema_habilitado = false;

// ==================== PARÁMETROS DEL PIN_ACTIVACIÓN ====================

// ==================== PARÁMETROS DEL SISTEMA ====================
#ifndef FS_HZ
#define FS_HZ 960
#endif
#ifndef EMA_ALPHA
#define EMA_ALPHA 0.08f
#endif
#ifndef PROCESS_PERIOD_MS
#define PROCESS_PERIOD_MS 300
#endif

#define NUM_PINES 4
#define RESULTADOS_POR_BLOQUE 20

#define LORA_PAYLOAD_MAX 220 // Payload máximo para DR3

// ==================== REGISTRO PARA LCD (NUEVA ESTRUCTURA) ====================
#define NUM_EVENTOS_LCD 3

// <<< CAMBIO: Nueva estructura de datos para la comunicación con la pantalla.
// Contiene toda la información necesaria para mostrar un envío.
struct DisplayInfo {
    unsigned long timestamp_s;
    bool sistema_activo;
    bool bateria_incluida;
    float valor_bateria;
    float primer_valor_corriente;
    float primer_valor_voltaje1; // Podríamos mostrar los 3, pero simplificamos
};

// <<< CAMBIO: Cola para enviar la información a la tarea de display.
QueueHandle_t queueDisplayInfo; 
// <<< ELIMINADO: Ya no necesitamos las variables globales antiguas para el LCD.
// volatile EventoLCD eventos_lcd[NUM_EVENTOS_LCD];
// volatile int idx_evento_lcd = 0;
// volatile bool lcd_needs_update = false;


// ==================== BATTERY LEVEL CONFIG ====================
#define BATTERY_PIN 14              // Cambia esto según tu conexión
#define BATTERY_BUFFER_SIZE 1       // Muestras por paquete
#define BATTERY_INTERVAL_MS 60000   // 60 segundos

struct ResultadoBattery {
    unsigned long timestamp;
    uint8_t nivel_codificado;
};

ResultadoBattery buffer_bateria[BATTERY_BUFFER_SIZE];
volatile int index_bateria = 0;
uint8_t id_serie_bateria = 0;
SemaphoreHandle_t mutex_bateria;

// ==================== ESTRUCTURAS DE DATOS ====================
typedef struct {
    int pin;
    float gain;
    ESP32AnalogRead reader;
    float ema_output;
    bool enabled;
    float last_rms;
    bool ema_initialized;
} PinConfig;

typedef struct {
    unsigned long timestamp;
    float valores[NUM_PINES];
} ResultadoRMS;

typedef struct {
    ResultadoRMS bloque[RESULTADOS_POR_BLOQUE];
    volatile int index;
    SemaphoreHandle_t mutex;
} BufferResultados;

// FIFO y sumas para cada pin
#define FIFO_SIZE 320
struct RMS_FIFO {
        uint16_t buffer[FIFO_SIZE];
        int head;
        int count;
        double sum_x;
        double sum_x2;
};
RMS_FIFO fifo_pins[NUM_PINES];

PinConfig pin_configs[NUM_PINES] = {
    {36, 1033.0f, ESP32AnalogRead(), 0.0f, true, 0.0f, false},
    {39, 1017.0f, ESP32AnalogRead(), 0.0f, true, 0.0f, false},
    {34, 1025.0f, ESP32AnalogRead(), 0.0f, true, 0.0f, false},
    {25, 99.0f, ESP32AnalogRead(), 0.0f, true, 0.0f, false}
};


BufferResultados bufferResultados;
QueueHandle_t queueResultados;

// Pines de corriente y voltaje (ajusta según tu hardware)
const int pines_voltaje[3] = {34, 39, 36};
const int pines_corriente[1]   = {25};

// ==================== ESTRUCTURAS PARA SENSORES EXTERNOS/FUTUROS ====================
#define MAX_SENSORES_EXTERNOS 4 // Sensores lógicos 2, 3, 4, 5
#define SENSOR_DATA_MAX_LEN 10  // Máximo 10 bytes de datos por sensor (muy flexible)

struct ExternalSensorData {
    bool is_new;                      // Flag para indicar si hay datos nuevos
    uint8_t data[SENSOR_DATA_MAX_LEN];  // Buffer para los datos del sensor
    uint8_t len;                      // Cuántos bytes de datos hay
    bool packed;                      // ¿Están los datos empaquetados en bits?
    bool extended;                    // ¿Cada muestra usa 2 bytes?
};

// Buffers globales para los datos de los sensores externos
ExternalSensorData external_sensors[MAX_SENSORES_EXTERNOS];
// Mutex para proteger el acceso a cada buffer
SemaphoreHandle_t mutex_external_sensors[MAX_SENSORES_EXTERNOS];


// ==================== LORA CONFIG ====================
void os_getArtEui(u1_t* buf) { memset(buf, 0, 8); }
void os_getDevEui(u1_t* buf) { memset(buf, 0, 8); }
void os_getDevKey(u1_t* buf) { memset(buf, 0, 16); }

static u1_t NWKSKEY[16] = { 0x49, 0x78, 0xCB, 0x8E, 0x7F, 0xFB, 0xD4, 0x6B, 0xC5, 0x70, 0xFE, 0x11, 0xF1, 0x7F, 0xA5, 0x6E };
static u1_t APPSKEY[16] = { 0x53, 0xC0, 0x20, 0x84, 0x14, 0x86, 0x26, 0x39, 0x81, 0xFA, 0x77, 0x35, 0x5D, 0x27, 0x87, 0x62 };
static const u4_t DEVADDR = 0x260CB229;

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {26, 33, 32}
};


// Cola para mensajes binarios LoRa
struct Fragmento {
    uint8_t data[LORA_PAYLOAD_MAX];
    size_t len;
};
QueueHandle_t queueFragmentos;

// Semáforo para controlar el envío de fragmentos
SemaphoreHandle_t semaforoEnvio;

// ==================== FUNCIONES DE PROCESAMIENTO ====================
float applyEMA_original(float input, float &previous_output) {
    previous_output = EMA_ALPHA * input + (1.0f - EMA_ALPHA) * previous_output;
    return previous_output;
}

float applyEMA_adsoluto(float input, float &previous_output) {
    // Parámetros de adaptación
    const float alpha_min = 0.05f;  // Suavizado máximo
    const float alpha_max = 0.3f;   // Respuesta rápida máxima
    const float delta_max = 15.0f;   // Cambio máximo esperado (ajusta según tu señal)

    float delta = fabs(input - previous_output);
    // Calcula alpha proporcional al cambio
    float alpha = alpha_min + (alpha_max - alpha_min) * (delta / delta_max);
    if (alpha > alpha_max) alpha = alpha_max;
    if (alpha < alpha_min) alpha = alpha_min;

    previous_output = alpha * input + (1.0f - alpha) * previous_output;
    return previous_output;
}

float applyEMA(float input, float &previous_output) {
    const float alpha_min = 0.05f;
    const float alpha_max = 0.3f;
    const float delta_rel_max = 0.3f; // 50% de cambio relativo
    const float epsilon = 0.01f;      // Para evitar división por cero

    float delta_rel = fabs(input - previous_output) / (fabs(previous_output) + epsilon);
    float alpha = alpha_min + (alpha_max - alpha_min) * (delta_rel / delta_rel_max);
    if (alpha > alpha_max) alpha = alpha_max;
    if (alpha < alpha_min) alpha = alpha_min;

    previous_output = alpha * input + (1.0f - alpha) * previous_output;
    return previous_output;
}

// ==================== EMPAQUETADOR DE BITS ====================
struct BitPacker {
    uint64_t buffer = 0; // acumulador de hasta 64 bits
    int bits_usados = 0; // cuántos bits válidos hay en el buffer

    void push(uint16_t valor, int nbits, std::vector<uint8_t>& out) {
        // desplazar buffer para dejar espacio
        buffer <<= nbits;
        // quedarnos con solo los bits válidos
        buffer |= (valor & ((1ULL << nbits) - 1));
        bits_usados += nbits;

        // mientras tengamos al menos 8 bits, sacar bytes
        while (bits_usados >= 8) {
            int shift = bits_usados - 8;
            uint8_t byte = (buffer >> shift) & 0xFF;
            out.push_back(byte);
            bits_usados -= 8;
            buffer &= (1 << bits_usados) - 1; // limpiar bits ya usados
        }
    }

    void flush(std::vector<uint8_t>& out) {
        if (bits_usados > 0) {
            uint8_t byte = buffer << (8 - bits_usados);
            out.push_back(byte);
            bits_usados = 0;
            buffer = 0;
        }
    }
};

// ===================== CODIFICACIÓN UNIFICADA =====================
void codificarUnificado(
    const BufferResultados& buffer,
    uint8_t id_mensaje,
    bool nueva_bateria, // Necesitamos saber si la batería se debe enviar, no solo el nivel
    uint8_t nivel_bateria,
    uint8_t data_len_rms, // ej: RESULTADOS_POR_BLOQUE
    bool sistema_habilitado,
    const ExternalSensorData datos_externos[MAX_SENSORES_EXTERNOS], // Nuevos datos
    std::vector<Fragmento>& fragmentos
) {
    std::vector<uint8_t> payload;
    payload.reserve(LORA_PAYLOAD_MAX);

    // ================== 1. CABECERA ==================
    payload.push_back(id_mensaje);

    // Timestamp (si el sistema está activo, usamos el del buffer, si no, el actual)
    unsigned long ts_s = (sistema_habilitado) ? (buffer.bloque[0].timestamp / 1000) : (millis() / 1000);
    payload.push_back((ts_s >> 24) & 0xFF);
    payload.push_back((ts_s >> 16) & 0xFF);
    payload.push_back((ts_s >> 8) & 0xFF);
    payload.push_back(ts_s & 0xFF);

    // Construir Activate byte
    uint8_t activate_byte = 0;
    if (sistema_habilitado) {
        // Asumimos que quieres enviar Voltaje y Corriente si el sistema está activo
        // Bit 0: Voltaje, Bit 1: Corriente
        activate_byte |= (1 << 0);
        activate_byte |= (1 << 1);
    }
    // Añadir los sensores externos que tengan datos nuevos
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        if (datos_externos[i].is_new) {
            activate_byte |= (1 << (i + 2)); // i=0 es sensor 2, i=1 es sensor 3, etc.
        }
    }
    payload.push_back(activate_byte);
    payload.push_back(nivel_bateria); // La batería siempre va

    // ================== 2. DESCRIPTORES DE LONGITUD ==================
    // Se añaden en orden de los bits de 'Activate'
    if ((activate_byte >> 0) & 0x01) { // Voltaje
        uint8_t len_byte = (data_len_rms & 0x1F); // No packed, no extended
        payload.push_back(len_byte);
    }
    if ((activate_byte >> 1) & 0x01) { // Corriente
        uint8_t len_byte = 0x80 | (data_len_rms & 0x1F); // Packed, no extended
        payload.push_back(len_byte);
    }
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        if (datos_externos[i].is_new) {
            uint8_t len_byte = 0;
            if (datos_externos[i].packed)   len_byte |= 0x80;
            if (datos_externos[i].extended) len_byte |= 0x40;
            // Para datos simples, la longitud suele ser 1
            len_byte |= (1 & 0x1F); 
            payload.push_back(len_byte);
        }
    }

    // ================== 3. BLOQUES DE DATOS ==================
    BitPacker packer;
    // --- Datos de Voltaje (si está activo) ---
    if ((activate_byte >> 0) & 0x01) {
        // ASUNCIÓN: pin_configs[0,1,2] son voltajes. Se rellenan con 0 si no están.
        const int pines_voltaje_idx[] = {0, 1, 2}; // Índices en pin_configs
        for (int ch = 0; ch < 3; ++ch) {
            int idx = pines_voltaje_idx[ch];
            for (int k = 0; k < data_len_rms; ++k) {
                uint8_t valor = 0;
                if (pin_configs[idx].enabled && !isnan(buffer.bloque[k].valores[idx])) {
                    valor = (uint8_t)round(buffer.bloque[k].valores[idx]);
                }
                payload.push_back(valor);
            }
        }
    }
    // --- Datos de Corriente (si está activo) ---
    if ((activate_byte >> 1) & 0x01) {
        // ASUNCIÓN: pin_configs[3] es la única corriente. Rellenamos 2 canales con ceros.
        int idx_corriente = 3; 
        // Canal 1 (real)
        for (int k = 0; k < data_len_rms; ++k) {
            uint16_t valor = 0;
            if (pin_configs[idx_corriente].enabled && !isnan(buffer.bloque[k].valores[idx_corriente])) {
                valor = (uint16_t)round(buffer.bloque[k].valores[idx_corriente] * 10.0f);
            }
            packer.push(valor, 10, payload);
        }
        // Canales 2 y 3 (relleno con ceros)
        for (int ch = 0; ch < 2; ++ch) {
            for (int k = 0; k < data_len_rms; ++k) {
                packer.push(0, 10, payload);
            }
        }
        packer.flush(payload);
    }
    // --- Datos de Sensores Externos ---
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        if (datos_externos[i].is_new) {
            payload.insert(payload.end(), datos_externos[i].data, datos_externos[i].data + datos_externos[i].len);
        }
    }

    // Empaquetar en fragmentos
    Fragmento f;
    f.len = payload.size();
    memcpy(f.data, payload.data(), f.len);
    fragmentos.push_back(f);
}


// ===================== ISR de muestreo ADC =====================
hw_timer_t *adcTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int isr_pin_index = 0;
volatile int num_pines_activos = 0;

// Cálculo RMS usando sumas acumuladas FIFO
float calculateRMS_fifo(const RMS_FIFO& fifo, float gain = 1.0f) {
    int cnt;
    double sx, sx2;

    portENTER_CRITICAL(&timerMux);
    cnt = fifo.count;
    sx  = fifo.sum_x;
    sx2 = fifo.sum_x2;
    portEXIT_CRITICAL(&timerMux);

    if (cnt <= 0) return NAN;
    double mean = sx / cnt;
    double var  = (sx2 / cnt) - (mean * mean);
    if (var < 0) var = 0;           // clamp por errores numéricos
    double rms = sqrt(var) * (3.3 / 4095.0);
    return (float)(rms * gain);
}

void actualizarNumPinesActivos() {
    num_pines_activos = 0;
    for (int i = 0; i < NUM_PINES; i++) {
        if (pin_configs[i].enabled) num_pines_activos++;
    }
}

void IRAM_ATTR onADCTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    if (!sistema_habilitado || num_pines_activos == 0) {
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }
    // Busca el siguiente pin activo
    int intentos = 0;
    while (!pin_configs[isr_pin_index].enabled && intentos < NUM_PINES) {
        isr_pin_index = (isr_pin_index + 1) % NUM_PINES;
        intentos++;
    }
    if (pin_configs[isr_pin_index].enabled) {
        uint16_t val = pin_configs[isr_pin_index].reader.readRaw();
        RMS_FIFO& fifo = fifo_pins[isr_pin_index];
        int h = fifo.head;
        if (fifo.count == FIFO_SIZE) {
            uint16_t old = fifo.buffer[h];
            fifo.sum_x -= old;
            fifo.sum_x2 -= (double)old * old;
        } else {
            fifo.count++;
        }
        fifo.buffer[h] = val;
        fifo.sum_x += val;
        fifo.sum_x2 += (double)val * val;
        fifo.head = (h + 1) % FIFO_SIZE;
    }
    isr_pin_index = (isr_pin_index + 1) % NUM_PINES;
    portEXIT_CRITICAL_ISR(&timerMux);
}


void TaskProcesamiento(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PROCESS_PERIOD_MS);

    while (true) {
        vTaskDelayUntil(&lastWakeTime, xFrequency);

        if (!sistema_habilitado) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        ResultadoRMS resultado;
        resultado.timestamp = millis();

        for (int i = 0; i < NUM_PINES; i++) {
            if (pin_configs[i].enabled && fifo_pins[i].count > 0) {
                float rms_value = calculateRMS_fifo(fifo_pins[i], pin_configs[i].gain);
                float rms_salida;
                if (!pin_configs[i].ema_initialized) {
                    pin_configs[i].ema_output = rms_value;
                    pin_configs[i].ema_initialized = true;
                    rms_salida = rms_value;
                } else {
                    rms_salida = applyEMA(rms_value, pin_configs[i].ema_output);
                }
                pin_configs[i].last_rms = rms_salida;
                resultado.valores[i] = rms_salida;
            } else {
                resultado.valores[i] = NAN;
            }
        }
        xQueueSend(queueResultados, &resultado, portMAX_DELAY);
    }
}

void TaskRegistroResultados(void *pvParameters) {
    (void)pvParameters;
    bufferResultados.index = 0;
    static uint8_t id_mensaje = 0;
    static uint8_t ultima_bateria = 0xFF; // valor inicial "desconocido"
    unsigned long tiempo_ultima_muestra = 0;
    ExternalSensorData datos_externos_para_envio[MAX_SENSORES_EXTERNOS];

    while (true) {
        ResultadoRMS resultado;
        if (xQueueReceive(queueResultados, &resultado, pdMS_TO_TICKS(500)) == pdTRUE) {
            bufferResultados.bloque[bufferResultados.index++] = resultado;

            // Reinicio por inactividad
            if (sistema_habilitado && millis() - tiempo_ultima_muestra > 30000) {
                Serial.println("[BUFFER] Reiniciando por inactividad");
                bufferResultados.index = 0;
            }
            tiempo_ultima_muestra = millis();

            if (sistema_habilitado && bufferResultados.index >= RESULTADOS_POR_BLOQUE) {
                // ===== Sistema habilitado: bloque completo =====
                id_mensaje = (id_mensaje + 1) % 256;

                // Checar si hay batería nueva
                bool nueva_bateria = false;
                uint8_t nivel_bateria = 0;
                xSemaphoreTake(mutex_bateria, portMAX_DELAY);
                if (index_bateria > 0) {
                    nueva_bateria = true;
                    nivel_bateria = buffer_bateria[0].nivel_codificado;
                    index_bateria = 0;
                }

                if (nueva_bateria) {
                    ultima_bateria = nivel_bateria;
                }
                xSemaphoreGive(mutex_bateria);




                // <<< NUEVO: Recolectar datos de sensores externos >>>
                //ExternalSensorData datos_externos_para_envio[MAX_SENSORES_EXTERNOS];
                for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
                    datos_externos_para_envio[i].is_new = false; // Por defecto, no hay datos nuevos
                    if (xSemaphoreTake(mutex_external_sensors[i], pdMS_TO_TICKS(10)) == pdTRUE) {
                        if (external_sensors[i].is_new) {
                            memcpy(&datos_externos_para_envio[i], &external_sensors[i], sizeof(ExternalSensorData));
                            external_sensors[i].is_new = false; // Marcar como "consumido"
                        }
                        xSemaphoreGive(mutex_external_sensors[i]);
                    }
                }

                std::vector<Fragmento> frags;
                codificarUnificado(
                    bufferResultados, 
                    id_mensaje, 
                    nueva_bateria, 
                    nueva_bateria ? nivel_bateria : ultima_bateria, 
                    RESULTADOS_POR_BLOQUE, 
                    true, // sistema_habilitado
                    datos_externos_para_envio, // <<< Pasamos los nuevos datos
                    frags
                );

                for (auto& f : frags) {
                    xQueueSend(queueFragmentos, &f, portMAX_DELAY);
                }

                // <<< CAMBIO: Preparar y enviar datos a la tarea de display
                DisplayInfo info = {};
                info.timestamp_s = bufferResultados.bloque[RESULTADOS_POR_BLOQUE - 1].timestamp / 1000;
                info.sistema_activo = true;
                info.bateria_incluida = nueva_bateria;
                if (nueva_bateria) {
                    info.valor_bateria = nivel_bateria / 10.0f;
                }
                // Extraer primer valor de Corriente y Voltaje para la pantalla
                for(int i = 0; i < NUM_PINES; ++i) {
                    if (pin_configs[i].pin == pines_corriente[0]) {
                        info.primer_valor_corriente = bufferResultados.bloque[0].valores[i];
                    }
                    if (pin_configs[i].pin == pines_voltaje[0]) { // Tomamos solo el primer pin de voltaje como referencia
                        info.primer_valor_voltaje1 = bufferResultados.bloque[0].valores[i];
                    }
                }
                xQueueSend(queueDisplayInfo, &info, 0); // Timeout 0 para no bloquear

                bufferResultados.index = 0;
            }
        }

        // ===== Sistema deshabilitado: solo batería =====
        if (!sistema_habilitado) {
            xSemaphoreTake(mutex_bateria, portMAX_DELAY);
            if (index_bateria > 0) {
                id_mensaje = (id_mensaje + 1) % 256;
                uint8_t nivel_bateria = buffer_bateria[0].nivel_codificado;
                index_bateria = 0;

                //ExternalSensorData datos_externos_para_envio[MAX_SENSORES_EXTERNOS];
                for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
                    datos_externos_para_envio[i].is_new = false; // Por defecto, no hay datos nuevos
                    if (xSemaphoreTake(mutex_external_sensors[i], pdMS_TO_TICKS(10)) == pdTRUE) {
                        if (external_sensors[i].is_new) {
                            memcpy(&datos_externos_para_envio[i], &external_sensors[i], sizeof(ExternalSensorData));
                            external_sensors[i].is_new = false; // Marcar como "consumido"
                        }
                        xSemaphoreGive(mutex_external_sensors[i]);
                    }
                }

                std::vector<Fragmento> frags;
                codificarUnificado(
                    bufferResultados, 
                    id_mensaje, 
                    true, 
                    nivel_bateria, 
                    RESULTADOS_POR_BLOQUE, 
                    false, // sistema_habilitado
                    datos_externos_para_envio, // <<< Pasamos los nuevos datos
                    frags
                );

                for (auto& f : frags) {
                    xQueueSend(queueFragmentos, &f, portMAX_DELAY);
                }
                //for (auto& f : frags) {
                //    xQueueSend(queueFragmentos, &f, portMAX_DELAY);
                //}

                // <<< CAMBIO: Preparar y enviar datos a la tarea de display (solo batería)
                DisplayInfo info = {};
                info.timestamp_s = millis() / 1000;
                info.sistema_activo = false;
                info.bateria_incluida = true;
                info.valor_bateria = nivel_bateria / 10.0f;
                xQueueSend(queueDisplayInfo, &info, 0);

            }
            xSemaphoreGive(mutex_bateria);
        }
    }
}

void TaskBatteryLevel(void *pvParameters) {
    (void)pvParameters;
    pinMode(BATTERY_PIN, INPUT);
    ESP32AnalogRead adc;
    adc.attach(BATTERY_PIN);
    const float voltage_divider_ratio = 51.0f / 11.0f;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(BATTERY_INTERVAL_MS));

        // Leer ADC y escalar (ejemplo genérico: divisor para 15V)
        uint16_t raw = adc.readRaw();
        float voltage = (raw / 4095.0f) * 3.3f * voltage_divider_ratio; // ← ajusta según divisor real
        uint8_t valor_codificado = (uint8_t)round(voltage * 10);

        ResultadoBattery res;
        res.timestamp = millis();
        res.nivel_codificado = valor_codificado;

        xSemaphoreTake(mutex_bateria, portMAX_DELAY);
        buffer_bateria[index_bateria++] = res;

        if (index_bateria >= BATTERY_BUFFER_SIZE) {
            // No se mandan fragmentos aquí.
            // Solo dejamos disponible la batería para TaskRegistroResultados.
            Serial.println("[BATTERY] Nueva muestra lista");
        }
        xSemaphoreGive(mutex_bateria);
    }
}

// <<< CAMBIO: Tarea de display completamente reescrita para ser más eficiente y clara
void TaskDisplay(void *pvParameters) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("No se detectó la pantalla OLED"));
        vTaskDelete(NULL);
    }

    display.clearDisplay();

    // === Logo 20x20 ===
    display.drawBitmap(0, 0, epd_bitmap_, 20, 20, SSD1306_WHITE);

    // === Nombre + versión al lado del logo ===
    display.setFont();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(26, 0);   // mover un poco a la derecha para no chocar con el logo
    display.print("NEMO");

    display.setTextSize(1);
    display.setCursor(100, 4);
    display.print("V1.0");

    // === Acrónimo expandido (TomThumb para que quepa todo) ===
    display.setFont(&TomThumb);
    display.setCursor(0, 28);
    display.print("Node for Electromechanical");
    display.setCursor(0, 40);
    display.print("Monitoring & Operation");

    // === Marca + año ===
    display.setFont();
    display.setTextSize(1);
    display.setCursor(65, 54);
    display.print("EMASA 2025");

    display.display();

    vTaskDelay(pdMS_TO_TICKS(1500));

    
    // Buffer local para mantener el historial de los últimos envíos
    static DisplayInfo historial_display[NUM_EVENTOS_LCD];
    static int idx_historial = 0;

    while (true) {
        DisplayInfo nuevo_evento;
        // Espera bloqueante hasta que llegue nueva información para mostrar
        if (xQueueReceive(queueDisplayInfo, &nuevo_evento, portMAX_DELAY) == pdTRUE) {
            
            // Añade el nuevo evento al historial circular
            historial_display[idx_historial] = nuevo_evento;
            idx_historial = (idx_historial + 1) % NUM_EVENTOS_LCD;

            // Actualiza la pantalla completa
            display.clearDisplay();

            // 1. Estado del sistema
            display.setCursor(0, 0);
            display.setTextSize(2);
            display.print(sistema_habilitado ? "ACTIVO" : "INACTIVO");

            // 2. Título de la sección de eventos
            display.setTextSize(1);
            display.setCursor(0, 20);
            display.print("Ultimos envios LoRa:");

            // 3. Itera sobre el historial para mostrar los eventos (del más nuevo al más viejo)
            int pos_actual = (idx_historial - 1 + NUM_EVENTOS_LCD) % NUM_EVENTOS_LCD;
            for (int i = 0; i < NUM_EVENTOS_LCD; i++) {
                display.setCursor(0, 32 + i * 10);
                DisplayInfo &evento = historial_display[pos_actual];

                // Si el timestamp es 0, es un registro vacío, no lo mostramos
                if (evento.timestamp_s > 0) {
                    if (evento.sistema_activo) {
                        display.printf("A T:%lus C:%.1fA V:%.0fV",
                                       evento.timestamp_s % 1000, // Mostramos últimos 3 dígitos para ahorrar espacio
                                       evento.primer_valor_corriente,
                                       evento.primer_valor_voltaje1);
                    } else { // Sistema inactivo, solo reportó batería
                         display.printf("I T:%lus Bat:%.1fV",
                                       evento.timestamp_s % 1000,
                                       evento.valor_bateria);
                    }
                }
                
                // Moverse al siguiente evento más antiguo en el buffer circular
                pos_actual = (pos_actual - 1 + NUM_EVENTOS_LCD) % NUM_EVENTOS_LCD;
            }

            display.display();
        }
    }
}
    
// ===================== LORA CALLBACKS Y TAREA =====================
volatile bool lora_tx_done = true;

void onEvent(ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        lora_tx_done = true;
        xSemaphoreGive(semaforoEnvio); // Libera el semáforo para el siguiente fragmento
        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println("[LORA] ACK recibido");
        } else {
            Serial.println("[LORA] TX completo, sin ACK");
        }
    }
}

void initLoRa() {
    os_init();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // 1% de tolerancia
    LMIC_reset();
    
    // ✅ Configuración específica para US915
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
    
    // ✅ Configuración US915 correcta
    LMIC_selectSubBand(7);

    // ✅ Data rates válidos para US915
    LMIC_setDrTxpow(US915_DR_SF7, 20); // DR3 = SF7BW125
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
    lora_tx_done = true;
}

// <<< CAMBIO: Tarea LoRa simplificada al máximo.
void tareaLoRa(void* pvParameters) {
    Fragmento frag;
    while (true) {
        // 1. Espera un fragmento para enviar
        if (xQueueReceive(queueFragmentos, &frag, portMAX_DELAY) == pdTRUE) {
            
            // 2. Toma el semáforo para asegurar que no se envíe nada más hasta que termine
            xSemaphoreTake(semaforoEnvio, portMAX_DELAY);
            lora_tx_done = false;
            
            // 3. Espera si la radio está ocupada
            while (LMIC.opmode & OP_TXRXPEND) {
                os_runloop_once();
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            
            // 4. Envía los datos
            LMIC_setTxData2(69, frag.data, frag.len, 0);
            Serial.printf("[LORA] Enviando paquete de %d bytes.\n", frag.len);

            // 5. Espera a que el envío termine (la ISR de onEvent liberará el semáforo)
            while (!lora_tx_done) {
                os_runloop_once();
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
        }
        // Ejecuta el runloop de LoRa periódicamente
        os_runloop_once();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void TaskMonitorPin(void *pvParameters) {
    pinMode(MONITOR_PIN, INPUT_PULLUP); // <<< CAMBIO: Usar PULLUP interno si es un switch a GND
    while (true) {
        // Asumiendo que un switch a GND activa el sistema (LOW = ACTIVO)
        sistema_habilitado = (digitalRead(MONITOR_PIN) == LOW); 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// ===================== SETUP Y LOOP =====================

void setup() {
    // --- Comunicación serie ---
    Serial.begin(921600);
    delay(1000);

    // --- Inicialización de lectores ADC ---
    for (int i = 0; i < NUM_PINES; i++) {
        pin_configs[i].reader.attach(pin_configs[i].pin);
    }

    // --- Inicialización de estructuras FIFO ---
    for (int i = 0; i < NUM_PINES; i++) {
        fifo_pins[i].head   = 0;
        fifo_pins[i].count  = 0;
        fifo_pins[i].sum_x  = 0;
        fifo_pins[i].sum_x2 = 0;
    }

    // --- Buffer de resultados ---
    bufferResultados.index = 0;
    bufferResultados.mutex = xSemaphoreCreateMutex();

    // --- Colas para resultados, fragmentos y display ---
    queueResultados = xQueueCreate(RESULTADOS_POR_BLOQUE * 2, sizeof(ResultadoRMS));
    queueFragmentos = xQueueCreate(10, sizeof(Fragmento));
    queueDisplayInfo = xQueueCreate(5, sizeof(DisplayInfo)); // <<< CAMBIO: Crear la nueva cola

    // --- Semáforo de envío LoRa ---
    semaforoEnvio = xSemaphoreCreateBinary();
    xSemaphoreGive(semaforoEnvio);  // Lo dejamos disponible al inicio

    // --- Configuración inicial de pines activos ---
    actualizarNumPinesActivos();
    isr_pin_index = 0;

    // --- Configuración del timer ADC ---
    int freq_isr = FS_HZ * num_pines_activos;
    if (freq_isr < 1) freq_isr = 1; // Frecuencia mínima de 1 Hz
    adcTimer = timerBegin(0, 80, true);   // Prescaler = 80 → tick de 1 µs (80 MHz / 80)
    timerAttachInterrupt(adcTimer, &onADCTimer, true);
    timerAlarmWrite(adcTimer, 1000000 / freq_isr, true);
    timerAlarmEnable(adcTimer);

    // --- Creación de tareas en FreeRTOS ---
    xTaskCreatePinnedToCore(TaskProcesamiento,      "Procesamiento",      4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskRegistroResultados, "RegistroResultados", 4096, NULL, 1, NULL, 0);

    // --- Tarea de monitoreo de batería ---
    mutex_bateria = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(TaskBatteryLevel, "BatteryLevel", 2048, NULL, 0, NULL, 0);

    // --- Tarea de monitor del pin ON/OFF ---
    xTaskCreatePinnedToCore(TaskMonitorPin, "MonitorPinTask", 2048, NULL, 1, NULL, 0);

    // --- Inicialización y tarea LoRa ---
    initLoRa();
    xTaskCreatePinnedToCore(tareaLoRa, "LoRaTask", 8192, NULL, 2, NULL, 1);

    // --- Tarea de display LCD ---
    // <<< CAMBIO: Aumenté ligeramente la memoria por el uso de printf
    xTaskCreatePinnedToCore(TaskDisplay, "DisplayTask", 2560, NULL, 1, NULL, 0);

    // --- Inicializa los mutex sensores externos ---
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        mutex_external_sensors[i] = xSemaphoreCreateMutex();
        external_sensors[i].is_new = false; // Asegurarse de que inician como leídos
    }
}  

void loop() {
    // El flujo principal está en las tareas (FreeRTOS).
    // El loop queda vacío.
    vTaskDelete(NULL); // Opcional: eliminar la tarea del loop de Arduino
}