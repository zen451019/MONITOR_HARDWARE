#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <ESP32AnalogRead.h>
#include <math.h>
#include <vector>
#include <stdint.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ==================== PARÁMETROS DEL PANTALLA ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // El pin reset no está conectado en el TTGO
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== PARÁMETROS DEL PIN_ACTIVACIÒN ====================
#define MONITOR_PIN 35
volatile bool sistema_habilitado = false;


// ==================== PARÁMETROS DEL SISTEMA ====================
// Permite configurar desde build flags, por ejemplo en platformio.ini:
// build_flags = -DFS_HZ=1000 -DWINDOW_MS=300 -DEMA_ALPHA=0.3 -DPROCESS_PERIOD_MS=300
#ifndef FS_HZ
#define FS_HZ 960
#endif
#ifndef WINDOW_MS
#define WINDOW_MS 16*15
#endif
#ifndef EMA_ALPHA
#define EMA_ALPHA 0.08f
#endif
#ifndef PROCESS_PERIOD_MS
#define PROCESS_PERIOD_MS 300
#endif

const int SAMPLING_FREQUENCY = FS_HZ; // Hz
const int BUFFER_SIZE = (FS_HZ * WINDOW_MS) / 1000; // muestras por ventana
#define NUM_PINES 4
#define RESULTADOS_POR_BLOQUE 50
#define RMS_SCALE 10

#define LORA_PAYLOAD_MAX 180 // Payload máximo para DR3


// ==================== REGISTRO PARA LCD ====================
#define NUM_EVENTOS_LCD 3

struct EventoLCD {
    char tipo; // 'C', 'V', 'B'
    unsigned long tiempo; // en segundos
    uint8_t frag_idx; // índice de fragmento (0 = primero)
    int first_value; // primer valor codificado del bloque (o -1 si no aplica)
};

volatile EventoLCD eventos_lcd[NUM_EVENTOS_LCD];
volatile int idx_evento_lcd = 0;
volatile bool lcd_needs_update = false;

// ==================== BATTERY LEVEL CONFIG ====================
#define BATTERY_PIN 14              // Cambia esto según tu conexión
#define BATTERY_BUFFER_SIZE 1     // Muestras por paquete
#define BATTERY_INTERVAL_MS 60000   // 30 segundos

struct ResultadoBattery {
    unsigned long timestamp;
    uint8_t nivel_codificado;
};

ResultadoBattery buffer_bateria[BATTERY_BUFFER_SIZE];
volatile int index_bateria = 0;
uint8_t id_serie_bateria = 0;
SemaphoreHandle_t mutex_bateria;


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
  PinConfig *pins;
  int num_pins;
  volatile bool acquisition_complete;
  SemaphoreHandle_t semaforo;
  TaskHandle_t taskHandle;
  unsigned long sequence_time_us;
} MultiPinSystem;

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
#define FIFO_SIZE 200
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
    .rst = 14,
    .dio = {26, 33, 32}
};

static osjob_t sendjob;

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

// ===================== CODIFICACIÓN Y FRAGMENTACIÓN =====================
void codificarYFragmentar(
    const BufferResultados& buffer,
    uint8_t tipo_sensor, // 1 = corriente, 2 = voltaje
    uint8_t id_serie,
    std::vector<Fragmento>& fragmentos
) {
    uint8_t datos[NUM_PINES * RESULTADOS_POR_BLOQUE * 2]; // Tamaño máximo requerido
    size_t datos_len = 0;

    const int* pines;
    int num_canales;

    if (tipo_sensor == 1) {
        pines = pines_corriente;
        num_canales = sizeof(pines_corriente) / sizeof(pines_corriente[0]);
    } else {
        pines = pines_voltaje;
        num_canales = sizeof(pines_voltaje) / sizeof(pines_voltaje[0]);
    }

    // Por sensor primero, luego por muestra
    for (int i = 0; i < num_canales; ++i) {
        int idx = -1;
        for (int j = 0; j < NUM_PINES; ++j)
            if (pin_configs[j].pin == pines[i]) {
                idx = j;
                break;
            }

        for (int k = 0; k < RESULTADOS_POR_BLOQUE; ++k) {
            if (tipo_sensor == 1) {
                uint16_t valor = 0;
                if (idx != -1 && !isnan(buffer.bloque[k].valores[idx]))
                    valor = (uint16_t)round(buffer.bloque[k].valores[idx] * 10.0f);
                datos[datos_len++] = (valor >> 8) & 0xFF;
                datos[datos_len++] = valor & 0xFF;
            } else {
                uint8_t valor = 0;
                if (idx != -1 && !isnan(buffer.bloque[k].valores[idx]))
                    valor = (uint8_t)round(buffer.bloque[k].valores[idx]);
                datos[datos_len++] = valor; 
            }
        }
    }

    // Fragmentación
    const size_t CABECERA = 8;
    size_t max_datos_por_fragmento = LORA_PAYLOAD_MAX - CABECERA;
    size_t total_fragmentos = (datos_len + max_datos_por_fragmento - 1) / max_datos_por_fragmento;

    for (size_t frag = 0; frag < total_fragmentos; ++frag) {
        Fragmento f;
        size_t offset = frag * max_datos_por_fragmento;
        size_t frag_len = (datos_len - offset > max_datos_por_fragmento) ? max_datos_por_fragmento : (datos_len - offset);

        // Cabecera
        f.data[0] = tipo_sensor;
        f.data[1] = id_serie;
        f.data[2] = frag;
        f.data[3] = total_fragmentos;
        unsigned long ts = buffer.bloque[0].timestamp;
        for (int i = 0; i < 4; ++i)
            f.data[4 + i] = (ts >> (8 * i)) & 0xFF;

        memcpy(f.data + CABECERA, datos + offset, frag_len);
        f.len = CABECERA + frag_len;
        fragmentos.push_back(f);
    }
}


void codificarYFragmentarBattery(
    ResultadoBattery* buffer,
    int num_muestras,
    uint8_t id_serie,
    std::vector<Fragmento>& fragmentos
) {
    const size_t CABECERA = 8;
    size_t max_datos_por_fragmento = LORA_PAYLOAD_MAX - CABECERA;
    size_t total_fragmentos = (num_muestras + max_datos_por_fragmento - 1) / max_datos_por_fragmento;

    for (size_t frag = 0; frag < total_fragmentos; ++frag) {
        Fragmento f;
        size_t offset = frag * max_datos_por_fragmento;
        size_t frag_len = (num_muestras - offset > max_datos_por_fragmento) ?
                          max_datos_por_fragmento : (num_muestras - offset);

        // Cabecera: tipo_sensor = 3 (batería)
        f.data[0] = 3;
        f.data[1] = id_serie;
        f.data[2] = frag;
        f.data[3] = total_fragmentos;

        // Timestamp de la primera muestra del bloque
        unsigned long ts = buffer[0].timestamp;
        for (int i = 0; i < 4; ++i)
            f.data[4 + i] = (ts >> (8 * i)) & 0xFF;

        for (size_t i = 0; i < frag_len; ++i)
            f.data[CABECERA + i] = buffer[offset + i].nivel_codificado;

        f.len = CABECERA + frag_len;
        fragmentos.push_back(f);
    }
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
    if (var < 0) var = 0;               // clamp por errores numéricos
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
    static uint8_t id_serie_corriente = 0;
    static uint8_t id_serie_voltaje = 0;
    unsigned long tiempo_ultima_muestra = 0;

    while (true) {
        ResultadoRMS resultado;
        if (xQueueReceive(queueResultados, &resultado, portMAX_DELAY) == pdTRUE) {
          
            //if (bufferResultados.index == 0) {
            //    tiempo_ultima_muestra = millis();
            //    }
                
            bufferResultados.bloque[bufferResultados.index++] = resultado;

        // Si pasan más de 30 segundos sin completar el bloque, reiniciar
        if (sistema_habilitado && millis() - tiempo_ultima_muestra > 30000) {
            Serial.println("[BUFFER] Reiniciando por inactividad");
            bufferResultados.index = 0;
        }

        tiempo_ultima_muestra = millis();

            if (bufferResultados.index >= RESULTADOS_POR_BLOQUE) {
                // Corriente
                std::vector<Fragmento> frags_corriente;
                id_serie_corriente = (id_serie_corriente + 1) % 256;
                codificarYFragmentar(bufferResultados, 1, id_serie_corriente, frags_corriente);
                Serial.print("[CORRIENTE] Fragmentos generados: ");
                Serial.println(frags_corriente.size());
                for (auto& f : frags_corriente) {
                    xQueueSend(queueFragmentos, &f, portMAX_DELAY);
                }
                // Voltaje
                std::vector<Fragmento> frags_voltaje;
                id_serie_voltaje = (id_serie_voltaje + 1) % 256;
                codificarYFragmentar(bufferResultados, 2, id_serie_voltaje, frags_voltaje);
                Serial.print("[VOLTAJE] Fragmentos generados: ");
                Serial.println(frags_voltaje.size());
                for (auto& f : frags_voltaje) {
                    xQueueSend(queueFragmentos, &f, portMAX_DELAY);
                }
                bufferResultados.index = 0;
            }
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
        float voltage = (raw / 4095.0f) * 3.3f * voltage_divider_ratio;  // ← ajusta según divisor real
        //if (voltage > 15.0f) voltage = 15.0f;
        //if (voltage < 0.0f) voltage = 0.0f;
        uint8_t valor_codificado = (uint8_t)round(voltage * 10);

        ResultadoBattery res;
        res.timestamp = millis();
        res.nivel_codificado = valor_codificado;

        xSemaphoreTake(mutex_bateria, portMAX_DELAY);
        buffer_bateria[index_bateria++] = res;

        if (index_bateria >= BATTERY_BUFFER_SIZE) {
            std::vector<Fragmento> fragmentos_bateria;
            id_serie_bateria = (id_serie_bateria + 1) % 256;
            codificarYFragmentarBattery(buffer_bateria, BATTERY_BUFFER_SIZE, id_serie_bateria, fragmentos_bateria);
            Serial.print("[BATTERY] Fragmentos generados: ");
            Serial.println(fragmentos_bateria.size());

            for (auto& f : fragmentos_bateria) {
                xQueueSend(queueFragmentos, &f, portMAX_DELAY);
            }
            index_bateria = 0;
        }

        xSemaphoreGive(mutex_bateria);
    }
}

void TaskDisplay(void *pvParameters) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("No se detectó la pantalla OLED"));
        vTaskDelete(NULL);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.print("VIVA CRISTO REY");
    display.display();
    vTaskDelay(pdMS_TO_TICKS(500));
    

    while (true) {
        display.clearDisplay();

        // Estado
        display.setCursor(0, 0);
        display.setTextSize(2);
        display.print(sistema_habilitado ? "ACTIVO" : "INACTIVO");

        // Registro de eventos
        display.setTextSize(1);
        display.setCursor(0, 20);
        display.print("Ultimos envios:");

        int pos = idx_evento_lcd;
        for (int i = 0; i < NUM_EVENTOS_LCD; i++) {
            pos = (pos - 1 + NUM_EVENTOS_LCD) % NUM_EVENTOS_LCD;
            char tipo = eventos_lcd[pos].tipo;
            unsigned long t = eventos_lcd[pos].tiempo;
            display.setCursor(0, 32 + i * 10);
            if (tipo != 0) {
            // Nota: frag_idx=0 indica primer fragmento del bloque, donde est�e1 el primer valor

                // Mostrar el primer valor si fue el primer fragmento del bloque
                if (eventos_lcd[pos].frag_idx == 0 && eventos_lcd[pos].first_value >= 0) {
                    if (tipo == 'C') {
                        // Corriente viene *10, mostramos con un decimal
                        float fv = eventos_lcd[pos].first_value / 10.0f;
                        display.printf("%c t=%lus v=%.1f", tipo, t, fv);
                    } else if (tipo == 'V' || tipo == 'B') {
                        // Voltaje y Batera son enteros codificados
                        display.printf("%c t=%lus v=%d", tipo, t, eventos_lcd[pos].first_value);
                    } else {
                        display.printf("%c t=%lus", tipo, t);
                    }
                } else {
                    display.printf("%c t=%lus", tipo, t);
                }
            }
        }

        display.display();
        vTaskDelay(pdMS_TO_TICKS(500));
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
    LMIC_selectSubBand(7); // Subband 1 (canales 0-7 + 64) - más común
    // Alternativas: subband 2-8 según tu gateway
    
    // ✅ Data rates válidos para US915
    LMIC_setDrTxpow(US915_DR_SF7, 20); // DR0 = SF10BW125 (más confiable)
    // Otras opciones: DR_SF9, DR_SF8, DR_SF7 (pero verifica compatibilidad)
    
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
    
    // ✅ Configuración adicional recomendada para US915
    /*
    for (int i = 0; i < 72; i++) {
        if (i != 56 && i != 57 && i != 58 && i != 59 && 
            i != 60 && i != 61 && i != 62 && i != 63 && i != 65) {
            LMIC_disableChannel(i);
        }
    }
    */

    lora_tx_done = true;
}
            // Guardamos �ndices y primer valor para mostrar en OLED

void tareaLoRa(void* pvParameters) {
    Fragmento frag;
    static int enviados_corriente = 0;
    static int enviados_voltaje = 0;
    while (true) {
        if (xQueueReceive(queueFragmentos, &frag, portMAX_DELAY) == pdTRUE) {
            xSemaphoreTake(semaforoEnvio, portMAX_DELAY);
            lora_tx_done = false;
            while (LMIC.opmode & OP_TXRXPEND) {
                os_runloop_once();
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            
            LMIC_setTxData2(1, frag.data, frag.len, 0);

            // Registrar el evento para la pantalla LCD, incluyendo primer valor del bloque
            char tipo = '?';
            int first_value = -1; // -1 = no aplica / desconocido
            if (frag.data[0] == 1) {
                tipo = 'C';
                // Corriente: valores codificados en 2 bytes (big-endian en nuestro armado: alto luego bajo)
                if (frag.data[2] == 0 && frag.len > 9) {
                    // CABECERA=7, primer dato empieza en 7
                    uint16_t v = ((uint16_t)frag.data[8] << 9) | frag.data[9];
                    first_value = (int)v; // ya viene escalado *10 en codificacion
                }
            } else if (frag.data[0] == 2) {
                tipo = 'V';
                // Voltaje: valores de 1 byte
                if (frag.data[2] == 0 && frag.len > 8) {
                    first_value = (int)frag.data[8];
                }
            } else if (frag.data[0] == 3) {
                tipo = 'B';
                // Bateria: valores de 1 byte
                if (frag.data[2] == 0 && frag.len > 8) {
                    first_value = (int)frag.data[8];
                }
            }

            eventos_lcd[idx_evento_lcd].tipo = tipo;
            eventos_lcd[idx_evento_lcd].tiempo = millis() / 1000;
            eventos_lcd[idx_evento_lcd].frag_idx = frag.data[2];
            eventos_lcd[idx_evento_lcd].first_value = first_value;
            idx_evento_lcd = (idx_evento_lcd + 1) % NUM_EVENTOS_LCD;
            lcd_needs_update = true;

            // Espera a que el envío termine
            while (!lora_tx_done) {
                os_runloop_once();
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
        }
        os_runloop_once();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
void TaskMonitorPin(void *pvParameters) {
    pinMode(MONITOR_PIN, INPUT);
    while (true) {
        sistema_habilitado = (digitalRead(MONITOR_PIN) == LOW); // HIGH = habilitado, LOW = deshabilitado
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// ===================== SETUP Y LOOP =====================


void setup() {
    Serial.begin(921600);
    delay(1000);

    // Inicializa los lectores ADC
    for (int i = 0; i < NUM_PINES; i++)
    pin_configs[i].reader.attach(pin_configs[i].pin);

    // Inicializa las estructuras FIFO
    for (int i = 0; i < NUM_PINES; i++) {
    fifo_pins[i].head = 0;
    fifo_pins[i].count = 0;
    fifo_pins[i].sum_x = 0;
    fifo_pins[i].sum_x2 = 0;
    }

    bufferResultados.index = 0;
    bufferResultados.mutex = xSemaphoreCreateMutex();

    queueResultados = xQueueCreate(RESULTADOS_POR_BLOQUE * 2, sizeof(ResultadoRMS));
    queueFragmentos = xQueueCreate(10, sizeof(Fragmento));

    semaforoEnvio = xSemaphoreCreateBinary();
    xSemaphoreGive(semaforoEnvio);

    actualizarNumPinesActivos();
    isr_pin_index = 0;
    // La frecuencia del timer depende de la cantidad de pines activos
    int freq_isr = FS_HZ * num_pines_activos;
    if (freq_isr < 1) freq_isr = 1;
    adcTimer = timerBegin(0, 80, true); // 80 prescaler: 1us tick (80MHz/80)
    timerAttachInterrupt(adcTimer, &onADCTimer, true);
    timerAlarmWrite(adcTimer, 1000000 / freq_isr, true);
    timerAlarmEnable(adcTimer);

    xTaskCreatePinnedToCore(TaskProcesamiento, "Procesamiento", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskRegistroResultados, "RegistroResultados", 4096, NULL, 1, NULL, 0);

    mutex_bateria = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(TaskBatteryLevel, "BatteryLevel", 2048, NULL, 0, NULL, 0);

    xTaskCreatePinnedToCore(
        TaskMonitorPin,
        "MonitorPinTask",
        2048,
        NULL,
        1,
        NULL,
        0
    );

    initLoRa();
    xTaskCreatePinnedToCore(
        tareaLoRa,
        "LoRaTask",
        8192,
        NULL,
        2,
        NULL,
        1
    );
    xTaskCreatePinnedToCore(
        TaskDisplay,
        "DisplayTask",
        2048,
        NULL,
        1,
        NULL,
        0
    );
    // Inicializar registro LCD
    for (int i = 0; i < NUM_EVENTOS_LCD; i++) {
        eventos_lcd[i].tipo = 0;
        eventos_lcd[i].tiempo = 0;
        eventos_lcd[i].frag_idx = 255;
        eventos_lcd[i].first_value = -1;
    }
}

void loop() {
  // Todo está en las tareas
}