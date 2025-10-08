#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// =================================================================
// --- CONFIGURACIÓN DEL SISTEMA ---
// =================================================================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define ADS_ALERT_PIN 4
 
const int NUM_CHANNELS = 3;
const int FIFO_SIZE = 320;
const int PROCESS_INTERVAL_MS = 300;
const int RMS_HISTORY_SIZE = 100;

// =================================================================
// --- ESTRUCTURAS DE DATOS Y VARIABLES GLOBALES ---
// =================================================================
Adafruit_ADS1015 ads;

struct ADC_Sample {
    int16_t value;
    uint8_t channel;
};

QueueHandle_t queue_adc_samples;

struct RMS_FIFO {
    int16_t buffer[FIFO_SIZE];
    int head = 0;
    int count = 0;
    int64_t sum_x = 0;
    int64_t sum_x2 = 0;
};
RMS_FIFO fifos[NUM_CHANNELS];

// --- Buffers circulares (arrays) para el historial de RMS ---
float rms_history_ch0[RMS_HISTORY_SIZE];
float rms_history_ch1[RMS_HISTORY_SIZE];
float rms_history_ch2[RMS_HISTORY_SIZE];
volatile int rms_history_head = 0; // Puntero a la PRÓXIMA posición a escribir
SemaphoreHandle_t rms_history_mutex; // Mutex para proteger el acceso

volatile bool adc_data_ready = false;
volatile uint8_t current_isr_channel = 0;

// =================================================================
// --- TAREAS Y FUNCIONES (Idénticas a la v3) ---
// =================================================================
void IRAM_ATTR on_adc_data_ready() { adc_data_ready = true; }

void task_adquisicion(void *pvParameters) {
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
    while (true) {
        if (adc_data_ready) {
            adc_data_ready = false;
            ADC_Sample sample = {ads.getLastConversionResults(), current_isr_channel};
            xQueueSend(queue_adc_samples, &sample, 0);
            current_isr_channel = (current_isr_channel + 1) % NUM_CHANNELS;
            ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0 + current_isr_channel, false);
        } else { vTaskDelay(1); }
    }
}

void task_procesamiento(void *pvParameters) {
    ADC_Sample sample;
    TickType_t last_process_time = xTaskGetTickCount();
    while (true) {
        while (xQueueReceive(queue_adc_samples, &sample, 0) == pdTRUE) {
            if (sample.channel < NUM_CHANNELS) {
                RMS_FIFO &fifo = fifos[sample.channel];
                if (fifo.count == FIFO_SIZE) {
                    int16_t old_val = fifo.buffer[fifo.head];
                    fifo.sum_x -= old_val;
                    fifo.sum_x2 -= (int64_t)old_val * old_val;
                } else { fifo.count++; }
                fifo.buffer[fifo.head] = sample.value;
                fifo.sum_x += sample.value;
                fifo.sum_x2 += (int64_t)sample.value * sample.value;
                fifo.head = (fifo.head + 1) % FIFO_SIZE;
            }
        }
        if (xTaskGetTickCount() - last_process_time >= pdMS_TO_TICKS(PROCESS_INTERVAL_MS)) {
            last_process_time = xTaskGetTickCount();
            float calculated_rms[NUM_CHANNELS] = {0};
            if (xSemaphoreTake(rms_history_mutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < NUM_CHANNELS; i++) {
                    if (fifos[i].count > 0) {
                        double mean = (double)fifos[i].sum_x / fifos[i].count;
                        double var = ((double)fifos[i].sum_x2 / fifos[i].count) - (mean * mean);
                        calculated_rms[i] = sqrt(var < 0 ? 0 : var);
                    }
                }
                rms_history_ch0[rms_history_head] = calculated_rms[0];
                rms_history_ch1[rms_history_head] = calculated_rms[1];
                rms_history_ch2[rms_history_head] = calculated_rms[2];
                rms_history_head = (rms_history_head + 1) % RMS_HISTORY_SIZE;
                xSemaphoreGive(rms_history_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =================================================================
// --- FUNCIÓN DE AYUDA PARA TU TAREA FUTURA ---
// =================================================================

/**
 * @brief Obtiene los últimos 'count' valores de historial para un canal.
 * @param channel El canal a consultar (0, 1, o 2).
 * @param output_buffer Un array donde se guardarán los resultados.
 * @param count El número de valores a obtener (debe ser <= que el tamaño del buffer).
 * @return El número de valores realmente copiados.
 */
int get_rms_history(int channel, float* output_buffer, int count) {
    if (count > RMS_HISTORY_SIZE || channel < 0 || channel >= NUM_CHANNELS) {
        return 0; // Petición inválida
    }

    float* source_buffer;
    switch(channel) {
        case 0: source_buffer = rms_history_ch0; break;
        case 1: source_buffer = rms_history_ch1; break;
        case 2: source_buffer = rms_history_ch2; break;
        default: return 0;
    }

    if (xSemaphoreTake(rms_history_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // El 'head' apunta a la próxima posición a escribir (la más antigua).
        // El dato más reciente está en la posición justo anterior al 'head'.
        int most_recent_idx = (rms_history_head == 0) ? (RMS_HISTORY_SIZE - 1) : (rms_history_head - 1);

        // Copiamos los datos al buffer de salida en el orden que tú quieres:
        // El más antiguo de la selección en la posición [0] y el más reciente en la [count-1].
        for (int i = 0; i < count; i++) {
            // Índice en el buffer de origen (circular)
            int source_idx = (most_recent_idx - (count - 1 - i) + RMS_HISTORY_SIZE) % RMS_HISTORY_SIZE;
            // Índice en el buffer de destino (lineal)
            output_buffer[i] = source_buffer[source_idx];
        }

        xSemaphoreGive(rms_history_mutex);
        return count;
    }
    
    return 0; // No se pudo obtener el mutex
}


// =================================================================
// --- SETUP Y LOOP ---
// =================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n===================================");
    Serial.println("Iniciando Procesador RMS con ADS1015 (v4 - Con Función de Ayuda)");
    Serial.println("===================================");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000L);
    if (!ads.begin()) { while (1); }
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(RATE_ADS1015_3300SPS);

    queue_adc_samples = xQueueCreate(FIFO_SIZE, sizeof(ADC_Sample));
    rms_history_mutex = xSemaphoreCreateMutex();
    
    pinMode(ADS_ALERT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ADS_ALERT_PIN), on_adc_data_ready, FALLING);
    ads.startComparator_SingleEnded(0, 1000);

    xTaskCreatePinnedToCore(task_adquisicion, "TaskAdquisicion", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_procesamiento, "TaskProcesamiento", 4096, NULL, 3, NULL, 0);

    Serial.println("INFO: Setup completado.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));

    Serial.println("\n--- DEMOSTRACIÓN PARA LA TAREA FUTURA ---");

    // --- Quiero los 5 datos más recientes del canal 1 ---
    const int num_datos_deseados = 5;
    float mis_datos[num_datos_deseados];

    int datos_obtenidos = get_rms_history(1, mis_datos, num_datos_deseados);

    const float FACTOR_ADC_TO_VOLTIOS = 6.144 / 2048.0;

    if (datos_obtenidos > 0) {
        Serial.printf("Se obtuvieron %d datos del historial del Canal 1 (en voltios):\n", datos_obtenidos);
        for (int i = 0; i < datos_obtenidos; i++) {
            float voltios = mis_datos[i] * FACTOR_ADC_TO_VOLTIOS;
            Serial.printf("  mi_array[%d] = %.3f V\n", i, voltios);
        }
        float voltios_reciente = mis_datos[datos_obtenidos - 1] * FACTOR_ADC_TO_VOLTIOS;
        Serial.printf("-> El dato más reciente (mis_datos[%d]) es: %.3f V\n", datos_obtenidos - 1, voltios_reciente);
    } else {
        Serial.println("No se pudieron obtener los datos del historial.");
    }

    // --- Quiero solo el dato más reciente del canal 0 ---
    float dato_unico[1];
    if (get_rms_history(0, dato_unico, 1) > 0) {
        float voltios = dato_unico[0] * FACTOR_ADC_TO_VOLTIOS;
        Serial.printf("\nEl valor más reciente del Canal 0 es: %.3f V\n", voltios);
    }
}