#include <Arduino.h>
#include <Wire.h>  // Keep for potential future use, but not used in simulation
#include "HardwareSerial.h"
#include "ModbusServerRTU.h"

// =================================================================
// --- CONFIGURACIÓN COMBINADA ---
// =================================================================
// Configuración simulada (sin ADS1015)
const int NUM_CHANNELS = 3;
const int FIFO_SIZE = 320;
const int PROCESS_INTERVAL_MS = 300;
const int RMS_HISTORY_SIZE = 100;

// Configuración Modbus
#define SLAVE_ID 2  // Cambiado a 2
#define NUM_REGISTERS 45  // 5 por canal (3 canales x 5 = 15)
#define RX_PIN 16
#define TX_PIN 17
const int MODBUS_UPDATE_INTERVAL_MS = 300;

// Factor para convertir float RMS a uint16_t
const float CONVERSION_FACTOR = 100.0f;

// =================================================================
// --- ESTRUCTURAS DE DATOS Y VARIABLES GLOBALES ---
// =================================================================
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

// Buffers circulares para historial de RMS
float rms_history_ch0[RMS_HISTORY_SIZE];
float rms_history_ch1[RMS_HISTORY_SIZE];
float rms_history_ch2[RMS_HISTORY_SIZE];
volatile int rms_history_head = 0;
SemaphoreHandle_t rms_history_mutex;

// Variables para Modbus
ModbusServerRTU MBserver(2000);
uint16_t holdingRegisters[NUM_REGISTERS];
SemaphoreHandle_t dataMutex;
TaskHandle_t dataUpdateTaskHandle;

// =================================================================
// --- FUNCIONES SIMULADAS (Sin ADC real) ---
// =================================================================
void task_adquisicion(void *pvParameters) {
    // Simula la adquisición generando valores aleatorios para cada canal
    // En lugar de leer del ADC, genera datos simulados (ruido aleatorio entre -2048 y 2047)
    while (true) {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            // Generar valor simulado (puedes cambiar a sinusoidal o lo que prefieras)
            int16_t simulated_value = random(0, 2047);
            ADC_Sample sample = {simulated_value, ch};
            xQueueSend(queue_adc_samples, &sample, 0);
            // Pequeño delay para simular el tiempo entre lecturas por canal
            vTaskDelay(pdMS_TO_TICKS(10));
        }
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

int get_rms_history(int channel, float* output_buffer, int count) {
    if (count > RMS_HISTORY_SIZE || channel < 0 || channel >= NUM_CHANNELS) {
        return 0;
    }

    float* source_buffer;
    switch(channel) {
        case 0: source_buffer = rms_history_ch0; break;
        case 1: source_buffer = rms_history_ch1; break;
        case 2: source_buffer = rms_history_ch2; break;
        default: return 0;
    }

    if (xSemaphoreTake(rms_history_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int most_recent_idx = (rms_history_head == 0) ? (RMS_HISTORY_SIZE - 1) : (rms_history_head - 1);
        for (int i = 0; i < count; i++) {
            int source_idx = (most_recent_idx - (count - 1 - i) + RMS_HISTORY_SIZE) % RMS_HISTORY_SIZE;
            output_buffer[i] = source_buffer[source_idx];
        }
        xSemaphoreGive(rms_history_mutex);
        return count;
    }
    return 0;
}

// =================================================================
// --- FUNCIONES DEL SEGUNDO CÓDIGO (Modbus) ---
// =================================================================
// Tarea de actualización de datos
void dataUpdateTask(void *pvParameters) {
    Serial.println("Tarea de actualización de datos iniciada en el Núcleo 0.");

    // Dinámico: calcular registros por canal
    const int samples_per_channel = NUM_REGISTERS / NUM_CHANNELS;

    while (true) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            // Arrays para cada canal
            float rms_channel[NUM_CHANNELS][samples_per_channel];
            int counts[NUM_CHANNELS];

            // Obtener RMS para cada canal, dinámicamente
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                counts[ch] = get_rms_history(ch, rms_channel[ch], samples_per_channel);
            }

            // Llenar holdingRegisters dinámicamente
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                for (int i = 0; i < samples_per_channel; i++) {
                    int idx = ch * samples_per_channel + i;
                    holdingRegisters[idx] = (counts[ch] > i) ? (uint16_t)(rms_channel[ch][i] * CONVERSION_FACTOR) : 0;
                }
            }

            xSemaphoreGive(dataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(MODBUS_UPDATE_INTERVAL_MS));
    }
}

ModbusMessage readHoldingRegistersWorker(ModbusMessage request) {
    uint16_t address, words;
    ModbusMessage response;

    request.get(2, address);
    request.get(4, words);

    if (address >= NUM_REGISTERS || (address + words) > NUM_REGISTERS) {
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
        return response;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
        for (uint16_t i = 0; i < words; ++i) {
            response.add(holdingRegisters[address + i]);
        }
        xSemaphoreGive(dataMutex);
    } else {
        response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_BUSY);
    }
    return response;
}

// =================================================================
// --- SETUP Y LOOP ---
// =================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n===================================");
    Serial.println("Iniciando Sistema Simulado: Sin ADC + Esclavo Modbus RMS (Slave ID 2)");
    Serial.println("===================================");

    // Inicializar Serial2 para Modbus
    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);

    // Crear colas y mutexes
    queue_adc_samples = xQueueCreate(FIFO_SIZE, sizeof(ADC_Sample));
    rms_history_mutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    if (!rms_history_mutex || !dataMutex) {
        Serial.println("ERROR: No se pudieron crear los mutexes.");
        while (1);
    }

    // Registrar worker Modbus
    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.setModbusTimeout(2000);

    // Crear tareas
    xTaskCreatePinnedToCore(task_adquisicion, "TaskAdquisicion", 4096, NULL, 5, NULL, 1);  // Núcleo 1
    xTaskCreatePinnedToCore(task_procesamiento, "TaskProcesamiento", 4096, NULL, 3, NULL, 0);  // Núcleo 0
    MBserver.begin(Serial2, 0);  // Modbus en Núcleo 0
    xTaskCreatePinnedToCore(dataUpdateTask, "DataUpdateTask", 2048, NULL, 1, &dataUpdateTaskHandle, 0);  // Núcleo 0

    Serial.println("INFO: Setup completado.");
}

void loop() {
    // Loop libre en Núcleo 1
    vTaskDelay(pdMS_TO_TICKS(5000));
    Serial.println("Loop principal activo...");

    // Demo: Mostrar algunos valores RMS (opcional, para debug)
    const int num_datos = 5;
    float datos_ch1[num_datos];
    int obtenidos = get_rms_history(1, datos_ch1, num_datos);
    if (obtenidos > 0) {
        Serial.printf("Últimos %d RMS del Canal 1 (en unidades ADC):\n", obtenidos);
        for (int i = 0; i < obtenidos; i++) {
            Serial.printf("  %.3f\n", datos_ch1[i]);
        }
    }
}