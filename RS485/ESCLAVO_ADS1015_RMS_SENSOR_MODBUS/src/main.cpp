#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "HardwareSerial.h"
#include "ModbusServerRTU.h"

// Define un objeto HardwareSerial para el UART2
HardwareSerial ModbusSerial(1);

// =================================================================
// --- CONFIGURACIÓN COMBINADA ---
// =================================================================
// Configuración ADS1015
// #define I2C_SDA_PIN 21
// #define I2C_SCL_PIN 22
// #define ADS_ALERT_PIN 4
#define I2C_SDA_PIN 9
#define I2C_SCL_PIN 8
#define ADS_ALERT_PIN 10
const int NUM_CHANNELS = 3;
const int FIFO_SIZE = 320;
const int PROCESS_INTERVAL_MS = 1000; // Intervalo de procesamiento RMS
const int RMS_HISTORY_SIZE = 100;

// Configuración Modbus
#define SLAVE_ID 1
#define NUM_REGISTERS 18  // 5 por canal (3 canales x 5 = 15)
// #define RX_PIN 16
// #define TX_PIN 17
#define RX_PIN 20
#define TX_PIN 21
const int MODBUS_UPDATE_INTERVAL_MS = 300;

// Factor para convertir float RMS a uint16_t
const float CONVERSION_FACTOR = 0.618f;

const float CONVERSION_FACTORS[NUM_CHANNELS] = {
    0.653f,  // Factor para Canal 0
    0.679f,  // Factor para Canal 1 (Ejemplo: pon aquí tu valor)
    1.133f   // Factor para Canal 2 (Ejemplo: pon aquí tu valor)
};

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

// Buffers circulares para historial de RMS
float rms_history_ch0[RMS_HISTORY_SIZE];
float rms_history_ch1[RMS_HISTORY_SIZE];
float rms_history_ch2[RMS_HISTORY_SIZE];
volatile int rms_history_head = 0;
SemaphoreHandle_t rms_history_mutex;

// Variables para ADC
volatile bool adc_data_ready = false;
volatile uint8_t current_isr_channel = 0;

// Variables para Modbus
ModbusServerRTU MBserver(2000);
uint16_t holdingRegisters[NUM_REGISTERS];
SemaphoreHandle_t dataMutex;
TaskHandle_t dataUpdateTaskHandle;

// =================================================================
// --- FUNCIONES (ADC y RMS) ---
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
            
            // --- CORRECCIÓN INICIO ---
            // Seleccionar explícitamente la constante correcta para cada canal
            uint16_t mux_config;
            switch(current_isr_channel) {
                case 0: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_0; break;
                case 1: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_1; break;
                case 2: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_2; break;
                default: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_0; break;
            }
            ads.startADCReading(mux_config, false);
            // --- CORRECCIÓN FIN ---

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
// --- FUNCIONES (Modbus) ---
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
                    
                    // USAR EL FACTOR ESPECÍFICO DEL CANAL ACTUAL [ch]
                    float volts = rms_channel[ch][i] * CONVERSION_FACTORS[ch]; 
                    
                    holdingRegisters[idx] = (counts[ch] > i) ? (uint16_t)round(volts) : 0;
                }
            }

            xSemaphoreGive(dataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(MODBUS_UPDATE_INTERVAL_MS));
    }
}

struct SensorData {
    uint16_t sensorID;
    uint16_t numberOfChannels;
    uint16_t startAddress;
    uint16_t maxRegisters;
    uint16_t samplingInterval;     // ms
    uint16_t dataType;              // 1=uint8, 2=uint16, 3=compressed bytes, 4=float16
    uint16_t scale;                 // 10^scale
    uint16_t compressedBytes;       // Solo si dataType= 3
};

SensorData sensor = {1, 3, 10, NUM_REGISTERS, PROCESS_INTERVAL_MS, 1, 1, 0};

ModbusMessage readHoldingRegistersWorker(ModbusMessage request) {
    uint16_t address, words;
    ModbusMessage response;
    Serial.printf("Modbus Request Received: ServerID=%d, FunctionCode=%d\n",
                  request.getServerID(), request.getFunctionCode());

    request.get(2, address);
    request.get(4, words);

    // Si la petición es para los primeros 8 registros, enviar los campos de 'sensor'
    if (address == 0 && words == 8) {
        response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
        response.add(sensor.sensorID);
        response.add(sensor.numberOfChannels);
        response.add(sensor.startAddress);
        response.add(sensor.maxRegisters);
        response.add(sensor.samplingInterval);
        response.add(sensor.dataType);
        response.add(sensor.scale);
        response.add(sensor.compressedBytes);
        return response;
    }
    // Si la petición es para los datos RMS (dirección 10, 60 registros)
    else if (address == 10 && words == NUM_REGISTERS)
    {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
            for (uint16_t i = 0; i < words; ++i) {
                response.add((uint16_t)holdingRegisters[i]);
            }
            xSemaphoreGive(dataMutex);
        } else {
            response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_BUSY);
        }
        return response;
    }
    // Para cualquier otra petición, ahora aplicamos la validación de límites
    else if (address >= NUM_REGISTERS || (address + words) > NUM_REGISTERS) {
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
        return response;
    }
    // Si no es un caso especial y está dentro de los límites (aunque no se maneje explícitamente)
    else {
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
        return response;
    }
}

// =================================================================
// --- SETUP Y LOOP ---
// =================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n===================================");
    Serial.println("Iniciando Sistema Combinado: ADS1015 + Esclavo Modbus RMS");
    Serial.println("===================================");

    // Inicializar I2C y ADS1015
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000L);
    if (!ads.begin()) { Serial.println("ERROR: ADS1015 no encontrado."); while (1); }
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(RATE_ADS1015_3300SPS);

    // Inicializar Serial2 para Modbus
    RTUutils::prepareHardwareSerial(ModbusSerial);
    ModbusSerial.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);

    // Crear colas y mutexes
    queue_adc_samples = xQueueCreate(FIFO_SIZE, sizeof(ADC_Sample));
    rms_history_mutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    if (!rms_history_mutex || !dataMutex) {
        Serial.println("ERROR: No se pudieron crear los mutexes.");
        while (1);
    }

    // Configurar interrupción ADS
    pinMode(ADS_ALERT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ADS_ALERT_PIN), on_adc_data_ready, FALLING);
    ads.startComparator_SingleEnded(0, 1000);

    // Registrar worker Modbus
    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.setModbusTimeout(2000);

    // Crear tareas
    xTaskCreatePinnedToCore(task_adquisicion, "TaskAdquisicion", 4096, NULL, 5, NULL, 0);  // Núcleo 1
    xTaskCreatePinnedToCore(task_procesamiento, "TaskProcesamiento", 4096, NULL, 3, NULL, 0);  // Núcleo 0
    MBserver.begin(ModbusSerial, 0);  // Modbus en Núcleo 0
    xTaskCreatePinnedToCore(dataUpdateTask, "DataUpdateTask", 2048, NULL, 1, &dataUpdateTaskHandle, 0);  // Núcleo 0

    Serial.println("INFO: Setup completado.");
}

void loop() {
    // Loop libre en Núcleo 1
    vTaskDelay(pdMS_TO_TICKS(5000));
    Serial.println("Loop principal activo...");

    // Demo: Mostrar algunos valores RMS (opcional, para debug)
    const int num_datos = 6;
    float datos_ch1[num_datos];
    int obtenidos = get_rms_history(2, datos_ch1, num_datos);
    if (obtenidos > 0) {
        Serial.printf("Últimos %d RMS del Canal 1 (en unidades ADC):\n", obtenidos);
        for (int i = 0; i < obtenidos; i++) {
            Serial.printf("  %.3f\n", datos_ch1[i]);
        }
    }
}