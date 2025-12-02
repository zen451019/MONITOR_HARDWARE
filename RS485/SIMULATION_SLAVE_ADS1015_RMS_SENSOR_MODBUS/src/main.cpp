#include <Arduino.h>
#include "HardwareSerial.h"
#include "ModbusServerRTU.h"

// =================================================================
// --- CONFIGURACIÓN ---
// =================================================================
#define SLAVE_ID 2             // ID del esclavo Modbus
#define NUM_CHANNELS 3
#define NUM_REGISTERS 18       // 6 valores por canal
#define MODBUS_UPDATE_INTERVAL_MS 300
#define PROCESS_INTERVAL_MS 1000
#define RX_PIN 16
#define TX_PIN 17
const float CONVERSION_FACTOR = 100.0f;

// =================================================================
// --- ESTRUCTURAS DE DATOS ---
// =================================================================
struct RMS_SimData {
    float rms[NUM_CHANNELS];
};
RMS_SimData simData;

ModbusServerRTU MBserver(2000);
uint16_t holdingRegisters[NUM_REGISTERS];
SemaphoreHandle_t dataMutex;

// =================================================================
// --- DATOS DEL SENSOR ---
// =================================================================
struct SensorData {
    uint16_t sensorID;
    uint16_t numberOfChannels;
    uint16_t startAddress;
    uint16_t maxRegisters;
    uint16_t samplingInterval; // ms
    uint16_t dataType;         // 1=uint8, 2=uint16, 3=compressed bytes, 4=float16
    uint16_t scale;            // 10^scale
    uint16_t compressedBytes;  // Solo si dataType=3
};

SensorData sensor = {2, 3, 10, NUM_REGISTERS, PROCESS_INTERVAL_MS, 1, 1, 0};

// =================================================================
// --- GENERADOR DE DATOS SIMULADOS ---
// =================================================================
void generarDatosSimulados() {
    // Genera valores RMS falsos entre 110V y 130V simulando 3 canales
    for (int i = 0; i < NUM_CHANNELS; i++) {
        float base = 120.0 + (float)(rand() % 2000 - 1000) / 100.0;  // 110–130 V
        simData.rms[i] = base;
    }
}

// =================================================================
// --- ACTUALIZACIÓN DE HOLDING REGISTERS ---
// =================================================================
void dataUpdateTask(void *pvParameters) {
    Serial.println("Tarea de actualización de datos simulados iniciada...");

    const int samples_per_channel = NUM_REGISTERS / NUM_CHANNELS;

    while (true) {
        generarDatosSimulados();

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            // Llenar registros con valores RMS simulados
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                for (int i = 0; i < samples_per_channel; i++) {
                    int idx = ch * samples_per_channel + i;
                    float volts = simData.rms[ch] * CONVERSION_FACTOR;  // escala
                    holdingRegisters[idx] = (uint16_t)round(volts);
                }
            }
            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(MODBUS_UPDATE_INTERVAL_MS));
    }
}

// =================================================================
// --- WORKER MODBUS ---
// =================================================================
ModbusMessage readHoldingRegistersWorker(ModbusMessage request) {
    uint16_t address, words;
    ModbusMessage response;

    request.get(2, address);
    request.get(4, words);

    // Estructura SensorData (dirección 0, 8 registros)
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
    // Datos RMS (dirección 10, NUM_REGISTERS registros)
    else if (address == 10 && words == NUM_REGISTERS) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
            for (uint16_t i = 0; i < words; ++i)
                response.add(holdingRegisters[i]);
            xSemaphoreGive(dataMutex);
        } else {
            response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_BUSY);
        }
        return response;
    }
    // Error si se sale de rango
    else {
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
        return response;
    }
}

// =================================================================
// --- SETUP ---
// =================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n===================================");
    Serial.println("Iniciando Simulador RMS Modbus (ID=2)");
    Serial.println("===================================");

    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);

    dataMutex = xSemaphoreCreateMutex();
    if (!dataMutex) {
        Serial.println("ERROR: No se pudo crear el mutex.");
        while (1);
    }

    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.setModbusTimeout(2000);
    MBserver.begin(Serial2, 0);

    xTaskCreatePinnedToCore(dataUpdateTask, "DataUpdateTask", 2048, NULL, 1, NULL, 0);

    Serial.println("INFO: Sistema listo para atender peticiones Modbus.");
}

// =================================================================
// --- LOOP ---
// =================================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(2000));
    Serial.println("Valores RMS simulados:");
    for (int i = 0; i < NUM_CHANNELS; i++) {
        Serial.printf(" Canal %d: %.2f V\n", i, simData.rms[i]);
    }
}
