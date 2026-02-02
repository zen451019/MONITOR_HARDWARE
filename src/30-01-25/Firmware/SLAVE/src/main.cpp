/**
 * @file main.cpp
 * @brief Modbus RTU Slave Firmware with ADS1015 and real-time RMS calculation (ESP32).
 * @details This system acquires analog signals from 3 channels using ADS1015 ADC,
 * calculates real-time RMS values using circular FIFOs, maintains a measurement
 * history and responds to Modbus RTU queries over RS485. Includes channel-specific
 * conversion factors and improved ADC multiplexer handling.
 * @date 2025-12-04
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "ModbusServerRTU.h"
#include "ADSManager.h"

// ===== CONFIGURACIÓN =====
#define SLAVE_ID 1
#define NUM_CHANNELS 3
#define NUM_REGISTERS 18
#define RX_PIN 16
#define TX_PIN 17

HardwareSerial ModbusSerial(1);
ModbusServerRTU MBserver(2000);

// ===== CONFIGURACIÓN ADS =====
const float CONVERSION_FACTORS[] = {0.653f, 0.679f, 1.133f};

ADSConfig adsConfig(
    ADSType::ADS1015,      // type
    0x48,                  // i2c_addr
    GAIN_TWOTHIRDS,        // gain
    NUM_CHANNELS,          // num_channels
    CONVERSION_FACTORS,    // conversion_factors
    19,                    // alert_pin
    3300,                  // samples_per_second
    320,                   // fifo_size
    100,                   // history_size
    1000                   // process_interval_ms
);

ADSManager* adsManager = nullptr;

// ===== MODBUS =====
uint16_t holdingRegisters[NUM_REGISTERS];
SemaphoreHandle_t dataMutex;

struct SensorData {
    uint16_t sensorID = 1;
    uint16_t numberOfChannels = NUM_CHANNELS;
    uint16_t startAddress = 10;
    uint16_t maxRegisters = NUM_REGISTERS;
    uint16_t samplingInterval = 1000;
    uint16_t dataType = 1;
    uint16_t scale = 1;
    uint16_t compressedBytes = 0;
} sensor;

// ===== TAREA ACTUALIZACIÓN MODBUS =====
void dataUpdateTask(void* pvParameters) {
    const int samples_per_channel = NUM_REGISTERS / NUM_CHANNELS;
    
    while (true) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                float rms_values[samples_per_channel];
                int count = adsManager->getRMSHistory(ch, rms_values, samples_per_channel);
                
                for (int i = 0; i < samples_per_channel; i++) {
                    int idx = ch * samples_per_channel + i;
                    holdingRegisters[idx] = (count > i) ? (uint16_t)round(rms_values[i]) : 0;
                }
            }
            xSemaphoreGive(dataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ===== WORKER MODBUS =====
ModbusMessage readHoldingRegistersWorker(ModbusMessage request) {
    uint16_t address, words;
    ModbusMessage response;
    
    request.get(2, address);
    request.get(4, words);
    
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
    else if (address == 10 && words == NUM_REGISTERS) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
            for (uint16_t i = 0; i < words; ++i) {
                response.add(holdingRegisters[i]);
            }
            xSemaphoreGive(dataMutex);
        } else {
            response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_BUSY);
        }
        return response;
    }
    
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    delay(1000); 
    Serial.println("Sistema Modbus + ADSManager (Refactorizado)");
    
    Wire.begin(); 
    Wire.setClock(400000L);
    
    adsManager = new ADSManager(adsConfig);
    if (!adsManager->begin()) {
        Serial.println("ERROR: ADS1015 no inicializado");
        while(1) { vTaskDelay(1000); }
    }
    
    Serial.println("ADS1015 inicializado correctamente");
    
    RTUutils::prepareHardwareSerial(ModbusSerial);
    ModbusSerial.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);
    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.begin(ModbusSerial, 0);
    
    dataMutex = xSemaphoreCreateMutex();
    
    adsManager->startSampling();
    xTaskCreatePinnedToCore(dataUpdateTask, "ModbusUpdate", 2048, NULL, 1, NULL, 0);
    
    Serial.println("Sistema listo - Iniciando muestreo...");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
    Serial.printf("CH0: %.1f | CH1: %.1f | CH2: %.1f V\n",
                  adsManager->getLatestRMS(0),
                  adsManager->getLatestRMS(1),
                  adsManager->getLatestRMS(2));
}