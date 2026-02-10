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

// ===== SELECCIÓN DE MODO =====
// Descomenta UNO de los dos para elegir el modo (o úsalo desde platformio.ini)
// #define MODE_RMS
#define MODE_TEMP  // <--- En este ejemplo activamos Temperatura

#if defined(MODE_RMS)
    #include "ADSManager.h"
#elif defined(MODE_TEMP)
    #include "TempADSManager.h"
#elif defined(MODE_PRESS)
    #include "PressADSManager.h"
#else
    #error "Debes definir MODE_RMS o MODE_TEMP"
#endif

// ===== CONFIGURACIÓN =====
#define SLAVE_ID 1
#define NUM_CHANNELS 3
#define NUM_REGISTERS 18
#define RX_PIN 16
#define TX_PIN 17

HardwareSerial ModbusSerial(1);
ModbusServerRTU MBserver(2000);

// ===== CONFIGURACIÓN ADS =====
#if defined(MODE_RMS)
    const float CONVERSION_FACTORS[] = {0.653f, 0.679f, 1.133f};
    ADSConfig config(
        ADSType::ADS1015,   // RMS puede usar el modelo más rápido (1015)
        0x48,
        GAIN_TWOTHIRDS,     // Ganancia común para señales de hasta ±6.144V 
        1000,               // Intervalo de procesamiento (1 segundo)
        NUM_CHANNELS,       // Número de canales a muestrear
        CONVERSION_FACTORS, // Puntero a los factores de conversión específicos por canal
        19,                 // alert_pin
        3300,               // sampling_rate: 3300 SPS (máximo para ADS1015, para asegurar mediciones rápidas y precisas)
        320,                // Fifo size: 320 muestras por canal (10 segundos de historial a 330 SPS)
        100                 // History size: 100 muestras por canal (para mantener un historial de ~3 segundos a 330 SPS)
    );
#elif defined(MODE_TEMP)
    // Configuración específica de Temp (R0, R_Serie, etc)
    ADSconfig config(
        ADSType::ADS1115,    // Temp necesita más resolución (16-bit)
        0x48,                
        GAIN_TWO,            // Ganancia más alta para medir mV pequeños
        1000,                // Intervalo
        4700,                // R serie
        100,                 // R0 (PT100)
        128,                 // Sample rate lento (más preciso)
        10,
        50                   // Historial
    );
#elif defined(MODE_PRESS)
    ADSconfig config(
        ADSType::ADS1115,    // Presión también se beneficia de 16-bit
        0x48,
        GAIN_TWOTHIRDS,     // Ganancia media para rango típico de sensores de presión
        1000,               // Intervalo
        0.5f,               // Voltaje mínimo del sensor (0.5V)
        4.5f,               // Voltaje máximo del sensor (4.5V)
        0.0f,               // Presión mínima (ajustar según sensor)
        100.0f,             // Presión máxima (ajustar según sensor)
        0b0001,            // Solo canal 0 activo
        128,                // Sample rate
        10,                 // Número de muestras para promediar
        50                  // Historial
    );
#endif

ADSBase* sensorDriver = nullptr; // Puntero Polimórfico

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
                
                // --- CAMBIO AQUÍ: getRMSHistory -> getHistory ---
                // Ahora usamos el método estandarizado que viene de ADSBase
                int count = sensorDriver->getHistory(ch, rms_values, samples_per_channel);
                
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
    
    // ===== INSTANCIACIÓN CONDICIONAL =====
    #if defined(MODE_RMS)
        Serial.println(">>> MODO: RMS MONITOR <<<");
        sensorDriver = new ADSManager(config);
    #elif defined(MODE_TEMP)
        Serial.println(">>> MODO: TEMPERATURA PT100 <<<");
        sensorDriver = new TempADSManager(config);
    #elif defined(MODE_PRESS)
        Serial.println(">>> MODO: PRESIÓN <<<");
        sensorDriver = new PressADSManager(config); 
    #endif

    if (!sensorDriver->begin()) {
        Serial.println("Error iniciando sensor");
        while(1);
    }
    
    Serial.println("ADS1015 inicializado correctamente");
    
    RTUutils::prepareHardwareSerial(ModbusSerial);
    ModbusSerial.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);
    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.begin(ModbusSerial, 0);
    
    dataMutex = xSemaphoreCreateMutex();
    
    sensorDriver->startSampling();
    xTaskCreatePinnedToCore(dataUpdateTask, "ModbusUpdate", 2048, NULL, 1, NULL, 0);
    
    Serial.println("Sistema listo - Iniciando muestreo...");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // --- CAMBIO AQUÍ: getLatestRMS -> getLatest ---
    // El método es agnóstico, no importa si devuelve RMS o Temp
    Serial.printf("CH0: %.1f | CH1: %.1f | CH2: %.1f V\n",
                  sensorDriver->getLatest(0),
                  sensorDriver->getLatest(1),
                  sensorDriver->getLatest(2));
}