#include <Arduino.h>
#include <cmath>
#include "ModbusServerRTU.h"

// =================================================================================================
// Configuración del bus RS485
// =================================================================================================
struct BusConfig {
    unsigned long baudRate;
    uint32_t      uartConfig;
    int           rxPin;
    int           txPin;
};

const BusConfig kBusCfg = {
    9600,
    SERIAL_8N1,
    3,   // RX = GPIO3
    1    // TX = GPIO1
};

// R/D del transceptor RS485 se pasa en el constructor de ModbusServerRTU (ver abajo)

// =================================================================================================
// Identidad del dispositivo
// =================================================================================================
#define SLAVE_ID 1

// =================================================================================================
// Tabla de Registros Modbus
// =================================================================================================
// DIR  | LONG  | TIPO  | CANAL | DESCRIPCIÓN
// -----+-------+-------+-------+----------------------------------------
// 0x00 |   1   | uint16| V L1  | Voltaje fase 1 (escala: 12000 = 120.00V)
// 0x01 |   1   | uint16| V L1  | Reservado / histórico
// 0x02 |   1   | uint16| V L2  | Voltaje fase 2
// 0x03 |   1   | uint16| V L2  | Reservado / histórico
// 0x04 |   1   | uint16| V L3  | Voltaje fase 3
// 0x05 |   1   | uint16| V L3  | Reservado / histórico
// 0x06 |   1   | uint16| I L1  | Corriente fase 1 (escala: 500 = 5.00A)
// 0x07 |   1   | uint16| I L1  | Reservado / histórico
// 0x08 |   1   | uint16| I L2  | Corriente fase 2
// 0x09 |   1   | uint16| I L2  | Reservado / histórico
// 0x0A |   1   | uint16| I L3  | Corriente fase 3
// 0x0B |   1   | uint16| I L3  | Reservado / histórico
// =================================================================================================

#define NUM_REGISTERS 12

// =================================================================================================
// Sensores simulados
// =================================================================================================
uint16_t holdingRegisters[NUM_REGISTERS];
SemaphoreHandle_t dataMutex;

void simulatedSensorTask(void* pvParameters) {
    while (true) {
        uint32_t t_ms = millis();
        float t = t_ms * 0.0001f;
        const float deg120 = 2.094395f;

        float v1 = 12000.0f + 500.0f * sinf(t);
        float v2 = 11950.0f + 500.0f * sinf(t + deg120);
        float v3 = 12050.0f + 500.0f * sinf(t + 2.0f * deg120);

        float i1 = 500.0f + 50.0f * sinf(t);
        float i2 = 480.0f + 50.0f * sinf(t + deg120);
        float i3 = 520.0f + 50.0f * sinf(t + 2.0f * deg120);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            holdingRegisters[0]  = (uint16_t)v1;
            holdingRegisters[1]  = 0;
            holdingRegisters[2]  = (uint16_t)v2;
            holdingRegisters[3]  = 0;
            holdingRegisters[4]  = (uint16_t)v3;
            holdingRegisters[5]  = 0;
            holdingRegisters[6]  = (uint16_t)i1;
            holdingRegisters[7]  = 0;
            holdingRegisters[8]  = (uint16_t)i2;
            holdingRegisters[9]  = 0;
            holdingRegisters[10] = (uint16_t)i3;
            holdingRegisters[11] = 0;
            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// =================================================================================================
// Worker Modbus
// =================================================================================================
ModbusMessage readHoldingRegistersWorker(ModbusMessage request) {
    uint16_t address, words;
    ModbusMessage response;

    request.get(2, address);
    request.get(4, words);

    if (address + words <= NUM_REGISTERS) {
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

    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
}

// =================================================================================================
// Setup
// =================================================================================================
ModbusServerRTU MBserver(2000, 22);  // timeout=2000ms, rtsPin=22 (R/D)

void setup() {
    dataMutex = xSemaphoreCreateMutex();

    RTUutils::prepareHardwareSerial(Serial);
    Serial.begin(kBusCfg.baudRate, kBusCfg.uartConfig, kBusCfg.rxPin, kBusCfg.txPin);
    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.begin(Serial, 0);

    xTaskCreatePinnedToCore(simulatedSensorTask, "SimSensor", 2048, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
}
