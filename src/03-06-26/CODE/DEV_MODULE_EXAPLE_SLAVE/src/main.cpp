#include <Arduino.h>
#include <HardwareSerial.h>
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
#define DIAG_LED 2  // GPIO2 = LED onboard (parpadea al recibir solicitud Modbus)

// =================================================================================================
// Identidad del dispositivo
// =================================================================================================
#define SLAVE_ID 5

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
// 0x0C |   1   | uint16| Ana1  | Sensor analógico 1 (suavizado + tratamiento opcional)
// 0x0D |   1   | uint16| Ana2  | Sensor analógico 2 (suavizado + tratamiento opcional)
// =================================================================================================

#define NUM_REGISTERS 14

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
// Sensores analógicos (ADC)
// =================================================================================================

// Cada canal tiene su propio buffer de suavizado (promedio móvil de 4 muestras).
// El suavizado se aplica siempre. La función de tratamiento es opcional
// (NULL = se guarda el ADC suavizado tal cual).
typedef uint16_t (*AnalogTreatmentFn)(uint16_t raw_adc);

struct AnalogChannel {
    uint8_t             adcPin;
    uint16_t            regIndex;
    AnalogTreatmentFn   treatment;
    uint16_t            history[4];
    uint8_t             histIdx;
};

// --- Funciones de tratamiento de ejemplo (copia y modifica según necesites) ---

// Raw: el ADC suavizado sin transformación (0-4095)
static uint16_t treatment_raw(uint16_t adc) { return adc; }

// mV en bornes de entrada: divisor resistivo 43k/10k, ESP32 ADC 0-3.3V = 0-4095
// Vout = Vin * 10/(43+10) = Vin * 0.18868
// ADC = Vout * 4095 / 3300
// Vin_mV = ADC * 3300 * (43+10) / (4095 * 10) = ADC * 17490 / 4095
static uint16_t treatment_mv(uint16_t adc) {
    return (uint16_t)((uint32_t)adc * 17490 / 4095);
}

// Presión en PSI × 100 (centi-PSI): sensor 0.5-4.5V → 0-1 MPa a través del divisor 43k/10k
// Fórmula del sensor: Vout = 5(0.8P + 0.1), P en MPa
// Despejando: P_MPa = (Vout - 0.5) / 4
// Rango ADC efectivo tras divisor: 117 (0 MPa) → 1054 (1 MPa)
// 1 MPa = 145.038 PSI → 14504 centi-PSI a fondo de escala
static uint16_t treatment_pressure_psi(uint16_t adc) {
    if (adc <= 117) return 0;
    uint32_t centiPsi = (uint32_t)(adc - 117) * 14504 / 937;
    return (uint16_t)(centiPsi > 65535 ? 65535 : centiPsi);
}

// --- Tabla de canales analógicos ---
// Define aquí: pin ADC, índice del registro Modbus y función de tratamiento.
// treatment = NULL equivale a treatment_raw (ADC suavizado tal cual).
AnalogChannel kAnalogChannels[] = {
    {32, 12, treatment_pressure_psi, {0}, 0},  // GPIO32 → reg 0x0C, presión PSI×100
    {33, 13, treatment_mv,           {0}, 0},  // GPIO33 → reg 0x0D, voltaje mV
};
constexpr size_t kAnalogChannelCount = sizeof(kAnalogChannels) / sizeof(kAnalogChannels[0]);

// Tarea que lee los ADC, aplica suavizado y escribe en holdingRegisters bajo mutex
void analogSensorTask(void* pvParameters) {
    while (true) {
        for (size_t i = 0; i < kAnalogChannelCount; ++i) {
            AnalogChannel& ch = kAnalogChannels[i];

            // Leer ADC y meter en el buffer circular de 4 muestras
            uint16_t sample = (uint16_t)analogRead(ch.adcPin);
            ch.history[ch.histIdx] = sample;
            ch.histIdx = (ch.histIdx + 1) & 3;

            // Promedio móvil (suavizado)
            uint16_t smoothed = (uint16_t)((ch.history[0] + ch.history[1] +
                                            ch.history[2] + ch.history[3]) / 4);

            // Aplicar tratamiento si existe, si no va el suavizado tal cual
            uint16_t value = ch.treatment ? ch.treatment(smoothed) : smoothed;

            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                holdingRegisters[ch.regIndex] = value;
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// =================================================================================================
// Worker Modbus
// =================================================================================================
ModbusMessage readHoldingRegistersWorker(ModbusMessage request) {
    digitalWrite(DIAG_LED, HIGH);

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
        digitalWrite(DIAG_LED, LOW);
        return response;
    }

    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    digitalWrite(DIAG_LED, LOW);
    return response;
}

// =================================================================================================
// Setup
// =================================================================================================
HardwareSerial ModbusSerial(1);       // UART1 para Modbus (evita UART0/USB)
ModbusServerRTU MBserver(2000, 22);  // timeout=2000ms, rtsPin=22 (R/D)

void setup() {
    // Inicializar R/D antes que nada (el constructor global podría no tener efecto
    // por ejecutarse antes de que el HAL GPIO del ESP32 esté listo)
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);

    pinMode(DIAG_LED, OUTPUT);
    digitalWrite(DIAG_LED, LOW);

    // Parpadeo rápido de confirmación de boot
    digitalWrite(DIAG_LED, HIGH);
    delay(100);
    digitalWrite(DIAG_LED, LOW);

    dataMutex = xSemaphoreCreateMutex();

    RTUutils::prepareHardwareSerial(ModbusSerial);
    ModbusSerial.begin(kBusCfg.baudRate, kBusCfg.uartConfig, kBusCfg.rxPin, kBusCfg.txPin);
    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.begin(ModbusSerial, 0);

    xTaskCreatePinnedToCore(simulatedSensorTask, "SimSensor",  2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(analogSensorTask,    "AnaSensor", 2048, NULL, 1, NULL, 0);

    // 3 parpadeos rápidos = sistema listo
    for (int i = 0; i < 3; i++) {
        digitalWrite(DIAG_LED, HIGH);
        delay(80);
        digitalWrite(DIAG_LED, LOW);
        delay(80);
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
}
