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
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "HardwareSerial.h"
#include "ModbusServerRTU.h"

/**
 * @brief Serial port instance for Modbus RS485 communication.
 * @ingroup group_modbus
 */
HardwareSerial ModbusSerial(1);

// =================================================================
// --- COMBINED CONFIGURATION ---
// =================================================================

/**
 * @def I2C_SDA_PIN
 * @brief I2C SDA pin for ADS1015 communication.
 * @ingroup group_hardware
 */
#define I2C_SDA_PIN 9

/**
 * @def I2C_SCL_PIN
 * @brief I2C SCL pin for ADS1015 communication.
 * @ingroup group_hardware
 */
#define I2C_SCL_PIN 8

/**
 * @def ADS_ALERT_PIN
 * @brief ADS1015 interrupt pin for completed conversions.
 * @ingroup group_hardware
 */
#define ADS_ALERT_PIN 10

/**
 * @var NUM_CHANNELS
 * @brief Number of ADC channels to sample.
 * @ingroup group_hardware
 */
const int NUM_CHANNELS = 3;

/**
 * @var FIFO_SIZE
 * @brief Circular FIFO buffer size for RMS calculation.
 * @ingroup group_adc_rms
 */
const int FIFO_SIZE = 320;

/**
 * @var PROCESS_INTERVAL_MS
 * @brief RMS processing interval in milliseconds.
 * @ingroup group_adc_rms
 */
const int PROCESS_INTERVAL_MS = 1000;

/**
 * @var RMS_HISTORY_SIZE
 * @brief RMS value history size for each channel.
 * @ingroup group_adc_rms
 */
const int RMS_HISTORY_SIZE = 100;

/**
 * @def SLAVE_ID
 * @brief Modbus slave device address.
 * @ingroup group_modbus
 */
#define SLAVE_ID 1

/**
 * @def NUM_REGISTERS
 * @brief Total number of Modbus registers available for RMS data.
 * @ingroup group_modbus
 */
#define NUM_REGISTERS 18

/**
 * @def RX_PIN
 * @brief UART receive pin for RS485.
 * @ingroup group_hardware
 */
#define RX_PIN 20

/**
 * @def TX_PIN
 * @brief UART transmit pin for RS485.
 * @ingroup group_hardware
 */
#define TX_PIN 21

/**
 * @var MODBUS_UPDATE_INTERVAL_MS
 * @brief Modbus register update interval in milliseconds.
 * @ingroup group_modbus
 */
const int MODBUS_UPDATE_INTERVAL_MS = 300;

/**
 * @var CONVERSION_FACTOR
 * @brief Global conversion factor (maintained for compatibility).
 * @deprecated Use CONVERSION_FACTORS array for channel-specific factors.
 * @ingroup group_adc_rms
 */
const float CONVERSION_FACTOR = 0.618f;

/**
 * @brief Channel-specific conversion factors for each ADC channel.
 * @details Enables individual channel calibration to compensate for differences
 * in sensors, signal conditioning or specific measurement ranges.
 * @ingroup group_adc_rms
 */
const float CONVERSION_FACTORS[NUM_CHANNELS] = {
    0.653f,  ///< Factor for Channel 0
    0.679f,  ///< Factor for Channel 1
    1.133f   ///< Factor for Channel 2
};

// =================================================================
// --- DATA STRUCTURES AND GLOBAL VARIABLES ---
// =================================================================

/**
 * @brief ADS1015 converter instance.
 * @ingroup group_hardware
 */
Adafruit_ADS1015 ads;

/**
 * @struct ADC_Sample
 * @brief ADC sample with value and associated channel.
 * @ingroup group_adc_rms
 */
struct ADC_Sample {
    int16_t value;   ///< ADC conversion value.
    uint8_t channel; ///< Channel from which the sample originates.
};

/**
 * @brief Communication queue for ADC samples.
 * @ingroup group_adc_rms
 */
QueueHandle_t queue_adc_samples;

/**
 * @struct RMS_FIFO
 * @brief Circular buffer for efficient RMS calculation.
 * @details Maintains cumulative sums for incremental mean and variance calculation.
 * @ingroup group_adc_rms
 */
struct RMS_FIFO {
    int16_t buffer[FIFO_SIZE]; ///< Circular sample buffer.
    int head = 0;              ///< Write index (buffer head).
    int count = 0;             ///< Number of valid samples in buffer.
    int64_t sum_x = 0;         ///< Cumulative sum of samples.
    int64_t sum_x2 = 0;        ///< Cumulative sum of squared samples.
};

/**
 * @brief Array of FIFOs for each measurement channel.
 * @ingroup group_adc_rms
 */
RMS_FIFO fifos[NUM_CHANNELS];

/**
 * @brief RMS value history for channel 0.
 * @ingroup group_adc_rms
 */
float rms_history_ch0[RMS_HISTORY_SIZE];

/**
 * @brief RMS value history for channel 1.
 * @ingroup group_adc_rms
 */
float rms_history_ch1[RMS_HISTORY_SIZE];

/**
 * @brief RMS value history for channel 2.
 * @ingroup group_adc_rms
 */
float rms_history_ch2[RMS_HISTORY_SIZE];

/**
 * @brief Write index for RMS histories.
 * @ingroup group_adc_rms
 */
volatile int rms_history_head = 0;

/**
 * @brief Mutex for concurrent access protection to RMS histories.
 * @ingroup group_adc_rms
 */
SemaphoreHandle_t rms_history_mutex;

/**
 * @brief ADC data ready flag (ISR).
 * @ingroup group_adc_rms
 */
volatile bool adc_data_ready = false;

/**
 * @brief Current channel in sampling sequence (ISR).
 * @ingroup group_adc_rms
 */
volatile uint8_t current_isr_channel = 0;

/**
 * @brief Modbus RTU server with 2000ms timeout.
 * @ingroup group_modbus
 */
ModbusServerRTU MBserver(2000);

/**
 * @brief Modbus registers for RMS data.
 * @ingroup group_modbus
 */
uint16_t holdingRegisters[NUM_REGISTERS];

/**
 * @brief Mutex for Modbus register protection.
 * @ingroup group_modbus
 */
SemaphoreHandle_t dataMutex;

/**
 * @brief Handle for Modbus data update task.
 * @ingroup group_modbus
 */
TaskHandle_t dataUpdateTaskHandle;

// =================================================================
// --- FUNCTIONS (ADC and RMS) ---
// =================================================================

/**
 * @brief Interrupt service routine for completed ADC conversions.
 * @details Executes when ADS1015 completes a conversion and activates ALERT line.
 * @ingroup group_adc_rms
 */
void IRAM_ATTR on_adc_data_ready() { adc_data_ready = true; }

/**
 * @brief ADC data acquisition task with improved multiplexer handling.
 * @details Manages ADC conversion sequence in round-robin mode between channels.
 * Implements explicit multiplexer configuration selection for each channel.
 * Enqueues samples in `queue_adc_samples` for subsequent processing.
 * @param pvParameters Task parameters (unused).
 * @ingroup group_adc_rms
 */
void task_adquisicion(void *pvParameters) {
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
    while (true) {
        if (adc_data_ready) {
            adc_data_ready = false;
            ADC_Sample sample = {ads.getLastConversionResults(), current_isr_channel};
            xQueueSend(queue_adc_samples, &sample, 0);
            
            current_isr_channel = (current_isr_channel + 1) % NUM_CHANNELS;
            
            /**
             * @brief Explicit multiplexer configuration selection per channel.
             * @details Direct mapping from channel number to ADS1015 multiplexer
             * configuration constant to avoid addressing errors.
             */
            uint16_t mux_config;
            switch(current_isr_channel) {
                case 0: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_0; break;
                case 1: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_1; break;
                case 2: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_2; break;
                default: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_0; break;
            }
            ads.startADCReading(mux_config, false);

        } else { vTaskDelay(1); }
    }
}

/**
 * @brief Sample processing and RMS calculation task.
 * @details Consumes samples from `queue_adc_samples`, stores them in circular FIFOs
 * and periodically calculates RMS values using efficient incremental algorithms.
 * Updates RMS history in a thread-safe manner.
 * @param pvParameters Task parameters (unused).
 * @ingroup group_adc_rms
 */
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

/**
 * @brief Gets RMS value history for a specific channel.
 * @details Extracts the last N RMS values in a thread-safe manner using mutex.
 * Values are returned in chronological order (oldest first).
 * @param channel Channel number (0, 1 or 2).
 * @param output_buffer Output buffer for RMS values.
 * @param count Number of requested values.
 * @return Number of values actually copied.
 * @ingroup group_adc_rms
 */
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
// --- FUNCTIONS (Modbus) ---
// =================================================================

/**
 * @brief Modbus register update task with per-channel conversion factors.
 * @details Periodically updates Modbus registers with latest RMS values
 * from all channels. Dynamically organizes data according to configured number of channels.
 * Applies channel-specific conversion factors from CONVERSION_FACTORS array.
 * @param pvParameters Task parameters (unused).
 * @ingroup group_modbus
 */
void dataUpdateTask(void *pvParameters) {
    Serial.println("Data update task started on Core 0.");

    const int samples_per_channel = NUM_REGISTERS / NUM_CHANNELS;

    while (true) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            float rms_channel[NUM_CHANNELS][samples_per_channel];
            int counts[NUM_CHANNELS];

            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                counts[ch] = get_rms_history(ch, rms_channel[ch], samples_per_channel);
            }

            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                for (int i = 0; i < samples_per_channel; i++) {
                    int idx = ch * samples_per_channel + i;
                    
                    /**
                     * @brief Application of channel-specific conversion factor.
                     * @details Uses CONVERSION_FACTORS[ch] instead of global factor
                     * to enable individual channel calibration.
                     */
                    float volts = rms_channel[ch][i] * CONVERSION_FACTORS[ch]; 
                    
                    holdingRegisters[idx] = (counts[ch] > i) ? (uint16_t)round(volts) : 0;
                }
            }

            xSemaphoreGive(dataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(MODBUS_UPDATE_INTERVAL_MS));
    }
}

/**
 * @struct SensorData
 * @brief Sensor configuration parameters for Modbus discovery.
 * @ingroup group_modbus
 */
struct SensorData {
    uint16_t sensorID;          ///< Unique sensor identifier.
    uint16_t numberOfChannels;  ///< Number of measurement channels.
    uint16_t startAddress;      ///< Data start address in Modbus registers.
    uint16_t maxRegisters;      ///< Maximum number of available registers.
    uint16_t samplingInterval;  ///< Sampling interval in milliseconds.
    uint16_t dataType;          ///< Data type: 1=uint8, 2=uint16, 3=compressed bytes, 4=float16.
    uint16_t scale;             ///< Scale factor (10^scale).
    uint16_t compressedBytes;   ///< Bytes per compressed value (only if dataType=3).
};

/**
 * @brief Sensor configuration for discovery responses.
 * @ingroup group_modbus
 */
SensorData sensor = {1, 3, 10, NUM_REGISTERS, PROCESS_INTERVAL_MS, 1, 1, 0};

/**
 * @brief Worker to handle Modbus holding register read requests.
 * @details Responds to two types of queries:
 * - Registers 0-7: Sensor configuration parameters
 * - Registers 10+: Historical RMS data
 * Implements address boundary validation to prevent invalid access.
 * @param request Received Modbus request message.
 * @return ModbusMessage Formatted Modbus response.
 * @ingroup group_modbus
 */
ModbusMessage readHoldingRegistersWorker(ModbusMessage request) {
    uint16_t address, words;
    ModbusMessage response;
    Serial.printf("Modbus Request Received: ServerID=%d, FunctionCode=%d\n",
                  request.getServerID(), request.getFunctionCode());

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
    else if (address >= NUM_REGISTERS || (address + words) > NUM_REGISTERS) {
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
        return response;
    }
    else {
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
        return response;
    }
}

// =================================================================
// --- SETUP AND LOOP ---
// =================================================================

/**
 * @brief System initialization function.
 * @details Configures hardware, initializes peripherals, creates FreeRTOS tasks and
 * registers Modbus handlers. Establishes complete embedded system architecture.
 * @ingroup group_hardware
 */
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n===================================");
    Serial.println("Starting Combined System: ADS1015 + Modbus RMS Slave");
    Serial.println("===================================");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000L);
    if (!ads.begin()) { Serial.println("ERROR: ADS1015 not found."); while (1); }
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(RATE_ADS1015_3300SPS);

    RTUutils::prepareHardwareSerial(ModbusSerial);
    ModbusSerial.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);

    queue_adc_samples = xQueueCreate(FIFO_SIZE, sizeof(ADC_Sample));
    rms_history_mutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    if (!rms_history_mutex || !dataMutex) {
        Serial.println("ERROR: Could not create mutexes.");
        while (1);
    }

    pinMode(ADS_ALERT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ADS_ALERT_PIN), on_adc_data_ready, FALLING);
    ads.startComparator_SingleEnded(0, 1000);

    MBserver.registerWorker(SLAVE_ID, READ_HOLD_REGISTER, &readHoldingRegistersWorker);
    MBserver.setModbusTimeout(2000);

    xTaskCreatePinnedToCore(task_adquisicion, "TaskAdquisicion", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(task_procesamiento, "TaskProcesamiento", 4096, NULL, 3, NULL, 0);
    MBserver.begin(ModbusSerial, 0);
    xTaskCreatePinnedToCore(dataUpdateTask, "DataUpdateTask", 2048, NULL, 1, &dataUpdateTaskHandle, 0);

    Serial.println("INFO: Setup completed.");
}

/**
 * @brief System main loop.
 * @details Executes demonstration and debugging tasks. Periodically displays
 * sample RMS values from the system-maintained history.
 * @ingroup group_hardware
 */
void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
    Serial.println("Main loop active...");

    const int num_datos = 6;
    float datos_ch1[num_datos];
    int obtenidos = get_rms_history(2, datos_ch1, num_datos);
    if (obtenidos > 0) {
        Serial.printf("Last %d RMS from Channel 1 (in ADC units):\n", obtenidos);
        for (int i = 0; i < obtenidos; i++) {
            Serial.printf("  %.3f\n", datos_ch1[i]);
        }
    }
}