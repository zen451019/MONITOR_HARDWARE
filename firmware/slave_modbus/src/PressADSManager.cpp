#include "PressADSManager.h"
#include <math.h>

// 1. CONSTRUCTOR
PressADSManager::PressADSManager(const ADSconfig& cfg)
    : ADSBase(cfg), config(cfg) {
    
    data_mutex = xSemaphoreCreateMutex();

    // Inicializar buffers solo para canales activos (según máscara de bits)
    for (int i = 0; i < 4; i++) {
        // Verificar si el bit 'i' está en 1
        if ((config.active_channels >> i) & 0x01) {
            if (config.history_size > 0) {
                pressure_histories[i] = new float[config.history_size](); // Init a 0
            }
        } else {
            pressure_histories[i] = nullptr;
        }
        history_heads[i] = 0;
    }
}

// 2. DESTRUCTOR
PressADSManager::~PressADSManager() {
    if (press_task_handle != nullptr) {
        vTaskDelete(press_task_handle);
    }

    // Liberar memoria de historiales
    for (int i = 0; i < 4; i++) {
        if (pressure_histories[i] != nullptr) {
            delete[] pressure_histories[i];
            pressure_histories[i] = nullptr;
        }
    }

    if (data_mutex != nullptr) {
        vSemaphoreDelete(data_mutex);
    }
}

// 3. BEGIN
bool PressADSManager::begin() {
    // Inicializar I2C y Ganancia (método padre)
    if (!initADS()) {
        return false;
    }

    // Configurar Data Rate (similar a TempADSManager, lectura DC)
    if (config.type == ADSType::ADS1115) {
        switch(config.sampling_rate) {
            case 8:    ads->setDataRate(RATE_ADS1115_8SPS); break;
            case 16:   ads->setDataRate(RATE_ADS1115_16SPS); break;
            case 32:   ads->setDataRate(RATE_ADS1115_32SPS); break;
            case 64:   ads->setDataRate(RATE_ADS1115_64SPS); break;
            case 128:  ads->setDataRate(RATE_ADS1115_128SPS); break;
            case 250:  ads->setDataRate(RATE_ADS1115_250SPS); break;
            case 475:  ads->setDataRate(RATE_ADS1115_475SPS); break;
            case 860:  ads->setDataRate(RATE_ADS1115_860SPS); break;
            default:   ads->setDataRate(RATE_ADS1115_128SPS); break;
        }
    } else {
        // ADS1015
        switch(config.sampling_rate) {
            case 128:  ads->setDataRate(RATE_ADS1015_128SPS); break;
            case 250:  ads->setDataRate(RATE_ADS1015_250SPS); break;
            case 1600: ads->setDataRate(RATE_ADS1015_1600SPS); break;
            default:   ads->setDataRate(RATE_ADS1015_1600SPS); break;
        }
    }

    return true;
}

// 4. START SAMPLING
void PressADSManager::startSampling() {
    // Creamos la tarea. Usamos un stack size moderado.
    xTaskCreatePinnedToCore(
        press_task_trampoline, 
        "PressTask", 
        3072,       // Stack size (un poco más que temp por si acaso hay 4 canales)
        this,       
        2,          // Prioridad media (entre Temp=1 y ADS_Acq=5)
        &press_task_handle, 
        1           // Core 1 (APP Core)
    );
}

// 5. TRAMPOLÍN
void PressADSManager::press_task_trampoline(void* arg) {
    PressADSManager* instance = static_cast<PressADSManager*>(arg);
    if (instance) {
        instance->press_task_body();
    }
}

// 6. BODY (Bucle Principal)
void PressADSManager::press_task_body() {
    while (true) {
        // Recorrer los 4 canales posibles
        for (int ch = 0; ch < 4; ch++) {
            // Chequear si el canal está activo en la config
            if ((config.active_channels >> ch) & 0x01) {
                
                float voltage = getRaw(ch, config.NUM_SAMPLES); // Promediar varias muestras para reducir ruido
                
                // Conversión a presión
                float pressure = convertVoltageToPressure(voltage);

                // Guardar en historial protegido por Mutex
                if (pressure_histories[ch] != nullptr) {
                    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        int head = history_heads[ch];
                        pressure_histories[ch][head] = pressure;
                        history_heads[ch] = (head + 1) % config.history_size;
                        xSemaphoreGive(data_mutex);
                    }
                }
                
                // Pequeña pausa entre canales para estabilidad del I2C
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }

        // Esperar el intervalo configurado antes del siguiente barrido
        vTaskDelay(pdMS_TO_TICKS(config.process_interval_ms));
    }
}

// 7. HELPER: Conversión Voltaje -> Presión
float PressADSManager::convertVoltageToPressure(float voltage) {
    // Ecuación de la recta (y = mx + b) interpolando entre min y max
    // P = Pmin + (V - Vmin) * (Pmax - Pmin) / (Vmax - Vmin)
    
    // Clamping: Asegurar que el voltaje esté dentro del rango esperado para evitar
    // valores de presión negativos absurdos o fuera de rango físico.
    float safe_volts = voltage;
    if (safe_volts < config.min_voltage) safe_volts = config.min_voltage;
    if (safe_volts > config.max_voltage) safe_volts = config.max_voltage;

    float slope = (config.max_pressure - config.min_pressure) / (config.max_voltage - config.min_voltage);
    float pressure = config.min_pressure + (safe_volts - config.min_voltage) * slope;

    return pressure;
}

// 8. API PÚBLICA: getLatest
float PressADSManager::getLatest(int channel) {
    float val = 0.0f;
    getHistory(channel, &val, 1);
    return val;
}

// 9. API PÚBLICA: getHistory
int PressADSManager::getHistory(int channel, float* output_buffer, int count) {
    if (channel < 0 || channel >= 4 || pressure_histories[channel] == nullptr) {
        return 0; // Canal inválido o no activo
    }
    if (count > config.history_size) {
        return 0;
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Cálculo del índice más reciente
        int head = history_heads[channel];
        int most_recent = (head == 0) ? (config.history_size - 1) : (head - 1);

        for (int i = 0; i < count; i++) {
            // Recorrer hacia atrás circularmente
            int idx = (most_recent - (count - 1 - i) + config.history_size) % config.history_size;
            output_buffer[i] = pressure_histories[channel][idx];
        }

        xSemaphoreGive(data_mutex);
        return count;
    }
    return 0;
}

float PressADSManager::getRaw(uint8_t channel, uint8_t NUM_SAMPLES) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        sum += readChannel(channel);
        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeña pausa entre muestras
    }
    
    float average_adc = (float)sum / NUM_SAMPLES;
    float voltage = ads->computeVolts(average_adc); // Usamos el método del ADS para convertir a voltios

    return voltage;
}