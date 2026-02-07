#include "TempADSManager.h"
#include <math.h>

// Variable privada para guardar la temperatura (debería estar en el .h, pero la usaremos aquí)
// Nota: Si quieres usar Mutex, recuerda declararlo en el .h primero.
static volatile float _current_temperature = 0.0f;

// 1. CONSTRUCTOR
TempADSManager::TempADSManager(const ADSconfig& cfg)
    : ADSBase(cfg), 
      config(cfg)
{
    // --- NUEVO: Inicializar Buffer ---
    if (config.history_size > 0) {
        temp_history = new float[config.history_size](); // Init a 0
    }
    history_head = 0;
    data_mutex = xSemaphoreCreateMutex(); // Crear el semáforo
}

// 2. DESTRUCTOR
TempADSManager::~TempADSManager() {
    if (temp_task_handle != nullptr) {
        vTaskDelete(temp_task_handle);
    }
    // --- NUEVO: Limpieza ---
    if (temp_history != nullptr) {
        delete[] temp_history;
    }
    if (data_mutex != nullptr) {
        vSemaphoreDelete(data_mutex);
    }
}

// 3. BEGIN
bool TempADSManager::begin() {
    // 1. Inicializa el I2C y la Ganancia (desde el padre)
    if (!initADS()) {
        return false;
    }
    
    // 2. CONFIGURAR DATA RATE (SAMPLING RATE)
    // El chip necesita traducir tu número entero a la constante correcta
    
    if (config.type == ADSType::ADS1115) {
        switch(config.sampling_rate) {
            case 8:    ads->setDataRate(RATE_ADS1115_8SPS); break;
            case 16:   ads->setDataRate(RATE_ADS1115_16SPS); break;
            case 32:   ads->setDataRate(RATE_ADS1115_32SPS); break;
            case 64:   ads->setDataRate(RATE_ADS1115_64SPS); break;
            case 128:  ads->setDataRate(RATE_ADS1115_128SPS); break; // Default común
            case 250:  ads->setDataRate(RATE_ADS1115_250SPS); break;
            case 475:  ads->setDataRate(RATE_ADS1115_475SPS); break;
            case 860:  ads->setDataRate(RATE_ADS1115_860SPS); break;
            default:   ads->setDataRate(RATE_ADS1115_128SPS); break; // Fallback seguro
        }
    } else {
        // Para ADS1015 (es más rápido)
        switch(config.sampling_rate) {
            case 128:  ads->setDataRate(RATE_ADS1015_128SPS); break;
            case 250:  ads->setDataRate(RATE_ADS1015_250SPS); break;
            case 490:  ads->setDataRate(RATE_ADS1015_490SPS); break;
            case 920:  ads->setDataRate(RATE_ADS1015_920SPS); break;
            case 1600: ads->setDataRate(RATE_ADS1015_1600SPS); break;
            case 2400: ads->setDataRate(RATE_ADS1015_2400SPS); break;
            case 3300: ads->setDataRate(RATE_ADS1015_3300SPS); break;
            default:   ads->setDataRate(RATE_ADS1015_1600SPS); break;
        }
    }

    return true;
}

// 4. START SAMPLING
void TempADSManager::startSampling() {
    // Creamos la tarea pinned to core 1 (app core)
    xTaskCreatePinnedToCore(
        temp_task_trampoline, 
        "TempTask", 
        2048,       // Stack size
        this,       // Pasamos "este" objeto como argumento
        1,          // Prioridad
        &temp_task_handle, 
        1           // Core
    );
}

// 5. TRAMPOLÍN (Puente C++ a C)
void TempADSManager::temp_task_trampoline(void* arg) {
    TempADSManager* sensor = static_cast<TempADSManager*>(arg);
    if (sensor != nullptr) {
        sensor->temp_task_body();
    }
}

// 6. BODY (La lógica del bucle)
void TempADSManager::temp_task_body() {
    while (true) {

        float Vref = fabsf(getRaw(32));   
        float Vcable = getRaw(31); 
        float Vpt100 = getRaw(30); 

        float temperature = -999.0f; // VALOR DE ERROR POR DEFECTO

        // VALIDACIÓN 1: Resistor de serie válido
        if (config.serie_resistor_ohms > 0.0f) {
            
            float I = Vref / config.serie_resistor_ohms;

            // VALIDACIÓN 2: Corriente válida (>1mA)
            if (I >= 0.0001f) {
                
                float R_cable = fabsf(Vcable / I);
                float Rpt100 = (Vpt100 / I) - (2 * R_cable);
                
                // VALIDACIÓN 3: Resultado físicamente válido
                if (Rpt100 >= 0.0f && !isnan(Rpt100) && !isinf(Rpt100)) {
                    temperature = Rpt100;
                }
            }
        }

        // GUARDAR (siempre guarda algo, incluso -999.0f si hay error)
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            temp_history[history_head] = temperature;
            history_head = (history_head + 1) % config.history_size;
            xSemaphoreGive(data_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(config.process_interval_ms));
    }
}

// --- NUEVOS GETTERS (Copiados la lógica de ADSManager) ---

float TempADSManager::getLatest(int channel) {
    // TempADSManager solo tiene 1 "canal" de temperatura, ignoramos el int channel
    float value = 0.0f;
    getHistory(0, &value, 1);
    return value;
}

int TempADSManager::getHistory(int channel, float* output_buffer, int count) {
    if (count > config.history_size || temp_history == nullptr) {
        return 0;
    }
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int most_recent = (history_head == 0) ? (config.history_size - 1) : (history_head - 1);
        
        for (int i = 0; i < count; i++) {
            // Lógica circular inversa (del más reciente hacia atras)
            int idx = (most_recent - (count - 1 - i) + config.history_size) % config.history_size;
            output_buffer[i] = temp_history[idx];
        }
        
        xSemaphoreGive(data_mutex);
        return count;
    }
    return 0;
}

float TempADSManager::getRaw(uint8_t channel, uint8_t NUM_SAMPLES) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        sum += readChannel(channel);
        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeña pausa entre muestras
    }
    
    float average_adc = (float)sum / NUM_SAMPLES;
    float voltage = ads->computeVolts(average_adc); // Usamos el método del ADS para convertir a voltios

    return voltage;
}