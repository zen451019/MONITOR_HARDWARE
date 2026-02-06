#include "TempADSManager.h"
#include <math.h>

// Variable privada para guardar la temperatura (debería estar en el .h, pero la usaremos aquí)
// Nota: Si quieres usar Mutex, recuerda declararlo en el .h primero.
static volatile float _current_temperature = 0.0f;

// 1. CONSTRUCTOR
TempADSManager::TempADSManager(const ADSconfig& cfg)
    // 1. Inicializamos al padre con los datos básicos
    // Al heredar ADSconfig de ADSBaseConfig, podemos pasar 'cfg' directamente
    : ADSBase(cfg), 
      
    // 2. Inicializamos nuestra variable local 'config'
    // Aquí se copian AUTOMÁTICAMENTE r0_ohms, serie_resistor_ohms, etc.
      config(cfg)   
{
}

// 2. DESTRUCTOR
TempADSManager::~TempADSManager() {
    if (temp_task_handle != nullptr) {
        vTaskDelete(temp_task_handle);
        temp_task_handle = nullptr;
    }
}

// 3. BEGIN
bool TempADSManager::begin() {
    // ERROR ANTERIOR: if (!ADSBase::begin()) ... -> Esto falla porque el padre es abstracto.
    
    // CORRECCIÓN: Llamamos a initADS(), la herramienta protegida que nos dio el padre
    // Esta función hace el ads->begin(addr) y ads->setGain(gain) internamente.
    if (!initADS()) {
        return false;
    }
    
    // Aquí puedes añadir configuraciones extras si quisieras
    // ads->setDataRate(RATE_ADS1115_8SPS); 
    
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
        // Leemos el canal definido en la configuración
        // Asumiendo que ADSBase tiene un método readChannel(int channel)
        int16_t adc_val = readChannel(config.num_channels); 
        
        // Calculamos temperatura
        float temp = calculatePT100Temp(adc_val);
        
        // Guardamos el valor (Aquí iría el Mutex si lo implementas en el futuro)
        _current_temperature = temp;
        
        // Esperamos el tiempo definido en la config
        vTaskDelay(pdMS_TO_TICKS(config.process_interval_ms));
    }
}

// 7. CÁLCULO PT100
float TempADSManager::calculatePT100Temp(int16_t raw_adc) {
    // Corrección: Usamos 'config' que es el nombre definido en tu .h
    
    if (raw_adc <= 0) return -99.9; // Error o desconectado

    // Ganancia 1: +/- 4.096V -> 1 bit = 0.125mV (Ajustar según tu ganancia real)
    float voltage = raw_adc * 0.000125f;
    
    float vcc = 3.3f; 
    // Fórmula divisor de tensión usando las variables de tu config
    float r_pt100 = (voltage * config.serie_resistor_ohms) / (vcc - voltage);

    // Fórmula lineal simple para temperatura
    float temp = (r_pt100 - config.r0_ohms) / (config.r0_ohms * 0.00385f);
    
    return temp;
}

// 8. GETTER
float TempADSManager::getTemperature() {
    // Aquí devolverías el valor protegido por mutex si existiera
    return _current_temperature;
}