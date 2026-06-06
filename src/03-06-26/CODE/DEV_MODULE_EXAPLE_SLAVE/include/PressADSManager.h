#ifndef PRESS_ADS_MANAGER_H
#define PRESS_ADS_MANAGER_H

#include "ADSBase.h"
#include <freertos/task.h>
#include <freertos/queue.h>

struct ADSconfig : public ADSBaseConfig {
    // Parámetros para sensor de presión 0.5-4.5V
    float min_voltage;           // Voltaje mínimo (0.5V)
    float max_voltage;           // Voltaje máximo (4.5V)
    float min_pressure;          // Presión mínima en unidades (ej: 0 PSI, 0 bar)
    float max_pressure;          // Presión máxima en unidades
    uint8_t active_channels;     // Máscara de bits para canales activos (0-3 bits para hasta 4 canales)
    uint16_t sampling_rate;      // Tasa de muestreo
    int history_size;            // Tamaño del historial
    u_int8_t NUM_SAMPLES;       // Número de muestras para promediar en cada lectura (para reducir ruido)

    ADSconfig(ADSType t, uint8_t addr, adsGain_t g, int process_interval_ms,
              float v_min, float v_max, float p_min, float p_max,
              uint8_t channels, uint16_t sampling_rate, uint8_t NUM_SAMPLES, int hist_size)
        : ADSBaseConfig{t, addr, g, process_interval_ms},
          min_voltage(v_min),
          max_voltage(v_max),
          min_pressure(p_min),
          max_pressure(p_max),
          active_channels(channels),
          sampling_rate(sampling_rate),
          NUM_SAMPLES(NUM_SAMPLES),
          history_size(hist_size) {}
    
    ADSconfig() 
        : ADSBaseConfig{ADSType::ADS1015, 0x48, GAIN_TWOTHIRDS, 0},
          min_voltage(0.5f),
          max_voltage(4.5f),
          min_pressure(0.0f),
          max_pressure(100.0f),      // Ajustar según sensor
          active_channels(0b0001),    // Solo canal 0 activo por defecto
          sampling_rate(128),
          NUM_SAMPLES(1),
          history_size(10) {}
    
};

struct ADCSample {
    int16_t value;
    uint8_t channel;
};

class PressADSManager : public ADSBase {
    private:
        ADSconfig config;
        TaskHandle_t press_task_handle = nullptr;

        // --- GESTIÓN DE DATOS ---
        // Array de punteros para buffers circulares (soporta hasta 4 canales del ADS1115)
        // pressure_histories[0] -> Historial Canal 0, etc.
        // Si el canal no está activo en 'active_channels', el puntero será nullptr.
        float* pressure_histories[4] = {nullptr, nullptr, nullptr, nullptr};
        
        // Cabeza (índice de escritura) para cada buffer circular
        int history_heads[4] = {0, 0, 0, 0};
        
        // Mutex para proteger la lectura/escritura simultánea de los historiales
        SemaphoreHandle_t data_mutex;

        // --- TAREAS ---
        static void press_task_trampoline(void* arg);
        void press_task_body();

        // Helper interno para convertir voltaje a unidades de presión
        float convertVoltageToPressure(float voltage);

        float getRaw(uint8_t channel, uint8_t NUM_SAMPLES = 1);

    public:
        PressADSManager(const ADSconfig& config);
        virtual ~PressADSManager();

        bool begin() override;
        void startSampling() override;

        // --- API ESTANDARIZADA (Igual a ADSManager/TempADSManager) ---
        
        // Obtiene el último valor de presión calculado para un canal específico
        float getLatest(int channel);

        // Copia el historial de presión de un canal específico al buffer de salida
        // Retorna la cantidad de datos copiados
        int getHistory(int channel, float* output_buffer, int count);
};

#endif