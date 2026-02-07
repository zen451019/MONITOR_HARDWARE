#ifndef TEMP_ADS_MANAGER_H
#define TEMP_ADS_MANAGER_H

#include "ADSBase.h"
#include <freertos/task.h>
#include <freertos/queue.h>


struct ADSconfig : public ADSBaseConfig {
    int serie_resistor_ohms;
    int r0_ohms;
    uint16_t sampling_rate;
    int history_size; // <--- NUEVO: Para igualar a ADSManager

    ADSconfig(ADSType t, uint8_t addr, adsGain_t g, int process_interval_ms,
              int series_resistor, int r0, uint16_t sampling_rate, int hist_size)
        : ADSBaseConfig{t, addr, g, process_interval_ms},
          serie_resistor_ohms(series_resistor),
          r0_ohms(r0),
          sampling_rate(sampling_rate),
          history_size(hist_size) {} // <--- Init
    ADSconfig() 
        : ADSBaseConfig{ADSType::ADS1015, 0x48, GAIN_TWOTHIRDS, 0},
          serie_resistor_ohms(0),
          r0_ohms(100),
          sampling_rate(128),
          history_size(0) {}
};

class TempADSManager : public ADSBase {
private:
    ADSconfig config;
    TaskHandle_t temp_task_handle = nullptr;
    
    // --- NUEVO PARA ALMACENAMIENTO ---
    float* temp_history = nullptr; // Buffer circular
    int history_head = 0;
    SemaphoreHandle_t data_mutex;  // Para proteger la lectura/escritura del historial
    // ---------------------------------

    static void temp_task_trampoline(void* arg);
    void temp_task_body();
    float getRaw(uint8_t channel, uint8_t NUM_SAMPLES = 1);

public:
    TempADSManager(const ADSconfig& config);
    virtual ~TempADSManager();

    bool begin() override;
    void startSampling() override;
    
    // --- MÉTODOS ESTANDARIZADOS (Iguales a ADSManager) ---
    float getLatest(int channel = 0); // Renombrado para que sea genérico
    int getHistory(int channel, float* output_buffer, int count);
};

#endif