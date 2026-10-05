#ifndef ADS_MANAGER_H
#define ADS_MANAGER_H

#include "ADSBase.h"
#include <freertos/task.h>
#include <freertos/queue.h>

// ===== CONFIGURACIÓN EXTENDIDA (hereda de ADSBaseConfig) =====
struct ADSConfig : public ADSBaseConfig {
    int alert_pin;
    int samples_per_second;
    int fifo_size;
    int history_size;
    int num_channels;
    const float* conversion_factors;
    
    // ← AGREGAR: Constructor para facilitar la inicialización
    ADSConfig(ADSType t, uint8_t addr, adsGain_t g, int interval, int ch, const float* factors,
              int alert, int sps, int fifo, int hist)
        : ADSBaseConfig{t, addr, g, interval},  // Inicializar clase base
          num_channels(ch),
          conversion_factors(factors),
          alert_pin(alert),
          samples_per_second(sps),
          fifo_size(fifo),
          history_size(hist) {}
    
    ADSConfig() 
        : ADSBaseConfig{ADSType::ADS1015, 0x48, GAIN_TWOTHIRDS, 0},
          num_channels(0),
          conversion_factors(nullptr),
          alert_pin(-1),
          samples_per_second(0),
          fifo_size(0),
          history_size(0) {}
};

// Estructura para una muestra leída (sin cambios)
struct ADCSample {
    int16_t value;
    uint8_t channel;
};

// Estructura para el buffer RMS (sin cambios)
struct RMS_FIFO {
    int16_t* buffer;
    int head;
    int count;
    int64_t sum_x;
    int64_t sum_x2;
};

// ===== CLASE HIJA (hereda de ADSBase) =====
class ADSManager : public ADSBase {
private:
    ADSConfig config;
    
    // Adquisición (sin cambios)
    QueueHandle_t sample_queue;
    volatile bool data_ready;
    volatile uint8_t current_channel;
    
    // Procesamiento RMS (sin cambios)
    RMS_FIFO* fifos;
    float** rms_histories;
    volatile int rms_history_head;
    SemaphoreHandle_t rms_mutex;
    
    // Tareas (sin cambios)
    TaskHandle_t acquisition_task_handle;
    TaskHandle_t processing_task_handle;
    
    static void IRAM_ATTR isr_handler(void* arg);
    static void acquisition_task_trampoline(void* arg);
    static void processing_task_trampoline(void* arg);
    void acquisition_task_body();
    void processing_task_body();

public:
    ADSManager(const ADSConfig& config);
    ~ADSManager();
    
    bool begin() override;
    void startSampling() override;
    
    // API para obtener datos procesados (sin cambios)
    int getHistory(int channel, float* output_buffer, int count);
    float getLatest(int channel);
    void getRMSAllChannels(float* output_array);
};

#endif // ADS_MANAGER_H