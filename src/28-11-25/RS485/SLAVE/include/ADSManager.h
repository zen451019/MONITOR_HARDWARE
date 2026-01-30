#ifndef ADS_MANAGER_H
#define ADS_MANAGER_H

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Enum para seleccionar el tipo de ADC
enum class ADSType { ADS1015, ADS1115 };

// Estructura para la configuración del ADC
struct ADSConfig {
    ADSType type;
    uint8_t i2c_addr;
    int alert_pin;
    adsGain_t gain;
    int samples_per_second;
    int num_channels;
    int fifo_size;
    int history_size;
    int process_interval_ms;
    const float* conversion_factors;
};

// Estructura para una muestra leída
struct ADCSample {
    int16_t value;
    uint8_t channel;
};

// Estructura para el buffer RMS
struct RMS_FIFO {
    int16_t* buffer;
    int head;
    int count;
    int64_t sum_x;
    int64_t sum_x2;
};

class ADSManager {
private:
    ADSConfig config;
    Adafruit_ADS1X15* ads;
    
    // Adquisición
    QueueHandle_t sample_queue;
    volatile bool data_ready;
    volatile uint8_t current_channel;
    
    // Procesamiento RMS
    RMS_FIFO* fifos;
    float** rms_histories;
    volatile int rms_history_head;
    SemaphoreHandle_t rms_mutex;
    
    // Tareas
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
    
    bool begin();
    void startSampling();
    
    // API para obtener datos procesados
    int getRMSHistory(int channel, float* output_buffer, int count);
    float getLatestRMS(int channel);
    void getRMSAllChannels(float* output_array);
};

#endif // ADS_MANAGER_H