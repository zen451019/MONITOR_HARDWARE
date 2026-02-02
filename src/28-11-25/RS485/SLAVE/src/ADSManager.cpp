#include "ADSManager.h"

// ===== CONSTRUCTOR =====
ADSManager::ADSManager(const ADSConfig& cfg) 
    // ← CAMBIO: Llamar al constructor de ADSBase primero
    : ADSBase({cfg.type, cfg.i2c_addr, cfg.gain, cfg.num_channels, cfg.conversion_factors}),
      config(cfg),  // Guardar config completo
      data_ready(false), 
      current_channel(0) {
    
    // Crear FIFOs (sin cambios)
    fifos = new RMS_FIFO[config.num_channels];
    for (int i = 0; i < config.num_channels; i++) {
        fifos[i].buffer = new int16_t[config.fifo_size];
        fifos[i].head = 0;
        fifos[i].count = 0;
        fifos[i].sum_x = 0;
        fifos[i].sum_x2 = 0;
    }
    
    // Crear historiales
    rms_histories = new float*[config.num_channels];
    for (int i = 0; i < config.num_channels; i++) {
        rms_histories[i] = new float[config.history_size]();
    }
    rms_history_head = 0;
    
    sample_queue = xQueueCreate(config.fifo_size, sizeof(ADCSample));
    rms_mutex = xSemaphoreCreateMutex();
}

// ===== DESTRUCTOR =====
ADSManager::~ADSManager() {
    for (int i = 0; i < config.num_channels; i++) {
        delete[] fifos[i].buffer;
        delete[] rms_histories[i];
    }
    delete[] fifos;
    delete[] rms_histories;
    
    vQueueDelete(sample_queue);
    vSemaphoreDelete(rms_mutex);
}

// ===== ISR =====
void IRAM_ATTR ADSManager::isr_handler(void* arg) {
    ADSManager* instance = static_cast<ADSManager*>(arg);
    instance->data_ready = true;
}

// ===== BEGIN =====
bool ADSManager::begin() {
    if (!initADS()) {
        return false;
    }
    
    if (config.type == ADSType::ADS1015) {
        ads->setDataRate(RATE_ADS1015_3300SPS);
    } else {
        ads->setDataRate(RATE_ADS1115_860SPS);
    }
    
    if (config.alert_pin != -1) {
        pinMode(config.alert_pin, INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(config.alert_pin), isr_handler, this, FALLING);
    }
    
    return true;
}

void ADSManager::startSampling() {
    xTaskCreatePinnedToCore(acquisition_task_trampoline, "ADS_Acq", 4096, this, 5, &acquisition_task_handle, 0);
    xTaskCreatePinnedToCore(processing_task_trampoline, "ADS_Proc", 4096, this, 3, &processing_task_handle, 0);
}

void ADSManager::acquisition_task_trampoline(void* arg) {
    static_cast<ADSManager*>(arg)->acquisition_task_body();
}

void ADSManager::acquisition_task_body() {
    ads->startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
    
    while (true) {
        if (data_ready) {
            data_ready = false;
            
            ADCSample sample;
            sample.value = ads->getLastConversionResults();
            sample.channel = current_channel;
            
            xQueueSend(sample_queue, &sample, 0);
            
            current_channel = (current_channel + 1) % config.num_channels;
            
            uint16_t mux_config;
            switch(current_channel) {
                case 0: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_0; break;
                case 1: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_1; break;
                case 2: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_2; break;
                default: mux_config = ADS1X15_REG_CONFIG_MUX_SINGLE_0; break;
            }
            ads->startADCReading(mux_config, false);
        } else {
            vTaskDelay(1);
        }
    }
}

void ADSManager::processing_task_trampoline(void* arg) {
    static_cast<ADSManager*>(arg)->processing_task_body();
}

void ADSManager::processing_task_body() {
    ADCSample sample;
    TickType_t last_process_time = xTaskGetTickCount();
    
    while (true) {
        while (xQueueReceive(sample_queue, &sample, 0) == pdTRUE) {
            if (sample.channel < config.num_channels) {
                RMS_FIFO& fifo = fifos[sample.channel];
                
                if (fifo.count == config.fifo_size) {
                    int16_t old_val = fifo.buffer[fifo.head];
                    fifo.sum_x -= old_val;
                    fifo.sum_x2 -= (int64_t)old_val * old_val;
                } else {
                    fifo.count++;
                }
                
                fifo.buffer[fifo.head] = sample.value;
                fifo.sum_x += sample.value;
                fifo.sum_x2 += (int64_t)sample.value * sample.value;
                fifo.head = (fifo.head + 1) % config.fifo_size;
            }
        }
        
        if (xTaskGetTickCount() - last_process_time >= pdMS_TO_TICKS(config.process_interval_ms)) {
            last_process_time = xTaskGetTickCount();
            
            if (xSemaphoreTake(rms_mutex, portMAX_DELAY) == pdTRUE) {
                for (int ch = 0; ch < config.num_channels; ch++) {
                    if (fifos[ch].count > 0) {
                        double mean = (double)fifos[ch].sum_x / fifos[ch].count;
                        double var = ((double)fifos[ch].sum_x2 / fifos[ch].count) - (mean * mean);
                        float rms = sqrt(var < 0 ? 0 : var);
                        
                        rms_histories[ch][rms_history_head] = rms * config.conversion_factors[ch];
                    }
                }
                rms_history_head = (rms_history_head + 1) % config.history_size;
                xSemaphoreGive(rms_mutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int ADSManager::getRMSHistory(int channel, float* output_buffer, int count) {
    if (channel < 0 || channel >= config.num_channels || count > config.history_size) {
        return 0;
    }
    
    if (xSemaphoreTake(rms_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int most_recent = (rms_history_head == 0) ? (config.history_size - 1) : (rms_history_head - 1);
        
        for (int i = 0; i < count; i++) {
            int idx = (most_recent - (count - 1 - i) + config.history_size) % config.history_size;
            output_buffer[i] = rms_histories[channel][idx];
        }
        
        xSemaphoreGive(rms_mutex);
        return count;
    }
    return 0;
}

float ADSManager::getLatestRMS(int channel) {
    float value = 0;
    getRMSHistory(channel, &value, 1);
    return value;
}

void ADSManager::getRMSAllChannels(float* output_array) {
    for (int ch = 0; ch < config.num_channels; ch++) {
        output_array[ch] = getLatestRMS(ch);
    }
}