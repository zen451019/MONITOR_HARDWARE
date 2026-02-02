#ifndef ADS_BASE_H
#define ADS_BASE_H

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

enum class ADSType { ADS1015, ADS1115 };

// ===== CONFIGURACIÓN BASE (lo que TODO ADC necesita) =====
struct ADSBaseConfig {
    ADSType type;
    uint8_t i2c_addr;
    adsGain_t gain;
    int num_channels;
    const float* conversion_factors;
};

// ===== CLASE BASE ABSTRACTA =====
class ADSBase {
protected:
    // Variables COMUNES a todas las clases hijas
    Adafruit_ADS1X15* ads;
    ADSBaseConfig base_config;
    SemaphoreHandle_t data_mutex;
    
    // Métodos PROTEGIDOS (solo accesibles por clases hijas)
    bool initADS() {
        if (!ads->begin(base_config.i2c_addr)) {
            return false;
        }
        ads->setGain(base_config.gain);
        return true;
    }
    
    // Función auxiliar para leer un canal
    int16_t readChannel(uint8_t channel) {
        switch(channel) {
            case 0: return ads->readADC_SingleEnded(0);
            case 1: return ads->readADC_SingleEnded(1);
            case 2: return ads->readADC_SingleEnded(2);
            case 3: return ads->readADC_SingleEnded(3);
            default: return 0;
        }
    }

public:
    ADSBase(const ADSBaseConfig& cfg) : base_config(cfg) {
        // Crear el objeto ADS según el tipo
        if (cfg.type == ADSType::ADS1015) {
            ads = new Adafruit_ADS1015();
        } else {
            ads = new Adafruit_ADS1115();
        }
        data_mutex = xSemaphoreCreateMutex();
    }
    
    virtual ~ADSBase() {
        delete ads;
        if (data_mutex) vSemaphoreDelete(data_mutex);
    }
    
    // ===== MÉTODOS VIRTUALES PUROS (obligatorios para clases hijas) =====
    virtual bool begin() = 0;
    virtual void startSampling() = 0;
};

#endif