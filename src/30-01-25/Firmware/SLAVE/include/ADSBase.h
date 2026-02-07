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
    int process_interval_ms;
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
            // --- Lecturas Single-Ended (Normales) ---
            case 0: return ads->readADC_SingleEnded(0);
            case 1: return ads->readADC_SingleEnded(1);
            case 2: return ads->readADC_SingleEnded(2);
            case 3: return ads->readADC_SingleEnded(3);
            
            // --- Lecturas Diferenciales ---
            case 10: // Código especial para Par 0-1
                return ads->readADC_Differential_0_1();
            case 30: // Código especial para Par 0-3
                return ads->readADC_Differential_0_3();
            case 31: // Código especial para Par 1-3
                return ads->readADC_Differential_1_3();
            case 32: // Código especial para Par 2-3
                return ads->readADC_Differential_2_3();
                
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
    virtual float getLatest(int channel) = 0; // Pure virtual
    virtual int getHistory(int channel, float* buffer, int count) = 0; // Pure virtual
};

// Función auxiliar para obtener Volts por Bit según ganancia y chip
static float getVoltsPerBit(adsGain_t gain, ADSType type) {
    float v_fsr = 0.0f; // Voltaje Full Scale Range (+/-)
    
    switch(gain) {
        case GAIN_TWOTHIRDS: v_fsr = 6.144f; break;
        case GAIN_ONE:       v_fsr = 4.096f; break;
        case GAIN_TWO:       v_fsr = 2.048f; break;
        case GAIN_FOUR:      v_fsr = 1.024f; break;
        case GAIN_EIGHT:     v_fsr = 0.512f; break;
        case GAIN_SIXTEEN:   v_fsr = 0.256f; break;
        default:             v_fsr = 6.144f;
    }

    // ADS1015 corre de -2048 a +2047 (12 bits -> shift 4 = efective 11 bits + signo) -> Rango total / 2048
    // ADS1115 corre de -32768 a +32767 (16 bits) -> Rango total / 32768
    
    // NOTA: La librería de Adafruit devuelve los valores del 1015 ya ajustados 
    // a rango de 12 bits pero desplazados como int16, pero para cálculo puro de voltaje:
    
    if (type == ADSType::ADS1015) {
         // Adafruit normaliza un poco raro el 1015, usualmente se divide por 2048 si es raw 12-bit
         // Pero si la librería devuelve el valor "shifteado", puede tratarse igual en rango.
         // Para simplificar, Adafruit provee un método computeVolts(bit), 
         // pero si quieres el factor RAW matemático:
         return v_fsr / 2048.0f; // ADS1015 (11 bits efectivos positivos)
    } else {
         return v_fsr / 32768.0f; // ADS1115 (15 bits efectivos positivos)
    }
}

#endif