#ifndef MEASURE_FRAME_H
#define MEASURE_FRAME_H

#include <cstdint>

// Maximo de tipos de sensor y canales por tipo. Coincide con SensorRegistry.h.
constexpr uint8_t MQTT_MAX_SENSORS  = 8;
constexpr uint8_t MQTT_MAX_CHANNELS = 4;

// Valores decodificados de un tipo de sensor dentro de un Device.
struct SensorValues {
    uint8_t sensorId;
    uint8_t channels;
    uint8_t validMask;                       // bit c = value[c] valido
    float   value[MQTT_MAX_CHANNELS];
};

// Resultado de un ciclo de polling para un Device concreto.
struct MeasureFrame {
    uint8_t  deviceIndex;                    // indice en kDevices
    uint32_t cycleId;
    uint64_t tsMs;                           // epoch UTC en ms (0 si NTP aun no sincronizo)
    uint8_t  count;                          // sensores presentes en sensors[]
    bool     scanOk;                         // todos los bloques de este Device respondieron
    bool     cycleEnd;                       // true en el ultimo frame del ciclo
    bool     cycleOk;                        // todos los Devices del ciclo respondieron
    uint8_t  failStreak;                     // ciclos consecutivos sin ninguna respuesta
    SensorValues sensors[MQTT_MAX_SENSORS];
};

#endif // MEASURE_FRAME_H
