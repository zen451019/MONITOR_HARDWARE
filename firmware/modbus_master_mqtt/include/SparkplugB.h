#ifndef SPARKPLUG_B_H
#define SPARKPLUG_B_H

#include <cstdint>
#include <cstddef>

#include "MeasureFrame.h"
#include "DevicesConfig.h"

class MqttTransport;

namespace spb {

struct Config {
    const char* groupId;
    const char* edgeNodeId;
    const char* ntp1;
    const char* ntp2;
};

struct NodeHealth {
    uint64_t uptimeMs;
    int32_t  rssiDbm;
    uint32_t freeHeapBytes;
    bool     scanOk;
};

void begin(const Config& cfg);

bool     timeReady();
uint64_t nowMs();

// Llamar antes de mqtt.connect(): incrementa bdSeq y prepara el NDEATH.
void        prepareDeath();
const char* deathTopic();
const uint8_t* deathPayload();
size_t      deathPayloadLen();

// Llamar tras CONNACK.
bool publishBirth(MqttTransport& mqtt,
                  const NodeHealth& health,
                  const MeasureFrame* lastFrames,
                  const bool* hasLast);

// DBIRTH individual (recuperacion de un Device).
bool publishDeviceBirth(MqttTransport& mqtt,
                        uint8_t deviceIndex,
                        const MeasureFrame* frame);

// DDATA; si frame.cycleEnd tambien publica NDATA con la salud del nodo.
bool publishData(MqttTransport& mqtt,
                 const MeasureFrame& frame,
                 const NodeHealth& health);

// NDATA independiente (por si el ultimo Device del ciclo esta offline).
bool publishNodeData(MqttTransport& mqtt,
                     const NodeHealth& health,
                     uint64_t tsMs);

// DDEATH individual.
bool publishDeviceDeath(MqttTransport& mqtt,
                        uint8_t deviceIndex,
                        uint64_t tsMs);

const char* ncmdTopic();
bool        handleCommand(const uint8_t* data, size_t len);
bool        rebirthPending();
void        clearRebirth();

} // namespace spb

#endif // SPARKPLUG_B_H
