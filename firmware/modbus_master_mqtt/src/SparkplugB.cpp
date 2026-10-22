#include "SparkplugB.h"
#include "SparkplugConfig.h"
#include "Log.h"

#include <AsyncMqttClient.h>
#include <Arduino.h>
#include <Preferences.h>
#include <sys/time.h>
#include <time.h>

#include <atomic>
#include <cstdio>
#include <cstring>

#include <pb_encode.h>
#include <pb_decode.h>
#include "sparkplug_b.pb.h"

namespace {

using Metric  = org_eclipse_tahu_protobuf_Payload_Metric;
using Payload = org_eclipse_tahu_protobuf_Payload;

constexpr const char* kNvsNamespace = "spb";
constexpr const char* kNvsKey       = "bdseq";
constexpr size_t kTopicSize = 160;

spb::Config g_cfg{};

char g_topicNbirth[kTopicSize];
char g_topicNdeath[kTopicSize];
char g_topicNdata[kTopicSize];
char g_topicNcmd[kTopicSize];
char g_topicDbirth[kMaxDevices][kTopicSize];
char g_topicDdata[kMaxDevices][kTopicSize];
char g_topicDdeath[kMaxDevices][kTopicSize];

uint8_t g_bdSeqNext = 0;
uint8_t g_bdSeq     = 0;
uint8_t g_seq       = 0;

uint8_t g_deathBuf[64];
size_t  g_deathLen = 0;

uint8_t g_txBuf[SPB_MAX_PAYLOAD_BYTES];

Payload g_payload;      // solo tarea de red (encode)
Payload g_cmdPayload;   // solo callback MQTT (decode)

std::atomic<bool> g_rebirthRequested{false};
uint32_t g_lastRebirthMs = 0;

void metricSetName(Metric& m, const char* name) {
    m.has_name = true;
    strncpy(m.name, name, sizeof(m.name) - 1);
    m.name[sizeof(m.name) - 1] = '\0';
}

void metricSetAlias(Metric& m, uint64_t alias) {
    m.has_alias = true;
    m.alias = alias;
}

void metricSetTimestamp(Metric& m, uint64_t ts) {
    if (ts > 0) {
        m.has_timestamp = true;
        m.timestamp = ts;
    }
}

void metricSetDatatype(Metric& m, uint32_t datatype) {
    m.has_datatype = true;
    m.datatype = datatype;
}

void metricSetNull(Metric& m) {
    m.has_is_null = true;
    m.is_null = true;
    m.which_value = 0;
}

void metricSetFloat(Metric& m, float value) {
    m.has_is_null = true;
    m.is_null = false;
    m.which_value = org_eclipse_tahu_protobuf_Payload_Metric_float_value_tag;
    m.value.float_value = value;
}

void metricSetDouble(Metric& m, double value) {
    m.has_is_null = true;
    m.is_null = false;
    m.which_value = org_eclipse_tahu_protobuf_Payload_Metric_double_value_tag;
    m.value.double_value = value;
}

void metricSetBool(Metric& m, bool value) {
    m.has_is_null = true;
    m.is_null = false;
    m.which_value = org_eclipse_tahu_protobuf_Payload_Metric_boolean_value_tag;
    m.value.boolean_value = value;
}

void metricSetInt32(Metric& m, int32_t value) {
    m.has_is_null = true;
    m.is_null = false;
    m.which_value = org_eclipse_tahu_protobuf_Payload_Metric_int_value_tag;
    m.value.int_value = (uint32_t)value;
}

void metricSetUInt32(Metric& m, uint32_t value) {
    m.has_is_null = true;
    m.is_null = false;
    m.which_value = org_eclipse_tahu_protobuf_Payload_Metric_int_value_tag;
    m.value.int_value = value;
}

void metricSetInt64(Metric& m, int64_t value) {
    m.has_is_null = true;
    m.is_null = false;
    m.which_value = org_eclipse_tahu_protobuf_Payload_Metric_long_value_tag;
    m.value.long_value = (uint64_t)value;
}

void metricSetUInt64(Metric& m, uint64_t value) {
    m.has_is_null = true;
    m.is_null = false;
    m.which_value = org_eclipse_tahu_protobuf_Payload_Metric_long_value_tag;
    m.value.long_value = value;
}

size_t encodeToBuffer(uint8_t* buf, size_t cap, const Payload& payload) {
    pb_ostream_t stream = pb_ostream_from_buffer(buf, cap);
    if (!pb_encode(&stream, org_eclipse_tahu_protobuf_Payload_fields, &payload)) {
        LOG_E("Sparkplug: pb_encode fallo: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool publishPayload(AsyncMqttClient& mqtt, const char* topic, uint8_t qos,
                    const Payload& payload, const char* what) {
    const size_t len = encodeToBuffer(g_txBuf, sizeof(g_txBuf), payload);
    if (len == 0) return false;

    const uint16_t packetId = mqtt.publish(
        topic, qos, false, reinterpret_cast<const char*>(g_txBuf), len);

    if (packetId == 0) {
        LOG_W("Sparkplug: publish fallo (%s, %s)", what, topic);
        return false;
    }

    LOG_D("Sparkplug: %s -> %s (%zu bytes, seq=%llu)",
          what, topic, len, (unsigned long long)payload.seq);
    return true;
}

void addBdSeqMetric(Payload& p, uint8_t bdSeq, uint64_t ts, bool withAlias) {
    Metric& m = p.metrics[p.metrics_count++];
    metricSetName(m, SPB_METRIC_BDSEQ);
    if (withAlias) metricSetAlias(m, SPB_ALIAS_NODE_BDSEQ);
    metricSetDatatype(m, SPB_DT_INT64);
    metricSetTimestamp(m, ts);
    metricSetInt64(m, (int64_t)bdSeq);
}

void addRebirthMetric(Payload& p, uint64_t ts) {
    Metric& m = p.metrics[p.metrics_count++];
    metricSetName(m, SPB_METRIC_REBIRTH);
    metricSetAlias(m, SPB_ALIAS_NODE_REBIRTH);
    metricSetDatatype(m, SPB_DT_BOOLEAN);
    metricSetTimestamp(m, ts);
    metricSetBool(m, false);
}

void addNodeHealthMetrics(Payload& p, const spb::NodeHealth& h,
                          uint64_t ts, bool isBirth) {
    {
        Metric& m = p.metrics[p.metrics_count++];
        if (isBirth) {
            metricSetName(m, SPB_METRIC_UPTIME);
            metricSetDatatype(m, SPB_DT_UINT64);
        }
        metricSetAlias(m, SPB_ALIAS_NODE_UPTIME);
        metricSetTimestamp(m, ts);
        metricSetUInt64(m, h.uptimeMs);
    }
    {
        Metric& m = p.metrics[p.metrics_count++];
        if (isBirth) {
            metricSetName(m, SPB_METRIC_RSSI);
            metricSetDatatype(m, SPB_DT_INT32);
        }
        metricSetAlias(m, SPB_ALIAS_NODE_RSSI);
        metricSetTimestamp(m, ts);
        metricSetInt32(m, h.rssiDbm);
    }
    {
        Metric& m = p.metrics[p.metrics_count++];
        if (isBirth) {
            metricSetName(m, SPB_METRIC_FREE_HEAP);
            metricSetDatatype(m, SPB_DT_UINT32);
        }
        metricSetAlias(m, SPB_ALIAS_NODE_FREE_HEAP);
        metricSetTimestamp(m, ts);
        metricSetUInt32(m, h.freeHeapBytes);
    }
    {
        Metric& m = p.metrics[p.metrics_count++];
        if (isBirth) {
            metricSetName(m, SPB_METRIC_SCAN_OK);
            metricSetDatatype(m, SPB_DT_BOOLEAN);
        }
        metricSetAlias(m, SPB_ALIAS_NODE_SCAN_OK);
        metricSetTimestamp(m, ts);
        metricSetBool(m, h.scanOk);
    }
}

bool findSignalValue(const MeasureFrame* frame, uint8_t sensorType, uint8_t channel,
                     float& out) {
    if (frame == nullptr || channel >= MQTT_MAX_CHANNELS) return false;

    for (uint8_t i = 0; i < frame->count; ++i) {
        const SensorValues& sv = frame->sensors[i];
        if (sv.sensorId != sensorType) continue;
        if (channel >= sv.channels) return false;
        if ((sv.validMask & (1u << channel)) == 0) return false;
        out = sv.value[channel];
        return true;
    }
    return false;
}

void fillDeviceMetrics(Payload& p, const DeviceConfig& dev,
                       const MeasureFrame* frame, uint64_t ts, bool isBirth) {
    p.metrics_count = 0;

    for (size_t i = 0; i < dev.metricCount && p.metrics_count < SPB_MAX_METRICS; ++i) {
        const SpbMetricDef& def = dev.metrics[i];
        Metric& m = p.metrics[p.metrics_count++];

        if (isBirth) {
            metricSetName(m, def.name);
            metricSetDatatype(m, def.datatype);
        }
        metricSetAlias(m, (uint64_t)dev.aliasBase + def.offset);
        metricSetTimestamp(m, ts);

        float value = 0.0f;
        const bool valid = findSignalValue(frame, def.sensorType, def.channel, value);

        if (!valid) {
            metricSetNull(m);
        } else if (def.datatype == SPB_DT_DOUBLE) {
            metricSetDouble(m, (double)value);
        } else {
            metricSetFloat(m, value);
        }
    }
}

bool publishDeviceBirthInternal(AsyncMqttClient& mqtt, uint8_t deviceIndex,
                                const MeasureFrame* frame) {
    if (deviceIndex >= kDeviceCount) return false;

    const DeviceConfig& dev = kDevices[deviceIndex];
    const uint64_t ts = (frame != nullptr && frame->tsMs != 0) ? frame->tsMs : spb::nowMs();
    if (ts == 0) return false;

    memset(&g_payload, 0, sizeof(g_payload));
    g_payload.has_timestamp = (ts > 0);
    g_payload.timestamp = ts;
    g_payload.has_seq = true;
    g_payload.seq = g_seq;

    fillDeviceMetrics(g_payload, dev, frame, ts, true);

    if (!publishPayload(mqtt, g_topicDbirth[deviceIndex], 0, g_payload, "DBIRTH")) {
        return false;
    }

    g_seq = (uint8_t)(g_seq + 1);
    return true;
}

bool publishNodeDataInternal(AsyncMqttClient& mqtt, const spb::NodeHealth& health,
                             uint64_t ts) {
    if (ts == 0) return false;

    memset(&g_payload, 0, sizeof(g_payload));
    g_payload.has_timestamp = (ts > 0);
    g_payload.timestamp = ts;
    g_payload.has_seq = true;
    g_payload.seq = g_seq;
    g_payload.metrics_count = 0;

    addNodeHealthMetrics(g_payload, health, ts, false);

    if (!publishPayload(mqtt, g_topicNdata, 0, g_payload, "NDATA")) {
        return false;
    }

    g_seq = (uint8_t)(g_seq + 1);
    return true;
}

} // namespace

namespace spb {

void begin(const Config& cfg) {
    g_cfg = cfg;

    snprintf(g_topicNbirth, sizeof(g_topicNbirth),
             "spBv1.0/%s/NBIRTH/%s", cfg.groupId, cfg.edgeNodeId);
    snprintf(g_topicNdeath, sizeof(g_topicNdeath),
             "spBv1.0/%s/NDEATH/%s", cfg.groupId, cfg.edgeNodeId);
    snprintf(g_topicNdata, sizeof(g_topicNdata),
             "spBv1.0/%s/NDATA/%s", cfg.groupId, cfg.edgeNodeId);
    snprintf(g_topicNcmd, sizeof(g_topicNcmd),
             "spBv1.0/%s/NCMD/%s", cfg.groupId, cfg.edgeNodeId);

    for (uint8_t d = 0; d < kDeviceCount; ++d) {
        snprintf(g_topicDbirth[d], sizeof(g_topicDbirth[d]),
                 "spBv1.0/%s/DBIRTH/%s/%s",
                 cfg.groupId, cfg.edgeNodeId, kDevices[d].deviceId);
        snprintf(g_topicDdata[d], sizeof(g_topicDdata[d]),
                 "spBv1.0/%s/DDATA/%s/%s",
                 cfg.groupId, cfg.edgeNodeId, kDevices[d].deviceId);
        snprintf(g_topicDdeath[d], sizeof(g_topicDdeath[d]),
                 "spBv1.0/%s/DDEATH/%s/%s",
                 cfg.groupId, cfg.edgeNodeId, kDevices[d].deviceId);
    }

    Preferences prefs;
    if (prefs.begin(kNvsNamespace, false)) {
        g_bdSeqNext = prefs.getUChar(kNvsKey, 0);
        prefs.end();
    }

    configTime(0, 0, cfg.ntp1, cfg.ntp2);

    uint32_t nextAlias = SPB_NODE_ALIAS_END + 1;
    for (uint8_t d = 0; d < kDeviceCount; ++d) {
        const DeviceConfig& dev = kDevices[d];
        if (dev.metricCount > SPB_MAX_METRICS) {
            LOG_E("Sparkplug: Device %s tiene %zu metricas (max %zu)",
                  dev.deviceId, dev.metricCount, SPB_MAX_METRICS);
        }
        if (dev.aliasBase < nextAlias) {
            LOG_E("Sparkplug: aliasBase de %s (%u) se solapa (siguiente libre=%lu)",
                  dev.deviceId, (unsigned)dev.aliasBase, (unsigned long)nextAlias);
        }
        nextAlias = (uint32_t)dev.aliasBase + dev.metricCount;
    }

    LOG_I("Sparkplug: grupo=%s nodo=%s devices=%u bdSeqNext=%u",
          cfg.groupId, cfg.edgeNodeId, (unsigned)kDeviceCount, (unsigned)g_bdSeqNext);
}

bool timeReady() {
    return time(nullptr) > 1700000000;
}

uint64_t nowMs() {
    if (!timeReady()) return 0;

    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

void prepareDeath() {
    // La spec exige incrementar bdSeq en cada nuevo CONNECT.
    g_bdSeq = g_bdSeqNext;
    g_bdSeqNext = (uint8_t)((g_bdSeq + 1) & 0xFF);

    Preferences prefs;
    if (prefs.begin(kNvsNamespace, false)) {
        prefs.putUChar(kNvsKey, g_bdSeqNext);
        prefs.end();
    }

    const uint64_t ts = nowMs();

    memset(&g_payload, 0, sizeof(g_payload));
    g_payload.has_timestamp = (ts > 0);
    g_payload.timestamp = ts;
    g_payload.has_seq = false;
    g_payload.metrics_count = 0;

    addBdSeqMetric(g_payload, g_bdSeq, ts, false);

    g_deathLen = encodeToBuffer(g_deathBuf, sizeof(g_deathBuf), g_payload);
    if (g_deathLen == 0) {
        LOG_E("Sparkplug: no se pudo construir NDEATH");
    } else {
        LOG_I("Sparkplug: NDEATH preparado (bdSeq=%u, %zu bytes)",
              (unsigned)g_bdSeq, g_deathLen);
    }
}

const char* deathTopic() {
    return g_topicNdeath;
}

const uint8_t* deathPayload() {
    return g_deathBuf;
}

size_t deathPayloadLen() {
    return g_deathLen;
}

bool publishBirth(AsyncMqttClient& mqtt, const NodeHealth& health,
                  const MeasureFrame* lastFrames, const bool* hasLast) {
    g_seq = 0;
    const uint64_t ts = nowMs();
    if (ts == 0) return false;

    memset(&g_payload, 0, sizeof(g_payload));
    g_payload.has_timestamp = (ts > 0);
    g_payload.timestamp = ts;
    g_payload.has_seq = true;
    g_payload.seq = g_seq;
    g_payload.metrics_count = 0;

    addBdSeqMetric(g_payload, g_bdSeq, ts, true);
    addRebirthMetric(g_payload, ts);
    addNodeHealthMetrics(g_payload, health, ts, true);

    if (!publishPayload(mqtt, g_topicNbirth, 0, g_payload, "NBIRTH")) {
        return false;
    }

    g_seq = 1;

    for (uint8_t d = 0; d < kDeviceCount; ++d) {
        const MeasureFrame* frame =
            (lastFrames != nullptr && hasLast != nullptr && hasLast[d]) ? &lastFrames[d] : nullptr;
        if (!publishDeviceBirthInternal(mqtt, d, frame)) {
            return false;
        }
    }

    return true;
}

bool publishDeviceBirth(AsyncMqttClient& mqtt, uint8_t deviceIndex,
                        const MeasureFrame* frame) {
    return publishDeviceBirthInternal(mqtt, deviceIndex, frame);
}

bool publishData(AsyncMqttClient& mqtt, const MeasureFrame& frame,
                 const NodeHealth& health) {
    if (frame.deviceIndex >= kDeviceCount) return false;

    const DeviceConfig& dev = kDevices[frame.deviceIndex];
    const uint64_t ts = (frame.tsMs != 0) ? frame.tsMs : nowMs();
    if (ts == 0) return false;

    memset(&g_payload, 0, sizeof(g_payload));
    g_payload.has_timestamp = (ts > 0);
    g_payload.timestamp = ts;
    g_payload.has_seq = true;
    g_payload.seq = g_seq;

    fillDeviceMetrics(g_payload, dev, &frame, ts, false);

    bool ok = publishPayload(mqtt, g_topicDdata[frame.deviceIndex], 0, g_payload, "DDATA");
    if (ok) {
        g_seq = (uint8_t)(g_seq + 1);
    }

    if (frame.cycleEnd) {
        if (!publishNodeDataInternal(mqtt, health, ts)) ok = false;
    }

    return ok;
}

bool publishNodeData(AsyncMqttClient& mqtt, const NodeHealth& health, uint64_t tsMs) {
    const uint64_t ts = (tsMs != 0) ? tsMs : nowMs();
    return publishNodeDataInternal(mqtt, health, ts);
}

bool publishDeviceDeath(AsyncMqttClient& mqtt, uint8_t deviceIndex, uint64_t tsMs) {
    if (deviceIndex >= kDeviceCount) return false;

    const uint64_t ts = (tsMs != 0) ? tsMs : nowMs();
    if (ts == 0) return false;

    memset(&g_payload, 0, sizeof(g_payload));
    g_payload.has_timestamp = (ts > 0);
    g_payload.timestamp = ts;
    g_payload.has_seq = true;
    g_payload.seq = g_seq;
    g_payload.metrics_count = 0;

    LOG_I("Sparkplug: DDEATH %s", kDevices[deviceIndex].deviceId);
    if (!publishPayload(mqtt, g_topicDdeath[deviceIndex], 0, g_payload, "DDEATH")) {
        return false;
    }

    g_seq = (uint8_t)(g_seq + 1);
    return true;
}

const char* ncmdTopic() {
    return g_topicNcmd;
}

bool handleCommand(const uint8_t* data, size_t len) {
    memset(&g_cmdPayload, 0, sizeof(g_cmdPayload));

    pb_istream_t stream = pb_istream_from_buffer(data, len);
    if (!pb_decode(&stream, org_eclipse_tahu_protobuf_Payload_fields, &g_cmdPayload)) {
        LOG_W("Sparkplug: NCMD decode fallo: %s", PB_GET_ERROR(&stream));
        return false;
    }

    bool found = false;

    for (pb_size_t i = 0; i < g_cmdPayload.metrics_count; ++i) {
        const Metric& m = g_cmdPayload.metrics[i];

        const bool byName =
            m.has_name && strcmp(m.name, SPB_METRIC_REBIRTH) == 0;
        const bool byAlias =
            m.has_alias && m.alias == SPB_ALIAS_NODE_REBIRTH;

        if (!byName && !byAlias) continue;

        bool requested = false;
        if (m.which_value ==
            org_eclipse_tahu_protobuf_Payload_Metric_boolean_value_tag) {
            requested = m.value.boolean_value;
        }

        if (requested) {
            const uint32_t now = millis();
            if (g_lastRebirthMs == 0 ||
                (uint32_t)(now - g_lastRebirthMs) >= SPB_REBIRTH_DEBOUNCE_MS) {
                g_lastRebirthMs = now;
                g_rebirthRequested.store(true);
                LOG_I("Sparkplug: Rebirth solicitado por NCMD");
                found = true;
            }
        }
    }

    return found;
}

bool rebirthPending() {
    return g_rebirthRequested.load();
}

void clearRebirth() {
    g_rebirthRequested.store(false);
}

} // namespace spb
