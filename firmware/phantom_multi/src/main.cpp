#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "Log.h"
#include "MultiDeviceSimulator.h"
#include "PhantomMultiConfig.h"
#include "SessionStore.h"

static SX1276 radio = new Module(18, 26, 23, 33);
static LoRaWANNode node(&radio, &US915, LORAWAN_SUB_BAND);

static PhantomDeviceState* g_states = nullptr;

static void phantomTask(void* pvParameters) {
    (void)pvParameters;

    const size_t count = sessionStoreCount();
    if (count == 0) {
        LOG_E("Sin dispositivos configurados");
        vTaskDelete(nullptr);
        return;
    }

    g_states = (PhantomDeviceState*)calloc(count, sizeof(PhantomDeviceState));
    if (g_states == nullptr) {
        LOG_E("RAM insuficiente para %u estados", (unsigned)count);
        vTaskDelete(nullptr);
        return;
    }

    const uint32_t nowMs = millis();
    simInitAll(g_states, count, nowMs);

    const float requiredUplinksPerSec =
        ((float)count * 1000.0f) / (float)PHANTOM_DEVICE_INTERVAL_MS;
    const float classACapacity = 1.0f / 2.1f;

    LOG_I("Phantom multi: %u dispositivos, intervalo %lu ms",
          (unsigned)count, (unsigned long)PHANTOM_DEVICE_INTERVAL_MS);
    LOG_I("Carga requerida: %.2f uplinks/s (Class A disponible ~%.2f)",
          requiredUplinksPerSec, classACapacity);

    if (requiredUplinksPerSec > classACapacity) {
        LOG_W("El nodo no podra mantener ese intervalo; el intervalo efectivo sera mayor");
    }

    uint8_t downBuf[RADIOLIB_LORAWAN_MAX_PAYLOAD_SIZE];
    uint8_t payload[LORA_PAYLOAD_MAX];

    uint32_t sent = 0;
    uint32_t errors = 0;

    for (;;) {
        const uint32_t now = millis();

        size_t idx = 0;
        int32_t bestDelta = INT32_MAX;

        for (size_t i = 0; i < count; ++i) {
            const int32_t delta = (int32_t)(g_states[i].nextDueMs - now);
            if (delta < bestDelta) {
                bestDelta = delta;
                idx = i;
            }
        }

        if (bestDelta > 0) {
            uint32_t wait = (uint32_t)bestDelta;
            if (wait > 1000u) wait = 1000u;
            vTaskDelay(pdMS_TO_TICKS(wait == 0u ? 1u : wait));
            continue;
        }

        PhantomDeviceState& st = g_states[idx];

        const uint32_t dt = now - st.lastCycleMs;
        st.lastCycleMs = now;
        simAdvance(st, dt);

        const size_t len = simBuildPayload(st, payload, sizeof(payload));
        st.msgId++;

        if (len == 0) {
            LOG_E("Payload invalido para dispositivo %u", (unsigned)idx);
            st.nextDueMs = now + st.intervalMs;
            continue;
        }

        if (!sessionStoreSelect(node, idx)) {
            ++errors;
            st.nextDueMs = now + st.intervalMs;
            continue;
        }

        size_t downLen = 0;
        LoRaWANEvent_t evUp{};
        LoRaWANEvent_t evDown{};

        const int16_t state = node.sendReceive(payload, len, FPORT_DATA,
                                               downBuf, &downLen, false,
                                               &evUp, &evDown);

        if (state < RADIOLIB_ERR_NONE) {
            LOG_E("Envio fallo en dispositivo %u (%d)", (unsigned)idx, state);
            ++errors;
        } else {
            ++sent;
        }

        sessionStorePersist(node, idx);
        st.nextDueMs = millis() + st.intervalMs;

        if (state > RADIOLIB_ERR_NONE) {
            LOG_D("Downlink dispositivo %u: %u bytes, FPort %u",
                  (unsigned)idx, (unsigned)downLen, evDown.fPort);
        }

        if ((sent > 0) && ((sent % PHANTOM_LOG_EVERY_N) == 0)) {
            LOG_I("Enviados=%lu errores=%lu ultimo=%u addr=0x%08lX fCnt=%lu len=%u",
                  (unsigned long)sent, (unsigned long)errors, (unsigned)idx,
                  (unsigned long)sessionStoreDevAddr(idx),
                  (unsigned long)sessionStoreFcntUp(idx), (unsigned)len);
        }

        vTaskDelay(1);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("phantom_multi: arranque");

    SPI.begin();

    const int16_t radioState = radio.begin();
    if (radioState != RADIOLIB_ERR_NONE) {
        LOG_E("radio.begin() fallo (%d)", radioState);
    }

    if (!sessionStoreInit(node)) {
        LOG_E("No se pudo inicializar SessionStore");
        while (true) {
            delay(1000);
        }
    }

    xTaskCreatePinnedToCore(phantomTask, "Phantom", 8192, nullptr, 3, nullptr, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
