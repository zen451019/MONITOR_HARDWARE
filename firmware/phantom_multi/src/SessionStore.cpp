#include "SessionStore.h"

#include <RadioLib.h>
#include <cstdlib>
#include <cstring>

#include "Log.h"
#include "MultiDeviceConfig.h"
#include "PhantomMultiConfig.h"

static uint8_t* s_sessionBuf = nullptr;
static uint8_t* s_noncesBuf = nullptr;
static uint32_t* s_lastFcnt = nullptr;
static size_t s_count = 0;

size_t sessionStoreCount() {
    return (size_t)kAbpDeviceCount;
}

uint32_t sessionStoreFcntUp(size_t index) {
    if (index >= s_count || s_lastFcnt == nullptr) return 0;
    return s_lastFcnt[index];
}

uint32_t sessionStoreDevAddr(size_t index) {
    if (index >= s_count) return 0;
    return kAbpDevices[index].dev_addr;
}

static void applyRuntimeConfig(LoRaWANNode& node) {
    node.setADR(false);
    node.setDutyCycle(false);
    node.setDwellTime(true);

    const int16_t drState = node.setDatarate(LORAWAN_DATARATE);
    if (drState != RADIOLIB_ERR_NONE) {
        LOG_W("No se pudo fijar DR%u (%d)", (unsigned)LORAWAN_DATARATE, drState);
    }

    const int16_t pwrState = node.setTxPower(LORAWAN_TX_POWER_DBM);
    if (pwrState != RADIOLIB_ERR_NONE) {
        LOG_W("No se pudo fijar TX power %d dBm (%d)", (int)LORAWAN_TX_POWER_DBM, pwrState);
    }
}

bool sessionStoreInit(LoRaWANNode& node) {
    s_count = (size_t)kAbpDeviceCount;
    if (s_count == 0) {
        LOG_E("No hay dispositivos en MultiDeviceConfig.h");
        return false;
    }

    s_sessionBuf = (uint8_t*)malloc(s_count * RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    s_noncesBuf = (uint8_t*)malloc(s_count * RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    s_lastFcnt = (uint32_t*)calloc(s_count, sizeof(uint32_t));

    if (s_sessionBuf == nullptr || s_noncesBuf == nullptr || s_lastFcnt == nullptr) {
        LOG_E("RAM insuficiente para %u sesiones", (unsigned)s_count);
        return false;
    }

    for (size_t i = 0; i < s_count; ++i) {
        const AbpDevice& d = kAbpDevices[i];

        int16_t state = node.beginABP(d.dev_addr, nullptr, nullptr, d.nwk_s_key, d.app_s_key);
        if (state != RADIOLIB_ERR_NONE) {
            LOG_E("beginABP fallo en %u (%d)", (unsigned)i, state);
            return false;
        }

        state = node.activateABP();
        if (state != RADIOLIB_LORAWAN_NEW_SESSION &&
            state != RADIOLIB_LORAWAN_SESSION_RESTORED &&
            state != RADIOLIB_ERR_NONE) {
            LOG_E("activateABP fallo en %u (%d)", (unsigned)i, state);
            return false;
        }

        applyRuntimeConfig(node);

        memcpy(s_sessionBuf + (i * RADIOLIB_LORAWAN_SESSION_BUF_SIZE),
               node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
        memcpy(s_noncesBuf + (i * RADIOLIB_LORAWAN_NONCES_BUF_SIZE),
               node.getBufferNonces(), RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

        s_lastFcnt[i] = 0;
    }

    LOG_I("SessionStore listo: %u identidades", (unsigned)s_count);
    return true;
}

bool sessionStoreSelect(LoRaWANNode& node, size_t index) {
    if (index >= s_count) return false;

    const AbpDevice& d = kAbpDevices[index];

    int16_t state = node.beginABP(d.dev_addr, nullptr, nullptr, d.nwk_s_key, d.app_s_key);
    if (state != RADIOLIB_ERR_NONE) {
        LOG_E("beginABP fallo al seleccionar %u (%d)", (unsigned)index, state);
        return false;
    }

    state = node.setBufferNonces(s_noncesBuf + (index * RADIOLIB_LORAWAN_NONCES_BUF_SIZE));
    if (state != RADIOLIB_ERR_NONE) {
        LOG_E("setBufferNonces fallo en %u (%d)", (unsigned)index, state);
        return false;
    }

    state = node.setBufferSession(s_sessionBuf + (index * RADIOLIB_LORAWAN_SESSION_BUF_SIZE));
    if (state != RADIOLIB_ERR_NONE) {
        LOG_E("setBufferSession fallo en %u (%d)", (unsigned)index, state);
        return false;
    }

    state = node.activateABP();
    if (state != RADIOLIB_LORAWAN_SESSION_RESTORED && state != RADIOLIB_ERR_NONE) {
        LOG_E("activateABP fallo al restaurar %u (%d)", (unsigned)index, state);
        return false;
    }

    applyRuntimeConfig(node);
    return true;
}

void sessionStorePersist(LoRaWANNode& node, size_t index) {
    if (index >= s_count) return;

    memcpy(s_sessionBuf + (index * RADIOLIB_LORAWAN_SESSION_BUF_SIZE),
           node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

    s_lastFcnt[index] = node.getFCntUp();
}
