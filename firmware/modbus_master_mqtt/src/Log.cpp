#include "Log.h"

#include <Arduino.h>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {
SemaphoreHandle_t s_logMutex = nullptr;
}

void logInit() {
    if (s_logMutex == nullptr) {
        s_logMutex = xSemaphoreCreateMutex();
    }
}

void logPrintf(const char* level, const char* fmt, ...) {
    char line[256];

    int n = snprintf(line, sizeof(line), "[%s] ", level);
    if (n < 0) return;

    size_t used = ((size_t)n < sizeof(line)) ? (size_t)n : sizeof(line) - 1;

    va_list args;
    va_start(args, fmt);
    int m = vsnprintf(line + used, sizeof(line) - used, fmt, args);
    va_end(args);

    if (m < 0) return;

    const size_t remaining = sizeof(line) - used;
    if ((size_t)m >= remaining) {
        used = sizeof(line) - 2;
    } else {
        used += (size_t)m;
    }

    line[used++] = '\n';

    if (s_logMutex != nullptr) {
        if (xSemaphoreTake(s_logMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;
        }
    }

    Serial.write(reinterpret_cast<const uint8_t*>(line), used);

    if (s_logMutex != nullptr) {
        xSemaphoreGive(s_logMutex);
    }
}
