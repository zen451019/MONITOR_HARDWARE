#ifndef LOG_H
#define LOG_H

#include <Arduino.h>

// =================================================================================================
// Lightweight leveled logger.
// =================================================================================================
// Levels: 0 = OFF, 1 = ERROR, 2 = WARN, 3 = INFO, 4 = DEBUG.
// Override at compile time with -DLOG_LEVEL=N in build_flags.

#ifndef LOG_LEVEL
#define LOG_LEVEL 3
#endif

// Serialized logger. logPrintf() formats one line and writes it with a single
// Serial.write() under a mutex, so lines from different tasks cannot interleave.
void logInit();
void logPrintf(const char* level, const char* fmt, ...);

// Internal: print only if the message level is <= LOG_LEVEL.
#define LOG_PRINT(level_tag, fmt, ...) \
    logPrintf(level_tag, fmt, ##__VA_ARGS__)

#if LOG_LEVEL >= 1
#define LOG_E(fmt, ...) LOG_PRINT("E", fmt, ##__VA_ARGS__)
#else
#define LOG_E(fmt, ...) do { } while (0)
#endif

#if LOG_LEVEL >= 2
#define LOG_W(fmt, ...) LOG_PRINT("W", fmt, ##__VA_ARGS__)
#else
#define LOG_W(fmt, ...) do { } while (0)
#endif

#if LOG_LEVEL >= 3
#define LOG_I(fmt, ...) LOG_PRINT("I", fmt, ##__VA_ARGS__)
#else
#define LOG_I(fmt, ...) do { } while (0)
#endif

#if LOG_LEVEL >= 4
#define LOG_D(fmt, ...) LOG_PRINT("D", fmt, ##__VA_ARGS__)
#else
#define LOG_D(fmt, ...) do { } while (0)
#endif

#endif // LOG_H
