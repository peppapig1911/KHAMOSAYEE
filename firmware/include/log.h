#pragma once

#include <cstdio>
#include "pico/time.h"

static inline void log_time_prefix()
{
    uint64_t total_ms = time_us_64() / 1000ULL;
    unsigned long long hours = static_cast<unsigned long long>(total_ms / 3600000ULL);
    unsigned long long minutes = static_cast<unsigned long long>((total_ms / 60000ULL) % 60ULL);
    unsigned long long seconds = static_cast<unsigned long long>((total_ms / 1000ULL) % 60ULL);
    unsigned long long millis = static_cast<unsigned long long>(total_ms % 1000ULL);

    std::printf("[%02llu:%02llu:%02llu.%03llu]", hours, minutes, seconds, millis);
}

/// @brief Log an error message.
#define LOGE(module, fmt, ...)                                     \
    do                                                             \
    {                                                              \
        log_time_prefix();                                         \
        std::printf("[E][%s] " fmt "\n", (module), ##__VA_ARGS__); \
    } while (0)

/// @brief Log an informational message.
#define LOGI(module, fmt, ...)                                     \
    do                                                             \
    {                                                              \
        log_time_prefix();                                         \
        std::printf("[I][%s] " fmt "\n", (module), ##__VA_ARGS__); \
    } while (0)

/// @brief Log a debug message. Use LOGD(MODULE, "format", args...) to log a message with the module name and printf-style formatting.
#define LOGD(module, fmt, ...)                                     \
    do                                                             \
    {                                                              \
        log_time_prefix();                                         \
        std::printf("[D][%s] " fmt "\n", (module), ##__VA_ARGS__); \
    } while (0)
