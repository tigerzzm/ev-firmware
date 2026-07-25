#pragma once

#include <cstdio>
#include <cstdarg>

// Global printf wrapper that logs to both serial and RAM
// Usage: Include this AFTER ram_logger.h in files where you want auto-logging
#ifdef __cplusplus
extern "C" {
#endif

int ram_printf(const char* format, ...);

#ifdef __cplusplus
}
#endif

// Redefine printf to use our wrapper
#define printf ram_printf

