#include "ram_printf.h"
#include "ram_logger.h"
#include <cstdio>
#include <cstdarg>

// Undefine the macro temporarily for implementation
#undef printf

extern "C" {
    int ram_printf(const char* format, ...) {
        va_list args1, args2;
        va_start(args1, format);
        va_copy(args2, args1);
        
        // Print to serial using real printf
        int result = vprintf(format, args1);
        va_end(args1);
        
        // Also log to RAM buffer
        char temp[512];
        vsnprintf(temp, sizeof(temp), format, args2);
        va_end(args2);
        
        RAMLogger::log("%s", temp);
        
        return result;
    }
}

// Redefine the macro after implementation
#define printf ram_printf

