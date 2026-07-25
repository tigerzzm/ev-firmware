#pragma once

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdarg>

class RAMLogger {
private:
    // Buffer to use ~20% of Pico's 264KB RAM for logging
    static constexpr int BUFFER_SIZE = 53000;  // 53KB buffer (~20% of Pico's 264KB RAM)
    static char buffer[BUFFER_SIZE];
    static int writeIndex;
    static bool overflow;

public:
    // Log a message to RAM buffer
    static void log(const char* format, ...);
    
    // Check if buffer is full
    static bool isFull() { return overflow; }
    
    // Dump all logs to serial
    static void dump();
    
    // Clear the buffer
    static void clear();
    
    // Get current buffer usage
    static int getUsedBytes() { return writeIndex; }
    static int getFreeBytes() { return BUFFER_SIZE - writeIndex; }
};

