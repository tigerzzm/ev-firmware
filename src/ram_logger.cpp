#include "ram_logger.h"
#include <cstdio>
#include <cstdarg>

// Static member definitions
char RAMLogger::buffer[BUFFER_SIZE];
int RAMLogger::writeIndex = 0;
bool RAMLogger::overflow = false;

void RAMLogger::log(const char* format, ...) {
    if (overflow) return;  // Buffer full, ignore new logs
    
    va_list args;
    va_start(args, format);
    
    // Calculate how much space we need
    char temp[256];
    int len = vsnprintf(temp, sizeof(temp), format, args);
    va_end(args);
    
    // Check if it fits
    if (writeIndex + len + 1 > BUFFER_SIZE) {
        overflow = true;
        // Write overflow message
        const char* msg = "[BUFFER OVERFLOW - LOGGING STOPPED]\n";
        int msgLen = strlen(msg);
        if (writeIndex + msgLen < BUFFER_SIZE) {
            strcpy(&buffer[writeIndex], msg);
            writeIndex += msgLen;
        }
        return;
    }
    
    // Copy to buffer
    strcpy(&buffer[writeIndex], temp);
    writeIndex += len;
}

void RAMLogger::dump() {
    // Use actual C printf to avoid any macro issues
    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf("=========================== RAM LOG DUMP START =================================\n");
    std::printf("================================================================================\n");
    std::printf("Buffer usage: %d / %d bytes (%.1f%%)\n", 
           writeIndex, BUFFER_SIZE, (writeIndex * 100.0f) / BUFFER_SIZE);
    if (overflow) {
        std::printf("WARNING: Buffer overflow occurred - some logs were lost!\n");
    }
    std::printf("--------------------------------------------------------------------------------\n");
    std::printf("%s", buffer);
    std::printf("--------------------------------------------------------------------------------\n");
    std::printf("============================ RAM LOG DUMP END ==================================\n");
    std::printf("================================================================================\n");
    std::printf("\n");
}

void RAMLogger::clear() {
    writeIndex = 0;
    overflow = false;
    buffer[0] = '\0';
}

