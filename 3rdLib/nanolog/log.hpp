#include "NanoLogCpp17.h"
#include <string>
using namespace NanoLog::LogLevels;

void initLog(char *file)
{
    NanoLog::setLogFile(file);
    NanoLog::preallocate();
    NanoLog::setLogLevel(NOTICE);
}
void LOG_INT(uint64_t time, int data, char *label)
{
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$INT %d %d %d '%s'", time_H, time_L, data, label);
}

void LOG_FLOAT(uint64_t time, float data, char *label)
{
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$FLOAT %d %d %f '%s'", time_H, time_L, data, label);    
}

void LOG_STRING(uint64_t time, char* data, char *label)
{
    uint32_t time_H = static_cast<uint32_t>(time / 1000000000);
    uint32_t time_L = static_cast<uint32_t>(time % 1000000000);
    NANO_LOG(NOTICE, "$STRING %d %d %s '%s'", time_H, time_L, data, label);    
}

void sync()
{
    NanoLog::sync();
}