#pragma once

#include <Arduino.h>
#include "../adi_driver/adi_eth_common.h"

// Centralized error logging for ADIN2111 driver
// Provides consistent error reporting across the codebase
namespace ErrorLog {

// Log an error from an ADI driver function
inline void logError(const char* function, adi_eth_Result_e result)
{
    if (result != ADI_ETH_SUCCESS)
    {
        Serial.print("[ADIN2111 ERROR] ");
        Serial.print(function);
        Serial.print(" failed with code: ");
        Serial.println((int)result);
    }
}

// Log a warning message
inline void logWarning(const char* message)
{
    Serial.print("[ADIN2111 WARNING] ");
    Serial.println(message);
}

// Log an initialization error
inline void logInitError(const char* step, adi_eth_Result_e result)
{
    Serial.print("[ADIN2111 INIT ERROR] ");
    Serial.print(step);
    Serial.print(" failed with code: ");
    Serial.println((int)result);
}

// Log a debug message (can be disabled for production)
inline void logDebug(const char* message)
{
#ifdef ADIN2111_DEBUG
    Serial.print("[ADIN2111 DEBUG] ");
    Serial.println(message);
#else
    (void)message;  // Suppress unused parameter warning
#endif
}

} // namespace ErrorLog
