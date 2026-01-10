#pragma once

#include <Arduino.h>
#include "../adi_driver/adi_eth_common.h"

// Centralized error logging for ADIN2111 driver
// Provides consistent error reporting across the codebase
namespace ErrorLog {

// Get human-readable error name
inline const char* getErrorName(adi_eth_Result_e result)
{
    switch(result) {
        case ADI_ETH_SUCCESS: return "SUCCESS";
        case ADI_ETH_MDIO_TIMEOUT: return "MDIO_TIMEOUT";
        case ADI_ETH_COMM_ERROR: return "COMM_ERROR";
        case ADI_ETH_COMM_TIMEOUT: return "COMM_TIMEOUT";
        case ADI_ETH_HW_ERROR: return "HW_ERROR";
        case ADI_ETH_INVALID_PARAM: return "INVALID_PARAM";
        case ADI_ETH_DEVICE_UNINITIALIZED: return "DEVICE_UNINITIALIZED";
        case ADI_ETH_SPI_ERROR: return "SPI_ERROR";
        case ADI_ETH_QUEUE_FULL: return "QUEUE_FULL";
        case ADI_ETH_QUEUE_EMPTY: return "QUEUE_EMPTY";
        default: return "UNKNOWN_ERROR";
    }
}

// Log an error from an ADI driver function
inline void logError(const char* function, adi_eth_Result_e result)
{
    if (result != ADI_ETH_SUCCESS)
    {
        Serial.print("[ADIN2111 ERROR] ");
        Serial.print(function);
        Serial.print(" failed: ");
        Serial.print(getErrorName(result));
        Serial.print(" (");
        Serial.print((int)result);
        Serial.println(")");
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

// Helper for error checking with automatic logging
// Returns true if there was an error
inline bool checkAndLog(adi_eth_Result_e result, const char* operation, volatile uint32_t* counterArray = nullptr)
{
    if (result != ADI_ETH_SUCCESS) {
        logError(operation, result);
        if (counterArray) {
            counterArray[(int)result]++;
        }
        return true;
    }
    return false;
}

} // namespace ErrorLog
