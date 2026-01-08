# Error Handling Guide

This document describes the error handling patterns and best practices used in the ADIN2111 Arduino driver.

## Error Handling Philosophy

The driver follows these principles:

1. **Check All Critical Operations**: All operations that can fail should have their return values checked
2. **Log Errors Consistently**: Use the ErrorLog namespace for standardized error reporting
3. **Track Error Occurrences**: Error counters track all failures for diagnostics
4. **Fail Gracefully**: Non-critical errors should log but continue; critical errors should return error codes
5. **User Visibility**: Users can access diagnostics via `printDiagnostics()` and `hasErrors()`

## Error Types

### ADI Driver Result Codes

All ADI driver functions return `adi_eth_Result_e`:
- `ADI_ETH_SUCCESS` (0): Operation succeeded
- Non-zero values: Various error conditions (see ErrorLog.h for names)

### Critical vs Non-Critical Errors

**Critical Errors** - Stop initialization:
- Memory allocation failure
- Device initialization failure
- Communication errors during setup
- MAC filter configuration failure

**Non-Critical Errors** - Log but continue:
- LED configuration errors
- Individual packet transmission failures
- Single buffer submission failures

## Using ErrorLog Functions

### Basic Error Logging

```cpp
#include "../utility/ErrorLog.h"

adi_eth_Result_e result = someOperation();
if (result != ADI_ETH_SUCCESS) {
    ErrorLog::logError("operation name", result);
}
```

### Checking with Counter Tracking

```cpp
adi_eth_Result_e result = someOperation();
if (ErrorLog::checkAndLog(result, "operation name", errorCounterArray)) {
    // Error occurred, handle it
    return result;  // For critical operations
}
```

### Error Types

```cpp
ErrorLog::logError(function, result);     // Error with code
ErrorLog::logWarning(message);            // Warning message
ErrorLog::logInitError(step, result);     // Initialization error
ErrorLog::logDebug(message);              // Debug (only if ADIN2111_DEBUG defined)
```

## Error Counter Arrays

Three counter arrays track errors by type:
- `initResultCounters[36]`: Errors during initialization
- `rxResultCounters[36]`: Errors during RX operations
- `txResultCounters[36]`: Errors during TX operations

Access via diagnostics:
```cpp
SinglePairEthernet::DiagnosticInfo info;
eth.getDiagnostics(info);

// Check for errors
if (info.initErrorCount > 0) {
    // Handle initialization errors
}
```

## Best Practices

### 1. Always Check Critical Operations

```cpp
// GOOD
adi_eth_Result_e result = submitRxBuffer(&rxBufDesc[i]);
if (result != ADI_ETH_SUCCESS) {
    ErrorLog::logInitError("submitting RX buffer", result);
    initResultCounters[(int)result]++;
    return result;  // Stop initialization
}

// BAD - No error checking
submitRxBuffer(&rxBufDesc[i]);
```

### 2. Use Descriptive Operation Names

```cpp
// GOOD
ErrorLog::checkAndLog(result, "LED polarity P1", initResultCounters);

// BAD - Generic name
ErrorLog::checkAndLog(result, "operation", initResultCounters);
```

### 3. Check Memory Allocation

```cpp
if (eth.poolInitFailed()) {
    Serial.println("ERROR: Memory allocation failed!");
    // Don't proceed with initialization
    return false;
}
```

### 4. Provide User-Facing Diagnostics

```cpp
// Periodically check for errors
if (eth.hasErrors()) {
    Serial.println("Errors detected!");
    eth.printDiagnostics();
}
```

### 5. Handle Buffer Exhaustion

```cpp
uint8_t* buf = txPool.allocate();
if (buf == nullptr) {
    ErrorLog::logWarning("TX buffer pool exhausted");
    return 0;  // Indicate failure to caller
}
```

## Error Recovery

### Memory Pool Exhaustion

**Symptom**: `getRxDroppedCount()` increasing

**Solutions**:
1. Increase `RX_POOL_COUNT` in PacketPool.h
2. Process packets faster in application code
3. Consider using static allocation for deterministic behavior

### Communication Errors

**Symptom**: `ADI_ETH_COMM_ERROR`, `ADI_ETH_SPI_ERROR`

**Solutions**:
1. Check SPI wiring and signal integrity
2. Reduce SPI clock speed if needed
3. Verify chip select timing
4. Check power supply stability

### Queue Full Errors

**Symptom**: `ADI_ETH_QUEUE_FULL` in TX counters

**Solutions**:
1. Space out packet transmissions
2. Check link status before sending
3. Increase `TX_POOL_COUNT` if possible

### Initialization Failures

**Symptom**: `begin()` returns false

**Solutions**:
1. Run `printDiagnostics()` to see specific errors
2. Check `initResultCounters` for error types
3. Verify hardware connections
4. Increase retry count/delay in Config

## Code Examples

### Initialization with Error Handling

```cpp
// Check memory first
if (eth.poolInitFailed()) {
    Serial.println("FATAL: Memory allocation failed!");
    while (1) delay(1000);
}

// Initialize device
uint8_t retries = 0;
if (!eth.begin(&retries, myMAC, CS, INT, RST, CFG0, CFG1)) {
    Serial.println("ERROR: Initialization failed");
    Serial.print("Retries: ");
    Serial.println(retries);

    // Print detailed diagnostics
    eth.printDiagnostics();

    // Don't proceed
    while (1) delay(1000);
}

Serial.println("Device initialized successfully");
```

### Periodic Error Monitoring

```cpp
void loop() {
    static uint32_t lastCheck = 0;

    if (millis() - lastCheck > 10000) {  // Every 10 seconds
        lastCheck = millis();

        if (eth.hasErrors()) {
            Serial.println("Errors detected:");
            eth.printDiagnostics();

            // Optional: Take corrective action
            // - Reset device
            // - Notify user
            // - Log to SD card
        }
    }
}
```

### Structured Error Checking

```cpp
SinglePairEthernet::DiagnosticInfo diag;
eth.getDiagnostics(diag);

// Check specific error types
if (diag.initErrorCount > 0) {
    Serial.println("Initialization errors present!");
    for (int i = 0; i < diag.initErrorCount; i++) {
        Serial.print("  - ");
        Serial.print(ErrorLog::getErrorName(diag.initErrors[i].code));
        Serial.print(": ");
        Serial.println(diag.initErrors[i].count);
    }
}

// Check buffer health
if (diag.rxDropped > 0) {
    Serial.print("WARNING: ");
    Serial.print(diag.rxDropped);
    Serial.println(" RX packets dropped!");
    Serial.println("Consider increasing RX_POOL_COUNT");
}
```

## Debugging Tips

1. **Enable Debug Logging**: Define `ADIN2111_DEBUG` for verbose output
2. **Use printDiagnostics()**: Comprehensive status report
3. **Monitor Counter Arrays**: Track error patterns over time
4. **Check Pool Levels**: Use `getLevel()` on RX/TX pools
5. **Verify Return Values**: Ensure all critical operations are checked

## Summary

- Always check return values for operations that can fail
- Use ErrorLog functions for consistent error reporting
- Track errors in counter arrays for diagnostics
- Provide user-facing diagnostic functions
- Handle errors gracefully - log and continue for non-critical, abort for critical
- Use structured diagnostic data for programmatic error handling
