/*
 * ADIN2111 Memory Configuration Example
 *
 * This example demonstrates the different memory allocation modes
 * available for packet buffers:
 *
 * 1. DYNAMIC (default): Uses malloc() to allocate buffers at runtime
 *    - Pros: Flexible, no compile-time memory reservation
 *    - Cons: Can fail if heap is exhausted
 *
 * 2. STATIC: Uses compile-time allocated arrays
 *    - Pros: Deterministic, no runtime allocation failures
 *    - Cons: Memory always reserved, must recompile to change size
 *
 * To enable static allocation:
 * 1. Open src/utility/PacketPool.h
 * 2. Uncomment: #define ADIN2111_STATIC_ALLOCATION
 * 3. Adjust RX_POOL_COUNT and TX_POOL_COUNT if needed
 * 4. Recompile
 *
 * This example shows how to check the allocation mode and
 * monitor memory usage.
 */

#include <SPI.h>
#include <adin2111_lwip.h>

// Pin definitions
#define CS_PIN    10
#define INT_PIN   2
#define RESET_PIN 7
#define CFG0_PIN  9
#define CFG1_PIN  8

uint8_t myMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};

ADIN2111_lwIP eth(CS_PIN, SPI, INT_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\n========================================");
    Serial.println("ADIN2111 Memory Configuration Example");
    Serial.println("========================================\n");

    // Print memory configuration
    printMemoryConfiguration();

    // Check if memory allocation was successful
    if (eth.poolInitFailed()) {
        Serial.println("\n❌ MEMORY ALLOCATION FAILED!");
        Serial.println("\nPossible causes:");
        Serial.println("1. Insufficient heap memory (if using DYNAMIC mode)");
        Serial.println("2. Try enabling STATIC allocation mode");
        Serial.println("3. Reduce RX_POOL_COUNT or TX_POOL_COUNT");
        while (1) delay(1000);
    }

    Serial.println("✓ Memory pools allocated successfully\n");

    // Initialize device
    eth.setPins(RESET_PIN, CFG0_PIN, CFG1_PIN);

    if (!eth.begin(myMAC)) {
        Serial.println("ERROR: Device initialization failed!");
        while (1) delay(1000);
    }

    Serial.println("✓ Device initialized\n");
    Serial.println("Monitoring memory usage...\n");
}

void loop() {
    static uint32_t lastReport = 0;

    // Print memory status every 5 seconds
    if (millis() - lastReport > 5000) {
        lastReport = millis();
        printMemoryStatus();
    }

    delay(10);
}

void printMemoryConfiguration() {
    Serial.println("--- Memory Configuration ---");

    // Check allocation mode
    Serial.print("Allocation Mode: ");
    Serial.println(eth.adin2111.rxPool.getAllocationMode());

    // Buffer configuration
    Serial.println("\nBuffer Pool Configuration:");
    Serial.print("  RX Buffers: ");
    Serial.print(RX_POOL_COUNT);
    Serial.print(" x ");
    Serial.print(RX_PACKET_SIZE);
    Serial.print(" bytes = ");
    Serial.print(RX_POOL_COUNT * RX_PACKET_SIZE);
    Serial.println(" bytes total");

    Serial.print("  TX Buffers: ");
    Serial.print(TX_POOL_COUNT);
    Serial.print(" x ");
    Serial.print(TX_PACKET_SIZE);
    Serial.print(" bytes = ");
    Serial.print(TX_POOL_COUNT * TX_PACKET_SIZE);
    Serial.println(" bytes total");

    size_t totalMemory = (RX_POOL_COUNT * RX_PACKET_SIZE) + (TX_POOL_COUNT * TX_PACKET_SIZE);
    Serial.print("\nTotal Buffer Memory: ");
    Serial.print(totalMemory);
    Serial.print(" bytes (");
    Serial.print(totalMemory / 1024.0, 1);
    Serial.println(" KB)");

#ifdef ADIN2111_STATIC_ALLOCATION
    Serial.println("\n✓ Using STATIC allocation");
    Serial.println("  - Memory reserved at compile time");
    Serial.println("  - No runtime allocation failures");
    Serial.println("  - Memory always reserved even if unused");
#else
    Serial.println("\n✓ Using DYNAMIC allocation");
    Serial.println("  - Memory allocated from heap at runtime");
    Serial.println("  - Can fail if heap is exhausted");
    Serial.println("  - Memory freed when object is destroyed");
#endif

    Serial.println();
}

void printMemoryStatus() {
    Serial.println("--- Memory Status ---");

    // Get diagnostic info
    SinglePairEthernet::DiagnosticInfo info;
    eth.adin2111.getDiagnostics(info);

    // RX pool status
    Serial.print("RX Pool: ");
    Serial.print(info.rxQueueLevel);
    Serial.print(" packets queued, ");
    size_t rxFree = RX_POOL_COUNT - info.rxQueueLevel;
    Serial.print(rxFree);
    Serial.print(" buffers free (");
    Serial.print((rxFree * 100) / RX_POOL_COUNT);
    Serial.println("%)");

    // TX pool status
    Serial.print("TX Pool: ");
    Serial.print(info.txQueueLevel);
    Serial.print(" buffers free (");
    Serial.print((info.txQueueLevel * 100) / TX_POOL_COUNT);
    Serial.println("%)");

    // Dropped packets
    if (info.rxDropped > 0) {
        Serial.print("⚠ RX Dropped: ");
        Serial.print(info.rxDropped);
        Serial.println(" packets");
        Serial.println("  → Consider increasing RX_POOL_COUNT");
    }

    // Memory health assessment
    float rxUtilization = (float)(RX_POOL_COUNT - rxFree) / RX_POOL_COUNT * 100;
    float txUtilization = (float)(TX_POOL_COUNT - info.txQueueLevel) / TX_POOL_COUNT * 100;

    Serial.print("\nMemory Health: ");
    if (info.rxDropped > 0) {
        Serial.println("⚠ POOR (packets being dropped)");
    } else if (rxUtilization > 80 || txUtilization > 80) {
        Serial.println("⚠ FAIR (high utilization)");
    } else {
        Serial.println("✓ GOOD");
    }

    Serial.println();
}
