/*
 * ADIN2111 Diagnostics Demo
 *
 * This example demonstrates the diagnostic and monitoring capabilities
 * of the ADIN2111 driver, including:
 * - Error tracking
 * - Buffer pool monitoring
 * - Link status
 * - Performance counters
 *
 * The new printDiagnostics() method provides formatted output of all
 * diagnostic information.
 */

#include <SPI.h>
#include <adin2111_lwip.h>

// Pin definitions - adjust for your hardware
#define CS_PIN    10
#define INT_PIN   2
#define RESET_PIN 7
#define CFG0_PIN  9
#define CFG1_PIN  8

// MAC address for this device
uint8_t myMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};

// Create ADIN2111 lwIP device
ADIN2111_lwIP eth(CS_PIN, SPI, INT_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\n========================================");
    Serial.println("ADIN2111 Diagnostics Demo");
    Serial.println("========================================\n");

    // Check memory allocation
    if (eth.poolInitFailed()) {
        Serial.println("ERROR: Memory pool allocation failed!");
        while (1) delay(1000);
    }

    // Configure and initialize
    eth.setPins(RESET_PIN, CFG0_PIN, CFG1_PIN);

    Serial.println("Initializing device...");
    if (!eth.begin(myMAC)) {
        Serial.println("ERROR: Initialization failed!");
        Serial.println("\nAttempting to print diagnostics anyway...");
        eth.printDiagnostics();
        while (1) delay(1000);
    }

    Serial.println("✓ Initialization successful\n");
    Serial.println("Commands:");
    Serial.println("  'd' - Print diagnostics");
    Serial.println("  'i' - Print device info");
    Serial.println("  's' - Check for errors");
    Serial.println("  'r' - Reset error counters (not implemented)");
    Serial.println();
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'd':
            case 'D':
                // Print full diagnostic report
                Serial.println();
                eth.printDiagnostics();
                Serial.println();
                break;

            case 'i':
            case 'I':
                // Print basic info
                printBasicInfo();
                break;

            case 's':
            case 'S':
                // Quick error check
                Serial.println();
                if (eth.hasErrors()) {
                    Serial.println("⚠ ERRORS DETECTED - Run 'd' for details");
                } else {
                    Serial.println("✓ No errors detected");
                }
                Serial.println();
                break;

            case '\n':
            case '\r':
                // Ignore newlines
                break;

            default:
                Serial.println("Unknown command. Available: d, i, s");
                break;
        }
    }

    // Periodic diagnostics (every 30 seconds)
    static uint32_t lastDiagnostics = 0;
    if (millis() - lastDiagnostics > 30000) {
        lastDiagnostics = millis();

        Serial.println("\n--- Periodic Status Check ---");
        if (eth.hasErrors()) {
            Serial.println("⚠ Errors detected!");
            eth.printDiagnostics();
        } else {
            Serial.println("✓ System healthy");
            printBasicInfo();
        }
    }

    delay(10);
}

void printBasicInfo() {
    Serial.println("\n--- Basic Information ---");

    // Link status
    Serial.print("Link: ");
    Serial.println(eth.isLinked() ? "UP" : "DOWN");

    // Using the structured diagnostic data
    SinglePairEthernet::DiagnosticInfo info;
    eth.adin2111.getDiagnostics(info);

    Serial.print("RX Queue: ");
    Serial.print(info.rxQueueLevel);
    Serial.print("/");
    Serial.println(RX_POOL_COUNT);

    Serial.print("TX Free: ");
    Serial.print(info.txQueueLevel);
    Serial.print("/");
    Serial.println(TX_POOL_COUNT);

    if (info.rxDropped > 0) {
        Serial.print("⚠ RX Dropped: ");
        Serial.println(info.rxDropped);
    }

    Serial.println();
}
