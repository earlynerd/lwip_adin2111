/*
 * ADIN2111 Basic Initialization Example
 *
 * This example demonstrates the minimal setup required to initialize
 * the ADIN2111 Single Pair Ethernet device and check its status.
 *
 * Hardware Connections:
 * - SPI MOSI, MISO, SCK to ADIN2111
 * - CS_PIN to ADIN2111 chip select
 * - INT_PIN to ADIN2111 interrupt output
 * - RESET_PIN to ADIN2111 reset input
 * - CFG0_PIN, CFG1_PIN to ADIN2111 configuration pins
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
    while (!Serial && millis() < 3000);  // Wait for Serial, timeout after 3s

    Serial.println("\n========================================");
    Serial.println("ADIN2111 Basic Initialization Example");
    Serial.println("========================================\n");

    // Step 1: Check memory pools allocated successfully
    if (eth.poolInitFailed()) {
        Serial.println("ERROR: Memory pool allocation failed!");
        Serial.println("The device cannot initialize without memory.");
        while (1) delay(1000);  // Halt
    }
    Serial.println("✓ Memory pools allocated successfully");

    // Step 2: Configure hardware pins
    eth.setPins(RESET_PIN, CFG0_PIN, CFG1_PIN);
    Serial.println("✓ Hardware pins configured");

    // Step 3: Initialize device
    Serial.println("\nInitializing ADIN2111...");
    if (!eth.begin(myMAC)) {
        Serial.println("ERROR: Device initialization failed!");
        Serial.println("Check your wiring and SPI connections.");
        while (1) delay(1000);  // Halt
    }
    Serial.println("✓ Device initialized successfully");

    // Step 4: Print device information
    Serial.println("\n--- Device Information ---");
    Serial.print("MAC Address: ");
    for (int i = 0; i < 6; i++) {
        if (myMAC[i] < 0x10) Serial.print("0");
        Serial.print(myMAC[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    Serial.print("Link Status: ");
    Serial.println(eth.isLinked() ? "UP" : "DOWN");

    Serial.println("\nSetup complete! Link status will be monitored in loop().");
}

void loop() {
    static uint32_t lastCheck = 0;
    static bool lastLinkState = false;

    // Check link status every second
    if (millis() - lastCheck > 1000) {
        lastCheck = millis();

        bool currentLink = eth.isLinked();
        if (currentLink != lastLinkState) {
            lastLinkState = currentLink;
            Serial.print("[");
            Serial.print(millis() / 1000);
            Serial.print("s] Link Status Changed: ");
            Serial.println(currentLink ? "UP" : "DOWN");
        }
    }

    // Simple LED blink to show we're alive
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink > 500) {
        lastBlink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
