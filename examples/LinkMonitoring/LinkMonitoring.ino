/*
 * ADIN2111 Link Monitoring Example
 *
 * This example demonstrates how to use callbacks to monitor
 * link status changes in real-time.
 *
 * The link callback is executed when the Ethernet link state changes,
 * allowing immediate response to connection/disconnection events.
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

// Link status tracking
volatile bool linkIsUp = false;
volatile uint32_t linkUpCount = 0;
volatile uint32_t linkDownCount = 0;
volatile uint32_t lastLinkChangeTime = 0;

// Link status change callback
void onLinkChange(bool connected) {
    linkIsUp = connected;
    lastLinkChangeTime = millis();

    if (connected) {
        linkUpCount++;
        Serial.println("\n*** LINK UP ***");
    } else {
        linkDownCount++;
        Serial.println("\n*** LINK DOWN ***");
    }

    // You could add additional actions here:
    // - Start/stop network services
    // - Update LED indicators
    // - Log to SD card
    // - Send notifications
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\n========================================");
    Serial.println("ADIN2111 Link Monitoring Example");
    Serial.println("========================================\n");

    // Check memory
    if (eth.poolInitFailed()) {
        Serial.println("ERROR: Memory allocation failed!");
        while (1) delay(1000);
    }

    // Register link callback BEFORE initialization
    eth.adin2111.setLinkCallback(onLinkChange);
    Serial.println("✓ Link callback registered");

    // Configure hardware
    eth.setPins(RESET_PIN, CFG0_PIN, CFG1_PIN);

    // Initialize device
    Serial.println("Initializing device...");
    if (!eth.begin(myMAC)) {
        Serial.println("ERROR: Initialization failed!");
        while (1) delay(1000);
    }
    Serial.println("✓ Device initialized");

    // Check initial link status
    linkIsUp = eth.isLinked();
    Serial.print("\nInitial link status: ");
    Serial.println(linkIsUp ? "UP" : "DOWN");

    Serial.println("\nMonitoring link status...");
    Serial.println("Try connecting/disconnecting the Ethernet cable.\n");
}

void loop() {
    static uint32_t lastStatusPrint = 0;

    // Print status every 5 seconds
    if (millis() - lastStatusPrint > 5000) {
        lastStatusPrint = millis();

        Serial.println("--- Status Report ---");
        Serial.print("Current Link: ");
        Serial.println(linkIsUp ? "UP" : "DOWN");

        Serial.print("Link Up Events: ");
        Serial.println(linkUpCount);

        Serial.print("Link Down Events: ");
        Serial.println(linkDownCount);

        if (lastLinkChangeTime > 0) {
            uint32_t timeSinceChange = (millis() - lastLinkChangeTime) / 1000;
            Serial.print("Last Change: ");
            Serial.print(timeSinceChange);
            Serial.println(" seconds ago");
        }

        // Calculate uptime percentage
        if (linkUpCount + linkDownCount > 0) {
            float uptime = (float)linkUpCount / (linkUpCount + linkDownCount) * 100;
            Serial.print("Uptime: ");
            Serial.print(uptime, 1);
            Serial.println("%");
        }

        Serial.println();
    }

    // Visual feedback with LED
    // Blink fast when link is down, slow when up
    static uint32_t lastBlink = 0;
    uint32_t blinkInterval = linkIsUp ? 1000 : 200;

    if (millis() - lastBlink > blinkInterval) {
        lastBlink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    delay(10);
}
