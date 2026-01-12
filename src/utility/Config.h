#pragma once

#include <stdint.h>

// Configuration structure for ADIN2111 device
// Allows customization of hardware and behavior settings
struct ADIN2111Config {
    // Buffer configuration
    uint8_t numBuffers = 4;          // Number of hardware buffer descriptors
    uint16_t rxPacketSize = 1524;    // RX packet buffer size (bytes)
    uint16_t txPacketSize = 1524;    // TX packet buffer size (bytes)

    // LED configuration
    uint16_t ledPolarity = 0x01;     // LED polarity (0x01 = active high)
    uint16_t ledControl = 0x8E84;    // LED control register value
    uint16_t ledBlinkTime = 0x0303;  // LED blink timing

    // Cut-through mode configuration
    bool cutThroughTx = false;       // TX cut-through mode
    bool cutThroughRx = false;       // RX cut-through mode
    bool cutThroughP2P = true;      // Port-to-port cut-through mode

    // Initialization configuration
    uint8_t initRetries = 5;         // Number of initialization attempts
    uint16_t initRetryDelay = 100;   // Delay between retries (ms)

    // Frame configuration
    uint16_t minFramePadding = 60;   // Minimum Ethernet frame size
    bool fcsCheckEnabled = false;    // Frame check sequence validation

    // Default constructor with sensible defaults
    ADIN2111Config() = default;

    // Create a configuration with custom settings
    static ADIN2111Config createCustom(
        uint8_t numBufs = 4,
        uint16_t rxSize = 1524,
        uint16_t txSize = 1524)
    {
        ADIN2111Config config;
        config.numBuffers = numBufs;
        config.rxPacketSize = rxSize;
        config.txPacketSize = txSize;
        return config;
    }
};
