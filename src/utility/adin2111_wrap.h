#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <EthernetCompat.h>
#include "../driver/SinglePairEthernet.h"


class ADIN2111_wrap {
public:
    SinglePairEthernet adin2111;
    ADIN2111_wrap(int8_t cs, SPIClass& spi, int8_t intr);
    void setPins(int8_t reset, int8_t cfg0, int8_t cfg1);
    struct netif *_netif = nullptr;
    // Lifecycle methods matching LwipIntfDev template
    bool begin(const uint8_t* macAddress, struct netif* netif);
    void end();

    // Capabilities
    constexpr bool needsSPI() { return true; }
    constexpr bool isLinkDetectable() { return true; }
    void checkLinkStatus();
    // Interrupt handling
    // Use LOW level triggering instead of FALLING edge - this ensures we don't
    // miss interrupts if INT is already asserted when lwIP attaches the handler.
    // The lwIP template masks GPIO interrupts during processing to prevent storms.
    bool interruptIsPossible() { return _intr != -1; }
    PinStatus interruptMode() { return LOW; } 

    // Frame Handling
    // 1. Check for incoming frame and return size (0 if none)
    uint16_t readFrameSize();

    // 2. Read the data for the frame found by readFrameSize
    //    Returns the number of bytes read
    uint16_t readFrameData(uint8_t *buffer, uint16_t len);

    // 3. Discard the frame found by readFrameSize
    void discardFrame(uint16_t len);

    // Send a packet
    uint16_t sendFrame(const uint8_t* data, uint16_t len);

    // Link status check
    bool linkStatus();
    bool isLinked();
    void printStatus();

private:
    bool _lastLinkState = false;

    static void rxcallback(adi_eth_BufDesc_t *);

    int8_t _cs;
    int8_t _intr;
    int8_t _reset;
    int8_t _cfg0;
    int8_t _cfg1;
    SPIClass& _spi;
    SPISettings _settings = {20000000, MSBFIRST, SPI_MODE0}; 
    uint8_t _mac[6];
    bool printResultCounters(uint32_t* counters);
    void printPhyState(adi_phy_State_e);
};