#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "boardsupport.h"

// Encapsulates board-specific configuration and state
// Replaces global static variables for better encapsulation
class BoardConfig {
private:
    // Pin configuration
    uint8_t cfg0_pin = 9;
    uint8_t cfg1_pin = 8;
    uint8_t status_led_pin = LED_BUILTIN;
    uint8_t interrupt_pin = 6;
    uint8_t reset_pin = DEFAULT_ETH_RESET_Pin;
    uint8_t chip_select_pin = DEFAULT_ETH_SPI_CS_Pin;

    // Callback state
    ADI_CB spiCallback = nullptr;
    void* spiCBParam = nullptr;
    ADI_CB gpioIntCallback = nullptr;
    void* gpioIntCBParam = nullptr;

    // SPI async state (for arduino-pico DMA transfers)
    volatile bool spiBusy = false;

    // SPI configuration
    SPIClass* spiInstance = &SPI;

    // Singleton instance
    BoardConfig() = default;

public:
    // Singleton accessor
    static BoardConfig& instance() {
        static BoardConfig instance;
        return instance;
    }

    // Delete copy/move constructors
    BoardConfig(const BoardConfig&) = delete;
    BoardConfig& operator=(const BoardConfig&) = delete;

    // Pin configuration accessors
    void setCfgPins(uint8_t cfg0, uint8_t cfg1) {
        cfg0_pin = cfg0;
        cfg1_pin = cfg1;
    }

    uint8_t getCfg0Pin() const { return cfg0_pin; }
    uint8_t getCfg1Pin() const { return cfg1_pin; }
    uint8_t getStatusLedPin() const { return status_led_pin; }
    uint8_t getInterruptPin() const { return interrupt_pin; }
    uint8_t getResetPin() const { return reset_pin; }
    uint8_t getChipSelectPin() const { return chip_select_pin; }

    void setSystemPins(uint8_t status, uint8_t interrupt, uint8_t reset, uint8_t cs) {
        status_led_pin = status;
        interrupt_pin = interrupt;
        reset_pin = reset;
        chip_select_pin = cs;
    }

    void setChipSelectPin(uint8_t cs) {
        chip_select_pin = cs;
    }

    // SPI accessors
    void setSPIClass(SPIClass* spi) {
        if (spi != nullptr) {
            spiInstance = spi;
        }
    }

    SPIClass* getSPIClass() const { return spiInstance; }

    // Callback accessors
    void setSPICallback(ADI_CB callback, void* param) {
        spiCallback = callback;
        spiCBParam = param;
    }

    ADI_CB getSPICallback() const { return spiCallback; }
    void* getSPICallbackParam() const { return spiCBParam; }

    void setGPIOIntCallback(ADI_CB callback, void* param) {
        gpioIntCallback = callback;
        gpioIntCBParam = param;
    }

    ADI_CB getGPIOIntCallback() const { return gpioIntCallback; }
    void* getGPIOIntCallbackParam() const { return gpioIntCBParam; }

    // SPI busy state accessors (for async transfers)
    void setSpiBusy(bool busy) { spiBusy = busy; }
    bool isSpiBusy() const { return spiBusy; }
};
