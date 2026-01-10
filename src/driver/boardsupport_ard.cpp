/*
 *---------------------------------------------------------------------------
 *
 * Copyright (c) 2020, 2021 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc.
 * and its licensors.By using this software you agree to the terms of the
 * associated Analog Devices Software License Agreement.
 *
 *---------------------------------------------------------------------------
 */
/*
 * This file has been ported from its original location to be included with
 * the SparkFun ADIN1110 Arduino driver, file contents have been changed to
 * give generic arduino board support
 */

#include "boardsupport.h"
#include "BoardConfig.h"
#include <string.h>
#define NET_SPI_DATARATE 25000000
#include <Arduino.h>
#include <SPI.h>

// Enable this define to print all spi messages, note this will severely impact performance
// #define DEBUG_SPI

// Enable async SPI on arduino-pico (RP2040/RP2350) for better performance
// The ADI driver's state machine is already designed for async operation
#if defined(ARDUINO_ARCH_RP2040)
#define USE_ASYNC_SPI
#endif

#if defined(ARDUINO_ARCH_MBED)
#include <mbed.h>
rtos::Thread thread;
rtos::Semaphore updates(0);
#endif

#define RESET_DELAY (5)
#define AFTER_RESET_DELAY (90)

#if defined(ARDUINO_ARCH_MBED)
// MBED will not allow SPI calls during ISR, which the callbacks may do
// So instead of directly calling in this function we start a thread that will call the callback after signalled to by this function
int num_int = 0;
void BSP_IRQCallback()
{
    // Signal to thread that this function was called
    updates.release();
}

void thread_fn(void)
{
    while (1)
    {
        // Once acquired, the IRQCallback function has run
        updates.acquire();

        ADI_CB callback = BoardConfig::instance().getGPIOIntCallback();
        if (callback)
        {
            (*callback)(BoardConfig::instance().getGPIOIntCallbackParam(), 0, NULL);
        }
    }
}
#else
// Outside of mbed cores, we just call the callback within the ISR
void BSP_IRQCallback()
{
    ADI_CB callback = BoardConfig::instance().getGPIOIntCallback();
    if (callback)
    {
        (*callback)(BoardConfig::instance().getGPIOIntCallbackParam(), 0, NULL);
    }
}
#endif

extern uint32_t HAL_INT_N_Register_Callback(ADI_CB const *pfCallback, void *const pCBParam);
uint32_t BSP_RegisterIRQCallback(ADI_CB const *intCallback, void *hDevice)
{
    BoardConfig::instance().setGPIOIntCallback((ADI_CB)intCallback, hDevice);
    return 0;
}
/*
 * Functions that are part of the driver, that do nothing in the arduino port
 */
void BSP_ErrorLed(bool on)
{ /*NO ERROR LED*/
}

void BSP_FuncLed1(bool on)
{ /* NO FuncLed1 LED */
}

void BSP_FuncLed1Toggle(void)
{ /* NO FuncLed1 LED */
}

void BSP_FuncLed2(bool on)
{ /* NO FuncLed2 LED */
}

void BSP_FuncLed2Toggle(void)
{ /* NO FuncLed2 LED */
}

void BSP_getConfigPins(uint16_t *value)
{ /* This board has no config pins, so odnt do anything */
}

void BSP_disableInterrupts(void)
{
    // interrupts();
    // detachInterrupt(interrupt_pin);
}

void BSP_enableInterrupts(void)
{
    // attachInterrupt(interrupt_pin, BSP_IRQCallback, FALLING);
    // if(!digitalRead(interrupt_pin)) BSP_IRQCallback();      //if int pin has gone low while we werent looking for an edge, call it now manually
    // interrupts();
}

/*
 * Blocking delay function
 */
void BSP_delayMs(uint32_t delayms)
{
    // volatile uint32_t now;
    // uint32_t checkTime  = BSP_SysNow();
    delay(delayms);
    /* Read SysTick Timer every Ms*/
    /*
    while (1)
    {
      now  = BSP_SysNow();
       if (now - checkTime >= delay)
       {
          break;
       }
       else yield();
    }
    */
}

/*
 * Hardware reset to DUT
 */
void BSP_HWReset(bool set)
{
    uint32_t buf = 0;
    BoardConfig& cfg = BoardConfig::instance();

    digitalWrite(cfg.getResetPin(), LOW);
    pinMode(cfg.getCfg0Pin(), OUTPUT);
    pinMode(cfg.getCfg1Pin(), OUTPUT);
#ifdef SPI_PROT_EN
    digitalWrite(cfg.getCfg0Pin(), LOW);
#else
    digitalWrite(cfg.getCfg0Pin(), HIGH);
#endif
#ifdef SPI_OA_EN
    digitalWrite(cfg.getCfg1Pin(), LOW);
#else
    digitalWrite(cfg.getCfg1Pin(), HIGH);
#endif

    BSP_delayMs(RESET_DELAY);
    cfg.getSPIClass()->beginTransaction(SPISettings(NET_SPI_DATARATE, MSBFIRST, SPI_MODE0));
    cfg.getSPIClass()->transfer(&buf, 4);
    cfg.getSPIClass()->endTransaction();
    BSP_delayMs(RESET_DELAY);
    digitalWrite(cfg.getResetPin(), HIGH);

    BSP_delayMs(AFTER_RESET_DELAY);
}

/* LED functions */
static void bspLedSet(uint16_t pin, bool on)
{

    digitalWrite(pin, on);
}

static void bspLedToggle(uint16_t pin)
{
    digitalWrite(pin, !digitalRead(pin));
}

/*
 * Heartbeat LED, On arduino we just default this to LED_BUILTIN
 */
void BSP_HeartBeat(void)
{
    bspLedToggle(BoardConfig::instance().getStatusLedPin());
}

/*
 * HeartBeat LED, On arduino we just default this to LED_BUILTIN
 */
void BSP_HeartBeatLed(bool on)
{
    bspLedSet(BoardConfig::instance().getStatusLedPin(), on);
}

/* All LEDs toggle, used to indicate hardware failure on the board */
void BSP_LedToggleAll(void)
{
    bspLedSet(BoardConfig::instance().getStatusLedPin(), HIGH);
}

// Function called on SPI transaction completion
void SPI_TxRxCpltCallback(void)
{
    BoardConfig& cfg = BoardConfig::instance();
    bspLedSet(cfg.getChipSelectPin(), HIGH);
    cfg.getSPIClass()->endTransaction();

#if defined(USE_ASYNC_SPI)
    cfg.setSpiBusy(false);
#endif

    ADI_CB callback = cfg.getSPICallback();
    if (callback)
    {
        (*callback)(cfg.getSPICallbackParam(), 0, NULL);
    }
}

#if defined(USE_ASYNC_SPI)
// Callback for arduino-pico async SPI DMA completion
void SPI_AsyncDoneCallback(void)
{
    SPI_TxRxCpltCallback();
}

// Wait for any ongoing async SPI transfer to complete
// Useful during shutdown or when synchronous behavior is required
void BSP_WaitForSpiComplete(void)
{
    BoardConfig& cfg = BoardConfig::instance();
    SPIClass* spi = cfg.getSPIClass();

    while (cfg.isSpiBusy())
    {
        // Poll until DMA transfer completes
        // finishedAsync() returns true when transfer is done
        if (spi->finishedAsync())
        {
            // Transfer finished but callback hasn't fired yet
            // This shouldn't normally happen, but handle it gracefully
            break;
        }
        yield(); // Allow other tasks to run
    }
}
#else
// Stub for platforms without async SPI - nothing to wait for
void BSP_WaitForSpiComplete(void)
{
    // Synchronous SPI - transfer is always complete when function returns
}
#endif

uint32_t BSP_spi2_write_and_read(uint8_t *pBufferTx, uint8_t *pBufferRx, uint32_t nbBytes, bool useDma)
{
    // Validate parameters
    if (!pBufferTx || !pBufferRx)
    {
        return 1;
    }

    BoardConfig& cfg = BoardConfig::instance();
    SPIClass* spi = cfg.getSPIClass();

#if defined(USE_ASYNC_SPI)
    // Arduino-pico async SPI using DMA
    // The ADI driver's state machine is already designed for async operation -
    // it calls HAL_SpiReadWrite, returns, and expects spiCallback to be called
    // when the transfer completes. We just need to make it actually async.

    if (cfg.isSpiBusy())
    {
        // SPI transfer already in progress - this shouldn't happen if the
        // driver's state machine is working correctly, but guard against it
        return 1;
    }

    cfg.setSpiBusy(true);

#ifdef DEBUG_SPI
    Serial.printf("async writing numbytes = %d: ", nbBytes);
    for (uint32_t i = 0; i < nbBytes; i++)
    {
        Serial.printf(" %02X", pBufferTx[i]);
    }
    Serial.println();
#endif

    spi->beginTransaction(SPISettings(NET_SPI_DATARATE, MSBFIRST, SPI_MODE0));
    bspLedSet(cfg.getChipSelectPin(), LOW);

    // Register completion callback and start async transfer
    spi->onTransferDone(SPI_AsyncDoneCallback);

    // transferAsync returns true if transfer was started successfully
    if (!spi->transferAsync(pBufferTx, pBufferRx, nbBytes))
    {
        // Async transfer failed to start - fall back to sync or report error
        bspLedSet(cfg.getChipSelectPin(), HIGH);
        spi->endTransaction();
        cfg.setSpiBusy(false);
        return 1;
    }

    // Transfer started - callback will fire when complete
    // The driver state machine will continue in spiCallback
    return 0;

#else
    // Synchronous SPI for other Arduino platforms
    if (useDma)
    {
        // No DMA support for generic arduino
        return 1;
    }

#ifdef DEBUG_SPI
    Serial.printf("writing numbytes = %d: ", nbBytes);
    for (uint32_t i = 0; i < nbBytes; i++)
    {
        Serial.printf(" %02X", pBufferTx[i]);
    }
    Serial.println();
#endif

    memcpy(pBufferRx, pBufferTx, nbBytes);
    spi->beginTransaction(SPISettings(NET_SPI_DATARATE, MSBFIRST, SPI_MODE0));
    bspLedSet(cfg.getChipSelectPin(), LOW);
    spi->transfer(pBufferRx, nbBytes);
    // Driver expects there to be an interrupt that fires after completion
    // Since SPI is blocking for arduino this is overly complicated, just call the "callback" function now
    SPI_TxRxCpltCallback();

#ifdef DEBUG_SPI
    Serial.printf("read: ");
    for (uint32_t i = 0; i < nbBytes; i++)
    {
        Serial.printf(" %02X", pBufferRx[i]);
    }
    Serial.println();
#endif

    return 0;
#endif // USE_ASYNC_SPI
}

// Register the SPI callback, in the driver the following macro is used:
// extern uint32_t HAL_SPI_Register_Callback(ADI_CB const *pfCallback, void *const pCBParam);
uint32_t BSP_spi2_register_callback(ADI_CB const *pfCallback, void *const pCBParam)
{
    BoardConfig::instance().setSPICallback((ADI_CB)pfCallback, pCBParam);
    return 0;
}

/*
 * Set the Chip select to desired level, true for HIGH or false for LOW
 */
void setSPI2Cs(bool set)
{
    bspLedSet(BoardConfig::instance().getChipSelectPin(), set ? HIGH : LOW);
}

uint32_t BSP_SysNow(void)
{
    return millis();
}

uint32_t BSP_InitSystem(void)
{
    BoardConfig& cfg = BoardConfig::instance();

#if defined(ARDUINO_ARCH_MBED)
    thread.start(thread_fn);
    thread.set_priority(osPriorityHigh);
#endif
    cfg.getSPIClass()->begin();

    uint8_t statusPin = cfg.getStatusLedPin();
    uint8_t intPin = cfg.getInterruptPin();
    uint8_t rstPin = cfg.getResetPin();
    uint8_t csPin = cfg.getChipSelectPin();

    if (statusPin != 255)
        pinMode(statusPin, OUTPUT_12MA);
    if (intPin != 255)
        pinMode(intPin, INPUT_PULLUP);

    pinMode(rstPin, OUTPUT_12MA);
    digitalWrite(rstPin, LOW);
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    pinMode(csPin, OUTPUT_12MA);
    return 0;
}

// Set all the pins that are used throughout this module
uint32_t BSP_ConfigSystem(uint8_t status, uint8_t interrupt, uint8_t reset, uint8_t chip_select)
{
    BoardConfig::instance().setSystemPins(status, interrupt, reset, chip_select);
    return 0;
}

// Change just the chip select pin
uint32_t BSP_ConfigSystemCS(uint8_t chip_select)
{
    BoardConfig::instance().setChipSelectPin(chip_select);
    return 0;
}

void BSP_SetSPIClass(void *spiClass)
{
    if (spiClass != NULL)
    {
        BoardConfig::instance().setSPIClass((SPIClass *)spiClass);
    }
}

void BSP_SetCfgPins(uint8_t cfg0, uint8_t cfg1)
{
    BoardConfig::instance().setCfgPins(cfg0, cfg1);
}
