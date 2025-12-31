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
#include <string.h>
#define NET_SPI_DATARATE 25000000
#include <Arduino.h>
#include <SPI.h>



#define NET_SPI_CFG0_PIN 9
#define NET_SPI_CFG1_PIN 8
// Enable this define to print all spi messages, note this will severely impact performance
// #define DEBUG_SPI

#if defined(ARDUINO_ARCH_MBED)
#include <mbed.h>
rtos::Thread thread;
rtos::Semaphore updates(0);
#endif

#define RESET_DELAY (5)
#define AFTER_RESET_DELAY (90)

static ADI_CB gpfSpiCallback = NULL;
static void *gpSpiCBParam = NULL;

static ADI_CB gpfGPIOIntCallback = NULL;
static void *gpGPIOIntCBParam = NULL;

static uint8_t status_led_pin = LED_BUILTIN;
static uint8_t interrupt_pin = 255;
static uint8_t reset_pin = DEFAULT_ETH_RESET_Pin;
static uint8_t chip_select_pin = DEFAULT_ETH_SPI_CS_Pin;

static SPIClass *SPI_instance = &SPI;

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

        if (gpfGPIOIntCallback)
        {
            (*gpfGPIOIntCallback)(gpGPIOIntCBParam, 0, NULL);
        }
    }
}
#else
// Outside of mbed cores, we just call the callback within the ISR
void BSP_IRQCallback()
{
    if (gpfGPIOIntCallback)
    {
        (*gpfGPIOIntCallback)(gpGPIOIntCBParam, 0, NULL);
    }
}
#endif
//void SPI_TxRxCpltCallback(void);
//void BSP_IRQCallback(void);
// Register the callback for the interrupt pin, in the driver the following macro is used:
// extern uint32_t HAL_INT_N_Register_Callback(ADI_CB const *pfCallback, void *const pCBParam);
uint32_t BSP_RegisterIRQCallback(ADI_CB const *intCallback, void *hDevice)
{
    gpfGPIOIntCallback = (ADI_CB)intCallback;
    gpGPIOIntCBParam = hDevice;

    //attachInterrupt(interrupt_pin, BSP_IRQCallback, FALLING);
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
    //interrupts();
    detachInterrupt(interrupt_pin);
}

void BSP_enableInterrupts(void)
{
    //attachInterrupt(interrupt_pin, BSP_IRQCallback, FALLING);
    //if(!digitalRead(interrupt_pin)) BSP_IRQCallback();      //if int pin has gone low while we werent looking for an edge, call it now manually
    //interrupts();
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
    uint32_t buf=0;
    digitalWrite(reset_pin, LOW);
    pinMode(NET_SPI_CFG0_PIN, OUTPUT);
    pinMode(NET_SPI_CFG1_PIN, OUTPUT);
    #ifdef SPI_PROT_EN
    digitalWrite(NET_SPI_CFG0_PIN, LOW);
    #else
    digitalWrite(NET_SPI_CFG0_PIN, HIGH);
    #endif
    #ifdef SPI_OA_EN
    digitalWrite(NET_SPI_CFG1_PIN, LOW);
    #else
    digitalWrite(NET_SPI_CFG1_PIN, HIGH);
    #endif
    
    BSP_delayMs(RESET_DELAY);
    SPI_instance->beginTransaction(SPISettings(NET_SPI_DATARATE, MSBFIRST, SPI_MODE3));
    SPI_instance->transfer(&buf, 4);
    SPI_instance->endTransaction();
    BSP_delayMs(RESET_DELAY);
    digitalWrite(reset_pin, HIGH);

    BSP_delayMs(AFTER_RESET_DELAY);
    //pinMode(NET_SPI_CFG0_PIN, INPUT);
    //pinMode(NET_SPI_CFG1_PIN, INPUT);
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
    bspLedToggle(status_led_pin);
}

/*
 * HeartBeat LED, On arduino we just default this to LED_BUILTIN
 */
void BSP_HeartBeatLed(bool on)
{
    bspLedSet(status_led_pin, on);
}

/* All LEDs toggle, used to indicate hardware failure on the board */
void BSP_LedToggleAll(void)
{
    bspLedSet(status_led_pin, HIGH);
}

// Function called on SPI transaction completion
void SPI_TxRxCpltCallback(void)
{
    // delayMicroseconds(1);
    bspLedSet(chip_select_pin, HIGH);
    SPI_instance->endTransaction();
    (*gpfSpiCallback)(gpSpiCBParam, 0, NULL);
}
uint32_t BSP_spi2_write_and_read(uint8_t *pBufferTx, uint8_t *pBufferRx, uint32_t nbBytes, bool useDma)
{
    // Validate parameters
    if (!pBufferTx || !pBufferRx)
    {
        return 1;
    }
    if (useDma)
    { // no DMA support for arduino
        return 1;
    }

#ifdef DEBUG_SPI
    Serial.printf("writing numbytes = %d: ", nbBytes);
    for (int i = 0; i < nbBytes; i++)
    {
        Serial.printf(" %02X", pBufferTx[i]);
    }
    Serial.println();
#endif

    memcpy(pBufferRx, pBufferTx, nbBytes);
    SPI_instance->beginTransaction(SPISettings(NET_SPI_DATARATE, MSBFIRST, SPI_MODE0));
    bspLedSet(chip_select_pin, LOW);
    // delayMicroseconds(1);
    SPI_instance->transfer(pBufferRx, nbBytes);
    // Driver expects there to be an interrupt that fires after completion
    // Since SPI is blocking for arduino this is overly complicated, just call the "callback" function now
    SPI_TxRxCpltCallback();

#ifdef DEBUG_SPI
    Serial.printf("read: ");
    for (int i = 0; i < nbBytes; i++)
    {
        Serial.printf(" %02X", pBufferRx[i]);
    }
    Serial.println();
#endif

    return 0;
}



// Register the SPI callback, in the driver the following macro is used:
// extern uint32_t HAL_SPI_Register_Callback(ADI_CB const *pfCallback, void *const pCBParam);
uint32_t BSP_spi2_register_callback(ADI_CB const *pfCallback, void *const pCBParam)
{
    gpfSpiCallback = (ADI_CB)pfCallback;
    gpSpiCBParam = pCBParam;
    return 0;
}

/*
 * Set the Chip select to desired level, true for HIGH or false for LOW
 */
void setSPI2Cs(bool set)
{
    if (set == true)
    {
        bspLedSet(chip_select_pin, HIGH);
    }
    else
    {
        bspLedSet(chip_select_pin, LOW);
    }
}





uint32_t BSP_SysNow(void)
{
    return millis();
}

uint32_t BSP_InitSystem(void)
{
#if defined(ARDUINO_ARCH_MBED)
    thread.start(thread_fn);
    thread.set_priority(osPriorityHigh);
#endif
    SPI_instance->begin();
    pinMode(status_led_pin, OUTPUT_12MA);
    pinMode(interrupt_pin, INPUT_PULLUP);
    digitalWrite(reset_pin, HIGH);
    pinMode(reset_pin, OUTPUT_12MA);
    pinMode(chip_select_pin, OUTPUT);
    digitalWrite(chip_select_pin, HIGH);
    pinMode(chip_select_pin, OUTPUT_12MA);
    return 0;
}

// Set all the pins that are uysed throughout this module
uint32_t BSP_ConfigSystem(uint8_t status, uint8_t interrupt, uint8_t reset, uint8_t chip_select)
{
    status_led_pin = status;
    interrupt_pin = interrupt;
    reset_pin = reset;
    chip_select_pin = chip_select;
    return 0;
}

// Change just the chip select pin
uint32_t BSP_ConfigSystemCS(uint8_t chip_select)
{
    chip_select_pin = chip_select;
    return 0;
}

// User in the functions below, prints debug and error messages from within the driver
uint32_t msgWrite(char *ptr)
{
    Serial.print(ptr);
    return 0;
}

char aDebugString[150u];

void common_Fail(char *FailureReason)
{
    char fail[] = "Failed: ";
    char term[] = "\n\r";

    /* Ignore return codes since there's nothing we can do if it fails */
    msgWrite(fail);
    msgWrite(FailureReason);
    msgWrite(term);
}

void common_Perf(char *InfoString)
{
    char term[] = "\n\r";

    /* Ignore return codes since there's nothing we can do if it fails */
    msgWrite(InfoString);
    msgWrite(term);
}
