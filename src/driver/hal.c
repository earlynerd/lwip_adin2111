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

/** @addtogroup hal HAL API
 *  @{
 */
/*
 * This file has been ported from its original location to be included with
 * the SparkFun ADIN1110 Arduino driver, its contents may have been changed
 * from its original form.
*/


#include <string.h>
#include "hal.h"
#include "hal_port_specific.h"

// Platform-specific interrupt control
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350) || defined(PICO_BOARD)
#include "hardware/sync.h"
#elif defined(ARDUINO)
#include <Arduino.h>
// Fallback for other Arduino platforms using Arduino primitives
static inline uint32_t hal_save_and_disable_interrupts(void) {
    noInterrupts();
    return 1;
}
static inline void hal_restore_interrupts(uint32_t state) {
    (void)state;
    interrupts();
}
#define save_and_disable_interrupts() hal_save_and_disable_interrupts()
#define restore_interrupts(s) hal_restore_interrupts(s)
#else
// Non-Arduino fallback - must be implemented per platform
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t state) { (void)state; }
#endif

#ifdef MDIO_GPIO
#include "../../mdio_gpio/mdio_gpio.h"
#endif


/*
 * @brief MDIO Read Clause 45
 *
 * @param [in]  phyAddr - Hardware PHY address
 * @param [in]  phyReg - Register address in clause 45 combined devType and regAddr
 * @param [out] phyData - pointer to the data buffer
 *
 * @return error if TA bit is not pulled down by the slave
 *
 * @details
 *
 * @sa
 */
uint32_t HAL_PhyRead(uint8_t hwAddr, uint32_t RegAddr, uint16_t *data)
{
#ifdef MDIO_GPIO
    return (uint32_t)mdioGPIORead_cl45(hwAddr, RegAddr, data);
#else
    //return (uint32_t)adi_MdioRead_Cl45(hwAddr, RegAddr, data);
    return 0;
#endif
}

/*
 * @brief MDIO Write Clause 45
 *
 * @param [in] phyAddr - Hardware Phy address
 * @param [in] phyReg - Register address in clause 45 combined devAddr and regAddr
 * @param [out] phyData -  data
 * @return none
 *
 * @details
 *
 * @sa
 */
uint32_t HAL_PhyWrite(uint8_t hwAddr, uint32_t RegAddr, uint16_t data)
{
#ifdef MDIO_GPIO
  return mdioGPIOWrite_cl45(hwAddr, RegAddr, data);
#else
  //return adi_MdioWrite_Cl45(hwAddr, RegAddr, data);
  return 0;
#endif
}


// Global state for interrupt nesting - ADI driver uses these for critical sections
static volatile uint32_t irqDisableCount = 0;
static volatile uint32_t savedInterruptState = 0;

uint32_t HAL_DisableIrq(void)
{
    // Use Pico SDK's save_and_disable_interrupts for proper critical section
    // This is nestable - we track the count and only save state on first disable
    if (irqDisableCount == 0) {
        savedInterruptState = save_and_disable_interrupts();
    }
    irqDisableCount++;
    return ADI_HAL_SUCCESS;
}

uint32_t HAL_EnableIrq(void)
{
    if (irqDisableCount > 0) {
        irqDisableCount--;
        if (irqDisableCount == 0) {
            restore_interrupts(savedInterruptState);
        }
    }
    return ADI_HAL_SUCCESS;
}

uint32_t HAL_SetPendingIrq(void)
{
    return ADI_HAL_SUCCESS; //TODO
    // NVIC_SetPendingIRQ(EXTI15_10_IRQn);

    // return ADI_HAL_SUCCESS;
}

uint32_t HAL_GetPendingIrq(void)
{
    return ADI_HAL_SUCCESS; //TODO
    // return NVIC_GetPendingIRQ(EXTI15_10_IRQn);
}

uint32_t HAL_GetEnableIrq(void)
{
    return ADI_HAL_SUCCESS; //TODO
    // return NVIC_GetEnableIRQ(EXTI15_10_IRQn);
}

/*
 * @brief  Register Phy IRQ Callback function
 *
 * @param [in] intCallback
 * @param [in] hDevice - Pointer to Phy device handler
 * @param [out] none
 * @return none
 *
 * @details
 *
 * @sa
 */
uint32_t HAL_RegisterCallback(HAL_Callback_t const *intCallback, void * hDevice)
{
    //return ADI_HAL_SUCCESS; //TODO
    return BSP_RegisterIRQCallback (intCallback, hDevice);
}

/*
 * @brief SPI write/read
 *
 * @param [in] pBufferTx - Pointer to transmit buffer
 * @param [in] nbBytes - Number bytes to send
 * @param [in] useDma - Enable/Disable DMA transfer for SPI
 * @param [out] pBufferRx - Pointer to receive buffer
 * @return none
 *
 * @details
 *
 * @sa
 */
uint32_t HAL_SpiReadWrite(uint8_t *pBufferTx, uint8_t *pBufferRx, uint32_t nbBytes, bool useDma)
{
    return BSP_spi2_write_and_read (pBufferTx, pBufferRx, nbBytes, useDma);
    // if(useDma)
    // {
    //     return ADI_HAL_ERROR;
    // }
    // for(int i = 0; i < nbBytes; i++)
    // {
    //     //pBufferRx[i] = SPI.transfer(pBufferTx[i]);
    // }
}

/*
 * @brief  Register SPI Callback function
 *
 * @param [in] spiCallback - Register SPI IRQ callback function
 * @param [in] hDevice - Pointer to Phy device handler
 * @param [out] none
 * @return none
 *
 * @details
 *
 * @sa
 */
uint32_t HAL_SpiRegisterCallback(HAL_Callback_t const *spiCallback, void * hDevice)
{

    //return ADI_HAL_SUCCESS;
    return BSP_spi2_register_callback (spiCallback, hDevice);
}

uint32_t HAL_Init_Hook(void)
{
    return ADI_HAL_SUCCESS;
}

uint32_t HAL_UnInit_Hook(void)
{
    return ADI_HAL_SUCCESS;
}

/** @}*/

