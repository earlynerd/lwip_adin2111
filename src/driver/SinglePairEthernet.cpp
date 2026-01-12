/*
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * Please review the LICENSE.md file included with this example. If you have any questions
 * or concerns with licensing, please contact techsupport@sparkfun.com.
 * Distributed as-is; no warranty is given.
 */
#include <Arduino.h>
#include "SinglePairEthernet.h"
#include "boardsupport.h"
#include "BoardConfig.h"
#include "../utility/ErrorLog.h"

// Configuration constants
const int ADIN2111_INIT_ITER = 5;
const uint16_t LED_POLARITY_ACTIVE_HIGH = 0x01;
const uint16_t LED_CONTROL_DEFAULT = 0x8E84;
const uint16_t LED_BLINK_TIME_DEFAULT = 0x0303;
const uint16_t MIN_FRAME_PADDING_SIZE = 60;

// Template helper to eliminate callback trampoline duplication
// This retrieves the C++ object from the C callback parameters and invokes the member function
template<void (SinglePairEthernet::*MemberFunc)(void*, uint32_t, void*)>
static void callbackTrampoline(void *pCBParam, uint32_t Event, void *pArg)
{
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(pCBParam);
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        (self->*MemberFunc)(pCBParam, Event, pArg);
    }
}

// Static member callback trampolines for C compatibility
// These use the template helper to avoid code duplication
void SinglePairEthernet::linkCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    callbackTrampoline<&SinglePairEthernet::linkCallback>(pCBParam, Event, pArg);
}

void SinglePairEthernet::txCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    callbackTrampoline<&SinglePairEthernet::txCallback>(pCBParam, Event, pArg);
}

void SinglePairEthernet::rxCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    callbackTrampoline<&SinglePairEthernet::rxCallback>(pCBParam, Event, pArg);
}

void SinglePairEthernet::txCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    adi_eth_BufDesc_t *pTxBufDesc = (adi_eth_BufDesc_t *)pArg;

    int idx = pTxBufDesc - &txBufDesc[0];

    // release memory in the packet pool
    if (pTxBufDesc->pBuf)
    {
        txPool.release(pTxBufDesc->pBuf);
    }
    // mark hardware buffer slot available
    if (idx >= 0 && idx < SPE_NUM_BUFS)
    {
        txBufAvailable[idx] = true;
    }
}

void SinglePairEthernet::rxCallback(void *pCBParam, uint32_t Event, void *pArg)
{

    adi_eth_BufDesc_t *pRxBufDesc = (adi_eth_BufDesc_t *)pArg;
    // rxSinceLastCheck = true;
    uint8_t *filledBuffer = pRxBufDesc->pBuf;
    uint32_t receivedLen = pRxBufDesc->trxSize;

    //  2. Try to get a NEW buffer from our pool
    uint8_t *freshBuffer = rxPool.getFreeBuffer();

    if (freshBuffer != nullptr)
    {
        // SUCCESS: We have a replacement.

        // A. Queue the filled buffer for lwIP to read later
        rxPool.queueRxPacket(filledBuffer, receivedLen);

        //  B. Update the descriptor with the fresh buffer
        //  When this callback returns (or you re-submit), the hardware
        //  will use this new memory location for the next packet.
        pRxBufDesc->pBuf = freshBuffer;
        // submitRxBuffer(pRxBufDesc);
    }
    else
    {
        // FAILURE: Pool is empty (lwIP is too slow / storm of packets)

        // We cannot save this packet. We leave pDesc->pBuf pointing
        // to the OLD buffer. The hardware will effectively overwrite
        // this unread packet with the next one.

        // Increment dropped packet counter for debugging
        rxPool.incrementDropped();
    }
    adi_eth_Result_e res = submitRxBuffer(pRxBufDesc);
    if (res != ADI_ETH_SUCCESS)
    {
        rxResultCounters[(int)res]++;
    }
}

void SinglePairEthernet::linkCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    // For ADIN2111, pArg is adi_mac_StatusRegisters_t*, not adi_eth_LinkStatus_e*
    // Query actual link status from PHYs - link is up if either port has link
    bool isUp = getLinkStatus();
    linkStatus = isUp ? ADI_ETH_LINK_STATUS_UP : ADI_ETH_LINK_STATUS_DOWN;

    // call user callback
    if (userLinkCallback)
    {
        userLinkCallback(isUp);
    }
}

void SinglePairEthernet::setSPI(SPIClass &spi)
{
    BSP_SetSPIClass(&spi);
}

bool SinglePairEthernet::begin(uint8_t *retries, const uint8_t *mac, uint8_t cs, uint8_t intr, uint8_t reset, uint8_t cfg0, uint8_t cfg1)
{
    adi_eth_Result_e result = ADI_ETH_SUCCESS;
    // Initialize cached link status to DOWN before callbacks are registered
    linkStatus = ADI_ETH_LINK_STATUS_DOWN;
    if (mac)
    {
        memcpy(macAddr, mac, 6);
    }
    // Using LED_BUILTIN as default status pin since it's not passed in
    // result = sfe_spe_advanced::begin(retries, LED_BUILTIN, intr, reset, cs);

    BSP_SetCfgPins(cfg0, cfg1);


    BSP_ConfigSystem(255, intr, reset, cs);

    if (BSP_InitSystem())
    {
        return false;
    }
    uint8_t count = 0;
    BSP_HWReset(true);


    // Note: INT is not asserted until after init() configures the device.
    // Skip waiting for INT here - the driver handles it properly. 
        

    for (uint32_t i = 0; i < config.initRetries; i++)
    {
        result = init(&drvConfig);
        count++;
        if (result == ADI_ETH_SUCCESS)
        {
            break;
        }
        delay(config.initRetryDelay);
    }
    *retries += count - 1;

    setUserContext((void *)this);
    if (result != ADI_ETH_SUCCESS)
    {
        initResultCounters[(int)result]++;
    }
    else
        result = enableDefaultBehavior();
    if (result != ADI_ETH_SUCCESS)
        initResultCounters[(int)result]++;
    hdev = getDevice();
    return (result == ADI_ETH_SUCCESS);
}

bool SinglePairEthernet::begin(uint8_t *retries, const uint8_t *mac, uint8_t cs_pin)
{
    // Use default pin values (will not be configured by BSP)
    // This simpler version doesn't configure cfg pins or interrupt
    return begin(retries, mac, cs_pin, 255, DEFAULT_ETH_RESET_Pin, 9, 8);
}

// Private helper: Configure MAC address filters
adi_eth_Result_e SinglePairEthernet::configureMACFilters()
{
    adi_eth_Result_e result = ADI_ETH_SUCCESS;
    uint8_t brcstMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // Configure broadcast MAC filter
    adi_mac_AddressRule_t addrRule;
    addrRule.VALUE16 = 0x0000;
    addrRule.TO_OTHER_PORT = 1;
    addrRule.APPLY2PORT1 = 1;
    addrRule.APPLY2PORT2 = 1;
    addrRule.HOST_PRI = 0;
    addrRule.TO_HOST = 1;
    result = addAddressFilter(brcstMAC, NULL, addrRule);
    if (result != ADI_ETH_SUCCESS)
    {
        ErrorLog::logInitError("setting broadcast MAC filter", result);
        initResultCounters[(int)result]++;
        return result;
    }

    // Configure own MAC filter
    adi_mac_AddressRule_t rule;
    rule.VALUE16 = 0;
    rule.APPLY2PORT1 = 1;
    rule.APPLY2PORT2 = 1;
    rule.TO_OTHER_PORT = 0;
    rule.HOST_PRI = 1;
    rule.TO_HOST = 1;
    uint8_t mask = 0xff;
    result = addAddressFilter(macAddr, &mask, rule);
    if (result != ADI_ETH_SUCCESS)
    {
        ErrorLog::logInitError("setting own MAC filter", result);
        initResultCounters[(int)result]++;
    }

    return result;
}

// Private helper: Register event callbacks
adi_eth_Result_e SinglePairEthernet::configureCallbacks()
{
    adi_eth_Result_e result = registerCallback(linkCallback_C_Compatible, ADI_MAC_EVT_LINK_CHANGE);
    if (result != ADI_ETH_SUCCESS)
    {
        ErrorLog::logInitError("registering link callback", result);
        initResultCounters[(int)result]++;
    }
    return result;
}

// Private helper: Initialize RX/TX buffer descriptors
adi_eth_Result_e SinglePairEthernet::configureBuffers()
{
    adi_eth_Result_e result = ADI_ETH_SUCCESS;

    for (uint32_t i = 0; i < SPE_NUM_BUFS; i++)
    {
        if (result != ADI_ETH_SUCCESS)
        {
            break;
        }

        uint8_t *buf = rxPool.getFreeBuffer();

        // Initialize TX descriptor
        txBufAvailable[i] = true;
        txBufDesc[i].cbFunc = txCallback_C_Compatible;
        txBufDesc[i].bufSize = TX_PACKET_SIZE;

        // Initialize RX descriptor
        rxBufDesc[i].pBuf = buf;
        rxBufDesc[i].bufSize = RX_PACKET_SIZE;
        rxBufDesc[i].cbFunc = rxCallback_C_Compatible;

        result = submitRxBuffer(&rxBufDesc[i]);
        if (result != ADI_ETH_SUCCESS)
        {
            ErrorLog::logInitError("submitting RX buffer", result);
            initResultCounters[(int)result]++;
        }
    }

    return result;
}

// Private helper: Configure LED behavior
void SinglePairEthernet::configureLEDs()
{
    // LED configuration errors are non-fatal, but we log them
    adi_eth_Result_e result;

    result = adin2111_PhyWrite(hdev, ADIN2111_PORT_1, ADDR_LED_POLARITY, config.ledPolarity);
    ErrorLog::checkAndLog(result, "LED polarity P1", initResultCounters);

    result = adin2111_PhyWrite(hdev, ADIN2111_PORT_2, ADDR_LED_POLARITY, config.ledPolarity);
    ErrorLog::checkAndLog(result, "LED polarity P2", initResultCounters);

    result = adin2111_PhyWrite(hdev, ADIN2111_PORT_1, ADDR_LED_CNTRL, config.ledControl);
    ErrorLog::checkAndLog(result, "LED control P1", initResultCounters);

    result = adin2111_PhyWrite(hdev, ADIN2111_PORT_2, ADDR_LED_CNTRL, config.ledControl);
    ErrorLog::checkAndLog(result, "LED control P2", initResultCounters);

    result = adin2111_PhyWrite(hdev, ADIN2111_PORT_1, ADDR_LED0_BLINK_TIME_CNTRL, config.ledBlinkTime);
    ErrorLog::checkAndLog(result, "LED blink time P1", initResultCounters);

    result = adin2111_PhyWrite(hdev, ADIN2111_PORT_2, ADDR_LED0_BLINK_TIME_CNTRL, config.ledBlinkTime);
    ErrorLog::checkAndLog(result, "LED blink time P2", initResultCounters);
}

// Private helper: Enable device and sync configuration
adi_eth_Result_e SinglePairEthernet::finalizeEnable()
{
    adi_eth_Result_e result = enable();
    if (result != ADI_ETH_SUCCESS)
    {
        ErrorLog::logInitError("enabling device", result);
        initResultCounters[(int)result]++;
        return result;
    }

    result = syncConfig();
    if (result != ADI_ETH_SUCCESS)
    {
        ErrorLog::logInitError("syncing configuration", result);
        initResultCounters[(int)result]++;
    }

    return result;
}

// Main initialization function - orchestrates all setup steps
adi_eth_Result_e SinglePairEthernet::enableDefaultBehavior()
{
    adi_eth_Result_e result = ADI_ETH_SUCCESS;

    // Step 1: Configure MAC address filters
    result = configureMACFilters();
    if (result != ADI_ETH_SUCCESS) return result;

    // Step 2: Register event callbacks
    result = configureCallbacks();
    if (result != ADI_ETH_SUCCESS) return result;

    // Step 3: Initialize buffer descriptors
    result = configureBuffers();
    if (result != ADI_ETH_SUCCESS) return result;

    // Step 4: Configure LED behavior
    configureLEDs();

    // Step 5: Set cut-through mode and enable device
    setCutThroughMode(config.cutThroughTx, config.cutThroughRx, config.cutThroughP2P);
    result = finalizeEnable();

    return result;
}
uint16_t SinglePairEthernet::sendFrame(const uint8_t *data, int dataLen)
{
    // 1. Find a free Hardware Descriptor slot
    int descIdx = -1;
    {
        // Short critical section to find a free slot
        uint32_t s = save_and_disable_interrupts();
        for (int i = 0; i < SPE_NUM_BUFS; i++)
        {
            if (txBufAvailable[i])
            {
                descIdx = i;
                txBufAvailable[i] = false; // Mark busy immediately so no one else grabs it
                break;
            }
        }
        restore_interrupts(s);
    }

    if (descIdx == -1)
    {
        // Hardware is full. Drop packet.
        // Returning 0 tells lwIP something went wrong (ERR_MEM),
        // TCP will retry later.
        ErrorLog::logWarning("TX descriptor queue full, dropping packet");
        return 0;
    }

    // 2. Get a buffer from our pool
    uint8_t *myBuffer = txPool.allocate();
    if (myBuffer == nullptr)
    {
        // Out of RAM buffers.
        ErrorLog::logWarning("TX buffer pool exhausted, dropping packet");
        txBufAvailable[descIdx] = true; // Release the descriptor slot we grabbed
        return 0;
    }

    // 3. COPY the data (Critical Step)
    // We must copy because 'data' is owned by lwIP and might vanish after return.
    if (dataLen > TX_PACKET_SIZE)
        dataLen = TX_PACKET_SIZE;
    memcpy(myBuffer, data, dataLen);
    uint32_t submitLen = dataLen;
    if (submitLen < config.minFramePadding)
    {
        // Zero out the padding area to avoid leaking data
        memset(myBuffer + dataLen, 0, config.minFramePadding - dataLen);
        submitLen = config.minFramePadding;
    }
    // 4. Setup the Descriptor
    txBufDesc[descIdx].pBuf = myBuffer;
    txBufDesc[descIdx].trxSize = submitLen;
    txBufDesc[descIdx].bufSize = TX_PACKET_SIZE;
    txBufDesc[descIdx].cbFunc = txCallback_C_Compatible; // Your static wrapper
    txBufDesc[descIdx].port = ADIN2111_TX_PORT_AUTO;
    txBufDesc[descIdx].egressCapt = ADI_MAC_EGRESS_CAPTURE_NONE;
    txBufDesc[descIdx].prio = ADI_MAC_RX_FIFO_PRIO_HIGH;
    txBufDesc[descIdx].timestampValid = false;
    txBufDesc[descIdx].refCount = 0;
    txBufDesc[descIdx].timestamp = 0;
    txBufDesc[descIdx].timestampExt = 0;

    // 5. Submit to Hardware
    // This function is specific to your ADI driver
    adi_eth_Result_e res = submitTxBuffer(ADIN2111_TX_PORT_AUTO, &txBufDesc[descIdx]);
    // Serial.printf("TX buf submit result: %d\r\n", res);

    if (res != ADI_ETH_SUCCESS)
    {
        // Rollback if submission failed
        // Serial.println("[ADIN] TX submit fail.");
        txResultCounters[(int)res]++;
        txPool.release(myBuffer);
        txBufAvailable[descIdx] = true;
        return 0;
    }

    return dataLen; // Success!
}

int SinglePairEthernet::getRxData(uint8_t *data, int dataLen, uint8_t *senderMac)
{
    (void)senderMac;
    return rxPool.readAndRecycle(data, dataLen);
}

void SinglePairEthernet::discardFrame()
{
    rxPool.discardCurrentPacket();
}

bool SinglePairEthernet::getRxAvailable()
{
    return rxPool.getLevel() > 0;
}

uint32_t SinglePairEthernet::getRxDroppedCount()
{
    return rxPool.getDroppedCount();
}

bool SinglePairEthernet::poolInitFailed()
{
    return rxPool.initFailed() || txPool.initFailed();
}

uint16_t SinglePairEthernet::getRxLength()
{
    // First check if we already have queued packets - fast path
    uint16_t sz = rxPool.peekNextPacketSize();
    if (sz > 0)
    {
        return sz;
    }

    // Queue is empty - check if INT pin is asserted (LOW) indicating pending data
    // Only process the interrupt if there's actually something to process
    uint8_t intPin = BoardConfig::instance().getInterruptPin();
    if (intPin != 255 && digitalRead(intPin) == LOW)
    {
        BSP_IRQCallback();
        sz = rxPool.peekNextPacketSize();
    }
    return sz;
}

adin2111_DeviceHandle_t SinglePairEthernet::getDevice()
{
    return hdev;
}

bool SinglePairEthernet::identicalMacs(const uint8_t *mac1, const uint8_t *mac2)
{
    if (!mac1 || !mac2)
    {
        return false;
    }
    return (memcmp(mac1, mac2, SPE_MAC_SIZE) == 0);
}

void SinglePairEthernet::setRxCallback(void (*cbFunc)(adi_eth_BufDesc_t *))
{
    userRxCallback = cbFunc;
}

void SinglePairEthernet::setLinkCallback(void (*cbFunc)(bool))
{
    userLinkCallback = cbFunc;
}

bool SinglePairEthernet::getLinkStatus(void)
{
    adi_phy_LinkStatus_e stat1;
    adi_phy_LinkStatus_e stat2;

    adi_phy_Device_t *phy1 = hdev->pPhyDevice[0];
    adi_phy_Device_t *phy2 = hdev->pPhyDevice[1];
    phyDriverEntry.GetLinkStatus(phy1, &stat1);
    phyDriverEntry.GetLinkStatus(phy2, &stat2);
    return ((stat1 == ADI_PHY_LINK_STATUS_UP) || ((stat2 == ADI_PHY_LINK_STATUS_UP)));
}

bool SinglePairEthernet::isLinkUp(void)
{
    // Use cached link status from interrupt callback (no SPI transaction)
    // This prevents link flapping from PHY read inconsistencies which
    // would cause lwIP to repeatedly restart DHCP
    return (linkStatus == ADI_ETH_LINK_STATUS_UP);
}

// Wrapper methods merged from sfe_spe_advanced

adi_eth_Result_e SinglePairEthernet::init(adin2111_DriverConfig_t *pCfg)
{
    return adin2111_Init(hdev, pCfg);
}

adi_eth_Result_e SinglePairEthernet::unInit()
{
    return adin2111_UnInit(hdev);
}

adi_eth_Result_e SinglePairEthernet::getDeviceId(adin2111_DeviceId_t *pDevId)
{
    return adin2111_GetDeviceId(hdev, pDevId);
}

adi_eth_Result_e SinglePairEthernet::enable()
{
    return adin2111_Enable(hdev);
}

adi_eth_Result_e SinglePairEthernet::disable()
{
    return adin2111_Disable(hdev);
}

adi_eth_Result_e SinglePairEthernet::reset(adi_eth_ResetType_e resetType)
{
    return adin2111_Reset(hdev, resetType);
}

adi_eth_Result_e SinglePairEthernet::syncConfig()
{
    return adin2111_SyncConfig(hdev);
}

adi_eth_Result_e SinglePairEthernet::getLinkStatus(adin2111_Port_e port, adi_eth_LinkStatus_e *linkStatus)
{
    return adin2111_GetLinkStatus(hdev, port, linkStatus);
}

adi_eth_Result_e SinglePairEthernet::getStatCounters(adin2111_Port_e port, adi_eth_MacStatCounters_t *stat)
{
    return adin2111_GetStatCounters(hdev, port, stat);
}

adi_eth_Result_e SinglePairEthernet::ledEn(adin2111_Port_e port, bool enable)
{
    return adin2111_LedEn(hdev, port, enable);
}

adi_eth_Result_e SinglePairEthernet::setLoopbackMode(adin2111_Port_e port, adi_phy_LoopbackMode_e loopbackMode)
{
    return adin2111_SetLoopbackMode(hdev, port, loopbackMode);
}

adi_eth_Result_e SinglePairEthernet::setTestMode(adin2111_Port_e port, adi_phy_TestMode_e testMode)
{
    return adin2111_SetTestMode(hdev, port, testMode);
}

adi_eth_Result_e SinglePairEthernet::addAddressFilter(uint8_t *macAddr, uint8_t *macAddrMask, adi_mac_AddressRule_t addrRule)
{
    return adin2111_AddAddressFilter(hdev, macAddr, macAddrMask, addrRule);
}

adi_eth_Result_e SinglePairEthernet::clearAddressFilter(uint32_t addrIndex)
{
    return adin2111_ClearAddressFilter(hdev, addrIndex);
}

adi_eth_Result_e SinglePairEthernet::submitTxBuffer(adin2111_TxPort_e port, adi_eth_BufDesc_t *pBufDesc)
{
    return adin2111_SubmitTxBuffer(hdev, port, pBufDesc);
}

adi_eth_Result_e SinglePairEthernet::submitRxBuffer(adi_eth_BufDesc_t *pBufDesc)
{
    return adin2111_SubmitRxBuffer(hdev, pBufDesc);
}

#if defined(ADI_MAC_ENABLE_RX_QUEUE_HI_PRIO)
adi_eth_Result_e SinglePairEthernet::submitRxBufferHp(adi_eth_BufDesc_t *pBufDesc)
{
    return adin2111_SubmitRxBufferHp(hdev, pBufDesc);
}
#endif

adi_eth_Result_e SinglePairEthernet::setPromiscuousMode(adin2111_Port_e port, bool bFlag)
{
    return adin2111_SetPromiscuousMode(hdev, port, bFlag);
}

adi_eth_Result_e SinglePairEthernet::getPromiscuousMode(adin2111_Port_e port, bool *pFlag)
{
    return adin2111_GetPromiscuousMode(hdev, port, pFlag);
}

#if defined(SPI_OA_EN)
adi_eth_Result_e SinglePairEthernet::setChunkSize(adi_mac_OaCps_e cps)
{
    return adin2111_SetChunkSize(hdev, cps);
}

adi_eth_Result_e SinglePairEthernet::getChunkSize(adi_mac_OaCps_e *pCps)
{
    return adin2111_GetChunkSize(hdev, pCps);
}
#endif

adi_eth_Result_e SinglePairEthernet::setCutThroughMode(bool txcte, bool rxcte, bool p2pcte)
{
    return adin2111_SetCutThroughMode(hdev, txcte, rxcte, p2pcte);
}

adi_eth_Result_e SinglePairEthernet::getCutThroughMode(bool *pTxcte, bool *pRxcte, bool *p2pcte)
{
    return adin2111_GetCutThroughMode(hdev, pTxcte, pRxcte, p2pcte);
}

adi_eth_Result_e SinglePairEthernet::setFifoSizes(adi_mac_FifoSizes_t fifoSizes)
{
    return adin2111_SetFifoSizes(hdev, fifoSizes);
}

adi_eth_Result_e SinglePairEthernet::getFifoSizes(adi_mac_FifoSizes_t *pFifoSizes)
{
    return adin2111_GetFifoSizes(hdev, pFifoSizes);
}

adi_eth_Result_e SinglePairEthernet::clearFifos(adi_mac_FifoClrMode_e clearMode)
{
    return adin2111_ClearFifos(hdev, clearMode);
}

adi_eth_Result_e SinglePairEthernet::tsEnable(adi_mac_TsFormat_e format)
{
    return adin2111_TsEnable(hdev, format);
}

adi_eth_Result_e SinglePairEthernet::tsClear()
{
    return adin2111_TsClear(hdev);
}

adi_eth_Result_e SinglePairEthernet::tsTimerStart(adi_mac_TsTimerConfig_t *pTimerConfig)
{
    return adin2111_TsTimerStart(hdev, pTimerConfig);
}

adi_eth_Result_e SinglePairEthernet::tsTimerStop()
{
    return adin2111_TsTimerStop(hdev);
}

adi_eth_Result_e SinglePairEthernet::tsSetTimerAbsolute(uint32_t seconds, uint32_t nanoseconds)
{
    return adin2111_TsSetTimerAbsolute(hdev, seconds, nanoseconds);
}

adi_eth_Result_e SinglePairEthernet::tsSyncClock(int64_t tError, uint64_t referenceTimeNsDiff, uint64_t localTimeNsDiff)
{
    return adin2111_TsSyncClock(hdev, tError, referenceTimeNsDiff, localTimeNsDiff);
}

adi_eth_Result_e SinglePairEthernet::tsGetEgressTimestamp(adi_mac_EgressCapture_e egressReg, adi_mac_TsTimespec_t *pCapturedTimespec)
{
    return adin2111_TsGetEgressTimestamp(hdev, egressReg, pCapturedTimespec);
}

adi_eth_Result_e SinglePairEthernet::tsConvert(uint32_t timestampLowWord, uint32_t timestampHighWord, adi_mac_TsFormat_e format, adi_mac_TsTimespec_t *pTimespec)
{
    return adin2111_TsConvert(timestampLowWord, timestampHighWord, format, pTimespec);
}

int64_t SinglePairEthernet::tsSubtract(adi_mac_TsTimespec_t *pTsA, adi_mac_TsTimespec_t *pTsB)
{
    return adin2111_TsSubtract(pTsA, pTsB);
}

adi_eth_Result_e SinglePairEthernet::registerCallback(adi_eth_Callback_t cbFunc, adi_mac_InterruptEvt_e cbEvent)
{
    return adin2111_RegisterCallback(hdev, cbFunc, cbEvent);
}

adi_eth_Result_e SinglePairEthernet::setUserContext(void *pContext)
{
    return adin2111_SetUserContext(hdev, pContext);
}

void *SinglePairEthernet::getUserContext()
{
    return adin2111_GetUserContext(hdev);
}

adi_eth_Result_e SinglePairEthernet::writeRegister(uint16_t regAddr, uint32_t regData)
{
    return adin2111_WriteRegister(hdev, regAddr, regData);
}

adi_eth_Result_e SinglePairEthernet::readRegister(uint16_t regAddr, uint32_t *regData)
{
    return adin2111_ReadRegister(hdev, regAddr, regData);
}

adi_eth_Result_e SinglePairEthernet::phyWrite(adin2111_Port_e port, uint32_t regAddr, uint16_t regData)
{
    return adin2111_PhyWrite(hdev, port, regAddr, regData);
}

adi_eth_Result_e SinglePairEthernet::phyRead(adin2111_Port_e port, uint32_t regAddr, uint16_t *regData)
{
    return adin2111_PhyRead(hdev, port, regAddr, regData);
}

adi_eth_Result_e SinglePairEthernet::getMseLinkQuality(adin2111_Port_e port, adi_phy_MseLinkQuality_t *mseLinkQuality)
{
    return adin2111_GetMseLinkQuality(hdev, port, mseLinkQuality);
}

adi_eth_Result_e SinglePairEthernet::frameGenEn(adin2111_Port_e port, bool enable)
{
    return adin2111_FrameGenEn(hdev, port, enable);
}

adi_eth_Result_e SinglePairEthernet::frameGenSetMode(adin2111_Port_e port, adi_phy_FrameGenMode_e mode)
{
    return adin2111_FrameGenSetMode(hdev, port, mode);
}

adi_eth_Result_e SinglePairEthernet::frameGenSetFrameCnt(adin2111_Port_e port, uint32_t frameCnt)
{
    return adin2111_FrameGenSetFrameCnt(hdev, port, frameCnt);
}

adi_eth_Result_e SinglePairEthernet::frameGenSetFramePayload(adin2111_Port_e port, adi_phy_FrameGenPayload_e payload)
{
    return adin2111_FrameGenSetFramePayload(hdev, port, payload);
}

adi_eth_Result_e SinglePairEthernet::frameGenSetFrameLen(adin2111_Port_e port, uint16_t frameLen)
{
    return adin2111_FrameGenSetFrameLen(hdev, port, frameLen);
}

adi_eth_Result_e SinglePairEthernet::frameGenSetIfgLen(adin2111_Port_e port, uint16_t ifgLen)
{
    return adin2111_FrameGenSetIfgLen(hdev, port, ifgLen);
}

adi_eth_Result_e SinglePairEthernet::frameGenRestart(adin2111_Port_e port)
{
    return adin2111_FrameGenRestart(hdev, port);
}

adi_eth_Result_e SinglePairEthernet::frameGenDone(adin2111_Port_e port, bool *fgDone)
{
    return adin2111_FrameGenDone(hdev, port, fgDone);
}

adi_eth_Result_e SinglePairEthernet::frameChkEn(adin2111_Port_e port, bool enable)
{
    return adin2111_FrameChkEn(hdev, port, enable);
}

adi_eth_Result_e SinglePairEthernet::frameChkSourceSelect(adin2111_Port_e port, adi_phy_FrameChkSource_e source)
{
    return adin2111_FrameChkSourceSelect(hdev, port, source);
}

adi_eth_Result_e SinglePairEthernet::frameChkReadFrameCnt(adin2111_Port_e port, uint32_t *cnt)
{
    return adin2111_FrameChkReadFrameCnt(hdev, port, cnt);
}

adi_eth_Result_e SinglePairEthernet::frameChkReadRxErrCnt(adin2111_Port_e port, uint16_t *cnt)
{
    return adin2111_FrameChkReadRxErrCnt(hdev, port, cnt);
}

adi_eth_Result_e SinglePairEthernet::frameChkReadErrorCnt(adin2111_Port_e port, adi_phy_FrameChkErrorCounters_t *cnt)
{
    return adin2111_FrameChkReadErrorCnt(hdev, port, cnt);
}

// Populate diagnostic information structure
void SinglePairEthernet::getDiagnostics(DiagnosticInfo& info)
{
    // Clear structure
    memset(&info, 0, sizeof(info));

    // Pool status
    info.rxDropped = rxPool.getDroppedCount();
    info.rxQueueLevel = rxPool.getLevel();
    info.txQueueLevel = txPool.getLevel();

    // Link status
    info.linkUp = isLinkUp();

    // Collect non-zero init errors
    info.initErrorCount = 0;
    for (int i = 1; i < 36 && info.initErrorCount < DiagnosticInfo::MAX_ERRORS; i++) {
        if (initResultCounters[i] > 0) {
            info.initErrors[info.initErrorCount].code = (adi_eth_Result_e)i;
            info.initErrors[info.initErrorCount].count = initResultCounters[i];
            info.initErrorCount++;
        }
    }

    // Collect non-zero RX errors
    info.rxErrorCount = 0;
    for (int i = 1; i < 36 && info.rxErrorCount < DiagnosticInfo::MAX_ERRORS; i++) {
        if (rxResultCounters[i] > 0) {
            info.rxErrors[info.rxErrorCount].code = (adi_eth_Result_e)i;
            info.rxErrors[info.rxErrorCount].count = rxResultCounters[i];
            info.rxErrorCount++;
        }
    }

    // Collect non-zero TX errors
    info.txErrorCount = 0;
    for (int i = 1; i < 36 && info.txErrorCount < DiagnosticInfo::MAX_ERRORS; i++) {
        if (txResultCounters[i] > 0) {
            info.txErrors[info.txErrorCount].code = (adi_eth_Result_e)i;
            info.txErrors[info.txErrorCount].count = txResultCounters[i];
            info.txErrorCount++;
        }
    }
}

// Quick check for any errors
bool SinglePairEthernet::hasErrors()
{
    // Check for non-zero error counters (skip index 0 = SUCCESS)
    for (int i = 1; i < 36; i++) {
        if (initResultCounters[i] > 0 || rxResultCounters[i] > 0 || txResultCounters[i] > 0) {
            return true;
        }
    }
    return rxPool.getDroppedCount() > 0;
}

// Print formatted diagnostics to Serial
void SinglePairEthernet::printDiagnostics()
{
    DiagnosticInfo info;
    getDiagnostics(info);

    Serial.println("========== ADIN2111 Diagnostics ==========");

    // Link Status
    Serial.print("Link Status: ");
    Serial.println(info.linkUp ? "UP" : "DOWN");

    // Buffer Pool Status
    Serial.println("\n--- Buffer Pools ---");
    Serial.print("Memory Mode: ");
    Serial.print(rxPool.getAllocationMode());
    Serial.print(" (RX: ");
    Serial.print(rxPool.getTotalMemory());
    Serial.print(" bytes, TX: ");
    Serial.print(txPool.getTotalMemory());
    Serial.println(" bytes)");

    Serial.print("RX Queue Level: ");
    Serial.print(info.rxQueueLevel);
    Serial.print("/");
    Serial.println(RX_POOL_COUNT);

    Serial.print("TX Queue Free: ");
    Serial.print(info.txQueueLevel);
    Serial.print("/");
    Serial.println(TX_POOL_COUNT);

    if (info.rxDropped > 0) {
        Serial.print("RX Packets Dropped: ");
        Serial.println(info.rxDropped);
    }

    // Error Counters
    if (info.initErrorCount > 0) {
        Serial.println("\n--- Initialization Errors ---");
        for (int i = 0; i < info.initErrorCount; i++) {
            Serial.print("  ");
            Serial.print(ErrorLog::getErrorName(info.initErrors[i].code));
            Serial.print(": ");
            Serial.println(info.initErrors[i].count);
        }
    }

    if (info.rxErrorCount > 0) {
        Serial.println("\n--- RX Errors ---");
        for (int i = 0; i < info.rxErrorCount; i++) {
            Serial.print("  ");
            Serial.print(ErrorLog::getErrorName(info.rxErrors[i].code));
            Serial.print(": ");
            Serial.println(info.rxErrors[i].count);
        }
    }

    if (info.txErrorCount > 0) {
        Serial.println("\n--- TX Errors ---");
        for (int i = 0; i < info.txErrorCount; i++) {
            Serial.print("  ");
            Serial.print(ErrorLog::getErrorName(info.txErrors[i].code));
            Serial.print(": ");
            Serial.println(info.txErrors[i].count);
        }
    }

    if (!hasErrors() && info.rxDropped == 0) {
        Serial.println("\nNo errors detected.");
    }

    Serial.println("==========================================");
}