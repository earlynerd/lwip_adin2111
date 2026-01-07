/*
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * Please review the LICENSE.md file included with this example. If you have any questions
 * or concerns with licensing, please contact techsupport@sparkfun.com.
 * Distributed as-is; no warranty is given.
 */
#include <Arduino.h>
#include "SinglePairEthernet.h"
#include "boardsupport.h"

const int ADIN2111_INIT_ITER = 5;

// The next three function are static member functions. Static member functions are needed to get a function
// pointer that we can shove into the C function that attaches the interrupt in the driver.
void SinglePairEthernet::linkCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(pCBParam);
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->linkCallback(pCBParam, Event, pArg);
    }
}
void SinglePairEthernet::txCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(pCBParam);
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->txCallback(pCBParam, Event, pArg);
    }
}

void SinglePairEthernet::rxCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(pCBParam);
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->rxCallback(pCBParam, Event, pArg);
    }
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
    if (idx >= 0 && idx < 4)
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

        // Optional: Increment a "dropped packet" counter
    }
    adi_eth_Result_e res = submitRxBuffer(pRxBufDesc);
    if (res != ADI_ETH_SUCCESS)
    {
        rxResultCounters[(int)res]++;
    }
    // uint32_t i;
    // for (i = 0; i < SPE_NUM_BUFS; i++)
    // {
    //    if (&rxBuf[i][0] == pRxBufDesc->pBuf)
    //    {
    //        rxBufAvailable[i] = true;
    //        break;
    //   }
    //}

    // Call user Callback
    // if (userRxCallback && (pRxBufDesc->trxSize > SPE_FRAME_HEADER_SIZE))
    //{
    // userRxCallback(pRxBufDesc);     //call rxcallback with whole pbuf, no trimming
    // pRxBufDesc->cbFunc = rxCallback_C_Compatible;

    // rxBufAvailable[i] = false;
    //}
}

void SinglePairEthernet::linkCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    linkStatus = *(adi_eth_LinkStatus_e *)pArg;
    // call user callback
    if (userLinkCallback)
    {

        userLinkCallback(linkStatus == ADI_ETH_LINK_STATUS_UP);
    }
}

void SinglePairEthernet::setSPI(SPIClass &spi)
{
    BSP_SetSPIClass(&spi);
}

bool SinglePairEthernet::begin(uint8_t *retries, uint8_t *mac, uint8_t cs, uint8_t intr, uint8_t reset, uint8_t cfg0, uint8_t cfg1)
{
    adi_eth_Result_e result = ADI_ETH_SUCCESS;
    if (mac)
    {
        memcpy(macAddr, mac, 6);
    }
    // Using LED_BUILTIN as default status pin since it's not passed in
    // result = sfe_spe_advanced::begin(retries, LED_BUILTIN, intr, reset, cs);

    BSP_SetCfgPins(cfg0, cfg1);

    BSP_ConfigSystem(255, 255, reset, cs);
    if (BSP_InitSystem())
    {
        return false;
    }
    uint8_t count = 0;
    BSP_HWReset(true);
    while (digitalRead(intr))
        ;
    for (uint32_t i = 0; i < ADIN2111_INIT_ITER; i++)
    {
        result = init(&drvConfig);
        count++;
        if (result == ADI_ETH_SUCCESS)
        {
            break;
        }
        delay(100);
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

bool SinglePairEthernet::begin(uint8_t *retries, uint8_t *mac, uint8_t cs_pin)
{

    adi_eth_Result_e result = ADI_ETH_SUCCESS;
    // txBufIdx =  0;
    if (mac)
    {
        memcpy(macAddr, mac, 6);
        // setMac(mac);
    }

    BSP_ConfigSystemCS(cs_pin);
    if (BSP_InitSystem())
    {
        return false;
    }

    BSP_HWReset(true);

    uint8_t count = 0;
    for (uint32_t i = 0; i < ADIN2111_INIT_ITER; i++)
    {
        result = init(&drvConfig);
        count++;
        if (result == ADI_ETH_SUCCESS)
        {
            break;
        }
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
/*
bool SinglePairEthernet::begin(uint8_t *retries, uint8_t *mac, uint8_t status, uint8_t interrupt, uint8_t reset, uint8_t chip_select)
{
    adi_eth_Result_e result;

    if (mac)
    {
        setMac(mac);
    }

    result = sfe_spe_advanced::begin(retries, status, interrupt, reset, chip_select);
    if(result) Serial.println("fail, sfe advanced begin");
    setUserContext((void *)this);
    if (result == ADI_ETH_SUCCESS)
    {
        result = enableDefaultBehavior();
        if(result) Serial.println("fail, enable defaults");
    }

    return (result == ADI_ETH_SUCCESS);
}
*/
adi_eth_Result_e SinglePairEthernet::enableDefaultBehavior()
{
    adi_eth_Result_e result = ADI_ETH_SUCCESS;
    int i = 0;
    uint8_t brcstMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if (result == ADI_ETH_SUCCESS)
    {
        adi_mac_AddressRule_t addrRule;
        addrRule.VALUE16 = 0x0000;
        addrRule.TO_OTHER_PORT = 1;
        addrRule.APPLY2PORT1 = 1;
        addrRule.APPLY2PORT2 = 1;
        addrRule.HOST_PRI = 0;
        addrRule.TO_OTHER_PORT = 1;
        addrRule.TO_HOST = 1;
        result = addAddressFilter(brcstMAC, NULL, addrRule);
        if (result)
        {
            Serial.println("fail, setting bcast mac");
            initResultCounters[(int)result]++;
        }
    }

    if (result == ADI_ETH_SUCCESS)
    {
        adi_mac_AddressRule_t rule;
        rule.VALUE16 = 0;
        rule.APPLY2PORT1 = 1;
        rule.APPLY2PORT2 = 1;
        rule.TO_OTHER_PORT = 0;
        rule.HOST_PRI = 1;
        rule.TO_HOST = 1;
        uint8_t mask = 0xff;
        result = addAddressFilter(macAddr, &mask, rule);
        if (result)
        {
            Serial.println("fail, setting own mac");
            initResultCounters[(int)result]++;
        }
    }
    if (result == ADI_ETH_SUCCESS)
    {
        // result = phyDriverEntry.RegisterCallback(hdev->pPhyDevice[0],linkCallback_C_Compatible,  ADI_PHY_EVT_LINK_STAT_CHANGE, (void*)1);
        result = registerCallback(linkCallback_C_Compatible, ADI_MAC_EVT_LINK_CHANGE);
        if (result)
        {
            Serial.println("fail, setting link callback");
            initResultCounters[(int)result]++;
        }
    }

    for (uint32_t i = 0; i < SPE_NUM_BUFS; i++)
    {
        if (result != ADI_ETH_SUCCESS)
        {
            initResultCounters[(int)result]++;
            break;
        }
        uint8_t *buf = rxPool.getFreeBuffer();
        // All tx buffers start available to write, no rx buffers start available to read
        txBufAvailable[i] = true;
        txBufDesc[i].cbFunc = txCallback_C_Compatible;
        txBufDesc[i].bufSize = TX_PACKET_SIZE;
        // rxBufAvailable[i] = false;
        rxBufDesc[i].pBuf = buf;
        rxBufDesc[i].bufSize = RX_PACKET_SIZE;
        rxBufDesc[i].cbFunc = rxCallback_C_Compatible;

        result = submitRxBuffer(&rxBufDesc[i]);
        if (result)
        {
            initResultCounters[(int)result]++;
            Serial.println("fail, submiting rxbuffer");
        }
    }

    adin2111_PhyWrite(hdev, ADIN2111_PORT_1, ADDR_LED_POLARITY, 0x01);
    adin2111_PhyWrite(hdev, ADIN2111_PORT_2, ADDR_LED_POLARITY, 0x01);
    adin2111_PhyWrite(hdev, ADIN2111_PORT_1, ADDR_LED_CNTRL, 0x8E84);
    adin2111_PhyWrite(hdev, ADIN2111_PORT_2, ADDR_LED_CNTRL, 0x8E84);
    adin2111_PhyWrite(hdev, ADIN2111_PORT_1, ADDR_LED0_BLINK_TIME_CNTRL, 0x0303);
    adin2111_PhyWrite(hdev, ADIN2111_PORT_2, ADDR_LED0_BLINK_TIME_CNTRL, 0x0303);

    uint16_t dat;
    // adin2111_PhyRead(hDevice, ADIN2111_PORT_1, ADDR_AN_ADV_ABILITY_H, &dat);
    // adin2111_PhyWrite(hDevice, ADIN2111_PORT_1, ADDR_AN_ADV_ABILITY_H, dat & ~(BITM_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_ABL));

    // adin2111_PhyRead(hDevice, ADIN2111_PORT_2, ADDR_AN_ADV_ABILITY_H, &dat);
    // adin2111_PhyWrite(hDevice, ADIN2111_PORT_2, ADDR_AN_ADV_ABILITY_H, dat & ~(BITM_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_ABL));
    // setPromiscuousMode(ADIN2111_PORT_1, true);
    // setPromiscuousMode(ADIN2111_PORT_2, true);
    uint32_t reg;

    /// setCutThroughMode(false, false, true);
    // if (result == ADI_ETH_SUCCESS)
    //{
    //     result = syncConfig();
    //     if(result) Serial.println("fail, sync config 1");
    // }
    setCutThroughMode(false, false, false);
    if (result == ADI_ETH_SUCCESS)
    {
        result = enable();
        if (result)
        {
            initResultCounters[(int)result]++;
            Serial.println("fail, while enable");
        }
    }
    if (result == ADI_ETH_SUCCESS)
    {
        result = syncConfig();
        if (result)
        {
            initResultCounters[(int)result]++;
            Serial.println("fail, sync config 2");
        }
    }

    return result;
}
uint16_t SinglePairEthernet::sendFrame(uint8_t *data, int dataLen)
{
    // 1. Find a free Hardware Descriptor slot
    int descIdx = -1;
    {
        // Short critical section to find a free slot
        // uint32_t s = save_and_disable_interrupts();
        for (int i = 0; i < 4; i++)
        {
            if (txBufAvailable[i])
            {
                descIdx = i;
                txBufAvailable[i] = false; // Mark busy immediately so no one else grabs it
                break;
            }
        }
        // restore_interrupts(s);
    }

    if (descIdx == -1)
    {
        // Hardware is full. Drop packet.
        // Returning 0 tells lwIP something went wrong (ERR_MEM),
        // TCP will retry later.
        Serial.println("[ADIN] Drop TX packet.");
        return 0;
    }

    // 2. Get a buffer from our pool
    uint8_t *myBuffer = txPool.allocate();
    if (myBuffer == nullptr)
    {
        // Out of RAM buffers.
        Serial.println("[ADIN] Out of TX Buffers.");
        txBufAvailable[descIdx] = true; // Release the descriptor slot we grabbed
        return 0;
    }

    // 3. COPY the data (Critical Step)
    // We must copy because 'data' is owned by lwIP and might vanish after return.
    if (dataLen > TX_PACKET_SIZE)
        dataLen = TX_PACKET_SIZE;
    memcpy(myBuffer, data, dataLen);
    uint32_t submitLen = dataLen;
    if (submitLen < 60)
    {
        // Zero out the padding area to avoid leaking data
        memset(myBuffer + dataLen, 0, 60 - dataLen);
        submitLen = 60;
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

/*
uint16_t SinglePairEthernet::sendFrame( uint8_t *data, int dataLen)
{
    adi_eth_Result_e result;

    if (dataLen > SPE_FRAME_SIZE)
    {
        Serial.println("TX fail, length exceed");
        return false;
    }

    //memcpy(&txBuf[txBufIdx][0], data, dataLen);
    int transmitLength = dataLen;
    // Pad with 0's to mininmum transmit length
    while (transmitLength < SPE_MIN_PAYLOAD_SIZE)
        txBuf[txBufIdx][transmitLength++] = 0;

    txBufDesc[txBufIdx].pBuf = &txBuf[txBufIdx][0];
    txBufDesc[txBufIdx].trxSize = transmitLength;
    txBufDesc[txBufIdx].bufSize = SPE_MAX_BUF_FRAME_SIZE;
    txBufDesc[txBufIdx].egressCapt = ADI_MAC_EGRESS_CAPTURE_NONE;
    txBufDesc[txBufIdx].cbFunc = txCallback_C_Compatible;

    txBufAvailable[txBufIdx] = false;

    result = submitTxBuffer(ADIN2111_TX_PORT_AUTO, &txBufDesc[txBufIdx]);
    if (result == ADI_ETH_SUCCESS)
    {
        txBufIdx = (txBufIdx + 1) % SPE_NUM_BUFS;
        //setDestMac(destMac); // save most recently successfully sent mac address as the mac to use if none is provided in future calls
    }
    else
    {
        Serial.println("TX fail, submit buffer");
        //If Tx buffer submission fails (for example the Tx queue
        // may be full), then mark the buffer available.
        txBufAvailable[txBufIdx] = true;
    }

    return (result == ADI_ETH_SUCCESS);
}
    */

int SinglePairEthernet::getRxData(uint8_t *data, int dataLen, uint8_t *senderMac)
{
    (void)senderMac;
    return rxPool.readAndRecycle(data, dataLen);
    // bool rxDataAvailable = false;
    // for (int i = 0; i < SPE_NUM_BUFS; i++)
    // {
    //     if (rxBufAvailable[rxBufIdx])
    //{
    //    rxDataAvailable = true;
    //    break;
    //}
    // rxBufIdx = (rxBufIdx + 1) % SPE_NUM_BUFS;
    //}
    // if (rxDataAvailable)
    //{
    // int cpyLen = rxBufDesc[rxBufIdx].trxSize - SPE_FRAME_HEADER_SIZE;
    // cpyLen = (cpyLen < dataLen) ? cpyLen : dataLen;
    // memcpy(senderMac, (char *)&(rxBufDesc[rxBufIdx].pBuf[SPE_MAC_SIZE]), SPE_MAC_SIZE); // second set of 6 bytes are senders MAC address
    // memcpy(data, (char *)&(rxBufDesc[rxBufIdx].pBuf[SPE_FRAME_HEADER_SIZE]), cpyLen);   // data starts 14 bytes in, after the frame header
    // submitRxBuffer(&rxBufDesc[rxBufIdx]);
    // rxBufAvailable[rxBufIdx] = false;
    // rxSinceLastCheck = false;
    // return cpyLen;
    //}
}

void SinglePairEthernet::discardFrame()
{
    rxPool.discardCurrentPacket();
}

bool SinglePairEthernet::getRxAvailable()
{
    return rxPool.getLevel() > 0;
}

uint16_t SinglePairEthernet::getRxLength()
{
    BSP_IRQCallback();
    uint16_t sz = rxPool.peekNextPacketSize();

    // bool rxDataAvailable = false;
    // uint32_t idx = rxBufIdx;                    //dont modify, we are peeking
    // for (int i = 0; i < SPE_NUM_BUFS; i++)
    //{
    //     if (rxBufAvailable[idx])
    //     {
    //        rxDataAvailable = true;
    //        break;
    //    }
    //   idx = (idx + 1) % SPE_NUM_BUFS;
    //}
    // if (rxDataAvailable)
    // {
    //     return rxBufDesc[rxBufIdx].trxSize - SPE_FRAME_HEADER_SIZE;
    // }
    return sz;
}

adin2111_DeviceHandle_t SinglePairEthernet::getDevice()
{
    return hdev;
}

bool SinglePairEthernet::indenticalMacs(uint8_t *mac1, uint8_t *mac2)
{
    if (!mac1 || !mac2)
    {
        return false;
    }
    return ((mac1[0] == mac2[0]) &&
            (mac1[1] == mac2[1]) &&
            (mac1[2] == mac2[2]) &&
            (mac1[3] == mac2[3]) &&
            (mac1[4] == mac2[4]) &&
            (mac1[5] == mac2[5]));
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
    return getLinkStatus();
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