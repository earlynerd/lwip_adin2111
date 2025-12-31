/*
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * Please review the LICENSE.md file included with this example. If you have any questions
 * or concerns with licensing, please contact techsupport@sparkfun.com.
 * Distributed as-is; no warranty is given.
 */
#include <Arduino.h>
#include "SinglePairEthernet.h"
#include "boardsupport.h"

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
    // Serial.printf("[ADIN] TX Callback, release pbuf %d\r\n", idx);
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
    // Serial.printf("[ADIN] RX Callback, len: %d\r\n", receivedLen);
    //  2. Try to get a NEW buffer from our pool
    uint8_t *freshBuffer = rxPool.getFreeBuffer();

    if (freshBuffer != nullptr)
    {
        // SUCCESS: We have a replacement.

        // A. Queue the filled buffer for lwIP to read later
        rxPool.queueRxPacket(filledBuffer, receivedLen);
        // Serial.println("packet queued");
        //  B. Update the descriptor with the fresh buffer
        //  When this callback returns (or you re-submit), the hardware
        //  will use this new memory location for the next packet.
        pRxBufDesc->pBuf = freshBuffer;
        // submitRxBuffer(pRxBufDesc);
    }
    else
    {
        // FAILURE: Pool is empty (lwIP is too slow / storm of packets)
        // Serial.println("Fail to get fresh rx buffer.");
        // We cannot save this packet. We leave pDesc->pBuf pointing
        // to the OLD buffer. The hardware will effectively overwrite
        // this unread packet with the next one.

        // Optional: Increment a "dropped packet" counter
    }
    adi_eth_Result_e res = submitRxBuffer(pRxBufDesc);
    if(res != ADI_ETH_SUCCESS)
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

bool SinglePairEthernet::begin(uint8_t *retries, uint8_t *mac, uint8_t cs_pin)
{

    adi_eth_Result_e result = ADI_ETH_SUCCESS;
    // txBufIdx =  0;
    if (mac)
    {
        memcpy(macAddr, mac, 6);
        // setMac(mac);
    }
    result = sfe_spe_advanced::begin(retries, cs_pin);

    setUserContext((void *)this);
    if (result != ADI_ETH_SUCCESS)
    {
        initResultCounters[(int)result]++;
    }
    else
        result = enableDefaultBehavior();
    if (result != ADI_ETH_SUCCESS)
        initResultCounters[(int)result]++;
    hdev = sfe_spe_advanced::getDevice();
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
        ;
        rxBufDesc[i].cbFunc = rxCallback_C_Compatible;

        result = submitRxBuffer(&rxBufDesc[i]);
        if (result)
        {
            initResultCounters[(int)result]++;
            Serial.println("fail, submiting rxbuffer");
        }
    }

    adin2111_PhyWrite(hDevice, ADIN2111_PORT_1, ADDR_LED_POLARITY, 0x01);
    adin2111_PhyWrite(hDevice, ADIN2111_PORT_2, ADDR_LED_POLARITY, 0x01);
    adin2111_PhyWrite(hDevice, ADIN2111_PORT_1, ADDR_LED_CNTRL, 0x8E84);
    adin2111_PhyWrite(hDevice, ADIN2111_PORT_2, ADDR_LED_CNTRL, 0x8E84);
    adin2111_PhyWrite(hDevice, ADIN2111_PORT_1, ADDR_LED0_BLINK_TIME_CNTRL, 0x0303);
    adin2111_PhyWrite(hDevice, ADIN2111_PORT_2, ADDR_LED0_BLINK_TIME_CNTRL, 0x0303);

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
    // setCutThroughMode(false, false, true);
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
    // Serial.printf("[ADIN] next packet size: %d\r\n", sz);

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