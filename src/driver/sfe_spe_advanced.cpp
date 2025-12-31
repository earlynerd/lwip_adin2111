/*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions 
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/

#include "sfe_spe_advanced.h"

const int ADIN2111_INIT_ITER = 5;

adin2111_DeviceHandle_t sfe_spe_advanced::getDevice()
{
    return &dev;
}

adi_eth_Result_e sfe_spe_advanced::begin(uint8_t* retries, uint8_t cs_pin)
{
    adi_eth_Result_e result;
    BSP_ConfigSystemCS(cs_pin);
    if(BSP_InitSystem())
    {
        return ADI_ETH_DEVICE_UNINITIALIZED;
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
    *retries += count-1;
    return result;
}

adi_eth_Result_e sfe_spe_advanced::begin(uint8_t* retries, uint8_t status, uint8_t interrupt, uint8_t reset, uint8_t chip_select)
{
    adi_eth_Result_e result;
    BSP_ConfigSystem(status, interrupt, reset, chip_select);
    
    if(BSP_InitSystem())
    {
        
        return ADI_ETH_DEVICE_UNINITIALIZED;
    }
    uint8_t count = 0;
    BSP_HWReset(true);
    while(digitalRead(interrupt));
    for (uint32_t i = 0; i < ADIN2111_INIT_ITER; i++)
    {
        result = init(&drvConfig);
        count++;
        if (result == ADI_ETH_SUCCESS)
        {
            break;
        }
    }
    *retries += count-1;
    return result;
}

adi_eth_Result_e    sfe_spe_advanced::init                    (adin2111_DriverConfig_t *pCfg) 
{
    return adin2111_Init(hDevice, pCfg);
}

adi_eth_Result_e    sfe_spe_advanced::unInit                  ()
{
    return adin2111_UnInit(hDevice);
}

adi_eth_Result_e    sfe_spe_advanced::getDeviceId             (adin2111_DeviceId_t *pDevId)
{
    return adin2111_GetDeviceId(hDevice, pDevId);
}

adi_eth_Result_e    sfe_spe_advanced::enable                  ()
{
    return adin2111_Enable(hDevice);
}

adi_eth_Result_e    sfe_spe_advanced::disable                 ()
{
    return adin2111_Disable(hDevice);
}

adi_eth_Result_e    sfe_spe_advanced::reset                   (adi_eth_ResetType_e resetType)
{
    return adin2111_Reset(hDevice, resetType);
}

adi_eth_Result_e    sfe_spe_advanced::syncConfig              ()
{
    return adin2111_SyncConfig(hDevice);
}

adi_eth_Result_e    sfe_spe_advanced::getLinkStatus           (adin2111_Port_e port, adi_eth_LinkStatus_e *linkStatus)
{
    return adin2111_GetLinkStatus(hDevice, port, linkStatus);
}

adi_eth_Result_e    sfe_spe_advanced::getStatCounters         (adin2111_Port_e port, adi_eth_MacStatCounters_t *stat)
{
    return adin2111_GetStatCounters(hDevice, port, stat);
}

adi_eth_Result_e    sfe_spe_advanced::ledEn                   (adin2111_Port_e port, bool enable)
{
    return adin2111_LedEn(hDevice, port, enable);
}

adi_eth_Result_e    sfe_spe_advanced::setLoopbackMode         (adin2111_Port_e port, adi_phy_LoopbackMode_e loopbackMode)
{
    return adin2111_SetLoopbackMode(hDevice, port, loopbackMode);
}

adi_eth_Result_e    sfe_spe_advanced::setTestMode             (adin2111_Port_e port, adi_phy_TestMode_e testMode)
{
    return adin2111_SetTestMode(hDevice, port, testMode);
}

adi_eth_Result_e    sfe_spe_advanced::addAddressFilter        (uint8_t *macAddr, uint8_t *macAddrMask, adi_mac_AddressRule_t addrRule)
{
    return adin2111_AddAddressFilter(hDevice, macAddr, macAddrMask, addrRule);
}

adi_eth_Result_e    sfe_spe_advanced::clearAddressFilter      (uint32_t addrIndex)
{
    return adin2111_ClearAddressFilter(hDevice, addrIndex);
}

adi_eth_Result_e    sfe_spe_advanced::submitTxBuffer          (adin2111_TxPort_e port, adi_eth_BufDesc_t *pBufDesc)
{
    return adin2111_SubmitTxBuffer(hDevice, port, pBufDesc);
}

adi_eth_Result_e    sfe_spe_advanced::submitRxBuffer          (adi_eth_BufDesc_t *pBufDesc)
{
    return adin2111_SubmitRxBuffer(hDevice, pBufDesc);
}

#if defined(ADI_MAC_ENABLE_RX_QUEUE_HI_PRIO)
adi_eth_Result_e    TwoWire_Eth::submitRxBufferHp        (adi_eth_BufDesc_t *pBufDesc)
{
    return adin2111_SubmitRxBufferHp(hDevice, pBufDesc);
}
#endif

adi_eth_Result_e    sfe_spe_advanced::setPromiscuousMode      (adin2111_Port_e port, bool bFlag)
{
    return adin2111_SetPromiscuousMode(hDevice, port, bFlag);
}

adi_eth_Result_e    sfe_spe_advanced::getPromiscuousMode      (adin2111_Port_e port, bool *pFlag)
{
    return adin2111_GetPromiscuousMode(hDevice, port, pFlag);
}

#if defined(SPI_OA_EN)
adi_eth_Result_e    sfe_spe_advanced::setChunkSize            (adi_mac_OaCps_e cps)
{
    return adin2111_SetChunkSize(hDevice, cps);
}

adi_eth_Result_e    sfe_spe_advanced::getChunkSize            (adi_mac_OaCps_e *pCps)
{
    return adin2111_GetChunkSize(hDevice, pCps);
}
#endif

adi_eth_Result_e    sfe_spe_advanced::setCutThroughMode       (bool txcte, bool rxcte, bool p2pcte)
{
    return adin2111_SetCutThroughMode(hDevice, txcte, rxcte, p2pcte);
}

adi_eth_Result_e    sfe_spe_advanced::getCutThroughMode       (bool *pTxcte, bool *pRxcte, bool* p2pcte)
{
    return adin2111_GetCutThroughMode(hDevice, pTxcte, pRxcte, p2pcte);
}

adi_eth_Result_e    sfe_spe_advanced::setFifoSizes            (adi_mac_FifoSizes_t fifoSizes)
{
    return adin2111_SetFifoSizes(hDevice, fifoSizes);
}

adi_eth_Result_e    sfe_spe_advanced::getFifoSizes            (adi_mac_FifoSizes_t *pFifoSizes)
{
    return adin2111_GetFifoSizes(hDevice, pFifoSizes);
}

adi_eth_Result_e    sfe_spe_advanced::clearFifos              (adi_mac_FifoClrMode_e clearMode)
{
    return adin2111_ClearFifos(hDevice, clearMode);
}

adi_eth_Result_e    sfe_spe_advanced::tsEnable                (adi_mac_TsFormat_e format)
{
    return adin2111_TsEnable(hDevice, format);
}

adi_eth_Result_e    sfe_spe_advanced::tsClear                 ()
{
    return adin2111_TsClear(hDevice);
}

adi_eth_Result_e    sfe_spe_advanced::tsTimerStart            (adi_mac_TsTimerConfig_t *pTimerConfig)
{
    return adin2111_TsTimerStart(hDevice, pTimerConfig);
}

adi_eth_Result_e    sfe_spe_advanced::tsTimerStop             ()
{
    return adin2111_TsTimerStop(hDevice);
}

adi_eth_Result_e    sfe_spe_advanced::tsSetTimerAbsolute      (uint32_t seconds, uint32_t nanoseconds)
{
    return adin2111_TsSetTimerAbsolute(hDevice, seconds, nanoseconds);
}

adi_eth_Result_e    sfe_spe_advanced::tsSyncClock             (int64_t tError, uint64_t referenceTimeNsDiff, uint64_t localTimeNsDiff)
{
    return adin2111_TsSyncClock(hDevice, tError, referenceTimeNsDiff, localTimeNsDiff);
}
/*
adi_eth_Result_e    sfe_spe_advanced::tsGetExtCaptTimestamp   (adi_mac_TsTimespec_t *pCapturedTimespec)
{
    return adin2111_TsGetExtCaptTimestamp(hDevice, pCapturedTimespec);
}
*/
adi_eth_Result_e    sfe_spe_advanced::tsGetEgressTimestamp    (adi_mac_EgressCapture_e egressReg, adi_mac_TsTimespec_t *pCapturedTimespec)
{
    return adin2111_TsGetEgressTimestamp(hDevice, egressReg, pCapturedTimespec);
}

adi_eth_Result_e    sfe_spe_advanced::tsConvert               (uint32_t timestampLowWord, uint32_t timestampHighWord, adi_mac_TsFormat_e format, adi_mac_TsTimespec_t *pTimespec)
{
    return adin2111_TsConvert(timestampLowWord, timestampHighWord, format, pTimespec);
}

int64_t             sfe_spe_advanced::tsSubtract              (adi_mac_TsTimespec_t *pTsA, adi_mac_TsTimespec_t *pTsB)
{
    return adin2111_TsSubtract(pTsA, pTsB);
}

adi_eth_Result_e    sfe_spe_advanced::registerCallback        (adi_eth_Callback_t cbFunc, adi_mac_InterruptEvt_e cbEvent)
{
    // return adin2111_RegisterCallback(hDevice, cbFunc, cbEvent);
    return adin2111_RegisterCallback(hDevice, cbFunc, cbEvent);
}

adi_eth_Result_e    sfe_spe_advanced::setUserContext          (void *pContext)
{
    return adin2111_SetUserContext(hDevice, pContext);
}

void *              sfe_spe_advanced::getUserContext          ()
{
    return adin2111_GetUserContext(hDevice);
}

adi_eth_Result_e    sfe_spe_advanced::writeRegister           (uint16_t regAddr, uint32_t regData)
{
    return adin2111_WriteRegister(hDevice, regAddr, regData);
}

adi_eth_Result_e    sfe_spe_advanced::readRegister            (uint16_t regAddr, uint32_t *regData)
{
    return adin2111_ReadRegister(hDevice, regAddr, regData);
}

adi_eth_Result_e    sfe_spe_advanced::phyWrite                (adin2111_Port_e port, uint32_t regAddr, uint16_t regData)
{
    return adin2111_PhyWrite(hDevice, port, regAddr, regData);
}

adi_eth_Result_e    sfe_spe_advanced::phyRead                 (adin2111_Port_e port, uint32_t regAddr, uint16_t *regData)
{
    return adin2111_PhyRead(hDevice, port, regAddr, regData);
}

adi_eth_Result_e    sfe_spe_advanced::getMseLinkQuality      (adin2111_Port_e port, adi_phy_MseLinkQuality_t *mseLinkQuality)
{
    return adin2111_GetMseLinkQuality(hDevice, port, mseLinkQuality);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenEn             (adin2111_Port_e port, bool enable)
{
    return adin2111_FrameGenEn(hDevice, port, enable);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenSetMode        (adin2111_Port_e port, adi_phy_FrameGenMode_e mode)
{
    return adin2111_FrameGenSetMode(hDevice, port, mode);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenSetFrameCnt    (adin2111_Port_e port, uint32_t frameCnt)
{
    return adin2111_FrameGenSetFrameCnt(hDevice, port, frameCnt);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenSetFramePayload(adin2111_Port_e port, adi_phy_FrameGenPayload_e payload)
{
    return adin2111_FrameGenSetFramePayload(hDevice, port, payload);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenSetFrameLen    (adin2111_Port_e port, uint16_t frameLen)
{
    return adin2111_FrameGenSetFrameLen(hDevice, port, frameLen);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenSetIfgLen      (adin2111_Port_e port, uint16_t ifgLen)
{
    return adin2111_FrameGenSetIfgLen(hDevice, port, ifgLen);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenRestart        (adin2111_Port_e port)
{
    return adin2111_FrameGenRestart(hDevice, port);
}

adi_eth_Result_e    sfe_spe_advanced::frameGenDone           (adin2111_Port_e port, bool *fgDone)
{
    return adin2111_FrameGenDone(hDevice, port, fgDone);
}

adi_eth_Result_e    sfe_spe_advanced::frameChkEn             (adin2111_Port_e port, bool enable)
{
    return adin2111_FrameChkEn(hDevice, port, enable);
}

adi_eth_Result_e    sfe_spe_advanced::frameChkSourceSelect   (adin2111_Port_e port, adi_phy_FrameChkSource_e source)
{
    return adin2111_FrameChkSourceSelect(hDevice, port, source);
}

adi_eth_Result_e    sfe_spe_advanced::frameChkReadFrameCnt   (adin2111_Port_e port, uint32_t *cnt)
{
    return adin2111_FrameChkReadFrameCnt(hDevice, port, cnt);
}

adi_eth_Result_e    sfe_spe_advanced::frameChkReadRxErrCnt   (adin2111_Port_e port, uint16_t *cnt)
{
    return adin2111_FrameChkReadRxErrCnt(hDevice, port, cnt);
}

adi_eth_Result_e    sfe_spe_advanced::frameChkReadErrorCnt   (adin2111_Port_e port, adi_phy_FrameChkErrorCounters_t *cnt)
{
    return adin2111_FrameChkReadErrorCnt(hDevice, port, cnt);
}
