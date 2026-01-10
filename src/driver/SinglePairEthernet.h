/*
 * ADIN2111 Single Pair Ethernet Driver for Arduino
 *
 * This driver provides Arduino-friendly access to the Analog Devices ADIN2111
 * dual-port 10BASE-T1L Single Pair Ethernet MAC/PHY chip.
 *
 * Features:
 * - Zero-copy packet buffer management with configurable memory allocation
 * - Interrupt-driven RX/TX with automatic buffer recycling
 * - Link status monitoring and callbacks
 * - Comprehensive diagnostics and error tracking
 * - Full access to ADIN2111 advanced features (timestamps, frame gen/check, etc.)
 *
 * Basic Usage:
 *   SinglePairEthernet eth;
 *   eth.setSPI(SPI);
 *   if (eth.begin(&retries, myMAC, CS_PIN, INT_PIN, RESET_PIN, CFG0_PIN, CFG1_PIN)) {
 *       // Device initialized successfully
 *   }
 *
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * Please review the LICENSE.md file included with this example.
 * Distributed as-is; no warranty is given.
 */
#ifndef __Sparkfun_SinglePairEth__
#define __Sparkfun_SinglePairEth__

#include <Arduino.h>
#include <SPI.h>
#include "../adi_driver/adin2111.h"
#include "../utility/PacketPool.h"
#include "../utility/Config.h"

//Default MAC addresses used in Analog Devices examples
#define MAC_ADDR_0_0        (0x00)
#define MAC_ADDR_0_1        (0xE0)
#define MAC_ADDR_0_2        (0x22)
#define MAC_ADDR_0_3        (0xFE)
#define MAC_ADDR_0_4        (0xDA)
#define MAC_ADDR_0_5        (0xC9)

#define MAC_ADDR_1_0        (0x00)
#define MAC_ADDR_1_1        (0xE0)
#define MAC_ADDR_1_2        (0x22)
#define MAC_ADDR_1_3        (0xFE)
#define MAC_ADDR_1_4        (0xDA)
#define MAC_ADDR_1_5        (0xCA)

const int SPE_NUM_BUFS = 4;
const int SPE_FRAME_SIZE = 1518;
/* Extra 4 bytes for FCS and 2 bytes for the frame header */
const int SPE_MAX_BUF_FRAME_SIZE = (SPE_FRAME_SIZE + 4 + 2);
const int SPE_MIN_PAYLOAD_SIZE = (46);

const int SPE_MAC_SIZE = 6;
const int SPE_FRAME_HEADER_SIZE = (2*SPE_MAC_SIZE + 2);
const int SPE_ETHERTYPE_IPV4_B0 = 0x80;
const int SPE_ETHERTYPE_IPV4_B1 = 0x00;

class SinglePairEthernet
{
private:
    uint8_t devMem[ADIN2111_DEVICE_SIZE];
    adin2111_DriverConfig_t drvConfig = {
        .pDevMem    = (void *)devMem,
        .devMemSize = sizeof(devMem),
        .fcsCheckEn = false,
    };
    adin2111_DeviceStruct_t dev;
    adin2111_DeviceHandle_t hdev = &dev;

    PacketPool rxPool;
    TxPacketPool txPool;
    //uint8_t rxBuf[SPE_NUM_BUFS][SPE_MAX_BUF_FRAME_SIZE] HAL_ALIGNED_ATTRIBUTE(4);
    //uint8_t txBuf[SPE_NUM_BUFS][SPE_MAX_BUF_FRAME_SIZE] HAL_ALIGNED_ATTRIBUTE(4);
    adi_eth_BufDesc_t rxBufDesc[SPE_NUM_BUFS];
    adi_eth_BufDesc_t txBufDesc[SPE_NUM_BUFS];
    bool txBufAvailable[SPE_NUM_BUFS];
    //bool rxBufAvailable[SPE_NUM_BUFS];
    //uint32_t txBufIdx;
    //uint32_t rxBufIdx;
    //bool rxSinceLastCheck;

    uint8_t macAddr[SPE_MAC_SIZE] = {MAC_ADDR_0_0, MAC_ADDR_0_1, MAC_ADDR_0_2, MAC_ADDR_0_3, MAC_ADDR_0_4, MAC_ADDR_0_5};

    volatile adi_eth_LinkStatus_e    linkStatus;
    ADIN2111Config config;  // Device configuration

public:
    //==========================================================================
    // ESSENTIAL USER API
    // Core methods for basic device operation
    //==========================================================================

    /**
     * @brief Configure SPI interface for communication with ADIN2111
     * @param spi Reference to Arduino SPI object (e.g., SPI, SPI1)
     * @note Must be called before begin()
     */
    void setSPI(SPIClass &spi);

    /**
     * @brief Initialize ADIN2111 device with specified MAC address and pins
     * @param retries Pointer to uint8_t that will be updated with retry count
     * @param mac Pointer to 6-byte MAC address array (NULL to use default)
     * @param cs Chip select pin number
     * @param intr Interrupt pin number (255 to disable)
     * @param reset Reset pin number
     * @param cfg0 Configuration pin 0 number
     * @param cfg1 Configuration pin 1 number
     * @return true if initialization successful, false otherwise
     * @note This is the recommended initialization method
     */
    bool begin(uint8_t* retries, const uint8_t* mac, uint8_t cs, uint8_t intr, uint8_t reset, uint8_t cfg0, uint8_t cfg1);

    /**
     * @brief Initialize ADIN2111 device with minimal pin configuration
     * @param retries Pointer to uint8_t that will be updated with retry count
     * @param mac Pointer to 6-byte MAC address array
     * @param cs_pin Chip select pin number
     * @return true if initialization successful, false otherwise
     * @note Uses default pins for interrupt, reset, cfg0, cfg1
     */
    bool begin(uint8_t* retries, const uint8_t* mac, uint8_t cs_pin);

    /**
     * @brief Check memory pool initialization status
     * @return true if memory allocation failed, false if successful
     * @note Should be checked before calling begin()
     */
    bool poolInitFailed();

    /**
     * @brief Get current link status (cached, no SPI transaction)
     * @return true if link is up, false if down
     * @note This returns the cached status updated by interrupt callback
     */
    bool isLinkUp();

    /**
     * @brief Set MAC address for this device
     * @param mac Pointer to 6-byte MAC address array
     */
    void setMac(const uint8_t* mac);

    /**
     * @brief Get current MAC address
     * @param mac Pointer to 6-byte buffer to receive MAC address
     */
    void getMac(uint8_t* mac);

    /**
     * @brief Register callback function for RX packet events
     * @param cbFunc Function pointer called when packet received
     * @note Callback executed in interrupt context
     */
    void setRxCallback(void (*cbFunc)(adi_eth_BufDesc_t*));

    /**
     * @brief Register callback function for link status changes
     * @param cbFunc Function pointer called when link state changes (receives bool: true=up, false=down)
     */
    void setLinkCallback(void (*cbFunc)(bool));

    //==========================================================================
    // DIAGNOSTIC & STATUS API
    // Methods for monitoring device health and performance
    //==========================================================================

    /**
     * @brief Diagnostic information structure
     */
    adi_eth_Result_e    init                    (adin2111_DriverConfig_t *pCfg);
    adi_eth_Result_e    unInit                  (void);
    adi_eth_Result_e    getDeviceId             (adin2111_DeviceId_t *pDevId);
    adi_eth_Result_e    enable                  (void);
    adi_eth_Result_e    disable                 (void);
    adi_eth_Result_e    reset                   (adi_eth_ResetType_e resetType);
    adi_eth_Result_e    syncConfig              (void);
    adi_eth_Result_e    getLinkStatus           (adin2111_Port_e port, adi_eth_LinkStatus_e *linkStatus);
    adi_eth_Result_e    getStatCounters         (adin2111_Port_e port, adi_eth_MacStatCounters_t *stat);
    adi_eth_Result_e    ledEn                   (adin2111_Port_e port, bool enable);
    adi_eth_Result_e    setLoopbackMode         (adin2111_Port_e port, adi_phy_LoopbackMode_e loopbackMode);
    adi_eth_Result_e    setTestMode             (adin2111_Port_e port, adi_phy_TestMode_e testMode);
    adi_eth_Result_e    addAddressFilter        (uint8_t *macAddr, uint8_t *macAddrMask, adi_mac_AddressRule_t addrRule);
    adi_eth_Result_e    clearAddressFilter      (uint32_t addrIndex);
    adi_eth_Result_e    submitTxBuffer          (adin2111_TxPort_e port, adi_eth_BufDesc_t *pBufDesc);
    adi_eth_Result_e    submitRxBuffer          (adi_eth_BufDesc_t *pBufDesc);
    #if defined(ADI_MAC_ENABLE_RX_QUEUE_HI_PRIO)
    adi_eth_Result_e    submitRxBufferHp        (adi_eth_BufDesc_t *pBufDesc);
    #endif
    adi_eth_Result_e    setPromiscuousMode      (adin2111_Port_e port, bool bFlag);
    adi_eth_Result_e    getPromiscuousMode      (adin2111_Port_e port, bool *pFlag);
    #if defined(SPI_OA_EN)
    adi_eth_Result_e    setChunkSize            (adi_mac_OaCps_e cps);
    adi_eth_Result_e    getChunkSize            (adi_mac_OaCps_e *pCps);
    #endif
    adi_eth_Result_e    setCutThroughMode       (bool txcte, bool rxcte, bool p2pcte);
    adi_eth_Result_e    getCutThroughMode       (bool *pTxcte, bool *pRxcte, bool *p2pcte);
    adi_eth_Result_e    setFifoSizes            (adi_mac_FifoSizes_t fifoSizes);
    adi_eth_Result_e    getFifoSizes            (adi_mac_FifoSizes_t *pFifoSizes);
    adi_eth_Result_e    clearFifos              (adi_mac_FifoClrMode_e clearMode);
    adi_eth_Result_e    tsEnable                (adi_mac_TsFormat_e format);
    adi_eth_Result_e    tsClear                 (void);
    adi_eth_Result_e    tsTimerStart            (adi_mac_TsTimerConfig_t *pTimerConfig);
    adi_eth_Result_e    tsTimerStop             (void);
    adi_eth_Result_e    tsSetTimerAbsolute      (uint32_t seconds, uint32_t nanoseconds);
    adi_eth_Result_e    tsSyncClock             (int64_t tError, uint64_t referenceTimeNsDiff, uint64_t localTimeNsDiff);
    //adi_eth_Result_e    tsGetExtCaptTimestamp   (adi_mac_TsTimespec_t *pCapturedTimespec);
    adi_eth_Result_e    tsGetEgressTimestamp    (adi_mac_EgressCapture_e egressReg, adi_mac_TsTimespec_t *pCapturedTimespec);
    adi_eth_Result_e    tsConvert               (uint32_t timestampLowWord, uint32_t timestampHighWord, adi_mac_TsFormat_e format, adi_mac_TsTimespec_t *pTimespec);
    int64_t             tsSubtract              (adi_mac_TsTimespec_t *pTsA, adi_mac_TsTimespec_t *pTsB);
    adi_eth_Result_e    registerCallback        (adi_eth_Callback_t cbFunc, adi_mac_InterruptEvt_e cbEvent);
    adi_eth_Result_e    setUserContext          (void *pContext);
    void *              getUserContext          (void);
    adi_eth_Result_e    writeRegister           (uint16_t regAddr, uint32_t regData);
    adi_eth_Result_e    readRegister            (uint16_t regAddr, uint32_t *regData);
    adi_eth_Result_e    phyWrite                (adin2111_Port_e port, uint32_t regAddr, uint16_t regData);
    adi_eth_Result_e    phyRead                 (adin2111_Port_e port, uint32_t regAddr, uint16_t *regData);
    adi_eth_Result_e    getMseLinkQuality       (adin2111_Port_e port, adi_phy_MseLinkQuality_t *mseLinkQuality);
    adi_eth_Result_e    frameGenEn              (adin2111_Port_e port, bool enable);
    adi_eth_Result_e    frameGenSetMode         (adin2111_Port_e port, adi_phy_FrameGenMode_e mode);
    adi_eth_Result_e    frameGenSetFrameCnt     (adin2111_Port_e port, uint32_t frameCnt);
    adi_eth_Result_e    frameGenSetFramePayload (adin2111_Port_e port, adi_phy_FrameGenPayload_e payload);
    adi_eth_Result_e    frameGenSetFrameLen     (adin2111_Port_e port, uint16_t frameLen);
    adi_eth_Result_e    frameGenSetIfgLen       (adin2111_Port_e port, uint16_t ifgLen);
    adi_eth_Result_e    frameGenRestart         (adin2111_Port_e port);
    adi_eth_Result_e    frameGenDone            (adin2111_Port_e port, bool *fgDone);
    adi_eth_Result_e    frameChkEn              (adin2111_Port_e port, bool enable);
    adi_eth_Result_e    frameChkSourceSelect    (adin2111_Port_e port, adi_phy_FrameChkSource_e source);
    adi_eth_Result_e    frameChkReadFrameCnt    (adin2111_Port_e port, uint32_t *cnt);
    adi_eth_Result_e    frameChkReadRxErrCnt    (adin2111_Port_e port, uint16_t *cnt);
    adi_eth_Result_e    frameChkReadErrorCnt    (adin2111_Port_e port, adi_phy_FrameChkErrorCounters_t *cnt);
    
    /**
     * @brief Diagnostic information structure
     */
    struct DiagnosticInfo {
        // Pool status
        uint32_t rxDropped;
        size_t rxQueueLevel;
        size_t txQueueLevel;

        // Link status
        bool linkUp;

        // Error counters (indexes to non-zero counters)
        struct ErrorCount {
            adi_eth_Result_e code;
            uint32_t count;
        };
        static const int MAX_ERRORS = 10;
        ErrorCount initErrors[MAX_ERRORS];
        ErrorCount rxErrors[MAX_ERRORS];
        ErrorCount txErrors[MAX_ERRORS];
        int initErrorCount;
        int rxErrorCount;
        int txErrorCount;
    };

    /**
     * @brief Populate diagnostic information structure
     * @param info Reference to DiagnosticInfo to fill with current status
     */
    void getDiagnostics(DiagnosticInfo& info);

    /**
     * @brief Print formatted diagnostics to Serial
     * @note Includes link status, buffer levels, and all error counters
     */
    void printDiagnostics();

    /**
     * @brief Quick check for any errors
     * @return true if any error counters are non-zero or packets dropped
     */
    bool hasErrors();

    /**
     * @brief Get number of RX packets dropped due to pool exhaustion
     * @return Count of dropped packets
     */
    uint32_t getRxDroppedCount();

    //==========================================================================
    // INTERNAL/LWIP ADAPTER API
    // Methods used by ADIN2111_wrap for lwIP integration
    // Most users won't need to call these directly
    //==========================================================================

    /**
     * @brief Send ethernet frame (used by lwIP wrapper)
     * @param data Pointer to frame data
     * @param dataLen Length of frame in bytes
     * @return Number of bytes sent, or 0 on error
     * @note This method is primarily for internal/wrapper use
     */
    uint16_t sendFrame(const uint8_t *data, int dataLen);

    /**
     * @brief Get length of next RX packet in queue
     * @return Packet length in bytes, or 0 if queue empty
     */
    uint16_t getRxLength();

    /**
     * @brief Read RX packet data and recycle buffer
     * @param data Destination buffer
     * @param dataLen Maximum bytes to read
     * @param senderMac Buffer to receive sender MAC (6 bytes, currently unused)
     * @return Number of bytes read
     */
    int getRxData(uint8_t *data, int dataLen, uint8_t *senderMac);

    /**
     * @brief Discard current RX packet and recycle buffer
     */
    void discardFrame();

    /**
     * @brief Check if RX packets are available
     * @return true if packets in RX queue
     */
    bool getRxAvailable();

    /**
     * @brief Get device handle for advanced operations
     * @return Pointer to ADIN2111 device handle
     */
    adin2111_DeviceHandle_t getDevice();

    /**
     * @brief Compare two MAC addresses
     * @return true if MAC addresses are identical
     */
    bool identicalMacs(const uint8_t* mac1, const uint8_t* mac2);

    /**
     * @brief Set destination MAC (currently unused)
     */
    void setDestMac(const uint8_t* mac);

    /**
     * @brief Helper methods for raw data transmission (deprecated)
     * @note Consider using sendFrame() instead
     */
    bool sendData(adin2111_TxPort_e port, const uint8_t *data, int dataLen, const uint8_t *destMac);
    bool sendData(adin2111_TxPort_e port, const uint8_t *data, int dataLen);

    //==========================================================================
    // ADVANCED/EXPERT API
    // Low-level ADIN2111 hardware access methods
    // These directly wrap ADI driver functions
    //==========================================================================

    /**
     * @brief Get link status from hardware (SPI transaction)
     * @note For cached status without SPI, use isLinkUp() instead
     */
    bool getLinkStatus();

    /**
     * @brief Enable default device behavior (called by begin)
     * @note Internal initialization helper
     */
    adi_eth_Result_e enableDefaultBehavior();

private:
    // Private helper methods for initialization
    adi_eth_Result_e    configureMACFilters    ();
    adi_eth_Result_e    configureCallbacks     ();
    adi_eth_Result_e    configureBuffers       ();
    void                configureLEDs          ();
    adi_eth_Result_e    finalizeEnable         ();

public:
    //User callbacks
    void (*userRxCallback)(adi_eth_BufDesc_t *);
    void (*userLinkCallback)(bool connected);
    
    //static functions available to pass to underlying C driver, will regain context and call appropriate member function 
    static void         txCallback_C_Compatible (void *pCBParam, uint32_t Event, void *pArg);
    static void         rxCallback_C_Compatible (void *pCBParam, uint32_t Event, void *pArg);
    static void         linkCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg);

    //functions called on successful rx, tx, or link status chage.
    void                txCallback              (void *pCBParam, uint32_t Event, void *pArg);
    void                rxCallback              (void *pCBParam, uint32_t Event, void *pArg);
    void                linkCallback            (void *pCBParam, uint32_t Event, void *pArg);
    volatile uint32_t txResultCounters[36] = {0};
    volatile uint32_t rxResultCounters[36] = {0};
    volatile uint32_t initResultCounters[36] = {0};
    // adin2111_DeviceHandle_t getDevice(); // Duplicated, removed
};

#endif
