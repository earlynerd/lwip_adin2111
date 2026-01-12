#include "../utility/adin2111_wrap.h"
// #include <EthernetCompat.h>
// #include "LwipEthernet.h"
//  ADIN2111 allows up to 25MHz SPI
#define ADIN_SPI_SPEED 25000000
#include "PacketPool.h"

// Platform compatibility: OUTPUT_12MA is RP2040-specific
#ifndef OUTPUT_12MA
#define OUTPUT_12MA OUTPUT
#endif

ADIN2111_wrap::ADIN2111_wrap(int8_t cs, SPIClass &spi, int8_t intr)
    : _cs(cs), _intr(intr), _spi(spi), _reset(7), _cfg0(9), _cfg1(8), _settings(ADIN_SPI_SPEED, MSBFIRST, SPI_MODE0)
{
}

void ADIN2111_wrap::setPins(int8_t reset, int8_t cfg0, int8_t cfg1)
{
    _reset = reset;
    pinMode(_reset, OUTPUT_12MA);
    digitalWrite(_reset, LOW);
    _cfg0 = cfg0;
    _cfg1 = cfg1;
    pinMode(_cfg0, OUTPUT_12MA);
    pinMode(_cfg1, OUTPUT_12MA);
    pinMode(_intr, INPUT_PULLUP);
    SPI.begin();
    SPI.beginTransaction(_settings);
    SPI.transfer(0);
    SPI.endTransaction();
}

bool ADIN2111_wrap::begin(const uint8_t *mac, struct netif *netif)
{
    _netif = netif;
    if (_netif)
    {
        netif_set_link_down(_netif); // prevent lwip from sending us data before things are ready
    }
    _lastLinkState = false;
    memcpy(_mac, mac, 6);
    uint8_t retry = 0;

    // Check if packet pools allocated successfully
    if (adin2111.poolInitFailed())
    {
        Serial.println("FATAL: Packet pool memory allocation failed!");
        return false;
    }

    adin2111.setSPI(_spi);

    bool success = adin2111.begin(&retry, (uint8_t *)mac, _cs, _intr, _reset, _cfg0, _cfg1);

    uint32_t dat = 0;
    if (success)
        success = (adin2111.readRegister(1, &dat) == ADI_ETH_SUCCESS);
    Serial.print("ADIN2111 Device ID: ");
    Serial.println(dat, HEX);
    if (success)
        Serial.println("Adin init success");
    else
        Serial.println("Adin init fail");
    return success;
}

void ADIN2111_wrap::end()
{
    // Disable the device to stop packet processing
    adin2111.disable();

    // Clear the netif reference
    _netif = nullptr;
    _lastLinkState = false;
}

void ADIN2111_wrap::checkLinkStatus()
{
    // Use cached link status from interrupt callback (no SPI transaction)
    bool currentHwState = adin2111.isLinkUp();

    // Update internal tracking
    if (currentHwState != _lastLinkState)
    {
        _lastLinkState = currentHwState;

        // CRITICAL: Update lwIP core!
        // LwipIntfDev DOES NOT do this for you automatically.
        if (_netif)
        {
            if (currentHwState)
            {
                netif_set_link_up(_netif);
            }
            else
            {
                netif_set_link_down(_netif);
            }
        }
    }
}

bool ADIN2111_wrap::isLinked()
{
    return adin2111.isLinkUp();
}

uint16_t ADIN2111_wrap::sendFrame(const uint8_t *data, uint16_t len)
{
    if (_lastLinkState)
    {
        uint16_t txd = adin2111.sendFrame((uint8_t *)data, (int)len);
        return txd;
    }
    else
    {
        return 0;
    }
}

uint16_t ADIN2111_wrap::readFrameSize()
{
    // Rate-limit link status checks to avoid excessive overhead
    uint32_t now = millis();
    if ((now - _lastLinkCheckMs) >= LINK_CHECK_INTERVAL_MS) {
        _lastLinkCheckMs = now;
        checkLinkStatus();
    }

    uint16_t fsz = adin2111.getRxLength();
    return fsz;
}

uint16_t ADIN2111_wrap::readFrameData(uint8_t *buffer, uint16_t len)
{
    uint8_t rxmac[6];
    return adin2111.getRxData(buffer, len, rxmac);
}

void ADIN2111_wrap::discardFrame(uint16_t len)
{
    (void)len;  // Unused parameter
    adin2111.discardFrame();
}

void ADIN2111_wrap::printStatus()
{
    adin2111_DeviceHandle_t hdev = adin2111.getDevice();
    adi_mac_Device_t *mac = hdev->pMacDevice;
    adi_phy_Device_t *phy1 = hdev->pPhyDevice[0];
    adi_phy_Device_t *phy2 = hdev->pPhyDevice[1];
    adi_eth_MacStatCounters_t macstat1, macstat2;
    macDriverEntry.GetStatCounters(mac, ADIN2111_PORT_1, &macstat1);
    macDriverEntry.GetStatCounters(mac, ADIN2111_PORT_2, &macstat2);
    adi_phy_LinkStatus_e stat1, stat2;
    phyDriverEntry.GetLinkStatus(phy1, &stat1);
    phyDriverEntry.GetLinkStatus(phy2, &stat2);

    Serial.println("---------- ADIN2111 Status ----------");
    Serial.printf("Status 0: 0x%08X\r\n", mac->statusRegisters.status0);
    Serial.printf("Status 1: 0x%08X\r\n", mac->statusRegisters.status1);
    // Serial.println("---------- MAC OA Error Stats ----------");
    // Serial.printf("FD Count: %d\r\n", mac->oaErrorStats.fdCount);
    // Serial.printf("Footer Parity Count: %d\r\n", mac->oaErrorStats.ftrParityErrorCount);
    // Serial.printf("Header Parity Count: %d\r\n", mac->oaErrorStats.hdrParityErrorCount);
    // Serial.printf("Invalid EV Count: %d\r\n", mac->oaErrorStats.invalidEvCount);
    // Serial.printf("Invalid SV Count: %d\r\n", mac->oaErrorStats.invalidSvCount);
    // Serial.printf("Sync Error Count: %d\r\n", mac->oaErrorStats.syncErrorCount);

    Serial.println("----------P1 Status ----------");
    Serial.printf("P1 Status: 0x%08X\r\n", mac->statusRegisters.p1Status);
    if (stat1 == ADI_PHY_LINK_STATUS_UP)
    {
        Serial.print("P1 Linkstatus: UP\r\n");
        Serial.printf("RX_FRM_CNT: %d,\tTX_FRM_CNT: %d,\tRX_BCAST_CNT: %d,\tTX_BCAST_CNT: %d\r\n", macstat1.RX_FRM_CNT, macstat1.TX_FRM_CNT, macstat1.RX_BCAST_CNT, macstat1.TX_BCAST_CNT);
        Serial.printf("RX_LS_ERR_CNT: %d,\tRX_PHY_ERR_CNT: %d,\tRX_CRC_ERR_CNT: %d,\t,RX_DROP_FILT_CNT: %d,\tRX_DROP_FULL_CNT: %d\r\n", macstat1.RX_LS_ERR_CNT, macstat1.RX_PHY_ERR_CNT, macstat1.RX_CRC_ERR_CNT, macstat1.RX_DROP_FILT_CNT, macstat1.RX_DROP_FULL_CNT);
    }
    else
        Serial.print("P1 Linkstatus: DOWN\r\n");

    adi_phy_State_e p1state = phy1->state;
    Serial.print("P1 Phy state: ");
    printPhyState(p1state);

    Serial.println("----------P2 Status ----------");
    Serial.printf("P2 Status: 0x%08X\r\n", mac->statusRegisters.p2Status);
    if (stat2 == ADI_PHY_LINK_STATUS_UP)
    {
        Serial.print("P2 Linkstatus: UP\r\n");
        Serial.printf("RX_FRM_CNT: %d,\tTX_FRM_CNT: %d,\tRX_BCAST_CNT: %d,\tTX_BCAST_CNT: %d\r\n", macstat2.RX_FRM_CNT, macstat2.TX_FRM_CNT, macstat2.RX_BCAST_CNT, macstat2.TX_BCAST_CNT);
        Serial.printf("RX_LS_ERR_CNT: %d,\tRX_PHY_ERR_CNT: %d,\tRX_CRC_ERR_CNT: %d,\tRX_DROP_FILT_CNT: %d,\tRX_DROP_FULL_CNT: %d\r\n", macstat2.RX_LS_ERR_CNT, macstat2.RX_PHY_ERR_CNT, macstat2.RX_CRC_ERR_CNT, macstat2.RX_DROP_FILT_CNT, macstat2.RX_DROP_FULL_CNT);
    }
    else
        Serial.print("P2 Linkstatus: DOWN\r\n");

    adi_phy_State_e p2state = phy2->state;
    Serial.print("P2 Phy state: ");
    printPhyState(p2state);
    Serial.print("init result counters:");
    if (!printResultCounters((uint32_t *)&adin2111.initResultCounters[0]))
        Serial.println(" no errors.");
    Serial.print("rx result counters:");
    if (!printResultCounters((uint32_t *)&adin2111.rxResultCounters[0]))
        Serial.println(" no errors.");
    Serial.print("tx result counters:");
    if (!printResultCounters((uint32_t *)&adin2111.txResultCounters[0]))
        Serial.println(" no errors.");
    uint32_t dropped = adin2111.getRxDroppedCount();
    if (dropped > 0)
        Serial.printf("\r\nRX packets dropped (pool exhausted): %lu", dropped);
    Serial.print("\r\n");
}

void ADIN2111_wrap::printPhyState(adi_phy_State_e s)
{
    switch (s)
    {
    case ADI_PHY_STATE_UNINITIALIZED:
        Serial.println("ADI_PHY_STATE_UNINITIALIZED");
        break;

    case ADI_PHY_STATE_HW_RESET:
        Serial.println("ADI_PHY_STATE_HW_RESET");
        break;

    case ADI_PHY_STATE_SOFTWARE_POWERDOWN:
        Serial.println("ADI_PHY_STATE_SOFTWARE_POWERDOWN");
        break;

    case ADI_PHY_STATE_OPERATION:
        Serial.println("ADI_PHY_STATE_OPERATION");
        break;

    case ADI_PHY_STATE_DIAGNOSTIC:
        Serial.println("ADI_PHY_STATE_DIAGNOSTIC");
        break;

    case ADI_PHY_STATE_ERROR:
        Serial.println("ADI_PHY_STATE_ERROR");
        break;
    }
}

bool ADIN2111_wrap::printResultCounters(uint32_t *counters)
{
    bool errors = false;
    const char *resultNames[] = {
        "ADI_ETH_SUCCESS",
        "ADI_ETH_MDIO_TIMEOUT",              /*!< MDIO timeout.                                              */
        "ADI_ETH_COMM_ERROR",                /*!< Communication error.                                       */
        "ADI_ETH_COMM_ERROR_SECOND",         /*!< Communication error.                                       */
        "ADI_ETH_COMM_TIMEOUT",              /*!< Communications timeout with the host.                      */
        "ADI_ETH_UNSUPPORTED_DEVICE",        /*!< Unsupported device.                                        */
        "ADI_ETH_DEVICE_UNINITIALIZED",      /*!< Device not initialized.                                    */
        "ADI_ETH_HW_ERROR",                  /*!< Hardware error.                                            */
        "ADI_ETH_INVALID_PARAM",             /*!< Invalid parameter.                                         */
        "ADI_ETH_PARAM_OUT_OF_RANGE",        /*!< Parameter out of range.                                    */
        "ADI_ETH_INVALID_HANDLE",            /*!< Invalid device handle.                                     */
        "ADI_ETH_IRQ_PENDING",               /*!< Interrupt request is pending.                              */
        "ADI_ETH_READ_STATUS_TIMEOUT",       /*!< Timeout when reading status registers.                     */
        "ADI_ETH_INVALID_POWER_STATE",       /*!< Invalid power state.                                       */
        "ADI_ETH_HAL_INIT_ERROR",            /*!< HAL initialization error.                                  */
        "ADI_ETH_INSUFFICIENT_FIFO_SPACE",   /*!< Insufficient TxFIFO space when trying to write a frame.    */
        "ADI_ETH_CRC_ERROR",                 /*!< SPI integrity check failure (generic SPI).                 */
        "ADI_ETH_PROTECTION_ERROR",          /*!< SPI integrity check failure (OPEN Alliance SPI).           */
        "ADI_ETH_QUEUE_FULL",                /*!< Transmit queue is full.                                    */
        "ADI_ETH_QUEUE_EMPTY",               /*!< Receive queue is empty.                                    */
        "ADI_ETH_BUFFER_TOO_SMALL",          /*!< Buffer is too small for received data.                     */
        "ADI_ETH_INVALID_PORT",              /*!< Invalid port value.                                        */
        "ADI_ETH_ADDRESS_FILTER_TABLE_FULL", /*!< Address filter table is full.                              */
        "ADI_ETH_MAC_BUSY",                  /*!< MAC is busy.                                               */
        "ADI_ETH_COMM_BUSY",                 /*!< SPI communication busy.                                    */
        "ADI_ETH_SPI_ERROR",                 /*!< SPI error.                                                 */
        "ADI_ETH_SW_RESET_TIMEOUT",          /*!< Software reset timeout.                                    */
        "ADI_ETH_CONFIG_SYNC_ERROR",         /*!< Configuration change attempted after configuration sync.   */
        "ADI_ETH_VALUE_MISMATCH_ERROR",      /*!< Value does not match expected value.                       */
        "ADI_ETH_FIFO_SIZE_ERROR",           /*!< Desired FIFO size exceeds 28k byte limit.                  */
        "ADI_ETH_TS_COUNTERS_DISABLED",      /*!< Timestamp counters are not enabled.                        */
        "ADI_ETH_NO_TS_FORMAT",              /*!< No timstamp format selected or timestamps captured.        */
        "ADI_ETH_NOT_IMPLEMENTED",           /*!< Not implemented in hardware.                               */
        "ADI_ETH_NOT_IMPLEMENTED_SOFTWARE",  /*!< Not implemented in software.                               */
        "ADI_ETH_UNSUPPORTED_FEATURE",       /*!< Hardware feature not supported by the software driver.     */
        "ADI_ETH_PLACEHOLDER_ERROR"};

    for (int i = 1; i < 35; i++)
    {
        if (counters[i])
        {
            Serial.printf("\r\n%s: %d", resultNames[i], counters[i]);
            errors = true;
        }
    }
    return errors;
}