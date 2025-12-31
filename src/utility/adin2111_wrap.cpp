#include "../utility/adin2111_wrap.h"
//#include <EthernetCompat.h>
//#include "LwipEthernet.h"
// ADIN2111 allows up to 25MHz SPI
#define ADIN_SPI_SPEED 25000000
#include "PacketPool.h"




ADIN2111_wrap::ADIN2111_wrap(int8_t cs, SPIClass &spi, int8_t intr)
    : _cs(cs), _intr(intr), _spi(spi), _settings(ADIN_SPI_SPEED, MSBFIRST, SPI_MODE0)
{
}

bool ADIN2111_wrap::begin(const uint8_t *mac, struct netif *netif)
{
    _netif = netif;
    if (_netif) {
            netif_set_link_down(_netif);
        }
    _lastLinkState = false;
    memcpy(_mac, mac, 6);
    uint8_t retry = 0;
    //adin2111.setRxCallback(rxcallback);
    bool success = adin2111.begin(&retry, (uint8_t*)mac, _cs);
    
    uint32_t dat = 0;
    if(success) success = (adin2111.readRegister(1, &dat) == ADI_ETH_SUCCESS);
    Serial.print("ADIN2111 Device ID: ");
    Serial.println(dat, HEX);
    if(success) Serial.println("Adin init success");
    else Serial.println("Adin init fail");
   return success;
    // return (readReg(ADIN_CONFIG0) & ADIN_CONFIG0_SYNC);
}

void ADIN2111_wrap::end()
{
    // uint32_t cfg = readReg(ADIN_CONFIG0);
    // cfg &= ~(ADIN_CONFIG0_TXE | ADIN_CONFIG0_RXE);
    // writeReg(ADIN_CONFIG0, cfg);
    //_spi.end();
}

void ADIN2111_wrap::checkLinkStatus() {
        // Query hardware (ADIN2111 register)
        bool currentHwState =  adin2111.getLinkStatus();

        // Update internal tracking
        if (currentHwState != _lastLinkState) {
            _lastLinkState = currentHwState;
            
            // CRITICAL: Update lwIP core!
            // LwipIntfDev DOES NOT do this for you automatically.
            if (_netif) {
                if (currentHwState) {
                    netif_set_link_up(_netif);
                } else {
                    netif_set_link_down(_netif);
                }
            }
        }
    }

bool ADIN2111_wrap::isLinked()
{
    return _lastLinkState;
}

uint16_t ADIN2111_wrap::sendFrame(const uint8_t *data, uint16_t len)
{
    if(_lastLinkState)
    {
    //Serial.printf("[ADIN] TX Frame, len %d...", len);
    //ethernet_arch_lwip_gpio_mask(); 
    uint16_t txd = adin2111.sendFrame((uint8_t*)data, (int)len);
    //ethernet_arch_lwip_gpio_unmask();
    //if(txd == len)Serial.println(" Success.");
    //else if(txd > 0) Serial.println("partial");
    //else Serial.println(" Fail...");
        
    return txd;
    }
    else
    {
        //Serial.println("TX frame before link up, dropping...");
        return 0;
    }
   
}

uint16_t ADIN2111_wrap::readFrameSize()
{
    checkLinkStatus();
    uint16_t fsz = adin2111.getRxLength();
    //Serial.printf("[ADIN] check frame size: %d\r\n", fsz);
    return  fsz;
}

uint16_t ADIN2111_wrap::readFrameData(uint8_t *buffer, uint16_t len)
{
    uint8_t rxmac[6];
    return adin2111.getRxData(buffer, len, rxmac);
}

void ADIN2111_wrap::discardFrame(uint16_t len)
{
    adin2111.discardFrame();
    //uint8_t buf[SPE_MAX_BUF_FRAME_SIZE];            
    //uint8_t rxmac[6];
    //adin2111.getRxData(buf, len, rxmac);
}

void ADIN2111_wrap::printStatus()
{
    adin2111_DeviceHandle_t hdev = adin2111.getDevice();
    adi_mac_Device_t* mac = hdev->pMacDevice;
    adi_phy_Device_t* phy1 = hdev->pPhyDevice[0];
    adi_phy_Device_t* phy2 = hdev->pPhyDevice[1];
    adi_eth_MacStatCounters_t macstat1, macstat2;
    macDriverEntry.GetStatCounters(mac, ADIN2111_PORT_1, &macstat1);
    macDriverEntry.GetStatCounters(mac, ADIN2111_PORT_2, &macstat2);
    Serial.println("---------- MAC Status ----------");
    Serial.printf("P1 Status: 0x%08X\r\n", mac->statusRegisters.p1Status);
    Serial.printf("P2 Status: 0x%08X\r\n", mac->statusRegisters.p2Status);
    Serial.printf("Status 0: 0x%08X\r\n", mac->statusRegisters.status0);
    Serial.printf("Status 1: 0x%08X\r\n", mac->statusRegisters.status1);
    Serial.println("---------- MAC OA Error Stats ----------");
    Serial.printf("FD Count: %d\r\n", mac->oaErrorStats.fdCount);
    Serial.printf("Footer Parity Count: %d\r\n", mac->oaErrorStats.ftrParityErrorCount);
    Serial.printf("Header Parity Count: %d\r\n", mac->oaErrorStats.hdrParityErrorCount);
    Serial.printf("Invalid EV Count: %d\r\n", mac->oaErrorStats.invalidEvCount);
    Serial.printf("Invalid SV Count: %d\r\n", mac->oaErrorStats.invalidSvCount);
    Serial.printf("Sync Error Count: %d\r\n", mac->oaErrorStats.syncErrorCount);

    Serial.println("---------- PHY Status ----------");
    adi_phy_LinkStatus_e stat1, stat2;
    phyDriverEntry.GetLinkStatus(phy1, &stat1);
    phyDriverEntry.GetLinkStatus(phy2, &stat2);
    if(stat1 == ADI_PHY_LINK_STATUS_UP) Serial.print("P1 PHY Linkstatus: UP\r\n");
    else Serial.print("P1 PHY Linkstatus: DOWN\r\n");
    
    adi_phy_State_e p1state = phy1->state;
    Serial.print("P1 Phy state: ");
    printPhyState(p1state);

    if(stat2 == ADI_PHY_LINK_STATUS_UP) Serial.print("P2 PHY Linkstatus: UP\r\n");
    else Serial.print("P2 PHY Linkstatus: DOWN\r\n");

    adi_phy_State_e p2state = phy2->state;
    Serial.print("P2 Phy state: ");
    printPhyState(p2state);

    Serial.println("\r\n");
}

void ADIN2111_wrap::printPhyState(adi_phy_State_e s)
{
    switch(s)
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