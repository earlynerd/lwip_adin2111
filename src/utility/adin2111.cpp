#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <string.h>
#include "adin2111.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

// CRC-8 table for polynomial 0x7
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    // ... full table generated for polynomial 0x7
    // Can be generated using no_os_crc8_populate_msb
};

ADIN2111::ADIN2111(int8_t cs, SPIClass& spi, int8_t intr, int8_t reset) : 
    _cs(cs), _spi(spi), _intr(intr), _reset(reset), _appendCrc(true)
{
    memset(_buff, 0, sizeof(_buff));
}

bool ADIN2111::begin(const uint8_t* macAddress, netif* net)
{
    PRINTF("ADIN2111: Initializing\n");
    
    _macAddress = macAddress;
    _netif = net;
    
    // Configure pins
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    _spi.begin();

    // Hardware reset if pin provided
    if(_reset >= 0) {
        PRINTF("ADIN2111: Performing hardware reset\n");
        pinMode(_reset, OUTPUT);
        digitalWrite(_reset, LOW);
        delay(10);
        digitalWrite(_reset, HIGH);
        delay(90);  // Wait for internal initialization
    }

    // Software reset sequence
    PRINTF("ADIN2111: Performing software reset\n");
    if(doReset() != 0) {
        PRINTF("ADIN2111: Reset failed\n");
        return false;
    }

    // Clear any stale state
    if(regWrite(ADIN2111_FIFO_CLR_REG, 
                ADIN2111_FIFO_CLR_RX_MASK | ADIN2111_FIFO_CLR_TX_MASK) != 0) {
        PRINTF("ADIN2111: Failed to clear FIFOs\n");
        return false;
    }

    // Configure PHY
    PRINTF("ADIN2111: Setting up PHY\n");
    if(setupPhy() != 0) {
        PRINTF("ADIN2111: PHY setup failed\n");
        return false;
    }

    // Configure MAC
    PRINTF("ADIN2111: Setting up MAC\n");
    if(setupMac() != 0) {
        PRINTF("ADIN2111: MAC setup failed\n");
        return false;
    }

    // Set MAC address filter
    PRINTF("ADIN2111: Setting MAC address\n");
    uint32_t addr_upr = (_macAddress[0] << 8) | _macAddress[1];
    uint32_t addr_lwr = (_macAddress[2] << 24) | (_macAddress[3] << 16) | 
                       (_macAddress[4] << 8) | _macAddress[5];
    
    addr_upr |= ADIN2111_MAC_ADDR_APPLY2PORT | ADIN2111_MAC_ADDR_TO_HOST;

    if(regWrite(ADIN2111_MAC_ADDR_FILT_UPR_REG(ADIN_MAC_P1_ADDR_SLOT), addr_upr) != 0 ||
       regWrite(ADIN2111_MAC_ADDR_FILT_LWR_REG(ADIN_MAC_P1_ADDR_SLOT), addr_lwr) != 0) {
        PRINTF("ADIN2111: Failed to set MAC address\n");
        return false;
    }

    // Enable broadcast filter
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    addr_upr = (broadcast[0] << 8) | broadcast[1];
    addr_lwr = (broadcast[2] << 24) | (broadcast[3] << 16) | 
               (broadcast[4] << 8) | broadcast[5];
    
    addr_upr |= ADIN2111_MAC_ADDR_APPLY2PORT | ADIN2111_MAC_ADDR_TO_HOST;

    if(regWrite(ADIN2111_MAC_ADDR_FILT_UPR_REG(ADIN_MAC_BROADCAST_ADDR_SLOT), addr_upr) != 0 ||
       regWrite(ADIN2111_MAC_ADDR_FILT_LWR_REG(ADIN_MAC_BROADCAST_ADDR_SLOT), addr_lwr) != 0) {
        PRINTF("ADIN2111: Failed to set broadcast filter\n");
        return false;
    }

    // Setup interrupt if pin specified
    if(_intr >= 0) {
        PRINTF("ADIN2111: Configuring interrupts\n");
        uint32_t irq_mask = ADIN2111_TX_RDY_IRQ | ADIN2111_RX_RDY_IRQ | ADIN2111_SPI_ERR_IRQ;
        if(regWrite(ADIN2111_IMASK1_REG, ~irq_mask) != 0) {
            PRINTF("ADIN2111: Failed to configure interrupts\n");
            return false;
        }
        pinMode(_intr, INPUT);
    }

    // Wait for link (optional but helps with initial setup)
    PRINTF("ADIN2111: Waiting for link\n");
    int timeout = 100;  // 1 second timeout
    while(!isLinked() && timeout > 0) {
        delay(10);
        timeout--;
    }
    if(timeout == 0) {
        PRINTF("ADIN2111: Link timeout\n");
        // Continue anyway - link might come up later
    }

    PRINTF("ADIN2111: Initialization complete\n");
    return true;
}

void ADIN2111::end()
{
    PRINTF("ADIN2111: Shutting down\n");
    
    // Disable all interrupts
    regWrite(ADIN2111_IMASK1_REG, 0xffffffff);

    // Clear FIFOs
    regWrite(ADIN2111_FIFO_CLR_REG, 
             ADIN2111_FIFO_CLR_RX_MASK | ADIN2111_FIFO_CLR_TX_MASK);

    // Software reset
    regWrite(ADIN2111_RESET_REG, 0x1);
}

uint16_t ADIN2111::sendFrame(const uint8_t* data, uint16_t datalen)
{
    uint32_t header_len = ADIN2111_WR_HDR_SIZE;
    uint32_t frame_offset;
    uint32_t padding = 0;
    
    PRINTF("ADIN2111: Sending frame, length %d\n", datalen);

    // Check minimum frame size (64 bytes including FCS)
    if (datalen + 4 < 64) {
        padding = 64 - (datalen + 4);
    }

    uint32_t padded_len = datalen + padding + ADIN2111_FRAME_HEADER_LEN;
    
    // Align to 4 bytes 
    uint32_t round_len = (padded_len + 3) & ~3;

    // Check TX buffer space
    uint32_t space;
    if(regRead(ADIN2111_TX_SPACE_REG, &space) != 0) {
        PRINTF("ADIN2111: Failed to read TX space\n");
        return 0;
    }

    if(padded_len > 2 * (space - ADIN2111_FRAME_HEADER_LEN)) {
        PRINTF("ADIN2111: TX buffer full\n");
        return 0;
    }

    // Set frame size
    if(regWrite(ADIN2111_TX_FSIZE_REG, padded_len) != 0) {
        PRINTF("ADIN2111: Failed to set frame size\n");
        return 0;
    }

    // Format TX header
    uint16_t addr = ADIN2111_TX_REG;
    addr |= ADIN2111_SPI_CD | ADIN2111_SPI_RW;
    
    _buff[0] = addr >> 8;
    _buff[1] = addr & 0xFF;

    if(_appendCrc) {
        _buff[2] = calculateCRC8(_buff, 2);
        header_len++;
    }

    // Port 0 for first port
    uint16_t port = 0;
    _buff[header_len] = port >> 8;
    _buff[header_len + 1] = port & 0xFF;
    
    frame_offset = header_len + ADIN2111_FRAME_HEADER_LEN;
    
    // Copy frame data
    memcpy(&_buff[frame_offset], data, datalen);

    // Add padding if needed
    if(padding > 0) {
        memset(&_buff[frame_offset + datalen], 0, padding);
    }

    // Write to device
    _spi.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer(_buff, round_len + header_len);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    PRINTF("ADIN2111: Frame sent\n");
    return datalen;
}

uint16_t ADIN2111::readFrame(uint8_t* buffer, uint16_t bufsize)
{
    uint16_t size = readFrameSize();
    if(size == 0) return 0;
    
    return readFrameData(buffer, bufsize);
}

uint16_t ADIN2111::readFrameSize()
{
    uint32_t size;
    
    if(regRead(ADIN2111_RX_FSIZE_REG, &size) != 0) {
        PRINTF("ADIN2111: Failed to read frame size\n");
        return 0;
    }
    
    if(size < (ADIN2111_FRAME_HEADER_LEN + 4)) {
        return 0;  // No valid frame
    }

    _frameSize = size;
    return size - ADIN2111_FRAME_HEADER_LEN;
}

//[Previous register access and helper methods remain the same...]

// New helper method - checks and waits for TX ready
bool ADIN2111::waitTxReady(uint32_t timeout_ms)
{
    uint32_t status;
    uint32_t start = millis();
    
    while((millis() - start) < timeout_ms) {
        if(regRead(ADIN2111_STATUS1_REG, &status) == 0) {
            if(status & ADIN2111_TX_RDY_IRQ) {
                return true;
            }
        }
        delay(1);
    }
    return false;
}

// New helper method - checks for RX data availability
bool ADIN2111::hasRxData()
{
    uint32_t status;
    if(regRead(ADIN2111_STATUS1_REG, &status) == 0) {
        return (status & ADIN2111_RX_RDY) != 0;
    }
    return false;
}

// Private helper methods for ADIN2111 driver

int ADIN2111::regWrite(uint16_t addr, uint32_t data)
{
    uint32_t header_len = ADIN2111_WR_HDR_SIZE;
    
    // Format register write header
    addr &= ADIN2111_ADDR_MASK;
    addr |= ADIN2111_CD_MASK | ADIN2111_RW_MASK;
    _buff[0] = addr >> 8;
    _buff[1] = addr & 0xFF;

    if(_appendCrc) {
        _buff[2] = calculateCRC8(_buff, 2);
        header_len++;
    }

    // Add data in big-endian format
    _buff[header_len + 0] = (data >> 24) & 0xFF;
    _buff[header_len + 1] = (data >> 16) & 0xFF;
    _buff[header_len + 2] = (data >> 8) & 0xFF;
    _buff[header_len + 3] = data & 0xFF;

    uint32_t len = header_len + ADIN2111_REG_LEN;

    if(_appendCrc) {
        _buff[len] = calculateCRC8(&_buff[header_len], ADIN2111_REG_LEN);
        len++;
    }

    _spi.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer(_buff, len);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    return 0;
}

int ADIN2111::regRead(uint16_t addr, uint32_t* data)
{
    uint32_t header_len = ADIN2111_RD_HDR_SIZE;

    // Format register read header
    addr &= ADIN2111_ADDR_MASK;
    addr |= ADIN2111_CD_MASK;  // No RW bit for reads
    _buff[0] = addr >> 8;
    _buff[1] = addr & 0xFF;
    _buff[2] = 0;

    if(_appendCrc) {
        _buff[2] = calculateCRC8(_buff, 2);
        _buff[3] = 0;
        header_len++;
    }

    uint32_t len = header_len + ADIN2111_REG_LEN;
    if(_appendCrc) {
        len += ADIN2111_CRC_LEN;
    }

    _spi.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer(_buff, len);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    if(_appendCrc) {
        uint8_t crc = calculateCRC8(&_buff[header_len], ADIN2111_REG_LEN);
        uint8_t recv_crc = _buff[header_len + ADIN2111_REG_LEN];
        if(crc != recv_crc) {
            return -1; // CRC error
        }
    }

    // Extract data in big-endian format
    *data = ((uint32_t)_buff[header_len] << 24) |
            ((uint32_t)_buff[header_len + 1] << 16) |
            ((uint32_t)_buff[header_len + 2] << 8) |
            ((uint32_t)_buff[header_len + 3]);

    return 0;
}

int ADIN2111::regUpdate(uint16_t addr, uint32_t mask, uint32_t data)
{
    uint32_t val;
    int ret = regRead(addr, &val);
    if(ret != 0) return ret;

    val &= ~mask;
    val |= (data & mask);

    return regWrite(addr, val);
}

int ADIN2111::mdioWrite(uint32_t phy_id, uint32_t reg, uint16_t data)
{
    uint32_t mdio_done = 0;
    uint32_t mdio_val;

    // Format MDIO write command
    uint32_t val = (0x1 << 29) |                    // Start of frame
                   (ADIN2111_MDIO_OP_WR << 26) |    // Write operation
                   ((phy_id & 0x1F) << 21) |        // PHY address
                   ((reg & 0x1F) << 16) |           // Register address
                   (data & 0xFFFF);                 // Data

    int ret = regWrite(ADIN2111_MDIOACC(0), val);
    if(ret != 0) return ret;

    // Wait for completion
    while(!mdio_done) {
        ret = regRead(ADIN2111_MDIOACC(0), &mdio_val);
        if(ret != 0) return ret;
        mdio_done = (mdio_val >> 31) & 0x1;  // Check TRDONE bit
    }

    return 0;
}

int ADIN2111::mdioRead(uint32_t phy_id, uint32_t reg, uint16_t* data)
{
    uint32_t mdio_done = 0;
    uint32_t mdio_val;

    // Format MDIO read command
    uint32_t val = (0x1 << 29) |                    // Start of frame
                   (ADIN2111_MDIO_OP_RD << 26) |    // Read operation
                   ((phy_id & 0x1F) << 21) |        // PHY address
                   ((reg & 0x1F) << 16);            // Register address

    int ret = regWrite(ADIN2111_MDIOACC(0), val);
    if(ret != 0) return ret;

    // Wait for completion
    while(!mdio_done) {
        ret = regRead(ADIN2111_MDIOACC(0), &mdio_val);
        if(ret != 0) return ret;
        mdio_done = (mdio_val >> 31) & 0x1;  // Check TRDONE bit
    }

    *data = mdio_val & 0xFFFF;  // Extract data portion
    return 0;
}

int ADIN2111::doReset()
{
    // Software reset sequence
    int ret = regWrite(ADIN2111_RESET_REG, 0x1);
    if(ret != 0) return ret;

    delay(1);  // Wait for reset to complete

    // Check reset status
    uint32_t val;
    ret = regRead(ADIN2111_STATUS0_REG, &val);
    if(ret != 0) return ret;

    if(!(val & ADIN2111_RESETC_MASK)) {
        return -1;  // Reset failed
    }

    return regUpdate(ADIN2111_CONFIG1_REG, ADIN2111_CONFIG1_SYNC, ADIN2111_CONFIG1_SYNC);
}

int ADIN2111::setupMac()
{
    // Enable CRC appending
    int ret = regUpdate(ADIN2111_CONFIG2_REG, ADIN2111_CRC_APPEND, ADIN2111_CRC_APPEND);
    if(ret != 0) return ret;

    // Configure interrupts if enabled
    if(_intr >= 0) {
        uint32_t irq_mask = ADIN2111_TX_RDY_IRQ | ADIN2111_RX_RDY_IRQ | ADIN2111_SPI_ERR_IRQ;
        ret = regWrite(ADIN2111_IMASK1_REG, ~irq_mask);
        if(ret != 0) return ret;
    }

    return 0;
}

int ADIN2111::setupPhy()
{
    // Check PHY ID
    uint16_t id_low, id_high;
    int ret = mdioRead(ADIN2111_MDIO_PHY_ID(0), 2, &id_high);
    if(ret != 0) return ret;
    
    ret = mdioRead(ADIN2111_MDIO_PHY_ID(0), 3, &id_low);
    if(ret != 0) return ret;

    uint32_t phy_id = ((uint32_t)id_high << 16) | id_low;
    if(phy_id != ADIN2111_PHY_ID) {
        return -1;  // Invalid PHY ID
    }

    // Get PHY out of software power down
    uint16_t ctrl;
    ret = mdioRead(ADIN2111_MDIO_PHY_ID(0), 0, &ctrl);
    if(ret != 0) return ret;

    if(ctrl & ADIN2111_MI_SFT_PD_MASK) {
        ctrl &= ~ADIN2111_MI_SFT_PD_MASK;
        ret = mdioWrite(ADIN2111_MDIO_PHY_ID(0), 0, ctrl);
        if(ret != 0) return ret;
    }

    return 0;
}

uint8_t ADIN2111::calculateCRC8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    
    while(len--) {
        crc = crc8_table[crc ^ *data++];
    }
    
    return crc;
}

void ADIN2111::discardFrame(uint16_t framesize) {
    // Read and discard the frame 
    readFrameData(nullptr, 0);
}

uint16_t ADIN2111::readFrameData(uint8_t* buffer, uint16_t framesize) {
    if(framesize == 0 || buffer == nullptr) {
        // Just discard the frame
        uint32_t val;
        regWrite(ADIN2111_FIFO_CLR_REG, ADIN2111_FIFO_CLR_RX_MASK);
        return 0;
    }

    uint32_t header_len = ADIN2111_RD_HDR_SIZE;
    uint16_t addr = ADIN2111_RX_REG | ADIN2111_SPI_CD;
    
    _buff[0] = addr >> 8;
    _buff[1] = addr & 0xFF;
    _buff[2] = 0;

    if(_appendCrc) {
        _buff[2] = calculateCRC8(_buff, 2);
        _buff[3] = 0;
        header_len++;
    }

    // Port 0 for first port
    uint16_t port = 0;
    _buff[header_len] = port >> 8;
    _buff[header_len + 1] = port & 0xFF;

    // Round to 4 byte boundary
    uint32_t round_len = (_frameSize + 3) & ~3;

    _spi.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    
    // Send header
    _spi.transfer(_buff, header_len + ADIN2111_FRAME_HEADER_LEN);
    
    // Read frame data
    _spi.transfer(buffer, framesize);
    
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    return framesize;
}

bool ADIN2111::isLinked() {
    uint32_t status;
    if(regRead(ADIN2111_STATUS1_REG, &status) == 0) {
        return (status & ADIN2111_STATUS1_LINK_STATE_MASK) != 0;
    }
    return false;
}