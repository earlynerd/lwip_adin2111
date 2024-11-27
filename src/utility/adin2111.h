/**
 * ADIN2111 Ethernet Controller Driver for Arduino
 */

#ifndef ADIN2111_H
#define ADIN2111_H

#include <Arduino.h>
#include <SPI.h>

// ADIN2111 registers - merging ADI's defines with our own
#define ADIN2111_FIFO_CLR_REG            0x36
#define ADIN2111_FIFO_CLR_RX_MASK        (1 << 0)
#define ADIN2111_FIFO_CLR_TX_MASK        (1 << 1)

// Status and configuration
#define ADIN2111_STATUS0_REG             0x08
#define ADIN2111_STATUS0_RESETC_MASK     (1 << 6)
#define ADIN2111_STATUS0_TXPE_MASK       (1 << 0)

#define ADIN2111_STATUS1_REG             0x09
#define ADIN2111_STATUS1_LINK_STATE_MASK (1 << 0)
#define ADIN2111_STATUS1_SPI_ERR         (1 << 10)
#define ADIN2111_STATUS1_RX_RDY          (1 << 4)

#define ADIN2111_CONFIG1_REG             0x04
#define ADIN2111_CONFIG1_SYNC            (1 << 15)

#define ADIN2111_CONFIG2_REG             0x06
#define ADIN2111_CONFIG2_CRC_APPEND      (1 << 5)

// Reset related
#define ADIN2111_RESET_REG               0x03
#define ADIN2111_SOFT_RST_REG            0x3C
#define ADIN2111_SWRESET_KEY1            0x4F1C
#define ADIN2111_SWRESET_KEY2            0xC1F4
#define ADIN2111_SWRELEASE_KEY1          0x6F1A
#define ADIN2111_SWRELEASE_KEY2          0xA1F6

// MAC address configuration
#define ADIN2111_MAC_RST_STATUS_REG      0x3B
#define ADIN2111_MAC_ADDR_FILT_UPR_REG(x) (0x50 + 2 * (x))
#define ADIN2111_MAC_ADDR_FILT_LWR_REG(x) (0x51 + 2 * (x))

// MAC address filter slots
#define ADIN_MAC_P1_ADDR_SLOT            2    // Port 1 MAC address slot
#define ADIN_MAC_BROADCAST_ADDR_SLOT      1    // Broadcast address filter slot

// Frame control
#define ADIN2111_CRC_APPEND              (1 << 5)    // Append CRC to outgoing frames

// TX/RX control registers
#define ADIN2111_TX_FSIZE_REG            0x30
#define ADIN2111_TX_REG                  0x31
#define ADIN2111_TX_SPACE_REG            0x32

#define ADIN2111_RX_FSIZE_REG            0x90
#define ADIN2111_RX_REG                  0x91

// Interrupt masks
#define ADIN2111_IMASK1_REG              0x0D
#define ADIN2111_TX_RDY_IRQ              (1 << 3)
#define ADIN2111_RX_RDY_IRQ              (1 << 4)
#define ADIN2111_SPI_ERR_IRQ             (1 << 10)

// Frame size and header constants
#define ADIN2111_FRAME_HEADER_LEN        2
#define ADIN2111_WR_HEADER_LEN           2
#define ADIN2111_RD_HEADER_LEN           3
#define ADIN2111_WR_HDR_SIZE             2
#define ADIN2111_RD_HDR_SIZE             3
#define ADIN2111_REG_LEN                 4
#define ADIN2111_CRC_LEN                 1

// MAC address slots
#define ADIN2111_MAC_MULTICAST_ADDR_SLOT 0
#define ADIN2111_MAC_BROADCAST_ADDR_SLOT 1
#define ADIN2111_MAC_P1_ADDR_SLOT        2
#define ADIN2111_MAC_P2_ADDR_SLOT        3
#define ADIN2111_MAC_FDB_ADDR_SLOT       4


// MAC address control bits
#define ADIN2111_MAC_ADDR_APPLY2PORT     (1 << 30)  // Apply filter to port
#define ADIN2111_MAC_ADDR_TO_HOST        (1 << 16)  // Forward matching frames to host

// SPI protocol bits
#define ADIN2111_SPI_CD                  (1 << 7)   // Control/Data bit
#define ADIN2111_SPI_RW                  (1 << 5)   // Read/Write bit
#define ADIN2111_ADDR_MASK               0x1FFF     // Address field mask
#define ADIN2111_CD_MASK                 (1 << 15)  // Control/Data mask in register addr
#define ADIN2111_RW_MASK                 (1 << 13)  // Read/Write mask in register addr

// MDIO access
#define ADIN2111_MDIOACC(x)              (0x20 + (x))  // MDIO access register base
#define ADIN2111_MDIO_TRDONE             (1 << 31)     // Transaction done
#define ADIN2111_MDIO_TAERR              (1 << 30)     // Transaction error
#define ADIN2111_MDIO_ST                 (0x3 << 28)   // Start of frame
#define ADIN2111_MDIO_OP_WR              0x1          // Write operation
#define ADIN2111_MDIO_OP_RD              0x3          // Read operation
#define ADIN2111_MDIO_PRTAD              (0x1F << 21) // Port address field
#define ADIN2111_MDIO_DEVAD              (0x1F << 16) // Device address field
#define ADIN2111_MDIO_DATA               0xFFFF       // Data field

// PHY control
#define ADIN2111_MI_SFT_PD_MASK          (1 << 11)    // Software power down
#define ADIN2111_MDIO_PHY_ID(x)          ((x) + 1)    // PHY ID calculation

// Status bits
#define ADIN2111_RX_RDY                  (1 << 4)     // RX ready status bit
#define ADIN2111_RESETC_MASK             (1 << 6)     // Reset complete mask

// Device-specific constants
#define ADIN2111_MAX_FRAME_SIZE 1518
#define ADIN2111_MIN_FRAME_SIZE 64
#define ADIN2111_BUFF_SIZE 1530

// Register definitions from ADI driver that we need in Arduino context
#define ADIN2111_PHY_ID              0x0283BCA1

// Status register 1 bits
#define ADIN2111_STS1_TX_RDY        (1 << 3)
#define ADIN2111_STS1_RX_RDY        (1 << 4)
#define ADIN2111_STS1_LINK_UP       (1 << 0)

// Configuration register bits
#define ADIN2111_CR1_SYNC           (1 << 15)
#define ADIN2111_CR1_TX_EN          (1 << 0)
#define ADIN2111_CR1_RX_EN          (1 << 1)

// MAC Address filter masks
#define ADIN2111_MAC_FILT_PORT      (1 << 30)
#define ADIN2111_MAC_FILT_HOST      (1 << 16)

// SPI protocol constants
#define ADIN2111_SPI_READ           0x00
#define ADIN2111_SPI_WRITE          (1 << 5)
#define ADIN2111_SPI_CTRL_DATA      (1 << 7)

// Field access macros (if not defined elsewhere)
#ifndef FIELD_GET
#define FIELD_GET(mask, reg)      (((reg) & (mask)) >> __builtin_ctz(mask))
#endif

#ifndef FIELD_PREP
#define FIELD_PREP(mask, val)     (((val) << __builtin_ctz(mask)) & (mask))
#endif

// Genmask helpers if needed
#ifndef GENMASK
#define GENMASK(h, l) (((~0UL) << (l)) & (~0UL >> (31 - (h))))
#endif

// Key registers not defined in ADI driver
#define ADIN2111_REG_ID             0x00
#define ADIN2111_REG_STATUS         0x01
#define ADIN2111_REG_CONFIG         0x02
#define ADIN2111_REG_IRQ_CFG        0x03
#define ADIN2111_REG_MAC_CFG        0x04
#define ADIN2111_REG_MAC_ADDR_0     0x05
#define ADIN2111_REG_MAC_ADDR_1     0x06
#define ADIN2111_REG_MAC_ADDR_2     0x07
#define ADIN2111_REG_PHY_CTRL       0x08
#define ADIN2111_REG_TX_CTRL        0x09
#define ADIN2111_REG_RX_CTRL        0x0A

// TX/RX buffer registers
#define ADIN2111_REG_TX_BUF_PTR     0x20
#define ADIN2111_REG_TX_BUF_LEN     0x21
#define ADIN2111_REG_TX_BUF_CFG     0x22
#define ADIN2111_REG_RX_BUF_PTR     0x23
#define ADIN2111_REG_RX_BUF_LEN     0x24
#define ADIN2111_REG_RX_BUF_CFG     0x25

// PHY registers accessible via MDIO
#define ADIN2111_PHY_BMCR           0x00    // Basic Control
#define ADIN2111_PHY_BMSR           0x01    // Basic Status
#define ADIN2111_PHY_PHYID1         0x02    // PHY ID 1
#define ADIN2111_PHY_PHYID2         0x03    // PHY ID 2
#define ADIN2111_PHY_ANAR           0x04    // Auto-Negotiation Advertisement
#define ADIN2111_PHY_ANLPAR         0x05    // Auto-Negotiation Link Partner Ability
#define ADIN2111_PHY_ANER           0x06    // Auto-Negotiation Expansion

// PHY Control register bits
#define ADIN2111_BMCR_RESET         (1 << 15)
#define ADIN2111_BMCR_LOOPBACK      (1 << 14)
#define ADIN2111_BMCR_SPEED_SELECT  (1 << 13)
#define ADIN2111_BMCR_AN_ENABLE     (1 << 12)
#define ADIN2111_BMCR_POWER_DOWN    (1 << 11)
#define ADIN2111_BMCR_ISOLATE       (1 << 10)
#define ADIN2111_BMCR_AN_RESTART    (1 << 9)
#define ADIN2111_BMCR_DUPLEX_MODE   (1 << 8)

// Error codes
#define ADIN2111_OK                 0
#define ADIN2111_ERR_SPI           -1
#define ADIN2111_ERR_ID            -2
#define ADIN2111_ERR_INIT          -3
#define ADIN2111_ERR_PHY           -4
#define ADIN2111_ERR_MAC           -5
#define ADIN2111_ERR_BUF           -6
#define ADIN2111_ERR_CRC           -7

class ADIN2111 {
public:
    /**
     * Constructor
     * @param cs Chip select pin
     * @param spi SPI interface to use
     * @param intr Interrupt pin (optional)
     * @param reset Reset pin (optional)
     */
    ADIN2111(int8_t cs = SS, SPIClass& spi = SPI, int8_t intr = -1, int8_t reset = -1);

    /**
     * Initialize the device
     * @param macAddress MAC address to use
     * @param net lwIP network interface
     * @return true if successful
     */
    bool begin(const uint8_t* macAddress, netif* net);

    /**
     * Shut down the device
     */
    void end();

    /**
     * Send an Ethernet frame
     * @param data Frame data
     * @param datalen Length of frame
     * @return Number of bytes sent or 0 on error
     */
    uint16_t sendFrame(const uint8_t* data, uint16_t datalen);

    /**
     * Read a received frame
     * @param buffer Buffer to store frame
     * @param bufsize Size of buffer
     * @return Number of bytes read or 0 if no frame available
     */
    uint16_t readFrame(uint8_t* buffer, uint16_t bufsize);

    /**
     * Read a received frames data
     * @param buffer Buffer to store frame
     * @param framesize Size of frame
     * @return Number of bytes read or 0 if no frame available
     */
    uint16_t readFrameData(uint8_t* buffer, uint16_t framesize);


    /**
     * Check if link is up
     * @return true if link is up
     */
    bool isLinked();

    /**
     * Report link detection capability
     * @return true since ADIN2111 supports link detection
     */
    constexpr bool isLinkDetectable() const {
        return true;
    }

    /**
     * Report SPI usage
     * @return true since ADIN2111 requires SPI
     */
    constexpr bool needsSPI() const {
        return true;
    }

    /**
     * Get the netif pointer
     */
    netif* _netif;

protected:
    static constexpr bool interruptIsPossible() {
        return true;
    }

    static constexpr PinStatus interruptMode() {
        return LOW;
    }

    /**
     * Get size of next frame
     * @return Frame size or 0 if no frame
     */
    uint16_t readFrameSize();

    /**
     * Discard current frame
     * @param framesize Size from readFrameSize()
     */
    void discardFrame(uint16_t framesize);



private:
    // Register access
    int regWrite(uint16_t addr, uint32_t data);
    int regRead(uint16_t addr, uint32_t* data);
    int regUpdate(uint16_t addr, uint32_t mask, uint32_t data);

    // MDIO access
    int mdioWrite(uint32_t phy_id, uint32_t reg, uint16_t data);
    int mdioRead(uint32_t phy_id, uint32_t reg, uint16_t* data);

    // Initialization helpers
    int doReset();
    int setupMac();
    int setupPhy();

    // Frame handling helpers
    bool waitTxReady(uint32_t timeout_ms = 100);
    bool hasRxData();
    uint8_t calculateCRC8(const uint8_t* data, size_t len);

    // Hardware interface
    int8_t _cs;
    int8_t _intr;
    int8_t _reset;
    SPIClass& _spi;
    const uint8_t* _macAddress;
    
    // State
    bool _appendCrc;
    uint32_t _frameSize;
    uint8_t _buff[ADIN2111_BUFF_SIZE];

    // Statistics
    uint32_t _txCount;
    uint32_t _rxCount;
    uint32_t _errorCount;
};

#endif // ADIN2111_H