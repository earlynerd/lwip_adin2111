/*
 * PacketPool implementation - Static memory allocation definitions
 * This file defines the static memory arrays used when ADIN2111_STATIC_ALLOCATION is enabled
 */

#include "PacketPool.h"

#ifdef ADIN2111_STATIC_ALLOCATION

// Define static memory for RX pool
uint8_t PacketPool::_staticMemory[RX_POOL_COUNT * RX_PACKET_SIZE];

// Define static memory for TX pool
uint8_t TxPacketPool::_staticMemory[TX_POOL_COUNT * TX_PACKET_SIZE];

#endif // ADIN2111_STATIC_ALLOCATION
