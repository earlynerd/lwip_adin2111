#pragma once

#include <Arduino.h>
#include <string.h>

// --- Configuration ---
#define RX_POOL_COUNT  16
#define RX_PACKET_SIZE 1524

// TX Pool: Increased from 10 to 16 for better throughput under load
// Should be at least 2x hardware queue depth (4) for good pipelining
#define TX_POOL_COUNT  16
#define TX_PACKET_SIZE 1524

// Memory allocation mode - uncomment to use static allocation instead of malloc
// #define ADIN2111_STATIC_ALLOCATION

// ==========================================================================
// RX POOL - Lock-free friendly design with fixed-size arrays
// ==========================================================================
class PacketPool {
public:
    struct RxPacket {
        uint8_t* buffer;
        uint32_t len;
    };

    PacketPool() {
#ifdef ADIN2111_STATIC_ALLOCATION
        _memoryBlock = _staticMemory;
        _initFailed = false;
#else
        _memoryBlock = (uint8_t*)malloc(RX_POOL_COUNT * RX_PACKET_SIZE);
        if (_memoryBlock == nullptr) {
            _initFailed = true;
            return;
        }
#endif
        // Initialize free buffer stack (using array instead of std::vector)
        for (int i = 0; i < RX_POOL_COUNT; i++) {
            _freeStack[i] = _memoryBlock + (i * RX_PACKET_SIZE);
        }
        _freeStackTop = RX_POOL_COUNT;

        // Initialize RX queue ring buffer
        _rxQueueHead = 0;
        _rxQueueTail = 0;
    }

    ~PacketPool() {
#ifndef ADIN2111_STATIC_ALLOCATION
        if (_memoryBlock != nullptr) {
            free(_memoryBlock);
        }
#endif
    }

    bool initFailed() const { return _initFailed; }
    uint32_t getDroppedCount() const { return _droppedCount; }
    void incrementDropped() { _droppedCount++; }

    // ISR-safe: Get a fresh buffer to give to hardware
    uint8_t* getFreeBuffer() {
        uint32_t save = save_and_disable_interrupts();
        if (_freeStackTop == 0) {
            restore_interrupts(save);
            return nullptr;
        }
        uint8_t* buf = _freeStack[--_freeStackTop];
        restore_interrupts(save);
        return buf;
    }

    // ISR-safe: Queue a filled buffer for lwIP
    void queueRxPacket(uint8_t* buffer, uint32_t len) {
        uint32_t save = save_and_disable_interrupts();
        // Check if queue is full (ring buffer)
        uint32_t nextHead = (_rxQueueHead + 1) % (RX_POOL_COUNT + 1);
        if (nextHead == _rxQueueTail) {
            // Queue full - this shouldn't happen if pool is sized correctly
            restore_interrupts(save);
            _droppedCount++;
            return;
        }
        _rxQueue[_rxQueueHead].buffer = buffer;
        _rxQueue[_rxQueueHead].len = len;
        _rxQueueHead = nextHead;
        restore_interrupts(save);
    }

    // Loop context: Check size of next packet (quick, minimal critical section)
    uint16_t peekNextPacketSize() {
        uint32_t save = save_and_disable_interrupts();
        if (_rxQueueHead == _rxQueueTail) {
            restore_interrupts(save);
            return 0;
        }
        uint16_t len = (uint16_t)_rxQueue[_rxQueueTail].len;
        restore_interrupts(save);
        return len;
    }

    // Loop context: Copy to lwIP and recycle buffer
    // OPTIMIZED: memcpy is now OUTSIDE the critical section
    uint16_t readAndRecycle(uint8_t* dst, uint16_t maxLen) {
        RxPacket pkt;

        // Critical section 1: Dequeue packet info (fast - just pointer ops)
        {
            uint32_t save = save_and_disable_interrupts();
            if (_rxQueueHead == _rxQueueTail) {
                restore_interrupts(save);
                return 0;
            }
            pkt = _rxQueue[_rxQueueTail];
            _rxQueueTail = (_rxQueueTail + 1) % (RX_POOL_COUNT + 1);
            restore_interrupts(save);
        }

        // OUTSIDE critical section: Do the expensive memcpy
        uint16_t toCopy = (pkt.len < maxLen) ? pkt.len : maxLen;
        memcpy(dst, pkt.buffer, toCopy);

        // Critical section 2: Return buffer to free stack (fast)
        {
            uint32_t save = save_and_disable_interrupts();
            _freeStack[_freeStackTop++] = pkt.buffer;
            restore_interrupts(save);
        }

        return toCopy;
    }

    // Loop context: Discard current packet without copying
    void discardCurrentPacket() {
        uint8_t* bufferToFree = nullptr;

        // Critical section 1: Dequeue
        {
            uint32_t save = save_and_disable_interrupts();
            if (_rxQueueHead == _rxQueueTail) {
                restore_interrupts(save);
                return;
            }
            bufferToFree = _rxQueue[_rxQueueTail].buffer;
            _rxQueueTail = (_rxQueueTail + 1) % (RX_POOL_COUNT + 1);
            restore_interrupts(save);
        }

        // Critical section 2: Return to free stack
        {
            uint32_t save = save_and_disable_interrupts();
            _freeStack[_freeStackTop++] = bufferToFree;
            restore_interrupts(save);
        }
    }

    // Thread-safe level check
    size_t getLevel() {
        uint32_t save = save_and_disable_interrupts();
        uint32_t head = _rxQueueHead;
        uint32_t tail = _rxQueueTail;
        restore_interrupts(save);

        if (head >= tail) {
            return head - tail;
        } else {
            return (RX_POOL_COUNT + 1) - tail + head;
        }
    }

    size_t getTotalMemory() const {
        return RX_POOL_COUNT * RX_PACKET_SIZE;
    }

    const char* getAllocationMode() const {
#ifdef ADIN2111_STATIC_ALLOCATION
        return "STATIC";
#else
        return "DYNAMIC";
#endif
    }

private:
#ifdef ADIN2111_STATIC_ALLOCATION
    static uint8_t _staticMemory[RX_POOL_COUNT * RX_PACKET_SIZE];
#endif
    uint8_t* _memoryBlock = nullptr;

    // Free buffer stack (replaces std::vector) - no dynamic allocation
    uint8_t* _freeStack[RX_POOL_COUNT];
    volatile uint32_t _freeStackTop = 0;

    // RX queue ring buffer (replaces std::deque) - no dynamic allocation
    // Size is POOL_COUNT + 1 to distinguish full from empty
    RxPacket _rxQueue[RX_POOL_COUNT + 1];
    volatile uint32_t _rxQueueHead = 0;
    volatile uint32_t _rxQueueTail = 0;

    bool _initFailed = false;
    volatile uint32_t _droppedCount = 0;
};

// ==========================================================================
// TX POOL - Lock-free friendly design with fixed-size array
// ==========================================================================
class TxPacketPool {
public:
    TxPacketPool() {
#ifdef ADIN2111_STATIC_ALLOCATION
        _memoryBlock = _staticMemory;
        _initFailed = false;
#else
        _memoryBlock = (uint8_t*)malloc(TX_POOL_COUNT * TX_PACKET_SIZE);
        if (_memoryBlock == nullptr) {
            _initFailed = true;
            return;
        }
#endif
        // Initialize free buffer stack
        for (int i = 0; i < TX_POOL_COUNT; i++) {
            _freeStack[i] = _memoryBlock + (i * TX_PACKET_SIZE);
        }
        _freeStackTop = TX_POOL_COUNT;
    }

    ~TxPacketPool() {
#ifndef ADIN2111_STATIC_ALLOCATION
        if (_memoryBlock != nullptr) {
            free(_memoryBlock);
        }
#endif
    }

    bool initFailed() const { return _initFailed; }

    // Call from sendFrame: Get a buffer to copy lwIP data INTO
    uint8_t* allocate() {
        uint32_t save = save_and_disable_interrupts();
        if (_freeStackTop == 0) {
            restore_interrupts(save);
            return nullptr;
        }
        uint8_t* buf = _freeStack[--_freeStackTop];
        restore_interrupts(save);
        return buf;
    }

    // Call from ISR (TxCallback): Hardware is done, recycle buffer
    void release(uint8_t* buf) {
        uint32_t save = save_and_disable_interrupts();
        if (_freeStackTop < TX_POOL_COUNT) {
            _freeStack[_freeStackTop++] = buf;
        }
        restore_interrupts(save);
    }

    // Thread-safe level check
    size_t getLevel() {
        uint32_t save = save_and_disable_interrupts();
        size_t level = _freeStackTop;
        restore_interrupts(save);
        return level;
    }

    size_t getTotalMemory() const {
        return TX_POOL_COUNT * TX_PACKET_SIZE;
    }

    const char* getAllocationMode() const {
#ifdef ADIN2111_STATIC_ALLOCATION
        return "STATIC";
#else
        return "DYNAMIC";
#endif
    }

private:
#ifdef ADIN2111_STATIC_ALLOCATION
    static uint8_t _staticMemory[TX_POOL_COUNT * TX_PACKET_SIZE];
#endif
    uint8_t* _memoryBlock = nullptr;

    // Free buffer stack (replaces std::vector) - no dynamic allocation
    uint8_t* _freeStack[TX_POOL_COUNT];
    volatile uint32_t _freeStackTop = 0;

    bool _initFailed = false;
};
