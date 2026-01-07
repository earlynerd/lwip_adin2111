#pragma once

#include <Arduino.h>
#include <deque>
#include <vector>

// --- Configuration ---
#define RX_POOL_COUNT  12
#define RX_PACKET_SIZE 1524

// TX Pool: Needs to match your hardware queue depth (4) 
// plus maybe 1-2 extra if you implement software queueing later.
#define TX_POOL_COUNT  4  
#define TX_PACKET_SIZE 1524

// ==========================================================================
// RX POOL (The one you are already using)
// ==========================================================================
class PacketPool {
public:
    struct RxPacket {
        uint8_t* buffer;
        uint32_t len;
    };

    PacketPool() {
        _memoryBlock = (uint8_t*)malloc(RX_POOL_COUNT * RX_PACKET_SIZE);
        if (_memoryBlock == nullptr) {
            _initFailed = true;
            return;
        }
        for (int i = 0; i < RX_POOL_COUNT; i++) {
            _freeBuffers.push_back(_memoryBlock + (i * RX_PACKET_SIZE));
        }
    }

    bool initFailed() const { return _initFailed; }
    uint32_t getDroppedCount() const { return _droppedCount; }
    void incrementDropped() { _droppedCount++; }

    // ISR: Get a fresh buffer to give to hardware
    uint8_t* getFreeBuffer() {
        uint32_t save = save_and_disable_interrupts();
        if (_freeBuffers.empty()) {
            restore_interrupts(save);
            return nullptr;
        }
        uint8_t* buf = _freeBuffers.back();
        _freeBuffers.pop_back();
        restore_interrupts(save);
        return buf;
    }

    // ISR: Queue a filled buffer for lwIP
    void queueRxPacket(uint8_t* buffer, uint32_t len) {
        uint32_t save = save_and_disable_interrupts();
        _rxQueue.push_back({buffer, len});
        restore_interrupts(save);
    }

    // Loop: Check size
    uint16_t peekNextPacketSize() {
        uint32_t save = save_and_disable_interrupts();
        if (_rxQueue.empty()) {
            restore_interrupts(save);
            return 0;
        }
        uint16_t len = (uint16_t)_rxQueue.front().len;
        restore_interrupts(save);
        return len;
    }

    // Loop: Copy to lwIP and recycle
    uint16_t readAndRecycle(uint8_t* dst, uint16_t maxLen) {
        uint32_t save = save_and_disable_interrupts();
        if (_rxQueue.empty()) {
            restore_interrupts(save);
            return 0;
        }
        RxPacket pkt = _rxQueue.front();
        _rxQueue.pop_front();
        
        uint16_t toCopy = (pkt.len < maxLen) ? pkt.len : maxLen;
        memcpy(dst, pkt.buffer, toCopy);
        _freeBuffers.push_back(pkt.buffer);
        restore_interrupts(save);
        return toCopy;
    }

    // Loop: Cleanup
    void discardCurrentPacket() {
        uint32_t save = save_and_disable_interrupts();
        if (!_rxQueue.empty()) {
            RxPacket pkt = _rxQueue.front();
            _rxQueue.pop_front();
            _freeBuffers.push_back(pkt.buffer);
        }
        restore_interrupts(save);
    }

    size_t getLevel()
    {
        return _rxQueue.size();
    }

private:
    uint8_t* _memoryBlock = nullptr;
    std::vector<uint8_t*> _freeBuffers;
    std::deque<RxPacket> _rxQueue;
    bool _initFailed = false;
    volatile uint32_t _droppedCount = 0;
};

// ==========================================================================
// TX POOL (New!)
// ==========================================================================
class TxPacketPool {
public:
    TxPacketPool() {
        _memoryBlock = (uint8_t*)malloc(TX_POOL_COUNT * TX_PACKET_SIZE);
        if (_memoryBlock == nullptr) {
            _initFailed = true;
            return;
        }
        for (int i = 0; i < TX_POOL_COUNT; i++) {
            _freeBuffers.push_back(_memoryBlock + (i * TX_PACKET_SIZE));
        }
    }

    bool initFailed() const { return _initFailed; }

    // Call from sendFrame: Get a buffer to copy lwIP data INTO
    uint8_t* allocate() {
        uint32_t save = save_and_disable_interrupts();
        if (_freeBuffers.empty()) {
            restore_interrupts(save);
            return nullptr;
        }
        uint8_t* buf = _freeBuffers.back();
        _freeBuffers.pop_back();
        restore_interrupts(save);
        return buf;
    }

    // Call from ISR (TxCallback): Hardware is done, recycle buffer
    void release(uint8_t* buf) {
        uint32_t save = save_and_disable_interrupts();
        _freeBuffers.push_back(buf);
        restore_interrupts(save);
    }

    size_t getLevel()
    {
        return _freeBuffers.size();
    }

private:
    uint8_t* _memoryBlock = nullptr;
    std::vector<uint8_t*> _freeBuffers;
    bool _initFailed = false;
};