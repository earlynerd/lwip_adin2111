# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino library that integrates the Analog Devices ADIN2111 dual-port 10BASE-T1L Single Pair Ethernet (SPE) MAC/PHY with the lwIP TCP/IP stack. The library provides a bridge between the ADI low-level C driver and Arduino's high-level networking interface.

## Architecture

### Layer Structure (Bottom to Top)

1. **ADI Driver Layer** (`src/adi_driver/`)
   - Low-level C drivers from Analog Devices (proprietary, mostly unmodified)
   - `adin2111.c/h`: Main MAC device driver
   - `adi_mac.c/h`: MAC layer implementation
   - `adi_phy.c/h`: PHY layer implementation
   - `adi_spi_oa.c/h` and `adi_spi_generic.c/h`: SPI communication layers
   - Register definition headers: `ADIN2111_mac_*.h`, `ADIN2111_phy_*.h`

2. **Hardware Abstraction Layer** (`src/driver/hal.h`, `src/driver/hal.c`)
   - Bridges ADI C driver to Arduino platform
   - Provides interrupt management, critical sections, SPI wrappers
   - Port-specific implementations in `hal_port_specific.h` and `boardsupport_ard.cpp`

3. **Arduino Driver Layer** (`src/driver/SinglePairEthernet.h/.cpp`)
   - Main C++ class wrapping ADIN2111 device
   - Manages packet pools (RX/TX buffers)
   - Handles callbacks from C driver using static trampoline functions
   - Result counter arrays for debugging (`txResultCounters`, `rxResultCounters`, `initResultCounters`)

4. **Memory Management** (`src/utility/PacketPool.h`)
   - `PacketPool`: RX buffer pool (16 buffers of 1524 bytes)
   - `TxPacketPool`: TX buffer pool (10 buffers of 1524 bytes)
   - ISR-safe allocation/deallocation using interrupt disable/restore
   - Queues filled RX packets for lwIP to process

5. **lwIP Adapter Layer** (`src/utility/adin2111_wrap.h/.cpp`)
   - `ADIN2111_wrap`: Implements lwIP netif interface
   - Translates between lwIP's `struct netif` and SinglePairEthernet driver
   - Key methods: `begin()`, `readFrameSize()`, `readFrameData()`, `sendFrame()`, `discardFrame()`
   - Manages link state updates to lwIP core (`netif_set_link_up/down`)

6. **Public API** (`src/adin2111_lwip.h`)
   - Type alias: `using ADIN2111_lwIP = LwipIntfDev<ADIN2111_wrap>;`
   - Users instantiate this to get full lwIP networking

### Critical Design Patterns

**Callback Bridging (C to C++)**
- ADI C driver uses C-style callbacks
- `SinglePairEthernet` provides static `*_C_Compatible()` trampolines
- Trampolines retrieve C++ object via `adin2111_GetUserContext()` and call member functions
- Pattern used for: RX, TX, and link status change callbacks

**Zero-Copy Buffer Management**
- RX: Hardware fills pool buffer → queue packet → lwIP reads → recycle to pool
- TX: Allocate from pool → copy lwIP data → submit to hardware → recycle in TX callback
- If pool exhausted: Drop packets (increment counter) rather than block

**Interrupt-Driven Operation**
- Hardware interrupt pin (`_intr`) triggers on packet RX/TX completion
- Uses `LOW` level trigger (not `FALLING` edge) to catch already-asserted interrupts
- lwIP template masks GPIO interrupts during processing to prevent interrupt storms

## Key Configuration Values

- `SPE_NUM_BUFS`: 4 hardware descriptor slots
- `RX_POOL_COUNT`: 16 (software buffers for queuing)
- `TX_POOL_COUNT`: 10 (matches hardware queue + extras)
- `SPE_MAX_BUF_FRAME_SIZE`: 1524 bytes (1518 frame + 4 FCS + 2 header)
- Default MAC addresses defined in `SinglePairEthernet.h` (Analog Devices examples)

## Development Notes

- **Target Platform**: Arduino-compatible boards with SPI (developed on ESP32-based boards based on imports)
- **Build System**: Arduino library format (no Makefile/CMake found)
- **No Tests**: No test framework or test files present
- **SPI Speed**: ADIN2111 supports up to 25 MHz SPI clock
- **Pin Configuration**: Typical pins include CS, interrupt, reset, cfg0, cfg1

## Common Workflows

### Adding New ADIN2111 Features
The `SinglePairEthernet` class exposes ~50 wrapper methods that directly call the underlying ADI driver. To add functionality:
1. Check if method already exposed in `SinglePairEthernet.h` (lines 83-144)
2. If missing, add wrapper in `SinglePairEthernet.cpp` calling `adin2111_*()` function
3. Update public interface if needed for user access

### Debugging Packet Flow
- Check result counters: `spe.txResultCounters[]`, `spe.rxResultCounters[]`
- Use `PacketPool::getDroppedCount()` for RX overflow
- Use `PacketPool::getLevel()` / `TxPacketPool::getLevel()` for queue depth
- Link state: `adin2111.isLinkUp()` returns cached status (no SPI transaction)

### Memory Issues
- Pools allocate with `malloc()` in constructor
- Check `poolInitFailed()` before using driver
- RX drop counter indicates insufficient pool size or slow lwIP processing

## Important Files by Task

- **Modifying hardware init**: `src/driver/SinglePairEthernet.cpp::begin()`
- **Changing buffer sizes**: `src/utility/PacketPool.h` (RX/TX_POOL_COUNT/SIZE)
- **Interrupt handling**: `src/driver/boardsupport_ard.cpp`, `src/driver/hal.c`
- **lwIP integration**: `src/utility/adin2111_wrap.cpp`
- **SPI communication**: `src/adi_driver/adi_spi_oa.c` (Open Alliance) or `adi_spi_generic.c`

## Git Branch Strategy

- Main branch: `main`
- Current development: `refactor` (recent work on interrupts and structural changes)
- Recent focus areas: Hardware interrupts, bug fixes, ping functionality
