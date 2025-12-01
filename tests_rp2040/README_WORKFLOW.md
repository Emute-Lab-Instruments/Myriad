# RP2040 Development Workflow Guide

**Platform:** Raspberry Pi RP2040 (Pico/Pico W compatible)
**SDK:** Pico SDK 2.1.0
**Toolchain:** ARM GCC (arm-none-eabi)
**Host:** Linux (tested on Ubuntu/Debian)

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Prerequisites](#prerequisites)
3. [Project Structure](#project-structure)
4. [Build System](#build-system)
5. [Flashing Firmware](#flashing-firmware)
6. [Monitoring Serial Output](#monitoring-serial-output)
7. [Development Workflow](#development-workflow)
8. [Troubleshooting](#troubleshooting)
9. [Common Commands Reference](#common-commands-reference)

---

## Quick Start

```bash
# 1. Build the firmware
cd tests_rp2040/build
cmake ..
make

# 2. Connect RP2040 via USB (ensure it's in bootloader mode or running)

# 3. Flash to device (auto-reboots into bootloader if needed)
picotool load -f test_sinetable_rp2040.uf2

# 4. Reboot into application
picotool reboot

# 5. Monitor serial output (wait ~3 seconds for device to boot)
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

---

## Prerequisites

### Required Tools

```bash
# Install ARM toolchain
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi \
                 libstdc++-arm-none-eabi-newlib

# Install build tools
sudo apt install cmake build-essential

# Install picotool (RP2040 flashing utility)
sudo apt install picotool
# OR build from source: https://github.com/raspberrypi/picotool

# Install serial monitor tools
sudo apt install minicom screen
# OR use built-in tools: cat, dd, stty
```

### Pico SDK Setup

The project expects Pico SDK to be available. Check your CMakeLists.txt for the SDK path:

```cmake
# In CMakeLists.txt
set(PICO_SDK_PATH "/path/to/pico-sdk")
# OR use environment variable:
# export PICO_SDK_PATH=/path/to/pico-sdk
```

If not installed:
```bash
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=$(pwd)
```

### USB Permissions

Add yourself to the `dialout` group for serial access:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

---

## Project Structure

```
tests_rp2040/
├── CMakeLists.txt              # Build configuration
├── test_sinetable_rp2040.cpp   # Main test program
├── build/                       # Build output directory
│   ├── test_sinetable_rp2040.elf    # ELF executable
│   ├── test_sinetable_rp2040.uf2    # Flash image (drag-drop or picotool)
│   ├── test_sinetable_rp2040.bin    # Raw binary
│   ├── test_sinetable_rp2040.hex    # Intel HEX format
│   └── test_sinetable_rp2040.dis    # Disassembly
├── README.md                    # Test documentation
├── HARDWARE_INTERPOLATOR_NOTES.md  # Performance analysis
└── README_WORKFLOW.md           # This file

../arduino_libmyriad/
├── sinetable_fixed.hpp          # Fixed-point sine table library
└── fixedpoint.hpp               # Fixed-point arithmetic library

../Myriad_A/
└── sinetable.h                  # Float-based sine table (reference)
```

---

## Build System

### Initial Setup (One-Time)

```bash
cd tests_rp2040
mkdir -p build
cd build
cmake ..
```

This generates:
- Makefiles for the project
- Pico SDK configuration
- Dependencies for libraries

### Building

```bash
cd tests_rp2040/build
make

# For verbose output (see actual commands):
make VERBOSE=1

# Clean build:
make clean
rm -rf *  # Nuclear option: delete everything in build/
cmake ..
make
```

### Build Output

After successful build:
```
[  2%] Built target bs2_default
[  5%] Built target bs2_default_library
[  6%] Building CXX object CMakeFiles/test_sinetable_rp2040.dir/test_sinetable_rp2040.cpp.o
[  7%] Linking CXX executable test_sinetable_rp2040.elf
[100%] Built target test_sinetable_rp2040
```

Key files created:
- `test_sinetable_rp2040.elf` - Debug symbols, disassembly
- `test_sinetable_rp2040.uf2` - **Flash this to RP2040**
- `test_sinetable_rp2040.bin` - Raw binary (advanced use)
- `test_sinetable_rp2040.dis` - Disassembly listing

### CMake Configuration Options

```bash
# Change build type (default: Release)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Specify Pico SDK path
cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..

# Use specific board (Pico, Pico W, etc.)
cmake -DPICO_BOARD=pico_w ..
```

---

## Flashing Firmware

### Method 1: picotool (Recommended)

**Automatic (force reboot into bootloader):**
```bash
cd tests_rp2040/build
picotool load -f test_sinetable_rp2040.uf2
picotool reboot
```

The `-f` flag forces the device to reboot into bootloader mode if it's currently running.

**Manual (device already in bootloader):**
```bash
# Put device in bootloader mode manually:
# - Hold BOOTSEL button while connecting USB, OR
# - Press BOOTSEL while pressing and releasing RESET

picotool load test_sinetable_rp2040.uf2
picotool reboot
```

**Check device info:**
```bash
# See device details (bootloader or running app)
picotool info

# Force reboot to bootloader to check
picotool info -f
```

### Method 2: Drag-and-Drop

1. Put RP2040 in bootloader mode (hold BOOTSEL, connect USB)
2. Device appears as USB mass storage: `RPI-RP2`
3. Drag `test_sinetable_rp2040.uf2` to the drive
4. Device automatically reboots and runs the firmware

### Method 3: Direct USB Write (Advanced)

```bash
# If device shows as /dev/sda1 (check with lsblk)
sudo dd if=test_sinetable_rp2040.uf2 of=/dev/sda1 bs=512
sync
```

**⚠️ WARNING:** Ensure correct device! `dd` can destroy data if pointed at wrong device.

---

## Monitoring Serial Output

### Check Serial Device

```bash
# List USB serial devices
ls -la /dev/ttyACM*

# Should show something like:
# crw-rw---- 1 root dialout 166, 0 Dec  1 09:03 /dev/ttyACM0

# Check which device is the RP2040
dmesg | tail -20
# Look for: "cdc_acm 3-1:1.0: ttyACM0: USB ACM device"
```

### Method 1: cat (Simple, Read-Only)

```bash
# Configure serial port
stty -F /dev/ttyACM0 115200 raw -echo

# Read output continuously
cat /dev/ttyACM0

# Read with timeout (12 seconds)
timeout 12 cat /dev/ttyACM0

# Read specific number of bytes
dd if=/dev/ttyACM0 bs=1 count=1000 status=progress
```

**Note:** The test program waits for USB connection before starting:
```cpp
while (!stdio_usb_connected()) {
    sleep_ms(100);
}
```

So you need to connect serial BEFORE the device boots, or it will wait indefinitely.

### Method 2: minicom (Interactive)

```bash
# Launch minicom
minicom -b 115200 -D /dev/ttyACM0

# Exit: Ctrl-A, then X
```

Configure minicom (one-time):
```bash
sudo minicom -s
# Serial port setup → Set speed to 115200, 8N1
# Save setup as default
```

### Method 3: screen (Interactive)

```bash
# Start screen session
screen /dev/ttyACM0 115200

# Detach: Ctrl-A, then D
# Reattach: screen -r
# Kill session: Ctrl-A, then K
```

### Method 4: Python Serial Monitor

```python
#!/usr/bin/env python3
import serial
import sys

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
print("Monitoring RP2040... (Ctrl-C to exit)")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore')
        if line:
            print(line, end='')
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
```

Save as `monitor.py`, run with `python3 monitor.py`.

---

## Development Workflow

### Typical Development Cycle

```bash
# 1. Edit source code
vim tests_rp2040/test_sinetable_rp2040.cpp
vim arduino_libmyriad/sinetable_fixed.hpp

# 2. Build (in build directory)
cd tests_rp2040/build
make

# 3. Flash and reboot (one command)
picotool load -f test_sinetable_rp2040.uf2 && picotool reboot

# 4. Monitor output (wait 3 seconds for boot)
sleep 3 && stty -F /dev/ttyACM0 115200 raw -echo && timeout 15 cat /dev/ttyACM0
```

### Full Build-Flash-Monitor Script

Create `tests_rp2040/build_and_test.sh`:
```bash
#!/bin/bash
set -e  # Exit on error

echo "=== Building firmware ==="
cd "$(dirname "$0")/build"
make

echo ""
echo "=== Flashing to RP2040 ==="
picotool load -f test_sinetable_rp2040.uf2 && picotool reboot

echo ""
echo "=== Waiting for device to boot ==="
sleep 3

echo ""
echo "=== Monitoring serial output (15 seconds) ==="
stty -F /dev/ttyACM0 115200 raw -echo
timeout 15 cat /dev/ttyACM0 || true

echo ""
echo "=== Done! ==="
```

Make executable and use:
```bash
chmod +x tests_rp2040/build_and_test.sh
./tests_rp2040/build_and_test.sh
```

### Debugging with Serial Print

Add debug output in your code:
```cpp
printf("DEBUG: index=%d, frac=0x%04X\n", index, frac);
printf("DEBUG: val0=%d, val1=%d\n", val0, val1);
```

View in real-time:
```bash
cat /dev/ttyACM0
```

### Performance Profiling

The test program uses hardware timer for cycle counting:
```cpp
uint32_t start = time_us_64() * 125;  // Convert µs to cycles @ 125 MHz
// ... code to measure ...
uint32_t end = time_us_64() * 125;
uint32_t cycles = end - start;
```

Resolution: **1 microsecond = 125 cycles** @ 125 MHz clock.

---

## Troubleshooting

### Device Not Found

**Problem:** `picotool` reports "No accessible RP-series devices"

**Solutions:**
```bash
# Check USB connection
lsusb | grep -i "rasp\|pico\|2040"

# Check for bootloader mode device
lsusb | grep "2e8a:0003"  # Bootloader
lsusb | grep "2e8a:000a"  # Running app with USB serial

# Force into bootloader mode
picotool reboot -f -u  # Reboot to bootloader via USB

# Manual bootloader mode:
# 1. Disconnect USB
# 2. Hold BOOTSEL button
# 3. Connect USB while holding BOOTSEL
# 4. Release BOOTSEL after 1 second

# Check device again
lsusb | grep 2e8a
```

### Permission Denied on /dev/ttyACM0

**Problem:** `cat /dev/ttyACM0` says "Permission denied"

**Solution:**
```bash
# Check permissions
ls -la /dev/ttyACM0
# Should show: crw-rw---- 1 root dialout ...

# Add yourself to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in, then verify
groups | grep dialout

# Temporary fix (until reboot):
sudo chmod 666 /dev/ttyACM0
```

### No Serial Output

**Problem:** Device boots but no output on serial

**Checklist:**
1. **Is serial monitor connected fast enough?**
   ```cpp
   // Code waits for USB connection:
   while (!stdio_usb_connected()) { sleep_ms(100); }
   ```
   Connect serial BEFORE device boots, or be quick!

2. **Check baud rate:**
   ```bash
   stty -F /dev/ttyACM0
   # Should show "speed 115200 baud"
   ```

3. **Wrong serial device?**
   ```bash
   ls /dev/ttyACM*
   # Try each one
   ```

4. **USB cable is power-only?**
   - Some micro-USB cables don't have data lines
   - Try a different cable

5. **Code crashed before printing?**
   - Check LED on board (should blink after tests complete)
   - Add early printf for debugging:
     ```cpp
     stdio_init_all();
     printf("Boot!\n");  // Immediate output
     ```

### Build Errors

**Problem:** `undefined reference to 'hardware_interp'`

**Solution:** Check CMakeLists.txt includes hardware libraries:
```cmake
target_link_libraries(test_sinetable_rp2040
    pico_stdlib
    hardware_interp   # Required!
    hardware_timer
    hardware_clocks
)
```

**Problem:** `pico/stdlib.h: No such file or directory`

**Solution:** Set PICO_SDK_PATH:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cd build
rm -rf *
cmake ..
make
```

**Problem:** Compilation slow or errors after editing header

**Solution:** Force full rebuild:
```bash
cd build
make clean
make
```

### Device Resets/Crashes During Tests

**Problem:** Serial output stops mid-test, device reboots

**Debugging steps:**
1. **Check for stack overflow:**
   ```cpp
   // BAD: Large array on stack
   void foo() {
       int buffer[10000];  // 40KB - too big!
   }

   // GOOD: Static or heap allocation
   static int buffer[10000];
   ```

2. **Check for divide-by-zero:**
   ```cpp
   // Add guards:
   if (denominator != 0) {
       result = numerator / denominator;
   }
   ```

3. **Hardware interpolator misconfiguration:**
   - Reading wrong register
   - Invalid configuration
   - Check against SDK examples

4. **Enable watchdog for debugging:**
   ```cpp
   #include "pico/bootrom.h"
   // If crash, force bootloader entry for reflashing
   ```

---

## Common Commands Reference

### Building

```bash
# Standard build
make

# Verbose (see commands)
make VERBOSE=1

# Clean and rebuild
make clean && make

# Nuclear clean (delete all build files)
rm -rf * && cmake .. && make
```

### Flashing

```bash
# Flash with auto-reboot
picotool load -f firmware.uf2 && picotool reboot

# Flash to specific device (if multiple connected)
picotool load -f firmware.uf2 -d /dev/bus/usb/003/016

# Check device info before flashing
picotool info

# Force device into bootloader
picotool reboot -f -u
```

### Serial Monitoring

```bash
# Quick read (12 second timeout)
timeout 12 cat /dev/ttyACM0

# Continuous monitoring
cat /dev/ttyACM0

# With timestamp
cat /dev/ttyACM0 | ts '[%H:%M:%S]'

# Save to file
cat /dev/ttyACM0 | tee output.log

# Interactive terminal
minicom -b 115200 -D /dev/ttyACM0
```

### Combined Workflows

```bash
# Build, flash, monitor (one line)
make && picotool load -f test_sinetable_rp2040.uf2 && picotool reboot && sleep 3 && timeout 15 cat /dev/ttyACM0

# Build only if source changed (make is smart!)
make && echo "Build OK"

# Check for build errors without stopping
make 2>&1 | tee build.log
```

### Device Management

```bash
# List USB devices (check connection)
lsusb | grep 2e8a

# Check serial ports
ls -la /dev/ttyACM*

# Device details
picotool info -a

# List all RP2040 devices
picotool info -l

# Force USB permissions (temporary)
sudo chmod 666 /dev/ttyACM0
```

---

## Performance Testing Best Practices

### 1. Warmup Loops

Always include warmup before timing:
```cpp
// Warmup (prime caches)
for (int i = 0; i < 100; i++) {
    volatile Q16_16 result = table.fast_sin(Q16_16(angle));
}

// Now measure
uint32_t start = time_us_64() * 125;
for (int i = 0; i < 1000; i++) {
    volatile Q16_16 result = table.fast_sin(Q16_16(angle));
}
uint32_t end = time_us_64() * 125;
```

### 2. Use `volatile` for Results

Prevent compiler from optimizing away the loop:
```cpp
// BAD: Compiler may optimize away entire loop
for (int i = 0; i < 1000; i++) {
    Q16_16 result = table.fast_sin(x);
}

// GOOD: Forces computation
for (int i = 0; i < 1000; i++) {
    volatile Q16_16 result = table.fast_sin(x);
}
```

### 3. Multiple Iterations

Use many iterations to reduce timing noise:
```cpp
constexpr int ITERATIONS = 1000;  // Good
// Not: constexpr int ITERATIONS = 10;  // Too few, noisy
```

### 4. Report Cycles, Not Microseconds

More meaningful for embedded:
```cpp
printf("Cost: %lu cycles (%.1f cycles/call)\n",
       total_cycles, (float)total_cycles / ITERATIONS);

// Not just: printf("Time: %lu µs\n", time_us);
```

---

## Additional Resources

- **Pico SDK Documentation:** https://raspberrypi.github.io/pico-sdk-doxygen/
- **RP2040 Datasheet:** https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
- **Picotool GitHub:** https://github.com/raspberrypi/picotool
- **Pico Examples:** https://github.com/raspberrypi/pico-examples
- **This Project:**
  - Test code: `tests_rp2040/test_sinetable_rp2040.cpp`
  - Library: `arduino_libmyriad/sinetable_fixed.hpp`
  - Performance notes: `tests_rp2040/HARDWARE_INTERPOLATOR_NOTES.md`

---

## Quick Reference Card

```
BUILD:    cd tests_rp2040/build && make
FLASH:    picotool load -f test_sinetable_rp2040.uf2 && picotool reboot
MONITOR:  sleep 3 && timeout 15 cat /dev/ttyACM0

DEVICE:   lsusb | grep 2e8a
SERIAL:   ls /dev/ttyACM*
INFO:     picotool info

BOOTLOADER: Hold BOOTSEL, connect USB, release BOOTSEL
FORCE BOOT: picotool reboot -f -u

FULL CYCLE:
  make && \
  picotool load -f test_sinetable_rp2040.uf2 && \
  picotool reboot && \
  sleep 3 && \
  timeout 15 cat /dev/ttyACM0
```

---

**Last Updated:** 2025-12-01
**Tested On:** Ubuntu 22.04 LTS, RP2040 (Pico), Pico SDK 2.1.0
