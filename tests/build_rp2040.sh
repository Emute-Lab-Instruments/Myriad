#!/bin/bash
# Build script for RP2040 hardware test

set -e  # Exit on error

echo "========================================"
echo "  Building RP2040 Hardware Test"
echo "========================================"

# Create build directory
mkdir -p build_rp2040
cd build_rp2040

# Configure with CMake
echo "Configuring..."
cmake -DCMAKE_BUILD_TYPE=Release -f ../CMakeLists_rp2040.txt ..

# Build
echo "Building..."
make -j$(nproc)

# Check if build succeeded
if [ -f test_sinetable_rp2040.uf2 ]; then
    echo ""
    echo "========================================"
    echo "  Build Successful!"
    echo "========================================"
    echo ""
    echo "Firmware: test_sinetable_rp2040.uf2"
    echo "Size: $(du -h test_sinetable_rp2040.uf2 | cut -f1)"
    echo ""
    echo "To flash:"
    echo "  1. Hold BOOTSEL button while plugging in Pico"
    echo "  2. cp test_sinetable_rp2040.uf2 /media/\$USER/RPI-RP2/"
    echo "  Or: picotool load test_sinetable_rp2040.uf2 && picotool reboot"
    echo ""
    echo "To monitor output:"
    echo "  minicom -b 115200 -D /dev/ttyACM0"
    echo "  Or: screen /dev/ttyACM0 115200"
    echo ""
else
    echo "Build failed!"
    exit 1
fi
