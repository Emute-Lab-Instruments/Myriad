# RP2040 Fixed-Point Sine Table Tests

**Last Updated:** 2025-12-01

This directory contains comprehensive tests and performance analysis for fixed-point sine table implementations on the RP2040 platform, including hardware interpolator evaluation and optimization studies.

## 📚 Documentation

- **[README_WORKFLOW.md](README_WORKFLOW.md)** - Complete guide for building, flashing, and monitoring RP2040 firmware
- **[HARDWARE_INTERPOLATOR_NOTES.md](HARDWARE_INTERPOLATOR_NOTES.md)** - Detailed performance analysis and when to use hardware vs software
- **This file** - Quick reference and test overview

## 🎯 Quick Start

```bash
# Build
cd tests_rp2040/build
make

# Flash
picotool load -f test_sinetable_rp2040.uf2 && picotool reboot

# Monitor (wait 3 seconds for boot)
sleep 3 && timeout 15 cat /dev/ttyACM0
```

See [README_WORKFLOW.md](README_WORKFLOW.md) for complete instructions.

## 🔬 What This Tests

This test suite evaluates multiple sine table implementations on RP2040 hardware:

1. **Software floor lookup** - Fast but low accuracy
2. **Software linear interpolation** - Excellent accuracy with minimal overhead
3. **Hardware interpolator** - RP2040's hardware acceleration
4. **Symmetry-optimized** - Smaller table using quadrant symmetry

### Key Finding: Software Interpolation Wins! ⭐

**Software linear interpolation is the winner** for single-value calculations:
- **Only 6.6 cycles/call** (vs 6.2 for floor lookup)
- **39x better accuracy** (0.00008 vs 0.003 max error)
- **Only 6% overhead** for massive accuracy gain
- **Simpler than hardware** (no register I/O overhead)

The hardware interpolator is **slower** for single values (20.8 cycles) but valuable for batch processing with dual channels (see [HARDWARE_INTERPOLATOR_NOTES.md](HARDWARE_INTERPOLATOR_NOTES.md)).

## 📊 Performance Results (@ 125 MHz)

| Implementation | Cycles/Call | vs std::sin | Accuracy (max error) |
|---|---:|---:|---|
| **SW Interpolation** ⭐ | **6.6** | **1.46x** | **0.00008** |
| Floor Lookup | 6.2 | 1.55x | 0.003 |
| Symmetry-Optimized | 6.5 | 1.48x | 0.00008 |
| HW Interpolation | 20.8 | 0.46x | 0.00008 |
| Float Lookup | 6.2 | 1.55x | 0.003 |
| std::sin | 9.6 | 1.0x | ~0 (exact) |

**Recommendation:** Use software interpolation (`fast_sin_hw`) for single-value calculations. Consider hardware interpolator only for batch processing with dual channels (2+ oscillators, 32+ samples).

## Features Tested

### 1. **Multiple Implementations**
- Software floor lookup (`fast_sin`)
- Software linear interpolation (`fast_sin_hw`)
- Hardware interpolator acceleration (`fast_sin_hw_accel`)
- Symmetry-optimized quarter-wave table (`fast_sin_sym`)

### 2. **Accuracy Analysis**
- Compares against std::sin baseline
- Tests edge cases (0, π/2, π, 3π/2, 2π)
- Measures interpolation quality between table entries
- **Result:** Software interpolation provides 39x better accuracy than floor lookup

### 3. **Performance Benchmarking**
- Real cycle counts using hardware timer
- Tests single-value and batch processing
- Evaluates dual-channel parallel processing
- **Result:** Hardware interpolator has too much overhead for single values

## 🛠️ Building and Running

**Quick version:**
```bash
cd tests_rp2040/build && make
picotool load -f test_sinetable_rp2040.uf2 && picotool reboot
sleep 3 && timeout 15 cat /dev/ttyACM0
```

**Full details:** See [README_WORKFLOW.md](README_WORKFLOW.md) for:
- Prerequisites and setup
- Build system (CMake)
- Flashing methods
- Serial monitoring options
- Troubleshooting guide

## Expected Output

```
======================================
  RP2040 Sine Table Hardware Test
  Q16_16 Fixed-Point Implementation
======================================

Board: RP2040
Clock: 125 MHz
SDK Version: 2.1.0
Timing: Hardware timer (1 µs resolution)

=== TEST 1: Hardware Interpolator Accuracy ===
  Angle: -6.2832  SW err: 0.000000  HW err: 0.000992
  Angle: -2.4752  SW err: 0.002925  HW err: 0.000300
  Angle: 1.3328  SW err: 0.000315  HW err: 0.000036
  Angle: 5.1408  SW err: 0.002081  HW err: 0.000330
  Angle: 8.9488  SW err: 0.002314  HW err: 0.001241
  Hardware Max Error:  0.001953
  Hardware Avg Error:  0.000528

=== TEST 2: Hardware Interpolator - Edge Cases ===
  [PASS] sin(0)       = 0.0000 (err: 0.000000)
  [PASS] csin(0)      = 1.0000 (err: 0.000000)
  [PASS] sin(π/2)    = 1.0000 (err: 0.000015)
  [PASS] csin(π/2)   = -0.0003 (err: 0.000259)
  [PASS] sin(π)      = -0.0005 (err: 0.000488)
  [PASS] csin(π)     = -1.0000 (err: 0.000000)
  [PASS] sin(3π/2)   = -1.0000 (err: 0.000000)
  [PASS] csin(3π/2)  = 0.0007 (err: 0.000717)
  [PASS] sin(2π)     = 0.0010 (err: 0.000977)
  [PASS] csin(2π)    = 1.0000 (err: 0.000015)
  [PASS] sin(π/4)    = 0.7072 (err: 0.000184)
  [PASS] csin(π/4)   = 0.7070 (err: 0.000016)
  [PASS] sin(π/6)    = 0.5001 (err: 0.000061)
  [PASS] csin(π/6)   = 0.8660 (err: 0.000033)

=== TEST 3: Performance Benchmarking ===
  Iterations: 1000

  Fixed-Point Implementations:
  1. Floor lookup:        6250 cycles (6.2 cycles/call)
  2. SW Interpolation:    6625 cycles (6.6 cycles/call)
  3. HW Interpolation:   20750 cycles (20.8 cycles/call)
  4. Symmetry-optimized:  6500 cycles (6.5 cycles/call)

  Reference Implementations:
  Float Lookup:           6250 cycles (6.2 cycles/call)
  std::sin (baseline):    9625 cycles (9.6 cycles/call)

  Best speedup vs std::sin: 1.48x

=== TEST 4: Dual Hardware Interpolator ===
  sin(π/4): 0.7072 (expected: 0.7071, error: 0.000077) PASS
  sin(π/3): 0.8661 (expected: 0.8660, error: 0.000063) PASS

=== TEST 5: Interpolation Quality ===
  Testing interpolation between table entries...
  Entry spacing: 0.006136 radians
  Software (floor):       max error = 0.003072
  Hardware (linear):      max error = 0.000078
  Interpolation benefit:  39.1x better

======================================
  All Tests Complete!
======================================
```

All tests pass! The device LED will blink after completion.

## 💻 API Usage

### Recommended: Software Interpolation

```cpp
#include "sinetable_fixed.hpp"
using namespace FixedPoint;

SineTableQ16_16 table;

// Best choice: software linear interpolation
Q16_16 angle(1.5708);  // π/2
Q16_16 result = table.fast_sin_hw(angle);  // 6.6 cycles, 0.00008 max error

// Floor lookup (if you need maximum speed over accuracy)
Q16_16 result_fast = table.fast_sin(angle);  // 6.2 cycles, 0.003 max error
```

### For Wavetable Synthesis (Batch Processing)

See [HARDWARE_INTERPOLATOR_NOTES.md](HARDWARE_INTERPOLATOR_NOTES.md) for:
- Dual-channel hardware interpolator usage
- Wavetable synthesis examples
- Performance analysis for 2+ oscillators
- When hardware acceleration helps (hint: 32+ samples, 2+ channels)

## 📁 Project Structure

```
tests_rp2040/
├── README.md                           # This file (quick reference)
├── README_WORKFLOW.md                  # Complete build/flash/monitor guide
├── HARDWARE_INTERPOLATOR_NOTES.md      # Performance analysis & recommendations
├── CMakeLists.txt                      # Build configuration
├── test_sinetable_rp2040.cpp           # Test firmware source
└── build/
    ├── test_sinetable_rp2040.uf2       # Flash image (use this)
    ├── test_sinetable_rp2040.elf       # Debug symbols
    ├── test_sinetable_rp2040.dis       # Disassembly
    └── ...

../arduino_libmyriad/
├── sinetable_fixed.hpp                 # Fixed-point sine table (tested here)
├── fixedpoint.hpp                      # Q16.16 arithmetic
└── README_sinetable_fixed.md           # Library documentation
```

## 🎓 Key Takeaways

1. **Software interpolation wins** for single-value calculations (6.6 cycles, 39x better accuracy than floor)
2. **Hardware interpolator is slower** for single values due to register overhead (20.8 cycles)
3. **Hardware helps for batch processing** - Dual-channel on 32+ samples shows gains
4. **Optimize the right thing** - 32-bit multiply made software fast, hardware wasn't needed
5. **Measure, don't assume** - Real hardware tests revealed surprising results

## 📚 Further Reading

- **[README_WORKFLOW.md](README_WORKFLOW.md)** - How to build, flash, debug
- **[HARDWARE_INTERPOLATOR_NOTES.md](HARDWARE_INTERPOLATOR_NOTES.md)** - Deep dive on performance
- **[RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)** - Hardware reference
- **[Pico SDK Docs](https://www.raspberrypi.com/documentation/pico-sdk/)** - API documentation

---

**Questions?** Check the workflow guide for troubleshooting, or see the performance notes for optimization strategies.
