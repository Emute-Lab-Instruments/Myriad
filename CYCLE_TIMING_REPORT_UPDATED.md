# MyriadB fillBuffer() Detailed Cycle Analysis
**UPDATED: Including impulseSDOscillatorModel**

**Target:** ARM Cortex-M0+ (RP2040) @ 125 MHz (8ns/cycle)
**Binary:** Myriad_B.ino.elf (Dec 9, 2024) + Source analysis for impulseSD
**Documentation:** https://www.breatharian.eu/hw/rasppicoasm/index_en.html
**Analysis Date:** December 9, 2024

## ARM Cortex-M0+ Instruction Timings

- Most ALU instructions: **1 cycle** (ADD, SUB, MOV, AND, OR, XOR, shifts, MUL, etc.)
- Memory load/store (RAM): **2 cycles** (LDR, STR)
- PUSH/POP with N registers: **1+N cycles**
- STM/LDM with N registers: **1+N cycles**
- Conditional branch not taken: **1 cycle**
- Conditional branch taken: **2 cycles**
- Unconditional branch: **2 cycles**
- BL (subroutine call): **3 cycles**
- BX/BLX: **2 cycles**

---

## Quick Reference - Summary Table

| Oscillator Model | Buffer Elements | Total Samples | Est. Cycles | Time @ 125MHz | Cycles/Sample |
|-----------------|-----------------|---------------|-------------|---------------|---------------|
| **noiseOscillatorModel2** | 4 | 4 | 400 | 3.2 µs | 100.0 |
| **silentOscillatorModel** | 16 | 16 | 68 | 0.5 µs | 4.3 |
| **whiteNoiseOscillatorModel** | 16 | 16 | 1,470 | 11.8 µs | 91.9 |
| **impulseSDOscillatorModel** | 8 words | 256 bits | 3,870 | 31.0 µs | 15.1 |
| **sawOscillatorModel** | 8 words | 256 bits | 5,170 | 41.4 µs | 20.2 |
| **pulseSDOscillatorModel** | 16 words | 512 bits | 7,730 | 61.8 µs | 15.1 |
| **smoothThreshSDOscillatorModel** | 16 words | 512 bits | 8,754 | 70.0 µs | 17.1 |
| **triSDVar1OscillatorModel** | 16 words | 512 bits | 9,266 | 74.1 µs | 18.1 |
| **triOscillatorModel** | 16 words | 512 bits | 9,778 | 78.2 µs | 19.1 |

---

## Detailed Analysis by Oscillator

### 1. silentOscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < bufferSize; ++i) {
        *(bufferA + i) = silentOscillatorModel::halfWaveLen;  // constant: 26
    }
}
```

**Parameters:** bufferSize = 16

**Analysis:** Compiler optimized with vectorized STMIA (store multiple) instructions.

**Result:**
- **Estimated cycles: 68**
- **Time @ 125 MHz: 0.54 µs**
- **Per buffer element: 4.25 cycles**

---

### 2. noiseOscillatorModel2::fillBuffer

**Parameters:** loopLength = 4 elements

**Analysis:** Uses hardware random + fixed-point multiplication (64-bit).

**Result:**
- **Estimated cycles: 400**
- **Time @ 125 MHz: 3.2 µs**
- **Per element: 100 cycles**

---

### 3. whiteNoiseOscillatorModel::fillBuffer

**Parameters:** loopLength = 16 elements (NOT bit-by-bit!)

**Analysis:** Multiple rand() calls per element with division operations.

**Result:**
- **Estimated cycles: 1,470**
- **Time @ 125 MHz: 11.8 µs**
- **Per element: 91.9 cycles**

---

### 4. impulseSDOscillatorModel::fillBuffer ⭐ NEW

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    const size_t wlen = this->wavelen;
    for (size_t i = 0; i < loopLength; ++i) {   // loopLength = 8
        size_t word=0U;
        size_t loopbits = 1;
        do {
            phase = phase >= wlen ? 0 : phase;   // wrap around
            if (phase == 0) {
                counter=0;
                target = targetInc;
                b1=0;
            }
            if (counter == target) {
                b1 = !b1;
                counter = 0;
                target += targetInc;
            }
            counter++;
            if (b1)
                word |= loopbits;

            phase++;
            loopbits <<= 1;
        } while (loopbits);
        *(bufferA + i) = word;
    }
}
```

**Parameters:** loopLength = 8 words × 32 bits = 256 samples

**Algorithm Overview:**
This oscillator generates a "sliding" impulse pattern where:
1. Binary output alternates at intervals determined by `targetInc`
2. Each interval, the target increases by `targetInc`, creating acceleration/deceleration
3. Pattern resets at phase wrap-around
4. No sigma-delta modulation - uses direct counter-based generation

**Analysis - Inner Loop (per bit):**

**Best case (no conditionals triggered):**
```
Phase wrap check:              2 cycles
Counter check (not taken):     2 cycles
Counter increment:             1 cycle
Conditional OR:                2-3 cycles
Phase increment:               1 cycle
Bit shift:                     1 cycle
Loop check:                    3 cycles
-------------------------------------------
Minimum per bit: ~13-14 cycles
```

**Worst case (phase wrap + counter hit):**
```
Best case path:                14 cycles
+ Phase wrap triggered:        +7 cycles
+ Counter target hit:          +9 cycles
-------------------------------------------
Total worst case: ~28-30 cycles per bit
```

**Average case:**
- Phase wrap: once per 32 bits = +0.22 cycles/bit
- Counter hit: varies with targetInc (0.08 to 0.7 cycles/bit)
- **Average per bit: ~15 cycles**

**Full Timing:**
```
Prologue:                      ~14 cycles
Outer loop (8 iterations):
  Inner loop: 32 × 15          480 cycles/word
  Store word:                  2 cycles
  Per word:                    482 cycles
  Total (8 words):             3,856 cycles
Epilogue:                      ~6 cycles
-------------------------------------------
Total: 3,870 cycles
```

**Result:**
- **Estimated cycles: 3,870**
- **Time @ 125 MHz: 30.96 µs**
- **Per bit sample: 15.1 cycles**

**Unique Characteristics:**
- **No sigma-delta modulation** - Simpler than saw/tri/pulse models
- **No multiplication** - Only comparisons, increments, shifts
- **Dynamic rhythmic patterns** - Creates "sliding" tempo changes
- **State persistence** - Counter maintains pattern across calls
- **Identifier:** "slide"

**Why it's faster than other bit-by-bit oscillators:**
1. No multiplication operations (unlike saw/tri which multiply every bit)
2. No sigma-delta error accumulation
3. Predictable branching patterns
4. Minimal arithmetic operations

---

### 5. sawOscillatorModel::fillBuffer

**Parameters:** loopLength = 8 words × 32 bits = 256 samples

**Analysis:** Sigma-delta sawtooth with phase multiplication.

**Result:**
- **Estimated cycles: 5,170**
- **Time @ 125 MHz: 41.4 µs**
- **Per bit sample: 20.2 cycles**

---

### 6. pulseSDOscillatorModel::fillBuffer

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis:** Sigma-delta pulse with variable width control.

**Result:**
- **Estimated cycles: 7,730**
- **Time @ 125 MHz: 61.8 µs**
- **Per bit sample: 15.1 cycles**

---

### 7. smoothThreshSDOscillatorModel::fillBuffer

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis:** Sharktooth wave with IIR-filtered threshold.

**Result:**
- **Estimated cycles: 8,754**
- **Time @ 125 MHz: 70.0 µs**
- **Per bit sample: 17.1 cycles**

---

### 8. triSDVar1OscillatorModel::fillBuffer

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis:** Triangle wave with error scaling (extra multiply).

**Result:**
- **Estimated cycles: 9,266**
- **Time @ 125 MHz: 74.1 µs**
- **Per bit sample: 18.1 cycles**

---

### 9. triOscillatorModel::fillBuffer

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis:** Variable triangle with complex rising/falling calculations.

**Result:**
- **Estimated cycles: 9,778**
- **Time @ 125 MHz: 78.2 µs**
- **Per bit sample: 19.1 cycles**

---

## Key Observations

1. **Bit-by-bit sigma-delta oscillators are most expensive**
   - Inner loop: 32 bits per word
   - Outer loop: 8-16 words
   - Total: 256-512 bit calculations per fillBuffer()

2. **impulseSD is the fastest bit-by-bit oscillator** at 15.1 cycles/bit
   - No multiplication operations
   - No sigma-delta accumulator
   - Simple counter-based logic

3. **Noise oscillators are fastest overall**
   - Generate full 32-bit words directly
   - Small buffer sizes (4-16 elements)
   - But expensive per-element due to rand() and division

4. **Most expensive operations:**
   - Multiply (MULS): 1 cycle, but used repeatedly
   - Memory access: 2 cycles
   - Conditional branches (taken): 2 cycles
   - Function calls (rand, division): 20-50+ cycles

5. **All oscillators can keep up with DMA**
   - Slowest: 78 µs (triOscillatorModel)
   - Typical DMA buffer refresh: 100+ µs
   - Comfortable margin for all models

6. **Special cases:**
   - **silentOscillatorModel**: Optimized to STMIA, just fills constant
   - **impulseSDOscillatorModel**: Creates rhythmic "sliding" patterns
   - **noiseOscillatorModel2**: Smallest buffer (4 elements), fastest noise

---

## Ranking by Execution Time

1. **silentOscillatorModel** - 0.5 µs (baseline/silence)
2. **noiseOscillatorModel2** - 3.2 µs (4 samples, HW random)
3. **whiteNoiseOscillatorModel** - 11.8 µs (16 samples, white noise)
4. **impulseSDOscillatorModel** - 31.0 µs (256 bits, rhythmic impulses) ⭐
5. **sawOscillatorModel** - 41.4 µs (256 bits, sawtooth)
6. **pulseSDOscillatorModel** - 61.8 µs (512 bits, pulse)
7. **smoothThreshSDOscillatorModel** - 70.0 µs (512 bits, sharktooth)
8. **triSDVar1OscillatorModel** - 74.1 µs (512 bits, triangle w/ scaling)
9. **triOscillatorModel** - 78.2 µs (512 bits, variable triangle)

---

## Methodology Notes

- Cycle counts are **estimates** based on:
  - Manual analysis of ARM disassembly (for Myriad_B models)
  - Source code analysis (for impulseSDOscillatorModel)
- Branch prediction not available on Cortex-M0+
- Memory access assumes RAM (2 cycles)
- Function calls (rand, division) have variable timing
- Average case assumes 50/50 branch distribution unless code suggests otherwise
- impulseSD analysis is source-based; actual compiled performance may vary

---

## Files Analyzed

- **Source:** `arduino_libmyriad/oscillatorModels.hpp`
- **Binary:** `Myriad_B/build/rp2040.rp2040.generic/Myriad_B.ino.elf`
- **impulseSD:** Source code analysis only (not in current Myriad_B binary)

## Tools Used

- `arm-none-eabi-objdump` for disassembly
- `arm-none-eabi-nm` for symbol analysis
- Manual instruction-by-instruction cycle counting
- Reference: https://www.breatharian.eu/hw/rasppicoasm/index_en.html
