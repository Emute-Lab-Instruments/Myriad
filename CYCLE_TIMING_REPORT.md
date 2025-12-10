# MyriadB fillBuffer() Detailed Cycle Analysis

**Target:** ARM Cortex-M0+ (RP2040) @ 125 MHz (8ns/cycle)
**Binary:** Myriad_B.ino.elf (Dec 9, 2024)
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

## 1. silentOscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < bufferSize; ++i) {
        *(bufferA + i) = silentOscillatorModel::halfWaveLen;  // constant: 26
    }
}
```

**Parameters:** bufferSize = 16

**Analysis:**
The compiler optimized this with vectorized STMIA (store multiple) instructions.

```
Prologue:
- push {r4, r5, r6, r7, lr}     6 cycles (1+5)
- Load bufferSize, setup         ~6 cycles

Main loop (8 iterations for 16 elements using paired stores):
- Per iteration:
  - stmia r3!, {r4, r5}          3 cycles (1+2 registers)
  - cmp r2, r3                   1 cycle
  - bne (taken 7x, not taken 1x) 2 cycles (taken)
  Total per iteration: 6 cycles × 8 = 48 cycles

Epilogue:
- Cleanup and pop {r4-r7, pc}    ~8 cycles

Total: ~68 cycles
```

**Result:**
- **Estimated cycles: 68**
- **Time @ 125 MHz: 544 ns (0.544 µs)**
- **Per buffer element: 4.25 cycles**

---

## 2. sawOscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    const size_t wlen = this->wavelen;
    for (size_t i = 0; i < loopLength; ++i) {   // loopLength = 8
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {   // 32-bit inner loop
            // Sigma-delta phase accumulator with wrapping
            phase = phase >= wlen ? 0 : phase + 1;
            size_t amp = (phase * phaseMul) >> 15U;
            amp = amp >= wlen ? 0 : amp;
            const bool y = amp >= err0 ? 1 : 0;
            err0 = (y ? wlen : 0) - amp + err0;
            word |= (y << bit);
        }
        *(bufferA + i) = word;
    }
}
```

**Parameters:** loopLength = 8 words × 32 bits = 256 samples

**Analysis - Inner Loop (per bit):**
```
Phase wrapping (branchless with masks):
- cmp, sbcs, ands, adds         4 cycles
- movs, muls, lsrs              3 cycles
- cmp, sbcs, ands               3 cycles

Error accumulator (sigma-delta):
- cmp                           1 cycle
- bhi/fall-through              1-2 cycles (average 1.5)
- subs, mov, lsls, adds         4 cycles
- orrs                          1 cycle
- cmp, bne                      2-3 cycles

Per bit: ~19-20 cycles
Per 32-bit word: 32 × 20 = 640 cycles
Per buffer (8 words): 8 × 640 = 5,120 cycles

Plus prologue/epilogue: ~50 cycles
```

**Result:**
- **Estimated cycles: 5,170**
- **Time @ 125 MHz: 41.36 µs**
- **Per bit sample: 20.2 cycles**

---

## 3. triOscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    const size_t wlen = this->wavelen;
    const int32_t triPeakPoint = (wlen * phaseRisingInvMul) >> qfp;

    for (size_t i = 0; i < loopLength; ++i) {  // loopLength = 16
        uint32_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {  // 32-bit inner loop
            phase = phase >= wlen ? 0 : phase;
            int32_t amp=0;

            if (phase <= triPeakPoint) {
                amp = (phase * phaseRisingMul) >> qfp;
            } else {
                const int32_t fallingPhase = ((phase - triPeakPoint) * phaseFallingMul) >> qfp;
                amp = wavelen - fallingPhase;
            }

            int32_t y = amp >= err0 ? 1 : 0;
            err0 = (y ? wlen : 0) - amp + err0;
            word |= (y << bit);
            phase++;
        }
        *(bufferA + i) = word;
    }
}
```

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis - Inner Loop (per bit):**
```
Phase management:
- cmp, conditional set          2 cycles
- mov, adds                     2 cycles

Triangle calculation (2 paths, average):
- cmp (triPeakPoint)            1 cycle
- Branch + mul + shift          ~8 cycles (average of both paths)

Sigma-delta:
- cmp, conditional              2 cycles
- subs, adds, orrs              3 cycles
- adds                          1 cycle

Per bit: ~19 cycles
Per 32-bit word: 32 × 19 = 608 cycles
Per buffer (16 words): 16 × 608 = 9,728 cycles

Plus prologue/epilogue: ~50 cycles
```

**Result:**
- **Estimated cycles: 9,778**
- **Time @ 125 MHz: 78.22 µs**
- **Per bit sample: 19.1 cycles**

---

## 4. pulseSDOscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    const size_t wlen = this->wavelen;
    for (size_t i = 0; i < loopLength; ++i) {   // loopLength = 16
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {  // 32-bit inner loop
            phase = phase >= wlen ? 0 : phase;
            size_t amp = phase > pulselen ? 0 : wlen;  // Pulse wave

            int32_t y = amp >= err0 ? 1 : 0;
            err0 = (y ? wlen : 0) - amp + err0;
            word |= (y << bit);
            phase++;
        }
        *(bufferA + i) = word;
    }
}
```

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis - Inner Loop (per bit):**
```
Phase management:
- cmp, adds                     2 cycles

Pulse amplitude (conditional):
- cmp                           1 cycle
- Branch/conditional set        ~2 cycles

Sigma-delta:
- cmp                           1 cycle
- Conditional branch            2 cycles (average)
- subs, adds, orrs              3 cycles
- adds                          1 cycle
- cmp, bne                      2 cycles

Per bit: ~14-15 cycles
Per 32-bit word: 32 × 15 = 480 cycles
Per buffer (16 words): 16 × 480 = 7,680 cycles

Plus prologue/epilogue: ~50 cycles
```

**Result:**
- **Estimated cycles: 7,730**
- **Time @ 125 MHz: 61.84 µs**
- **Per bit sample: 15.1 cycles**

---

## 5. triSDVar1OscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    const size_t wlen = this->wavelen;
    const size_t midphase = wlen >> 1;
    for (size_t i = 0; i < loopLength; ++i) {   // loopLength = 16
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {  // 32-bit inner loop
            phase = phase >= wlen ? 0 : phase;
            size_t amp = phase < midphase ?
                phase << 1 :
                wlen - ((phase-midphase) << 1);  // Triangle

            const bool y = amp >= err0 ? 1 : 0;
            err0 = (y ? wlen : 0) - amp + err0;
            err0 = (err0 * mul) >> qfp;  // ERROR SCALING (unique feature)
            word |= (y << bit);
            phase++;
        }
        *(bufferA + i) = word;
    }
}
```

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis - Inner Loop (per bit):**
```
Phase management:
- cmp, conditional              2 cycles

Triangle amplitude:
- cmp                           1 cycle
- Conditional arithmetic        ~5 cycles (lsls, subs)

Sigma-delta with ERROR SCALING:
- cmp                           1 cycle
- Conditional                   2 cycles
- subs, adds                    2 cycles
- MULS (error scaling)          1 cycle  ← Extra multiply
- lsrs (shift)                  1 cycle  ← Extra shift
- orrs, adds                    2 cycles

Per bit: ~17-18 cycles
Per 32-bit word: 32 × 18 = 576 cycles
Per buffer (16 words): 16 × 576 = 9,216 cycles

Plus prologue/epilogue: ~50 cycles
```

**Result:**
- **Estimated cycles: 9,266**
- **Time @ 125 MHz: 74.13 µs**
- **Per bit sample: 18.1 cycles**

---

## 6. smoothThreshSDOscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    const size_t wlen = this->wavelen;
    for (size_t i = 0; i < loopLength; ++i) {   // loopLength = 16
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {  // 32-bit inner loop
            phase = phase >= wlen ? 0 : phase;
            size_t amp = phase << 1;
            amp = amp > wlen ? phase : amp;  // Sharktooth

            const bool y = amp >= thr ? 1 : 0;
            size_t err0 = (y ? wlen : 0) - amp + thr;
            thr = ((err0 * alpha) + (thr)) >> qfp;  // SMOOTH THRESHOLD

            word |= (y << bit);
            phase++;
        }
        *(bufferA + i) = word;
    }
}
```

**Parameters:** loopLength = 16 words × 32 bits = 512 samples

**Analysis - Inner Loop (per bit):**
```
Phase management:
- cmp, conditional              2 cycles

Sharktooth amplitude:
- lsls                          1 cycle
- cmp, conditional              ~3 cycles

Smooth threshold (IIR filter on threshold):
- cmp                           1 cycle
- Conditional set               2 cycles
- subs, adds                    2 cycles
- MULS (alpha * err0)           1 cycle  ← Smoothing multiply
- adds                          1 cycle
- lsrs (>> qfp)                 1 cycle  ← Smoothing shift
- orrs, adds                    2 cycles

Per bit: ~16-17 cycles
Per 32-bit word: 32 × 17 = 544 cycles
Per buffer (16 words): 16 × 544 = 8,704 cycles

Plus prologue/epilogue: ~50 cycles
```

**Result:**
- **Estimated cycles: 8,754**
- **Time @ 125 MHz: 70.03 µs**
- **Per bit sample: 17.1 cycles**

---

## 7. whiteNoiseOscillatorModel::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < loopLength; ++i) {   // loopLength = 16
        if ((rand() % 200000) > alpha) {
            int inc = beta>>2;
            acc += (rand() & 1) ? inc : -inc;
        }
        if (rand() % 1000 > beta)
            acc ^= (rand() & 127U);

        *(bufferA + i) = acc;
    }
}
```

**Parameters:** loopLength = 16 elements (NOT bit-by-bit!)

**Analysis - Per Element:**
```
First rand() call:
- bl rand                       3 cycles (function call)
- Function execution            ~20-30 cycles (PRNG)
- Division (% 200000)           ~20-40 cycles (__aeabi_idiv)
- cmp, conditional branch       2-3 cycles

Second conditional block (taken ~50%):
- bl rand                       3 cycles
- Shift, add/subtract           ~5 cycles

Third conditional block:
- bl rand                       3 cycles
- % 1000 division               ~20-30 cycles
- cmp, branch                   2-3 cycles
- XOR operation                 ~3 cycles (if taken)

Per element: ~60-120 cycles (highly variable due to rand() and division)
Average: ~90 cycles per element

Per buffer (16 elements): 16 × 90 = 1,440 cycles
Plus prologue/epilogue: ~30 cycles
```

**Result:**
- **Estimated cycles: 1,470**
- **Time @ 125 MHz: 11.76 µs**
- **Per element: 91.9 cycles**

**Note:** This is much faster than bit-by-bit oscillators because it generates 16 values, not 512 samples. It relies on hardware rand() which is expensive per call but generates full 32-bit words.

---

## 8. noiseOscillatorModel2::fillBuffer

**Source Code:**
```cpp
inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < loopLength; ++i) {   // loopLength = 4
        Q16_16 rnd = Q16_16::random_hw(Q16_16(0), randMult);
        *(bufferA + i) = (WvlenFPType(wavelen) * WvlenFPType(0.01f)).mulWith(rnd).to_int();
    }
}
```

**Parameters:** loopLength = 4 elements

**Analysis - Per Element:**
```
Hardware random:
- bl get_rand_32                3 cycles (function call)
- PRNG execution                ~15-20 cycles
- Fixed-point setup             ~5 cycles

Fixed-point multiplication (64-bit):
- bl __aeabi_lmul               3 cycles (function call)
- 64-bit multiply execution     ~20-30 cycles

Second multiplication:
- Another lmul call             3 + 20-30 cycles

Shifts and conversions:
- lsls, lsrs, orrs, asrs        ~10 cycles

Per element: ~80-100 cycles
Average: ~90 cycles

Per buffer (4 elements): 4 × 90 = 360 cycles
Plus prologue/epilogue: ~40 cycles
```

**Result:**
- **Estimated cycles: 400**
- **Time @ 125 MHz: 3.2 µs**
- **Per element: 100 cycles**

**Note:** Very fast because buffer size is only 4, and it doesn't use bit-by-bit generation.

---

## Summary Table

| Oscillator Model | Buffer Elements | Total Samples | Est. Cycles | Time @ 125MHz | Cycles/Sample |
|-----------------|-----------------|---------------|-------------|---------------|---------------|
| **noiseOscillatorModel2** | 4 | 4 | 400 | 3.2 µs | 100.0 |
| **silentOscillatorModel** | 16 | 16 | 68 | 0.5 µs | 4.3 |
| **whiteNoiseOscillatorModel** | 16 | 16 | 1,470 | 11.8 µs | 91.9 |
| **sawOscillatorModel** | 8 words | 256 bits | 5,170 | 41.4 µs | 20.2 |
| **pulseSDOscillatorModel** | 16 words | 512 bits | 7,730 | 61.8 µs | 15.1 |
| **smoothThreshSDOscillatorModel** | 16 words | 512 bits | 8,754 | 70.0 µs | 17.1 |
| **triSDVar1OscillatorModel** | 16 words | 512 bits | 9,266 | 74.1 µs | 18.1 |
| **triOscillatorModel** | 16 words | 512 bits | 9,778 | 78.2 µs | 19.1 |

## Key Observations

1. **Bit-by-bit sigma-delta oscillators** (saw, tri, pulse, etc.) are the most expensive because they generate each bit individually with sigma-delta modulation
   - Inner loop runs 32 times per word
   - Outer loop runs 8-16 times depending on model
   - Total: 256-512 bit calculations per fillBuffer() call

2. **Noise oscillators** are faster because they don't use bit-by-bit generation
   - Generate full 32-bit words directly
   - Much smaller buffer sizes (4-16 elements)

3. **Most expensive operations:**
   - Multiply (MULS): 1 cycle - but used in every inner loop iteration
   - Memory access (LDR/STR): 2 cycles each
   - Conditional branches: 2 cycles when taken
   - Function calls (rand, division): 20-50+ cycles

4. **silentOscillatorModel is special:**
   - Highly optimized by compiler to use STMIA (store multiple)
   - Just fills buffer with constant value
   - No computation, pure memory writes

5. **At typical DMA sample rates:**
   - If DMA consumes buffer every ~100 µs, all oscillators can keep up
   - Critical path is the bit-by-bit oscillators with 512 samples @ ~78 µs max

---

## Methodology Notes

- Cycle counts are **estimates** based on manual analysis of disassembly
- Branch prediction not available on Cortex-M0+, so branches are predictable (1 or 2 cycles)
- Memory access assumed to be RAM (2 cycles); flash would be different
- Function calls like `rand()` and `__aeabi_idiv` have variable execution time
- Average case assumed for conditional branches (50/50 split unless code structure suggests otherwise)
- Analysis performed on disassembly from `Myriad_B/build/rp2040.rp2040.generic/Myriad_B.ino.elf`

## Tools Used

- `arm-none-eabi-objdump` for disassembly
- Manual instruction-by-instruction cycle counting based on ARM Cortex-M0+ documentation
- Reference: https://www.breatharian.eu/hw/rasppicoasm/index_en.html
