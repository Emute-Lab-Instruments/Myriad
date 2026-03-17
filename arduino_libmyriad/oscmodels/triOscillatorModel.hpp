#pragma once
#include "oscillatorModel.hpp"

volatile bool triTableGenerated=false;
static int32_t __not_in_flash("tri") MULTIPLIER_TABLE_RISING[1000];
static int32_t __not_in_flash("tri") MULTIPLIER_TABLE_RISING_INV[1000];
static int32_t __not_in_flash("tri") MULTIPLIER_TABLE_FALLING[1000];

class triOscillatorModel : public virtual oscillatorModel {
  public:

    triOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      if (!triTableGenerated) {
        triTableGenerated=true;
        for(size_t i = 0; i < 1000; ++i) {
          float v = i/1000.f;
          float gradRising = 2.f + (v * 7.f);
          float gradRisingInv = 1.f/gradRising;
          float gradFalling = 1.f/(1.f - (1.f / gradRising));
          phaseRisingMul = static_cast<size_t>((gradRising * qfpMul) + 0.5f);
          phaseRisingInvMul = static_cast<size_t>((gradRisingInv * qfpMul) + 0.5f);
          phaseFallingMul = static_cast<size_t>((gradFalling * qfpMul) + 0.5f);
          MULTIPLIER_TABLE_RISING[i] = phaseRisingMul;
          MULTIPLIER_TABLE_RISING_INV[i] = phaseRisingInvMul;
          MULTIPLIER_TABLE_FALLING[i] = phaseFallingMul;
        }
      }
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {
        const int32_t wlen = this->wavelen;

        const int32_t triPeakPoint = (wlen * phaseRisingInvMul) >> qfp;

        const int32_t local_fadeLevel = fadeLevel; // Load once


        const uint32_t rising_int = phaseRisingMul >> qfp;
        const uint32_t rising_frac = phaseRisingMul & 0x3FFF;

        // Compute the ACTUAL peak amplitude using the same math as the rising side
        const int32_t peakAmp = triPeakPoint * rising_int +
                                (((uint32_t)triPeakPoint * rising_frac) >> qfp);

        // Derive falling multiplier from peakAmp so peak and zero-crossing are exact
        const int32_t fallingSpan = wlen - triPeakPoint;
        const int32_t derivedFallingMul = (static_cast<int64_t>(peakAmp) << qfp) / fallingSpan;

        const uint32_t falling_int = derivedFallingMul >> qfp;
        const uint32_t falling_frac = derivedFallingMul & 0x3FFF;

        int32_t local_phase = phase;     // Load once
        int32_t local_err0 = err0;       // Load once


        for (size_t i = 0; i < loopLength; ++i) {
            uint32_t word = 0U;
            for (size_t bit = 0U; bit < 32U; bit++) {
                local_phase = local_phase >= wlen ? 0 : local_phase;
                int32_t amp;

                if (local_phase <= triPeakPoint) {
                    amp = local_phase * rising_int + (((uint32_t)local_phase * rising_frac) >> qfp);
                } else {
                    const int32_t delta = local_phase - triPeakPoint;
                    const int32_t fallingPhase = delta * falling_int +
                                                (((uint32_t)delta * falling_frac) >> qfp);
                    amp = peakAmp - fallingPhase;
                }

                amp = (amp * local_fadeLevel) >> fadeBitResolution;

                int32_t y = amp >= local_err0 ? 1 : 0;

                local_err0 = (y ? peakAmp : 0) - amp + local_err0;

                word <<= 1;
                word |= y;

                local_phase++;
            }

            *(bufferA + i) = word;
        }

        updateFade();
        phase = local_phase;   // Store once at end
        err0 = local_err0;     // Store once at end
    }

    void ctrl(const Q16_16 v) override {
      const size_t index = (v * Q16_16(900)).to_int();  //static_cast<size_t>(v * 900.f + 0.5f);
      if (index != lastPW || wavelen != lastWavelen) {
        lastPW = index;
        lastWavelen = wavelen;
        phaseRisingMul = MULTIPLIER_TABLE_RISING[index];
        phaseRisingInvMul = MULTIPLIER_TABLE_RISING_INV[index];
        phaseFallingMul = MULTIPLIER_TABLE_FALLING[index];
        risingInc = phaseRisingMul;
        fallingInc = -phaseFallingMul;
      }
      // PERIODIC_RUN(
      //   Serial.printf("fadeShift: %d\n", fadeShift);
      // , 500);
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "tri";
    }

    void reset() override{
      phase=0;
      err0=0;
    }


  private:
    int32_t phase=0;
    size_t phaseMul = 0;
    bool y=0;
    int32_t err0=0;
    size_t val=0;

    int32_t phaseRisingMul=2;
    int32_t phaseRisingInvMul=2;
    int32_t phaseFallingMul=2;
    // int32_t triPeakPoint = 1;
    int32_t risingInc =1;
    int32_t fallingInc = -1;
    int32_t amp_fp = 0; // accumulator for amplitude in fixed point

    static constexpr size_t qfp = 14U;
    static constexpr float qfpMul = 1U << qfp;

    size_t lastPW = 0;
    size_t lastWavelen = 1;
    volatile bool ctrlChange = false;
    // size_t bitshift = 0;
    // uint32_t bitmul = 1U; // multiplier for bit shift, used to scale the output


};
