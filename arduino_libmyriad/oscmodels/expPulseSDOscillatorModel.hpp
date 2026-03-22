#pragma once
#include "oscillatorModel.hpp"

class expPulseSDOscillatorModel : public virtual oscillatorModel {
  private:
    // size_t loopLengthBits=0;
  public:


    expPulseSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      // loopLengthBits = 1 << __builtin_clz(loopLength);
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    Fixed<16,16> counterFP = Q16_16(0);
    Fixed<16,16> targetIncFP = Q16_16(1000);
    Fixed<16,16> targetIncFPOrg = Q16_16(1000);
    Fixed<16,16> targetFP= Q16_16(1000);
    Fixed<16,16> targIncMul = Q16_16(1.1f);
    bool b1=0;

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      static int32_t FADE_REF = 1 << 12;
      const bool fading = fadeDirection != 0;
      const int32_t volumePeak = fading ? static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      ) : 0;
      int32_t lErr = err0;


      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        // size_t loopbits = 1;
        for (size_t bit = 0U; bit < 32U; bit++) {
          // phase = phase >= wlen ? 0 : phase; // wrap around
          uint32_t wrap = (phase < wlen);  // 1 if in range, 0 if needs wrap
          phase *= wrap;  // Becomes 0 if phase >= wlen

          [[unlikely]]
          if (phase == 0) {
            counterFP=Q16_16(0);
            targetFP = targetIncFPOrg;
            targetIncFP = targetIncFPOrg;
            b1=0;

          }

          [[unlikely]]
          if (counterFP >= targetFP) {
            b1 = !b1;
            counterFP = Q16_16(0);
            targetFP += targetIncFP;
            targetIncFP *= targIncMul;
          }
          counterFP += Q16_16(1);

          // word <<= 1;
          // word |= b1;

          int32_t y;
          if (fading) [[unlikely]] {
            int32_t amp = b1 ? FADE_REF : 0;
            y = amp >= lErr ? 1 : 0;
            lErr = (y ? volumePeak : 0) - amp + lErr;
          } else {
            y = b1;
          }

          word <<= 1;
          word |= y;

          phase++;

          // loopbits <<= 1;
        }
        *(bufferA ++) = word;
      }
      err0 = lErr;
      updateFade();
    }


    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<16,16>;
      fptype vinv = fptype(1) - v;
      fptype targmax = fptype(1/32.f).mulWith(WvlenFPType(this->wavelen));
      targetIncFPOrg = ((vinv * targmax) + fptype(10));
      // targetIncFPOrg = targetIncFP;
      targIncMul = (vinv * Q16_16(0.4f)) + Q16_16(1);

    }


    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "exp1";
    }

    void reset() override {
      phase = 0;
      y = 0;
      err0 = 0;
      b1 = 0;
      counterFP = Q16_16(0);
      targetIncFP = targetIncFPOrg;
      targetFP = targetIncFPOrg;
    }

  private:
    size_t phase=0;
    bool y=0;
    int32_t err0=0;


    int pulselen=10000;

};
