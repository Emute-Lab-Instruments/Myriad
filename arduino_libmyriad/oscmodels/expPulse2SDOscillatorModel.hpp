#pragma once
#include "oscillatorModel.hpp"

class expPulse2SDOscillatorModel : public virtual oscillatorModel {
  private:
    size_t loopLengthBits=0;
  public:


    expPulse2SDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    Fixed<16,16> counterFP = Q16_16(0);
    Fixed<16,16> targetIncFP = Q16_16(1000);
    Fixed<16,16> targetIncFPOrg = Q16_16(1000);
    Fixed<16,16> targetFP= Q16_16(1000);
    Fixed<16,16> targIncMul = Q16_16(1.1f);

    // Fixed<16,16> counterFPPhaseInc = Q16_16(1);
    // Fixed<16,16> counterFPPhaseIncInc = Q16_16(1.01f);
    // Fixed<16,16> counterFPPhaseIncIncPos = Q16_16(1.01f);
    // Fixed<16,16> counterFPPhaseIncIncNeg = Q16_16(0.99f);

    // Fixed<16,16> counterModFP = Q16_16(0);
    // Fixed<16,16> targetModFP= Q16_16(1000);
    // Fixed<16,16> targetModIncFPOrg= Q16_16(1000);
    // Fixed<16,16> targetModIncFP = Q16_16(1000);
    // Fixed<16,16> targModIncMul = Q16_16(1.1f);
    bool b1=0;
    // bool b2=0;

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      // const size_t modwlen = (Q16_16(wlen) * Q16_16(7.5f)).to_int(); // Modulator is much slower, creates pulse width modulation
      // const size_t modwlenhalf =modwlen >> 1;
      const Q16_16 onefp = Q16_16(1);
      const Q16_16 zerofp = Q16_16(0);

      static int32_t FADE_REF = 1 << 12;

      const bool fading = fadeDirection != 0;
      const int32_t volumePeak = fading ? static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      ) : 0;
      int32_t lErr = err0;

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        size_t loopbits = 1;
        for (size_t bit = 0U; bit < 32U; bit++) {
          // uint32_t wrap = (phase < wlen);  // 1 if in range, 0 if needs wrap
          // phase *= wrap;  // Becomes 0 if phase >= wlen
          phase = phase >= wlen ? 0 : phase; // wrap around
          // modphase = modphase >= modwlen ? 0 : modphase; // wrap around

          [[unlikely]]
          if (phase == 0) {
            counterFP=zerofp;
            targetFP = targetIncFPOrg;
            targetIncFP = targetIncFPOrg;
            b1=0;
          }

          // [[unlikely]]
          // if (modphase == 0) {
          //   b2=0;
          //   counterModFP=zerofp;
          //   targetModFP = targetModIncFPOrg;
          //   targetModIncFP = targetModIncFPOrg;
          // }

          [[unlikely]]
          if (counterFP >= targetFP) {
            b1 = !b1;
            counterFP = zerofp;
            targetFP += targetIncFP;
            targetIncFP *= targIncMul;
            targetIncFP *= targIncMul;
          }

          // [[unlikely]]
          // if (counterModFP >= targetModFP) {
          //   b2 = !b2;
          //   if (b2) {
          //     counterFPPhaseInc = Q16_16(1.9); // When b2 is high, speed up the phase increment for a few cycles, creating a wider pulse
          //   } else {
          //     counterFPPhaseInc = Q16_16(0.1); // Normal speed
          //   }
          //   counterModFP = zerofp;
          //   targetModFP += targetModIncFP;
          //   targetModIncFP *= targModIncMul;
          // }


          counterFP += onefp;
          // counterModFP += onefp;

          // counterFPPhaseInc *= counterFPPhaseIncInc;
          // if (counterFPPhaseInc > Q16_16(1.1)) {
          //   // counterFPPhaseInc = Q16_16(1);
          //   counterFPPhaseIncInc = counterFPPhaseIncIncNeg;
          // }
          // if (counterFPPhaseInc < Q16_16(0.9)) {
          //   counterFPPhaseIncInc = counterFPPhaseIncIncPos;
          // }

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

        }
        *(bufferA ++) = word;

      }
      err0 = lErr;
      updateFade();
    }


    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<16,16>;
      fptype vinv = fptype(1) - v;

      fptype targmax = fptype(1/16.f).mulWith(WvlenFPType(this->wavelen));
      targetIncFPOrg = ((vinv * targmax) + fptype(3));
      targIncMul = Q16_16(1) - (vinv * Q16_16(0.8f)) ;

      // counterFPPhaseIncIncNeg = Q16_16(1) - (v * Q16_16(0.05f));
      // counterFPPhaseIncIncPos = Q16_16(1) + (v * Q16_16(0.05f));

      // fptype targModmax = fptype(1/2.f).mulWith(WvlenFPType(this->wavelen));
      // targetModIncFPOrg = ((vinv * targModmax) + fptype(10));
      // targModIncMul = (vinv * Q16_16(0.1f)) + Q16_16(1);

    }


    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "slide";
    }

    void reset() override {
      phase = 0;
      modphase = 0;
      err0 = 0;
      b1 = 0;
      counterFP = Q16_16(0);
      targetIncFP = targetIncFPOrg;
      targetFP = targetIncFPOrg;
    }

  private:
    size_t phase=0;
    size_t modphase=0;
    int32_t err0=0;


    int pulselen=10000;

};
