#pragma once
#include "oscillatorModel.hpp"

class noiseOscillatorModelSD : public virtual oscillatorModel {
  public:
    noiseOscillatorModelSD() : oscillatorModel(){
      loopLength=8;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true;
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {
      static int32_t FADE_REF = 1 << 12;
      const bool fading = fadeDirection != 0;
      const int32_t volumePeak = fading ? static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      ) : 0;
      int32_t lErr = err0;

      for (size_t i = 0; i < loopLength; ++i) {
        uint32_t word = 0U;

        for(uint32_t bit = 0; bit < 32; ++bit) {
          if (counter == 0) {
            on = !on;  // Toggle state
            Q16_16 rnd = Q16_16::random(Q16_16(0),randMult);
            counter = 1 + (WvlenFPType(wavelen) * WvlenFPType(0.01f)).mulWith(rnd).to_int();
          }
          counter--;

          word <<= 1;
          word |= on;

          // int32_t y;
          // if (fading) [[unlikely]] {
          //   int32_t amp = on ? FADE_REF : 0;
          //   y = amp >= lErr ? 1 : 0;
          //   lErr = (y ? volumePeak : 0) - amp + lErr;
          // } else {
          //   y = on;
          // }

          // word <<= 1;
          // word |= y;
        }

        *(bufferA + i) = word;

      }
      err0 = lErr;
      updateFade();
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    void ctrl(const Q16_16 v) override {
      randMult = Q16_16(-1) + (v.mul_split(v).mul_split(Q16_16(500)));
    }

    String getIdentifier() override {
      return "n2";
    }

  private:
    uint32_t densityThreshold = 0x80000000;  // 50% density by default
    Q16_16 randMult = Q16_16(100);
    bool on=0;
    size_t counter=0;
    int32_t err0 = 0;

};
