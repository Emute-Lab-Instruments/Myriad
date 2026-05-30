#pragma once
#include "oscillatorModel.hpp"

class whiteNoiseOscillatorModel : public virtual oscillatorModel {
  public:


    whiteNoiseOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      setClockModShift(1);
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      const bool fading = fadeDirection != 0;
      const int inc = (int)(wlen >> 8) + 1;

      for (size_t i = 0; i < loopLength; ++i) {
        phase += 32;
        while (phase >= wlen) {
          phase -= wlen;
          acc = get_rand_32();
        }

        if ((rand() % 200000) < alpha) {
          acc += (get_rand_32() & 1) ? inc : -inc;
        }

        if (fading) [[unlikely]] {
          if ((get_rand_32() & fadeMaxLevel) >= fadeLevel) {
            *(bufferA + i) = 0xAAAAAAAA;
          } else {
            *(bufferA + i) = acc;
          }
        } else {
          *(bufferA + i) = acc;
        }
      }
      updateFade();

    }

    void ctrl(const Q16_16 v) override {
      alpha = (v * Q16_16(200000)).to_int();
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "wn";
    }

    void reset() override {
      acc = 0;
      phase = 0;
    }

  private:
   int acc=0;
   int alpha=0;
   size_t phase=0;
};
