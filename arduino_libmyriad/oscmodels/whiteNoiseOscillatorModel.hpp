#pragma once
#include "oscillatorModel.hpp"

class whiteNoiseOscillatorModel : public virtual oscillatorModel {
  public:


    whiteNoiseOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      setClockModShift(1);
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      wv20 = WvlenFPType(getWavelenAtFrequency(20.f));
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const bool fading = fadeDirection != 0;

      for (size_t i = 0; i < loopLength; ++i) {
        if ((rand() % 200000) > alpha) {
          int inc = beta>>2;
          acc += (get_rand_32() & 1) ? inc : -inc;
        }
        if (rand() % 1000 > beta)
          acc ^= (rand() & 127U);

        // *(bufferA + i) = acc;
        if (fading) [[unlikely]] {
          // Probabilistically replace words with silence
          // Higher fadeLevel = fewer replacements
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
      alpha = (v * Q16_16(10000)).to_int() + 190000;
      // Serial.printf("alpha: %d\n", alpha);
      beta = 500 + ( (WvlenFPType(this->wavelen) / wv20) * WvlenFPType(400)).to_int();
      // Serial.printf("alpha: %d, beta: %d\n", alpha, beta);
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "wn";
    }

  private:
    // int err0=0;

   int integrator = 0, acc=0;
   int alpha=255, beta=0;
   WvlenFPType wv20;
};
