#pragma once
#include "oscillatorModel.hpp"

//sharktooth
//smooth threshold
//great oscillator - quite nonlinear, nice thin sounds
class smoothThreshSDOscillatorModel : public virtual oscillatorModel {
  public:

    smoothThreshSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;

      int32_t local_phase = phase;     // Load once
      int32_t local_thr = thr;         // Load once
      const bool fading = fadeDirection != 0;

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          local_phase = local_phase >= wlen ? 0 : local_phase; // wrap around

          size_t amp = local_phase << 1;
          amp =  amp > wlen ? local_phase : amp;


          const bool y = amp >= local_thr ? 1 : 0;
          size_t err0 = (y ? wlen : 0) - amp + local_thr;

          local_thr = ((err0 * alpha) + (local_thr)) >> qfp;

          word <<= 1;
          word |= y;

          local_phase++;
        }

        if (fading) [[unlikely]] {
          if ((get_rand_32() & fadeMaxLevel) >= fadeLevel)
            word = 0xAAAAAAAA;
        }
        *(bufferA + i) = word;

      }
      updateFade();
      phase = local_phase;   // Store once at end
      thr = local_thr;       // Store once at end
    }

    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<1,30>;
      fptype v18(v);
      v18 = (fptype(0.1f) * (v18) * (v18)) + fptype(0.03f);
      alpha = v18.raw()>>12;
      alpha_inv = (1<<qfp) - alpha;
      // Serial.printf("%f %d %d %d %d\n", v18.to_float(), alpha, alphatest, alpha_inv, alpha_invtest);

    }


    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "sdt10";
    }


  private:
    size_t phase=0;
    bool y=0;
    int err0=0;

    size_t mul=1;
    size_t lim;
    size_t thr=0;

    static constexpr size_t qfp = 18U;
    static constexpr float qfpMul = 1U << qfp;
    float alphaf = 0.01f; // alpha value for smoothing
    size_t alpha = alphaf * qfpMul;
    size_t alpha_inv = (1.0-alphaf) * qfpMul;

};
