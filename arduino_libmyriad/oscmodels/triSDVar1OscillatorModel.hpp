#pragma once
#include "oscillatorModel.hpp"

//error scaling, sounds good with ctrl
class triSDVar1OscillatorModel : public virtual oscillatorModel {
  public:

    triSDVar1OscillatorModel() : oscillatorModel(){
      loopLength=16;
      setClockModShift(1);
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const int32_t wlen = this->wavelen;
      const int32_t midphase = wlen >> 1;
      int32_t lPhase = phase;
      int32_t local_fadeLevel = fadeLevel; // Load once

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          lPhase = lPhase >= wlen ? 0 : lPhase; // wrap around


          int32_t amp = lPhase < midphase ?
                lPhase << 1
                :
                wlen - ((lPhase-midphase) << 1); //tri wave

          amp = (amp * local_fadeLevel) >> fadeBitResolution;


          bool y = amp >= err0 ? 1 : 0;

          err0 = (y ? wlen : 0) - amp + err0;

          // err0 = (err0) & mul;
          err0 = (err0 * mul) >> qfp;


          word <<= 1;
          word |= y;

          lPhase++;
        }
        *(bufferA + i) = word;

      }
      phase = lPhase;
      updateFade();

    }


    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<18,14>;
      mul = (fptype(1) - (fptype(v) * fptype(0.9f))).raw();
    }


    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "trv10";
    }


  private:
    int32_t phase=0;
    bool y=0;
    int err0=0;

    int32_t mul=1;

    static constexpr size_t qfp = 14U;
    static constexpr float qfpMul = 1U << qfp;

};
