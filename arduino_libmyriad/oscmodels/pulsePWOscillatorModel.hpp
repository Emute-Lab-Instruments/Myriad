#pragma once
#include "oscillatorModel.hpp"

class pulsePWOscillatorModel : public virtual oscillatorModel {
  private:

    WvlenFPType phase = WvlenFPType(0);
    WvlenFPType pulseWidth = WvlenFPType(0.5);

    // int32_t transitionCounter=0;
    int32_t lastOn=0;

    int32_t err0=0;

  public:


    pulsePWOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      // setClockModShift(1);
    }


    inline void fillBuffer(uint32_t* bufferA) {
      const WvlenFPType wlen = WvlenFPType(this->wavelen);
      const WvlenFPType transitionPoint = wlen * pulseWidth;
      // const int32_t transitionCount = wlen < WvlenFPType(1250) ?  wlen.to_int() >> 4 : 0; // Number of samples to randomize after a transition, adjust as needed
      // const WvlenFPType wlenHalf = WvlenFPType(this->wavelen>>1);
      static WvlenFPType zerofp = WvlenFPType(0);
      static WvlenFPType onefp = WvlenFPType(1);

      static int32_t FADE_REF = 1 << 12;

      const int32_t volumePeak = static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      );
      int32_t lErr = err0;


      WvlenFPType lPhase = phase;

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for (size_t bit = 0U; bit < 32U; bit++) {

          // //wrap phase
          // lPhase = (lPhase < wlen) ? lPhase : zerofp;


          // int32_t on = lPhase < transitionPoint ?  1 : 0;

          // word <<= 1;
          // word |= on;

          // //carrier phase
          // lPhase = lPhase + onefp;
          lPhase = (lPhase < wlen) ? lPhase : zerofp;
          int32_t amp = lPhase < transitionPoint ? FADE_REF : 0;
          int32_t y = amp >= lErr ? 1 : 0;
          lErr = (y ? volumePeak : 0) - amp + lErr;
          word <<= 1;
          word |= y;
          lPhase = lPhase + onefp;
        }
        *(bufferA ++) = word;
      }
      phase = lPhase;
      err0 = lErr;
      updateFade();
    }


    void ctrl(const Q16_16 v) override {
      pulseWidth = WvlenFPType(0.5) - (WvlenFPType(v) * WvlenFPType(0.38));
    }


    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "pulsesd";
    }



};
