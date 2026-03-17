#pragma once
#include "oscillatorModel.hpp"

class sharkTeethSDOscillatorModel : public virtual oscillatorModel {
  public:
    using fptype = Fixed<20,11>;


    sharkTeethSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      const fptype wlenFP = fptype::from_int(wlen);

      int32_t local_phase = phase;     // Load once
      int32_t local_err = err0;

      fptype toothPhase =fptype::from_int(local_phase) * bite;

      toothPhase = toothPhase.fmod(wlenFP);


      //amplitude
      const bool isFading = fadeDirection != 0;
      int32_t volumePeak;
      if (isFading) {
        // If fading, compute the actual volume peak for this fade level
        volumePeak = static_cast<int32_t>(
            (static_cast<int64_t>(wlen) * fadeInvTable[fadeLevel]) >> 16
        );
      }else{
        volumePeak = wlen;
      }

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {

          local_phase = local_phase >= wlen ? 0 : local_phase; // wrap around


          if (toothPhase >= wlenFP) {
            toothPhase -= wlenFP;
          }
          int32_t amp = toothPhase.to_int();

          // int32_t doublePhase = local_phase << 1;
          // if (amp > doublePhase) {
          //   amp = doublePhase;
          // }

          if (local_phase >= (wlen >> 1)) {
            amp = 0;
          }




          const bool y = amp >= local_err ? 1 : 0;
          local_err = (y ? volumePeak : 0) - amp + local_err;


          word <<= 1;
          word |= y;

          local_phase++;
          toothPhase += bite;
        }

        *(bufferA + i) = word;

      }
      updateFade();
      phase = local_phase;   // Store once at end
      err0 = local_err;
    }

    void ctrl(const Q16_16 v) override
    {
      static fptype maxTeeth(19.f);
      static fptype minTeeth(1.f);
      bite = minTeeth + (fptype(v) * maxTeeth);
      bite = FixedPoint::floor(bite);

    }


    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "sdt10";
    }


  private:
    size_t phase=0;
    int32_t err0=0;

    fptype bite = fptype(1);
};
