#pragma once
#include "oscillatorModel.hpp"

class triTeethOscillatorModel : public virtual oscillatorModel {
  public:
    using fptype = Fixed<22,9>;


    triTeethOscillatorModel() : oscillatorModel(){
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

      const fptype twoWlenFP = wlenFP + wlenFP;
      fptype local_toothPhase = toothPhase;
      fptype local_toothInc = toothInc >= fptype::from_int(0) ? bite : -bite;

      const int32_t wlenInt = (int32_t)wlen;
      int32_t triPhaseInt = local_phase * 2;
      bool triUp = triPhaseInt < wlenInt;
      int32_t triIncInt = 2;
      if (!triUp) {
        triPhaseInt = wlenInt - (triPhaseInt - wlenInt);
        triIncInt = -2;
      }



      static constexpr int32_t FADE_REF = 1 << 14;
      const int32_t ampScale = (FADE_REF << 16) / wlenInt;
      const int32_t volumePeak = static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      );

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {

          local_phase = local_phase >= wlen ? 0 : local_phase; // wrap around

          if (triPhaseInt > wlenInt) {
            triPhaseInt = 2 * wlenInt - triPhaseInt;
            triIncInt = -triIncInt;
          } else if (triPhaseInt < 0) {
            triPhaseInt = -triPhaseInt;
            triIncInt = -triIncInt;
          }

          if (local_toothPhase >= wlenFP) {
            local_toothPhase = twoWlenFP - local_toothPhase;
            local_toothInc = -local_toothInc;
          } else if (local_toothPhase < fptype::from_int(0)) {
            local_toothPhase = -local_toothPhase;
            local_toothInc = -local_toothInc;
          }
          int32_t raw_amp;

          if (local_toothPhase < fptype::from_int(triPhaseInt)) {
            raw_amp = triPhaseInt;
          } else {
            raw_amp = local_toothPhase.to_int();
          }

          const int32_t amp = (raw_amp * ampScale) >> 16;

          const bool y = amp >= local_err ? 1 : 0;
          local_err = (y ? volumePeak : 0) - amp + local_err;


          word <<= 1;
          word |= y;

          local_phase++;
          local_toothPhase += local_toothInc;
          triPhaseInt += triIncInt;
        }

        *(bufferA + i) = word;

      }
      updateFade();
      phase = local_phase;   // Store once at end
      err0 = local_err;
      toothPhase = local_toothPhase;
      toothInc = local_toothInc;
    }

    void ctrl(const Q16_16 v) override
    {
      static fptype maxTeeth(24.f);
      static fptype minTeeth(2.f);
      bite = minTeeth + (fptype(v) * maxTeeth);
      bite = FixedPoint::floor(bite) * minTeeth; //reusing the x2

    }


    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "trv10";
    }

    void reset() override {
      phase = 0;
      err0 = 0;
      toothPhase = fptype(0);
      toothInc = fptype(1);
    }

  private:
    size_t phase=0;
    int32_t err0=0;

    fptype bite = fptype(1);
    fptype toothPhase = fptype(0);
    fptype toothInc = fptype(1);  // sign encodes direction
};
