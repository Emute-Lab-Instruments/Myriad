#pragma once
#include "oscillatorModel.hpp"

class sawOscillatorModel : public virtual oscillatorModel {
  public:

    sawOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;

      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {


      const int32_t wlen = this->wavelen;
      const int32_t pm_int = phaseMul >> 15;      // Integer part (2..11)
      const int32_t pm_frac = phaseMul & 0x7FFF;  // Fractional part

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
      int32_t local_phase = phase;     // Load once
      int32_t local_err0 = err0;       // Load once

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for (uint32_t bit = 0; bit < 32; ++bit) {
          if (local_phase>=wlen) {
            local_phase = 0U;
          }

          int32_t amp = local_phase * pm_int + ((local_phase * pm_frac) >> 15);

          if (amp >= wlen) {
            amp = 0;
          }

          int32_t y = amp >= local_err0 ? 1 : 0;
          local_err0 = (y ? volumePeak : 0) - amp + local_err0;

          word |= y;
          word <<= 1;

          local_phase++;
        }

        *(bufferA + i) = word;

      }

      updateFade();
      phase = local_phase;   // Store once at end
      err0 = local_err0;     // Store once at end

    }

    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<17,15>;
      fptype newPhaseMul = fptype(5) + (fptype(40).mulWith(v));
      phaseMul = newPhaseMul.raw();
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "saw";
    }


  private:
    int32_t phase=0;
    int32_t phaseMul = 2 << 15;
    bool y=0;
    int err0=0;
    size_t val=0;

};

// class sawOscillatorModel : public virtual oscillatorModel {
//   public:

//     sawOscillatorModel() : oscillatorModel(){
//       loopLength=16;
//       prog=bitbybit_program;

//       updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
//     }

//     inline void fillBuffer(uint32_t* bufferA) {
//       int32_t wlen = this->wavelen;
//       int32_t pm_int = phaseMul >> 15;      // Integer part (2..11)
//       int32_t pm_frac = phaseMul & 0x7FFF;  // Fractional part

//       int32_t local_phase = phase;     // Load once
//       int32_t local_err0 = err0;       // Load once

//       for (size_t i = 0; i < loopLength; ++i) {
//         size_t word=0U;
//         for (uint32_t bit = 0; bit < 32; ++bit) {
//           local_phase++;
//           if (local_phase>=wlen) {
//             local_phase = 0U;
//           }

//           int32_t amp = local_phase * pm_int + ((local_phase * pm_frac) >> 15);

//           if (amp >= wlen) {
//             amp = 0;
//           }

//           int32_t y = amp >= local_err0 ? 1 : 0;
//           local_err0 = (y ? wlen : 0) - amp + local_err0;

//           word |= y;
//           word <<= 1;
//         }

//         *(bufferA + i) = word;

//       }

//       phase = local_phase;   // Store once at end
//       err0 = local_err0;     // Store once at end

//     }

//     void ctrl(const Q16_16 v) override {
//       using fptype = Fixed<17,15>;
//       fptype newPhaseMul = fptype(5) + (fptype(40).mulWith(v));
//       phaseMul = newPhaseMul.raw();
//     }

//     pio_sm_config getBaseConfig(uint offset) {
//       return bitbybit_program_get_default_config(offset);
//     }

//     String getIdentifier() override {
//       return "saw";
//     }


//   private:
//     int32_t phase=0;
//     int32_t phaseMul = 2 << 15;
//     bool y=0;
//     int err0=0;
//     size_t val=0;

// };
