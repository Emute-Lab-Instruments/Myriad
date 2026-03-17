#pragma once
#include "oscillatorModel.hpp"

class parasineSDOscillatorModel : public virtual oscillatorModel {
  public:

    parasineSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true;
      setClockModShift(1);
      // // Build table once - parabolic sine, 0..2048 range
      // for (int i = 0; i < 256; i++) {
      //   int p = i < 128 ? i : 255 - i;          // triangle 0→127→0
      //   int val = (p * (128 - p)) >> 4;          // parabola, peak = 256
      //   sine_table[i] = (i < 128) ? 1024 + val : 1024 - val;
      // }
        for (int i = 0; i < 256; i++) {
            float s = sinf(i * (2.0f * 3.14159265f / 256.0f));
            sine_table[i] = static_cast<uint16_t>(1024.0f + s * 1024.0f + 0.5f);
        }
      }

    inline void fillBuffer(uint32_t* bufferA) {
      // 16.16 fixed-point phase increment
      // At wlen=100: inc=167772. At wlen=600000: inc=27. Never zero.
      const uint32_t phase_inc = (256U << 16) / static_cast<uint32_t>(this->wavelen);

      uint32_t local_phase = phase;
      int32_t local_err = err;
      const int32_t sp = shape_pos_raw;
      const int32_t sn = shape_neg_raw;

      static int32_t FADE_REF = 1 << 11;

      const int32_t volumePeak = static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      );


      for (size_t i = 0; i < loopLength; ++i) {
        size_t word = 0U;
        for (size_t bit = 0U; bit < 32U; bit++) {
          uint32_t idx = (local_phase >> 16) & 0xFF;
          int32_t raw = sine_table[idx];

          // Asymmetric shape: scale deviation from midpoint
          int32_t dev = raw - 1024;
          int32_t amp = 1024 + ((dev * (dev >= 0 ? sp : sn)) >> 10);

          // Sigma-delta, fixed range
          int32_t y = amp >= local_err ? 1 : 0;
          local_err = local_err - amp + (y * volumePeak);
          word <<= 1;
          word |= y;

          local_phase += phase_inc;
        }
        *(bufferA + i) = word;
      }
      phase = local_phase;
      err = local_err;
      updateFade();
    }

    void ctrl(const Q16_16 v) override {
      int32_t vraw = v.raw();
      // shape as 10-bit fixed point, 1024 = 1.0
      shape_pos_raw = 1024 - ((vraw * 102) >> 16);   // 1.0 - 0.1*v
      shape_neg_raw = 1024 - ((vraw * 921) >> 16);    // 1.0 - 0.9*v  (921/1024 ≈ 0.9)
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "sinesd";
    }

  private:
    uint32_t phase = 0;
    int32_t err = 0;
    int32_t shape_pos_raw = 1024;
    int32_t shape_neg_raw = 1024;
    uint16_t sine_table[256];
};
