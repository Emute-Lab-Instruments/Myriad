#pragma once
#include "oscillatorModel.hpp"

class formantSDOscillatorModel : public virtual oscillatorModel {
  public:

    formantSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true;
      setClockModShift(1);
      if (!tablesGenerated) {
        buildTables();
        tablesGenerated = true;
      }
    }

    void buildTables() {
        auto psin = [](float ph) -> float {
            return sinf(ph * (2.0f * 3.14159265f / 512.0f));
        };

        static const float aw[] = {0, 0.39f, 1.0f, 0.75f, 0.2f, 0.1f, 0.5f, 0.2f, 0.06f};
        static const float ew[] = {0, 1.0f, 0.2f, 0.1f, 0.05f, 0.2f, 0.75f, 0.5f, 0.38f};

        static float raw_a[512], raw_b[512];
        float min_a = 0, max_a = 0, min_b = 0, max_b = 0;

        for (int i = 0; i < 512; i++) {
            float sa = 0, sb = 0;
            for (int h = 1; h < 9; h++) {
                float s = psin(static_cast<float>(h * i));
                sa += s * aw[h];
                sb += s * ew[h];
            }
            raw_a[i] = sa;
            raw_b[i] = sb;
            if (sa < min_a) min_a = sa;
            if (sa > max_a) max_a = sa;
            if (sb < min_b) min_b = sb;
            if (sb > max_b) max_b = sb;
        }

        float range_a = max_a - min_a;
        float range_b = max_b - min_b;
        if (range_a == 0) range_a = 1;
        if (range_b == 0) range_b = 1;

        for (int i = 0; i < 512; i++) {
            table_a[i] = static_cast<uint16_t>(
                ((raw_a[i] - min_a) / range_a) * 2048.0f + 0.5f
            );
            table_b[i] = static_cast<uint16_t>(
                ((raw_b[i] - min_b) / range_b) * 2048.0f + 0.5f
            );
        }
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const uint32_t phase_inc =
          (512U << 16) / static_cast<uint32_t>(this->wavelen);

      uint32_t local_phase = phase;
      int32_t local_err = err;
      const int32_t local_morph = morph;

      static int32_t FADE_REF = 1 << 11;

      const int32_t volumePeak = static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      );


      for (size_t i = 0; i < loopLength; ++i) {
        size_t word = 0U;
        for (size_t bit = 0U; bit < 32U; bit++) {
          uint32_t idx = (local_phase >> 16) & 0x1FF;
          int32_t a = table_a[idx];
          int32_t b = table_b[idx];
          int32_t amp = a + (((b - a) * local_morph) >> 10);

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
      // v is 0..1 in Q16.16, map to 0..1024
      morph = v.raw() >> 6;
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "formant";
    }

    void reset() override {
      phase = 0;
      err = 0;
    }

    inline static bool tablesGenerated = false;

    private:
    uint32_t phase = 0;
    int32_t err = 0;
    int32_t morph = 0;
    inline static __not_in_flash("tables") uint16_t table_a[512];
    inline static __not_in_flash("tables") uint16_t table_b[512];
};
