#pragma once
#include "oscillatorModel.hpp"

class bellSDOscillatorModel : public virtual oscillatorModel {
  public:
    bellSDOscillatorModel() : oscillatorModel(){
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
      // Parabolic sine: 0..2047 input = one cycle, returns -512..512
      auto psin = [](int32_t ph) -> int32_t {
        ph = ((ph % 2048) + 2048) % 2048;
        int32_t p = ph < 1024 ? ph : 2047 - ph;
        int32_t val = (p * (1024 - p)) >> 8;
        return ph < 1024 ? val : -val;
      };

      static int32_t raw_a[512], raw_b[512];
      int32_t min_a = 0, max_a = 0, min_b = 0, max_b = 0;

      for (int i = 0; i < 512; i++) {
        int32_t carrier = i * 4;  // 0..2044, maps to one cycle
        // Modulator: 3.5x carrier frequency
        int32_t mod_phase = (i * 14) % 2048;
        int32_t mod_val = psin(mod_phase);     // -512..512
        // Gentle: carrier + 0.3 * modulator
        int32_t ph_a = carrier + ((mod_val * 3) >> 1);
        raw_a[i] = psin(ph_a);
        // Harsh: carrier + 8.0 * modulator
        int32_t ph_b = carrier + (mod_val << 4);
        raw_b[i] = psin(ph_b);
      }

      for (int i = 0; i < 512; i++) {
        if (raw_a[i] < min_a) min_a = raw_a[i];
        if (raw_a[i] > max_a) max_a = raw_a[i];
        if (raw_b[i] < min_b) min_b = raw_b[i];
        if (raw_b[i] > max_b) max_b = raw_b[i];
      }
      int32_t range_a = max_a - min_a;
      int32_t range_b = max_b - min_b;
      if (range_a == 0) range_a = 1;
      if (range_b == 0) range_b = 1;

      for (int i = 0; i < 512; i++) {
        table_a[i] = static_cast<uint16_t>(
            ((raw_a[i] - min_a) * 2048) / range_a
        );
        table_b[i] = static_cast<uint16_t>(
            ((raw_b[i] - min_b) * 2048) / range_b
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
      morph = v.raw() >> 6;
    }
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }
    String getIdentifier() override {
      return "metalic";
    }
    void reset() override {
      phase = 0;
      err = 0;
    }
  private:
    uint32_t phase = 0;
    int32_t err = 0;
    int32_t morph = 0;
    inline static __not_in_flash("tables") uint16_t table_a[512];
    inline static __not_in_flash("tables") uint16_t table_b[512];
    inline static bool tablesGenerated = false;
};
