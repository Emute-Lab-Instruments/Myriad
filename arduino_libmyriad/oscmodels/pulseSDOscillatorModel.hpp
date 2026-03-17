#pragma once
#include "oscillatorModel.hpp"

class pulseSDOscillatorModel : public virtual oscillatorModel {
  public:

    pulseSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    // inline void fillBuffer(uint32_t* bufferA) {
    //   const size_t wlen = this->wavelen;
    //   for (size_t i = 0; i < loopLength; ++i) {
    //     size_t word=0U;
    //     for(size_t bit=0U; bit < 32U; bit++) {
    //       phase = phase >= wlen ? 0 : phase; // wrap around

    //       // size_t amp = phase;
    //       size_t amp =  phase > pulselen ? 0 : wlen;


    //       int32_t y = amp >= err0 ? 1 : 0;
    //       err0 = (y ? wlen : 0) - amp + err0;

    //       word |= y;
    //       word <<= 1;

    //       phase++;
    //     }
    //     *(bufferA + i) = word;

    //   }
    // }

  inline void fillBuffer(uint32_t* bufferA) {
      const int32_t wlen = this->wavelen;

      int32_t lPhase = phase;
      int32_t lErr = err0;
      int32_t lLp = lp;

      for (size_t i = 0; i < loopLength; ++i) {
          uint32_t word = 0U;
          for (size_t bit = 0U; bit < 32U; bit++) {
              lPhase = lPhase >= wlen ? 0 : lPhase;

              // Raw pulse — just 0 or wlen
              int32_t amp = lPhase < pulselen ? wlen : 0;

              // One-pole lowpass: lp += (amp - lp) >> k
              lLp += (amp - lLp) >> lpShift;

              // Sigma-delta on filtered signal
              int32_t y = lLp >= lErr ? 1 : 0;
              lErr += y * wlen - lLp;

              word |= y;
              word <<= 1;
              lPhase++;
          }
          *(bufferA + i) = word;
      }

      phase = lPhase;
      err0 = lErr;
      lp = lLp;
  }

  // void ctrl(const Q16_16 v) override {
  //     using fptype = Fixed<1,30>;
  //     fptype pw = (fptype(1) - fptype(v)) * fptype(0.5f);
  //     pw = pw < fptype(0.01f) ? fptype(0.01f) : pw;
  //     pulselen = WvlenFPType(this->wavelen).mulWith(pw).to_int();
  //     // pw= std::max((1.f-v) * 0.5f, 0.01f);
  //     // pulselen = this->wavelen * pw;
  //   }

    void ctrl(const Q16_16 v) override {
        using fptype = Fixed<1,30>;
        fptype pw = (fptype(1) - fptype(v)) * fptype(0.5f);
        pw = pw < fptype(0.01f) ? fptype(0.01f) : pw;
        pulselen = WvlenFPType(this->wavelen).mulWith(pw).to_int();

        // More smoothing at high frequencies
        if (wavelen < 100) lpShift = 2;       // ~100kHz cutoff
        else if (wavelen < 500) lpShift = 3;  // ~50kHz
        else lpShift = 4;                      // ~25kHz
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "pulsesd";
    }


  private:
    size_t phase=0;
    bool y=0;
    int32_t err0=0;
    int pulselen=10000;
    int32_t amp=0;

    int32_t lp = 0;
    int32_t lpShift = 3;  // cutoff ~= bitrate / (2π × 2^k)


};
