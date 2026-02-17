#pragma once

#include "hardware/pio.h"
#include <memory>
#include <array>
#include <functional>
#include "oscDisplayModes.hpp"
#include "clockfreq.h"
#include <limits>
#include "oscmodels/oscillatorModel.hpp"

using WvlenFPType = Fixed<20,11>;




class silentOscillatorModel : public oscillatorModel {
public:
  silentOscillatorModel() : oscillatorModel() {
    loopLength=bufferSize;
    prog=pin_ctrl_program;

  }
  inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < bufferSize; ++i) {
        *(bufferA + i) = silentOscillatorModel::halfWaveLen;
    }
  }

  String getIdentifier() override {
    return "sil";
  }

private:
  const size_t bufferSize=16;
  static constexpr uint32_t halfWaveLen = sampleClock / 100000;
};



  


class sawOscillatorModel : public virtual oscillatorModel {
  public:

    sawOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;

      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA) {
      int32_t wlen = this->wavelen;
      int32_t pm_int = phaseMul >> 15;      // Integer part (2..11)
      int32_t pm_frac = phaseMul & 0x7FFF;  // Fractional part
            
      int32_t local_phase = phase;     // Load once
      int32_t local_err0 = err0;       // Load once
            
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for (uint32_t bit = 0; bit < 32; ++bit) {
          local_phase++;
          if (local_phase>=wlen) {
            local_phase = 0U;
          }

          int32_t amp = local_phase * pm_int + ((local_phase * pm_frac) >> 15);        

          if (amp >= wlen) {
            amp = 0;
          }

          int32_t y = amp >= local_err0 ? 1 : 0;
          local_err0 = (y ? wlen : 0) - amp + local_err0;

          word |= y;
          word <<= 1;
        } 

        *(bufferA + i) = word;

      }

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

volatile bool triTableGenerated=false;
static int32_t __not_in_flash("tri") MULTIPLIER_TABLE_RISING[1000];
static int32_t __not_in_flash("tri") MULTIPLIER_TABLE_RISING_INV[1000];
static int32_t __not_in_flash("tri") MULTIPLIER_TABLE_FALLING[1000];

class triOscillatorModel : public virtual oscillatorModel {
  public:

    triOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      if (!triTableGenerated) {
        triTableGenerated=true;
        for(size_t i = 0; i < 1000; ++i) {
          float v = i/1000.f;
          float gradRising = 2.f + (v * 7.f);
          float gradRisingInv = 1.f/gradRising; 
          float gradFalling = 1.f/(1.f - (1.f / gradRising));
          phaseRisingMul = static_cast<size_t>((gradRising * qfpMul) + 0.5f);
          phaseRisingInvMul = static_cast<size_t>((gradRisingInv * qfpMul) + 0.5f);
          phaseFallingMul = static_cast<size_t>((gradFalling * qfpMul) + 0.5f);
          MULTIPLIER_TABLE_RISING[i] = phaseRisingMul;
          MULTIPLIER_TABLE_RISING_INV[i] = phaseRisingInvMul;
          MULTIPLIER_TABLE_FALLING[i] = phaseFallingMul;
        }
      }
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {      
        const int32_t wlen = this->wavelen;

        const int32_t triPeakPoint = (wlen * phaseRisingInvMul) >> qfp;

        const uint32_t rising_int = phaseRisingMul >> qfp;
        const uint32_t rising_frac = phaseRisingMul & 0x3FFF;

        // Compute the ACTUAL peak amplitude using the same math as the rising side
        const int32_t peakAmp = triPeakPoint * rising_int + 
                                (((uint32_t)triPeakPoint * rising_frac) >> qfp);

        // Derive falling multiplier from peakAmp so peak and zero-crossing are exact
        const int32_t fallingSpan = wlen - triPeakPoint;
        const int32_t derivedFallingMul = (static_cast<int64_t>(peakAmp) << qfp) / fallingSpan;
        const uint32_t falling_int = derivedFallingMul >> qfp;
        const uint32_t falling_frac = derivedFallingMul & 0x3FFF;

        int32_t local_phase = phase;     // Load once
        int32_t local_err0 = err0;       // Load once


        for (size_t i = 0; i < loopLength; ++i) {
            uint32_t word = 0U;
            for (size_t bit = 0U; bit < 32U; bit++) {
                local_phase = local_phase >= wlen ? 0 : local_phase;
                int32_t amp;

                if (local_phase <= triPeakPoint) {
                    amp = local_phase * rising_int + (((uint32_t)local_phase * rising_frac) >> qfp);
                } else {
                    const int32_t delta = local_phase - triPeakPoint;            
                    const int32_t fallingPhase = delta * falling_int + 
                                                (((uint32_t)delta * falling_frac) >> qfp);
                    amp = peakAmp - fallingPhase;
                }

                // Clamp just in case of residual rounding at the very last sample
                amp = amp < 0 ? 0 : amp;

                int32_t y = amp >= local_err0 ? 1 : 0;
                local_err0 = (y ? peakAmp : 0) - amp + local_err0;

                word <<= 1;
                word |= y;

                local_phase++;
            }
            *(bufferA + i) = word;
        }

        phase = local_phase;   // Store once at end
        err0 = local_err0;     // Store once at end
    }

    void ctrl(const Q16_16 v) override {
      const size_t index = (v * Q16_16(900)).to_int();  //static_cast<size_t>(v * 900.f + 0.5f);
      if (index != lastPW || wavelen != lastWavelen) {
        lastPW = index;
        lastWavelen = wavelen;
        phaseRisingMul = MULTIPLIER_TABLE_RISING[index];
        phaseRisingInvMul = MULTIPLIER_TABLE_RISING_INV[index];
        phaseFallingMul = MULTIPLIER_TABLE_FALLING[index];
        risingInc = phaseRisingMul;
        fallingInc = -phaseFallingMul;  
        // ctrlChange=true;    
        // amp_fp=0;
      }
      //   bitmul = static_cast<uint32_t>(1U << 15) * v * 3.9f;
      // Serial.printf("%zu, phaseRisingMul: %zu, phaseRisingInvMul: %zu, phaseFallingMul: %zu\n", index, phaseRisingMul,phaseRisingInvMul, phaseFallingMul);
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "tri";
    }
  
  private:
    int32_t phase=0;
    size_t phaseMul = 0;
    bool y=0;
    int32_t err0=0;
    size_t val=0;
    
    int32_t phaseRisingMul=2;
    int32_t phaseRisingInvMul=2;
    int32_t phaseFallingMul=2;
    // int32_t triPeakPoint = 1;
    int32_t risingInc =1;
    int32_t fallingInc = -1;      
    int32_t amp_fp = 0; // accumulator for amplitude in fixed point

    static constexpr size_t qfp = 14U;
    static constexpr float qfpMul = 1U << qfp;

    size_t lastPW = 0;
    size_t lastWavelen = 1;
    volatile bool ctrlChange = false;
    size_t bitshift = 0;
    uint32_t bitmul = 1U; // multiplier for bit shift, used to scale the output
    size_t lastWord=0, lastWord2=0, lastWord3=0, lastWord4=0, lastWord5=0, lastWord6=0, lastWord7=0, lastWord8=0;

};

class triOscillator2Model : public virtual oscillatorModel {
public:
    triOscillator2Model() : oscillatorModel() {
        loopLength = 16;
        prog = bitbybit_program;
        // setClockModShift(1);
        updateBufferInSyncWithDMA = true;
    }

    inline void fillBuffer(uint32_t* bufferA) {
        const int32_t wlen = this->wavelen;

        // Precompute into pending slots (cheap — runs outside tight loop)
        if (wlen != cachedWlen || ctrlChanged) {
            cachedWlen = wlen;
            ctrlChanged = false;
            recompute(wlen);
            pendingApply = true;
        }

        int32_t lAmp = amp;
        int32_t lPhase = phase;
        int32_t lErr = err0;
        int32_t lInc = inc;
        int32_t lPeak = peakAmp;
        int32_t lPeakPt = peakPoint;
        int32_t lRiseInc = risingInc;
        int32_t lFallInc = fallingInc;

        for (size_t i = 0; i < loopLength; ++i) {
            uint32_t word = 0U;
            for (size_t bit = 0U; bit < 32U; bit++) {

                int32_t y = (lAmp >= lErr) ? 1 : 0;
                lErr += y * lPeak - lAmp;

                word |= y;
                word <<= 1;

                lPhase++;
                lAmp += lInc;

                if (lPhase >= wlen) [[unlikely]] {
                    lPhase = 0;
                    lAmp = 0;
                    
                    // Apply pending params at safe point
                    if (pendingApply) [[unlikely]] {
                        pendingApply = false;
                        lPeak = pendPeakAmp;
                        lPeakPt = pendPeakPoint;
                        lRiseInc = pendRisingInc;
                        lFallInc = pendFallingInc;
                        // Clamp error to new range
                        if (lErr > lPeak) lErr = lPeak;
                        if (lErr < 0) lErr = 0;
                    }
                    
                    lInc = lRiseInc;
                } else if (lPhase == lPeakPt) [[unlikely]] {
                    lAmp = lPeak;
                    lInc = -lFallInc;
                }
            }
            *(bufferA + i) = word;
        }

        // Store back
        amp = lAmp;
        phase = lPhase;
        err0 = lErr;
        inc = lInc;
        peakAmp = lPeak;
        peakPoint = lPeakPt;
        risingInc = lRiseInc;
        fallingInc = lFallInc;
    }

    void ctrl(const Q16_16 v) override {
        using fptype = Fixed<18, 14>;
        gradFP = (fptype(2) + fptype(7).mulWith(v)).raw();
        ctrlChanged = true;
    }

    pio_sm_config getBaseConfig(uint offset) {
        return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
        return "tri";
    }

private:
    void recompute(int32_t wlen) {
        int32_t pp = (wlen << QFP) / gradFP;
        if (pp < 2) pp = 2;
        if (pp >= wlen) pp = wlen - 1;
        const int32_t fl = wlen - pp;

        int32_t ri = AMP_SCALE / pp;
        int32_t pa = ri * pp;
        int32_t fi = pa / fl;

        pendPeakPoint = pp;
        pendPeakAmp = pa;
        pendRisingInc = ri;
        pendFallingInc = fi;
    }

    // Live state
    int32_t phase = 0;
    int32_t amp = 0;
    int32_t err0 = 0;
    int32_t inc = 0;

    // Active params
    int32_t peakPoint = 1;
    int32_t peakAmp = 0;
    int32_t risingInc = 0;
    int32_t fallingInc = 0;

    // Pending params (applied at wrap)
    int32_t pendPeakPoint = 1;
    int32_t pendPeakAmp = 0;
    int32_t pendRisingInc = 0;
    int32_t pendFallingInc = 0;
    bool pendingApply = true;

    int32_t gradFP = 2 << 14;
    int32_t cachedWlen = 0;
    bool ctrlChanged = true;

    int32_t QFP = 14;
    int32_t AMP_SCALE = 1 << 24;
}; 

  
class noiseOscillatorModelSD : public virtual oscillatorModel {
  public:
    noiseOscillatorModelSD() : oscillatorModel(){
      loopLength=8;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true;
      setClockModShift(1);
    }
    
    inline void fillBuffer(uint32_t* bufferA) {
      for (size_t i = 0; i < loopLength; ++i) {
        uint32_t word = 0U;
        
        for(uint32_t bit = 0; bit < 32; ++bit) {
          if (counter == 0) {
            on = !on;  // Toggle state
            Q16_16 rnd = Q16_16::random_hw(Q16_16(0),randMult);   
            counter = 1 + (WvlenFPType(wavelen) * WvlenFPType(0.01f)).mulWith(rnd).to_int();                     
          }
          counter--;

          word <<= 1;
          word |= on;
        }
        
        *(bufferA + i) = word;
      }
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }
    
    void ctrl(const Q16_16 v) override {
      randMult = Q16_16(-1) + (v.mul_split(v).mul_split(Q16_16(500)));
    }
  
    String getIdentifier() override {
      return "n2";
    }
  
  private:
    uint32_t densityThreshold = 0x80000000;  // 50% density by default
    Q16_16 randMult = Q16_16(100);
    bool on=0;
    size_t counter=0;
};


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
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          lPhase = lPhase >= wlen ? 0 : lPhase; // wrap around

        
          int32_t amp = lPhase < midphase ? 
                lPhase << 1 
                : 
                wlen - ((lPhase-midphase) << 1); //tri wave

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
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wlen ? 0 : phase; // wrap around

          size_t amp = phase << 1;
          amp =  amp > wlen ? phase : amp; 


          const bool y = amp >= thr ? 1 : 0;
          size_t err0 = (y ? wlen : 0) - amp + thr;

          // thr = ((err0 * alpha) + (thr * alpha_inv)) >> qfp;
          thr = ((err0 * alpha) + (thr)) >> qfp;

          word <<= 1;
          word |= y;

          phase++;
        }
        *(bufferA + i) = word;

      }
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

class whiteNoiseOscillatorModel : public virtual oscillatorModel {
  public:


    whiteNoiseOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      setClockModShift(1);
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      wv20 = WvlenFPType(getWavelenAtFrequency(20.f));
    }

    inline void fillBuffer(uint32_t* bufferA) {
      for (size_t i = 0; i < loopLength; ++i) {
        if ((rand() % 200000) > alpha) {
          int inc = beta>>2;
          acc += (get_rand_32() & 1) ? inc : -inc;
        }
        if (rand() % 1000 > beta)
          acc ^= (rand() & 127U);
        
        *(bufferA + i) = acc;
      }

    }

    void ctrl(const Q16_16 v) override {
      alpha = (v * Q16_16(10000)).to_int() + 190000;
      // Serial.printf("alpha: %d\n", alpha);
      beta = 500 + ( (WvlenFPType(this->wavelen) / wv20) * WvlenFPType(400)).to_int();
      // Serial.printf("alpha: %d, beta: %d\n", alpha, beta);
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "wn";
    }
  
  private:
    bool y=0;
    int err0=0;

   int integrator = 0, acc=0;  
   int alpha=255, beta=0;
   WvlenFPType wv20;
};


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


// class squareSineSDOscillatorModel : public virtual oscillatorModel {
// public:
//     squareSineSDOscillatorModel() : oscillatorModel() {
//         loopLength = 16;
//         prog = bitbybit_program;
//         updateBufferInSyncWithDMA = true;
//     }

//     inline void fillBuffer(uint32_t* bufferA) {
//         const int32_t wlen = this->wavelen;
        
//         int32_t lPhase = phase;
//         int32_t lErr = err0;
//         int32_t lLp1 = lp1;
//         int32_t lLp2 = lp2;
//         int32_t lLp3 = lp3;
//         const int32_t k = lpShift;
//         const int32_t hw = halfWlen;

//         for (size_t i = 0; i < loopLength; ++i) {
//             uint32_t word = 0U;
//             for (size_t bit = 0U; bit < 32U; bit++) {
//                 lPhase = lPhase >= wlen ? 0 : lPhase;

//                 int32_t amp = lPhase < hw ? AMP : 0;

//                 lLp1 += (amp - lLp1) >> k;
//                 lLp2 += (lLp1 - lLp2) >> k;
//                 lLp3 += (lLp2 - lLp3) >> k;

//                 int32_t y = lLp3 >= lErr ? 1 : 0;
//                 lErr += y * AMP - lLp3;

//                 word |= y;
//                 word <<= 1;
//                 lPhase++;
//             }
//             *(bufferA + i) = word;
//         }

//         phase = lPhase;
//         err0 = lErr;
//         lp1 = lLp1;
//         lp2 = lLp2;
//         lp3 = lLp3;
//     }

//     void ctrl(const Q16_16 v) override {
//         halfWlen = this->wavelen >> 1;

//         int32_t log2wl = 31 - __builtin_clz(this->wavelen);  

//         int32_t offset = 3 - (v * Q16_16(5)).to_int();
//         lpShift = log2wl + offset;

//         if (lpShift < 1) lpShift = 1;
//         if (lpShift > 12) lpShift = 12;
//     }

//     pio_sm_config getBaseConfig(uint offset) {
//         return bitbybit_program_get_default_config(offset);
//     }

//     String getIdentifier() override {
//         return "sinesd";
//     }

// private:
//     static constexpr int32_t AMP = 1 << 20;  // ~1M levels, constant across all pitches

//     int32_t phase = 0;
//     int32_t err0 = 0;
//     int32_t lp1 = 0;
//     int32_t lp2 = 0;
//     int32_t lp3 = 0;
//     int32_t lpShift = 6;
//     int32_t halfWlen = 1;
// };

class squareSineSDOscillatorModel : public virtual oscillatorModel {
public:
    squareSineSDOscillatorModel() : oscillatorModel() {
        loopLength = 16;
        prog = bitbybit_program;
        updateBufferInSyncWithDMA = true;
    }

inline void fillBuffer(uint32_t* bufferA) {
        const int32_t wlen = this->wavelen;

        if (wlen != cachedWlen) {
            cachedWlen = wlen;
            // Target cutoff ≈ 3 × fundamental
            // cutoff ≈ 2.5MHz / (2π × 2^k)
            // 2^k ≈ wlen / (3 × 2π) ≈ wlen / 19
            // k ≈ log2(wlen) - 4
            int32_t log2wl = 31 - __builtin_clz(wlen);
            lpShift = log2wl - 4;
            if (lpShift < 1) lpShift = 1;
            if (lpShift > 12) lpShift = 12;
        }
        
        int32_t lPhase = phase;
        int32_t lErr = err0;
        int32_t lLp1 = lp1;
        int32_t lLp2 = lp2;
        int32_t lLp3 = lp3;
        int32_t lLp4 = lp4;
        const int32_t k = lpShift;
        const int32_t pw = pulselen;

        for (size_t i = 0; i < loopLength; ++i) {
            uint32_t word = 0U;
            for (size_t bit = 0U; bit < 32U; bit++) {
                lPhase = lPhase >= wlen ? 0 : lPhase;

                int32_t amp = lPhase < pw ? AMP : 0;

                lLp1 += (amp - lLp1) >> k;
                lLp2 += (lLp1 - lLp2) >> k;
                lLp3 += (lLp2 - lLp3) >> k;
                lLp4 += (lLp3 - lLp4) >> k;

                int32_t y = lLp4 >= lErr ? 1 : 0;
                lErr += y * AMP - lLp4;

                word |= y;
                word <<= 1;
                lPhase++;
            }
            *(bufferA + i) = word;
        }

        phase = lPhase;
        err0 = lErr;
        lp1 = lLp1;
        lp2 = lLp2;
        lp3 = lLp3;
        lp4 = lLp4;
    }

    void ctrl(const Q16_16 v) override {
        using fptype = Fixed<1, 30>;
        fptype pw = (fptype(1) - fptype(v)) * fptype(0.5f);
        pw = pw < fptype(0.01f) ? fptype(0.01f) : pw;
        pulselen = WvlenFPType(this->wavelen).mulWith(pw).to_int();
        if (pulselen < 1) pulselen = 1;
    }

    pio_sm_config getBaseConfig(uint offset) {
        return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
        return "sinesd";
    }

private:
    static constexpr int32_t AMP = 1 << 20;

    int32_t phase = 0;
    int32_t err0 = 0;
    int32_t lp1 = 0;
    int32_t lp2 = 0;
    int32_t lp3 = 0;
    int32_t lp4 = 0;
    int32_t lpShift = 6;
    int32_t pulselen = 1;
    int32_t cachedWlen = 0;
};

class expPulseSDOscillatorModel : public virtual oscillatorModel {
  private:
    // size_t loopLengthBits=0;
  public:


    expPulseSDOscillatorModel() : oscillatorModel(){
      loopLength=8;
      // loopLengthBits = 1 << __builtin_clz(loopLength);
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    Fixed<16,16> counterFP = Q16_16(0);
    Fixed<16,16> targetIncFP = Q16_16(1000);
    Fixed<16,16> targetIncFPOrg = Q16_16(1000);
    Fixed<16,16> targetFP= Q16_16(1000);
    Fixed<16,16> targIncMul = Q16_16(1.1f);
    bool b1=0;

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        // size_t loopbits = 1;
        for (size_t bit = 0U; bit < 32U; bit++) {
          // phase = phase >= wlen ? 0 : phase; // wrap around
          uint32_t wrap = (phase < wlen);  // 1 if in range, 0 if needs wrap
          phase *= wrap;  // Becomes 0 if phase >= wlen

          [[unlikely]]
          if (phase == 0) {
            counterFP=Q16_16(0);
            targetFP = targetIncFPOrg;
            targetIncFP = targetIncFPOrg;
            b1=0;

          }

          [[unlikely]]
          if (counterFP >= targetFP) {
            b1 = !b1;
            counterFP = Q16_16(0);
            targetFP += targetIncFP;
            targetIncFP *= targIncMul;
          }
          counterFP += Q16_16(1);

          // word |= loopbits & -(int32_t)(b1);  // Set bit if either is true
          word <<= 1;
          word |= b1;  

          phase++;

          // loopbits <<= 1;
        }
        *(bufferA ++) = word;

      }
    }


    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<16,16>;
      fptype vinv = fptype(1) - v;
      fptype targmax = fptype(1/32.f).mulWith(WvlenFPType(this->wavelen));
      targetIncFPOrg = ((vinv * targmax) + fptype(10));
      // targetIncFPOrg = targetIncFP;
      targIncMul = (vinv * Q16_16(0.4f)) + Q16_16(1);
      
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "exp1";
    }

  
  private:
    size_t phase=0;
    bool y=0;
    int32_t err0=0;


    int pulselen=10000;

};

class pulsePMSDOscillatorModel : public virtual oscillatorModel {
  private:

    WvlenFPType phase = WvlenFPType(0);

    WvlenFPType modphase = WvlenFPType(0);
    WvlenFPType phaseIncLow=WvlenFPType(1);
    WvlenFPType phaseIncHigh=WvlenFPType(1);
    // int32_t err0=0;

  public:


    pulsePMSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      // setClockModShift(1);
    }


    inline void fillBuffer(uint32_t* bufferA) {
      const WvlenFPType wlen = WvlenFPType(this->wavelen);
      // const int32_t local_wlen = this->wavelen;
      const WvlenFPType wlenHalf = WvlenFPType(this->wavelen>>1);
      const WvlenFPType modwlen = WvlenFPType(3.9f) * wlen;
      const WvlenFPType modwlenPW = modwlen.div_pow2(1);
      static WvlenFPType zerofp = WvlenFPType(0);
      static WvlenFPType onefp = WvlenFPType(1);
      WvlenFPType phaseIncMul = WvlenFPType(1.0f);
      WvlenFPType phaseIncMulInc = WvlenFPType(0.1f);

      // int32_t SD_POSITIVE = 1 << 14;
      // int32_t SD_NEGATIVE = -SD_POSITIVE;
  

      // int32_t local_err0 = err0;

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for (size_t bit = 0U; bit < 32U; bit++) {

          phase = (phase < wlen) ? phase : zerofp;  

          int32_t on = phase > wlenHalf ?  1 : 0;
          // int32_t desired = (phase > wlenHalf) ? SD_POSITIVE : SD_NEGATIVE;
          
          // First-order sigma-delta: integrate error, threshold, feedback
          // local_err0 += desired;
          // uint32_t on;
          // if (local_err0 > 0) {
          //   on = 1;
          //   local_err0 -= SD_POSITIVE;  // feedback
          // } else {
          //   on = 0;
          //   local_err0 -= SD_NEGATIVE;  // feedback (adds since NEGATIVE is negative)
          // }

          word <<= 1;
          word |= on;

           
          //modulator
          modphase = (modphase < modwlen) ? modphase : zerofp;  
          WvlenFPType phaseInc = modphase > modwlenPW ?  phaseIncLow : phaseIncHigh;


          //carrier phase 
          phase = phase + phaseInc;

          //modulation phase
          modphase += onefp;
          

        }

        *(bufferA ++) = word;
      }
      // err0 = local_err0;
    }


    void ctrl(const Q16_16 v) override {
      WvlenFPType modIdx = WvlenFPType(0.05f) + WvlenFPType(0.55f).mulWith(v);
      phaseIncLow = WvlenFPType(1) - modIdx;
      phaseIncHigh = WvlenFPType(1) + modIdx;
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "slide";
    }

  

};

// class parasineSDOscillatorModel : public virtual oscillatorModel {
//   public:

//     parasineSDOscillatorModel() : oscillatorModel(){
//       loopLength=16;
//       prog=bitbybit_program;
//       updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
//       setClockModShift(1);
//     }

//     // inline void fillBuffer(uint32_t* bufferA) {
//     //   const size_t wlen = this->wavelen;
//     //   const size_t half_wlen = wlen >> 1;
      
//     //   const int32_t inv_half_wlen_x4 = (static_cast<int64_t>(4) << WvlenFPType::FRACTIONAL_BITS) / half_wlen;
      
//     //   for (size_t i = 0; i < loopLength; ++i) {
//     //     size_t word = 0U;
//     //     for(size_t bit = 0U; bit < 32U; bit++) {
//     //       phase = phase >= wlen ? 0 : phase; // wrap around
          
//     //       // Parabolic sine approximation with asymmetric shaping
//     //       int32_t sine_val;
//     //       if (phase < half_wlen) {
//     //         int32_t temp = phase * (half_wlen - phase);
//     //         int32_t base = (temp * inv_half_wlen_x4) >> WvlenFPType::FRACTIONAL_BITS;
//     //         sine_val = (shape_pos.raw() * base) >> WvlenFPType::FRACTIONAL_BITS;
//     //       } else {
//     //         size_t p = phase - half_wlen;
//     //         int32_t temp = p * (half_wlen - p);
//     //         int32_t base = (temp * inv_half_wlen_x4) >> WvlenFPType::FRACTIONAL_BITS;
//     //         sine_val = -((shape_neg.raw() * base) >> WvlenFPType::FRACTIONAL_BITS);
//     //       }
          
//     //       // Convert from [-half_wlen, +half_wlen] to [0, wlen]
//     //       auto amp = (half_wlen + sine_val);
          
//     //       int32_t y = amp >= err0 ? 1 : 0;
//     //       err0 = (y ? wlen : 0) - amp + err0;
//     //       word <<= 1;
//     //       word |= y;

//     //       phase++;
//     //     }
//     //     *(bufferA + i) = word;
//     //   }
//     // }
//     inline void fillBuffer(uint32_t* bufferA) {
//       const size_t wlen = this->wavelen;
//       const size_t half_wlen = wlen >> 1;
      
//       // Precompute reciprocal: (4 / half_wlen) as fixed-point
//       // This replaces TWO divisions per bit with ONE precomputed multiply
//       const int32_t inv_half_wlen_x4 = (static_cast<int64_t>(4) << WvlenFPType::FRACTIONAL_BITS) / half_wlen;
      
//       for (size_t i = 0; i < loopLength; ++i) {
//         size_t word = 0U;
//         for(size_t bit = 0U; bit < 32U; bit++) {
//           phase = phase >= wlen ? 0 : phase; // wrap around
          
//           // Parabolic sine approximation with asymmetric shaping
//           int32_t sine_val;
//           if (phase < half_wlen) {
//             // base = (4 * phase * (half_wlen - phase)) / half_wlen
//             // becomes: base = (phase * (half_wlen - phase) * inv_half_wlen_x4) >> FRAC_BITS
//             int32_t temp = phase * (half_wlen - phase);
//             int32_t base = (temp * inv_half_wlen_x4) >> WvlenFPType::FRACTIONAL_BITS;
//             sine_val = (shape_pos.raw() * base) >> WvlenFPType::FRACTIONAL_BITS;
//           } else {
//             size_t p = phase - half_wlen;
//             int32_t temp = p * (half_wlen - p);
//             int32_t base = (temp * inv_half_wlen_x4) >> WvlenFPType::FRACTIONAL_BITS;
//             sine_val = -((shape_neg.raw() * base) >> WvlenFPType::FRACTIONAL_BITS);
//           }
          
//           // Convert from [-half_wlen, +half_wlen] to [0, wlen]
//           auto amp = (half_wlen + sine_val);
          
//           int32_t y = amp >= err0 ? 1 : 0;
//           err0 = (y ? wlen : 0) - amp + err0;
//           word <<= 1;
//           word |= y;

//           phase++;
//         }
//         *(bufferA + i) = word;
//       }
//     }

//     void ctrl(const Q16_16 v) override {
//       shape_pos = WvlenFPType(1.f) - WvlenFPType(0.1f).mulWith(v);
//       shape_neg = WvlenFPType(1.f) - WvlenFPType(0.9f).mulWith(v);
//     }
      
  
//     pio_sm_config getBaseConfig(uint offset) {
//       return bitbybit_program_get_default_config(offset);
//     }

//     String getIdentifier() override {
//       return "sinesd";
//     }

  
//   private:
//     int phase = 0;
//     bool y=0;
//     int err0 = 0;

//     WvlenFPType shape_pos = WvlenFPType(1.0);  
//     WvlenFPType shape_neg = WvlenFPType(1.0);  


// };

class parasineSDOscillatorModel : public virtual oscillatorModel {
  public:

    parasineSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true;
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const int32_t wlen = static_cast<int32_t>(this->wavelen);
      const int32_t half_wlen = wlen >> 1;
      
      // Rescale phase and err0 when wavelength changes
      if (wlen != prev_wlen && prev_wlen > 0) {
          phase = static_cast<int32_t>(
              (static_cast<uint32_t>(phase) * static_cast<uint32_t>(wlen)) 
              / static_cast<uint32_t>(prev_wlen)
          );
          err0 = static_cast<int32_t>(
              (static_cast<int64_t>(err0) * wlen) / prev_wlen
          );
      }
      prev_wlen = wlen;

      static int RECIP_SHIFT = 15;
      const int32_t inv_half_wlen_x4 = (4 << RECIP_SHIFT) / half_wlen;

      int32_t local_err0 = err0;
      int32_t local_phase = phase;
      
      const bool large_wlen = (half_wlen > 46340);
      
      for (size_t i = 0; i < loopLength; ++i) {
          size_t word = 0U;
          for(size_t bit = 0U; bit < 32U; bit++) {
              local_phase = local_phase >= wlen ? 0 : local_phase;
              
              int32_t sine_val;
              if (local_phase < half_wlen) {
                  int32_t p = local_phase;
                  int32_t temp = p * (half_wlen - p);
                  int32_t base = large_wlen 
                      ? ((temp >> 2) * inv_half_wlen_x4) >> 13
                      : ((temp >> 1) * inv_half_wlen_x4) >> 14;
                  sine_val = (shape_pos.raw() * base) >> 11;
              } else {
                  int32_t p = local_phase - half_wlen;
                  int32_t temp = p * (half_wlen - p);
                  int32_t base = large_wlen
                      ? ((temp >> 2) * inv_half_wlen_x4) >> 13
                      : ((temp >> 1) * inv_half_wlen_x4) >> 14;
                  sine_val = -((shape_neg.raw() * base) >> 11);
              }
              
              int32_t amp = half_wlen + sine_val;
              
              int32_t y = amp >= local_err0 ? 1 : 0;
              local_err0 = (y ? wlen : 0) - amp + local_err0;
              word <<= 1;
              word |= y;
              
              local_phase++;
          }
          *(bufferA + i) = word;
      }
      err0 = local_err0;
      phase = local_phase;
    }

    void ctrl(const Q16_16 v) override {
      shape_pos = WvlenFPType(1.f) - WvlenFPType(0.1f).mulWith(v);
      shape_neg = WvlenFPType(1.f) - WvlenFPType(0.9f).mulWith(v);
    }
      
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "sinesd";
    }

  private:
    int32_t phase = 0;
    int32_t err0 = 0;
    int32_t prev_wlen = 0;

    WvlenFPType shape_pos = WvlenFPType(1.0);  
    WvlenFPType shape_neg = WvlenFPType(1.0);  
};

class pulsePWOscillatorModel : public virtual oscillatorModel {
  private:

    WvlenFPType phase = WvlenFPType(0);
    WvlenFPType pulseWidth = WvlenFPType(0.5);

    int32_t transitionCounter=0;
    int32_t lastOn=0;

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
      const int32_t transitionCount = wlen < WvlenFPType(1250) ?  wlen.to_int() >> 4 : 0; // Number of samples to randomize after a transition, adjust as needed
      // const WvlenFPType wlenHalf = WvlenFPType(this->wavelen>>1);
      static WvlenFPType zerofp = WvlenFPType(0);
      static WvlenFPType onefp = WvlenFPType(1);



      WvlenFPType lPhase = phase;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for (size_t bit = 0U; bit < 32U; bit++) {

          //wrap phase
          lPhase = (lPhase < wlen) ? lPhase : zerofp;  


          int32_t on = lPhase < transitionPoint ?  1 : 0;

          word <<= 1;
          word |= on;

          //carrier phase 
          lPhase = lPhase + onefp;
        }

        *(bufferA ++) = word;
      }
      phase = lPhase;
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


class expPulse2SDOscillatorModel : public virtual oscillatorModel {
  private:
    size_t loopLengthBits=0;
  public:


    expPulse2SDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    Fixed<16,16> counterFP = Q16_16(0);
    Fixed<16,16> targetIncFP = Q16_16(1000);
    Fixed<16,16> targetIncFPOrg = Q16_16(1000);
    Fixed<16,16> targetFP= Q16_16(1000);
    Fixed<16,16> targIncMul = Q16_16(1.1f);

    // Fixed<16,16> counterFPPhaseInc = Q16_16(1);
    // Fixed<16,16> counterFPPhaseIncInc = Q16_16(1.01f);
    // Fixed<16,16> counterFPPhaseIncIncPos = Q16_16(1.01f);
    // Fixed<16,16> counterFPPhaseIncIncNeg = Q16_16(0.99f);

    // Fixed<16,16> counterModFP = Q16_16(0);
    // Fixed<16,16> targetModFP= Q16_16(1000);
    // Fixed<16,16> targetModIncFPOrg= Q16_16(1000);
    // Fixed<16,16> targetModIncFP = Q16_16(1000);
    // Fixed<16,16> targModIncMul = Q16_16(1.1f);
    bool b1=0;
    // bool b2=0;

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      // const size_t modwlen = (Q16_16(wlen) * Q16_16(7.5f)).to_int(); // Modulator is much slower, creates pulse width modulation
      // const size_t modwlenhalf =modwlen >> 1;
      const Q16_16 onefp = Q16_16(1);
      const Q16_16 zerofp = Q16_16(0);

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        size_t loopbits = 1;
        for (size_t bit = 0U; bit < 32U; bit++) {
          // uint32_t wrap = (phase < wlen);  // 1 if in range, 0 if needs wrap
          // phase *= wrap;  // Becomes 0 if phase >= wlen
          phase = phase >= wlen ? 0 : phase; // wrap around
          // modphase = modphase >= modwlen ? 0 : modphase; // wrap around

          [[unlikely]]
          if (phase == 0) {
            counterFP=zerofp;
            targetFP = targetIncFPOrg;
            targetIncFP = targetIncFPOrg;
            b1=0;          
          }

          // [[unlikely]]
          // if (modphase == 0) {
          //   b2=0;          
          //   counterModFP=zerofp;
          //   targetModFP = targetModIncFPOrg;
          //   targetModIncFP = targetModIncFPOrg;
          // }

          [[unlikely]]
          if (counterFP >= targetFP) {
            b1 = !b1;
            counterFP = zerofp;
            targetFP += targetIncFP;
            targetIncFP *= targIncMul;
            targetIncFP *= targIncMul;
          }

          // [[unlikely]]
          // if (counterModFP >= targetModFP) {
          //   b2 = !b2;
          //   if (b2) {
          //     counterFPPhaseInc = Q16_16(1.9); // When b2 is high, speed up the phase increment for a few cycles, creating a wider pulse
          //   } else {
          //     counterFPPhaseInc = Q16_16(0.1); // Normal speed
          //   }
          //   counterModFP = zerofp;
          //   targetModFP += targetModIncFP;
          //   targetModIncFP *= targModIncMul;
          // }


          counterFP += onefp;
          // counterModFP += onefp;

          // counterFPPhaseInc *= counterFPPhaseIncInc;
          // if (counterFPPhaseInc > Q16_16(1.1)) {
          //   // counterFPPhaseInc = Q16_16(1);
          //   counterFPPhaseIncInc = counterFPPhaseIncIncNeg;
          // }
          // if (counterFPPhaseInc < Q16_16(0.9)) {
          //   counterFPPhaseIncInc = counterFPPhaseIncIncPos;
          // }

          word <<= 1;
          word |= b1;  
          phase++;

        }
        *(bufferA ++) = word;

      }
    }


    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<16,16>;
      fptype vinv = fptype(1) - v;

      fptype targmax = fptype(1/16.f).mulWith(WvlenFPType(this->wavelen));
      targetIncFPOrg = ((vinv * targmax) + fptype(3));
      targIncMul = Q16_16(1) - (vinv * Q16_16(0.8f)) ;

      // counterFPPhaseIncIncNeg = Q16_16(1) - (v * Q16_16(0.05f));
      // counterFPPhaseIncIncPos = Q16_16(1) + (v * Q16_16(0.05f));

      // fptype targModmax = fptype(1/2.f).mulWith(WvlenFPType(this->wavelen));
      // targetModIncFPOrg = ((vinv * targModmax) + fptype(10));
      // targModIncMul = (vinv * Q16_16(0.1f)) + Q16_16(1);
      
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "exp1";
    }

  
  private:
    size_t phase=0;
    size_t modphase=0;
    // bool y=0;
    // int32_t err0=0;


    int pulselen=10000;

};


using oscModelPtr = std::shared_ptr<oscillatorModel>;


const size_t __not_in_flash("mydata") N_OSCILLATOR_MODELS = 11;

// Array of "factory" lambdas returning oscModelPtr

std::array<std::function<oscModelPtr()>, N_OSCILLATOR_MODELS> __not_in_flash("mydata") oscModelFactories = {
  
  
  

  []() { return std::make_shared<sawOscillatorModel>(); } 
  ,
  []() { return std::make_shared<expPulseSDOscillatorModel>(); } 
  ,
  []() { return std::make_shared<smoothThreshSDOscillatorModel>(); } //sharktooth 10
  ,
  []() { return std::make_shared<parasineSDOscillatorModel>(); } 
  // ,
  // []() { return std::make_shared<squareSineSDOscillatorModel>(); } 
  
  ,
  //squares
  []() { return std::make_shared<pulsePWOscillatorModel>(); }
  ,
  []() { return std::make_shared<expPulse2SDOscillatorModel>(); } 
  ,
  // []() { return std::make_shared<pulsePMSDOscillatorModel>(); } 
  // ,


  // //tris
  []() { return std::make_shared<triOscillatorModel>(); } 
  ,
  []() { return std::make_shared<triSDVar1OscillatorModel>(); } //tri with nice mod
  ,

  // //noise
  []() { return std::make_shared<noiseOscillatorModelSD>();} 
  ,
  []() { return std::make_shared<whiteNoiseOscillatorModel>(); }
  ,

  //silent
  []() { return std::make_shared<silentOscillatorModel>();}
};