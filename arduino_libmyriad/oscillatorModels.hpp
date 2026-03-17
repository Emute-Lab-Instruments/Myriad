#pragma once

#include "hardware/pio.h"
#include <memory>
#include <array>
#include <functional>
#include "oscDisplayModes.hpp"
#include "clockfreq.h"
#include <limits>
#include "utils.hpp"
#include "oscmodels/oscillatorModel.hpp"

using WvlenFPType = Fixed<20,11>;




class silentOscillatorModel : public oscillatorModel {
public:
  silentOscillatorModel() : oscillatorModel() {
    loopLength=bufferSize;
    prog=bitbybit_program;

  }
  inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < bufferSize; ++i) {
        *(bufferA + i) = 0xAAAAAAAA;;
        updateFade();
    }
  }

  String getIdentifier() override {
    return "sil";
  }

private:
  const size_t bufferSize=16;
  // static constexpr uint32_t halfWaveLen = sampleClock / 100000;
};



  


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
          local_phase++;
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

        const int32_t local_fadeLevel = fadeLevel; // Load once


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

                amp = (amp * local_fadeLevel) >> fadeBitResolution;  

                int32_t y = amp >= local_err0 ? 1 : 0;

                local_err0 = (y ? peakAmp : 0) - amp + local_err0;

                word <<= 1;
                word |= y;

                local_phase++;
            }

            *(bufferA + i) = word;
        }

        updateFade();
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
      }
      // PERIODIC_RUN(
      //   Serial.printf("fadeShift: %d\n", fadeShift);
      // , 500);
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "tri";
    }

    void reset() override{
      phase=0;
      err0=0;
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
    // size_t bitshift = 0;
    // uint32_t bitmul = 1U; // multiplier for bit shift, used to scale the output


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
      static int32_t FADE_REF = 1 << 12;
      const bool fading = fadeDirection != 0;
      const int32_t volumePeak = fading ? static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      ) : 0;
      int32_t lErr = err0;

      for (size_t i = 0; i < loopLength; ++i) {
        uint32_t word = 0U;
        
        for(uint32_t bit = 0; bit < 32; ++bit) {
          if (counter == 0) {
            on = !on;  // Toggle state
            Q16_16 rnd = Q16_16::random_hw(Q16_16(0),randMult);   
            counter = 1 + (WvlenFPType(wavelen) * WvlenFPType(0.01f)).mulWith(rnd).to_int();                     
          }
          counter--;

          // word <<= 1;
          // word |= on;

          int32_t y;
          if (fading) [[unlikely]] {
            int32_t amp = on ? FADE_REF : 0;
            y = amp >= lErr ? 1 : 0;
            lErr = (y ? volumePeak : 0) - amp + lErr;
          } else {
            y = on;
          }

          word <<= 1;
          word |= y;
        }
        
        *(bufferA + i) = word;

      }
      err0 = lErr;
      updateFade();
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
    int32_t err0 = 0;    
    
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

      int32_t local_phase = phase;     // Load once
      int32_t local_thr = thr;         // Load once
      const bool fading = fadeDirection != 0;

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          local_phase = local_phase >= wlen ? 0 : local_phase; // wrap around

          size_t amp = local_phase << 1;
          amp =  amp > wlen ? local_phase : amp; 


          const bool y = amp >= local_thr ? 1 : 0;
          size_t err0 = (y ? wlen : 0) - amp + local_thr;

          local_thr = ((err0 * alpha) + (local_thr)) >> qfp;

          word <<= 1;
          word |= y;

          local_phase++;
        }

        if (fading) [[unlikely]] {
          if ((get_rand_32() & fadeMaxLevel) >= fadeLevel)
            word = 0xAAAAAAAA;
        }        
        *(bufferA + i) = word;

      }
      updateFade();
      phase = local_phase;   // Store once at end
      thr = local_thr;       // Store once at end
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
      const bool fading = fadeDirection != 0;

      for (size_t i = 0; i < loopLength; ++i) {
        if ((rand() % 200000) > alpha) {
          int inc = beta>>2;
          acc += (get_rand_32() & 1) ? inc : -inc;
        }
        if (rand() % 1000 > beta)
          acc ^= (rand() & 127U);
        
        // *(bufferA + i) = acc;
        if (fading) [[unlikely]] {
          // Probabilistically replace words with silence
          // Higher fadeLevel = fewer replacements
          if ((get_rand_32() & fadeMaxLevel) >= fadeLevel) {
            *(bufferA + i) = 0xAAAAAAAA;
          } else {
            *(bufferA + i) = acc;
          }
        } else {
          *(bufferA + i) = acc;
        }
      }
      updateFade();

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
    // int err0=0;

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




class expPulseSDOscillatorModel : public virtual oscillatorModel {
  private:
    // size_t loopLengthBits=0;
  public:


    expPulseSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
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
      static int32_t FADE_REF = 1 << 12;
      const bool fading = fadeDirection != 0;
      const int32_t volumePeak = fading ? static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      ) : 0;
      int32_t lErr = err0;      


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

          // word <<= 1;
          // word |= b1;  

          int32_t y;
          if (fading) [[unlikely]] {
            int32_t amp = b1 ? FADE_REF : 0;
            y = amp >= lErr ? 1 : 0;
            lErr = (y ? volumePeak : 0) - amp + lErr;
          } else {
            y = b1;
          }

          word <<= 1;
          word |= y;

          phase++;

          // loopbits <<= 1;
        }
        *(bufferA ++) = word;
      }
      err0 = lErr;
      updateFade();
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

      static int32_t FADE_REF = 1 << 12;

      const bool fading = fadeDirection != 0;
      const int32_t volumePeak = fading ? static_cast<int32_t>(
          (static_cast<int64_t>(FADE_REF) * fadeInvTable[fadeLevel]) >> 16
      ) : 0;   
      int32_t lErr = err0;   

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

          // word <<= 1;
          // word |= b1;  
          int32_t y;
          if (fading) [[unlikely]] {
            int32_t amp = b1 ? FADE_REF : 0;
            y = amp >= lErr ? 1 : 0;
            lErr = (y ? volumePeak : 0) - amp + lErr;
          } else {
            y = b1;
          }

          word <<= 1;
          word |= y;
          phase++;

        }
        *(bufferA ++) = word;

      }
      err0 = lErr;
      updateFade();
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
      return "slide";
    }

  
  private:
    size_t phase=0;
    size_t modphase=0;
    int32_t err0=0;


    int pulselen=10000;

};

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

    inline static bool tablesGenerated = false;

    private:
    uint32_t phase = 0;
    int32_t err = 0;
    int32_t morph = 0;
    inline static __not_in_flash("tables") uint16_t table_a[512];
    inline static __not_in_flash("tables") uint16_t table_b[512];
};


using oscModelPtr = oscillatorModel*;


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
  private:
    uint32_t phase = 0;
    int32_t err = 0;
    int32_t morph = 0;
    inline static __not_in_flash("tables") uint16_t table_a[512];
    inline static __not_in_flash("tables") uint16_t table_b[512];
    inline static bool tablesGenerated = false;
};

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
          size_t amp = toothPhase.to_int(); 

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


const size_t __not_in_flash("mydata") N_OSCILLATOR_MODELS = 13;

