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
      const int32_t wlen = this->wavelen;
      const int32_t pm_int = phaseMul >> 15;      // Integer part (2..11)
      const int32_t pm_frac = phaseMul & 0x7FFF;  // Fractional part
            
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for (uint32_t bit = 0; bit < 32; ++bit) {
          phase++;
          if (phase>=wlen) {
            phase = 0U;
          }
          // const size_t phasemask = -(phase < wlen);  // 0xFFFFFFFF if phase < wavelen, 0 otherwise
          // phase &= phasemask;

          int32_t amp = phase * pm_int + ((phase * pm_frac) >> 15);        

          // const int32_t mask = -(amp < wlen);  // 0xFFFFFFFF if amp < wavelen, 0 otherwise
          // amp &= mask;
          if (amp >= wlen) {
            amp = 0;
          }
          // uint32_t y = -(amp >= err0);
          // err0 = (y ? wlen : 0) - amp + err0;

          // word |= (y << bit);
          int32_t y = amp >= err0 ? 1 : 0;
          err0 = (y ? wlen : 0) - amp + err0;
          // word |= (y << bit);
          word |= y;
          word <<= 1;
        } 

        *(bufferA + i) = word;

      }

    }

    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<17,15>;
      fptype newPhaseMul = fptype(2) + (fptype(40).mulWith(v));
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
      setClockModShift(1);
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
    }

    inline void fillBuffer(uint32_t* bufferA) {      
      const size_t wlen = this->wavelen;

      const int32_t triPeakPoint = (wlen * phaseRisingInvMul) >> qfp;

      const uint32_t rising_int = phaseRisingMul >> qfp;     // 2..9
      const uint32_t rising_frac = phaseRisingMul & 0x3FFF;  // 14-bit fractional
      const uint32_t falling_int = phaseFallingMul >> qfp;
      const uint32_t falling_frac = phaseFallingMul & 0x3FFF;      

      for (size_t i = 0; i < loopLength; ++i) {
        uint32_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wlen ? 0 : phase; // wrap around
          int32_t amp=0;

          if (phase <= triPeakPoint) {
            // amp = ( phase * phaseRisingMul) >> qfp; 
            amp = phase * rising_int + (((uint32_t)phase * rising_frac) >> qfp);
          }else{
            const int32_t delta = phase - triPeakPoint;            
            const int32_t fallingPhase = delta * falling_int + 
                                      (((uint32_t)delta * falling_frac) >> qfp);
            // const int32_t fallingPhase = ((phase - triPeakPoint) * phaseFallingMul) >> qfp; 
            // fallingPhase = (fallingPhase * phaseFallingMul) >> qfp;
            amp = wavelen - fallingPhase; 
          }

          int32_t y = amp >= err0 ? 1 : 0;
          err0 = (y ? wlen : 0) - amp + err0;


          // word |= (y << bit);
          word |= y;
          word <<= 1;

          phase++;

        }
        *(bufferA + i) = word;
      }

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
          // word |= (on << bit);
          word |= on;
          word <<= 1;
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
      const size_t wlen = this->wavelen;
      const size_t midphase = wlen >> 1;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wlen ? 0 : phase; // wrap around

        
          size_t amp = phase < midphase ? 
                phase << 1 
                : 
                wlen - ((phase-midphase) << 1); //tri wave
          // size_t amp = phase; // saw
          // size_t amp = phase < midphase  ? 0 : wavelen; // pulse

          const bool y = amp >= err0 ? 1 : 0;
          err0 = (y ? wlen : 0) - amp + err0;
          // err0 = (err0) & mul;
          err0 = (err0 * mul) >> qfp;


          // word |= (y << bit);
          word |= y;
          word <<= 1;

          phase++;
        }
        *(bufferA + i) = word;

      }

    }


    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<18,14>;
      mul = (fptype(1) - (fptype(v) * fptype(0.9f))).raw();
      // mul = ((1.f - (v * 0.9f)) * qfpMul);
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "trv10";
    }

  
  private:
    size_t phase=0;
    bool y=0;
    int err0=0;

    size_t mul=1;

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

          word |= (y << bit);

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

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wlen ? 0 : phase; // wrap around

          // size_t amp = phase;
          size_t amp =  phase > pulselen ? 0 : wlen; 


          int32_t y = amp >= err0 ? 1 : 0;
          err0 = (y ? wlen : 0) - amp + err0;

          // word |= (y << bit);
          word |= y;
          word <<= 1;

          phase++;
        }
        *(bufferA + i) = word;

      }
    }


    void ctrl(const Q16_16 v) override {
      using fptype = Fixed<1,30>;
      fptype pw = (fptype(1) - fptype(v)) * fptype(0.5f);
      pw = pw < fptype(0.01f) ? fptype(0.01f) : pw;
      pulselen = WvlenFPType(this->wavelen).mulWith(pw).to_int();
      // pw= std::max((1.f-v) * 0.5f, 0.01f);
      // pulselen = this->wavelen * pw;
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
    int err0=0;
    int pulselen=10000;

};

class expPulseSDOscillatorModel : public virtual oscillatorModel {
  private:
    size_t loopLengthBits=0;
  public:


    expPulseSDOscillatorModel() : oscillatorModel(){
      loopLength=8;
      loopLengthBits = 1 << __builtin_clz(loopLength);
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
        size_t loopbits = 1;
        do {
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

          word |= loopbits & -(int32_t)(b1);  // Set bit if either is true

          phase++;

          loopbits <<= 1;
        } while (loopbits);
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
    int err0=0;

    int pulselen=10000;

};

class pulsePMSDOscillatorModel : public virtual oscillatorModel {
  private:

    WvlenFPType phase = WvlenFPType(0);

    WvlenFPType modphase = WvlenFPType(0);
    WvlenFPType phaseIncLow=WvlenFPType(1);
    WvlenFPType phaseIncHigh=WvlenFPType(1);


  public:


    pulsePMSDOscillatorModel() : oscillatorModel(){
      loopLength=8;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }


    inline void fillBuffer(uint32_t* bufferA) {
      const WvlenFPType wlen = WvlenFPType(this->wavelen);
      const WvlenFPType wlenHalf = WvlenFPType(this->wavelen>>1);
      const WvlenFPType modwlen = WvlenFPType(2.9f) * wlen;
      const WvlenFPType modwlenPW = modwlen.div_pow2(1);

      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        size_t loopbits = 1;
        do {
          //carrier
          phase = (phase < wlen) ? phase : WvlenFPType(0);  

          size_t on = phase > wlenHalf ?  0xFFFFFFFF : 0;
          word |= loopbits & on;  // Set bit if either is true

           
          //modulator
          modphase = (modphase < modwlen) ? modphase : WvlenFPType(0);  
          WvlenFPType phaseInc = modphase > modwlenPW ?  phaseIncLow : phaseIncHigh;


          //carrier phase 
          phase = phase + phaseInc;

          //modulation phase
          modphase += WvlenFPType(1);

          loopbits <<= 1;
        } while (loopbits);

        *(bufferA ++) = word;
      }
    }


    void ctrl(const Q16_16 v) override {
      WvlenFPType modIdx = WvlenFPType(0.6f).mulWith(v);
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

class parasineSDOscillatorModel : public virtual oscillatorModel {
  public:

    parasineSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
      setClockModShift(1);
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      const size_t half_wlen = wlen >> 1;
      
      // Precompute reciprocal: (4 / half_wlen) as fixed-point
      // This replaces TWO divisions per bit with ONE precomputed multiply
      const int32_t inv_half_wlen_x4 = (static_cast<int64_t>(4) << WvlenFPType::FRACTIONAL_BITS) / half_wlen;
      
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word = 0U;
        for(size_t bit = 0U; bit < 32U; bit++) {
          phase = phase >= wlen ? 0 : phase; // wrap around
          
          // Parabolic sine approximation with asymmetric shaping
          int32_t sine_val;
          if (phase < half_wlen) {
            // base = (4 * phase * (half_wlen - phase)) / half_wlen
            // becomes: base = (phase * (half_wlen - phase) * inv_half_wlen_x4) >> FRAC_BITS
            int32_t temp = phase * (half_wlen - phase);
            int32_t base = (temp * inv_half_wlen_x4) >> WvlenFPType::FRACTIONAL_BITS;
            sine_val = (shape_pos.raw() * base) >> WvlenFPType::FRACTIONAL_BITS;
          } else {
            size_t p = phase - half_wlen;
            int32_t temp = p * (half_wlen - p);
            int32_t base = (temp * inv_half_wlen_x4) >> WvlenFPType::FRACTIONAL_BITS;
            sine_val = -((shape_neg.raw() * base) >> WvlenFPType::FRACTIONAL_BITS);
          }
          
          // Convert from [-half_wlen, +half_wlen] to [0, wlen]
          auto amp = (half_wlen + sine_val);
          
          int32_t y = amp >= err0 ? 1 : 0;
          err0 = (y ? wlen : 0) - amp + err0;
          word |= y;
          word <<= 1;

          phase++;
        }
        *(bufferA + i) = word;
      }
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
    int phase = 0;
    bool y=0;
    int err0 = 0;

    WvlenFPType shape_pos = WvlenFPType::from_float_ct(1.0);  
    WvlenFPType shape_neg = WvlenFPType::from_float_ct(1.0);  


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
  ,
  //squares
  []() { return std::make_shared<pulseSDOscillatorModel>(); }
  ,
  []() { return std::make_shared<pulsePMSDOscillatorModel>(); } 
  ,


  // //tris
  []() { return std::make_shared<triOscillatorModel>(); } //var tri, sounds great
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