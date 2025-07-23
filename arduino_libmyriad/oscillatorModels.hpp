#pragma once

#include "hardware/pio.h"
#include <memory>
#include <array>
#include <functional>
#include "oscVisData.hpp"
#include "oscDisplayModes.hpp"
#include "clockfreq.h"
#include <limits>



//todo: try sliding window across template array
//todo: try feedback system to generate template values
//todo: perlin noise
//todo: time varying buffers

class oscillatorModel {
public:
  oscillatorModel() {
    vis.mode = oscDisplayModes::MODES::SILENCE;
    vis.data.resize(1);
    vis.data[0] = 0;
    newFreq = false;
    updateBufferInSyncWithDMA = false;
  };
  
  pio_program prog;
  virtual void fillBuffer(uint32_t* bufferA, size_t wavelen)=0;
  size_t loopLength;
  virtual ~oscillatorModel() = default;  
  uint loadProg(PIO pioUnit) {
    return pio_add_program(pioUnit, &prog);
  }

  oscDisplayModes vis;

  volatile bool newFreq;
  bool updateBufferInSyncWithDMA; //if true, update buffer every time one is consumed by DMA
  size_t wavelen=100000;

  virtual void ctrl(const float v) {
    //receive a control parameter
  }

  virtual pio_sm_config getBaseConfig(uint offset) {
    return pin_ctrl_program_get_default_config(offset);
  }

  virtual void reset() {
  }
};


class pulse10OscillatorModel : public virtual oscillatorModel {
public:
  pulse10OscillatorModel() : oscillatorModel(){
    loopLength=2;
    prog=pin_ctrl_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(24);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 4 == 0 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen * 0.5);
    }
  }
  std::vector<float> oscTemplate {0.1,0.9};

  void ctrl(const float v) override {
    //receive a control parameter
    const float v1 = v * 0.98;
    const float v2 = 1.0 - v;
    oscTemplate [0] = v1;
    oscTemplate [1] = v2;
  }

};

class squareOscillatorModel : public virtual oscillatorModel {
public:
  squareOscillatorModel() : oscillatorModel(){
    loopLength=2;
    prog=pulse_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(24);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 7 ==0 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen);
    }
  }
 pio_sm_config getBaseConfig(uint offset) override {
    return pulse_program_get_default_config(offset);
  }

  const std::vector<float> oscTemplate {0.5,0.5}; 


};

class squareOscillatorModel2 : public oscillatorModel {
public:
  squareOscillatorModel2() : oscillatorModel() {
    loopLength=10;
    prog=pin_ctrl_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(24);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 5 < 2 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen * 2.5f);
    }
  }

  void ctrl(const float v) override {
    constexpr float bound = 0.001;
    constexpr float range = 0.2 - (bound * 2);
    float spread = (v * range) * 0.2;
    float accSpread=bound;

    oscTemplate [0] = accSpread;
    oscTemplate [1] = 0.2-accSpread;
    accSpread += spread;

    oscTemplate [2] = accSpread;
    oscTemplate [3] = 0.2 - accSpread;
    accSpread += spread;

    oscTemplate [4] =accSpread;
    oscTemplate [5] = 0.2 - accSpread;
    accSpread += spread;

    oscTemplate [6] = accSpread;
    oscTemplate [7] = 0.2 - accSpread;
    accSpread += spread;

    oscTemplate [8] = accSpread;
    oscTemplate [9] = 0.2 - accSpread;

  }

  //ratios 9:1,8:2,7:3,6:4,5:5
  std::vector<float> oscTemplate {0.18181818181818, 0.018181818181818, 0.16363636363636, 0.036363636363636, 0.14545454545455, 0.054545454545455, 0.12727272727273, 0.072727272727273, 0.10909090909091, 0.090909090909091};
};


class squareOscillatorModel14 : public oscillatorModel {
public:
  squareOscillatorModel14() : oscillatorModel() {
    loopLength=14;
    prog=pin_ctrl_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(24);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 5 < 2 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    wavelen = wavelen * 2;
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen * 1.75f);
    }
  }

  void ctrl(const float v) override {
    constexpr float bound = 0.001;
    constexpr float range = oneOver7 - (bound * 2);
    float spread = (v * range) * oneOver7;
    float accSpread=bound;

    oscTemplate [0] = accSpread;
    oscTemplate [1] = oneOver7-accSpread;
    accSpread += spread;

    oscTemplate [2] = accSpread;
    oscTemplate [3] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [4] =accSpread;
    oscTemplate [5] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [6] = accSpread;
    oscTemplate [7] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [8] = accSpread;
    oscTemplate [9] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [10] = accSpread;
    oscTemplate [11] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [12] = accSpread;
    oscTemplate [13] = oneOver7 - accSpread;

  }

  //ratios 9:1,8:2,7:3,6:4,5:5
  std::vector<float> oscTemplate {oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,
  oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14};
private:
  static constexpr float oneOver14 = 1.f/14.f;
  static constexpr float oneOver7 = 1.f/7.f;
};

class silentOscillatorModel : public oscillatorModel {
public:
  silentOscillatorModel() : oscillatorModel() {
    loopLength=bufferSize;
    prog=pin_ctrl_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(24);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i < 1 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < bufferSize; ++i) {
        *(bufferA + i) = silentOscillatorModel::halfWaveLen;
    }
  }
private:
  const size_t bufferSize=16;
  static constexpr uint32_t halfWaveLen = sampleClock / 100000;
};

class noiseOscillatorModel : public virtual oscillatorModel {
public:
  noiseOscillatorModel() : oscillatorModel(){
    loopLength=16;
    prog=pulse_program;
    // vis.mode = oscDisplayModes::MODES::SPECTRAL;
    // vis.data.resize(24);
    // for(size_t i=0; i < vis.data.size(); i++) {
    //   vis.data[i] = i % 12 ==0 ? 1 : 0;
    // }
    randBaseMin = randMin = sampleClock/20000;
    randBaseMax = randMax = sampleClock/20;
    randRange = randMax - randBaseMin;
    updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    // randMax = randBaseMax - wavelen;
    // randRange = randMax - randMin;
    for (size_t i = 0; i < loopLength; ++i) {
        *(bufferA + i) = static_cast<uint32_t>(random(randMin,randMax));
    }
  }
  pio_sm_config getBaseConfig(uint offset) {
    return pulse_program_get_default_config(offset);
  }
  void ctrl(const float v) override {
    randMax = randBaseMin + (v * randRange);
    Serial.println(randMax);
  }


private:
  long randMin, randMax, randBaseMin, randRange, randBaseMax;
  // const std::vector<float> oscTemplate {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};


};


//this is bonkers
class accOscillatorModel : public virtual oscillatorModel {
public:
  accOscillatorModel() : oscillatorModel(){
    loopLength=8;
    prog=pulse_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(24);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = (i % 6)  < 2 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < loopLength; ++i) {
      if (w <= 0) {
        acc += wavelen;
        w = acc >> 5;
      }
      int val;
      val = w;
      acc -= w;

      w=w - (w>>4) - 1;
      w=std::max(w,acc>>2);
      *(bufferA + i) = static_cast<uint32_t>(val);

    }
  }
  pio_sm_config getBaseConfig(uint offset) {
    return pulse_program_get_default_config(offset);
  }
  void ctrl(const float v) override {
  }


private:
  float phase=0;
  int acc=0;
  int w=1;
  int state=1;

};


class sinOscillatorModel8 : public oscillatorModel {
public:
  sinOscillatorModel8() : oscillatorModel() {
    loopLength=8;
    prog=pulse_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(24);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 5 < 2 ? 1 : 0;
    }
    tempOscTemplate.resize(loopLength,1.0/loopLength);
    oscTemplate.resize(loopLength,1.0/loopLength);
    ctrl(0.0);

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen * 2.55f);
    }
  }

  void ctrl(const float v) override {
    float vscale = ((v * 0.01) + 0.65) * 8;
    float total=0;
    for(float i=0; i < loopLength; i++) {
      const float w = std::max(0.0001f, fabs(sinf(i * vscale)));
      tempOscTemplate[i] = w;
      total += w;
    }

    //normalise
    const float lenscale = total/4.69f;
    const float scale=1.f/total;
    for(size_t i=0; i < loopLength; i++) {
      oscTemplate[i] = tempOscTemplate[i] * lenscale;
      // Serial.printf("%f\t",oscTemplate[i]);
    }
    Serial.println(lenscale);
  }

  std::vector<float> tempOscTemplate;
  std::vector<float> oscTemplate;

  pio_sm_config getBaseConfig(uint offset) {
    return pulse_program_get_default_config(offset);
  }

};

class expdecOscillatorModel1 : public virtual oscillatorModel {
  public:
  expdecOscillatorModel1() : oscillatorModel(){
      loopLength=8;
      prog=expdec_program;
    }
    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      for (size_t i = 0; i < oscTemplate.size(); ++i) {
          *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen * (7.19f * 1.f));
      }
      // wmult = wmult * wmultmult;
      // if (wmult < 0.125f) {
      //   wmult=1.f;
      // }
    }
    std::vector<float> oscTemplate {0.05,0.025,0.01,0.001,0.001,0.01,0.025,0.05};
    // std::vector<float> oscTemplate {0.1,0.5};
  
    void ctrl(const float v) override {
      //receive a control parameter
      // const float v1 = v * 0.98;
      // const float v2 = 1.0 - v;
      // oscTemplate [0] = v1;
      // oscTemplate [1] = v2;
      oscTemplate[0] = 0.05 + (v * 0.02);
      oscTemplate[1] = 0.025 - (v * 0.02);

      oscTemplate[4] = 0.001 - (v * 0.0005);
      oscTemplate[5] = 0.001 + (v * 0.0005);

      oscTemplate[5] = 0.01 - (v * 0.05);
      oscTemplate[6] = 0.025 + (v * 0.05);
    }

    pio_sm_config getBaseConfig(uint offset) {
      return expdec_program_get_default_config(offset);
    }
  
  private:
  
  };

class expdecOscillatorBytebeatModel : public virtual oscillatorModel {
    public:
    expdecOscillatorBytebeatModel() : oscillatorModel(){
        loopLength=8;
        prog=expdec_program;
        updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
        
    }
    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      for (size_t i = 0; i < oscTemplate.size(); ++i) {
          *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen * 7.19f * wmult);
      }
      wmult = wmult * wmultmult;
      if (wmult < 0.125f) {
        wmult=1.f;
      }
    }
    std::vector<float> oscTemplate {0.05,0.025,0.01,0.001,0.001,0.01,0.025,0.05};
    // std::vector<float> oscTemplate {0.1,0.5};
  
    void ctrl(const float v) override {
      wmultmult = 0.01f + (v*0.5f);
    }

    pio_sm_config getBaseConfig(uint offset) {
      return expdec_program_get_default_config(offset);
    }
  
  private:
    float wmult=1;
    float wmultmult=1.f;
  
};
  
#define WAVETABLE_SIZE 1024

class DeltaSigmaGenerator {
private:
    float accumulator;
    float error;
    
public:
    DeltaSigmaGenerator() : accumulator(0.0f), error(0.0f) {}
    
    // Reset the modulator state (call when starting new waveform)
    void reset() {
        accumulator = 0.0f;
        error = 0.0f;
    }
    
    // Generate a single bit output for given input amplitude (0.0 to 1.0)
    bool generateBit(float input) {
        // Add input to accumulator
        accumulator += input;
        
        bool output;
        if (accumulator >= 0.5f) {
            output = true;
            accumulator -= 1.0f;  // Subtract quantization level
        } else {
            output = false;
        }
        
        return output;
    }
    

  };

size_t debugcount=0;

class sawOscillatorModel : public virtual oscillatorModel {
  public:

    constexpr uint32_t generate_multiplier(int index) {
        // Convert (1.0 + index * 9.0/99.0) to 16.16 fixed point
        return (uint32_t)((2.0 + index * 9.0/990.0) * 32768.0 + 0.5);
    }  

    sawOscillatorModel() : oscillatorModel(){
      loopLength=128;
      prog=bitbybit_program;
      // generateSawtoothTable();
      for(size_t i = 0; i < 1000; ++i) {
        MULTIPLIER_TABLE[i] = generate_multiplier(i);
      }
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      // float waveleninv = 1.f/wavelen;
      // // Serial.println(amp);
      // // Serial.println(index);
      // for (size_t i = 0; i < loopLength; ++i) {
      //     float amp = (float) phase * waveleninv;
      //     // if (amp > 1.0f) {
      //     //   amp = 1.0f; // clamp to 1.0
      //     // } 
      //     size_t index = static_cast<size_t>(amp * (WAVETABLE_SIZE-1));
      //     if (index >= WAVETABLE_SIZE) {
      //       index = 0; // wrap around
      //     }
      //     // *(bufferA + i) = phase < wavelen/2 ?  4294967295U: 0; //sawtooth_table[index];
      //     *(bufferA + i) = sawtooth_table[index];

      //     // if (debugcount++ < 10000) {
      //     //   Serial.print(debugcount);
      //     //   Serial.print("\t");
      //     //   Serial.print(i);
      //     //   Serial.print("\t");
      //     //   Serial.print(phase);
      //     //   Serial.print("\t");
      //     //   Serial.print(index);
      //     //   Serial.print("\t");
      //     //   Serial.print(*(bufferA + i));
      //     //   Serial.print("\t");
      //     //   Serial.print("\n");
      //     // }

      //     phase += 32;
      //     if (phase >= wavelen) {
      //       phase -= wavelen;
      //     }        
      // }
      // Serial.println(amp);
      // Serial.println(index);

      
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          // if (phase>=wavelen) {
          //   phase = 0U;
          // }
          const size_t phasemask = -(phase < wavelen);  // 0xFFFFFFFF if phase < wavelen, 0 otherwise
          phase &= phasemask;
          phase++;

        
          size_t amp = (phase * phaseMul) >> 15U;
          // if (amp >= wavelen) {
          //   amp = 0U; // wrap around
          // }
          const size_t mask = -(amp < wavelen);  // 0xFFFFFFFF if amp < wavelen, 0 otherwise
          amp &= mask;

          const bool y = amp >= err0 ? 1 : 0;
          err0 = (y ? wavelen : 0) - amp + err0;

          word |= (y << bit);

        }
        // word = word << 1U;
        *(bufferA + i) = word;

      }

    }

    void ctrl(const float v) override {
      const size_t index = v < 0.05 ? 1 : static_cast<size_t>((0.2f +(v * 0.8f)) * 950.f);
      phaseMul = MULTIPLIER_TABLE[index];
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

  
  private:
    size_t phase=0;
    size_t phaseMul = 0;
    bool y=0;
    int err0=0;
    size_t val=0;
    // size_t sawtooth_table[WAVETABLE_SIZE];
    uint32_t MULTIPLIER_TABLE[1000];

    // DeltaSigmaGenerator deltaSigma;

    // void generateSawtoothTable() {
    //   double increment = 1.0 / (WAVETABLE_SIZE * 32.0);
    //   double phase=0;
    //     for (size_t i = 0; i < WAVETABLE_SIZE; ++i) {
    //         size_t word = 0U;
            
    //         // Generate each bit with slightly different amplitude
    //         for (int bit = 0; bit < 32; bit++) {
                
    //             // if (phase < 0.5) {
    //             //     word |= (1U << bit);
    //             // }
    //             if (deltaSigma.generateBit((float)phase)) {
    //                 word |= (1U << bit);
    //             }
                
    //             phase += increment;
    //         }
    //         sawtooth_table[i] = word;
    //         // sawtooth_table[i] = phase < 0.5 ?  4294967295U : 0U;
    //         // Serial.println(phase);
    //         // Serial.println(sawtooth_table[i]);
    //     }
    // }

  };

class triOscillatorModel : public virtual oscillatorModel {
  public:

    triOscillatorModel() : oscillatorModel(){
      loopLength=64;
      prog=bitbybit_program;
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
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {      

      if (ctrlChange) {
        ctrlChange = false;
      }

      const int32_t triPeakPoint = (wavelen * phaseRisingInvMul) >> qfp;

      for (size_t i = 0; i < loopLength; ++i) {
        uint32_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          // if (phase>=wavelen) {
          //   phase = 0U;
          // }
          phase = phase >= wavelen ? 0 : phase; // wrap around

          
          // int32_t isRising = phase <= triPeakPoint;
          // amp_fp += isRising ? risingInc : fallingInc;
          
          // // Handle wraparound
          // if (phase == 0) amp_fp = 0;
          // if (phase == triPeakPoint) amp_fp = wavelen << qfp;
          
          // size_t amp = amp_fp >> qfp;

          int32_t amp=0;
          // if (phase <= triPeakPoint) {
          //   amp = ( phase * phaseRisingMul) >> qfp; 
          // }else{
          //   const int32_t fallingPhase = ((phase - triPeakPoint) * phaseFallingMul) >> qfp; 
          //   // fallingPhase = (fallingPhase * phaseFallingMul) >> qfp;
          //   amp = wavelen - fallingPhase; 
          // }

          int32_t temp;  // declare this before the asm block
          if (phase <= triPeakPoint) {
              // Rising phase - use inline assembly for the multiply and shift
              asm volatile (
                  "mul    %[result], %[multiplicand]  \n\t"
                  "asr    %[result], %[shift_amount]  \n\t"
                  : [result] "=&l" (amp)
                  : [multiplicand] "l" (phaseRisingMul), 
                    [shift_amount] "l" (qfp),
                    "0" (phase)  // input tied to output register
                  : "cc"
              );
          } else {
              // Falling phase
              int32_t fallingPhase;
              asm volatile (
                  "sub    %[temp], %[ph], %[peak]     \n\t"
                  "mul    %[temp], %[fallMul]         \n\t"
                  "asr    %[temp], %[shift]           \n\t"
                  "sub    %[result], %[wlen], %[temp] \n\t"
                  : [result] "=&l" (amp), [temp] "=&l" (fallingPhase)
                  : [ph] "l" (phase), [peak] "l" (triPeakPoint),
                    [fallMul] "l" (phaseFallingMul), [shift] "l" (qfp),
                    [wlen] "l" (wavelen)
                  : "cc"
              );
          }
          // size_t amp = (phase <= triPeakPoint) 
          //     ? (phase * phaseRisingMul) >> qfp
          //     : wavelen - (((phase - triPeakPoint) * phaseFallingMul) >> qfp);
        
          // size_t amp = (phase * phaseMul) >> 15U;
          // if (amp >= wavelen) {
          //   amp = 0U; // wrap around
          // }
          // const size_t mask = -(amp < wavelen);  // 0xFFFFFFFF if amp < wavelen, 0 otherwise
          // amp &= mask;


          //delta-sigma modulation

          int32_t y = amp >= err0 ? 1 : 0;
          err0 = (y ? wavelen : 0) - amp + err0;

          // bit operations TODO: - get some aliasing here, but maybe works on faster saw osc.
          if (phase <= triPeakPoint) {
            y = !y;
          }

          word |= (y << bit);

          phase++;

        }
        // word = (word * bitmul) >> 15U;;
        // word = ~word;
        // uint64_t word64 = word << bitshift;
        // word = word << 20U; 
        // *(bufferA + i) = static_cast<uint32_t>(word64);  
        // *(bufferA + i) = word & lastWord & lastWord2 & lastWord3; 
        // lastWord2 = lastWord;
        // lastWord3 = lastWord2;                                                              
        // lastWord = word;
        *(bufferA + i) = word;

      }

    }

    void ctrl(const float v) override {
      // float gradRising = 2.f + (v * v * 100.f);
      // float gradFalling = 1.f/(1.f - (1.f / gradRising));
      // phaseRisingMul = static_cast<size_t>((gradRising * 32768.f) + 0.5f);
      // phaseFallingMul = static_cast<size_t>((gradFalling * 32768.f) + 0.5f);


      const size_t index = static_cast<size_t>(v * 999.f + 0.5f);
      if (index != lastPW || wavelen != lastWavelen) {
        lastPW = index;
        lastWavelen = wavelen;
        // Serial.printf("index: %zu, wavelen: %zu\n", index, wavelen);
        phaseRisingMul = MULTIPLIER_TABLE_RISING[index];
        phaseRisingInvMul = MULTIPLIER_TABLE_RISING_INV[index];
        phaseFallingMul = MULTIPLIER_TABLE_FALLING[index];
        risingInc = phaseRisingMul;
        fallingInc = -phaseFallingMul;  
        ctrlChange=true;    
        // amp_fp=0;
      }
        // bitmul = static_cast<uint32_t>(1U << 15) * v * 3.9f;
      // Serial.printf("%zu, phaseRisingMul: %zu, phaseRisingInvMul: %zu, phaseFallingMul: %zu\n", index, phaseRisingMul,phaseRisingInvMul, phaseFallingMul);
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

  
  private:
    int32_t phase=0;
    size_t phaseMul = 0;
    bool y=0;
    int32_t err0=0;
    size_t val=0;
    int32_t MULTIPLIER_TABLE_RISING[1000];
    int32_t MULTIPLIER_TABLE_RISING_INV[1000];
    int32_t MULTIPLIER_TABLE_FALLING[1000];
    
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


//bit by bit square wave oscillator
//better latency at lower frequencies than the timing buffer model
class squareBBBOscillatorModel : public virtual oscillatorModel {
  public:
    squareBBBOscillatorModel() : oscillatorModel(){
      loopLength=512;
      prog=bitbybit_program;
      // generate_sawtooth_wavetable();   
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      for (size_t i = 0; i < loopLength; ++i) {
        if (phase >= wavelen/2) {
            val = 4294967295U;
        }else{
          val = 0;
        }
        *(bufferA + i) = val;
        phase+=32;
        if (phase >= wavelen) {
          phase  -=wavelen; // wrap around
        }
      }
    }
    void ctrl(const float v) override {
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

  
  private:
    size_t phase=0;
    bool y=0;
    int err0;
    size_t val=0;

  };

class slideOscillatorModel : public virtual oscillatorModel {
  public:
  slideOscillatorModel() : oscillatorModel(){
      loopLength=6;
      prog=pulse_program;
      vis.mode = oscDisplayModes::MODES::SPECTRAL;
      vis.data.resize(24);
      for(size_t i=0; i < vis.data.size(); i++) {
        vis.data[i] = i % 7 ==0 ? 1 : 0;
      }
      // for(size_t i=0; i < oscTemplate.size()-loopLength; i++) {
      //   float total=0;
      //   for(size_t j=0; j < loopLength; j++) {
      //     total += oscTemplate[i+j];
      //   }
      //   scales.push_back(1.f/total);
      // }
      oscInterpTemplate.resize(loopLength);
      ctrl(0);
  
    }
    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      for (size_t i = 0; i < oscTemplate.size(); ++i) {
        // *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i+offset] * wavelen * scales[offset]);
        *(bufferA + i) = static_cast<uint32_t>(oscInterpTemplate[i] * wavelen);
      }
    }

    void ctrl(const float v) override {
      float scaledV = v*9.99;
      offset = static_cast<size_t>(scaledV);
      size_t offsetNext = offset+1;
      float remainder = scaledV - offset; 
      float remainderInv = 1.f - remainder;
      float total=0.f;
      for(size_t i=0; i < loopLength; i++) {
        float val = (remainderInv * oscTemplate[i+offset]) + (remainder * oscTemplate[i+offsetNext]);
        oscInterpTemplate[i] = val;
        total += val;
      }
      float scale = 1.f/total;
      for(size_t i=0; i < loopLength; i++) {
        oscInterpTemplate[i] *= scale;
        // Serial.printf("%f ", oscInterpTemplate[i]);
      }
      // Serial.println();
      // Serial.printf("%f %f %f\n", oscInterpTemplate[0], oscTemplate[offset], oscTemplate[offsetNext]);

    }

    pio_sm_config getBaseConfig(uint offset) override {
      return pulse_program_get_default_config(offset);
    }
  
    const std::vector<float> oscTemplate {0.1, 0.3, 0.001, 0.2, 0.003,
      0.0023, 0.00001, 0.04, 0.057, 0.034,
     0.002, 0.003, 0.001, 0.2, 0.023,
    0.4}; 
  
  private:
  size_t offset=0;
  std::vector<float> scales;
  std::vector<float> oscInterpTemplate;

  };
    

class noiseOscillatorModel2 : public virtual oscillatorModel {
  public:
    noiseOscillatorModel2() : oscillatorModel(){
      loopLength=16;
      prog=pulse_program;
      // vis.mode = oscDisplayModes::MODES::SPECTRAL;
      // vis.data.resize(24);
      // for(size_t i=0; i < vis.data.size(); i++) {
      //   vis.data[i] = i % 12 ==0 ? 1 : 0;
      // }
      randBaseMin = randMin = sampleClock/10000;
      randBaseMax = randMax = sampleClock/20;
      randRange = randMax - randBaseMin;
  
    }
    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      for (size_t i = 0; i < loopLength; ++i) {
          // bool on = random(0,1000) > 500;
          *(bufferA + i) = static_cast<uint32_t>(wavelen * 0.01f, random(0,randMult) * wavelen * 0.01f);
      }
    }
    pio_sm_config getBaseConfig(uint offset) {
      return pulse_program_get_default_config(offset);
    }
    void ctrl(const float v) override {
      randMult = 5 + (v * 500);
    }
  
  
  private:
    long randMin, randMax, randBaseMin, randRange, randBaseMax;
    float randMult=100.f;
    // const std::vector<float> oscTemplate {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
  
  
  };
  

//this sounds interesting in tonal variation of the mask but also clicks at regular points
class triErrMaskOscillatorModel : public virtual oscillatorModel {
  public:

    triErrMaskOscillatorModel() : oscillatorModel(){
      loopLength=64;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      const size_t midphase = wavelen >> 1;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wavelen ? 0 : phase; // wrap around

        
          size_t amp = phase > midphase ? phase >> 1 : phase << 1; //tri wave

          const bool y = amp >= err0 ? 1 : 0;
          err0 = (y ? wavelen : 0) - amp + err0;
          err0 = (err0) & mul;


          word |= (y << bit);

          phase++;
        }
        *(bufferA + i) = word;

      }

    }

    inline int find_msb(uint32_t x) {
      if (x == 0) return -1;
      int msb = 0;
      if (x >= 1 << 16) { msb += 16; x >>= 16; }
      if (x >= 1 << 8)  { msb += 8;  x >>= 8;  }
      if (x >= 1 << 4)  { msb += 4;  x >>= 4;  }
      if (x >= 1 << 2)  { msb += 2;  x >>= 2;  }
      if (x >= 1 << 1)  { msb += 1; }
      return msb;
    }

    void ctrl(const float v) override {
      constexpr size_t mask = std::numeric_limits<size_t>::max();
      // constexpr size_t mask2 = 0b10010010010010010010010010010010; //std::numeric_limits<size_t>::max();
      int msb = find_msb(wavelen);
      size_t shift = (32-msb) + static_cast<size_t>((msb-8) * v);
      if (shift < 14) {
        shift = 14;
      }
      mul = (mask >> shift) ;
      // mul = mul | wavelen;
      // aug = static_cast<size_t>(wavelen * v * 0.2f);
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    void reset() override {
      oscillatorModel::reset();
      phase = 0;
      y = 0;
      err0 = 0;
      mul = 1;
      aug = 1;
    }

  
  private:
    size_t phase=0;
    bool y=0;
    int err0=0;

    size_t mul=1;
    size_t aug=1;


  };



class triSDVar1OscillatorModel : public virtual oscillatorModel {
  public:

    triSDVar1OscillatorModel() : oscillatorModel(){
      loopLength=64;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      const size_t midphase = wavelen >> 1;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wavelen ? 0 : phase; // wrap around

        
          size_t amp = phase < midphase ? 
                phase << 1 
                : 
                wavelen - ((phase-midphase) << 1); //tri wave
          // size_t amp = phase; // saw
          // size_t amp = phase < midphase  ? 0 : wavelen; // pulse

          const bool y = amp >= err0 ? 1 : 0;
          err0 = (y ? wavelen : 0) - amp + err0;
          // err0 = (err0) & mul;
          err0 = (err0 * mul) >> qfp;


          word |= (y << bit);

          phase++;
        }
        *(bufferA + i) = word;

      }

    }


    void ctrl(const float v) override {
      mul = ((1.f - (v * 0.9)) * qfpMul);
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    void reset() override {
      oscillatorModel::reset();
      phase = 0;
      y = 0;
      err0 = 0;
      mul = 1;
    }

  
  private:
    size_t phase=0;
    bool y=0;
    int err0=0;

    size_t mul=1;

    static constexpr size_t qfp = 14U;
    static constexpr float qfpMul = 1U << qfp;

};

class triSDFBVar1OscillatorModel : public virtual oscillatorModel {
  public:

    triSDFBVar1OscillatorModel() : oscillatorModel(){
      loopLength=64;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      const int midphase = wavelen >> 1;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wavelen ? 0 : phase; // wrap around

        
          int amp = phase < midphase ? 
                phase << 1 
                : 
                wavelen - ((phase-midphase) << 1); //tri wave
          // size_t amp = phase; // saw
          // int amp = phase < midphase  ? 0 : wavelen; // pulse

          const bool y = amp >= err0 ? 1 : 0;
          err0 = (y ? wavelen : 0) - amp + err0 -  err1 ;
          // err0 = (err0) & mul;
          // err0 = (err0 * mul) >> qfp;
          err1 = err1 + ((err0 * mul) >> qfp);

          word |= (y << bit);

          phase++;
        }
        *(bufferA + i) = word;

      }

    }


    void ctrl(const float v) override {
      mul = ((v * 0.9)) * qfpMul;
      Serial.println(mul);
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    void reset() override {
      oscillatorModel::reset();
      phase = 0;
      y = 0;
      err0 = 0;
      mul = 1;
      err1=0;
    }

  
  private:
    int phase=0;
    bool y=0;
    int err0=0;
    int err1=0;
    int mul=1;

    static constexpr int qfp = 15U;
    static constexpr float qfpMul = 1U << qfp;

};

using oscModelPtr = std::shared_ptr<oscillatorModel>;


const size_t __not_in_flash("mydata") N_OSCILLATOR_MODELS = 8;

// Array of "factory" lambdas returning oscModelPtr

std::array<std::function<oscModelPtr()>, N_OSCILLATOR_MODELS> __not_in_flash("mydata") oscModelFactories = {
  
  
  
  // []() { return std::make_shared<triSDFBVar1OscillatorModel>(); }
  // []() { return std::make_shared<triSDVar1OscillatorModel>(); }
  // []() { return std::make_shared<triOscillatorModel>(); }
  []() { return std::make_shared<sawOscillatorModel>(); }
  // []() { return std::make_shared<squareBBBOscillatorModel>(); }
  // []() { return std::make_shared<squareOscillatorModel>(); }
  
  ,
  []() { return std::make_shared<expdecOscillatorModel1>(); }
  ,
  []() { return std::make_shared<pulse10OscillatorModel>(); }
  ,
  []() { return std::make_shared<squareOscillatorModel2>();}
  ,
  []() { return std::make_shared<squareOscillatorModel14>();}
  ,
  []() { return std::make_shared<expdecOscillatorBytebeatModel>(); }
  ,
  []() { return std::make_shared<noiseOscillatorModel2>();}
  ,
  []() { return std::make_shared<silentOscillatorModel>();}
};