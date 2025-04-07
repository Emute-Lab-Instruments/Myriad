#pragma once

#include "hardware/pio.h"
#include <memory>
#include <array>
#include <functional>
#include "oscVisData.hpp"
#include "oscDisplayModes.hpp"
#include "clockfreq.h"



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
  };
  
  pio_program prog;
  virtual void fillBuffer(uint32_t* bufferA, size_t wavelen)=0;
  size_t loopLength;
  virtual ~oscillatorModel() = default;  
  uint loadProg(PIO pioUnit) {
    return pio_add_program(pioUnit, &prog);
  }

  oscDisplayModes vis;

  virtual void ctrl(const float v) {
    //receive a control parameter
  }

  virtual pio_sm_config getBaseConfig(uint offset) {
    return pin_ctrl_program_get_default_config(offset);
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
          *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen * 7.19f);
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
        wmultmult = 0.3f + (v*0.5f);
      }
  
      pio_sm_config getBaseConfig(uint offset) {
        return expdec_program_get_default_config(offset);
      }
    
    private:
      float wmult=1;
      float wmultmult=1.f;
    
    };
  
class sawOscillatorModel : public virtual oscillatorModel {
  public:
    sawOscillatorModel() : oscillatorModel(){
      loopLength=8;
      prog=pin_ctrl_program;
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      for (size_t i = 0; i < loopLength; ++i) {
  
      }
    }
    void ctrl(const float v) override {
    }
  
  
  private:
    int phase=0;
    bool y=0;
    int err0;

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
          *(bufferA + i) = static_cast<uint32_t>(wavelen * 0.01, random(0,randMult) * wavelen * 0.01);
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
    float randMult=100;
    // const std::vector<float> oscTemplate {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
  
  
  };
  

using oscModelPtr = std::shared_ptr<oscillatorModel>;


const size_t __not_in_flash("mydata") N_OSCILLATOR_MODELS = 8;

// Array of "factory" lambdas returning oscModelPtr

std::array<std::function<oscModelPtr()>, N_OSCILLATOR_MODELS> __not_in_flash("mydata") oscModelFactories = {
  // []() { return std::make_shared<noiseOscillatorModel2>(); }
  []() { return std::make_shared<squareOscillatorModel>(); }
  
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