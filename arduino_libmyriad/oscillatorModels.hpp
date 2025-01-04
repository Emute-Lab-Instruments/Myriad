#pragma once

#include "hardware/pio.h"
#include <memory>
#include <array>
#include <functional>
#include "oscVisData.hpp"
#include "oscDisplayModes.hpp"


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
};


class pulse10OscillatorModel : public virtual oscillatorModel {
public:
  pulse10OscillatorModel() : oscillatorModel(){
    loopLength=2;
    prog=pin_ctrl_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(64);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 20 == 0 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen);
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
    prog=pin_ctrl_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(64);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 20 <7 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen);
    }
  }
  const std::vector<float> oscTemplate {0.5,0.5};


};

class squareOscillatorModel2 : public oscillatorModel {
public:
  squareOscillatorModel2() : oscillatorModel() {
    loopLength=10;
    prog=pin_ctrl_program;
    vis.mode = oscDisplayModes::MODES::SPECTRAL;
    vis.data.resize(64);
    for(size_t i=0; i < vis.data.size(); i++) {
      vis.data[i] = i % 10 < 3 ? 1 : 0;
    }

  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen);
    }
  }
  const std::vector<float> oscTemplate {0.18181818181818, 0.018181818181818, 0.16363636363636, 0.036363636363636, 0.14545454545455, 0.054545454545455, 0.12727272727273, 0.072727272727273, 0.10909090909091, 0.090909090909091};
};


using oscModelPtr = std::shared_ptr<oscillatorModel>;


const size_t N_OSCILLATOR_MODELS = 3;

// Array of "factory" lambdas returning oscModelPtr

std::array<std::function<oscModelPtr()>, N_OSCILLATOR_MODELS> oscModelFactories = {
  []() { return std::make_shared<squareOscillatorModel>(); }
  ,
  []() { return std::make_shared<pulse10OscillatorModel>(); }
  ,
  []() { return std::make_shared<squareOscillatorModel2>();}
};