#pragma once

#include "hardware/pio.h"
#include <memory>
#include <array>
#include "oscVisData.hpp"


//todo: try sliding window across template array
//todo: try feedback system to generate template values
//todo: perlin noise
//todo: time varying buffers

class oscillatorModel {
public:
  oscillatorModel() {
    visData.spec.resize(64);
    for(size_t i=0; i < visData.spec.size(); i++) {
      visData.spec[i] = i % 10 == 0 ? 1 : 0;
    }
  };
  pio_program prog;
  virtual void fillBuffer(uint32_t* bufferA, size_t wavelen)=0;
  size_t loopLength;
  virtual ~oscillatorModel() = default;  
  uint loadProg(PIO pioUnit) {
    return pio_add_program(pioUnit, &prog);
  }

  oscVisData visData;
  inline oscVisData& getVisData() {
    return visData;
  }
};


class squareOscillatorModel : public virtual oscillatorModel {
public:
  squareOscillatorModel() : oscillatorModel(){
    loopLength=2;
    prog=pin_ctrl_program;
    for(size_t i=0; i < visData.spec.size(); i++) {
      visData.spec[i] = i % 20 == 0 ? 1 : 0;
    }
  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen);
    }
  }
  const std::vector<float> oscTemplate {0.1,0.9};
};

class squareOscillatorModel2 : public oscillatorModel {
public:
  squareOscillatorModel2() : oscillatorModel() {
    loopLength=10;
    prog=pin_ctrl_program;
    for(size_t i=0; i < visData.spec.size(); i++) {
      visData.spec[i] = i % 5 == 0 ? 1 : 0;
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

oscModelPtr __not_in_flash("mydata") oscModel1 = std::make_shared<squareOscillatorModel>();
oscModelPtr __not_in_flash("mydata") oscModel2 = std::make_shared<squareOscillatorModel2>();

std::array<oscModelPtr,2> oscModels = {oscModel1, oscModel2};