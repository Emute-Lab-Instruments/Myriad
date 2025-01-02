#pragma once

#include <stdlib.h>
#include <array>
#include "Arduino.h"

#include "src/memlp/MLP.h"
#include "drawing.h"


template <typename T>
class BoundedEncoderValue {
public:
  BoundedEncoderValue(T _min, T _max, T _scale) : maxVal(_max), minVal(_min), scaleVal(_scale), value(_min) {}
  BoundedEncoderValue() : maxVal(T(0.9)), minVal(T(0)), scaleVal(T(0.01)), value(T(0)) {}


  void update(const int change) {
    value += (change * scaleVal);
    value = std::max(minVal, value);
    value = std::min(maxVal, value);
  }

  inline T getValue() {
    return value;
  }

  void setScale(T newScale) {scaleVal = newScale;}
  void setMax(T newMax) {maxVal = newMax; value = std::min(maxVal, value);}
  void setMin(T newMin) {minVal = newMin; value = std::max(minVal, value);}
  
private:
  T value;
  T maxVal, minVal, scaleVal;
};


template<size_t N>
class metaOsc {
public:
    metaOsc() {};
    virtual ~metaOsc() = default;
    // float depth=0;
    // float speed=0;
    // BoundedEncoderValue<float> moddepth(0.f, 0.9f, 0.01f);
    BoundedEncoderValue<float> moddepth;
    BoundedEncoderValue<float> modspeed;

    virtual std::array<float, N> update(const float (&adcs)[4]) =0;    

    void setDepth(int delta) {
        Serial.println("base class set depth");
        moddepth.update(delta);
    }

    void setSpeed(int delta) {
        modspeed.update(delta);
    }

    // virtual void draw(TFT_eSPI &tft) =0;

protected:
  //add screen bounds
};

constexpr float TWOPI  = PI * 2;

template<size_t N>
class metaOscSines : public metaOsc<N> {
public:
    metaOscSines() {
        //init with equally spread phase
        const float phaseGap = TWOPI/N;
        for(size_t i=0; i < N; i++) {
            phasors[i] = i*phaseGap;
        }
    }

    std::array<float, N> update(const float (&adcs)[4]) override {
        for(size_t i=0; i < N; i++) {
            sines[i] = sinf(phasors[i]) * this->moddepth.getValue();
            phasors[i] += this->modspeed.getValue();
        }
        return sines;
    }

    

private:
    std::array<float, N> phasors;
    std::array<float, N> sines;
};


template<size_t N>
class metaOscMLP : public metaOsc<N> {
public:
    metaOscMLP() {
      net =  new MLP<float>({ 6, 16, 8, 8, N }, { ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::LINEAR, ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::SIGMOID });
      net->DrawWeights();
    }

    std::array<float, N> update(const float (&adcs)[4]) override {
      // Serial.printf("nn ph %f %d\n",phasor, clockCount);
      if (clockCount == 0) {
        std::vector<float> netInput {phasor,adcs[0] * adcMul,adcs[1] * adcMul,adcs[2] * adcMul,adcs[3] * adcMul,1.f};
        std::vector<float> output(N);
        net->GetOutput(netInput, &output);
        phasor += this->modspeed.getValue();
        if (phasor > 1.f) phasor -= 1.f;
        // Serial.println(netInput[0]);
        // Serial.println(output[1]);
        std::copy(output.begin(), output.begin() + output.size(), arrOutput.begin());
        for(size_t i=0; i < N; i++) {
          arrOutput[i] *= this->moddepth.getValue();
        }
      }
      clockCount++;
      if (clockCount == clockDiv) {
        clockCount = 0;
      }
      return arrOutput;
    }


private:
  MLP<float> *net; //({ 6, 16, 8, 8, N }, { ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::LINEAR, ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::SIGMOID });
  float phasor=0;
  std::array<float, N> arrOutput;
  size_t clockDiv = 2;
  size_t clockCount=0;
  const float adcMul = 1/4096.0;
};


