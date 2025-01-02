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

  float getNormalisedValue() {
    return (value - minVal) / (maxVal - minVal);
  }
  
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
    

    virtual String getName() {return "";}

    virtual void draw(TFT_eSPI &tft) {};


protected:
  //add screen bounds
};

constexpr float TWOPI  = PI * 2;

template<size_t N>
class metaOscNone : public metaOsc<N> {
public:
    metaOscNone() {
      vals.fill(0);
    }

    String getName() override {return "no modulation";}


    std::array<float, N> update(const float (&adcs)[4]) override {
        return vals;
    }

private:
  std::array<float, N> vals;
};


template<size_t N>
class metaOscSines : public metaOsc<N> {
public:
    metaOscSines() {
        //init with equally spread phase
        const float phaseGap = TWOPI/N;
        for(size_t i=0; i < N; i++) {
            phasors[i] = i*phaseGap;
        }
        this->modspeed.setMax(0.1);
        this->modspeed.setScale(0.001);
        this->moddepth.setMax(0.1);
        this->moddepth.setScale(0.0009);
    }

    String getName() override {return "sines";}


    std::array<float, N> update(const float (&adcs)[4]) override {
        for(size_t i=0; i < N; i++) {
            sines[i] = sinf(phasors[i]) * this->moddepth.getValue();
            phasors[i] += this->modspeed.getValue();
        }
        return sines;
    }

    void draw(TFT_eSPI &tft) override { 
      const int32_t barwidth=4;
      constexpr float step = sqwidth / N;
      constexpr float stepoffset = (step+barwidth) / 2.f;
      for(size_t i=0; i < N; i++) {
        const int h = sines[i] * sqhalfwidth * 10.f;
        const int left = sqbound + (step*i) + stepoffset;
        tft.fillRect(left,sqbound,barwidth, sqwidth, ELI_BLUE);
        if (h <0) {
          tft.fillRect(left,120+h,barwidth, -h, TFT_WHITE);
        }else{
          tft.fillRect(left,120,barwidth, std::max(h,1), TFT_WHITE);
        }
      }
    }

private:
    std::array<float, N> phasors;
    std::array<float, N> sines;

};


template<size_t N>
class metaOscMLP : public metaOsc<N> {
public:
    metaOscMLP() {
      net =  new MLP<float>({ 6, 16, 8, 5, N }, { ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::LINEAR, ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::SIGMOID });
      net->SetCachedLayerOutputs(true);
      net->DrawWeights();
      this->modspeed.setMax(0.1);
      this->modspeed.setScale(0.001);
    }

    String getName() override {return "neural net";}

    std::array<float, N> update(const float (&adcs)[4]) override {
      // Serial.printf("nn ph %f %d\n",phasor, clockCount);
      if (clockCount == 0) {
        std::vector<float> netInput {sin(phasor),adcs[0] * adcMul,adcs[1] * adcMul,adcs[2] * adcMul,adcs[3] * adcMul,1.f};
        for(size_t i=0; i < netInput.size(); i++) {
          netInput[i] *= this->moddepth.getValue();
        }
        std::vector<float> output(N);
        net->GetOutput(netInput, &output);
        phasor += this->modspeed.getValue();
        if (phasor > TWOPI) phasor -= TWOPI;
        // Serial.println(netInput[0]);
        // Serial.println(output[1]);
        std::copy(output.begin(), output.begin() + output.size(), arrOutput.begin());
        // for(size_t i=0; i < N; i++) {
        //   arrOutput[i] *= this->moddepth.getValue();
        // }
      }
      clockCount++;
      if (clockCount == clockDiv) {
        clockCount = 0;
      }
      return arrOutput;
    }

    
    //draw NN
    void draw(TFT_eSPI &tft) override { 
      
      float steph = sqwidth / net->m_layers.size();
      float barh = steph * 0.5;
      float barOffset = (steph-barh) * 0.5f;
      for(size_t i_layer=0; i_layer < net->m_layers.size(); i_layer++) {
        auto layerOutputs = net->m_layers[i_layer].cachedOutputs;
        const float step = sqwidth / layerOutputs.size();
        const float barwidth=step * 0.5f;
        const float stepoffset = (step-barwidth) * 0.5f;
        for(size_t i=0; i < layerOutputs.size(); i++) {
          const int32_t col = 32767 + (20000 * layerOutputs[i]);
          const float left = sqbound + (step*i) + stepoffset;
          tft.fillRect(left,sqbound + (steph * i_layer) + barOffset,barwidth, barh, col);
        }
      }
    }


private:
  MLP<float> *net; //({ 6, 16, 8, 8, N }, { ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::LINEAR, ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::SIGMOID });
  float phasor=0;
  std::array<float, N> arrOutput;
  size_t clockDiv = 2;
  size_t clockCount=0;
  const float adcMul = 1/4096.0;
};


template <size_t N>
using metaOscPtr = std::shared_ptr<metaOsc<N>>;

