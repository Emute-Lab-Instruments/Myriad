#pragma once

#include <stdlib.h>
#include <array>
#include "Arduino.h"

#include "src/memlp/MLP.h"



constexpr float TWOPI  = PI * 2;

template<size_t N>
class metaOsc {
public:
    metaOsc() {};
    virtual ~metaOsc() = default;
    float depth=0;
    float speed=0;

    virtual std::array<float, N> update(const float (&adcs)[4]) =0;    

    virtual void setDepth(int newDepth) {
        Serial.println("base class set depth");
        depth=newDepth;
    }
    virtual void setSpeed(int newSpeed) {
        speed = newSpeed;
    }
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
    }

    std::array<float, N> update(const float (&adcs)[4]) override {
        for(size_t i=0; i < N; i++) {
            sines[i] = sinf(phasors[i]) * this->depth;
            phasors[i] += this->speed;
        }
        return sines;
    }

    void setDepth(int newDepth) override {
      Serial.println("sines set depth");
        this->depth= newDepth * 0.01;
    }

    void setSpeed(int newSpeed) override {
        this->speed = newSpeed * 0.005;
        
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
        phasor += this->speed;
        if (phasor > 1.f) phasor -= 1.f;
        // Serial.println(netInput[0]);
        // Serial.println(output[1]);
        std::copy(output.begin(), output.begin() + output.size(), arrOutput.begin());
        for(size_t i=0; i < N; i++) {
          arrOutput[i] *= this->depth;
        }
      }
      clockCount++;
      if (clockCount == clockDiv) {
        clockCount = 0;
      }
      return arrOutput;
    }

    void setDepth(int newDepth) override {
      this->depth= newDepth * 0.001;
      Serial.printf("mlp set depth %f\n", this->depth);
    }

    void setSpeed(int newSpeed) override {
        this->speed = newSpeed * 0.001;
        Serial.printf("Speed %f\n", this->speed);

        
    }



private:
  MLP<float> *net; //({ 6, 16, 8, 8, N }, { ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::LINEAR, ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::SIGMOID });
  float phasor=0;
  std::array<float, N> arrOutput;
  size_t clockDiv = 2;
  size_t clockCount=0;
  const float adcMul = 1/4096.0;
};


