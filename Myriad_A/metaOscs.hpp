#pragma once

#include <stdlib.h>
#include <array>
#include "Arduino.h"

#include "src/memlp/MLP.h"
#include "drawing.h"
#include <deque>

enum MODTARGETS {PITCH, EPSILON, PITCH_AND_EPSILON} modTarget = MODTARGETS::PITCH;


template <typename T>
class BoundedEncoderValue {
public:
  BoundedEncoderValue(T _min, T _max, T _scale) : maxVal(_max), minVal(_min), scaleVal(_scale), value(_min) {setMax(_max); setMin(_min);}
  BoundedEncoderValue() : scaleVal(T(0.01)), value(T(0)) {setMax(0.9); setMin(0.0);}


  void update(const int change) {
    value += (change * scaleVal);
    value = std::max(minVal, value);
    value = std::min(maxVal, value);
  }

  inline T getValue() {
    return value;
  }

  void setScale(T newScale) {scaleVal = newScale;}
  void setMax(T newMax) {
    maxVal = newMax; value = std::min(maxVal, value);
    if (maxVal != 0) {
      invMax = 1.0/maxVal;
    }
  }
  void setMin(T newMin) {minVal = newMin; value = std::max(minVal, value);
  }

  T getNormalisedValue() {
    return (value - minVal) / (maxVal - minVal);
  }

  T getInvMax() {
    return invMax;
  }
  
private:
  T value;
  T maxVal, minVal, scaleVal, invMax;
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

    virtual std::array<float, N>& getValues() =0;

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

    std::array<float, N>& getValues() override {
      return vals;
    };


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

    std::array<float, N>& getValues() override {
      return sines;
    };

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
class metaOscSinesFMultiple : public metaOsc<N> {
public:
    metaOscSinesFMultiple() {
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

    String getName() override {return "speedy sines";}


    std::array<float, N> update(const float (&adcs)[4]) override {
        for(size_t i=0; i < N; i++) {
            sines[i] = sinf(phasors[i] * (i+1)) * this->moddepth.getValue();
            phasors[i] += this->modspeed.getValue();
        }
        return sines;
    }

    std::array<float, N>& getValues() override {
      return sines;
    };

    void draw(TFT_eSPI &tft) override { 
      const int32_t barwidth=4;
      constexpr float step = sqwidth / N;
      constexpr float stepoffset = (step+barwidth) / 2.f;
      for(size_t i=0; i < N; i++) {
        const int h = sines[i] * sqhalfwidth * 10.f;
        const int left = sqbound + (step*i) + stepoffset;
        tft.fillRect(left,sqbound,barwidth, sqwidth, ELI_BLUE);
        if (h <0) {
          tft.fillRect(left,120+h,barwidth, -h, TFT_RED);
        }else{
          tft.fillRect(left,120,barwidth, std::max(h,1), TFT_RED);
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

    std::array<float, N>& getValues() override {
      return arrOutput;
    };


private:
  MLP<float> *net; //({ 6, 16, 8, 8, N }, { ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::LINEAR, ACTIVATION_FUNCTIONS::RELU, ACTIVATION_FUNCTIONS::SIGMOID });
  float phasor=0;
  std::array<float, N> arrOutput;
  size_t clockDiv = 2;
  size_t clockCount=0;
  const float adcMul = 1/4096.0;
};

template<size_t N>
class metaDrunkenWalkers : public metaOsc<N> {
public:

    struct point {float x; float y;};
    std::array<point, N> walkers;
    std::array<point, N> walkersBefore;

    metaDrunkenWalkers() {
        //init with equally spread phase
        for(size_t i=0; i < N; i++) {
            walkersBefore[i].x=120;
            walkersBefore[i].y=120;
            walkers[i].x=120;
            walkers[i].y=120;
        }
        this->modspeed.setMax(0.1);
        this->modspeed.setScale(0.001);
        this->moddepth.setMax(0.1);
        this->moddepth.setScale(0.0009);
    }

    String getName() override {return "drunken walkers";}


    std::array<float, N> update(const float (&adcs)[4]) override {
        for(size_t i=0; i < N; i++) {
          walkersBefore[i].x = walkers[i].x;
          walkersBefore[i].y = walkers[i].y;
          
          //random walk
          walkers[i].x += (random(-2000,2000)/100.0) * this->modspeed.getValue();
          walkers[i].y += (random(-2000,2000)/100.0) * this->modspeed.getValue();

          //wrap
          if (walkers[i].x < sqbound) {
            walkers[i].x += sqwidth;
          }else if (walkers[i].x > sqboundBR) {
            walkers[i].x -= sqwidth;
          }
          if (walkers[i].y < sqbound) {
            walkers[i].y += sqwidth;
          }else if (walkers[i].y > sqboundBR) {
            walkers[i].y -= sqwidth;
          }

          //distance from the centre
          const float dx = 120 - walkers[i].x;
          const float dy = 120 - walkers[i].y;
          mods[i] = std::sqrt((dx * dx) + (dy * dy)) * this->moddepth.getValue() * 0.01; 
        }
        return mods;
    }

    void draw(TFT_eSPI &tft) override { 

      for(size_t i=0; i < N; i++) {
        tft.fillRect(walkersBefore[i].x, walkersBefore[i].y, 2, 2, ELI_BLUE);
        tft.fillRect(walkers[i].x, walkers[i].y, 2, 2, TFT_GREEN);
      }
    }

    std::array<float, N>& getValues() override {
      return mods;
    };

private:
  std::array<float, N> mods;
};


template<size_t N>
class metaLorenz : public metaOsc<N> {
public:
    struct point {float x; float y;};

    metaLorenz() {
        this->modspeed.setMax(0.1);
        this->modspeed.setScale(0.001);
        this->moddepth.setMax(0.1);
        this->moddepth.setScale(0.0009);
        lastPoint = {120,120};
        lastlastPoint = {120,120};
        pastPoints.push_back(lastPoint);
    }

    String getName() override {return "lorenz";}

    void lorenz(const std::vector<float>& state, std::vector<float>& deriv) {
        float x = state[0];
        float y = state[1];
        float z = state[2];
        deriv[0] = sigma * (y - x);
        deriv[1] = x * (rho - z) - y;
        deriv[2] = x * y - beta * z;
    }

    void runge_kutta(std::vector<float>& state) {
        std::vector<float> k1(3), k2(3), k3(3), k4(3), temp(3);

        lorenz(state, k1);
        for (int i = 0; i < 3; ++i) temp[i] = state[i] + 0.5 * dt * k1[i];

        lorenz(temp, k2);
        for (int i = 0; i < 3; ++i) temp[i] = state[i] + 0.5 * dt * k2[i];

        lorenz(temp, k3);
        for (int i = 0; i < 3; ++i) temp[i] = state[i] + dt * k3[i];

        lorenz(temp, k4);
        for (int i = 0; i < 3; ++i) state[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }    

    std::array<float, N> update(const float (&adcs)[4]) override {
      sigma = 10.0 + (this->modspeed.getValue()*50.f);
      rho = 27 + (this->modspeed.getValue()*10.f);
      dt = (this->modspeed.getValue()*0.2) + 0.001;
      runge_kutta(state);    
      pastStates.push_back(state);
      if (pastStates.size() > 40) {
        pastStates.pop_front();
      }
      // for(size_t i=0; i < 3; i++) {
      //   Serial.printf("%f, ", state[i]);
      // }  
      // Serial.println();
      //position
      mods[0] = state[0] * 0.1 * this->moddepth.getValue();
      mods[1] = state[1] * 0.1 * this->moddepth.getValue();
      mods[2] = state[2] * 0.01 * this->moddepth.getValue();


      size_t top=pastStates.size()-1;
      if (top > 1) {
        //velocity
        mods[3] = (pastStates[top][0] - pastStates[top-1][0]) * 2.f * this->moddepth.getValue();
        mods[4] = (pastStates[top][1] - pastStates[top-1][1]) * 2.f * this->moddepth.getValue();
        mods[5] = (pastStates[top][2] - pastStates[top-1][2]) * 2.f * this->moddepth.getValue();
      }
      if (top > 2) {
        //velocity

        mods[6] = ((pastStates[top][0] - pastStates[top-1][0]) - (pastStates[top-1][0] - pastStates[top-2][0])) * 10.f * this->moddepth.getValue();
        mods[7] = ((pastStates[top][1] - pastStates[top-1][1]) - (pastStates[top-1][1] - pastStates[top-2][1])) * 10.f * this->moddepth.getValue();
        mods[8] = ((pastStates[top][2] - pastStates[top-1][2]) - (pastStates[top-1][2] - pastStates[top-2][2])) * 10.f * this->moddepth.getValue();
      }
      for(size_t i=6; i < 9; i++) {
        Serial.printf("%f, ", mods[i]);
      }  
      Serial.println();
      return mods;
    }

    //TODO: need to keep a deque of recent values (maybe 100), then draw a selection of these each time, and keep a track of which ones were drawn, 
    // each time delete the last line and add a new line
    // so need states deque and drawnsates deque

    void draw(TFT_eSPI &tft) override {
      constexpr size_t lineLength = 70;
      // if (pastPoints.size() > lineLength) {
      // }
      // tft.drawLine(lastlastPoint.x, lastlastPoint.y, lastPoint.x, lastPoint.y, ELI_BLUE); 
      // point newPoint = {120.f + (state[1] * 4.f), 120.f + (state[1] * 4.f)}; 
      point newPoint = {120.f + (state[1] * 4.f), sqbound + (state[2] * 3.5f)}; 
      tft.drawLine(pastPoints.back().x, pastPoints.back().y, newPoint.x, newPoint.y, TFT_CYAN); 
      // lastlastPoint = lastPoint;
      // lastPoint = newPoint;)
      pastPoints.push_back(newPoint);
      if (pastPoints.size() > lineLength) {
        tft.drawLine(pastPoints[0].x, pastPoints[0].y, pastPoints[1].x, pastPoints[1].y, ELI_BLUE); 
        pastPoints.pop_front();
      }
    }

    std::array<float, N>& getValues() override {
      return mods;
    };

private:
  std::array<float, N> mods;
  std::vector<float> state = {1.0, 1.0, 1.0};
  std::deque<std::vector<float> > pastStates;
  float sigma = 10.0;
  float rho = 28.0;
  const float beta = 8.0 / 3.0;
  double dt = 0.01;  // Time step
  point lastPoint, lastlastPoint;
  std::deque<point> pastPoints;
};

template <size_t N>
using metaOscPtr = std::shared_ptr<metaOsc<N>>;

