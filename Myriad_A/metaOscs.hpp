#pragma once

#include <stdlib.h>
#include <array>
#include "Arduino.h"

#include "src/memlp/MLP.h"
#include "drawing.h"
#include <deque>
#include "fixedpoint.hpp"
#include "sinetable_fixed.hpp"
using namespace FixedPoint;

// Shared sine table for all fixed-point oscillators (saves ~4KB per oscillator type)
namespace {
    SineTableQ16_16 g_shared_sine_table_fp;
}

enum MODTARGETS {PITCH, EPSILON, PITCH_AND_EPSILON} modTarget = MODTARGETS::PITCH;

enum class MetaOscType {
    NONE,
    MLP, 
    SINES,
    SINES_F_MULTIPLE,
    DRUNKEN_WALKERS,
    LORENZ,
    ROSSLER,
    BOIDS,
    AIZAWA
};

template <typename T>
class BoundedEncoderValue {
public:
  BoundedEncoderValue(T _min, T _max, T _scale) : maxVal(_max), minVal(_min), scaleVal(_scale), value(_min) {setMax(_max); setMin(_min);}
  BoundedEncoderValue() : scaleVal(T(0.01)), value(T(0)) {setMax(T(0.9)); setMin(T(0.0));}


  void update(const T change) {
    value += (change * scaleVal);
    value = value < minVal ? minVal : value; //std::max(minVal, value);
    value = value > maxVal ? maxVal : value; //std::min(maxVal, value);
  }

  inline T getValue() {
    return value;
  }

  inline void setValue(const T v) {
    value = v;
    value = value < minVal ? minVal : value; //std::max(minVal, value);
    value = value > maxVal ? maxVal : value; //std::min(maxVal, value);
  }

  void setScale(T newScale) {scaleVal = newScale;}
  
  void setMax(T newMax) {
    maxVal = newMax;
    value = value > maxVal ? maxVal : value; //std::min(maxVal, value);
    if (maxVal != T(0)) {
      invMax = T(1.0)/maxVal;
    }
  }
  void setMin(T newMin) {
    minVal = newMin; 
    value = value < minVal ? minVal : value; //std::max(minVal, value);
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

    virtual MetaOscType getType() const = 0;

    virtual std::array<float, N> update(const size_t (adcs)[4]) =0;    

    void setDepth(float delta) {
        moddepth.update(delta);
    }

    float getDepth() {
      return moddepth.getValue();
    }

    void restoreDepth(const float v) {
      moddepth.setValue(v);
    }

    void setSpeed(float delta) {
        modspeed.update(delta);
    }
    
    float getSpeed() {
      return modspeed.getValue();
    }

    void restoreSpeed(const float v) {
      modspeed.setValue(v);
    }

    virtual String getName() {return "";}

    virtual void draw(TFT_eSPI &tft) {};

    virtual std::array<float, N>& getValues() =0;

    virtual size_t getTimerMS() {
      return 20;
    }


protected:
  //add screen bounds
};

template<size_t N>
class metaOscFP {
public:
    metaOscFP() {};
    virtual ~metaOscFP() = default;
    BoundedEncoderValue<Q16_16> moddepth;
    BoundedEncoderValue<Q16_16> modspeed;

    virtual MetaOscType getType() const = 0;

    virtual std::array<Q16_16, N> update(const size_t (adcs)[4]) =0;    

    void setDepth(Q16_16 delta) {
        moddepth.update(delta);
    }

    Q16_16 getDepth() {
      return moddepth.getValue();
    }

    void restoreDepth(const Q16_16 v) {
      moddepth.setValue(v);
    }

    void setSpeed(Q16_16 delta) {
        modspeed.update(delta);
    }
    
    Q16_16 getSpeed() {
      return modspeed.getValue();
    }

    void restoreSpeed(const Q16_16 v) {
      modspeed.setValue(v);
    }

    virtual String getName() {return "";}

    virtual void draw(TFT_eSPI &tft) {};

    virtual std::array<Q16_16, N>& getValues() =0;

    virtual size_t getTimerMS() {
      return 20;
    }


protected:
  //add screen bounds
};

constexpr float TWOPI  = PI * 2.f;

template<size_t N>
class metaOscNone : public metaOsc<N> {
public:
    metaOscNone() {
      vals.fill(0);
    }

    String getName() override {return "no modulation";}

    MetaOscType getType() const override { return MetaOscType::NONE; }


    std::array<float, N> update(const size_t (adcs)[4]) override {
        return vals;
    }

    std::array<float, N>& getValues() override {
      return vals;
    };



private:
  std::array<float, N> vals;
};

template<size_t N>
class metaOscNoneFP : public metaOscFP<N> {
public:
    metaOscNoneFP() {
      vals.fill(Q16_16(0));
    }

    String getName() override {return "no modulation";}

    MetaOscType getType() const override { return MetaOscType::NONE; }


    std::array<Q16_16, N> update(const size_t (adcs)[4]) override {
        return vals;
    }

    std::array<Q16_16, N>& getValues() override {
      return vals;
    };



private:
  std::array<Q16_16, N> vals;
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

    MetaOscType getType() const override { return MetaOscType::SINES; }

    std::array<float, N> update(const size_t (adcs)[4]) override {
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
    std::array<float, N> sines{0};

};

template<size_t N>
class metaOscSinesFP : public metaOscFP<N>  {
public:
    using FixedType = Q16_16;

    /**
     * Constructor - Initializes N oscillators with equally spaced phases
     */
    metaOscSinesFP() {
        // Two PI in fixed-point
        constexpr FixedType TWOPI_FIXED = FixedType::from_float_ct(6.28318530718);

        // Initialize with equally spread phases (0, 2π/N, 4π/N, ...)
        const FixedType phaseGap = TWOPI_FIXED / FixedType(N);

        for(size_t i = 0; i < N; i++) {
            phasors[i] = phaseGap * FixedType(i);
        }
        this->modspeed.setMax(FixedType(0.1));
        this->modspeed.setScale(FixedType(0.001));
        this->moddepth.setMax(FixedType(0.1));
        this->moddepth.setScale(FixedType(0.0009));

    }

    /**
     * Update all oscillators and return sine wave values
     *
     * @param adcs Array of 4 ADC values (unused in sine version but kept for interface compatibility)
     * @return Array of N sine wave values in Q16.16 format, range ≈ [-moddepth, +moddepth]
     */
    std::array<FixedType, N> update(const size_t adcs[4]) {
        for(size_t i = 0; i < N; i++) {
            // Lookup sine value using optimized table
            sines[i] = g_shared_sine_table_fp.fast_sin_hw(phasors[i]) * this->moddepth.getValue();

            // Advance phase
            phasors[i] += this->modspeed.getValue();

            // Wrap phase (optional - sine table handles wraparound, but keeping phase bounded is cleaner)
            // The sine table supports -2π to 4π range, so we only wrap if way out of bounds
            if (phasors[i] > FixedType(12.566f)) {  // 2π * 2
                phasors[i] -= FixedType(6.283f);    // 2π
            }
        }

        return sines;
    }

    void draw(TFT_eSPI &tft) override { 
      const int32_t barwidth=4;
      constexpr int step = sqwidth / N;
      constexpr int stepoffset = (step+barwidth) / 2;
      for(size_t i=0; i < N; i++) {
        const int h = (sines[i] * sqhalfwidthFP * Q16_16(10)).to_int();
        const int left = sqbound + (step*i) + stepoffset;
        tft.fillRect(left,sqbound,barwidth, sqwidth, ELI_BLUE);
        if (h <0) {
          tft.fillRect(left,120+h,barwidth, -h, TFT_WHITE);
        }else{
          tft.fillRect(left,120,barwidth, std::max(h,1), TFT_WHITE);
        }
      }
    }


    /**
     * Get current sine values without updating
     */
    std::array<FixedType, N>& getValues() override {
        return sines;
    }


    /**
     * Get name of oscillator type
     */
    String getName() override {
        return "sines";
    }

    /**
     * Get oscillator type
     */
    MetaOscType getType() const override {
        return MetaOscType::SINES;
    }

    /**
     * Get current phasor values (for debugging/visualization)
     */
    const std::array<FixedType, N>& getPhasors() const {
        return phasors;
    }

private:
    // Oscillator state
    std::array<FixedType, N> phasors;   // Current phase for each oscillator
    std::array<FixedType, N> sines;     // Current sine values

};

template<size_t N>
class metaOscSinesFMultipleFP : public metaOscFP<N>  {
public:
    using FixedType = Q16_16;

    /**
     * Constructor - Initializes N oscillators with equally spaced phases and frequency multiples
     */
    metaOscSinesFMultipleFP() {
        // Two PI in fixed-point
        constexpr FixedType TWOPI_FIXED = FixedType::from_float_ct(6.28318530718);

        // Initialize with equally spread phases (0, 2π/N, 4π/N, ...)
        const FixedType phaseGap = TWOPI_FIXED / FixedType(N);

        for(size_t i = 0; i < N; i++) {
            phasors[i] = phaseGap * FixedType(i);
        }
        this->modspeed.setMax(FixedType(0.1));
        this->modspeed.setScale(FixedType(0.001));
        this->moddepth.setMax(FixedType(0.1));
        this->moddepth.setScale(FixedType(0.0009));

    }

    /**
     * Update all oscillators with frequency multiples and return sine wave values
     *
     * Each oscillator i runs at (i+1) times the base frequency, creating harmonic content.
     *
     * @param adcs Array of 4 ADC values (unused in sine version but kept for interface compatibility)
     * @return Array of N sine wave values in Q16.16 format, range ≈ [-moddepth, +moddepth]
     */
    std::array<FixedType, N> update(const size_t adcs[4]) {
        for(size_t i = 0; i < N; i++) {
            // Multiply phasor by frequency multiple (i+1) before sine lookup
            // This creates harmonics: osc 0 = 1x, osc 1 = 2x, osc 2 = 3x, etc.
            sines[i] = g_shared_sine_table_fp.fast_sin_hw(phasors[i] * FixedType(i + 1)) * this->moddepth.getValue();

            // Advance phase
            phasors[i] += this->modspeed.getValue();

            // Wrap phase (optional - sine table handles wraparound, but keeping phase bounded is cleaner)
            // The sine table supports -2π to 4π range, so we only wrap if way out of bounds
            if (phasors[i] > FixedType(12.566f)) {  // 2π * 2
                phasors[i] -= FixedType(6.283f);    // 2π
            }
        }

        return sines;
    }

    void draw(TFT_eSPI &tft) override {
      const int32_t barwidth=4;
      constexpr int step = sqwidth / N;
      constexpr int stepoffset = (step+barwidth) / 2;
      for(size_t i=0; i < N; i++) {
        const int h = (sines[i] * sqhalfwidthFP * Q16_16(10)).to_int();
        const int left = sqbound + (step*i) + stepoffset;
        tft.fillRect(left,sqbound,barwidth, sqwidth, ELI_BLUE);
        if (h <0) {
          tft.fillRect(left,120+h,barwidth, -h, TFT_RED);
        }else{
          tft.fillRect(left,120,barwidth, std::max(h,1), TFT_RED);
        }
      }
    }


    /**
     * Get current sine values without updating
     */
    std::array<FixedType, N>& getValues() override {
        return sines;
    }


    /**
     * Get name of oscillator type
     */
    String getName() override {
        return "speedy sines";
    }

    /**
     * Get oscillator type
     */
    MetaOscType getType() const override {
        return MetaOscType::SINES_F_MULTIPLE;
    }

    /**
     * Get current phasor values (for debugging/visualization)
     */
    const std::array<FixedType, N>& getPhasors() const {
        return phasors;
    }

private:
    // Oscillator state
    std::array<FixedType, N> phasors;   // Current phase for each oscillator
    std::array<FixedType, N> sines;     // Current sine values

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

    MetaOscType getType() const override { return MetaOscType::SINES_F_MULTIPLE; }

    std::array<float, N> update(const size_t (adcs)[4]) override {
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
    std::array<float, N> sines{0};

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

    MetaOscType getType() const override { return MetaOscType::MLP; }

    void randomise() {
      net->DrawWeights();
    }

    String getName() override {return "neural net";}

    std::array<float, N> update(const size_t (adcs)[4]) override {
      // Serial.printf("nn ph %f %d\n",phasor, clockCount);
      if (clockCount == 0) {
        //send controls and phasor into the neural network
        std::vector<float> netInput {sinf(phasor*3.f),adcs[0] * adcMul,adcs[1] * adcMul,adcs[2] * adcMul,adcs[3] * adcMul,1.f};
        for(size_t i=0; i < netInput.size(); i++) {
          netInput[i] *= this->moddepth.getValue();
        }
        std::vector<float> output(N);
        net->GetOutput(netInput, &output);
        phasor += this->modspeed.getValue();
        if (phasor > TWOPI) phasor -= TWOPI;
        std::copy(output.begin(), output.begin() + output.size(), arrOutput.begin());
        for(size_t i=0; i < N; i++) {
          arrOutput[i] -= 0.5f;
          arrOutput[i] *= 0.1f;
          // Serial.printf("%f\t",arrOutput[i]);
        }
        // Serial.println();
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
          // const int32_t col = 32767 + (20000 * layerOutputs[i]);
          size_t colourIdx = static_cast<size_t>(layerOutputs[i] * 64.f);
          colourIdx = std::min(colourIdx,63U);
          const int16_t col = gradient_colors[colourIdx];
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
    std::vector<std::deque<point>> pastStates;
    std::array<int, N> colours;


    metaDrunkenWalkers() {
        //init with equally spread phase
        for(size_t i=0; i < N; i++) {
            walkers[i].x=120;
            walkers[i].y=120;
            colours[i] = rgbTo565(i, 100 + (i * 10), i*3);
        }
        this->modspeed.setMax(0.1);
        this->modspeed.setScale(0.001);
        this->moddepth.setMax(0.1);
        this->moddepth.setScale(0.0009);

        pastStates.resize(N);
    }

    String getName() override {return "drunk walkers";}
    MetaOscType getType() const override { return MetaOscType::DRUNKEN_WALKERS; }
    size_t getTimerMS() override {
      return 5;
    }



    std::array<float, N> update(const size_t adcs[4]) override {
        for(size_t i=0; i < N; i++) {

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
          mods[i] = sqrtf((dx * dx) + (dy * dy)) * this->moddepth.getValue() * 0.01; 


        }
        return mods;
    }

    void draw(TFT_eSPI &tft) override { 

      for(size_t i=0; i < N; i++) {
        while(pastStates[i].size() > 30) {
          tft.fillRect(pastStates[i].front().x, pastStates[i].front().y, 2, 2, ELI_BLUE);
          pastStates[i].pop_front();
        }
        tft.fillRect(walkers[i].x, walkers[i].y, 2, 2, colours[i]);
        pastStates[i].push_back(walkers[i]);

      }
    }

    std::array<float, N>& getValues() override {
      return mods;
    };

private:
  std::array<float, N> mods;
};

template<size_t N>
class metaDrunkenWalkersFP : public metaOscFP<N> {
public:
    using FixedType = Fixed<10,22>;

    struct pointFP {
        FixedType x;
        FixedType y;
    };

    std::array<pointFP, N> walkers;
    std::vector<std::deque<pointFP>> pastStates;
    std::array<int, N> colours;

    metaDrunkenWalkersFP() {
        // Initialize all walkers at center (120, 120)
        for(size_t i=0; i < N; i++) {
            walkers[i].x = FixedType(120);
            walkers[i].y = FixedType(120);
            colours[i] = rgbTo565(i, 100 + (i * 10), i*3);
        }

        // Set parameter limits (matching float version)
        this->modspeed.setMax(Q16_16(0.1));
        this->modspeed.setScale(Q16_16(0.001));
        this->moddepth.setMax(Q16_16(0.1));
        this->moddepth.setScale(Q16_16(0.0009));

        pastStates.resize(N);
    }

    String getName() override { return "drunk walkers"; }

    MetaOscType getType() const override {
        return MetaOscType::DRUNKEN_WALKERS;
    }

    size_t getTimerMS() override {
        return 5;
    }

    std::array<Q16_16, N> update(const size_t adcs[4]) override {
        // Fixed-point constants for spatial bounds
        const FixedType sqboundFP = FixedType(sqbound);
        const FixedType sqboundBRFP = FixedType(sqboundBR);
        const FixedType sqwidthFP = FixedType(sqwidth);
        const FixedType centerX = FixedType(120);
        const FixedType centerY = FixedType(120);
        const FixedType depthScale = FixedType(0.01f);


        for(size_t i=0; i < N; i++) {
            // Random walk: convert random(-2000, 2000) to fixed-point range [-20, +20]
            FixedType randomDeltaX = FixedType::random_hw(FixedType::from_int(-20), FixedType::from_int(20));
            FixedType randomDeltaY = FixedType::random_hw(FixedType::from_int(-20), FixedType::from_int(20));

            FixedType deltaX = randomDeltaX.mulWith(this->modspeed.getValue());
            FixedType deltaY = randomDeltaY.mulWith(this->modspeed.getValue());

        static int debugCounter = 0;
        if (debugCounter++ % 100 == 0 && i==0) {
            Serial.printf("%f\t%f\n", randomDeltaX.to_float(), randomDeltaY.to_float());
        }

            walkers[i].x += deltaX;
            walkers[i].y += deltaY;

            // Wrap at boundaries
            if (walkers[i].x < sqboundFP) {
                walkers[i].x += sqwidthFP;
            } else if (walkers[i].x > sqboundBRFP) {
                walkers[i].x -= sqwidthFP;
            }

            if (walkers[i].y < sqboundFP) {
                walkers[i].y += sqwidthFP;
            } else if (walkers[i].y > sqboundBRFP) {
                walkers[i].y -= sqwidthFP;
            }

            // Calculate distance from center
            const FixedType dx = centerX - walkers[i].x;
            const FixedType dy = centerY - walkers[i].y;

            // Distance = sqrt(dx^2 + dy^2)
            const FixedType dx2 = dx * dx;
            const FixedType dy2 = dy * dy;
            const FixedType distance = FixedPoint::sqrt(dx2 + dy2);

            // Modulation output: distance scaled by depth
             FixedType modVal = distance * FixedType(this->moddepth.getValue()) * depthScale;
             mods[i] = Q16_16(modVal); // convert to q16_16
        }

        return mods;
    }

    void draw(TFT_eSPI &tft) override {
        for(size_t i=0; i < N; i++) {
            // Erase old trail segments
            while(pastStates[i].size() > 30) {
                int px = pastStates[i].front().x.to_int();
                int py = pastStates[i].front().y.to_int();
                tft.fillRect(px, py, 2, 2, ELI_BLUE);
                pastStates[i].pop_front();
            }

            // Draw current position
            int wx = walkers[i].x.to_int();
            int wy = walkers[i].y.to_int();
            tft.fillRect(wx, wy, 2, 2, colours[i]);

            // Add to trail history
            pastStates[i].push_back(walkers[i]);
        }
    }

    std::array<Q16_16, N>& getValues() override {
        return mods;
    }

private:
    std::array<Q16_16, N> mods;
};

// Forward declaration for 2D screen point
struct point {float x; float y;};

// Base class for fixed-point chaotic attractors with Runge-Kutta integration
template<typename FixedType, size_t N>
class metaAttractorFP : public metaOscFP<N> {
protected:
    // 3D state vector in fixed-point
    std::array<FixedType, 3> state;

    // History for velocity/acceleration calculations
    std::deque<std::array<FixedType, 3>> pastStates;

    // Drawing history (use float for screen coordinates)
    std::deque<point> pastPoints;

    // Time step (variable)
    FixedType dt;

    // Output modulations (Q16_16 for interface consistency)
    std::array<Q16_16, N> mods;

public:
    // Virtual methods to be implemented by derived classes
    virtual void iterate(const std::array<FixedType, 3>& state,
                        std::array<FixedType, 3>& deriv) = 0;

    virtual void updateCoefficients() = 0;

    virtual point project2D(const std::array<FixedType, 3>& state) = 0;

    size_t getTimerMS() override {
        return 5;
    }

    // Shared RK4 implementation
    void runge_kutta(std::array<FixedType, 3>& state) {
        std::array<FixedType, 3> k1, k2, k3, k4, temp;

        // k1 = f(state)
        iterate(state, k1);

        // temp = state + 0.5*dt*k1
        const FixedType halfDt = dt * FixedType(0.5);
        for (int i = 0; i < 3; ++i) {
            temp[i] = state[i] + halfDt * k1[i];
        }

        // k2 = f(temp)
        iterate(temp, k2);

        // temp = state + 0.5*dt*k2
        for (int i = 0; i < 3; ++i) {
            temp[i] = state[i] + halfDt * k2[i];
        }

        // k3 = f(temp)
        iterate(temp, k3);

        // temp = state + dt*k3
        for (int i = 0; i < 3; ++i) {
            temp[i] = state[i] + dt * k3[i];
        }

        // k4 = f(temp)
        iterate(temp, k4);

        // state += (dt/6) * (k1 + 2k2 + 2k3 + k4)
        const FixedType dtDiv6 = dt * FixedType(1.0/6.0);
        const FixedType two = FixedType(2);

        for (int i = 0; i < 3; ++i) {
            FixedType sum = k1[i] + two*k2[i] + two*k3[i] + k4[i];
            state[i] += dtDiv6 * sum;
        }
    }

    // Shared update logic
    std::array<Q16_16, N> update(const size_t adcs[4]) override {
        // Update dynamic parameters
        updateCoefficients();

        // Integrate one step
        runge_kutta(state);

        // Store history (limit to 85 frames)
        pastStates.push_back(state);
        if (pastStates.size() > 85) {
            pastStates.pop_front();
        }

        // Convert internal fixed-point to Q16_16 output
        // Position modulations
        mods[0] = Q16_16(state[0].to_float() * 0.1) * this->moddepth.getValue();
        mods[1] = Q16_16(state[1].to_float() * 0.1) * this->moddepth.getValue();
        mods[2] = Q16_16(state[2].to_float() * 0.01) * this->moddepth.getValue();

        // Velocity modulations (if N >= 6)
        if (N >= 6 && pastStates.size() > 1) {
            size_t top = pastStates.size() - 1;
            for (int i = 0; i < 3; i++) {
                FixedType vel = (pastStates[top][i] - pastStates[top-1][i]) * FixedType(2.0);
                mods[3+i] = Q16_16(vel.to_float()) * this->moddepth.getValue();
            }
        }

        // Acceleration modulations (if N >= 9)
        if (N >= 9 && pastStates.size() > 2) {
            size_t top = pastStates.size() - 1;
            for (int i = 0; i < 3; i++) {
                FixedType accel = ((pastStates[top][i] - pastStates[top-1][i]) -
                                  (pastStates[top-1][i] - pastStates[top-2][i])) * FixedType(10.0);
                mods[6+i] = Q16_16(accel.to_float()) * this->moddepth.getValue();
            }
        }

        return mods;
    }

    // Shared drawing logic
    void draw(TFT_eSPI &tft) override {
        constexpr size_t lineLength = 70;
        const point newPoint = project2D(state);

        if (!pastPoints.empty()) {
            tft.drawLine(pastPoints.back().x, pastPoints.back().y,
                        newPoint.x, newPoint.y, TFT_CYAN);
        }

        pastPoints.push_back(newPoint);

        // Erase old trail segments
        if (pastPoints.size() > lineLength) {
            tft.drawLine(pastPoints[0].x, pastPoints[0].y,
                        pastPoints[1].x, pastPoints[1].y, ELI_BLUE);
            pastPoints.pop_front();
        }
    }

    std::array<Q16_16, N>& getValues() override {
        return mods;
    }
};

template<size_t N>
class metaLorenzFP : public metaAttractorFP<Q16_16, N> {
public:
    using FixedType = Q16_16;

    metaLorenzFP() {
        // Initial state
        this->state[0] = FixedType(1.1);
        this->state[1] = FixedType(2.0);
        this->state[2] = FixedType(7.0);

        // Parameters
        sigma = FixedType(10.0);
        rho = FixedType(28.0);
        beta = FixedType(8.0/3.0);
        this->dt = FixedType(0.01);

        // Base class parameter setup
        this->modspeed.setMax(Q16_16(0.1));
        this->modspeed.setScale(Q16_16(0.001));
        this->moddepth.setMax(Q16_16(0.1));
        this->moddepth.setScale(Q16_16(0.0009));
    }

    String getName() override { return "lorenz"; }

    MetaOscType getType() const override {
        return MetaOscType::LORENZ;
    }

    void iterate(const std::array<FixedType, 3>& state,
                 std::array<FixedType, 3>& deriv) override {
        FixedType x = state[0];
        FixedType y = state[1];
        FixedType z = state[2];

        // Lorenz equations:
        // dx/dt = σ(y - x)
        // dy/dt = x(ρ - z) - y
        // dz/dt = xy - βz
        deriv[0] = sigma * (y - x);
        deriv[1] = x * (rho - z) - y;
        deriv[2] = x * y - beta * z;
    }

    void updateCoefficients() override {
        // Convert modspeed (Q16_16) to Q12_20 for calculation
        float speed = this->modspeed.getValue().to_float();

        sigma = FixedType(10.0 + speed * 50.0);
        rho = FixedType(28.0 + speed * 10.0);
        this->dt = FixedType(0.001 + speed * 0.07);
    }

    point project2D(const std::array<FixedType, 3>& state) override {
        point p;
        p.x = 120.0f + (state[1].to_float() * 4.0f);
        p.y = 20.0f + (state[2].to_float() * 3.5f);
        return p;
    }

private:
    FixedType sigma, rho, beta;
};

template<size_t N>
class metaRosslerFP : public metaAttractorFP<Q12_20, N> {
public:
    using FixedType = Q12_20;

    metaRosslerFP() {
        // Initial state
        this->state[0] = FixedType(10.0);
        this->state[1] = FixedType(0.0);
        this->state[2] = FixedType(10.0);

        // Parameters
        a = FixedType(0.2);
        b = FixedType(0.2);
        c = FixedType(5.7);
        this->dt = FixedType(0.01);

        // Base class parameter setup
        this->modspeed.setMax(Q16_16(0.1));
        this->modspeed.setScale(Q16_16(0.001));
        this->moddepth.setMax(Q16_16(0.1));
        this->moddepth.setScale(Q16_16(0.0009));
    }

    String getName() override { return "rossler"; }

    MetaOscType getType() const override {
        return MetaOscType::ROSSLER;
    }

    void iterate(const std::array<FixedType, 3>& state,
                 std::array<FixedType, 3>& deriv) override {
        FixedType x = state[0];
        FixedType y = state[1];
        FixedType z = state[2];

        // Rössler equations:
        // dx/dt = -(y + z)
        // dy/dt = x + ay
        // dz/dt = b + z(x - c)
        deriv[0] = -(y + z);
        deriv[1] = x + a * y;
        deriv[2] = b + z * (x - c);
    }

    void updateCoefficients() override {
        // Convert modspeed (Q16_16) to Q12_20 for calculation
        float speed = this->modspeed.getValue().to_float();

        a = FixedType(0.2 + speed * 0.5);
        b = FixedType(0.2 + speed * (-0.1));
        c = FixedType(5.7 + speed * (-2.4));
        this->dt = FixedType(0.001 + speed * 0.3);
    }

    point project2D(const std::array<FixedType, 3>& state) override {
        point p;
        p.x = 120.0f + (state[0].to_float() * 7.0f);
        p.y = 120.0f + (state[1].to_float() * 4.0f);
        return p;
    }

private:
    FixedType a, b, c;
};

template<size_t N>
class metaOscBoidsFP : public metaOscFP<N> {
public:
    using FixedType = Q16_16;

    struct boidFP {
        Q16_16 x, y;        // Current position
        Q16_16 px, py;      // Previous position (for drawing)
        Q16_16 vx, vy;      // Current velocity
        Q16_16 pvx, pvy;    // Previous velocity (for drawing)
    };

    metaOscBoidsFP() {
        boids = std::vector<boidFP>(N);

        for(auto &v: boids) {
            v.x = Q16_16(random(sqbound, sqboundBR));
            v.y = Q16_16(random(sqbound, sqboundBR));
            v.px = v.x;
            v.py = v.y;
            v.vx = Q16_16(((random(1000) / 1000.f) - 0.5f) * 0.8f);
            v.vy = Q16_16(((random(1000) / 1000.f) - 0.5f) * 0.8f);
            v.pvx = v.vx;
            v.pvy = v.vy;
        }

        centerX = Q16_16(120);
        centerY = Q16_16(120);

        // Use Q16_16 parameters
        this->modspeed.setMax(Q16_16(1.0));
        this->modspeed.setScale(Q16_16(0.01));
        this->moddepth.setMax(Q16_16(0.03));
        this->moddepth.setScale(Q16_16(0.0003));
    }

    String getName() override { return "boids"; }

    MetaOscType getType() const override {
        return MetaOscType::BOIDS;
    }

    inline Q16_16 distBetween(Q16_16 x1, Q16_16 y1, Q16_16 x2, Q16_16 y2) {
        Q16_16 dx = x2 - x1;
        Q16_16 dy = y2 - y1;
        return FixedPoint::sqrt(dx * dx + dy * dy);
    }

    std::array<Q16_16, N> update(const size_t adcs[4]) override {
        // Calculate center of mass
        Q16_16 sumX = Q16_16(0);
        Q16_16 sumY = Q16_16(0);
        for(auto &v: boids) {
            sumX += v.x;
            sumY += v.y;
        }
        centerX = sumX / Q16_16(N);
        centerY = sumY / Q16_16(N);

        // Fixed-point constants
        const Q16_16 cohesionRadius = Q16_16(45.0);
        const Q16_16 separationRadius = Q16_16(30.0);
        const Q16_16 alignmentRadius = Q16_16(40.0);
        const Q16_16 cohesionStrength = Q16_16(0.005);
        const Q16_16 separationMul = Q16_16(0.2);
        const Q16_16 alignmentStrength = Q16_16(0.05);
        const Q16_16 maxSpeed = Q16_16(2.5);

        const Q16_16 sqboundFP = Q16_16(sqbound);
        const Q16_16 sqboundBRFP = Q16_16(sqboundBR);
        const Q16_16 sqwidthFP = Q16_16(sqwidth);

        Q16_16 modvel = Q16_16(0.1) + this->modspeed.getValue();

        // Neighbor cache for optimization
        struct NeighborInfo {
            Q16_16 dist;
            boidFP* ptr;
        };
        NeighborInfo neighbors[N];

        size_t idx = 0;
        for(auto &v: boids) {
            // Cache all distances once
            int neighborCount = 0;
            for(auto &other: boids) {
                if (&v != &other) {
                    neighbors[neighborCount].dist = distBetween(v.x, v.y, other.x, other.y);
                    neighbors[neighborCount].ptr = &other;
                    neighborCount++;
                }
            }

            // Rule 1: Cohesion - move towards center of mass
            Q16_16 dcxr1 = (centerX - v.x) * cohesionStrength;
            Q16_16 dcyr1 = (centerY - v.y) * cohesionStrength;

            // Rule 2 & 3: Separation and Alignment
            Q16_16 dcxr2 = Q16_16(0);
            Q16_16 dcyr2 = Q16_16(0);
            Q16_16 dcxr3 = Q16_16(0);
            Q16_16 dcyr3 = Q16_16(0);
            Q16_16 avgVx = Q16_16(0);
            Q16_16 avgVy = Q16_16(0);
            int alignCount = 0;

            for(int i = 0; i < neighborCount; i++) {
                auto& info = neighbors[i];
                auto* other = info.ptr;

                // Rule 2: Separation - keep away from other boids
                if (info.dist < separationRadius && info.dist > Q16_16(0)) {
                    Q16_16 force = Q16_16(1) / info.dist;
                    dcxr2 += (v.x - other->x) * force;
                    dcyr2 += (v.y - other->y) * force;
                }

                // Rule 3: Alignment - match velocity with nearby boids
                if (info.dist < alignmentRadius) {
                    avgVx += other->vx;
                    avgVy += other->vy;
                    alignCount++;
                }
            }

            dcxr2 = dcxr2 * separationMul;
            dcyr2 = dcyr2 * separationMul;

            if (alignCount > 0) {
                avgVx = avgVx / Q16_16(alignCount);
                avgVy = avgVy / Q16_16(alignCount);
                dcxr3 = (avgVx - v.vx) * alignmentStrength;
                dcyr3 = (avgVy - v.vy) * alignmentStrength;
            }

            // Update velocity
            v.vx += dcxr1 + dcxr2 + dcxr3;
            v.vy += dcyr1 + dcyr2 + dcyr3;

            // Speed limiting
            Q16_16 speed = FixedPoint::sqrt(v.vx * v.vx + v.vy * v.vy);

            if (speed > maxSpeed) {
                v.vx = (v.vx / speed) * maxSpeed;
                v.vy = (v.vy / speed) * maxSpeed;
                speed = maxSpeed;
            }

            // Update position
            v.x += v.vx * modvel;
            v.y += v.vy * modvel;

            // Toroidal wrapping
            if (v.x > sqboundBRFP) {
                v.x -= sqwidthFP;
            } else if (v.x < sqboundFP) {
                v.x += sqwidthFP;
            }

            if (v.y > sqboundBRFP) {
                v.y -= sqwidthFP;
            } else if (v.y < sqboundFP) {
                v.y += sqwidthFP;
            }

            // Output: speed scaled by depth
            mods[idx] = speed * this->moddepth.getValue();
            idx++;
        }

        return mods;
    }

    void draw(TFT_eSPI &tft) override {
        const Q16_16 lineLength = Q16_16(4);
        const int sqboundInt = (int)sqbound;
        const int sqboundBRInt = (int)sqboundBR;

        for(auto &v: boids) {
            // Convert fixed-point to screen coordinates
            int px = v.px.to_int();
            int py = v.py.to_int();
            int x = v.x.to_int();
            int y = v.y.to_int();

            // Draw previous position and velocity
            tft.drawCircle(px, py, 3, ELI_BLUE);
            int tx = constraini(px + (v.pvx * lineLength).to_int(), sqboundInt, sqboundBRInt);
            int ty = constraini(py + (v.pvy * lineLength).to_int(), sqboundInt, sqboundBRInt);
            tft.drawLine(px, py, tx, ty, ELI_BLUE);

            // Draw current position and velocity
            tft.drawCircle(x, y, 3, TFT_GREENYELLOW);
            tx = constraini(x + (v.vx * lineLength).to_int(), sqboundInt, sqboundBRInt);
            ty = constraini(y + (v.vy * lineLength).to_int(), sqboundInt, sqboundBRInt);
            tft.drawLine(x, y, tx, ty, TFT_YELLOW);

            // Update history
            v.pvx = v.vx;
            v.pvy = v.vy;
            v.px = v.x;
            v.py = v.y;
        }
    }

    std::array<Q16_16, N>& getValues() override {
        return mods;
    }

    inline int constraini(int val, int minVal, int maxVal) {
        if (val < minVal) return minVal;
        if (val > maxVal) return maxVal;
        return val;
    }

private:
    std::array<Q16_16, N> mods;
    std::vector<boidFP> boids;
    Q16_16 centerX, centerY;
};

template<size_t N>
class metaOscMLPFP : public metaOscFP<N> {
public:
    using FixedType = Q16_16;

    metaOscMLPFP() {
        // Create float-based neural network (architecture unchanged)
        net = new MLP<float>({ 6, 16, 8, 5, N }, {
            ACTIVATION_FUNCTIONS::RELU,
            ACTIVATION_FUNCTIONS::LINEAR,
            ACTIVATION_FUNCTIONS::RELU,
            ACTIVATION_FUNCTIONS::SIGMOID
        });
        net->SetCachedLayerOutputs(true);
        net->DrawWeights();

        // Fixed-point state initialization
        phasor = Q16_16(0);

        // Fixed-point parameters
        this->modspeed.setMax(Q16_16(0.1));
        this->modspeed.setScale(Q16_16(0.001));
        this->moddepth.setMax(Q16_16(0.1));
        this->moddepth.setScale(Q16_16(0.0009));
    }

    ~metaOscMLPFP() {
        delete net;
    }

    String getName() override { return "neural net"; }

    MetaOscType getType() const override {
        return MetaOscType::MLP;
    }

    void randomise() {
        net->DrawWeights();
    }

    std::array<Q16_16, N> update(const size_t adcs[4]) override {
        if (clockCount == 0) {
            // ================================================================
            // FIXED → FLOAT CONVERSION FOR MLP INPUT
            // ================================================================

            // Calculate sine of phasor using shared sine table
            Q16_16 phasorScaled = phasor * Q16_16(3);
            Q16_16 sineValueFP = g_shared_sine_table_fp.fast_sin_hw(phasorScaled);

            // Get depth as Q16_16
            Q16_16 depth = this->moddepth.getValue();

            // Prepare MLP inputs (convert Q16_16 → float)
            std::vector<float> netInput(6);
            netInput[0] = (sineValueFP * depth).to_float();
            netInput[1] = ((Q16_16(adcs[0]) * adcMul) * depth).to_float();
            netInput[2] = ((Q16_16(adcs[1]) * adcMul) * depth).to_float();
            netInput[3] = ((Q16_16(adcs[2]) * adcMul) * depth).to_float();
            netInput[4] = ((Q16_16(adcs[3]) * adcMul) * depth).to_float();
            netInput[5] = depth.to_float();  // Bias term scaled by depth

            // ================================================================
            // RUN NEURAL NETWORK (FLOAT DOMAIN)
            // ================================================================

            std::vector<float> output(N);
            net->GetOutput(netInput, &output);

            // ================================================================
            // FLOAT → FIXED CONVERSION FOR OUTPUT
            // ================================================================

            // Post-process and convert to Q16_16
            const Q16_16 half = Q16_16(0.5);
            const Q16_16 scale = Q16_16(0.1);

            for(size_t i = 0; i < N; i++) {
                // Convert float output to Q16_16, center and scale
                Q16_16 value = Q16_16(output[i]);
                arrOutput[i] = (value - half) * scale;
            }

            // ================================================================
            // UPDATE PHASOR (FIXED-POINT)
            // ================================================================

            phasor += this->modspeed.getValue();

            // Wrap phasor at 2π
            const Q16_16 TWOPI_FP = Q16_16(6.28318530718);
            if (phasor > TWOPI_FP) {
                phasor -= TWOPI_FP;
            }
        }

        // Clock divider
        clockCount++;
        if (clockCount == clockDiv) {
            clockCount = 0;
        }

        return arrOutput;
    }

    // Draw neural network layer visualization
    void draw(TFT_eSPI &tft) override {
        float steph = sqwidth / net->m_layers.size();
        float barh = steph * 0.5f;
        float barOffset = (steph - barh) * 0.5f;

        for(size_t i_layer = 0; i_layer < net->m_layers.size(); i_layer++) {
            auto layerOutputs = net->m_layers[i_layer].cachedOutputs;
            const float step = sqwidth / layerOutputs.size();
            const float barwidth = step * 0.5f;
            const float stepoffset = (step - barwidth) * 0.5f;

            for(size_t i = 0; i < layerOutputs.size(); i++) {
                size_t colourIdx = static_cast<size_t>(layerOutputs[i] * 64.f);
                colourIdx = std::min(colourIdx, 63U);
                const int16_t col = gradient_colors[colourIdx];
                const float left = sqbound + (step * i) + stepoffset;
                tft.fillRect(left, sqbound + (steph * i_layer) + barOffset,
                            barwidth, barh, col);
            }
        }
    }

    std::array<Q16_16, N>& getValues() override {
        return arrOutput;
    }

private:
    MLP<float> *net;                    // Neural network (float internally)
    Q16_16 phasor;                      // Phasor state (Q16_16 for determinism)
    std::array<Q16_16, N> arrOutput;    // Output buffer (Q16_16)
    size_t clockDiv = 2;                // Update every N calls
    size_t clockCount = 0;              // Clock counter
    const Q16_16 adcMul = Q16_16(1.0 / 4096.0);  // ADC normalization constant
};

template<size_t N>
class metaLorenz : public metaOsc<N> {
public:

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

  MetaOscType getType() const override { return MetaOscType::LORENZ; }    

    size_t getTimerMS() override {
      return 5;
    }


    virtual void iterate(const std::vector<float>& state, std::vector<float>& deriv) {
        float x = state[0];
        float y = state[1];
        float z = state[2];
        deriv[0] = sigma * (y - x);
        deriv[1] = x * (rho - z) - y;
        deriv[2] = x * y - beta * z;
    }

    void runge_kutta(std::vector<float>& state) {
        std::vector<float> k1(3), k2(3), k3(3), k4(3), temp(3);

        iterate(state, k1);
        for (int i = 0; i < 3; ++i) temp[i] = state[i] + 0.5f * dt * k1[i];

        iterate(temp, k2);
        for (int i = 0; i < 3; ++i) temp[i] = state[i] + 0.5f * dt * k2[i];

        iterate(temp, k3);
        for (int i = 0; i < 3; ++i) temp[i] = state[i] + dt * k3[i];

        iterate(temp, k4);
        for (int i = 0; i < 3; ++i) state[i] += (dt * (1.0/6.0f)) * (k1[i] + 2.f * k2[i] + 2.f * k3[i] + k4[i]);
    }    

    virtual void updateCoefficients() {
      sigma = 10.0 + (this->modspeed.getValue()*50.f);
      rho = 28.0 + (this->modspeed.getValue()*10.f);
      dt = (this->modspeed.getValue()*0.07f) + 0.001f;
    }

    std::array<float, N> update(const size_t (adcs)[4]) override {
      // sigma = 10.0 + (this->modspeed.getValue()*50.f);
      // rho = 27 + (this->modspeed.getValue()*10.f);
      // dt = (this->modspeed.getValue()*0.2) + 0.001;
      updateCoefficients();
      runge_kutta(state);    
      pastStates.push_back(state);
      if (pastStates.size() > 85) {
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
      // for(size_t i=6; i < 9; i++) {
      //   Serial.printf("%f, ", mods[i]);
      // }  
      // Serial.println();
      return mods;
    }

    virtual point project2D(const std::vector<float>& state) {
      //project xD state to 2D
      point p;
      p.x = 120.f + (state[1] * 4.f);
      p.y = 20.f + (state[2] * 3.5f);
      // Serial.printf("projected point: %f, %f\n", p.x, p.y);
      return p;
    }

    void draw(TFT_eSPI &tft) override {
      constexpr size_t lineLength = 70;
      const point newPoint = project2D(state); 
      tft.drawLine(pastPoints.back().x, pastPoints.back().y, newPoint.x, newPoint.y, TFT_CYAN); 
      // lastlastPoint = lastPoint;
      // lastPoint = newPoint;)
      pastPoints.push_back(newPoint);
      //blank out last segment
      if (pastPoints.size() > lineLength) {
        tft.drawLine(pastPoints[0].x, pastPoints[0].y, pastPoints[1].x, pastPoints[1].y, ELI_BLUE); 
        pastPoints.pop_front();
      }
    }

    std::array<float, N>& getValues() override {
      return mods;
    };

protected:
  double dt = 0.01;  // Time step
  point lastPoint, lastlastPoint;
  std::deque<point> pastPoints;
  std::array<float, N> mods;
  std::vector<float> state = {1.1,2,7};
  std::deque<std::vector<float> > pastStates;

private:
  float sigma = 10.0;
  float rho = 28.0;
  const float beta = 8.0 / 3.0;
};


//this one is (apparently well know to be) unstable, doesn't work great without small dt values
template<size_t N>
class metaAizawa : public metaLorenz<N> {
public:

    metaAizawa() : metaLorenz<N>() {
        // this->modspeed.setMax(0.1);
        // this->modspeed.setScale(0.001);
        // this->moddepth.setMax(0.1);
        // this->moddepth.setScale(0.0009);
        // lastPoint = {120,120};
        // lastlastPoint = {120,120};
        // pastPoints.push_back(lastPoint);
        this->state[0] = -0.8f; // x
        this->state[1] = -0.37f; // y
        this->state[2] = -0.66f; // z
    }

    String getName() override {return "aizawa";}

    MetaOscType getType() const override { return MetaOscType::AIZAWA; }    

    void iterate(const std::vector<float>& currstate, std::vector<float>& deriv) override {
        float x = currstate[0];
        float y = currstate[1];
        float z = currstate[2];
        deriv[0] = (z-b) * x - d*y;
        deriv[1] = d * x + (z-b) * y;
        deriv[2] = c + (a*z) - ((z*z*z)*0.3333333f) - (x*x) + (f * z * (x*x*x));
    }

    void updateCoefficients() override{
      // a = 0.95 + (this->modspeed.getValue() * 0.01);
      this->dt = (this->modspeed.getValue()*0.2) + 0.001;
    }

    point project2D(const std::vector<float>& currstate) override {
      //project 3D state to 2D
      point p;
      p.x = 120.f + (currstate[1] * 40.f);
      p.y = 120.f + (currstate[2] * 40.f);
      return p;
    }

private:
    float a=1.52;
    float b= 1; 
    float c=1.14;
    float d=1.86;
    float e=1.81;
    float f=0.85;
};


//hopefully this is more stable than Aizawa?
template<size_t N>
class metaRossler : public metaLorenz<N> {
public:

    metaRossler() : metaLorenz<N>() {
        this->state[0] = 10.f; // x
        this->state[1] = 0.f; // y
        this->state[2] = 10.f; // z
    }

    String getName() override {return "rossler";}

    MetaOscType getType() const override { return MetaOscType::ROSSLER; }    

    void iterate(const std::vector<float>& currstate, std::vector<float>& deriv) override {
        float x = currstate[0];
        float y = currstate[1];
        float z = currstate[2];
        deriv[0] = -(y+z);
        deriv[1] = x + (a * y);
        deriv[2] = b + (z * (x - c));
    }

    void updateCoefficients() override{
      a = 0.2 + (this->modspeed.getValue() * 0.5f);
      b = 0.2 + (this->modspeed.getValue() * -0.1f);
      c = 5.7 + (this->modspeed.getValue() * -2.4f);
      this->dt = (this->modspeed.getValue()*0.3f) + 0.001f;
    }

    point project2D(const std::vector<float>& currstate) override {
      //project 3D state to 2D
      point p;
      p.x = 120.f + (currstate[0] * 7.f);
      p.y = 120.f + (currstate[1] * 4.f);
      return p;
    }

private:
    float a=0.2;
    float b= 0.2; 
    float c=5.7;
};

// struct boid {
//   float px;
//   float py;
//   float pvx;
//   float pvy;
//   float x;
//   float y;
//
//   float vx;
//   float vy;
// };
//
// template<size_t N>
// class metaOscBoids : public metaOsc<N> {
// public:
//     metaOscBoids() {
//       boids = std::vector<boid>(N);
//       for(auto &v: boids) {
//         v.x = random(sqbound,sqboundBR);
//         v.y = random(sqbound,sqboundBR);
//         v.px=v.x;
//         v.py=v.y;
//         v.vx = ((random(1000) / 1000.f) - 0.5f) * 0.8f;
//         v.vy = ((random(1000) / 1000.0f) - 0.5f) * 0.8f;
//         v.pvx=v.vx;
//         v.pvy=v.vy;
//       }
//       centerX = 120;
//       centerY = 120;
//
//         this->modspeed.setMax(1.f);
//         this->modspeed.setScale(0.01);
//         this->moddepth.setMax(0.03f);
//         this->moddepth.setScale(0.0003f);
//     }
//
//     String getName() override {return "boids";}
//
//     MetaOscType getType() const override { return MetaOscType::BOIDS; }    
//
//     inline float distBetween(float x1, float y1, float x2, float y2) {
//       float dx = x2-x1;
//       float dy = y2-y1;
//       return sqrtf((dx * dx) + (dy * dy));
//     }
//
//
//     std::array<float, N> update(const float (adcs)[4]) override {
//       centerX = 0;
//       centerY = 0;
//       for(auto &v: boids) {
//         centerX += v.x;
//         centerY += v.y;
//       }
//       centerX *= avgMul;
//       centerY *= avgMul;
//
//       constexpr float cohesionRadius = 45.0f;    // Rule 1
//       constexpr float separationRadius = 30.0f;  // Rule 2 (smaller)
//       constexpr float alignmentRadius = 40.0f;   // Rule 3
//
//       size_t idx=0;
//       float modvel = 0.1 + this->modspeed.getValue();
//       for(auto &v: boids) {
//
//         //rule 1 move towards center of mass
//         constexpr float cohesionStrength = 0.005f;
//         float dcxr1 = (centerX - v.x) * cohesionStrength;
//         float dcyr1 = (centerY - v.y) * cohesionStrength;
//
//
//         //rule 2 keep away from other boids
//         float dcxr2=0;
//         float dcyr2=0;
//         for(auto &otherBoid: boids) {
//           if (&v != &otherBoid) {
//             float dist = distBetween(v.x, v.y, otherBoid.x, otherBoid.y);
//             if (dist < separationRadius && dist > 0) {
//               float force = 1.0f / dist;  // Stronger when closer
//               dcxr2 += (v.x - otherBoid.x) * force;  // Removed minus sign
//               dcyr2 += (v.y - otherBoid.y) * force;  // Removed minus sign
//             }
//           }
//         }
//         dcxr2 *= 0.2f;
//         dcyr2 *= 0.2f;
//
//
//         //rule 3 match velocity with nearby boids
//         float dcxr3=0;
//         float dcyr3=0;
//
//         float avgVx = 0, avgVy = 0;
//         int neighborCount = 0;
//
//         constexpr float alignmentStrength = 0.05f;
//
//         for(auto &otherBoid: boids) {
//             if (&v != &otherBoid) {
//                 float dist = distBetween(v.x, v.y, otherBoid.x, otherBoid.y);
//                 if (dist < alignmentRadius) {
//                     avgVx += otherBoid.vx;
//                     avgVy += otherBoid.vy;
//                     neighborCount++;
//                 }
//             }
//         }
//
//         if (neighborCount > 0) {
//             avgVx /= neighborCount;
//             avgVy /= neighborCount;
//
//             dcxr3 = (avgVx - v.vx) * alignmentStrength; 
//             dcyr3 = (avgVy - v.vy) * alignmentStrength;
//
//         }     
//
//         v.vx += (dcxr1 + dcxr2 + dcxr3);
//         v.vy += (dcyr1 + dcyr2 + dcyr3);
//
//         float speed = sqrtf(v.vx * v.vx + v.vy * v.vy);
//         const float maxSpeed = 2.5f;
//         if (speed > maxSpeed) {
//             v.vx = (v.vx / speed) * maxSpeed;
//             v.vy = (v.vy / speed) * maxSpeed;
//         }        
//
//         v.x += v.vx * modvel;
//         v.y += v.vy * modvel;
//
//
//         //wrapping
//         if (v.x > sqboundBR) {
//           v.x-=sqwidth;
//           // v.x = -v.x;
//         }else if (v.x < sqbound) {
//           v.x += sqwidth;
//           // v.x = -v.x;
//         }
//
//         if (v.y > sqboundBR) {
//           v.y-=sqwidth;
//           // v.y = -v.y;
//         }else if (v.y < sqbound) {
//           v.y += sqwidth;
//           // v.y = -v.y;
//         }
//
//         mods[idx] = speed * this->moddepth.getValue();
//         idx++;
//      }
//       return mods;
//     }
//
//     std::array<float, N>& getValues() override {
//       return mods;
//     };
//
//     inline float constrainf(float val, float minVal, float maxVal) {
//       if (val < minVal) return minVal;
//       if (val > maxVal) return maxVal;
//       return val;
//     }
//
//     void draw(TFT_eSPI &tft) override { 
//       constexpr float lineLength = 4.f;
//       for(auto &v: boids) {
//         tft.drawCircle(v.px, v.py, 3, ELI_BLUE);
//         float tx = constrainf(v.px + (v.pvx * lineLength), sqbound, sqboundBR);
//         float ty = constrainf(v.py + (v.pvy * lineLength), sqbound, sqboundBR);
//         tft.drawLine(v.px, v.py,  tx, ty, ELI_BLUE);
//
//         tft.drawCircle(v.x, v.y, 3, TFT_GREENYELLOW);
//         tx = constrainf(v.x + (v.vx * lineLength), sqbound, sqboundBR);
//         ty = constrainf(v.y + (v.vy * lineLength), sqbound, sqboundBR);
//         tft.drawLine(v.x, v.y, tx, ty, TFT_YELLOW);
//         v.pvx = v.vx;
//         v.pvy = v.vy;
//         v.px = v.x;
//         v.py = v.y;
//       }
//     }
//
// private:
//   std::array<float, N> mods;
//   std::vector<boid> boids;
//   float centerX;
//   float centerY;
//   const float avgMul = 1.f/N;
//   float speedMul = 1.f;
//
// };

// ============================================================================
// LEGACY FIXED-POINT SYSTEM - DEPRECATED
// ============================================================================
// This typedef-based fixed-point system is retained only for backward
// compatibility with the old metaOscBoids implementation.
//
// NEW CODE SHOULD USE: Q16_16 from FixedPoint namespace (fixedpoint.hpp)
// See metaOscBoidsFP for the modern implementation with operator overloading.
// ============================================================================

// Q16.16 fixed-point (LEGACY - use Q16_16 for new code)
typedef int32_t fixed_t;
constexpr int FIXED_SHIFT = 16;
constexpr fixed_t FIXED_ONE = 1 << FIXED_SHIFT;

inline fixed_t float_to_fixed(float f) {
    return (fixed_t)(f * FIXED_ONE + (f >= 0 ? 0.5f : -0.5f));
}

inline float fixed_to_float(fixed_t f) {
    return (float)f / FIXED_ONE;
}

inline fixed_t fixed_mul(fixed_t a, fixed_t b) {
    return (fixed_t)(((int64_t)a * b) >> FIXED_SHIFT);
}

inline fixed_t fixed_div(fixed_t a, fixed_t b) {
    return (fixed_t)(((int64_t)a << FIXED_SHIFT) / b);
}

// Integer sqrt
inline uint32_t isqrt32(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n;
    uint32_t y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return x;
}

inline fixed_t fixed_sqrt(fixed_t x) {
    if (x <= 0) return 0;
    return isqrt32(x) << 8;
}

struct boid_fixed {
    fixed_t x, y;
    fixed_t px, py;
    fixed_t vx, vy;
    fixed_t pvx, pvy;
};

template<size_t N>
class metaOscBoids : public metaOsc<N> {
public:
    metaOscBoids() {
        boids = std::vector<boid_fixed>(N);
        
        for(auto &v: boids) {
            v.x = float_to_fixed(random(sqbound, sqboundBR));
            v.y = float_to_fixed(random(sqbound, sqboundBR));
            v.px = v.x;
            v.py = v.y;
            v.vx = float_to_fixed(((random(1000) / 1000.f) - 0.5f) * 0.8f);
            v.vy = float_to_fixed(((random(1000) / 1000.f) - 0.5f) * 0.8f);
            v.pvx = v.vx;
            v.pvy = v.vy;
        }
        
        centerX = float_to_fixed(120);
        centerY = float_to_fixed(120);
        
        this->modspeed.setMax(1.f);
        this->modspeed.setScale(0.01);
        this->moddepth.setMax(0.03f);
        this->moddepth.setScale(0.0003f);
    }

    String getName() override { return "boids"; }
    MetaOscType getType() const override { return MetaOscType::BOIDS; }

    inline fixed_t distBetween(fixed_t x1, fixed_t y1, fixed_t x2, fixed_t y2) {
        fixed_t dx = x2 - x1;
        fixed_t dy = y2 - y1;
        fixed_t dx2 = fixed_mul(dx, dx);
        fixed_t dy2 = fixed_mul(dy, dy);
        return fixed_sqrt(dx2 + dy2);
    }

    std::array<float, N> update(const size_t (adcs)[4]) override {
        // Calculate center of mass
        int64_t sumX = 0, sumY = 0;
        for(auto &v: boids) {
            sumX += v.x;
            sumY += v.y;
        }
        centerX = (fixed_t)(sumX / N);
        centerY = (fixed_t)(sumY / N);
        
        // Fixed-point constants
        const fixed_t cohesionRadius = float_to_fixed(45.0f);
        const fixed_t separationRadius = float_to_fixed(30.0f);
        const fixed_t alignmentRadius = float_to_fixed(40.0f);
        
        const fixed_t cohesionStrength = float_to_fixed(0.005f);
        const fixed_t separationMul = float_to_fixed(0.2f);
        const fixed_t alignmentStrength = float_to_fixed(0.05f);
        const fixed_t maxSpeed = float_to_fixed(2.5f);
        
        const fixed_t sqboundFixed = float_to_fixed(sqbound);
        const fixed_t sqboundBRFixed = float_to_fixed(sqboundBR);
        const fixed_t sqwidthFixed = float_to_fixed(sqwidth);

        size_t idx = 0;
        fixed_t modvel = float_to_fixed(0.1f + this->modspeed.getValue());
        
        // Temporary storage for neighbor info
        struct NeighborInfo {
            fixed_t dist;
            boid_fixed* ptr;
        };
        NeighborInfo neighbors[N];
        
        for(auto &v: boids) {
            // Cache all distances once
            int neighborCount = 0;
            for(auto &otherBoid: boids) {
                if (&v != &otherBoid) {
                    fixed_t dist = distBetween(v.x, v.y, otherBoid.x, otherBoid.y);
                    neighbors[neighborCount].dist = dist;
                    neighbors[neighborCount].ptr = &otherBoid;
                    neighborCount++;
                }
            }
            
            // Rule 1: Move towards center of mass
            fixed_t dcxr1 = fixed_mul(centerX - v.x, cohesionStrength);
            fixed_t dcyr1 = fixed_mul(centerY - v.y, cohesionStrength);
            
            // Rules 2 & 3: Process all neighbors in one loop
            fixed_t dcxr2 = 0, dcyr2 = 0;
            fixed_t dcxr3 = 0, dcyr3 = 0;
            int64_t avgVx = 0, avgVy = 0;
            int alignCount = 0;
            
            for(int i = 0; i < neighborCount; i++) {
                auto& info = neighbors[i];
                auto* other = info.ptr;
                
                // Rule 2: Separation
                if (info.dist < separationRadius && info.dist > 0) {
                    fixed_t force = fixed_div(FIXED_ONE, info.dist);
                    dcxr2 += fixed_mul(v.x - other->x, force);
                    dcyr2 += fixed_mul(v.y - other->y, force);
                }
                
                // Rule 3: Alignment
                if (info.dist < alignmentRadius) {
                    avgVx += other->vx;
                    avgVy += other->vy;
                    alignCount++;
                }
            }
            
            dcxr2 = fixed_mul(dcxr2, separationMul);
            dcyr2 = fixed_mul(dcyr2, separationMul);
            
            if (alignCount > 0) {
                avgVx /= alignCount;
                avgVy /= alignCount;
                dcxr3 = fixed_mul((fixed_t)avgVx - v.vx, alignmentStrength);
                dcyr3 = fixed_mul((fixed_t)avgVy - v.vy, alignmentStrength);
            }
            
            // Update velocity
            v.vx += (dcxr1 + dcxr2 + dcxr3);
            v.vy += (dcyr1 + dcyr2 + dcyr3);
            
            // Speed limiting
            fixed_t vx2 = fixed_mul(v.vx, v.vx);
            fixed_t vy2 = fixed_mul(v.vy, v.vy);
            fixed_t speed = fixed_sqrt(vx2 + vy2);
            
            if (speed > maxSpeed) {
                v.vx = fixed_mul(fixed_div(v.vx, speed), maxSpeed);
                v.vy = fixed_mul(fixed_div(v.vy, speed), maxSpeed);
                speed = maxSpeed;
            }
            
            // Update position
            v.x += fixed_mul(v.vx, modvel);
            v.y += fixed_mul(v.vy, modvel);
            
            // Wrapping
            if (v.x > sqboundBRFixed) {
                v.x -= sqwidthFixed;
            } else if (v.x < sqboundFixed) {
                v.x += sqwidthFixed;
            }
            
            if (v.y > sqboundBRFixed) {
                v.y -= sqwidthFixed;
            } else if (v.y < sqboundFixed) {
                v.y += sqwidthFixed;
            }
            
            mods[idx] = fixed_to_float(speed) * this->moddepth.getValue();
            idx++;
        }
        return mods;
    }

    std::array<float, N>& getValues() override {
        return mods;
    }

    inline int constraini(int val, int minVal, int maxVal) {
        if (val < minVal) return minVal;
        if (val > maxVal) return maxVal;
        return val;
    }

    void draw(TFT_eSPI &tft) override {
        constexpr int lineLength = 4;
        const fixed_t lineLengthFixed = lineLength << FIXED_SHIFT;
        const int sqboundInt = (int)sqbound;
        const int sqboundBRInt = (int)sqboundBR;
        
        for(auto &v: boids) {
            int px = v.px >> FIXED_SHIFT;
            int py = v.py >> FIXED_SHIFT;
            int x = v.x >> FIXED_SHIFT;
            int y = v.y >> FIXED_SHIFT;
            
            tft.drawCircle(px, py, 3, ELI_BLUE);
            int tx = constraini(px + (fixed_mul(v.pvx, lineLengthFixed) >> FIXED_SHIFT), sqboundInt, sqboundBRInt);
            int ty = constraini(py + (fixed_mul(v.pvy, lineLengthFixed) >> FIXED_SHIFT), sqboundInt, sqboundBRInt);
            tft.drawLine(px, py, tx, ty, ELI_BLUE);
            
            tft.drawCircle(x, y, 3, TFT_GREENYELLOW);
            tx = constraini(x + (fixed_mul(v.vx, lineLengthFixed) >> FIXED_SHIFT), sqboundInt, sqboundBRInt);
            ty = constraini(y + (fixed_mul(v.vy, lineLengthFixed) >> FIXED_SHIFT), sqboundInt, sqboundBRInt);
            tft.drawLine(x, y, tx, ty, TFT_YELLOW);
            
            v.pvx = v.vx;
            v.pvy = v.vy;
            v.px = v.x;
            v.py = v.y;
        }
    }

private:
    std::array<float, N> mods;
    std::vector<boid_fixed> boids;
    fixed_t centerX;
    fixed_t centerY;
};

template <size_t N>
using metaOscPtr = std::shared_ptr<metaOsc<N>>;

template <size_t N>
using metaOscFPPtr = std::shared_ptr<metaOscFP<N>>;

// ============================================================================
// COMPILE-TIME METAOSC VARIANT (eliminates heap allocation)
// ============================================================================

template<size_t N>
using MetaOscFPVariant = std::variant<
    metaOscNoneFP<N>,
    metaOscSinesFP<N>
>;

// Visitor helper for accessing variant members without type-switching
// Note: All FP oscillators use Q16_16 as their FixedType
template<size_t N>
struct MetaOscVisitor {
    static auto& getValues(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) -> auto& {
            return osc.getValues();
        }, var);
    }

    static std::array<Q16_16, N> update(MetaOscFPVariant<N>& var, const size_t adcs[4]) {
        return std::visit([&adcs](auto& osc) {
            return osc.update(adcs);
        }, var);
    }

    static void draw(MetaOscFPVariant<N>& var, const std::array<Q16_16, N>& vals) {
        std::visit([&vals](auto& osc) {
            osc.draw(vals);
        }, var);
    }

    static const char* getName(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) {
            return osc.getName();
        }, var);
    }

    static void setDepth(MetaOscFPVariant<N>& var, Q16_16 value) {
        std::visit([value](auto& osc) {
            osc.setDepth(value);
        }, var);
    }

    static Q16_16 getDepth(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) {
            return osc.getDepth();
        }, var);
    }

    static void setSpeed(MetaOscFPVariant<N>& var, Q16_16 value) {
        std::visit([value](auto& osc) {
            osc.setSpeed(value);
        }, var);
    }

    static Q16_16 getSpeed(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) {
            return osc.getSpeed();
        }, var);
    }

    static uint32_t getTimerMS(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) {
            return osc.getTimerMS();
        }, var);
    }

    static float getNormalisedDepth(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) {
            return osc.getNormalisedDepth();
        }, var);
    }

    static float getNormalisedSpeed(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) {
            return osc.getNormalisedSpeed();
        }, var);
    }

    static void restoreDepth(MetaOscFPVariant<N>& var, Q16_16 value) {
        std::visit([value](auto& osc) {
            osc.restoreDepth(value);
        }, var);
    }

    static void restoreSpeed(MetaOscFPVariant<N>& var, Q16_16 value) {
        std::visit([value](auto& osc) {
            osc.restoreSpeed(value);
        }, var);
    }

    // Direct access to moddepth/modspeed parameters
    static auto& getModDepth(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) -> auto& {
            return osc.moddepth;
        }, var);
    }

    static auto& getModSpeed(MetaOscFPVariant<N>& var) {
        return std::visit([](auto& osc) -> auto& {
            return osc.modspeed;
        }, var);
    }
};

// ============================================================================
// VARIANT STORAGE WRAPPER (deferred construction for RP2040 compatibility)
// ============================================================================
// Provides compile-time allocated storage for std::variant without triggering
// global constructor crashes on ARM Cortex-M0+. The variant is constructed
// in setup() after C++ runtime initialization completes.

template<size_t N>
class MetaOscFPVariantStorage {
private:
    using VariantType = MetaOscFPVariant<N>;
    alignas(VariantType) std::byte storage[sizeof(VariantType)];
    bool constructed = false;

public:
    // Trivial constructor - no variant construction at global init
    constexpr MetaOscFPVariantStorage() : storage{}, constructed(false) {}

    // Destructor - properly destroy variant if it was constructed
    ~MetaOscFPVariantStorage() {
        if (constructed) {
            get()->~VariantType();
        }
    }

    // Construct variant in-place using placement new
    template<typename T, typename... Args>
    void emplace(Args&&... args) {
        if (constructed) {
            get()->~VariantType();
        }
        new (storage) VariantType(std::in_place_type<T>, std::forward<Args>(args)...);
        constructed = true;
    }

    // Access the variant (assumes constructed - caller must check)
    VariantType* get() {
        return reinterpret_cast<VariantType*>(storage);
    }

    const VariantType* get() const {
        return reinterpret_cast<const VariantType*>(storage);
    }

    // Convenience operators
    VariantType& operator*() { return *get(); }
    const VariantType& operator*() const { return *get(); }
    VariantType* operator->() { return get(); }
    const VariantType* operator->() const { return get(); }

    // Check if variant has been constructed
    bool isConstructed() const { return constructed; }
};

