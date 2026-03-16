#ifndef OSCILLATORMODEL_HPP
#define OSCILLATORMODEL_HPP

#include "../clockfreq.h"
#include "fixedpoint.hpp"

using namespace FixedPoint;

#define fadeBitResolution 7 //  bits for fade level
#define fadeMaxLevel 127

static constexpr size_t FADE_TABLE_SIZE = 1 << fadeBitResolution;
int32_t fadeInvTable[FADE_TABLE_SIZE];  
bool fadeTableInitialized = false;


class oscillatorModel {
public:
  oscillatorModel() {
    newFreq = false;
    updateBufferInSyncWithDMA = false;
    if (!fadeTableInitialized) {
      for (size_t i = 0; i < FADE_TABLE_SIZE; i++) {
          fadeInvTable[i] = (fadeMaxLevel << 16) / max(i, (size_t)1);
      }      
      fadeTableInitialized = true;
    }
  };
  
  pio_program prog;
  size_t loopLength;


  virtual ~oscillatorModel() = default;  
  uint loadProg(PIO pioUnit) {
    return pio_add_program_at_offset(pioUnit, &prog, 0);
  }

  void unloadProg(PIO pioUnit) {
    pio_remove_program(pioUnit, &prog, 0);
  }

  // oscDisplayModes vis;

  volatile bool newFreq;
  bool updateBufferInSyncWithDMA; //if true, update buffer every time one is consumed by DMA
  size_t clockModShift = 0;

  virtual void fillBuffer(uint32_t* bufferA)=0;

  virtual void ctrl(const Q16_16 v) {
    //receive a control parameter
  }

  virtual pio_sm_config getBaseConfig(uint offset) {
    return pin_ctrl_program_get_default_config(offset);
  }

  virtual void reset() {
  }

  void setClockModShift(const size_t shift) {
    clockModShift = shift;
  }

  size_t getClockDiv() const {
    return clockdiv << clockModShift;
  }

  inline size_t getWavelen() const {
    return wavelen;
  }

  inline void setWavelen(const size_t wlen) {
    if (wlen >=  minWavelen) {
      wavelen = wlen >> clockModShift;
      newFreq = true;
    }
  }

  size_t getWavelenAtFrequency(const float freq) {
    //calculate wavelen for a given frequency
    //wavelen = clockdiv / freq;
    const size_t w = static_cast<size_t>(sampleClock / freq ) >> clockModShift;
    return w;
  }

  size_t getMinWavelen() const {
    return static_cast<size_t>(sampleClock /20000.f) >> clockModShift;
  }

  virtual String getIdentifier() {
    return "Base";
  }

  int fadeDirection=0;


  void startFadeIn() {
    fadeDirection=1;
    fadeLevel=0;
  }

  void startFadeOut() {
    fadeDirection=-1;
    fadeLevel=fadeMaxLevel;
  }

  bool isFadingOut() const {
    return fadeDirection == -1;
  }

  __force_inline void updateFade() {
        fadeLevel += fadeDirection; // Apply fade in/out
        if (fadeLevel == 0) { 
          fadeDirection = 0; // Stop at fully faded out
        } else if (fadeLevel == fadeMaxLevel) {
            fadeDirection = 0; // Stop at fully faded in
        }
  }

protected:
  size_t wavelen=100000;
  int32_t fadeLevel=fadeMaxLevel;


};

#endif // OSCILLATORMODEL_HPP