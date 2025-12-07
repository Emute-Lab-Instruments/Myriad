#ifndef OSCILLATORMODEL_HPP
#define OSCILLATORMODEL_HPP

#include "../clockfreq.h"

class oscillatorModel {
public:
  oscillatorModel() {
    newFreq = false;
    updateBufferInSyncWithDMA = false;
  };
  
  pio_program prog;
  virtual void fillBuffer(uint32_t* bufferA)=0;
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
  // float clockmodinv = 1.f;
  size_t clockModShift = 0;

  virtual void ctrl(const float v) {
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

protected:
  size_t wavelen=100000;
};

#endif // OSCILLATORMODEL_HPP