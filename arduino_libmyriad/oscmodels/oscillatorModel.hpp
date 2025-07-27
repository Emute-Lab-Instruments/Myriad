#ifndef OSCILLATORMODEL_HPP
#define OSCILLATORMODEL_HPP

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
    return pio_add_program(pioUnit, &prog);
  }

  oscDisplayModes vis;

  volatile bool newFreq;
  bool updateBufferInSyncWithDMA; //if true, update buffer every time one is consumed by DMA
  float clockmod = 1.f;
  float clockmodinv = 1.f;

  virtual void ctrl(const float v) {
    //receive a control parameter
  }

  virtual pio_sm_config getBaseConfig(uint offset) {
    return pin_ctrl_program_get_default_config(offset);
  }

  virtual void reset() {
  }

  void setClockMod(const float mod) {
    clockmod = mod;
    clockmodinv = 1.f / clockmod;
  }

  size_t getClockDiv() const {
    return static_cast<size_t>(clockdiv * clockmod);
  }

  inline size_t getWavelen() const {
    return wavelen;
  }

  inline void setWavelen(const size_t wlen) {
    wavelen = wlen * clockmodinv;
    newFreq = true;
  }

protected:
  size_t wavelen=100000;
};

#endif // OSCILLATORMODEL_HPP