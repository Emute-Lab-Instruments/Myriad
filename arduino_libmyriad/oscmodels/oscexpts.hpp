#ifndef OSCEXPTS_HPP
#define OSCEXPTS_HPP

#if 0

class slideOscillatorModel : public virtual oscillatorModel {
  public:
  slideOscillatorModel() : oscillatorModel(){
      loopLength=6;
      prog=pulse_program;
      oscInterpTemplate.resize(loopLength,1.0f/loopLength);
      smoothTemplate.resize(loopLength,1.0f/loopLength);
      smoothingThreshold = getWavelenAtFrequency(450.f);
      smoothingThresholdInv = 1.f / smoothingThreshold;
      ctrl(0);
      // updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA

    }
    inline void fillBuffer(uint32_t* bufferA) {
      for (size_t i = 0; i < oscInterpTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(smoothTemplate[i] * this->wavelen);
      }
    }

    void ctrl(const float v) override {

      float scaledV = v*0.999f * (oscTemplate.size() - loopLength);
      offset = static_cast<size_t>(scaledV);
      size_t offsetNext = offset+1;
      float remainder = scaledV - offset; 
      float remainderInv = 1.f - remainder;
      float total=0.f;
      //interpolate from an offset
      for(size_t i=0; i < loopLength; i++) {
        float val = (remainderInv * oscTemplate[i+offset]) + (remainder * oscTemplate[i+offsetNext]);
        oscInterpTemplate[i] = val;
        total += val;
      }


      //normalise so that the sum of the values is 1
      if (wavelen < smoothingThreshold) {
        // smoothTemplateAlpha = 0.05f; // smoothing factor for the control value
        smoothTemplateAlpha = 0.03f - (0.02f * (1.f - ((float)wavelen*smoothingThresholdInv))); // smoothing factor for the control value
      } else {
        smoothTemplateAlpha = 0.1f; // smoothing factor for the control value
      };
      smoothTemplateAlphaInv = 1.f - smoothTemplateAlpha;
      // Serial.printf("smoothTemplateAlpha: %f, smoothTemplateAlphaInv: %f\n", smoothTemplateAlpha, smoothTemplateAlphaInv);

      float scale = 1.f/total;
      for(size_t i=0; i < loopLength; i++) {
        oscInterpTemplate[i] *= scale;

        //smooth the template transition
        smoothTemplate[i] = (smoothTemplate[i] * smoothTemplateAlphaInv) + (oscInterpTemplate[i] * smoothTemplateAlpha);
        // Serial.printf("%f ", oscInterpTemplate[i]);
      }
      // Serial.println();
      // Serial.printf("%f %f %f\n", oscInterpTemplate[0], oscTemplate[offset], oscTemplate[offsetNext]);

    }

    pio_sm_config getBaseConfig(uint offset) override {
      return pulse_program_get_default_config(offset);
    }
  
    const std::vector<float> oscTemplate {0.1, 0.3, 0.01, 0.2, 0.03,
      0.02, 0.01, 0.04, 0.057, 0.034,
     0.002, 0.03, 0.001, 0.2, 0.023,
    0.4,0.36,0.7, 0.072, 0.0808}; 

    String getIdentifier() override {
      return "slide";
    }
  
  private:
  size_t offset=0;
  std::vector<float> scales;
  std::vector<float> oscInterpTemplate;

  float smoothTemplateAlpha = 0.05f; // smoothing factor for the control value
  float smoothTemplateAlphaInv = 1.f - smoothTemplateAlpha;
  std::vector<float> smoothTemplate;
  size_t smoothingThreshold=0;
  size_t smoothingThresholdInv=1;

  };


//this is bonkers
class accOscillatorModel : public virtual oscillatorModel {
public:
  accOscillatorModel() : oscillatorModel(){
    loopLength=8;
    prog=pulse_program;

  }
  inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < loopLength; ++i) {
      if (w <= 0) {
        acc += this->wavelen;
        w = acc >> 5;
      }
      int val;
      val = w;
      acc -= w;

      w=w - (w>>4) - 1;
      w=std::max(w,acc>>2);
      *(bufferA + i) = static_cast<uint32_t>(val);

    }
  }
  pio_sm_config getBaseConfig(uint offset) {
    return pulse_program_get_default_config(offset);
  }
  void ctrl(const float v) override {
  }

  String getIdentifier() override {
    return "acc";
  }

private:
  float phase=0;
  int acc=0;
  int w=1;
  int state=1;

};


class sinOscillatorModel8 : public oscillatorModel {
public:
  sinOscillatorModel8() : oscillatorModel() {
    loopLength=8;
    prog=pulse_program;
    tempOscTemplate.resize(loopLength,1.0/loopLength);
    oscTemplate.resize(loopLength,1.0/loopLength);
    ctrl(0.0);

  }
  inline void fillBuffer(uint32_t* bufferA) {
    const float wlen = wavelen * 2.55f;
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wlen);
    }
  }

  void ctrl(const float v) override {
    float vscale = ((v * 0.01) + 0.65) * 8;
    float total=0;
    for(float i=0; i < loopLength; i++) {
      const float w = std::max(0.0001f, fabs(sinf(i * vscale)));
      tempOscTemplate[i] = w;
      total += w;
    }

    //normalise
    const float lenscale = total/4.69f;
    const float scale=1.f/total;
    for(size_t i=0; i < loopLength; i++) {
      oscTemplate[i] = tempOscTemplate[i] * lenscale;
      // Serial.printf("%f\t",oscTemplate[i]);
    }
    Serial.println(lenscale);
  }

  std::vector<float> tempOscTemplate;
  std::vector<float> oscTemplate;

  pio_sm_config getBaseConfig(uint offset) {
    return pulse_program_get_default_config(offset);
  }

  String getIdentifier() override {
    return "sin8";
  }

};

class expdecOscillatorModel1 : public virtual oscillatorModel {
  public:
  expdecOscillatorModel1() : oscillatorModel(){
      loopLength=8;
      prog=expdec_program;
    }
    inline void fillBuffer(uint32_t* bufferA) {
      const float wlen = this->wavelen * 3.57f;
      for (size_t i = 0; i < oscTemplate.size(); ++i) {
          *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wlen);
      }
    }
    std::vector<float> oscTemplate {0.05,0.025,0.01,0.001,0.001,0.01,0.025,0.05};
  
    void ctrl(const float v) override {
      const float mul = 0.25f;
      oscTemplate[0] = 0.05 + (v * 0.02 * mul);
      oscTemplate[1] = 0.025 - (v * 0.02 * mul);

      oscTemplate[4] = 0.001 - (v * 0.0005 * mul);
      oscTemplate[5] = 0.001 + (v * 0.0005 * mul);

      oscTemplate[5] = 0.01 - (v * 0.05 * mul);
      oscTemplate[6] = 0.025 + (v * 0.05 * mul);
    }

    pio_sm_config getBaseConfig(uint offset) {
      return expdec_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "exp1";
    }
  
  private:
  
  };

class expdecOscillatorBytebeatModel : public virtual oscillatorModel {
    public:
    expdecOscillatorBytebeatModel() : oscillatorModel(){
        loopLength=8;
        prog=expdec_program;
        updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
        
    }
    inline void fillBuffer(uint32_t* bufferA) {
      const float wlen = this->wavelen * 7.19f * wmult ;
      for (size_t i = 0; i < oscTemplate.size(); ++i) {
          *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wlen);
      }
      wmult = wmult * wmultmult;
      if (wmult < 0.125f) {
        wmult=1.f;
      }
    }
    std::vector<float> oscTemplate {0.05,0.025,0.01,0.001,0.001,0.01,0.025,0.05};
  
    void ctrl(const float v) override {
      wmultmult = 0.01f + (v*0.5f);
    }

    pio_sm_config getBaseConfig(uint offset) {
      return expdec_program_get_default_config(offset);
    }

    String getIdentifier() override {
      return "expb";
    }
  
  private:
    float wmult=1;
    float wmultmult=1.f;
  
};


class pulse10OscillatorModel : public virtual oscillatorModel {
public:
  pulse10OscillatorModel() : oscillatorModel(){
    loopLength=4;
    prog=pin_ctrl_program;
    // updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA

  }
  inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * this->wavelen * 0.5f);
    }
  }
  std::vector<float> oscTemplate {0.1,0.9,0.1,0.9};

  void ctrl(const float v) override {
    //receive a control parameter
    const float v1 = v * 0.98;
    const float v2 = 1.0 - v;
    oscTemplate [0] = v1;
    oscTemplate [1] = v2;
    oscTemplate [2] = v1;
    oscTemplate [3] = v2;
  }

  pio_sm_config getBaseConfig(uint offset) override {
    return pin_ctrl_program_get_default_config(offset);
  }

  void reset() override {
    oscillatorModel::reset();
    // Clear any internal state if needed
  }
  String getIdentifier() override {
    return "p100";
  }

};

class squareOscillatorModel : public virtual oscillatorModel {
public:
  squareOscillatorModel() : oscillatorModel(){
    loopLength=2;
    prog=pulse_program;
  }
  inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * this->wavelen);
    }
  }
  pio_sm_config getBaseConfig(uint offset) override {
    return pulse_program_get_default_config(offset);
  }

  std::vector<float> oscTemplate {0.5,0.5}; 
  void ctrl(const float v) override {
    //receive a control parameter
    const float v1 = std::max((1.f-v) * 0.5f, 0.01f);
    const float v2 = 1.0 - v;
    oscTemplate [0] = v1;
    oscTemplate [1] = v2;
  }


  String getIdentifier() override {
    return "sq";
  }

};

class squareOscillatorModel2 : public oscillatorModel {
public:
  squareOscillatorModel2() : oscillatorModel() {
    loopLength=4;
    prog=pin_ctrl_program;
    updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA

  }
  inline void fillBuffer(uint32_t* bufferA) {
    const float wlen = this->wavelen * 2.5f;
    // for (size_t i = 0; i < oscTemplate.size(); ++i) {
    //     *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wlen);
    // }
    for (size_t i = 0; i < loopLength; ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[bufferIndex] * wlen);
        bufferIndex++;
        if (bufferIndex >= oscTemplate.size()) bufferIndex=0;
    }
  }

  void ctrl(const float v) override {
    constexpr float bound = 0.001;
    constexpr float range = 0.2 - (bound * 2);
    float spread = (v * range) * 0.2;
    float accSpread=bound;

    oscTemplate [0] = accSpread;
    oscTemplate [1] = 0.2-accSpread;
    accSpread += spread;

    oscTemplate [2] = accSpread;
    oscTemplate [3] = 0.2 - accSpread;
    accSpread += spread;

    oscTemplate [4] =accSpread;
    oscTemplate [5] = 0.2 - accSpread;
    accSpread += spread;

    oscTemplate [6] = accSpread;
    oscTemplate [7] = 0.2 - accSpread;
    accSpread += spread;

    oscTemplate [8] = accSpread;
    oscTemplate [9] = 0.2 - accSpread;

  }

  void reset() override {
    oscillatorModel::reset();
    bufferIndex=0;
  }


  //ratios 9:1,8:2,7:3,6:4,5:5
  std::array<float, 10> oscTemplate {0.18181818181818, 0.018181818181818, 0.16363636363636, 0.036363636363636, 0.14545454545455, 0.054545454545455, 0.12727272727273, 0.072727272727273, 0.10909090909091, 0.090909090909091};

  size_t bufferIndex=0;

  String getIdentifier() override {
    return "sq2";
  }

};


class squareOscillatorModel14 : public oscillatorModel {
public:
  squareOscillatorModel14() : oscillatorModel() {
    loopLength=4;
    prog=pin_ctrl_program;
    updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
  }
  
  inline void fillBuffer(uint32_t* bufferA) {
    const float wlen = this->wavelen * 1.75f * 2.0f;
    // for (size_t i = 0; i < oscTemplate.size(); ++i) {
    //     *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wlen);
    // }
    for (size_t i = 0; i < loopLength; ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[bufferIndex] * wlen);
        bufferIndex++;
        if (bufferIndex >= oscTemplate.size()) bufferIndex=0;
    }
  }

  void ctrl(const float v) override {
    constexpr float bound = 0.001;
    constexpr float range = oneOver7 - (bound * 2);
    float spread = (v * range) * oneOver7;
    float accSpread=bound;

    oscTemplate [0] = accSpread;
    oscTemplate [1] = oneOver7-accSpread;
    accSpread += spread;

    oscTemplate [2] = accSpread;
    oscTemplate [3] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [4] =accSpread;
    oscTemplate [5] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [6] = accSpread;
    oscTemplate [7] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [8] = accSpread;
    oscTemplate [9] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [10] = accSpread;
    oscTemplate [11] = oneOver7 - accSpread;
    accSpread += spread;

    oscTemplate [12] = accSpread;
    oscTemplate [13] = oneOver7 - accSpread;

  }

  std::array<float, 14> oscTemplate {oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,
  oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14,oneOver14};

  size_t bufferIndex=0;

  void reset() override {
    oscillatorModel::reset();
    bufferIndex=0;
  }


  String getIdentifier() override {
    return "sq14";
  }
  
private:
  static constexpr float oneOver14 = 1.f/14.f;
  static constexpr float oneOver7 = 1.f/7.f;
};


//this sounds interesting in tonal variation of the mask but also clicks at regular points
class triErrMaskOscillatorModel : public virtual oscillatorModel {
  public:

    triErrMaskOscillatorModel() : oscillatorModel(){
      loopLength=64;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
      const size_t midphase = wavelen >> 1;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wavelen ? 0 : phase; // wrap around

        
          size_t amp = phase > midphase ? phase >> 1 : phase << 1; //tri wave

          const bool y = amp >= err0 ? 1 : 0;
          err0 = (y ? wavelen : 0) - amp + err0;
          err0 = (err0) & mul;


          word |= (y << bit);

          phase++;
        }
        *(bufferA + i) = word;

      }

    }

    inline int find_msb(uint32_t x) {
      if (x == 0) return -1;
      int msb = 0;
      if (x >= 1 << 16) { msb += 16; x >>= 16; }
      if (x >= 1 << 8)  { msb += 8;  x >>= 8;  }
      if (x >= 1 << 4)  { msb += 4;  x >>= 4;  }
      if (x >= 1 << 2)  { msb += 2;  x >>= 2;  }
      if (x >= 1 << 1)  { msb += 1; }
      return msb;
    }

    void ctrl(const float v) override {
      constexpr size_t mask = std::numeric_limits<size_t>::max();
      // constexpr size_t mask2 = 0b10010010010010010010010010010010; //std::numeric_limits<size_t>::max();
      int msb = find_msb(wavelen);
      size_t shift = (32-msb) + static_cast<size_t>((msb-8) * v);
      if (shift < 14) {
        shift = 14;
      }
      mul = (mask >> shift) ;
      // mul = mul | wavelen;
      // aug = static_cast<size_t>(wavelen * v * 0.2f);
    }
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    void reset() override {
      oscillatorModel::reset();
      phase = 0;
      y = 0;
      err0 = 0;
      mul = 1;
      aug = 1;
    }

  
  private:
    size_t phase=0;
    bool y=0;
    int err0=0;

    size_t mul=1;
    size_t aug=1;


  };


  //this one creates aliasing
class triSDFBVar1OscillatorModel : public virtual oscillatorModel {
  public:

    triSDFBVar1OscillatorModel() : oscillatorModel(){
      loopLength=64;
      prog=bitbybit_program;
      setClockModShift(1);
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      const int midphase = wlen >> 1;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wlen ? 0 : phase; // wrap around

          int amp=phase;
          // int amp = phase < midphase ? 
          //       phase << 1 
          //       : 
          //       wlen - ((phase-midphase) << 1); //tri wave
          // size_t amp = phase; // saw
          // int amp = phase < midphase  ? 0 : wavelen; // pulse

          const bool y = amp >= err0 ? 1 : 0;
          err0 = (y ? wlen   : 0) - amp + err0 -  err1 ;
          // err0 = (err0) & mul;
          // err0 = (err0 * mul) >> qfp;
          err1 = err1 + ((err0 * mul) >> qfp);

          word |= (y << bit);

          phase++;
        }
        *(bufferA + i) = word;

      }

    }


    void ctrl(const float v) override {
      mul = ((v * 0.9)) * qfpMul;
      Serial.println(mul);
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    void reset() override {
      oscillatorModel::reset();
      phase = 0;
      y = 0;
      err0 = 0;
      mul = 1;
      err1=0;
    }

  
  private:
    int phase=0;
    bool y=0;
    int err0=0;
    int err1=0;
    int mul=1;

    static constexpr int qfp = 15U;
    static constexpr float qfpMul = 1U << qfp;

};


//bit by bit square wave oscillator
//better latency at lower frequencies than the timing buffer model
//but hf gets load of aliasing - need special case if wavelen high
class squareBBBOscillatorModel : public virtual oscillatorModel {
  public:
    squareBBBOscillatorModel() : oscillatorModel(){
      loopLength=32;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      const size_t halfwavelen = wlen / 2;
      for (size_t i = 0; i < loopLength; ++i) {
        if (phase >= halfwavelen) {
            val = 4294967295U;
        }else{
          val = 0;
        }
        *(bufferA + i) = val;
        phase+=32;
        if (phase >= wlen) {
          phase  -=wlen; // wrap around
        }
      }
    }

    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

  
    String getIdentifier() override {
      return "sqbbb";
    }
  private:
    size_t phase=0;
    bool y=0;
    int err0;
    size_t val=0;

  };

//sharktooth
//rate limiting causes some zipper noise
class rateLimSDOscillatorModel : public virtual oscillatorModel {
  public:

    rateLimSDOscillatorModel() : oscillatorModel(){
      loopLength=16;
      prog=bitbybit_program;
      updateBufferInSyncWithDMA = true; //update buffer every time one is consumed by DMA
    }

    inline void fillBuffer(uint32_t* bufferA) {
      const size_t wlen = this->wavelen;
      for (size_t i = 0; i < loopLength; ++i) {
        size_t word=0U;
        for(size_t bit=0U; bit < 32U; bit++) {
          phase = phase >= wlen ? 0 : phase; // wrap around

          size_t amp = phase << 1;
          amp =  amp > wlen ? phase : amp; 


          const bool y = amp >= err0 ? 1 : 0;
          size_t newerr = (y ? wlen : 0) - amp + err0;
          
          if ((newerr)> lim) {
              newerr = lim;
          }
          err0=newerr;

          word |= (y << bit);

          phase++;
        }
        *(bufferA + i) = word;

      }

    }


    void ctrl(const Q16_16 v) override {
      // lim = this->wavelen * (0.05f + (v * 0.95f));
      
    }
      
  
    pio_sm_config getBaseConfig(uint offset) {
      return bitbybit_program_get_default_config(offset);
    }

    void reset() override {
      oscillatorModel::reset();
      phase = 0;
      y = 0;
      err0 = 0;
      mul = 1;
      lim=this->wavelen;
    }

    String getIdentifier() override {
      return "sdr10";
    }

  
  private:
    size_t phase=0;
    bool y=0;
    int err0=0;

    size_t mul=1;
    size_t lim;

    static constexpr size_t qfp = 14U;
    static constexpr float qfpMul = 1U << qfp;

};

#endif //if 0

#endif // OSCEXPTS_HPP