#ifndef OSCEXPTS_HPP
#define OSCEXPTS_HPP

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


#endif // OSCEXPTS_HPP