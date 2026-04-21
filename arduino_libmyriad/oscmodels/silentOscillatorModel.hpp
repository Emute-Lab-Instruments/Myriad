#pragma once
#include "oscillatorModel.hpp"

class silentOscillatorModel : public oscillatorModel {
public:
  silentOscillatorModel() : oscillatorModel() {
    loopLength=bufferSize;
    prog=bitbybit_program;
    updateBufferInSyncWithDMA = true;
  }
  inline void fillBuffer(uint32_t* bufferA) {
    for (size_t i = 0; i < bufferSize; ++i) {
        *(bufferA + i) = 0xAAAAAAAA;
    }
    updateFade();
  }

  String getIdentifier() override {
    return "sil";
  }

private:
  const size_t bufferSize=16;
  // static constexpr uint32_t halfWaveLen = sampleClock / 100000;
};
