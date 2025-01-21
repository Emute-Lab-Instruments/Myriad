#include <cmath>
#include <TFT_eSPI.h>
#include "drawing.h"
#include "oscVisData.hpp"
#include <array>
#include "metaOscs.hpp"
#include "oscDisplayModes.hpp"
#include "clockfreq.h"
#include "sinetable.h"

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library


template<size_t N_OSCS, size_t N_OSC_BANKS, size_t N_OSCILLATOR_MODELS>
class displayPortal {
public:
  enum SCREENMODES {OSCBANKS, METAOSCVIS};

  std::vector<oscVisData*> oscVisDataPtrs;

  std::vector<oscDisplayModes*> oscvis;

  struct OscBankScreenStates {
    size_t oscModel[3];
    std::array<float, N_OSCS> oscWavelengths;
  };

  struct MetaOscVisScreenStates {
    int metaOsc;
    float moddepth;
    float modspeed;
    metaOscPtr<N_OSCS> ptr;
  };

  struct displayStates {
    SCREENMODES screenMode;
    OscBankScreenStates oscBankScreenState;
    MetaOscVisScreenStates metaOscVisScreenState;
    bool redraw=false; //override and redraw anyway
  };

  displayPortal() {
    nextState.redraw = true;
  }

  void setDisplayWavelengths(const std::array<float, N_OSCS> &wavelengths) {
    nextState.oscBankScreenState.oscWavelengths = wavelengths;
  }

  void update() {
    // Serial.println("update");
    bool redraw = nextState.redraw;
    if (!redraw) {
      if(nextState.screenMode != currState.screenMode) {
        redraw = true;
      }
    }else{
      nextState.redraw = false;
    }
    if (nextState.screenMode == SCREENMODES::OSCBANKS) {
      drawOscBankScreen(currState.oscBankScreenState, nextState.oscBankScreenState, redraw);
    }else{
      drawMetaModScreen(currState.metaOscVisScreenState, nextState.metaOscVisScreenState, redraw);
    }
    currState = nextState;
  }


  void toggleScreen() {
    if (currState.screenMode == SCREENMODES::OSCBANKS) {
      nextState.screenMode = SCREENMODES::METAOSCVIS;
    }else{
      nextState.screenMode = SCREENMODES::OSCBANKS;
    }
  }

  void setScreen(SCREENMODES newMode) {
    nextState.screenMode = newMode;
  }

  void setOscBankModel(size_t newBank, size_t newOscModel) {
    nextState.oscBankScreenState.oscModel[newBank] = newOscModel;
  }

  void setMetaOsc(size_t newIdx, metaOscPtr<N_OSCS> newPtr) {
    nextState.metaOscVisScreenState.metaOsc = newIdx;
    nextState.metaOscVisScreenState.ptr = newPtr; 
    nextState.metaOscVisScreenState.moddepth = newPtr->moddepth.getNormalisedValue();
    nextState.metaOscVisScreenState.modspeed = newPtr->modspeed.getNormalisedValue();
    nextState.redraw = true;   
  }

  void setMetaModDepth(float newDepth) {
    nextState.metaOscVisScreenState.moddepth = newDepth;
  }

  void setMetaModSpeed(float newSpeed) {
    nextState.metaOscVisScreenState.modspeed = newSpeed;
  }

private:
  void updateOscVis(size_t oscIdx, size_t oscModelIdx) {
    uint32_t startAngle = 120*oscIdx;
    uint32_t endAngle = 120 + (120*oscIdx);
    tft.drawSmoothArc(120, 120, 120, 0, startAngle, endAngle, ELI_BLUE, ELI_BLUE);
    uint32_t col1;
    const uint32_t cols[3] = {TFT_PURPLE, TFT_DARKGREEN, TFT_DARKCYAN};
    col1 = cols[oscIdx];
    // for(size_t i=0; i < oscVisDataPtrs.at(oscModelIdx)->spec.size(); i++) {
    //   if (oscVisDataPtrs.at(oscModelIdx)->spec.at(i)> 0) {    
    //     tft.drawSmoothArc(120, 120, 120-i, 120-i-1, startAngle, endAngle, col1, col1);
    //   }
    // }
    switch(oscvis.at(oscModelIdx)->mode) {
      case oscDisplayModes::MODES::SPECTRAL:
      {
        const size_t blocksize=5;
        for(size_t i=0; i < oscvis.at(oscModelIdx)->data.size(); i++) {
          if (oscvis.at(oscModelIdx)->data.at(i)> 0) {    
            tft.drawSmoothArc(120, 120, 115-(i*blocksize), 115-((i-1)*blocksize), startAngle, endAngle, col1, col1);
          }
        }
        break;
      }
      case oscDisplayModes::MODES::NOISE:
      {
        break;
      }
      case oscDisplayModes::MODES::SILENCE:
      {
        break;
      }
    }
    const int32_t bankTxtX[3] = {120-87, 120,120+87};
    const int32_t bankTxtY[3] = {120+49,20,120+49};
    tft.setFreeFont(&FreeMono9pt7b);
    tft.setTextDatum(CC_DATUM);
    tft.drawString(String(oscModelIdx), bankTxtX[oscIdx], bankTxtY[oscIdx]);

  }

  std::array<float,N_OSCS> oscVisPhase{};

  void drawOscBankScreen(const OscBankScreenStates &currState, const OscBankScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
    }
    
    // for(size_t i=0; i < N_OSCS; i++) {
    //   size_t pos = oscVisPhase[i] * 120;
    //   tft.drawArc(120, 120, pos, pos-2, (i*40),(i*40) + 40, ELI_BLUE, ELI_BLUE);
    //   float normwavelen = 1.0 - ((nextState.oscWavelengths[i] - fastestWavelen) * rangeWavelenRcp);
    //   normwavelen *= normwavelen;
    //   const float speed = (rangeOscVisSpeed * normwavelen) + slowestOscVisSpeed;
    //   Serial.println(speed); 
    //   oscVisPhase[i] += speed;
    //   if (oscVisPhase[i] >= 1.f) {
    //     oscVisPhase[i] = 0.f;
    //   }
    //   pos = oscVisPhase[i] * 120;
    //   tft.drawArc(120, 120, pos, pos-, (i*40),(i*40) + 40, ELI_PINK, ELI_PINK);
    // }  

    static constexpr float slowestWavelen = sampleClock/20.f;
    static constexpr float fastestWavelen = sampleClock/20000.f;
    static constexpr float rangeWavelen = slowestWavelen - fastestWavelen;
    static constexpr float rangeWavelenRcp = 1.0/rangeWavelen;
    static constexpr float slowestOscVisSpeed = 0.001;
    static constexpr float fastestOscVisSpeed = 0.03;
    static constexpr float rangeOscVisSpeed = fastestOscVisSpeed - slowestOscVisSpeed;
    static constexpr float unitR=9;
    for(size_t i=0; i < N_OSCS; i++) {
      float pos = oscVisPhase[i] * TWOPI;
      // tft.drawLine(120,120, 120+ (100 * cos(pos)), 120+ (100 * sin(pos)), ELI_BLUE );
      const size_t cx = 120+ (((i+1)*unitR) * sineTable::fast_cos(pos));
      const size_t cy = 120+ (((i+1)*unitR) * sineTable::fast_sin(pos));
      tft.drawLine(120,120, cx,cy, ELI_BLUE );
      tft.drawCircle(cx, cy, 5, ELI_BLUE);
      float normwavelen = 1.0 - ((nextState.oscWavelengths[i] - fastestWavelen) * rangeWavelenRcp);
      normwavelen= normwavelen * normwavelen * normwavelen;
      // normwavelen = std::sqrt(normwavelen);
      const float speed = (rangeOscVisSpeed * normwavelen) + slowestOscVisSpeed;
      if (i==0) {
        Serial.println(normwavelen);
      }
      oscVisPhase[i] += speed;
      if (oscVisPhase[i] >= 1.f) {
        oscVisPhase[i] = 0.f;
      }
      pos = oscVisPhase[i] * TWOPI;
      // tft.drawLine(120,120, 120+ (100 * cos(pos)), 120+ (100 * sin(pos)), ELI_PINK );
      // tft.drawCircle(120+ (((i+1)*10) * cos(pos)), 120+ (((i+1)*10) * sin(pos)), 5, ELI_PINK);
      const size_t cx2 = 120+ (((i+1)*unitR) * sineTable::fast_cos(pos));
      const size_t cy2 = 120+ (((i+1)*unitR) * sineTable::fast_sin(pos));
      tft.drawLine(120,120, cx2,cy2, ELI_PINK );
      tft.drawCircle(cx2, cy2, 5, ELI_PINK);

    }    
    constexpr float angleRange = 120/ N_OSCILLATOR_MODELS;
    constexpr std::array<size_t, 10> = {TFT_RED, TFT_GREEN, TFT_PURPLE, TFT_CYAN, TFT_MAGENTA, TFT_YELLOW,TFT_ORANGE, TFT_GOLD,  TFT_GREENYELLOW,TFT_BLUE };

    for(size_t i=0; i < 3; i++) {
      if (fullRedraw || currState.oscModel[i] != nextState.oscModel[i]) {
    //     updateOscVis(i, nextState.oscModel[i]);
        static const int32_t bankTxtX[3] = {120-87, 120,120+87};
        static const int32_t bankTxtY[3] = {120+49,20,120+49};
        tft.setFreeFont(&FreeMono9pt7b);
        tft.setTextDatum(CC_DATUM);
        tft.drawRect(bankTxtX[i]-5, bankTxtY[i]-5, 10, 10, ELI_BLUE);
        tft.drawString(String(nextState.oscModel[i]), bankTxtX[i], bankTxtY[i]);
        tft.drawArc(120,120,120,115,(i*120), (i+1) * 120, ELI_BLUE,  ELI_BLUE);
        tft.drawArc(120,120,120,115,(i*120) + (nextState.oscModel[i] * angleRange), (i* 120) + ((nextState.oscModel[i]+1) * angleRange), ELI_PINK,  ELI_PINK);
      }
    }
}

  void drawMetaModScreen(const MetaOscVisScreenStates &currState, const MetaOscVisScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
      // tft.drawRect(sqbound, sqbound, sqwidth, sqwidth, TFT_RED);
    }
    if (fullRedraw || currState.metaOsc != nextState.metaOsc) {
      tft.fillRect(sqbound,0,84+84,sqbound-1, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString(nextState.ptr->getName(), 120, 26);
    }
    if (fullRedraw || currState.moddepth != nextState.moddepth) {
      const int h = (84+84) * nextState.moddepth;
      Serial.printf("depth redraw %d\n", h);
      tft.fillRectVGradient(240-sqbound+1, sqbound, sqbound, h, TFT_DARKGREEN, TFT_GREEN);
      tft.fillRect(240-sqbound+1, sqbound + h, sqbound,(84+84) -h, ELI_BLUE);
    }
    if (fullRedraw || currState.modspeed != nextState.modspeed) {
      const int h = (84+84) * (1.0-nextState.modspeed);
      Serial.printf("depth redraw %d\n", h);
      tft.fillRect(0, sqbound, sqbound-1,h, ELI_BLUE);
      tft.fillRectVGradient(0, sqbound + h, sqbound-1, (84+84) -h, TFT_VIOLET, TFT_PURPLE);
    }
    nextState.ptr->draw(tft);

  }

  displayStates currState, nextState;

};

