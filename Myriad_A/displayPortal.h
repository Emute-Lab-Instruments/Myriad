#include <TFT_eSPI.h>
#include "drawing.h"
#include "oscVisData.hpp"
#include <array>
#include "metaOscs.hpp"
#include "oscDisplayModes.hpp"


TFT_eSPI tft = TFT_eSPI();  // Invoke custom library


template<size_t N_OSCS, size_t N_OSC_BANKS>
class displayPortal {
public:
  enum SCREENMODES {OSCBANKS, METAOSCVIS};

  std::vector<oscVisData*> oscVisDataPtrs;

  std::vector<oscDisplayModes*> oscvis;

  struct OscBankScreenStates {
    size_t oscModel[3];
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
        for(size_t i=0; i < oscvis.at(oscModelIdx)->data.size(); i++) {
          if (oscvis.at(oscModelIdx)->data.at(i)> 0) {    
            tft.drawSmoothArc(120, 120, 120-i, 120-i-1, startAngle, endAngle, col1, col1);
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

  void drawOscBankScreen(const OscBankScreenStates &currState, const OscBankScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
    }
    for(size_t i=0; i < 3; i++) {
      if (fullRedraw || currState.oscModel[i] != nextState.oscModel[i]) {
        updateOscVis(i, nextState.oscModel[i]);
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

