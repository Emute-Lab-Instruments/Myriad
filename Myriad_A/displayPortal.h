#include <TFT_eSPI.h>
#include "drawing.h"
#include "oscVisData.hpp"
#include <array>


TFT_eSPI tft = TFT_eSPI();  // Invoke custom library


class displayPortal {
public:
  enum SCREENMODES {OSCBANKS, METAOSCVIS};

  std::vector<oscVisData*> oscVisDataPtrs;

  struct OscBankScreenStates {
    size_t oscModel[3];
  };

  struct MetaOscVisScreenStates {
    int metaOsc;
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
      //draw whole oscbank
      drawOscBankScreen(currState.oscBankScreenState, nextState.oscBankScreenState, redraw);
    }else{
      //draw whole metaosc
      if (redraw)
        tft.fillScreen(TFT_RED);
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


private:
  void updateOscVis(size_t oscIdx, size_t oscModelIdx) {
    uint32_t startAngle = 120*oscIdx;
    uint32_t endAngle = 120 + (120*oscIdx);
    tft.drawSmoothArc(120, 120, 120, 0, startAngle, endAngle, ELI_BLUE, ELI_BLUE);
    uint32_t col1;
    const uint32_t cols[3] = {TFT_PURPLE, TFT_DARKGREEN, TFT_DARKCYAN};
    col1 = cols[oscIdx];
    for(size_t i=0; i < oscVisDataPtrs.at(oscModelIdx)->spec.size(); i++) {
      if (oscVisDataPtrs.at(oscModelIdx)->spec.at(i)> 0) {    
        tft.drawSmoothArc(120, 120, 120-i, 120-i-1, startAngle, endAngle, col1, col1);
      }
    }
  }

  void drawOscBankScreen(OscBankScreenStates currState, OscBankScreenStates nextState, bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
    }
    for(size_t i=0; i < 3; i++) {
      if (fullRedraw || currState.oscModel[i] != nextState.oscModel[i]) {
        updateOscVis(i, nextState.oscModel[i]);
      }
    }
  }

  void drawMetaModScreen() {
    tft.fillScreen(ELI_PINK);
  }


  displayStates currState, nextState;

};

displayPortal display;
