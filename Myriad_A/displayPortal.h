#include <TFT_eSPI.h>
#include "drawing.h"
#include "oscVisData.hpp"
#include <array>


TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

class displayPortal {
public:
  enum SCREENMODES {OSCBANKS, METAOSCVIS};

  void toggleScreen() {
    if (currScreenMode == SCREENMODES::OSCBANKS) {
      setScreen(SCREENMODES::METAOSCVIS);
    }else{
      setScreen(SCREENMODES::OSCBANKS);
    }
  }

  void setScreen(SCREENMODES newMode) {
    if (newMode != currScreenMode) {
      switch(newMode) {
        case SCREENMODES::OSCBANKS: {
          drawOscBankScreen();
        }
        break;
        case SCREENMODES::METAOSCVIS: {
          drawMetaModScreen();
        }
        break;
      }
    }
    currScreenMode = newMode;
  }

  void drawOscBankScreen() {
    tft.fillScreen(ELI_BLUE);
    drawOsc0(oldOsc0);
    drawOsc1(oldOsc1);
    drawOsc2(oldOsc2);
  }

  void drawMetaModScreen() {
    tft.fillScreen(ELI_PINK);
  }

  void drawOsc0(int osc) {
    tft.setCursor(50,120);
    tft.setTextColor(ELI_BLUE);
    tft.println(oldOsc0);
    tft.setCursor(50,120);
    tft.setTextColor(ELI_PINK);
    tft.println(osc);
    oldOsc0 = osc;

    // tft.drawSmoothArc(120, 120, 120, 110, 0, 120, TFT_RED, TFT_RED);
    // tft.drawSmoothArc(120, 120, 35, 34, 0, 120, TFT_RED, TFT_RED);
  }
  void drawOsc1(int osc) {
    tft.setCursor(120,120);
    tft.setTextColor(ELI_BLUE);
    tft.println(oldOsc1);
    tft.setCursor(120,120);
    tft.setTextColor(ELI_PINK);
    tft.println(osc);
    oldOsc1 = osc;
    tft.drawSmoothArc(120, 120, 80, 75, 120, 240, ELI_PINK, ELI_PINK);
  }
  void drawOsc2(int osc) {
    tft.setCursor(190,120);
    tft.setTextColor(ELI_BLUE);
    tft.println(oldOsc2);
    tft.setCursor(190,120);
    tft.setTextColor(ELI_PINK);
    tft.println(osc);
    oldOsc2 = osc;
    tft.drawSmoothArc(120, 120, 50, 40, 240, 360, TFT_DARKGREEN, TFT_DARKGREEN);
  }

  void updateOscVis(oscVisData &visData, size_t oscIdx) {
    uint32_t startAngle = 120*oscIdx;
    uint32_t endAngle = 120 + (120*oscIdx);
    tft.drawSmoothArc(120, 120, 120, 0, startAngle, endAngle, ELI_BLUE, ELI_BLUE);
    uint32_t col1;
    const uint32_t cols[3] = {TFT_PURPLE, TFT_DARKGREEN, TFT_DARKCYAN};
    col1 = cols[oscIdx];
    for(size_t i=0; i < visData.spec.size(); i++) {
      if (visData.spec.at(i)> 0) {    
        tft.drawSmoothArc(120, 120, 120-i, 120-i-1, startAngle, endAngle, col1, col1);
      }
    }
  }

  void redrawOsc(const size_t idx) {
    redrawOscFlags.at(idx) = true;
  }

  //update so drawing happens outside of interrupts, using redraw flags
  void update() {
    if (currScreenMode == SCREENMODES::OSCBANKS) {
      for(size_t i=0; i < 3; i++) {
        if(redrawOscFlags.at(i)) {

        }
      }
    }
  }

private:
  int oldOsc0=0;
  int oldOsc1=0;
  int oldOsc2=0;
  SCREENMODES lastScreenMode = SCREENMODES::OSCBANKS;
  SCREENMODES currScreenMode = SCREENMODES::OSCBANKS;
  std::array<bool, 3> redrawOscFlags = {true, true, true};
};

displayPortal display;
