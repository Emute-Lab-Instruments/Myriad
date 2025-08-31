#include <cmath>
#include <TFT_eSPI.h>
#include "drawing.h"
#include <array>
#include "metaOscs.hpp"
#include "oscDisplayModes.hpp"
#include "clockfreq.h"
#include "sinetable.h"
#include <sstream>
#include <iomanip>

TFT_eSPI __not_in_flash("display") tft = TFT_eSPI();  // Invoke custom library


template<size_t N_OSCS, size_t N_OSC_BANKS, size_t N_OSCILLATOR_MODELS>
class displayPortal {
public:
  enum SCREENMODES {OSCBANKS, METAOSCVIS, TUNING, CALIBRATE};

  // std::vector<oscDisplayModes*> oscvis;

  struct OscBankScreenStates {
    size_t oscModel[3];
    std::array<float, N_OSCS> oscWavelengths;
    metaOscPtr<N_OSCS> ptr;
    std::array<float, N_OSCS> modLineX, modLineY, modLineXStart, modLineYStart;
  };

  struct MetaOscVisScreenStates {
    int metaOsc;
    float moddepth;
    float modspeed;
    MODTARGETS modTarget;
    metaOscPtr<N_OSCS> ptr;
  };

  struct TuningScreenStates {
    int octtune=0;
    int semitonetune=0;
    int finetune=0;
  };

  struct CalibrationScreenStates {
    size_t adc0,adc1,adc2,adc3;
    size_t adcMin0,adcMin1,adcMin2,adcMin3;
    size_t adcMax0,adcMax1,adcMax2,adcMax3;
    size_t adcfilt0,adcfilt1,adcfilt2,adcfilt3;
    int encDelta[3];
    bool encSw[3];
    std::string calibScreenTitle;
    int heapSize;
  };

  struct displayStates {
    SCREENMODES screenMode;
    OscBankScreenStates oscBankScreenState;
    MetaOscVisScreenStates metaOscVisScreenState;
    CalibrationScreenStates calibrationScreenState;
    TuningScreenStates tuningState;
    bool redraw=false; //override and redraw anyway
  };

  // TFT_eSprite iconX = TFT_eSprite(&tft);
  // TFT_eSprite iconY = TFT_eSprite(&tft);
  // TFT_eSprite iconZ = TFT_eSprite(&tft);
  TFT_eSprite icon = TFT_eSprite(&tft);

  using eSpritePtr = std::shared_ptr<TFT_eSprite>;
  using oscModelSprites = std::array<eSpritePtr, N_OSCILLATOR_MODELS>;
  std::array<oscModelSprites, N_OSC_BANKS> oscModelIcons;

  
  constexpr static int iconw = 17;
  constexpr static int iconh = 15;
  static constexpr int colBank0 = ELI_PINK;
  static constexpr int colBank1 = ELI_PINK3;
  static constexpr int colBank2 = ELI_PINK2;

  static constexpr int bankColArray[3] = {colBank0, colBank1, colBank2};


  displayPortal() {
    //create icons etc
    // TFT_eSprite* icons[] = {&iconX, &iconY, &iconZ};
    // String iconChars[] = {"X","Y","Z"};
    // for(size_t i=0; i < 3; i++) {
    //   icons[i]->createSprite(17,14);
    //   icons[i]->fillRect(0,0,17,14, TFT_RED);
    //   icons[i]->setFreeFont(&FreeSansBold9pt7b);
    //   icons[i]->setTextColor(TFT_BLACK, TFT_SILVER);
    //   icons[i]->drawString(iconChars[i],0,0);
    //   icons[i]->setPivot(8, 110);
    // }
    for(size_t bank=0; bank < N_OSC_BANKS; bank++) {
      for(size_t model=0; model < N_OSCILLATOR_MODELS; model++) {
        oscModelIcons[bank][model] = std::make_shared<TFT_eSprite>(&tft);
        oscModelIcons[bank][model]->createSprite(iconw, iconh);
        oscModelIcons[bank][model]->fillRect(0,0,iconw, iconh, bankColArray[bank]);
        oscModelIcons[bank][model]->setFreeFont(&FreeSansBold9pt7b);
        // oscModelIcons[bank][model]->setTextColor(TFT_BLACK, TFT_SILVER);
        // String idxstr = String(model);
        // oscModelIcons[bank][model]->drawString(idxstr.c_str(),0,0);
      }
    }
    icon.createSprite(20,20);

    tft.setPivot(120,120);
    nextState.redraw = true;
  }

  void __force_inline setDisplayWavelengths(const std::array<float, N_OSCS> &wavelengths) {
    nextState.oscBankScreenState.oscWavelengths = wavelengths;
  }

  void update() {
    // Serial.println("update");
    bool redraw = nextState.redraw;
    // if (!redraw) {
    //   if(nextState.screenMode != currState.screenMode) {
    //     redraw = true;
    //   }
    // }else{
    //   nextState.redraw = false;
    // }
    switch(nextState.screenMode) {
      case SCREENMODES::OSCBANKS:
      {
        drawOscBankScreen(currState.oscBankScreenState, nextState.oscBankScreenState, redraw);
        break;
      }
      case SCREENMODES::METAOSCVIS:
      {
        drawMetaModScreen(currState.metaOscVisScreenState, nextState.metaOscVisScreenState, redraw);
        break;
      }
      case SCREENMODES::TUNING:
      {
        drawTuningScreen(currState.tuningState, nextState.tuningState, redraw);
        break;
      }
      case SCREENMODES::CALIBRATE:
      {
        drawCalibrationScreen(currState.calibrationScreenState, nextState.calibrationScreenState, redraw);
        break;
      }

    }
    currState = nextState;
    nextState.redraw = false;
  }

  void setScreen(SCREENMODES newMode) {
    nextState.screenMode = newMode;
    nextState.redraw = true;
  }

  void setModTarget(MODTARGETS target) {
    nextState.metaOscVisScreenState.modTarget = target;
  }

  void setOscBankModel(size_t newBank, size_t newOscModel) {
    nextState.oscBankScreenState.oscModel[newBank] = newOscModel;
  }

  void setMetaOsc(size_t newIdx, metaOscPtr<N_OSCS> newPtr) {
    nextState.metaOscVisScreenState.metaOsc = newIdx;
    nextState.metaOscVisScreenState.ptr = newPtr; 
    nextState.oscBankScreenState.ptr = newPtr; 
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

  void setCalibADCValues(size_t adc0, size_t adc1, size_t adc2, size_t adc3) {
    nextState.calibrationScreenState.adc0 = adc0;
    nextState.calibrationScreenState.adc1 = adc1;
    nextState.calibrationScreenState.adc2 = adc2;
    nextState.calibrationScreenState.adc3 = adc3;
  }

  void setCalibADCFiltValues(size_t adc0, size_t adc1, size_t adc2, size_t adc3) {
    nextState.calibrationScreenState.adcfilt0 = adc0;
    nextState.calibrationScreenState.adcfilt1 = adc1;
    nextState.calibrationScreenState.adcfilt2 = adc2;
    nextState.calibrationScreenState.adcfilt3 = adc3;
  }

  void setCalibADCMinMaxValues(size_t adcMins[4], size_t adcMaxs[4]) {
    nextState.calibrationScreenState.adcMin0 = adcMins[0];
    nextState.calibrationScreenState.adcMin1 = adcMins[1];
    nextState.calibrationScreenState.adcMin2 = adcMins[2];
    nextState.calibrationScreenState.adcMin3 = adcMins[3];
    nextState.calibrationScreenState.adcMax0 = adcMaxs[0];
    nextState.calibrationScreenState.adcMax1 = adcMaxs[1];
    nextState.calibrationScreenState.adcMax2 = adcMaxs[2];
    nextState.calibrationScreenState.adcMax3 = adcMaxs[3];
  }

  void setCalibEncoderDelta(int encIdx, int delta) {
    if (encIdx < 3) {
      nextState.calibrationScreenState.encDelta[encIdx] = delta;
    }
  }

  void setCalibEncoderSwitch(int encIdx, bool sw) {
    if (encIdx < 3) {
      nextState.calibrationScreenState.encSw[encIdx] = sw;
    }
  }

  void setCalibScreenTitle(const std::string &title) {
    nextState.calibrationScreenState.calibScreenTitle = title;
  } 

  void setTuning(int oct, int semi, int fine) {
    nextState.tuningState.octtune = oct;
    nextState.tuningState.semitonetune = semi;
    nextState.tuningState.finetune = fine;
  }

  void setFreeHeap(int freeMem) {
    nextState.calibrationScreenState.heapSize = freeMem;
  }

private:

  std::array<float,N_OSCS> oscVisPhase{};

  void drawOscBankScreen(const OscBankScreenStates &currState, OscBankScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
    }
    
    static constexpr float slowestWavelen = sampleClock/20.f;
    static constexpr float fastestWavelen = sampleClock/20000.f;
    static constexpr float rangeWavelen = slowestWavelen - fastestWavelen;
    static constexpr float rangeWavelenRcp = 1.0/rangeWavelen;
    static constexpr float slowestOscVisSpeed = 0.001;
    static constexpr float fastestOscVisSpeed = 0.03;
    static constexpr float rangeOscVisSpeed = fastestOscVisSpeed - slowestOscVisSpeed;
    static constexpr float unitR=7;
    float bank=0;
    auto modVals = nextState.ptr->getValues();
    static constexpr int oscColArray[9] = {colBank0, colBank0, colBank0, colBank1, colBank1, colBank1, colBank2, colBank2, colBank2};

    for(size_t i=0; i < N_OSCS; i++) {
      const float prevpos = oscVisPhase[i] * TWOPI;
      const float prevcospos = sineTable::fast_cos(prevpos);
      const float prevsinpos = sineTable::fast_sin(prevpos);
      const float linelen=18 + ((i+1)*unitR);
      const float halflinelen = linelen * 0.5;

      const size_t cx = 120+ (linelen * prevcospos);
      const size_t cy = 120+ (linelen * prevsinpos);
      tft.drawLine(120,120, cx,cy, ELI_BLUE );
      tft.drawLine(currState.modLineXStart[i], currState.modLineYStart[i], currState.modLineX[i], currState.modLineY[i], ELI_BLUE );
      tft.drawCircle(cx, cy, 5, ELI_BLUE);

      float normwavelen = 1.0 - ((nextState.oscWavelengths[i] - fastestWavelen) * rangeWavelenRcp);
      normwavelen= normwavelen * normwavelen * normwavelen;
      normwavelen = std::max(normwavelen,0.f);
      const float speed = (rangeOscVisSpeed * normwavelen) + slowestOscVisSpeed;

      oscVisPhase[i] += speed;
      if (oscVisPhase[i] >= 1.f) {
        oscVisPhase[i] -=1.f;
      }
      const float pos = oscVisPhase[i] * TWOPI;
      
      const float cospos = sineTable::fast_cos(pos);
      const float sinpos = sineTable::fast_sin(pos);
      const size_t cx2 = 120+ (linelen * cospos);
      const size_t cy2 = 120+ (linelen * sinpos);
      tft.drawLine(120,120, cx2,cy2, oscColArray[i] );

      if (modVals[i] != 0.f) {
        float modv = (modVals[i] * nextState.ptr->moddepth.getInvMax() * 0.5f);
        // if (i==0) {
          // Serial.print(modv);
          // Serial.print("\t");
        // }
        const size_t cx3start = 120+ (halflinelen * cospos);
        const size_t cy3start = 120+ (halflinelen * sinpos);
        const size_t cx3 = 120+ ((halflinelen + (modv * halflinelen)) * cospos);
        const size_t cy3 = 120+ ((halflinelen + (modv * halflinelen)) * sinpos);
        tft.drawLine(cx3start,cy3start, cx3,cy3, TFT_GREEN );
        nextState.modLineX[i] = cx3;
        nextState.modLineY[i] = cy3;
        nextState.modLineXStart[i] = cx3start;
        nextState.modLineYStart[i] = cy3start;
      }
      

      tft.drawCircle(cx2, cy2, 5, oscColArray[i]);

    } 

    constexpr float angleRange = 120.f/ N_OSCILLATOR_MODELS;
    constexpr std::array<size_t, 15> colours = {TFT_RED, TFT_GREEN, TFT_MAGENTA, TFT_CYAN, TFT_YELLOW,TFT_ORANGE, TFT_GOLD,  TFT_GREENYELLOW,TFT_BLUE,TFT_PURPLE, TFT_SKYBLUE, TFT_VIOLET, TFT_PINK, TFT_LIGHTGREY, TFT_WHITE};

    // icon.deleteSprite();
    // icon.createSprite(50,50);
    // tft.setPivot(120,120);
    for(size_t i=0; i < 3; i++) {
      if (fullRedraw || currState.oscModel[i] != nextState.oscModel[i]) {
      icon.fillSprite(TFT_RED);
      icon.setTextFont(2);
      icon.setTextColor(TFT_BLACK);
      icon.drawString("X",0,0);
      icon.setPivot(8, 120);
    // // icon.pushSprite(bankTxtX[i]-20, bankTxtY[i]);
      // icon.pushRotated(120, TFT_BLACK);


        static const int bankTxtX[3] = {120-87, 120,120+87};
        static const int bankTxtY[3] = {120+49,20,120+49};
        tft.setFreeFont(&FreeMono9pt7b);
        tft.setTextDatum(CC_DATUM);
        const int textWidth00 = tft.textWidth("00");
        tft.fillRect(bankTxtX[i]-(textWidth00>>1), bankTxtY[i]-(tft.fontHeight()>>1), textWidth00, tft.fontHeight(), ELI_BLUE);
        tft.setTextColor(bankColArray[i], ELI_BLUE);
        tft.drawString(String(nextState.oscModel[i]), bankTxtX[i], bankTxtY[i]);
        static const int iconX[3] = {120-77, 130,120+67};
        static const int iconY[3] = {120+59,10,120+59};
        oscModelIcons[i][nextState.oscModel[i]]->pushSprite(iconX[i], iconY[i]);
        // icon.fillSprite(TFT_RED);
        // icon.setTextFont(2);
        // icon.setTextColor(TFT_BLACK);
        // icon.drawString("X",0,0);
        // icon.setPivot(8, 120);
        // icon.pushSprite(bankTxtX[i]-20, bankTxtY[i]);
        // icon.pushRotated(120, TFT_BLACK);
        // // icon.deleteSprite();

        tft.drawArc(120,120,120,115,(i*120), (i+1) * 120, ELI_BLUE,  ELI_BLUE);
        size_t colour = colours.at(nextState.oscModel[i] % colours.size());
        tft.drawArc(120,120,120,115,(i*120) + (nextState.oscModel[i] * angleRange), (i* 120) + ((nextState.oscModel[i]+1) * angleRange), colour,  colour);
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
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString(nextState.ptr->getName(), 120, 26);
    }
    constexpr float innerArc = 117;
    if (fullRedraw || currState.moddepth != nextState.moddepth) {
      // const int h = (84+84) * nextState.moddepth;
      // Serial.printf("depth redraw %d\n", h);
      // tft.fillRect(240-sqbound+1, sqbound, sqbound,h, ELI_BLUE);
      // tft.fillRect(240-sqbound+1, sqbound+h, 5, (84+84) -h, TFT_GREEN);
      const float angle = 220 + (currState.moddepth * 80);
      tft.drawSmoothArc(120, 120, 120, innerArc, angle , angle+10, ELI_BLUE, ELI_BLUE);
      if (nextState.metaOsc > 0.f) {
        const float angle2 = 220.f + (nextState.moddepth * 80.f);
        tft.drawSmoothArc(120, 120, 120, innerArc, angle2 , angle2+10, ELI_PINK2, ELI_PINK2);
      }
    }
    if (fullRedraw || currState.modspeed != nextState.modspeed) {
      // const int h = (84+84) * (1.0-nextState.modspeed);
      // // Serial.printf("depth redraw %d\n", h);
      // tft.fillRect(0, sqbound, sqbound-1,h, ELI_BLUE);
      // tft.fillRect(sqbound-5, sqbound + h, 5, (84+84) -h, TFT_VIOLET);
      const float angle = 140.f - (currState.modspeed * 80.f);
      tft.drawSmoothArc(120, 120, 120, innerArc, angle-10 , angle, ELI_BLUE, ELI_BLUE);
      if (nextState.metaOsc > 0) {
        const float angle2 = 140 - (nextState.modspeed * 80);
        tft.drawSmoothArc(120, 120, 120, innerArc, angle2-10 , angle2, ELI_PINK, ELI_PINK);
        // Serial.printf("angle: %f %f\n", angle, angle2);
      }
    }
    if (fullRedraw || currState.modTarget != nextState.modTarget) {
      tft.fillRect(0,200,240,240, ELI_BLUE);
      tft.setFreeFont(&FreeMonoOblique9pt7b);
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setTextDatum(CC_DATUM);
      switch(nextState.modTarget) {
        case MODTARGETS::PITCH: {
        tft.drawString(">pitch<", 120, 210);
          break;
        }
        case MODTARGETS::EPSILON: {
          tft.drawString(">eps<", 120, 210);
          break;
        }
        case MODTARGETS::PITCH_AND_EPSILON: {
          tft.drawString(">pit+eps<", 120, 210);
          break;
        }
      }
      tft.setFreeFont(&FreeMonoBold9pt7b);
      tft.setTextColor(TFT_DARKGREEN, TFT_LIGHTGREY);
      tft.drawString("Z", 170, 210);

    }

    nextState.ptr->draw(tft);

  }

  std::string padNumberWithZeros(int number, int width) {
      std::ostringstream ss;
      ss << std::setw(width) << std::setfill('0') << number;
      return ss.str();
  }

  void drawCalibrationScreen(const CalibrationScreenStates &currState, const CalibrationScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
      tft.setTextColor(ELI_BLUE, ELI_PINK);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString(nextState.calibScreenTitle.c_str(), 120, 26);

      constexpr char compile_date[] = __DATE__;
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.drawString(compile_date, 100, 46);

    }

    if (fullRedraw || currState.heapSize != nextState.heapSize) {
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString(String(nextState.heapSize).c_str() + String(" bytes"), 100, 66);
    }


    constexpr int textgap = 23;
    constexpr int adcy4 = 155;
    constexpr int adcy3 = adcy4 - textgap;
    constexpr int adcy2 = adcy3 - textgap;
    constexpr int adcy1 = adcy2 - textgap;

    if (fullRedraw || currState.adc0 != nextState.adc0) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adc0,4);
      tft.drawString(str.c_str(), 30, adcy1);
    }
    if (fullRedraw || currState.adc1 != nextState.adc1) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adc1,4);
      tft.drawString(str.c_str(), 30, adcy2);
    }
    if (fullRedraw || currState.adc2 != nextState.adc2) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adc2,4);
      tft.drawString(str.c_str(), 30, adcy3);
    }
    if (fullRedraw || currState.adc3 != nextState.adc3) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adc3,4);
      tft.drawString(str.c_str(), 30, adcy4);
    }
    //---------

    if (fullRedraw || currState.adcfilt0 != nextState.adcfilt0) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcfilt0,4);
      tft.drawString(str.c_str(), 100, adcy1);
    }
    if (fullRedraw || currState.adcfilt1 != nextState.adcfilt1) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcfilt1,4);
      tft.drawString(str.c_str(), 100, adcy2);
    }
    if (fullRedraw || currState.adcfilt2 != nextState.adcfilt2) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcfilt2,4);
      tft.drawString(str.c_str(), 100, adcy3);
    }
    if (fullRedraw || currState.adcfilt3 != nextState.adcfilt3) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcfilt3,4);
      tft.drawString(str.c_str(), 100, adcy4);
    }
    //----------

    if (fullRedraw || currState.adcMin0 != nextState.adcMin0) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMin0,2);
      tft.drawString(str.c_str(), 150, adcy1);
    }
    if (fullRedraw || currState.adcMin1 != nextState.adcMin1) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMin1,2);
      tft.drawString(str.c_str(), 150, adcy2);
    }
    if (fullRedraw || currState.adcMin2 != nextState.adcMin2) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMin2,2);
      tft.drawString(str.c_str(), 150, adcy3);
    }
    if (fullRedraw || currState.adcMin3 != nextState.adcMin3) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMin3,2);
      tft.drawString(str.c_str(), 150, adcy4);
    }

    //-------------

    if (fullRedraw || currState.adcMax0 != nextState.adcMax0) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMax0,4);
      tft.drawString(str.c_str(), 180, adcy1);
    }
    if (fullRedraw || currState.adcMax1 != nextState.adcMax1) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMax1,4);
      tft.drawString(str.c_str(), 180, adcy2);
    }
    if (fullRedraw || currState.adcMax2 != nextState.adcMax2) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMax2,4);
      tft.drawString(str.c_str(), 180, adcy3);
    }
    if (fullRedraw || currState.adcMax3 != nextState.adcMax3) {
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(TL_DATUM);
      std::string str=padNumberWithZeros(nextState.adcMax3,4);
      tft.drawString(str.c_str(), 180, adcy4);
    }

    //rotary encs
    for (auto &j: {0,1,2}) {
      constexpr size_t indexMap[3] = {1,0,2};
      size_t i = indexMap[j];
      if (fullRedraw || currState.encDelta[i] != nextState.encDelta[i]) {
        int tlx = 50 + (j*50);
        int tly = 190;
        tft.fillRect(tlx, tly, 45,22, ELI_BLUE);
        tft.setTextColor(ELI_PINK, ELI_BLUE);
        tft.setFreeFont(&FreeMono9pt7b);
        tft.setTextDatum(TL_DATUM);
        std::string str=padNumberWithZeros(nextState.encDelta[i],1);
        tft.drawString(str.c_str(),tlx, tly);
      }
      if (fullRedraw || currState.encSw[i] != nextState.encSw[i]) {
        int tlx = 70 + (j*50);
        int tly = 210;
        tft.fillRect(tlx, tly, 45,22, ELI_BLUE);
        tft.setTextColor(ELI_PINK, ELI_BLUE);
        tft.setFreeFont(&FreeMono9pt7b);
        tft.setTextDatum(TL_DATUM);
        std::string str=nextState.encSw[i] ? "1" : "0";
        tft.drawString(str.c_str(), tlx, tly);
      }
    }

  }

  void drawTuningScreen(const TuningScreenStates &currState, const TuningScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString("Tuning", 120, 26);
      tft.setTextDatum(TR_DATUM);
      tft.drawString("Oct", 70, 70);
      tft.drawString("Semi", 70, 120);
      tft.drawString("Fine", 70, 170);
    }
    if (fullRedraw || currState.octtune != nextState.octtune) {
      const int TLY=70;
      tft.fillRect(90,TLY-20,100,40, ELI_BLUE);
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.octtune, 120, TLY);
    }
    if (fullRedraw || currState.semitonetune != nextState.semitonetune) {
      const int TLY=120;
      tft.fillRect(90,TLY-20,100,40, ELI_BLUE);
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.semitonetune, 120, TLY);
    }
    if (fullRedraw || currState.finetune != nextState.finetune) {
      const int TLY=170;
      tft.fillRect(90,TLY-20,190,40, ELI_BLUE);
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.finetune, 120, TLY);
    }
  }


  displayStates currState, nextState;

};

