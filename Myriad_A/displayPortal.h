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
#include <functional>

TFT_eSPI __not_in_flash("display") tft = TFT_eSPI();  // Invoke custom library


template<size_t N_OSCS, size_t N_OSC_BANKS, size_t N_OSCILLATOR_MODELS>
class displayPortal {
public:
  enum SCREENMODES {OSCBANKS, METAOSCVIS, TUNING, CALIBRATE, UTILITY, QUANTISE};

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
    bool bypass=false;
  };

  struct QuantiseScreenStates {
    size_t pull=0;
    size_t notesperoct=0;
  };

  struct UtilScreenStates {
    int dummy=0;
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
    UtilScreenStates utilityState;
    QuantiseScreenStates quantiseState;
    bool redraw=false; //override and redraw anyway
  };

  TFT_eSprite iconXPush = TFT_eSprite(&tft);
  TFT_eSprite iconYPush = TFT_eSprite(&tft);
  TFT_eSprite iconZPush = TFT_eSprite(&tft);
  TFT_eSprite iconXTurn = TFT_eSprite(&tft);
  TFT_eSprite iconYTurn = TFT_eSprite(&tft);
  TFT_eSprite iconZTurn = TFT_eSprite(&tft);
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
  using iconDrawFunction = std::function<void(eSpritePtr&, int)>;
  std::map<String, iconDrawFunction> iconDrawFunctions;


  displayPortal() {
    //create icons etc
    TFT_eSprite* icons[] = {&iconXPush, &iconYPush, &iconZPush};
    TFT_eSprite* iconsturn[] = {&iconXTurn, &iconYTurn, &iconZTurn};
    String iconChars[] = {"X","Y","Z"};
    for(size_t i=0; i < 3; i++) {
      icons[i]->createSprite(11,9);
      icons[i]->fillSprite(ELI_BLUE);
      icons[i]->setTextFont(1);
      icons[i]->setTextDatum(CC_DATUM);
      icons[i]->setTextColor(TFT_SILVER, ELI_BLUE);
      icons[i]->drawString(iconChars[i],6,4);
      // icons[i]->drawLine(3,0,3,8, TFT_SILVER);
      icons[i]->drawLine(0,8,10,8, TFT_SILVER);
      icons[i]->drawLine(0,8,0,6, TFT_SILVER);
      icons[i]->drawLine(10,8,10,6, TFT_SILVER);

      iconsturn[i]->createSprite(15,15);
      iconsturn[i]->fillSprite(ELI_BLUE);
      iconsturn[i]->setTextFont(1);
      iconsturn[i]->setTextDatum(CC_DATUM);
      iconsturn[i]->setTextColor(TFT_SILVER, ELI_BLUE);
      iconsturn[i]->drawString(iconChars[i],8,8);
      iconsturn[i]->drawCircle(7,7, 7, TFT_SILVER);
    }
    // icon.createSprite(20,20);

    // tft.setPivot(120,120);
    nextState.redraw = true;
  }

  void init(std::vector<String> &oscModelIDs) {

      


    iconDrawFunctions["saw"] = [](eSpritePtr& sprite, int col) {
        // sprite->fillSprite(TFT_RED);
        // sprite->drawLine(0,0,0,iconh, col);
        sprite->drawLine(0,iconh-1,iconw-1,0, col);
        sprite->drawLine(iconw-1,0, iconw-1,iconh-1, col);
    };

    iconDrawFunctions["sdt10"] = [](eSpritePtr& sprite, int col) {
        sprite->drawLine(0,iconh-1,iconw>>1,0, col);
        sprite->drawLine(iconw>>1,0, iconw>>1,iconh-1, col);
        sprite->drawLine(iconw>>1,iconh-1, iconw-1,iconh-1, col);
    };
    
    auto drawPulse = [](eSpritePtr& sprite, int col, int top, int pw) {
        sprite->drawLine(0,iconh-1,0,top, col);
        sprite->drawLine(0,top,pw,top, col);
        sprite->drawLine(pw,top,pw,iconh-1, col);
        sprite->drawLine(pw,iconh-1,iconw-1,iconh-1, col);
    };

    iconDrawFunctions["pulsesd"] = [drawPulse](eSpritePtr& sprite, int col) {
      drawPulse(sprite, col, 0, iconw>>1);
    };

    iconDrawFunctions["sq2"] = [drawPulse](eSpritePtr& sprite, int col) {
      drawPulse(sprite, col, 0, 3);
      drawPulse(sprite, col, iconh*0.25, 6);
      drawPulse(sprite, col, iconh*0.5, 9);
      drawPulse(sprite, col, iconh*0.75, 12);
    };

    iconDrawFunctions["sq14"] = [drawPulse](eSpritePtr& sprite, int col) {
      drawPulse(sprite, col, 0, iconw * 0.2);
      drawPulse(sprite, col, 0.25, iconw * 0.4);
      drawPulse(sprite, col, 0, iconw * 0.6);
      drawPulse(sprite, col, 0, iconw * 0.8);
      drawPulse(sprite, col, 0, iconw -1);
    };

    iconDrawFunctions["tri"] = [](eSpritePtr& sprite, int col) {
        sprite->drawLine(0,iconh-1,iconw>>1,0, col);
        sprite->drawLine(iconw>>1,0, iconw-1,iconh-1, col);
    };

    iconDrawFunctions["trv10"] = [](eSpritePtr& sprite, int col) {
        sprite->drawLine(0,iconh-1,iconw>>1,0, col);
        sprite->drawLine(iconw>>1,0, iconw-1,iconh-1, col);
        // sprite->drawLine(0,iconh-1,iconw>>1,iconh*0.33, col);
        // sprite->drawLine(iconw>>1,iconh*0.33, iconw-1,iconh-1, col);
        sprite->drawLine(0,iconh-1,iconw>>1,iconh*0.66, col);
        sprite->drawLine(iconw>>1,iconh*0.66, iconw-1,iconh-1, col);
    };

    iconDrawFunctions["slide"] = [](eSpritePtr& sprite, int col) {
        sprite->drawLine(0,iconh-1,0,0, col);
        sprite->drawLine(iconw-1,iconh-1,iconw-1,0, col);
        sprite->drawLine(0,iconh>>1, iconw-1,iconh>>1, col);
        sprite->fillCircle(iconw*0.33,iconh>>1,3, col);
    };

    iconDrawFunctions["exp1"] = [](eSpritePtr& sprite, int col) {
        for(float i=1; i >0; i-= 0.2) {
          float offset = (1.0-i) * iconw;
          sprite->drawLine(offset,iconh-1, offset, iconh-(i*i*iconh), col);
        }
    };


    iconDrawFunctions["p100"] = [](eSpritePtr& sprite, int col) {
      sprite->drawRect(0,(iconh>>1) - (iconh * 0.5), iconw * 0.3, iconh, col);
      sprite->drawRect(0,(iconh>>1) - (iconh * 0.3), iconw * 0.7, iconh * 0.6, col);
      sprite->drawRect(0,(iconh>>1) - (iconh * 0.1), iconw * 0.99, iconh * 0.2, col);
    };

    iconDrawFunctions["n2"] = [](eSpritePtr& sprite, int col) {
      constexpr std::array<float, 30> randomFloats = {
          0.374f, 0.950f, 0.731f, 0.598f, 0.156f, 0.058f, 0.455f, 0.206f, 0.828f, 0.976f,
          0.004f, 0.373f, 0.513f, 0.952f, 0.916f, 0.635f, 0.717f, 0.141f, 0.606f, 0.817f,
          0.314f, 0.585f, 0.479f, 0.860f, 0.123f, 0.752f, 0.492f, 0.018f, 0.394f, 0.681f
      };
      constexpr std::array<float, 30> randomFloats2 = {
          0.287f, 0.574f, 0.861f, 0.148f, 0.435f, 0.722f, 0.009f, 0.296f, 0.583f, 0.870f,
          0.157f, 0.444f, 0.731f, 0.018f, 0.305f, 0.592f, 0.879f, 0.166f, 0.453f, 0.740f,
          0.027f, 0.314f, 0.601f, 0.888f, 0.175f, 0.462f, 0.749f, 0.036f, 0.323f, 0.610f
      };      
      for(size_t i=0; i < 10; i++) {
        sprite->drawCircle(randomFloats[i]*iconw, randomFloats2[i]*iconh, 3, col);
      }

    };


    iconDrawFunctions["sil"] = [](eSpritePtr& sprite, int col) {
      auto smallest = std::min(iconw, iconh);
      sprite->drawCircle(iconw>>1,iconh>>1,smallest>>1, col);
    };

    iconDrawFunctions["wn"] = [](eSpritePtr& sprite, int col) {
      constexpr std::array<float, 30> randomFloats = {
          0.642f, 0.108f, 0.795f, 0.321f, 0.967f, 0.453f, 0.089f, 0.736f, 0.274f, 0.851f,
          0.418f, 0.063f, 0.729f, 0.395f, 0.982f, 0.527f, 0.164f, 0.810f, 0.346f, 0.923f,
          0.559f, 0.195f, 0.772f, 0.408f, 0.034f, 0.681f, 0.217f, 0.864f, 0.490f, 0.126f
      };
      constexpr std::array<float, 30> randomFloats2 = {
          0.913f, 0.247f, 0.584f, 0.028f, 0.765f, 0.392f, 0.819f, 0.156f, 0.693f, 0.430f,
          0.067f, 0.804f, 0.541f, 0.178f, 0.715f, 0.452f, 0.089f, 0.826f, 0.263f, 0.600f,
          0.937f, 0.374f, 0.711f, 0.048f, 0.785f, 0.322f, 0.659f, 0.096f, 0.833f, 0.570f
      };
      for(size_t i=0; i < 30; i++) {
        sprite->drawPixel(randomFloats[i]*iconw, randomFloats2[i]*iconh, col);
      }

    };

    for(size_t bank=0; bank < N_OSC_BANKS; bank++) {
      for(size_t model=0; model < N_OSCILLATOR_MODELS; model++) {
        oscModelIcons[bank][model] = std::make_shared<TFT_eSprite>(&tft);
        oscModelIcons[bank][model]->createSprite(iconw, iconh);
        auto it = iconDrawFunctions.find(oscModelIDs[model]);
        if (it != iconDrawFunctions.end()) {
            oscModelIcons[bank][model]->fillSprite(ELI_BLUE);
            it->second(oscModelIcons[bank][model],bankColArray[bank]);
        }
        else
        {        
          oscModelIcons[bank][model]->fillRect(0,0,iconw, iconh, bankColArray[bank]);
          oscModelIcons[bank][model]->setFreeFont(&FreeSansBold9pt7b);
          oscModelIcons[bank][model]->setTextColor(TFT_BLACK, TFT_SILVER);
          String idxstr = String(oscModelIDs[model][0]);
          oscModelIcons[bank][model]->drawString(idxstr.c_str(),0,0);
        }      
      }
    }
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
      case SCREENMODES::QUANTISE:
      {
        drawQuantiseScreen(currState.quantiseState, nextState.quantiseState, redraw);
        break;
      }
      case SCREENMODES::CALIBRATE:
      {
        drawCalibrationScreen(currState.calibrationScreenState, nextState.calibrationScreenState, redraw);
        break;
      }
      case SCREENMODES::UTILITY:
      {
        drawUtilityScreen(currState.utilityState, nextState.utilityState, redraw);
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

  void setTuningBypass(bool val) {
    nextState.tuningState.bypass = val;
  }

  void setQuant(size_t pull, size_t notes) {
    nextState.quantiseState.pull = pull;
    nextState.quantiseState.notesperoct = notes;
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
        static const int iconX[3] = {120-79, 133,120+67};
        static const int iconY[3] = {120+61,15,120+61};
        // tft.fillRect(iconX[i], iconY[i], iconw, iconh, ELI_BLUE);
        oscModelIcons[i][nextState.oscModel[i]]->pushSprite(iconX[i], iconY[i]);
        // icon.fillSprite(TFT_RED);
        // icon.setTextFont(2);
        // icon.setTextColor(TFT_BLACK);
        // icon.drawString("X",0,0);
        // icon.setPivot(8, 120);
        // icon.pushSprite(bankTxtX[i]-20, bankTxtY[i]);
        // icon.pushRotated(120, TFT_BLACK);
        // // icon.deleteSprite();

        iconYPush.pushSprite(105,218);
        tft.setTextFont(1);
        tft.setTextColor(TFT_SILVER);
        tft.drawString("mod",130,224);

        iconXTurn.pushSprite(120-101,120+21);
        iconYTurn.pushSprite(92,18);
        iconZTurn.pushSprite(120+95, 120+21);

        tft.drawArc(120,120,120,115,(i*120), (i+1) * 120, ELI_BLUE, ELI_BLUE);
        // tft.drawArc(120,120,120,119,(i*120), (i+1) * 120, bankColArray[i],  bankColArray[i]);
        size_t colour = colours.at(nextState.oscModel[i] % colours.size());
        tft.drawArc(120,120,120,115,(i*120) + (nextState.oscModel[i] * angleRange), (i* 120) + ((nextState.oscModel[i]+1) * angleRange), colour,  colour);

        // tft.drawArc(120,120,120,110, (i*120)-1,(i*120)+1, ELI_PINK, ELI_PINK);
        // float pos = (i*120) - 30.f;
        // if (pos < 0.f) pos += 360.f;
        // pos = pos / 360.f * TWOPI;
        // const float cospos = sineTable::fast_cos(pos);
        // const float sinpos = sineTable::fast_sin(pos);
        // // Serial.printf("%f %f %f", pos, cospos, sinpos);
        // const size_t cx = 120+ (120.f * cospos);
        // const size_t cy = 120+ (120.f * sinpos);
        // const size_t cx2 = 120+ (110.f * cospos);
        // const size_t cy2 = 120+ (110.f * sinpos);
        // tft.drawLine(cx, cy, cx2, cy2, ELI_PINK);

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
        // iconXTurn.pushSprite(120-20,120-110);
        // iconYTurn.pushSprite(120,116);
        // iconZTurn.pushSprite(120+20, 120-110);

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
      // tft.setFreeFont(&FreeMonoBold9pt7b);
      // tft.setTextColor(TFT_DARKGREEN, TFT_LIGHTGREY);
      // tft.drawString("Z", 170, 210);
      iconZPush.pushSprite(172,205);

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
    const bool bypassRedraw = currState.bypass != nextState.bypass;
    if (fullRedraw || bypassRedraw) {
      tft.fillScreen(ELI_BLUE);

      tft.setTextColor(nextState.bypass ? GREYED_OUT_COL : ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString("Tuning", 120, 26);
      tft.setTextDatum(TR_DATUM);
      iconXTurn.pushSprite(170,65);
      tft.drawString("Oct", 70, 70);

      iconYTurn.pushSprite(170,105);
      tft.drawString("Semi", 70, 110);
      
      iconZTurn.pushSprite(170,145);
      tft.drawString("Fine", 70, 150);

      tft.setTextFont(1);
      tft.setTextColor(TFT_SILVER, ELI_BLUE);
      tft.setTextDatum(CC_DATUM);

      iconXPush.pushSprite(75,190);
      tft.drawString("Bypass", 120,195);

      iconYPush.pushSprite(75,205);
      tft.drawString("Quantise", 120,210);
      iconZPush.pushSprite(85,220);
      tft.drawString("Exit", 120,225);
    }
    if (fullRedraw || currState.octtune != nextState.octtune || bypassRedraw) {
      const int TLY=70;
      tft.fillRect(90,TLY-20,60,40, ELI_BLUE);
      tft.setTextColor(nextState.bypass ? GREYED_OUT_COL : TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.octtune, 120, TLY);
    }
    if (fullRedraw || currState.semitonetune != nextState.semitonetune || bypassRedraw) {
      const int TLY=110;
      tft.fillRect(90,TLY-20,60,40, ELI_BLUE);
      tft.setTextColor(nextState.bypass ? GREYED_OUT_COL : TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.semitonetune, 120, TLY);
    }
    if (fullRedraw || currState.finetune != nextState.finetune || bypassRedraw) {
      const int TLY=150;
      tft.fillRect(90,TLY-20,60,40, ELI_BLUE);
      tft.setTextColor(nextState.bypass ? GREYED_OUT_COL : TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.finetune, 120, TLY);
    }
  }

  void drawQuantiseScreen(const QuantiseScreenStates &currState, const QuantiseScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillScreen(ELI_BLUE);
      tft.setTextColor(ELI_PINK, ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString("Quantise", 120, 26);
      tft.setTextDatum(TR_DATUM);
      iconXTurn.pushSprite(170,65);
      tft.drawString("Pull", 70, 70);
      iconYTurn.pushSprite(170,115);
      tft.drawString("Notes", 70, 120);
      // iconZTurn.pushSprite(170,165);
      // tft.drawString("Fine", 70, 170);

      tft.setTextFont(1);
      tft.setTextColor(TFT_SILVER, ELI_BLUE);
      tft.setTextDatum(CC_DATUM);
      iconXPush.pushSprite(75,205);
      tft.drawString("Tuning", 120,210);
      iconZPush.pushSprite(85,220);
      tft.drawString("Exit", 120,225);
    }
    if (fullRedraw || currState.pull != nextState.pull) {
      const int TLY=70;
      tft.fillRect(90,TLY-20,60,40, ELI_BLUE);
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.pull, 120, TLY);
    }
    if (fullRedraw || currState.notesperoct != nextState.notesperoct) {
      const int TLY=120;
      tft.fillRect(90,TLY-20,60,40, ELI_BLUE);
      tft.setTextColor(TFT_WHITE, ELI_BLUE);
      tft.setFreeFont(&FreeMonoBold18pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawNumber(nextState.notesperoct, 120, TLY);
    }
    // if (fullRedraw || currState.finetune != nextState.finetune) {
    //   const int TLY=170;
    //   tft.fillRect(90,TLY-20,60,40, ELI_BLUE);
    //   tft.setTextColor(TFT_WHITE, ELI_BLUE);
    //   tft.setFreeFont(&FreeMonoBold18pt7b);
    //   tft.setTextDatum(CC_DATUM);
    //   tft.drawNumber(nextState.finetune, 120, TLY);
    // }
  }

  void drawUtilityScreen(const UtilScreenStates &currState, const UtilScreenStates &nextState, const bool fullRedraw) {
    if (fullRedraw) {
      tft.fillRect(0,0,240,240,ELI_BLUE);
      tft.setFreeFont(&FreeMono9pt7b);
      tft.setTextDatum(CC_DATUM);
      tft.drawString("Utility", 120,20);
      tft.drawString("Save State", 120,100);
      tft.drawString("Exit", 120,140);

      iconZPush.pushSprite(114,82);
      iconXPush.pushSprite(114,122);


    }
  }


  displayStates currState, nextState;

};

