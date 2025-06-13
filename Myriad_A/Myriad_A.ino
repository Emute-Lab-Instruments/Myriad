/*
todo:
metaosc modes

make sure no -ve frequencies
control/constrain speed and depth individually for each mode


cfg
08:25:59.913 -> 1073872128
08:25:59.913 -> 1073741824
*/



#include <optional>
#include "displayPortal.h"
#include "myriad_pins.h"
// #include "myriad_setup.h"
// #include "pio_expdec.h"
#include "pios/pio_sq.h"
#include "pios/pio_pulse.h"
#include "pios/pio_expdec.h"
#include "pios/pio_bitbybit.h"

#include "smBitStreamOsc.h"
#include "oscillatorModels.hpp"
#include "bitstreams.hpp"
#include "metaOscs.hpp"
#include "debouncer.hpp"


#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

#include "MedianFilter.h"
#include "MAFilter.h"
#include "ResponsiveFilter.hpp"

#include "freqlookup.h"
#include "myriad_messages.h"
#include "SLIP.h"

#include "boids.h"


// #define FAST_MEM __not_in_flash("myriaddata")

// #define SINGLEOSCILLATOR


bool core1_separate_stack = true;

constexpr size_t N_OSCILLATORS=9;
constexpr size_t N_OSC_BANKS=3;

using portal = displayPortal<N_OSCILLATORS,N_OSC_BANKS,N_OSCILLATOR_MODELS>;

portal FAST_MEM display;

enum CONTROLMODES {OSCMODE, METAOSCMODE, CALIBRATEMODE, TUNINGMODE} controlMode = CONTROLMODES::OSCMODE;


#define RUN_OSCS
// oscModelPtr FAST_MEM currOscModelBank0;
std::array<oscModelPtr, 3> currOscModels;

// oscModelPtr FAST_MEM currOscModelBank0A;
// oscModelPtr FAST_MEM currOscModelBank0B;
// oscModelPtr FAST_MEM currOscModelBank0C;
volatile bool FAST_MEM oscsReadyToStart=false;
volatile bool FAST_MEM restartOscsFlag=false;

//a list of reference models for display data
std::array<oscModelPtr,N_OSCILLATOR_MODELS> FAST_MEM oscModelsDisplayRef;

metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscBlank = std::make_shared<metaOscNone<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscNN = std::make_shared<metaOscMLP<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscSines1 = std::make_shared<metaOscSines<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscSinesFMultiple1 = std::make_shared<metaOscSinesFMultiple<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaDrunkenWalkers1 = std::make_shared<metaDrunkenWalkers<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaLorenz1 = std::make_shared<metaLorenz<N_OSCILLATORS>>();


std::array<metaOscPtr<N_OSCILLATORS>, 6> __not_in_flash("mydata") metaOscsList = {metaOscBlank, metaLorenz1, metaOscSines1, metaOscSinesFMultiple1, metaOscNN, metaDrunkenWalkers1};

size_t currMetaMod = 0;

DEFINE_TIMING_SWAPBUFFERS(0)
DEFINE_TIMING_SWAPBUFFERS(1)
DEFINE_TIMING_SWAPBUFFERS(2)

uint32_t FAST_MEM smOsc0_dma_chan;
uint32_t FAST_MEM smOsc0_dma_chan_bit;

uint32_t FAST_MEM smOsc1_dma_chan;
uint32_t FAST_MEM smOsc1_dma_chan_bit;

uint32_t FAST_MEM smOsc2_dma_chan;
uint32_t FAST_MEM smOsc2_dma_chan_bit;

volatile bool FAST_MEM bufSent0 = false;
volatile bool FAST_MEM bufSent1 = false;
volatile bool FAST_MEM bufSent2 = false;

volatile size_t newBufRq0 =0;
volatile size_t bufCalcCount0=0;

/////////////////////////////////   IRQ 000000000000000000000000000000000000
void __isr dma_irh() {
  //TEST
  // Serial.println("dma");
  uint32_t triggered_channels = dma_hw->ints1;
  if (triggered_channels & smOsc0_dma_chan_bit) {
    dma_hw->ints1 = smOsc0_dma_chan_bit;  
    dma_hw->ch[smOsc0_dma_chan].al3_read_addr_trig = nextTimingBuffer0;
    bufSent0 = true;
    newBufRq0++;
  }
  else
  if (triggered_channels & smOsc1_dma_chan_bit) {
    dma_hw->ints1 = smOsc1_dma_chan_bit;
    dma_hw->ch[smOsc1_dma_chan].al3_read_addr_trig = nextTimingBuffer1;
    bufSent1 = true;
  }
  else
  if (triggered_channels & smOsc2_dma_chan_bit) {
    dma_hw->ints1 = smOsc2_dma_chan_bit;
    dma_hw->ch[smOsc2_dma_chan].al3_read_addr_trig = nextTimingBuffer2;
    bufSent2 = true;
  }
}

smBitStreamOsc FAST_MEM smOsc0;
smBitStreamOsc FAST_MEM smOsc1;
smBitStreamOsc FAST_MEM smOsc2;

volatile bool oscsRunning = false;

void startOscBankA() {

  Serial.println("StartoscbankA");

  //reset the pio block
  pio_clear_instruction_memory(pio1);
  uint programOffset = currOscModels[0]->loadProg(pio1);
  pio_sm_config baseConfig = currOscModels[0]->getBaseConfig(programOffset);
  Serial.printf("Offset %d\n", programOffset);

  smOsc0_dma_chan = smOsc0.init(pio1, 0, OSC7_PIN, programOffset, baseConfig, nextTimingBuffer0, dma_irh, clockdiv, currOscModels[0]->loopLength, DMA_IRQ_1);
  // Serial.println("init");
  // if (smOsc0_dma_chan < 0) {
    // Serial.println("dma chan allocation error");
  // }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();
  
  #ifndef SINGLEOSCILLATOR

  // smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, nextTimingBuffer1, dma_irh, clockdiv, currOscModelBank0B->loopLength, DMA_IRQ_1);
  smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, baseConfig, nextTimingBuffer1, dma_irh, clockdiv, currOscModels[1]->loopLength, DMA_IRQ_1);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  // smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, nextTimingBuffer2, dma_irh, clockdiv, currOscModelBank0C->loopLength, DMA_IRQ_1);
  smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, baseConfig, nextTimingBuffer2, dma_irh, clockdiv, currOscModels[2]->loopLength, DMA_IRQ_1);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();
  // Serial.println("started");
#endif

  oscsRunning = true;
}

void stopOscBankA() {
  oscsRunning = false;
  smOsc0.stop();
#ifndef SINGLEOSCILLATOR
  smOsc1.stop();
  smOsc2.stop();
#endif
}

#define SCREEN_WIDTH tft.width()    //
#define SCREEN_HEIGHT tft.height()  // Taille de l'écran

#define ENCODER1_A_PIN 9
#define ENCODER1_B_PIN 14
#define ENCODER1_SWITCH 8

#define ENCODER2_A_PIN 16
#define ENCODER2_B_PIN 21
#define ENCODER2_SWITCH 15

#define ENCODER3_A_PIN 24
#define ENCODER3_B_PIN 25
#define ENCODER3_SWITCH 23


namespace controls {
  static int FAST_MEM encoderValues[3] = {0,0,0};
  static int FAST_MEM encoderAltValues[3] = {0,0,0};
  static bool FAST_MEM encoderSwitches[3] = {0,0,0};
  static bool FAST_MEM calibrateButton=0;
};

boidsSim boids;

// MovingAverageFilter<float> FAST_MEM adcFilters[4];
// MedianFilter<float> FAST_MEM adcMedians[4];
ResponsiveFilter FAST_MEM adcRsFilters[4];

static float __not_in_flash("mydata") controlValues[4] = {0,0,0,0};
// static size_t __not_in_flash("mydata") controlValueMedians[4] = {0,0,0,0};

static uint16_t __not_in_flash("mydata") capture_buf[16] __attribute__((aligned(2048)));

static float __not_in_flash("mydata") octave0=1;
static float __not_in_flash("mydata") octave1=1;
static float __not_in_flash("mydata") octave2=1;
static float __not_in_flash("mydata") octave3=1;
static float __not_in_flash("mydata") octave4=1;
static float __not_in_flash("mydata") octave5=1;
static float __not_in_flash("mydata") octave6=1;
static float __not_in_flash("mydata") octave7=1;
static float __not_in_flash("mydata") octave8=1;
static std::array<float, N_OSCILLATORS> __not_in_flash("mydata") octaves = {1,1,1, 1,1,1, 1,1,1};

int __not_in_flash("mydata") oscTypeBank0=0;
int __not_in_flash("mydata") oscTypeBank1=0;
int __not_in_flash("mydata") oscTypeBank2=0;

static FAST_MEM std::array<size_t,4> adcMins{50,50,50,50};
static FAST_MEM std::array<size_t,4> adcMaxs{4025,4025,4025,4025};
static FAST_MEM std::array<size_t,4> adcRanges;
static FAST_MEM float courseTuning=0;
static FAST_MEM float fineTuning=0;
static FAST_MEM int tuningOctaves=0;
static FAST_MEM int tuningMillis=0;

void setup_adcs() {
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  adc_gpio_init(29);
  adc_set_round_robin(15);
  adc_fifo_setup(
    true,   // Write each completed conversion to the sample FIFO
    true,   // Enable DMA data request (DREQ)
    1,      // DREQ (and IRQ) asserted when at least 1 sample present
    false,  // We won't see the ERR bit because of 8 bit reads; disable.
    false   // Shift each sample to 8 bits when pushing to FIFO
  );

  // Divisor of 0 -> full speed. Free-running capture with the divider is
  // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
  // cycles (div not necessarily an integer). Each conversion takes 96
  // cycles, so in general you want a divider of 0 (hold down the button
  // continuously) or > 95 (take samples less frequently than 96 cycle
  // intervals). This is all timed by the 48 MHz ADC clock.
  // adc_set_clkdiv(96 * 512);
  adc_set_clkdiv(960);
  // adc_set_clkdiv(0);

  // printf("Arming DMA\n");
  // sleep_ms(1);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_ring(&cfg, true, 3);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  dma_channel_configure(dma_chan, &cfg,
                        capture_buf,    // dst
                        &adc_hw->fifo,  // src
                        -1,             // transfer count
                        true            // start immediately
  );

  adc_select_input(0);
  adc_run(true);
}


void __force_inline __not_in_flash_func(sendToMyriadB) (uint8_t msgType, float value) {
  static uint8_t __not_in_flash("sendToMyriadData") slipBuffer[64];
  spiMessage msg {msgType, value};
  unsigned int slipSize = SLIP::encode(reinterpret_cast<uint8_t*>(&msg), sizeof(spiMessage), &slipBuffer[0]);
  int res = Serial1.write(reinterpret_cast<uint8_t*>(&slipBuffer), slipSize);
}

inline float __not_in_flash_func(adcMap)(const size_t adcIndex) {
  return std::max(0.f,(controlValues[adcIndex] - (adcMins[adcIndex]))) / adcRanges[adcIndex];
}


size_t msgCt=0;
bool __not_in_flash_func(adcProcessor)(__unused struct repeating_timer *t) {
  static size_t __not_in_flash("acdData") lastOctaveIdx = 0;
  static size_t __not_in_flash("acdData") adcCount = 0;
  static size_t __not_in_flash("acdData") adcAccumulator[4] = {0,0,0,0};
  adcAccumulator[0] += capture_buf[0];
  adcAccumulator[1] += capture_buf[1];
  adcAccumulator[2] += capture_buf[2];
  adcAccumulator[3] += capture_buf[3];
  adcCount++;
  if (adcCount == 256U) {
    adcCount=0;
    adcAccumulator[0] = adcAccumulator[0] >> 8U;
    adcAccumulator[1] = adcAccumulator[1] >> 8U;
    adcAccumulator[2] = adcAccumulator[2] >> 8U;
    adcAccumulator[3] = adcAccumulator[3] >> 8U;
  }else{
    return true;
  }

  controlValues[0] = adcRsFilters[0].process(adcAccumulator[0]);
  controlValues[1] = adcRsFilters[1].process(adcAccumulator[1]);
  controlValues[2] = adcRsFilters[2].process(adcAccumulator[2]);
  controlValues[3] = adcRsFilters[3].process(adcAccumulator[3]);

  if (controlMode == CONTROLMODES::CALIBRATEMODE) {
    for(size_t i=0; i < 4; i++) {
      if (controlValues[i] < adcMins[i]) {
        adcMins[i] = controlValues[i];
      }
      if (controlValues[i] > adcMaxs[i]) {
        adcMaxs[i] = controlValues[i];
      }
      adcRanges[i] = adcMaxs[i] - adcMins[i];
    }
    display.setCalibADCValues(adcAccumulator[0], adcAccumulator[1], adcAccumulator[2], adcAccumulator[3]);
    display.setCalibADCFiltValues(controlValues[0], controlValues[1], controlValues[2], controlValues[3]);
    display.setCalibADCMinMaxValues(adcMins.data(), adcMaxs.data());
  }
  

  // size_t octaveIdx = static_cast<size_t>(controlValues[3]) >> 8;  // div by 256 -> 16 divisions
  size_t octaveIdx = static_cast<size_t>(controlValues[3]) >> 8;  // div by 256 -> 16 divisions
  if (octaveIdx != lastOctaveIdx) {
    lastOctaveIdx = octaveIdx;
    switch(octaveIdx) {
      case 0:   {octaves = {1, 1,  1,    1,  1,  1,    1,  1,  1}; break;}
      case 1:   {octaves = {0.5, 1,  1,    0.5,  1,  1,    0.5,  1,  1}; break;}
      case 2:   {octaves = {0.5, 0.5,  1,    0.5,  0.5,  1,    0.5,  0.5,  1}; break;}
      case 3:   {octaves = {0.25, 0.5,  1,    0.25,  0.5,  1,    0.25,  0.5,  1}; break;}

      case 4:   {octaves = {0.25, 0.5,  2,    0.25,  0.5,  2,    0.25,  0.5,  2}; break;}
      case 5:   {octaves = {0.25, 0.5,  4,    0.25,  0.5,  4,    0.25,  0.5,  4}; break;}
      case 6:   {octaves = {0.25, 0.25,  4,    0.25,  0.25,  4,    0.25,  0.25,  4}; break;}
      case 7:   {octaves = {0.5, 0.25,  4,    0.5,  0.25,  4,    0.5,  0.25,  4}; break;}

      case 8:   {octaves = {0.25, 2,  4,    0.25,  2,  4,    0.25,  2,  4}; break;}
      case 9:   {octaves = {0.25, 4,  4,    0.25,  4,  4,    0.25,  4,  4}; break;}
      case 10:  {octaves = {0.5, 4,  4,    0.5,  4,  4,    0.25,  4,  4}; break;}
      case 11:  {octaves = {1, 4,  4,    1,  4,  4,    1,  4,  4}; break;}

      case 12:  {octaves = {2, 4,  4,    2,  4,  4,    2,  4,  4}; break;}
      case 13:  {octaves = {2, 2,  4,    2,  2,  4,    2,  2,  4}; break;}
      case 14:  {octaves = {1, 2,  4,    1,  2,  4,    1,  2,  4}; break;}
      case 15:  {octaves = {1, 2,  2,    1,  2,  2,    1,  2,  2}; break;}
      default:;
    }
  }

  // float acc =  1.f - (adcMap(2) * 0.02f);

  // float new_wavelen0 = freqtable[std::lround(controlValues[0])];
  // constexpr float wavelen20hz = sampleClock/20.f;
  constexpr float wavelen20hz = sampleClock/20.f;
  constexpr float wavelenTesthz = sampleClock/0.01f;
  float pitchCV = adcMap(0);
  float freq = powf(2.f, (pitchCV * 10.f) + ((courseTuning + fineTuning) * 10.f)); // 10 octaves
  float new_wavelen0 = 1.0f/freq * wavelen20hz ; 

  // float new_wavelen0 = 1.0f/freq * wavelenTesthz ; 

  float detune = (adcMap(1) * 0.01f) * new_wavelen0;
  float acc = 1.f - (adcMap(1) * 0.02f);
  // new_wavelen0 = 200000;
  // detune=0;
  // acc=1.f;

  float new_wavelen1 = (new_wavelen0 - detune) * acc;
  float new_wavelen2 = (new_wavelen1 - detune) * acc;
  float new_wavelen3 = (new_wavelen2 - detune) * acc;
  float new_wavelen4 = (new_wavelen3 - detune) * acc;
  float new_wavelen5 = (new_wavelen4 - detune) * acc;


  float new_wavelen6 = (new_wavelen5 - detune) * acc;
  float new_wavelen7 = (new_wavelen6 - detune) * acc;
  float new_wavelen8 = (new_wavelen7 - detune) * acc;


  new_wavelen0 = new_wavelen0 * octaves[0];
  new_wavelen1 = new_wavelen1 * octaves[1];
  new_wavelen2 = new_wavelen2 * octaves[2];
  new_wavelen3 = new_wavelen3 * octaves[3];
  new_wavelen4 = new_wavelen4 * octaves[4];
  new_wavelen5 = new_wavelen5 * octaves[5];
  new_wavelen6 = new_wavelen6 * octaves[6];
  new_wavelen7 = new_wavelen7 * octaves[7];
  new_wavelen8 = new_wavelen8 * octaves[8];

  float ctrlVal0 = adcMap(2);
  float ctrlVal1 = ctrlVal0;
  float ctrlVal2 = ctrlVal0;
  float ctrlVal3 = ctrlVal0;
  float ctrlVal4 = ctrlVal0;
  float ctrlVal5 = ctrlVal0;
  float ctrlVal6 = ctrlVal0;
  float ctrlVal7 = ctrlVal0;
  float ctrlVal8 = ctrlVal0;

  // sendToMyriadB(messageTypes::CTRL, ctrlVal0);


  if (currMetaMod > 0) {
    auto metamods = metaOscsList.at(currMetaMod)->update(controlValues);

    if (modTarget == MODTARGETS::PITCH_AND_EPSILON || modTarget == MODTARGETS::PITCH ) {
        new_wavelen0 *= (1.f + (metamods[0]));
        new_wavelen1 *= (1.f + (metamods[1]));
        new_wavelen2 *= (1.f + (metamods[2]));
        new_wavelen3 *= (1.f + (metamods[3]));
        new_wavelen4 *= (1.f + (metamods[4]));
        new_wavelen5 *= (1.f + (metamods[5]));
        new_wavelen6 *= (1.f + (metamods[6]));
        new_wavelen7 *= (1.f + (metamods[7]));
        new_wavelen8 *= (1.f + (metamods[8]));
    }

    if (modTarget == MODTARGETS::PITCH_AND_EPSILON || modTarget == MODTARGETS::EPSILON ) {
        ctrlVal0 *= (1.f + (metamods[0] * 5.f));
        ctrlVal1 *= (1.f + (metamods[1] * 5.f));
        ctrlVal2 *= (1.f + (metamods[2] * 5.f));
        ctrlVal3 *= (1.f + (metamods[3] * 5.f));
        ctrlVal4 *= (1.f + (metamods[4] * 5.f));
        ctrlVal5 *= (1.f + (metamods[5] * 5.f));
        ctrlVal6 *= (1.f + (metamods[6] * 5.f));
        ctrlVal7 *= (1.f + (metamods[7] * 5.f));
        ctrlVal8 *= (1.f + (metamods[8] * 5.f));
    }
    
    // for(int i=0; i < 6; i++) {
    //   Serial.print(metamods[i]);
    //   Serial.print("\t");
    // }
    // Serial.println();

    // if (msgCt == 80) {
    //   Serial.println(metamods[0]);
    // }
  }

  if (msgCt == 100) {
    // Serial.printf("%f \n", controlValues[0]);
    Serial.printf("%f %f %f %f\n",adcMap(0), freq, new_wavelen0, wavelen20hz);
    msgCt=0;
  }
  msgCt++;

  //send crtl vals
  sendToMyriadB(messageTypes::CTRL0, ctrlVal3);
  sendToMyriadB(messageTypes::CTRL1, ctrlVal4);
  sendToMyriadB(messageTypes::CTRL2, ctrlVal5);
  sendToMyriadB(messageTypes::CTRL3, ctrlVal6);
  sendToMyriadB(messageTypes::CTRL4, ctrlVal7);
  sendToMyriadB(messageTypes::CTRL5, ctrlVal8);

  currOscModels[0]->ctrl(ctrlVal0);
  currOscModels[1]->ctrl(ctrlVal1);
  currOscModels[2]->ctrl(ctrlVal2);

  sendToMyriadB(messageTypes::WAVELEN0, new_wavelen3);
  sendToMyriadB(messageTypes::WAVELEN1, new_wavelen4);
  sendToMyriadB(messageTypes::WAVELEN2, new_wavelen5);
  sendToMyriadB(messageTypes::WAVELEN3, new_wavelen6);
  sendToMyriadB(messageTypes::WAVELEN4, new_wavelen7);
  sendToMyriadB(messageTypes::WAVELEN5, new_wavelen8);

  // updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModels[0], new_wavelen0);
  // updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModels[1], new_wavelen1);
  // updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModels[2], new_wavelen2);
  currOscModels[0]->wavelen = new_wavelen0;
  currOscModels[0]->newFreq = true;
  
  currOscModels[1]->wavelen = new_wavelen1;
  currOscModels[1]->newFreq = true;

  currOscModels[2]->wavelen = new_wavelen2;
  currOscModels[2]->newFreq = true;

  oscsReadyToStart = true;

  display.setDisplayWavelengths({new_wavelen0,new_wavelen1,new_wavelen2,
                                  new_wavelen3,new_wavelen4,new_wavelen5,
                                  new_wavelen6,new_wavelen7,new_wavelen8 });

  return true;
}


inline bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
  display.update();
  return true;
}

int8_t __not_in_flash_func(read_rotary)(uint8_t &prevNextCode, uint16_t &store, int a_pin, int b_pin) {
  static int8_t FAST_MEM rot_enc_table[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

  prevNextCode <<= 2;
  if (digitalRead(b_pin)) prevNextCode |= 0x02;
  if (digitalRead(a_pin)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;
  // Serial.println(prevNextCode);

  // If valid then store as 16 bit data.
  if (rot_enc_table[prevNextCode]) {
    store <<= 4;
    store |= prevNextCode;
    if ((store & 0xff) == 0x2b) return -1;
    if ((store & 0xff) == 0x17) return 1;
  }
  return 0;
}

static uint8_t FAST_MEM enc1Code = 0;
static uint16_t FAST_MEM enc1Store = 0;

static uint8_t FAST_MEM enc2Code = 0;
static uint16_t FAST_MEM enc2Store = 0;
static uint8_t FAST_MEM enc3Code = 0;
static uint16_t FAST_MEM enc3Store = 0;

void __force_inline calculateOscBuffers() {
  if ((currOscModels[0]->updateBufferInSyncWithDMA && bufSent0) || (!currOscModels[0]->updateBufferInSyncWithDMA && currOscModels[0]->newFreq)) {
    currOscModels[0]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModels[0], currOscModels[0]->wavelen);
    bufSent0 = false;
    bufCalcCount0++;
  }
  if ((currOscModels[1]->updateBufferInSyncWithDMA && bufSent1) || (!currOscModels[1]->updateBufferInSyncWithDMA && currOscModels[1]->newFreq)) {
    currOscModels[1]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModels[1], currOscModels[1]->wavelen);
    bufSent1 = false;
  }
  if ((currOscModels[2]->updateBufferInSyncWithDMA && bufSent2) || (!currOscModels[2]->updateBufferInSyncWithDMA && currOscModels[2]->newFreq)) {
    currOscModels[2]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModels[2], currOscModels[2]->wavelen);
    bufSent2 = false;
  }
}

void __not_in_flash_func(assignOscModels)(const size_t modelIdx) {
  Serial.printf("assign %d\n", modelIdx);

  for(auto &model: currOscModels) {
    model = oscModelFactories[modelIdx](); 

  }
}

bool __not_in_flash_func(updateOscBank)(int &currOscBank, int change, std::optional<messageTypes> OSCBANKMSG = std::nullopt) {
  int newOscTypeBank = currOscBank + change;
  bool changed=false;
  //clip
  newOscTypeBank = max(0, newOscTypeBank);
  newOscTypeBank = min(N_OSCILLATOR_MODELS-1, newOscTypeBank);
  if (newOscTypeBank != currOscBank) {
    //send
    currOscBank = newOscTypeBank;
    changed=true;
    if (OSCBANKMSG.has_value()) {
      sendToMyriadB(OSCBANKMSG.value(), currOscBank);
    }else{
      //assume bank C
      Serial.println("bank change");
      // // Serial.println(decodeMsg.value);
      stopOscBankA();

      assignOscModels(currOscBank);

      //refill from new oscillator
      //trigger buffer refills
      currOscModels[0]->newFreq=true;
      currOscModels[1]->newFreq=true;
      currOscModels[2]->newFreq=true;

      calculateOscBuffers();

      startOscBankA();

    }
    Serial.println(currOscBank);  
  } 
  return changed;
}

void __not_in_flash_func(updateMetaOscMode)(size_t &currMetaModMode, const int change) {
  int newMetaModMode = static_cast<int>(currMetaModMode) + change;
  //clip
  newMetaModMode = max(0, newMetaModMode);
  newMetaModMode = min(metaOscsList.size()-1, newMetaModMode);

  if (newMetaModMode != currMetaModMode) {
    //send
    currMetaModMode = newMetaModMode;
    display.setMetaOsc(currMetaModMode, metaOscsList[currMetaModMode]);

    //assume bank C
    Serial.printf("meta mod mode change %d\n", newMetaModMode);
  } 

}

void updateTuning() {
  courseTuning = tuningOctaves *  0.1;
  fineTuning = tuningMillis * 0.001 * 0.1;
  display.setTuning(tuningOctaves, tuningMillis);
  Serial.printf("%f %f\n", courseTuning, fineTuning);
}

//centre
void __isr encoder1_callback() {
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  switch(controlMode) {
    case CONTROLMODES::METAOSCMODE:
    {
      controls::encoderAltValues[0] += change;
      updateMetaOscMode(currMetaMod, change);
      break;
    }
    case CONTROLMODES::OSCMODE:
    {
      controls::encoderValues[0] += change;
      bool changed = updateOscBank(oscTypeBank1, change, messageTypes::BANK1);
      if (changed) {
        display.setOscBankModel(1, oscTypeBank1);
        Serial.printf("model %d\n", oscTypeBank1);
      }
      break;
    }
    case CONTROLMODES::CALIBRATEMODE:
    {
      break;
    }
    case CONTROLMODES::TUNINGMODE:
    {
      break;
    }
  }
  // if (controlMode == CONTROLMODES::METAOSCMODE) {
  //   controls::encoderAltValues[0] += change;
  //   updateMetaOscMode(currMetaMod, change);
  // }else{
  //   controls::encoderValues[0] += change;
  //   bool changed = updateOscBank(oscTypeBank1, change, messageTypes::BANK1);
  //   if (changed) {
  //     display.setOscBankModel(1, oscTypeBank1);
  //     Serial.printf("model %d\n", oscTypeBank1);
  //   }
  // }
}

//left
void __isr encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  switch(controlMode) {
    case CONTROLMODES::METAOSCMODE:
    {
      controls::encoderAltValues[1] += change;
      if (currMetaMod > 0) {
        metaOscsList.at(currMetaMod)->setSpeed(change);
        display.setMetaModSpeed(metaOscsList.at(currMetaMod)->modspeed.getNormalisedValue());
      }
      break;
    }
    case CONTROLMODES::OSCMODE:
    {
      controls::encoderValues[1] += change;
      bool changed = updateOscBank(oscTypeBank0, change, messageTypes::BANK0);
      if (changed) {
        display.setOscBankModel(0, oscTypeBank0);
        Serial.printf("enc 2 model %d\n", oscTypeBank0);

      }  
      break;
    }
    case CONTROLMODES::CALIBRATEMODE:
    {
      break;
    }
    case CONTROLMODES::TUNINGMODE:
    {
      tuningOctaves += change;
      updateTuning();
      break;
    }
  }

  // if (controlMode == CONTROLMODES::METAOSCMODE) {
  //   controls::encoderAltValues[1] += change;
  //   if (currMetaMod > 0) {
  //     metaOscsList.at(currMetaMod)->setSpeed(change);
  //     display.setMetaModSpeed(metaOscsList.at(currMetaMod)->modspeed.getNormalisedValue());
  //   }
  // }else{
  //   controls::encoderValues[1] += change;
  //   bool changed = updateOscBank(oscTypeBank0, change, messageTypes::BANK0);
  //   if (changed) {
  //     display.setOscBankModel(0, oscTypeBank0);
  //     Serial.printf("enc 2 model %d\n", oscTypeBank0);

  //   }  
  // }

}

//right
void __isr encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
  switch(controlMode) {
    case CONTROLMODES::METAOSCMODE:
    {
      controls::encoderAltValues[2] += change;
      if (currMetaMod > 0) {
        metaOscsList.at(currMetaMod)->setDepth(change);
        display.setMetaModDepth(metaOscsList.at(currMetaMod)->moddepth.getNormalisedValue());
      }
      break;
    }
    case CONTROLMODES::OSCMODE:
    {
      bool changed = updateOscBank(oscTypeBank2, change, std::nullopt);
      if (changed) {
        display.setOscBankModel(2, oscTypeBank2);

      }
      break;
    }
    case CONTROLMODES::CALIBRATEMODE:
    {
      break;
    }
    case CONTROLMODES::TUNINGMODE:
    {
      tuningMillis += change;
      updateTuning();
      break;
    }
  }

  // if (controlMode == CONTROLMODES::METAOSCMODE) {
  //   controls::encoderAltValues[2] += change;
  //   if (currMetaMod > 0) {
  //     metaOscsList.at(currMetaMod)->setDepth(change);
  //     display.setMetaModDepth(metaOscsList.at(currMetaMod)->moddepth.getNormalisedValue());
  //   }
  // }else{    controls::encoderValues[2] += change;
  //   bool changed = updateOscBank(oscTypeBank2, change, std::nullopt);
  //   if (changed) {
  //     display.setOscBankModel(2, oscTypeBank2);

  //   }
  // }
}

debouncer FAST_MEM enc1Debouncer;
debouncer FAST_MEM enc2Debouncer;
debouncer FAST_MEM enc3Debouncer;
debouncer FAST_MEM calibrateButtonDebouncer;

void switchToOSCMode() {
  controlMode = CONTROLMODES::OSCMODE;
  display.setScreen(portal::SCREENMODES::OSCBANKS);  
}

//central
void __isr encoder1_switch_callback() {
  //gpio pull-up, so the value is inverted
  controls::encoderSwitches[0] = 1 - enc1Debouncer.debounce(ENCODER1_SWITCH);  
  // Serial.printf("enc %d\n", controls::encoderSwitches[0]);
  if (controls::encoderSwitches[0]) {
    switch(controlMode) {
      case CONTROLMODES::METAOSCMODE:
      {
        switchToOSCMode();
        break;
      }
      case CONTROLMODES::OSCMODE:
      {
        controlMode = CONTROLMODES::METAOSCMODE;
        display.setScreen(portal::SCREENMODES::METAOSCVIS);
        break;
      }
      case CONTROLMODES::CALIBRATEMODE:
      {
        break;
      }
      case CONTROLMODES::TUNINGMODE:
      {
        break;
      }
    }

    // if (controlMode == CONTROLMODES::OSCMODE) {
    //   controlMode = CONTROLMODES::METAOSCMODE;
    //   // display.toggleScreen();
    //   display.setScreen(portal::SCREENMODES::METAOSCVIS);
    // }else{
    //   controlMode = CONTROLMODES::OSCMODE;
    //   display.setScreen(portal::SCREENMODES::OSCBANKS);
    // }
  }
}

//left
void __isr encoder2_switch_callback() {
  controls::encoderSwitches[1] = 1 - enc2Debouncer.debounce(ENCODER2_SWITCH);  
}

//right
void __isr encoder3_switch_callback() {
  controls::encoderSwitches[2] = 1 - enc3Debouncer.debounce(ENCODER3_SWITCH); 
  if (controls::encoderSwitches[2]) {
    switch(controlMode) {
      case CONTROLMODES::METAOSCMODE:
      {
        switch(modTarget) {
          case MODTARGETS::PITCH: {
            modTarget = MODTARGETS::EPSILON;
            break;
          }
          case MODTARGETS::EPSILON: {
            modTarget = MODTARGETS::PITCH_AND_EPSILON;
            break;
          }
          case MODTARGETS::PITCH_AND_EPSILON: {
            modTarget = MODTARGETS::PITCH;
            break;
          }
        }
        display.setModTarget(modTarget);
        break;
      }
      case CONTROLMODES::OSCMODE:
      {
        controlMode = CONTROLMODES::TUNINGMODE;
        display.setScreen(portal::SCREENMODES::TUNING);
        display.setTuning(tuningOctaves, tuningMillis);
        break;
      }
      case CONTROLMODES::CALIBRATEMODE:
      {
        break;
      }
      case CONTROLMODES::TUNINGMODE:
      {
        controlMode = CONTROLMODES::OSCMODE;
        display.setScreen(portal::SCREENMODES::OSCBANKS);
        break;
      }
    }
  }
 
}

void calibrate_button_callback() {
  controls::calibrateButton = 1 - calibrateButtonDebouncer.debounce(CALIBRATE_BUTTON); 

  if (controls::calibrateButton == 1) {
    if (controlMode != CONTROLMODES::CALIBRATEMODE) {
      controlMode = CONTROLMODES::CALIBRATEMODE;
      display.setScreen(portal::SCREENMODES::CALIBRATE);
    }else{
      switchToOSCMode();
    }

  }
}

struct repeating_timer timerAdcProcessor;
struct repeating_timer timerDisplay;
struct repeating_timer timerFreqReceiver;



void setup() {


  //USB Serial
  Serial.begin();
  // while(!Serial) {}
  //get vis data from osc models and store in memory
  // for(size_t i=0; i < oscModels.size(); i++) {
  //   display.oscVisDataPtrs.push_back(&oscModels.at(i)->visData);
  // }


  //create reference models lists to keep display data
  for (size_t i=0; i < oscModelsDisplayRef.size(); i++) {
    oscModelsDisplayRef[i] = oscModelFactories[i]();
    display.oscvis.push_back(&oscModelsDisplayRef.at(i)->vis);
  }

  tft.begin();
  tft.setRotation(3);
  tft.setFreeFont(&FreeSans18pt7b);
  display.setScreen(portal::SCREENMODES::OSCBANKS);
  display.setMetaOsc(0, metaOscsList[0]);
  display.setModTarget(MODTARGETS::PITCH);

  for(size_t i=0; i < 4; i++) {
    adcRanges[i] = adcMaxs[i] - adcMins[i];
  }

  // comms to Myriad B
  Serial1.setRX(13);
  Serial1.setTX(12);
  Serial1.begin(115200);

  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  setup_adcs();  

  //Encoders
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_SWITCH, INPUT_PULLUP);
  pinMode(ENCODER3_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_SWITCH, INPUT_PULLUP);
  pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
  

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B_PIN), encoder1_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B_PIN), encoder2_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A_PIN), encoder3_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_B_PIN), encoder3_callback,
                    CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_SWITCH), encoder1_switch_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_SWITCH), encoder2_switch_callback,
                    CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_SWITCH), encoder3_switch_callback,
                    CHANGE);

  attachInterrupt(digitalPinToInterrupt(CALIBRATE_BUTTON), calibrate_button_callback,
                    CHANGE);

  add_repeating_timer_us(20, adcProcessor, NULL, &timerAdcProcessor);
  add_repeating_timer_ms(-39, displayUpdate, NULL, &timerDisplay);


}


int count=0;

void loop() {
  // Serial.println();
  // delay(1);
  // Serial.println("Hello from Myriad A");

  // __wfi();
  delay(1);
}


void setup1() {
  assignOscModels(0);

#ifdef RUN_OSCS
//wait for the first ADC readings
  while(!oscsReadyToStart) {
    ;
  }
  Serial.println("Start");
  delay(10);
  startOscBankA();
  delay(10);
  Serial.println("Started");
#endif

}


void loop1() {
  if (oscsRunning) {
    calculateOscBuffers();
  }
}
