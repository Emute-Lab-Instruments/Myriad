/*
todo:
metaosc modes

make sure no -ve frequencies
control/constrain speed and depth individually for each mode



*/



#include <optional>
#include "displayPortal.h"
#include "myriad_pins.h"
// #include "myriad_setup.h"
// #include "pio_expdec.h"
#include "pios/pio_sq.h"

#include "smBitStreamOsc.h"
#include "oscillatorModels.hpp"
#include "bitstreams.hpp"
#include "metaOscs.hpp"


#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

#include "MedianFilter.h"
#include "MAFilter.h"

#include "freqlookup.h"
#include "myriad_messages.h"
#include "SLIP.h"

#include "boids.h"

#define FAST_MEM __not_in_flash("mydata")

bool core1_separate_stack = true;

constexpr size_t N_OSCILLATORS=9;
constexpr size_t N_OSC_BANKS=3;

displayPortal<N_OSCILLATORS, N_OSC_BANKS> FAST_MEM display;

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


// enum METAMODMODES {NONE=0, MLP1, SINES, METAMODLENGTH};
// constexpr int MAXMETAMODMODE = static_cast<int>(METAMODMODES::METAMODLENGTH);


metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscBlank = std::make_shared<metaOscNone<N_OSCILLATORS>>();

metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscNN = std::make_shared<metaOscMLP<N_OSCILLATORS>>();

metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscSines1 = std::make_shared<metaOscSines<N_OSCILLATORS>>();

std::array<metaOscPtr<N_OSCILLATORS>, 3> metaOscsList = {metaOscBlank, metaOscSines1, metaOscNN};

size_t currMetaMod = 0;

// metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") currMetaMod;

// METAMODMODES FAST_MEM metaModMode = METAMODMODES::NONE;


DEFINE_TIMING_SWAPBUFFERS(0)
DEFINE_TIMING_SWAPBUFFERS(1)
DEFINE_TIMING_SWAPBUFFERS(2)

uint32_t FAST_MEM smOsc0_dma_chan;
uint32_t FAST_MEM smOsc0_dma_chan_bit;

uint32_t FAST_MEM smOsc1_dma_chan;
uint32_t FAST_MEM smOsc1_dma_chan_bit;

uint32_t FAST_MEM smOsc2_dma_chan;
uint32_t FAST_MEM smOsc2_dma_chan_bit;


/////////////////////////////////   IRQ 000000000000000000000000000000000000
void __isr dma_irh() {
  //TEST
  // Serial.println("dma");
  uint32_t triggered_channels = dma_hw->ints1;
  if (triggered_channels & smOsc0_dma_chan_bit) {
    dma_hw->ints1 = smOsc0_dma_chan_bit;  
    dma_hw->ch[smOsc0_dma_chan].al3_read_addr_trig = nextTimingBuffer0;
  }
  else
  if (triggered_channels & smOsc1_dma_chan_bit) {
    dma_hw->ints1 = smOsc1_dma_chan_bit;
    dma_hw->ch[smOsc1_dma_chan].al3_read_addr_trig = nextTimingBuffer1;
  }
  else
  if (triggered_channels & smOsc2_dma_chan_bit) {
    dma_hw->ints1 = smOsc2_dma_chan_bit;
    dma_hw->ch[smOsc2_dma_chan].al3_read_addr_trig = nextTimingBuffer2;
  }
}

smBitStreamOsc FAST_MEM smOsc0;
smBitStreamOsc FAST_MEM smOsc1;
smBitStreamOsc FAST_MEM smOsc2;

void startOscBankA() {

  Serial.println("StartoscbankA");

  pio_clear_instruction_memory(pio1);
  // uint programOffset = currOscModelBank0A->loadProg(pio1);
  uint programOffset = currOscModels[0]->loadProg(pio1);
  // updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModelBank0, 50000);

  // smOsc0_dma_chan = smOsc0.init(pio1, 0, OSC7_PIN, programOffset, nextTimingBuffer0, dma_irh, clockdiv, currOscModelBank0A->loopLength, DMA_IRQ_1);
  smOsc0_dma_chan = smOsc0.init(pio1, 0, OSC7_PIN, programOffset, nextTimingBuffer0, dma_irh, clockdiv, currOscModels[0]->loopLength, DMA_IRQ_1);
  // Serial.println("init");
  // if (smOsc0_dma_chan < 0) {
    // Serial.println("dma chan allocation error");
  // }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();

  // smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, nextTimingBuffer1, dma_irh, clockdiv, currOscModelBank0B->loopLength, DMA_IRQ_1);
  smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, nextTimingBuffer1, dma_irh, clockdiv, currOscModels[1]->loopLength, DMA_IRQ_1);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  // smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, nextTimingBuffer2, dma_irh, clockdiv, currOscModelBank0C->loopLength, DMA_IRQ_1);
  smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, nextTimingBuffer2, dma_irh, clockdiv, currOscModels[2]->loopLength, DMA_IRQ_1);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();
  // Serial.println("started");
}

void stopOscBankA() {
  smOsc0.stop();
  smOsc1.stop();
  smOsc2.stop();
}




// bool core1_separate_stack = true;


// const uint16_t MAX_ITERATION = 300;  // Nombre de couleurs

// const size_t BUF_LEN = 32;
// uint8_t spi_out_buf[BUF_LEN];


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

// SerialPIO uartOut(13, SerialPIO::NOPIN, 64);


namespace controls {
  static int FAST_MEM encoderValues[3] = {0,0,0};
  static int FAST_MEM encoderAltValues[3] = {0,0,0};
  static bool FAST_MEM encoderSwitches[3] = {0,0,0};
};

boidsSim boids;

MovingAverageFilter<float> adcFilters[4];
static float __not_in_flash("mydata") controlValues[4] = {0,0,0,0};


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

static FAST_MEM std::array<size_t,4> adcMins{22,22,22,22};
static FAST_MEM std::array<size_t,4> adcMaxs{4085,4085,4085,4085};
std::array<size_t,4> adcRanges;

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
  adc_set_clkdiv(96 * 128);

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

  // printf("Starting capture\n");
  adc_select_input(0);
  adc_run(true);
}


inline void __not_in_flash_func(sendToMyriadB) (uint8_t msgType, float value) {
  static uint8_t __not_in_flash("sendToMyriadData") slipBuffer[64];
  spiMessage msg {msgType, value};
  unsigned int slipSize = SLIP::encode(reinterpret_cast<uint8_t*>(&msg), sizeof(spiMessage), &slipBuffer[0]);
  int res = Serial1.write(reinterpret_cast<uint8_t*>(&slipBuffer), slipSize);
}

inline float __not_in_flash_func(adcMap)(const size_t adcIndex) {
  return (controlValues[adcIndex] - adcMins[adcIndex]) / adcRanges[adcIndex];
}


size_t msgCt=0;
bool __not_in_flash_func(adcProcessor)(__unused struct repeating_timer *t) {
  static size_t __not_in_flash("acdData") lastOctaveIdx = 0;
  // controlValues[0] = capture_buf[0];
  // controlValues[1] = capture_buf[1];
  // controlValues[2] = capture_buf[2];
  // controlValues[3] = capture_buf[3];

  controlValues[0] = adcFilters[0].process(capture_buf[0]);
  controlValues[1] = adcFilters[1].process(capture_buf[1]);
  controlValues[2] = adcFilters[2].process(capture_buf[2]);
  controlValues[3] = adcFilters[3].process(capture_buf[3]);

  size_t octaveIdx = static_cast<int>(controlValues[3]) >> 8;  // div by 256 -> 16 divisions
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
  float acc = 1;

  float new_wavelen0 = freqtable[std::lround(controlValues[0])];
  float detune = (adcMap(1) * 0.02f) * new_wavelen0;

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

  if (currMetaMod > 0) {
    auto metamods = metaOscsList.at(currMetaMod)->update(controlValues);
    // for(int i=0; i < 6; i++) {
    //   Serial.print(metamods[i]);
    //   Serial.print("\t");
    // }
    // Serial.println();

    new_wavelen0 *= (1.f + (metamods[0]));
    new_wavelen1 *= (1.f + (metamods[1]));
    new_wavelen2 *= (1.f + (metamods[2]));
    new_wavelen3 *= (1.f + (metamods[3]));
    new_wavelen4 *= (1.f + (metamods[4]));
    new_wavelen5 *= (1.f + (metamods[5]));
    new_wavelen6 *= (1.f + (metamods[6]));
    new_wavelen7 *= (1.f + (metamods[7]));
    new_wavelen8 *= (1.f + (metamods[8]));
    if (msgCt == 40) {
      Serial.println(metamods[0]);
    }
  }

  if (msgCt == 40) {
    Serial.printf("%f \n", controlValues[0]);
    msgCt=0;
  }
  msgCt++;

  sendToMyriadB(messageTypes::WAVELEN0, new_wavelen0);
  sendToMyriadB(messageTypes::WAVELEN1, new_wavelen1);
  sendToMyriadB(messageTypes::WAVELEN2, new_wavelen2);
  sendToMyriadB(messageTypes::WAVELEN3, new_wavelen3);
  sendToMyriadB(messageTypes::WAVELEN4, new_wavelen4);
  sendToMyriadB(messageTypes::WAVELEN5, new_wavelen5);
  
  const float ctrlVal = adcMap(2);
  currOscModels[0]->ctrl(ctrlVal);
  currOscModels[1]->ctrl(ctrlVal);
  currOscModels[2]->ctrl(ctrlVal);

  updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModels[0], new_wavelen6);
  updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModels[1], new_wavelen7);
  updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModels[2], new_wavelen8);
  // updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModelBank0A, new_wavelen6);
  // updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModelBank0B, new_wavelen7);
  // updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModelBank0C, new_wavelen8);

  oscsReadyToStart = true;

  return true;
}


inline bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
  display.update();
  return true;
}

int8_t read_rotary(uint8_t &prevNextCode, uint16_t &store, int a_pin, int b_pin) {
  static int8_t rot_enc_table[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

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

static uint8_t enc1Code = 0;
static uint16_t enc1Store = 0;
static uint8_t enc2Code = 0;
static uint16_t enc2Store = 0;
static uint8_t enc3Code = 0;
static uint16_t enc3Store = 0;

void assignOscModels(size_t modelIdx) {
  for(auto &model: currOscModels) {
    model = oscModelFactories[modelIdx](); 

  }
  // currOscModels[0] = std::make_shared<squareOscillatorModel>(); 
  // currOscModels[1] = std::make_shared<squareOscillatorModel>(); 
  // currOscModels[2] = std::make_shared<squareOscillatorModel>(); 
}

bool updateOscBank(int &currOscBank, int change, std::optional<messageTypes> OSCBANKMSG = std::nullopt) {
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
      // delay(500);
      // auto  newOscModelBank0 = std::make_unique<squareOscillatorModel>();
      // currOscModelBank0 = std::move(newOscModelBank0);
      switch(currOscBank) {
        case 0:
        // currOscModelBank0 = oscModel1; 
        // currOscModelBank0 = std::make_shared<squareOscillatorModel>(); 
        // currOscModels[0] = std::make_shared<squareOscillatorModel>(); 
        // currOscModels[1] = std::make_shared<squareOscillatorModel>(); 
        // currOscModels[2] = std::make_shared<squareOscillatorModel>(); 
        assignOscModels(0);
        break;
        case 1:
        // currOscModelBank0 = oscModel2; 
        // currOscModelBank0 = std::make_shared<squareOscillatorModel2>(); 
        // currOscModels[0] = std::make_shared<squareOscillatorModel2>(); 
        // currOscModels[1] = std::make_shared<squareOscillatorModel2>(); 
        // currOscModels[2] = std::make_shared<squareOscillatorModel2>(); 
        assignOscModels(1);
        break;              
      }
      // Serial.println("Starting");
      startOscBankA();
      // restartOscsFlag = true;
      // oscsReadyToStart = true;
    }
    Serial.println(currOscBank);  
  } 
  return changed;
}

void updateMetaOscMode(size_t &currMetaModMode, const int change) {
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

void encoder1_callback() {
  Serial.println("enc1");
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  // Serial.println("enc1");
  if (controls::encoderSwitches[0]) {
    display.setScreen(displayPortal<N_OSCILLATORS, N_OSC_BANKS>::SCREENMODES::METAOSCVIS);

    controls::encoderAltValues[0] += change;
    // Serial.println(controls::encoderAltValues[0]);
    updateMetaOscMode(currMetaMod, change);
    // display.setScreen(displayPortal::SCREENMODES::METAOSCVIS);
  }else{
    controls::encoderValues[0] += change;
    bool changed = updateOscBank(oscTypeBank0, change, messageTypes::BANK0);
    // display.setScreen(displayPortal::SCREENMODES::OSCBANKS);
    // display.drawOsc0(oscTypeBank0);
    if (changed) {
      // display.updateOscVis(oscModels.at(oscTypeBank0)->getVisData(), 0);
      display.setOscBankModel(0, oscTypeBank0);
    }
  }
}

void encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);

  if (controls::encoderSwitches[1]) {
    display.setScreen(displayPortal<N_OSCILLATORS,N_OSC_BANKS>::SCREENMODES::METAOSCVIS);
    controls::encoderAltValues[1] += change;
    if (currMetaMod > 0) {
      metaOscsList.at(currMetaMod)->setSpeed(change);
      display.setMetaModSpeed(metaOscsList.at(currMetaMod)->modspeed.getNormalisedValue());
      // Serial.println(currMetaMod->modspeed.getValue());
    }
  }else{
    controls::encoderValues[1] += change;
    bool changed = updateOscBank(oscTypeBank1, change, messageTypes::BANK1);
    if (changed) {
      // display.drawOsc1(oscTypeBank1);
      // display.updateOscVis(oscModels.at(oscTypeBank1)->getVisData(), 1);
      display.setOscBankModel(1, oscTypeBank1);

    }  
  }

}

void encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
  if (controls::encoderSwitches[2]) {
    display.setScreen(displayPortal<N_OSCILLATORS,N_OSC_BANKS>::SCREENMODES::METAOSCVIS);
    controls::encoderAltValues[2] += change;
    if (currMetaMod > 0) {
      metaOscsList.at(currMetaMod)->setDepth(change);
      display.setMetaModDepth(metaOscsList.at(currMetaMod)->moddepth.getNormalisedValue());
      // Serial.println(metaOscsList.at(currMetaMod)->moddepth.getNormalisedValue());
    }
  }else{
    controls::encoderValues[2] += change;
    bool changed = updateOscBank(oscTypeBank2, change, std::nullopt);
    if (changed) {
      // display.drawOsc2(oscTypeBank2);
      // display.updateOscVis(oscModels.at(oscTypeBank2)->getVisData(), 2);
      display.setOscBankModel(2, oscTypeBank2);

    }
  }
}

struct debouncer {
  debouncer() {
    ts = millis();
    val=0;
  }

  bool debounce(int pin) {
    auto now = millis();
    int gap = ts < now ? now - ts : std::numeric_limits<unsigned long>::max() - ts + now;
    if(gap > 100) {
      val = digitalRead(ENCODER1_SWITCH);
      ts = millis();
    }
    return val;
  }
  unsigned long ts;
  bool val;
};

debouncer enc1Debouncer;

void encoder1_switch_callback() {
  //gpio pull-up, so the value is inverted
  controls::encoderSwitches[0] = 1 - enc1Debouncer.debounce(ENCODER1_SWITCH);  
  Serial.printf("encswitch 1 %d\n",controls::encoderSwitches[0]);
  if (controls::encoderSwitches[0]) {
    display.toggleScreen();
  }
}

void encoder2_switch_callback() {
  controls::encoderSwitches[1] = 1 - digitalRead(ENCODER2_SWITCH);  
}

void encoder3_switch_callback() {
  controls::encoderSwitches[2] = 1 - digitalRead(ENCODER3_SWITCH);  
}


struct repeating_timer timerAdcProcessor;
struct repeating_timer timerDisplay;
struct repeating_timer timerFreqReceiver;



void setup() {


  //USB Serial
  Serial.begin();

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
  display.setScreen(displayPortal<N_OSCILLATORS,N_OSC_BANKS>::SCREENMODES::OSCBANKS);
  display.setMetaOsc(0, metaOscsList[0]);

  for(size_t i=0; i < 4; i++) {
    adcRanges[i] = adcMaxs[i] - adcMins[i];
  }

  // comms to Myriad B
  Serial1.setRX(13);
  Serial1.setTX(12);
  Serial1.begin(115200);

  // boids.init();

  // queue_init(&coreCommsQueue, sizeof(queueItem), 5);


  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  //ADCs
  const size_t filterSize=5;
  for(auto &filter: adcFilters) {
    filter.init(filterSize);
  }
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


  add_repeating_timer_ms(5, adcProcessor, NULL, &timerAdcProcessor);
  add_repeating_timer_ms(-39, displayUpdate, NULL, &timerDisplay);


}


int count=0;

void loop() {
  // for(size_t i=0; i < 3; i++) {
  //   Serial.print(i + "\t");  
  //   Serial.print(controls::encoderSwitches[i]);
  //   Serial.print("\t");  
  //   Serial.print(controls::encoderValues[i]);  
  //   Serial.print("\t");  
  //   Serial.print(controls::encoderAltValues[i]);  
  //   Serial.print("\t|\t");  
  // }
  // Serial.println();
  // delay(1);
  // Serial.println("Hello from Myriad A");

  // __wfi();
}


void setup1() {
// static FAST_MEM smBitStreamOsc smOsc0;
// static FAST_MEM smBitStreamOsc smOsc1;
// static FAST_MEM smBitStreamOsc smOsc2;
// #ifdef RUNCORE0_OSCS
//   smOsc0_dma_chan = smOsc0.init(pio1, 0, OSC7_PIN, programOffset, timing_swapbuffer_0_A, dma_irh, clockdiv, DMA_IRQ_1);
//   smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
//   smOsc0.go();

//   smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, timing_swapbuffer_1_A, dma_irh, clockdiv, DMA_IRQ_1);
//   smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
//   smOsc1.go();

//   smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, timing_swapbuffer_2_A, dma_irh, clockdiv, DMA_IRQ_1);
//   smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
//   smOsc2.go();
// #endif
//wait for serial
  // currOscModelBank0 = oscModel2;
  // currOscModels[0] = std::make_shared<squareOscillatorModel2>(); 
  // currOscModels[1] = std::make_shared<squareOscillatorModel2>(); 
  // currOscModels[2] = std::make_shared<squareOscillatorModel2>(); 
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
  // __wfi();
  // if (restartOscsFlag) {
  //   restartOscsFlag=false;
  //   startOscBankA();
  // }
  // tight_loop_contents();
  // if (oscsReadyToStart) {
  //   startOscBankA();
  //   oscsReadyToStart = false;
  // }  
}
