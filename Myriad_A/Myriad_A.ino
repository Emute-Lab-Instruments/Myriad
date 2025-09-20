#define MYRIAD_VERSION "1.0.0"

#include <FS.h>
#include <LittleFS.h>

#include <optional>
#include "calibration.hpp"
#include "displayPortal.h"
#include "myriad_pins.h"
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


#include "tuning.hpp"
#include "state.hpp"

// #define SINGLEOSCILLATOR

// #define TEST_ARPEGGIATOR
#ifdef TEST_ARPEGGIATOR
float arpCount=0;
float arpIndex= 0;
// std::array<float, 4> arpNotes = {0.2,0.4,0.6, 0.8}; //1, 3, 5
std::array<float, 102> arpNotes = 
{
  0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,
  0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,
  0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,
  0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,
  0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,0.462696,

  0.520930,

  0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,
0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,
0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,
0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,
0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,0.537597,
0.482853

};
#endif


constexpr int cal_data[11] = {19, 758, 1517,2298,3051, 3815, 4589, 5342, 6125, 6878, 7639};

ADCCalibrator<13,11,float> __not_in_flash("pitchadclookup") pitchADCMap(cal_data);


bool core1_separate_stack = true;

constexpr size_t N_OSCILLATORS=9;
constexpr size_t N_OSC_BANKS=3;

using portal = displayPortal<N_OSCILLATORS,N_OSC_BANKS,N_OSCILLATOR_MODELS>;

portal FAST_MEM display;

enum CONTROLMODES {OSCMODE, METAOSCMODE, CALIBRATEMODE, TUNINGMODE, UTILITYMODE, QUANTISESETTINGSMODE} controlMode = CONTROLMODES::OSCMODE;

#define RUN_OSCS
std::array<oscModelPtr, 3> FAST_MEM currOscModels;

volatile bool FAST_MEM oscsReadyToStart=false;
volatile bool FAST_MEM restartOscsFlag=false;

//a list of reference models for display data
// std::array<oscModelPtr,N_OSCILLATOR_MODELS> FAST_MEM oscModelsDisplayRef;

metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscBlank = std::make_shared<metaOscNone<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscNN = std::make_shared<metaOscMLP<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscSines1 = std::make_shared<metaOscSines<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaOscSinesFMultiple1 = std::make_shared<metaOscSinesFMultiple<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaDrunkenWalkers1 = std::make_shared<metaDrunkenWalkers<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaLorenz1 = std::make_shared<metaLorenz<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaRossler1 = std::make_shared<metaRossler<N_OSCILLATORS>>();
metaOscPtr<N_OSCILLATORS> __not_in_flash("mydata") metaBoids1 = std::make_shared<metaOscBoids<N_OSCILLATORS>>();


std::array<metaOscPtr<N_OSCILLATORS>, 8> __not_in_flash("mydata") metaOscsList = {metaOscBlank, metaBoids1, metaLorenz1, metaOscSines1, metaRossler1, metaOscSinesFMultiple1, metaOscNN, metaDrunkenWalkers1};

size_t FAST_MEM currMetaMod = 0;

DEFINE_TIMING_SWAPBUFFERS(0)
DEFINE_TIMING_SWAPBUFFERS(1)
DEFINE_TIMING_SWAPBUFFERS(2)

#define CORE1_FAST_MEM __scratch_y("myriad")

uint32_t CORE1_FAST_MEM smOsc0_dma_chan;
uint32_t CORE1_FAST_MEM smOsc0_dma_chan_bit;

uint32_t CORE1_FAST_MEM smOsc1_dma_chan;
uint32_t CORE1_FAST_MEM smOsc1_dma_chan_bit;

uint32_t CORE1_FAST_MEM smOsc2_dma_chan;
uint32_t CORE1_FAST_MEM smOsc2_dma_chan_bit;

volatile bool FAST_MEM bufSent0 = false;
volatile bool FAST_MEM bufSent1 = false;
volatile bool FAST_MEM bufSent2 = false;


struct repeating_timer FAST_MEM timerMetaModUpdate;


/////////////////////////////////   IRQ 000000000000000000000000000000000000
void __isr dma_irh() {
  //TEST
  // Serial.println("dma");
  uint32_t triggered_channels = dma_hw->ints1;
  if (triggered_channels & smOsc0_dma_chan_bit) {
    dma_hw->ints1 = smOsc0_dma_chan_bit;  
    dma_hw->ch[smOsc0_dma_chan].al3_read_addr_trig = nextTimingBuffer0;
    bufSent0 = true;
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

volatile bool FAST_MEM oscsRunning = false;
static spin_lock_t FAST_MEM *calcOscsSpinlock;
static spin_lock_t FAST_MEM *displaySpinlock;
static spin_lock_t FAST_MEM *adcSpinlock;

void startOscBankA() {

  Serial.println("StartoscbankA");

  //reset the pio block
  pio_clear_instruction_memory(pio1);
  uint programOffset = currOscModels[0]->loadProg(pio1);
  pio_sm_config baseConfig = currOscModels[0]->getBaseConfig(programOffset);
  Serial.printf("Offset %d\n", programOffset);

  const size_t modelClockDiv = currOscModels[0]->getClockDiv();

  smOsc0_dma_chan = smOsc0.init(pio1, 0, OSC7_PIN, programOffset, baseConfig, nextTimingBuffer0, dma_irh, modelClockDiv, currOscModels[0]->loopLength, DMA_IRQ_1);
  // Serial.println("init");
  // if (smOsc0_dma_chan < 0) {
    // Serial.println("dma chan allocation error");
  // }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();
  
  #ifndef SINGLEOSCILLATOR

  // smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, nextTimingBuffer1, dma_irh, clockdiv, currOscModelBank0B->loopLength, DMA_IRQ_1);
  smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, baseConfig, nextTimingBuffer1, dma_irh, modelClockDiv, currOscModels[1]->loopLength, DMA_IRQ_1);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  // smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, nextTimingBuffer2, dma_irh, clockdiv, currOscModelBank0C->loopLength, DMA_IRQ_1);
  smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, baseConfig, nextTimingBuffer2, dma_irh, modelClockDiv, currOscModels[2]->loopLength, DMA_IRQ_1);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();
  // Serial.println("started");
#endif

  oscsRunning = true;
}

void __not_in_flash_func(stopOscBankA)() {
//   uint32_t save = spin_lock_blocking(calcOscsSpinlock);  
//   if (oscsRunning) {
//     oscsRunning = false;
//     smOsc0.stop();
// #ifndef SINGLEOSCILLATOR
//     smOsc1.stop();
//     smOsc2.stop();
// #endif
//   }
//   bufSent0 = false;
//   bufSent1 = false;
//   bufSent2 = false;  
//   spin_unlock(calcOscsSpinlock, save);
//   delayMicroseconds(100);
}

// #define SCREEN_WIDTH tft.width()    //
// #define SCREEN_HEIGHT tft.height()  // Taille de l'écran

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
  static unsigned long FAST_MEM encoderSwitchUPTS[3] = {0,0,0};
  static bool FAST_MEM calibrateButton=0;
};

// MovingAverageFilter<float> FAST_MEM adcFilters[4];
// MedianFilter<float> FAST_MEM adcMedians[4];
ResponsiveFilter FAST_MEM adcRsFilters[4];

static float __scratch_x("adc") controlValues[4] = {0,0,0,0};
// static size_t __not_in_flash("mydata") controlValueMedians[4] = {0,0,0,0};

static uint16_t __not_in_flash("adc") capture_buf[16] __attribute__((aligned(2048)));
static uint16_t __not_in_flash("adc") capture_buf_a[16] __attribute__((aligned(2048)));
static uint16_t __not_in_flash("adc") capture_buf_b[16] __attribute__((aligned(2048)));

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


size_t FAST_MEM oscBankTypes[3] = {0,0,0}; 

volatile int32_t knobssm[4] = {0,0,0,0};
volatile uint16_t knobs[4] = {0,0,0,0};

void __not_in_flash_func(adc_irq_handler)() {
    // Get ADC data from FIFO
    const uint16_t adc = adc_fifo_get_blocking();
    // Serial.printf("%d %d\n", adc, adc_get_selected_input());
    switch(adc_get_selected_input()) {
        case 0:
            // Knob 0 - stronger smoothing for knobs
            // knobssm[0] = (127 * (knobssm[0]) + 16 * adc) >> 7;
            // knobs[0] = knobssm[0] >> 4;
            // Serial.print("K0: "); Serial.println(adc);
            break;
        case 1:
        {
            // Knob 1
            knobssm[1] = (127 * (knobssm[1]) + 16 * adc) >> 7;
            knobs[1] = knobssm[1] >> 4;
            Serial.print("K1: "); Serial.println(adc);
            bool err = (adc & 0b1000000000000000) > 0;
            if (err) {
              Serial.println("ADC ERROR");
            }
          }
            break;
        case 2:
            // Knob 2
            knobssm[2] = (127 * (knobssm[2]) + 16 * adc) >> 7;
            knobs[2] = knobssm[2] >> 4;
            break;
        case 3:
            // Knob 3
            knobssm[3] = (127 * (knobssm[3]) + 16 * adc) >> 7;
            knobs[3] = knobssm[3] >> 4;
            break;
    }
}

uint __scratch_x("adc") dma_chan;
uint __scratch_x("adc") dma_chan2;

void adc_dma_irq_handler() {
  uint16_t *adcReadings = nullptr;
  if (dma_channel_get_irq0_status(dma_chan)) {
      dma_channel_acknowledge_irq0(dma_chan);
      adcReadings = capture_buf_a;      
      // Your buffer is full or at the ring boundary
      // Process data in capture_buf here
      // Serial.printf("%d %d %d %d\n",capture_buf_a[0],capture_buf_a[1],capture_buf_a[2],capture_buf_a[3]);
      // dma_channel_set_write_addr(dma_chan, capture_buf, false);
      // dma_channel_set_trans_count(dma_chan, 4, true);        
  }
  if (dma_channel_get_irq0_status(dma_chan2)) {
      dma_channel_acknowledge_irq0(dma_chan2);
      adcReadings = capture_buf_b;      
      // Your buffer is full or at the ring boundary
      // Process data in capture_buf here
      // Serial.printf("%d %d %d %d\n",capture_buf_b[0],capture_buf_b[1],capture_buf_b[2],capture_buf_b[3]);
      // dma_channel_set_write_addr(dma_chan, capture_buf, false);
      // dma_channel_set_trans_count(dma_chan, 4, true);        
  }
  if (adcReadings) {
    
    Serial.printf("%d %d %d %d %d\n",micros(), adcReadings[0],adcReadings[1],adcReadings[2],adcReadings[3]);
  }
}
void setup_adcs() {
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  adc_gpio_init(29);

  // // STOP ADC to ensure clean start
  // adc_run(false);
  
  // // Flush any existing FIFO data
  // while (!adc_fifo_is_empty()) {
  //     adc_fifo_get_blocking();
  // }  

  adc_set_round_robin(15);
  adc_fifo_setup(
    true,   // Write each completed conversion to the sample FIFO
    true,   // Enable DMA data request (DREQ)
    4,      // DREQ (and IRQ) asserted when at least 1 sample present
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
  // adc_set_clkdiv(adcClockDiv); 
  // adc_set_clkdiv(0);
  constexpr size_t adcSystemFreq = 100 * 4; //allow for 4 readings
  adc_set_clkdiv((48000000 / 400) - 1);

  //setup two dma channels in ping pong
  dma_chan = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

  dma_chan2 = dma_claim_unused_channel(true);
  dma_channel_config cfg2 = dma_channel_get_default_config(dma_chan2);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_ring(&cfg, true, 8);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);
  channel_config_set_chain_to(&cfg, dma_chan2);


  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg2, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg2, false);
  channel_config_set_write_increment(&cfg2, true);
  channel_config_set_ring(&cfg2, true, 8);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg2, DREQ_ADC);
  channel_config_set_chain_to(&cfg2, dma_chan);


  dma_channel_configure(dma_chan, &cfg,
                        capture_buf_a,    // dst
                        &adc_hw->fifo,  // src
                        4,             // transfer count
                        false            // start immediately
  );

  dma_channel_configure(dma_chan2, &cfg2,
                        capture_buf_b,    // dst
                        &adc_hw->fifo,  // src
                        4,             // transfer count
                        false            // start immediately
  );

  dma_channel_start(dma_chan);

  adc_select_input(0);

  //set up interrupts
  dma_channel_set_irq0_enabled(dma_chan, true);
  dma_channel_set_irq0_enabled(dma_chan2, true);
  irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_irq_handler);
  irq_set_enabled(DMA_IRQ_0, true);  

  adc_run(true);
}


void __force_inline __not_in_flash_func(sendToMyriadB) (uint8_t msgType, float value) {
  static uint8_t __not_in_flash("sendToMyriadData") slipBuffer[64];
  spiMessage msg {msgType, value};
  unsigned int slipSize = SLIP::encode(reinterpret_cast<uint8_t*>(&msg), sizeof(spiMessage), &slipBuffer[0]);
  int res = Serial1.write(reinterpret_cast<uint8_t*>(&slipBuffer), slipSize);
}

inline float __not_in_flash_func(adcMap)(const size_t adcIndex) {
  //get the value
  uint32_t save = spin_lock_blocking(adcSpinlock);  
  const float val = controlValues[adcIndex];
  spin_unlock(adcSpinlock, save);

  //mapping
  float mappedVal = (val - (CalibrationSettings::adcMins[adcIndex])) * CalibrationSettings::adcRangesInv[adcIndex];
  if (mappedVal < 0.f) mappedVal = 0.f;

  return mappedVal;
}

bool __not_in_flash_func(metaModUpdate)(__unused struct repeating_timer *t) {
  if (currMetaMod > 0) {
    uint32_t save = spin_lock_blocking(adcSpinlock);  
    auto metamods = metaOscsList.at(currMetaMod)->update(controlValues);
    spin_unlock(adcSpinlock, save);
  }
  return true;
}

size_t FAST_MEM msgCt=0;
volatile bool FAST_MEM adcReadyFlag = false;

static size_t __not_in_flash("adcData") lastOctaveIdx = 0;
static size_t __scratch_x("adc") adcCount = 0;
// static size_t __scratch_x("adc") adcAccumulator[4] = {0,0,0,0};
static size_t __scratch_x("adc") adcAccumulator0=0;
static size_t __scratch_x("adc") adcAccumulator1=0;
static size_t __scratch_x("adc") adcAccumulator2=0;
static size_t __scratch_x("adc") adcAccumulator3=0;

bool __not_in_flash_func(adcProcessor)(__unused struct repeating_timer *t) {
  adcAccumulator0 += capture_buf[0];
  adcAccumulator1 += capture_buf[1];
  adcAccumulator2 += capture_buf[2];
  adcAccumulator3 += capture_buf[3];
  if (adcCount == accumulatorCount-1) {
    adcCount=0;
    // adcAccumulator0 = adcAccumulator0 >> accumulatorBits;
    // adcAccumulator1 = adcAccumulator1 >> accumulatorBits;
    // adcAccumulator2 = adcAccumulator2 >> accumulatorBits;
    // adcAccumulator3 = adcAccumulator3 >> accumulatorBits;
    // controlValues[0] = adcRsFilters[0].process(adcAccumulator[0]);
    // controlValues[1] = adcRsFilters[1].process(adcAccumulator[1]);
    // controlValues[2] = adcRsFilters[2].process(adcAccumulator[2]);
    // controlValues[3] = adcRsFilters[3].process(adcAccumulator[3]);
    uint32_t save = spin_lock_blocking(adcSpinlock);  
    controlValues[0] = adcAccumulator0 >> adcDivisor;
    controlValues[1] = adcAccumulator1 >> adcDivisor;
    controlValues[2] = adcAccumulator2 >> adcDivisor;
    controlValues[3] = adcAccumulator3 >> adcDivisor;
    adcReadyFlag = true;
    spin_unlock(adcSpinlock, save);

    adcAccumulator0=0;
    adcAccumulator1=0;
    adcAccumulator2=0;
    adcAccumulator3=0;

  }
  adcCount++;
  return true;
}



void __not_in_flash_func(processAdc)() {

  Serial.printf("%f\n", controlValues[0]);


  if (controlMode == CONTROLMODES::CALIBRATEMODE) {

    for(size_t i=0; i < 4; i++) {
      uint32_t save = spin_lock_blocking(adcSpinlock);  
      if (controlValues[i] < CalibrationSettings::adcMins[i] && controlValues[i] >= 0) {
        CalibrationSettings::adcMins[i] = controlValues[i];
      }
      if (controlValues[i] > CalibrationSettings::adcMaxs[i] && controlValues[i] < adcScaleMax) {
        CalibrationSettings::adcMaxs[i] = controlValues[i];
      }
      CalibrationSettings::adcRanges[i] = CalibrationSettings::adcMaxs[i] - CalibrationSettings::adcMins[i];
      CalibrationSettings::adcRangesInv[i] = 1.f / CalibrationSettings::adcRanges[i];
      spin_unlock(adcSpinlock, save);
    }
    // display.setCalibADCValues(adcAccumulator0, adcAccumulator1, adcAccumulator2, adcAccumulator3);
    display.setCalibADCValues(0,0,0,0);
    display.setCalibADCFiltValues(controlValues[0], controlValues[1], controlValues[2], controlValues[3]);
    display.setCalibADCMinMaxValues(CalibrationSettings::adcMins, CalibrationSettings::adcMaxs);
  }
  

  // size_t octaveIdx = static_cast<size_t>(controlValues[3]) >> 8;  // div by 256 -> 16 divisions
  uint32_t save = spin_lock_blocking(adcSpinlock);  
  static __scratch_x("adc") size_t octScaleDiv = adcResBits-4;
  size_t octaveIdx = static_cast<size_t>(controlValues[3]) >> octScaleDiv;  // 16 divisions
  spin_unlock(adcSpinlock, save);

  if (octaveIdx != lastOctaveIdx) {
    lastOctaveIdx = octaveIdx;
    switch(octaveIdx) {
      case 0:   {octaves = {1.0f, 1.0f,  1.0f,    1.0f,  1.0f,  1.0f,    1.0f,  1.0f,  1.0f}; break;}
      case 1:   {octaves = {0.5f, 1.0f,  1.0f,    0.5f,  1.0f,  1.0f,    0.5f,  1.0f,  1.0f}; break;}
      case 2:   {octaves = {0.5f, 0.5f,  1.0f,    0.5f,  0.5f,  1.0f,    0.5f,  0.5f,  1.0f}; break;}
      case 3:   {octaves = {0.25f, 0.5f,  1.0f,    0.25f,  0.5f,  1.0f,    0.25f,  0.5f,  1.0f}; break;}

      case 4:   {octaves = {0.25f, 0.5f,  2.0f,    0.25f,  0.5f,  2.0f,    0.25f,  0.5f,  2.0f}; break;}
      case 5:   {octaves = {0.25f, 0.5f,  4.0f,    0.25f,  0.5f,  4.0f,    0.25f,  0.5f,  4.0f}; break;}
      case 6:   {octaves = {0.25f, 0.25f,  4.0f,    0.25f,  0.25f,  4.0f,    0.25f,  0.25f,  4.0f}; break;}
      case 7:   {octaves = {0.5f, 0.25f,  4.0f,    0.5f,  0.25f,  4.0f,    0.5f,  0.25f,  4.0f}; break;}

      case 8:   {octaves = {0.25f, 2.0f,  4.0f,    0.25f,  2.0f,  4.0f,    0.25f,  2.0f,  4.0f}; break;}
      case 9:   {octaves = {0.25f, 4.0f,  4.0f,    0.25f,  4.0f,  4.0f,    0.25f,  4.0f,  4.0f}; break;}
      case 10:  {octaves = {0.5f, 4.0f,  4.0f,    0.5f,  4.0f,  4.0f,    0.25f,  4.0f,  4.0f}; break;}
      case 11:  {octaves = {1.0f, 4.0f,  4.0f,    1.0f,  4.0f,  4.0f,    1.0f,  4.0f,  4.0f}; break;}

      case 12:  {octaves = {2.0f, 4.0f,  4.0f,    2.0f,  4.0f,  4.0f,    2.0f,  4.0f,  4.0f}; break;}
      case 13:  {octaves = {2.0f, 2.0f,  4.0f,    2.0f,  2.0f,  4.0f,    2.0f,  2.0f,  4.0f}; break;}
      case 14:  {octaves = {1.0f, 2.0f,  4.0f,    1.0f,  2.0f,  4.0f,    1.0f,  2.0f,  4.0f}; break;}
      case 15:  {octaves = {1.0f, 2.0f,  2.0f,    1.0f,  2.0f,  2.0f,    1.0f,  2.0f,  2.0f}; break;}
      default:;
    }
  }

  
#ifdef TEST_ARPEGGIATOR
  float pitchCV=arpNotes[arpIndex];
  // if (arpCount==0) {
    // arpCount=0;
    arpIndex+=1;
    if (arpIndex >= arpNotes.size()) {
      arpIndex=0;
    }
  // }
  // arpCount+=1;
#else
  // float pitchCV = adcMap(0);
  size_t index = (size_t)controlValues[0];
  float voltage = pitchADCMap[index];
  float pitchCV = (voltage+5.f) * 0.1f;
  Serial.printf("%d, %f, %f\n", index, voltage, pitchCV);
#endif

  //quantise?
  // constexpr float step = 0.1f / 5.f;
  // constexpr float stepinv = 1.f/step;
  if (!TuningSettings::bypass && TuningSettings::quantPull > 0.f) {
    // float quantStep = 0.1f / TuningSettings::quantNotesPerOct;    
    // float quantStepInv = 1.f/quantStep;
    float quantCV = std::round(pitchCV * TuningSettings::quantStepInv) * TuningSettings::quantStep;
    // float TuningSettings::quantAlpha = TuningSettings::quantPull * 0.01f;
    float diff = quantCV - pitchCV;
    pitchCV = pitchCV + (diff * TuningSettings::quantAlpha);
  }

  //exponential conversion
  float freq = powf(2.f, (pitchCV * 10.f) ); 
  
  
  //calc wavelenth
  const float wvlen = TuningSettings::bypass ? TuningSettings::wavelenC1 : TuningSettings::baseWavelen;
  float new_wavelen0 = 1.0f/freq * wvlen; 
  new_wavelen0 = new_wavelen0 > TuningSettings::wavelenC1 ?  TuningSettings::wavelenC1 : new_wavelen0;

  // static int printts = 0;
  // auto now = millis();
  // if (now - printts > 100) {
  //   Serial.printf("%f %f %f %f\n", pitchCV, freq, sampleClock/new_wavelen0, TuningSettings::baseFrequency);
  //   printts = now;
  // }
  float detuneControl = adcMap(1);
  detuneControl *= detuneControl; //exponential mapping

  float detune = (detuneControl * 0.01f) * new_wavelen0;
  float acc = 1.f - (detuneControl * 0.02f);
  
  float new_wavelen1 = (new_wavelen0 - detune) * acc;
  float new_wavelen2 = (new_wavelen1 - detune) * acc;
  float new_wavelen3 = (new_wavelen2 - detune) * acc;
  float new_wavelen4 = (new_wavelen3 - detune) * acc;
  float new_wavelen5 = (new_wavelen4 - detune) * acc;


  float new_wavelen6 = (new_wavelen5 - detune) * acc;
  float new_wavelen7 = (new_wavelen6 - detune) * acc;
  float new_wavelen8 = (new_wavelen7 - detune) * acc;

  // float maxOctaveMul = floorf(TuningSettings::wavelenC1 / new_wavelen0);
  //   Serial.println(maxOctaveMul);

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

  if (currMetaMod > 0) {
    // auto metamods = metaOscsList.at(currMetaMod)->update(controlValues);
    auto metamods = metaOscsList.at(currMetaMod)->getValues();

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
    
  }

  if (msgCt == 100U) {
    // Serial.printf("%f %f %f %f\n",adcMap(0), freq, new_wavelen0, wavelen20hz);
    Serial.print(".");
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

  currOscModels[0]->setWavelen(new_wavelen0);  
  currOscModels[1]->setWavelen(new_wavelen1);
  currOscModels[2]->setWavelen(new_wavelen2);

  oscsReadyToStart = true;

  if (controlMode == CONTROLMODES::OSCMODE) {
    display.setDisplayWavelengths({new_wavelen0,new_wavelen1,new_wavelen2,
                                  new_wavelen3,new_wavelen4,new_wavelen5,
                                  new_wavelen6,new_wavelen7,new_wavelen8 });
  }

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

void __force_inline __not_in_flash_func(calculateOscBuffers)() {
  if ((currOscModels[0]->updateBufferInSyncWithDMA && bufSent0) || (!currOscModels[0]->updateBufferInSyncWithDMA && currOscModels[0]->newFreq)) {
    currOscModels[0]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModels[0]);
    bufSent0 = false;
  }
  if ((currOscModels[1]->updateBufferInSyncWithDMA && bufSent1) || (!currOscModels[1]->updateBufferInSyncWithDMA && currOscModels[1]->newFreq)) {
    currOscModels[1]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModels[1]);
    bufSent1 = false;
  }
  if ((currOscModels[2]->updateBufferInSyncWithDMA && bufSent2) || (!currOscModels[2]->updateBufferInSyncWithDMA && currOscModels[2]->newFreq)) {
    currOscModels[2]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModels[2]);
    bufSent2 = false;
  }
}

void __not_in_flash_func(assignOscModels)(const size_t modelIdx) {
  

  for(auto &model: currOscModels) {
    model = oscModelFactories[modelIdx](); 

  }
}

size_t FAST_MEM oscModeBankChangeTS[3] = {0,0,0};
// size_t FAST_MEM oscModeBankChange[3] = {0,0,0};

// inline bool __not_in_flash_func(oscModeChangeMonitor)(__unused struct repeating_timer *t) {
inline bool __not_in_flash_func(oscModeChangeMonitor)() {
  for(size_t bank=0; bank < 3; bank++) {
    if (oscModeBankChangeTS[bank] > 0) {
      size_t elapsed = millis() - oscModeBankChangeTS[bank];
      if (elapsed > 80) {
        
        oscModeBankChangeTS[bank] = 0;

        if (bank ==2) {

          uint32_t save = spin_lock_blocking(calcOscsSpinlock);  
          if (oscsRunning) {
            oscsRunning = false;
            smOsc0.stop();
        #ifndef SINGLEOSCILLATOR
            smOsc1.stop();
            smOsc2.stop();
        #endif
          }
          bufSent0 = false;
          bufSent1 = false;
          bufSent2 = false;  
        // delayMicroseconds(100);
          // stopOscBankA();

          dma_hw->ints1 = smOsc0_dma_chan_bit | smOsc1_dma_chan_bit | smOsc2_dma_chan_bit;

          auto w1 = currOscModels[0]->getWavelen();
          auto w2 = currOscModels[1]->getWavelen();
          auto w3 = currOscModels[2]->getWavelen();

        
          // MyriadState::setOscBank(2, oscBankTypes[2]);

          assignOscModels(oscBankTypes[2]);

          //refill from new oscillator
          //trigger buffer refills
          currOscModels[0]->reset();
          currOscModels[1]->reset();
          currOscModels[2]->reset();

          currOscModels[0]->setWavelen(w1);
          currOscModels[1]->setWavelen(w2);
          currOscModels[2]->setWavelen(w3);

          // currOscModels[0]->newFreq=true;
          // currOscModels[1]->newFreq=true;
          // currOscModels[2]->newFreq=true;

          calculateOscBuffers();

          startOscBankA();

          spin_unlock(calcOscsSpinlock, save);


        } else {

          sendToMyriadB(bank == 1 ? messageTypes::BANK1 : messageTypes::BANK0, oscBankTypes[bank]);
          // MyriadState::setOscBank(bank, oscBankTypes[bank]);
        
        }
      }
    }

  }

  return true;
}

void __not_in_flash_func(setMetaOscMode)(size_t mode) {
  currMetaMod = mode;
  display.setMetaOsc(currMetaMod, metaOscsList[currMetaMod]);

  cancel_repeating_timer(&timerMetaModUpdate);
  add_repeating_timer_ms(metaOscsList[currMetaMod]->getTimerMS(), metaModUpdate, NULL, &timerMetaModUpdate);

  Serial.printf("meta mod mode change %d\n", currMetaMod);

}

// void __not_in_flash_func(updateMetaOscMode)(size_t &currMetaModMode, const int change) {
//   int newMetaModMode = static_cast<int>(currMetaModMode) + change;
//   //clip
//   newMetaModMode = max(0, newMetaModMode);
//   newMetaModMode = min(metaOscsList.size()-1, newMetaModMode);

//   if (newMetaModMode != currMetaModMode) {
//     setMetaOscMode(newMedaModMode);
//   } 
// }

void updateTuning() {
  TuningSettings::update();
  display.setTuning(TuningSettings::octaves, TuningSettings::semitones, TuningSettings::cents);
  display.setTuningBypass(TuningSettings::bypass);
  // Serial.printf("%f %f\n", courseTuning, fineTuning);
}

constexpr size_t postSwitchUpPause = 200; //ms

void __isr encoder1_callback() {
  auto timeSinceSwitchUp = millis() - controls::encoderSwitchUPTS[0];
  if (!controls::encoderSwitches[0] && timeSinceSwitchUp > postSwitchUpPause)
  { 
    int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
    switch(controlMode) {
      case CONTROLMODES::METAOSCMODE:
      {
        controls::encoderAltValues[0] += change;
        int newMetaModMode = static_cast<int>(currMetaMod) + change;
        //clip
        newMetaModMode = max(0, newMetaModMode);
        newMetaModMode = min(metaOscsList.size()-1, newMetaModMode);

        if (newMetaModMode != currMetaMod) {
          setMetaOscMode(newMetaModMode);
        } 
        break;
      }
      case CONTROLMODES::OSCMODE:
      {
        controls::encoderValues[0] += change;
        int newOscTypeBank = oscBankTypes[1] + change;
        //clip
        newOscTypeBank = max(0, newOscTypeBank);
        newOscTypeBank = min(N_OSCILLATOR_MODELS-1, newOscTypeBank);
        //prepare for osc mode change
        if (newOscTypeBank != oscBankTypes[1]) {
          oscModeBankChangeTS[1] = millis();
          oscBankTypes[1] = newOscTypeBank;
          display.setOscBankModel(1, oscBankTypes[1]);
        }
        // bool changed = updateOscBank(oscTypeBank1, change, messageTypes::BANK1);
        // if (changed) {
        //   display.setOscBankModel(1, oscTypeBank1);
        //   Serial.printf("model %d\n", oscTypeBank1);
        // }
        break;
      }
      case CONTROLMODES::CALIBRATEMODE:
      {
        display.setCalibEncoderDelta(0,change);      
        break;
      }
      case CONTROLMODES::TUNINGMODE:
      {
        TuningSettings::semitones += change;
        updateTuning();
        break;
      }
      case CONTROLMODES::QUANTISESETTINGSMODE:
      {
        auto newVal = TuningSettings::quantNotesPerOct + change;
        if (newVal < 1) {
          newVal = 1;
        }
        if (newVal > 39) {
          newVal = 39;
        }
        TuningSettings::quantNotesPerOct = newVal;
        TuningSettings::updateQuant();
        display.setQuant(TuningSettings::quantPull, TuningSettings::quantNotesPerOct);

        break;
      }
    }
  }
}

//left
struct encAccelData {
  size_t prevChangeTS = 0;
  float acc = 0.f;
};

encAccelData FAST_MEM enc2Accel;

float __not_in_flash_func(calcAcceleration)(const int change, encAccelData &data) {
  auto now = millis();
  if (change != 0) {
    auto gap = millis() - data.prevChangeTS;
    data.acc += std::max(50.f-gap, -50.f)*0.012f;
  }
  data.acc *= 0.95f;
  data.prevChangeTS = now;
  float changeWithAcc = change * (1.f + data.acc);
  if (change == 1 && changeWithAcc < 0.f) {
    changeWithAcc = 0.f;
  }
  if (change == -1 && changeWithAcc > 0.f) {
    changeWithAcc = 0.f;
  }
  // Serial.printf("change %d acc %f -> %f\n", change, accEnc2, changeWithAcc);
  return changeWithAcc;
}

void __isr encoder2_callback() {
    auto timeSinceSwitchUp = millis() - controls::encoderSwitchUPTS[1];
    if (!controls::encoderSwitches[1] && timeSinceSwitchUp > postSwitchUpPause) {
  
    // if (!controls::encoderSwitches[1]) {  
      int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
      switch(controlMode) {
        case CONTROLMODES::METAOSCMODE:
        {
          controls::encoderAltValues[1] += change;
          if (currMetaMod > 0) {
            const float changeWithAcc = calcAcceleration(change, enc2Accel);
            metaOscsList.at(currMetaMod)->setSpeed(changeWithAcc);
            display.setMetaModSpeed(metaOscsList.at(currMetaMod)->modspeed.getNormalisedValue());
          }
          break;
        }
        case CONTROLMODES::OSCMODE:
        {
          controls::encoderValues[1] += change;
          int newOscTypeBank = oscBankTypes[0] + change;
          //clip
          newOscTypeBank = max(0, newOscTypeBank);
          newOscTypeBank = min(N_OSCILLATOR_MODELS-1, newOscTypeBank);
          //prepare for osc mode change
          if (newOscTypeBank != oscBankTypes[0]) {
            oscModeBankChangeTS[0] = millis();
            // oscModeBankChange[2] = newOscTypeBank;
            oscBankTypes[0] = newOscTypeBank;
            display.setOscBankModel(0, oscBankTypes[0]);
          }
          // bool changed = updateOscBank(oscTypeBank0, change, messageTypes::BANK0);
          // if (changed) {
          //   display.setOscBankModel(0, oscTypeBank0);
          //   Serial.printf("enc 2 model %d\n", oscTypeBank0);

          // }  
          break;
        }
        case CONTROLMODES::CALIBRATEMODE:
        {
          display.setCalibEncoderDelta(1,change);      
          break;
        }
        case CONTROLMODES::TUNINGMODE:
        {
          TuningSettings::octaves += change;
          updateTuning();
          break;
        }
        case CONTROLMODES::QUANTISESETTINGSMODE:
        {
          int newVal = TuningSettings::quantPull + change;
          if (newVal < 0) {
            newVal = 0;
          }
          if (newVal > 100) {
            newVal = 100;
          }
          TuningSettings::quantPull = static_cast<size_t>(newVal);
          TuningSettings::updateQuant();
          display.setQuant(TuningSettings::quantPull, TuningSettings::quantNotesPerOct);

          break;
        }

      }
    }
}

//right
encAccelData FAST_MEM enc3Accel;

void __isr encoder3_callback() {
  auto timeSinceSwitchUp = millis() - controls::encoderSwitchUPTS[2];
  // Serial.printf("enc3 tsu %d\n", timeSinceSwitchUp);
  if (!controls::encoderSwitches[2] && timeSinceSwitchUp > postSwitchUpPause)
  {  
    int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
    switch(controlMode) {
      case CONTROLMODES::METAOSCMODE:
      {
        controls::encoderAltValues[2] += change;
        if (currMetaMod > 0) {
          const float changeWithAcc = calcAcceleration(change, enc2Accel);
          metaOscsList.at(currMetaMod)->setDepth(changeWithAcc);
          display.setMetaModDepth(metaOscsList.at(currMetaMod)->moddepth.getNormalisedValue());
        }
        break;
      }
      case CONTROLMODES::OSCMODE:
      {
        controls::encoderValues[2] += change;
        int newOscTypeBank = oscBankTypes[2] + change;
        //clip
        newOscTypeBank = max(0, newOscTypeBank);
        newOscTypeBank = min(N_OSCILLATOR_MODELS-1, newOscTypeBank);
        //prepare for osc mode change
        if (newOscTypeBank != oscBankTypes[2]) {
          oscModeBankChangeTS[2] = millis();
          // oscModeBankChange[2] = newOscTypeBank;
          oscBankTypes[2] = newOscTypeBank;
          display.setOscBankModel(2, oscBankTypes[2]);
        }

        // bool changed = updateOscBank(oscTypeBank2, change, std::nullopt);
        // if (changed) {
        //   display.setOscBankModel(2, oscTypeBank2);

        // }
        break;
      }
      case CONTROLMODES::CALIBRATEMODE:
      {
        display.setCalibEncoderDelta(2, change);      
        break;
      }
      case CONTROLMODES::TUNINGMODE:
      {
        TuningSettings::cents += change;
        updateTuning();
        break;
      }
    }
  }

}

debouncer FAST_MEM enc1Debouncer;
debouncer FAST_MEM enc2Debouncer;
debouncer FAST_MEM enc3Debouncer;
debouncer FAST_MEM calibrateButtonDebouncer;

void switchToOSCMode() {
  controlMode = CONTROLMODES::OSCMODE;
  display.setScreen(portal::SCREENMODES::OSCBANKS);  
}

bool __not_in_flash_func(processSwitchEvent)(size_t encoderIdx, debouncer &encDebouncer, size_t pin) {
  auto currentState = controls::encoderSwitches[encoderIdx];
  //gpio pull-up, so the value is inverted
  auto newState = 1 - encDebouncer.debounce(pin);  
  bool switchDownEvent = false;
  if (newState != currentState) {
    controls::encoderSwitches[encoderIdx] = newState;
    if (newState == 1) {
      switchDownEvent = true;
    }else{
      controls::encoderSwitchUPTS[encoderIdx] = millis();
      Serial.printf("enc %d switch up ts %d\n", encoderIdx, controls::encoderSwitchUPTS[encoderIdx]);
    }
  }
  return switchDownEvent;
}

void switchToTuningMode() {
  uint32_t save = spin_lock_blocking(displaySpinlock);  

  controlMode = CONTROLMODES::TUNINGMODE;
  display.setScreen(portal::SCREENMODES::TUNING);
  display.setTuning(TuningSettings::octaves, TuningSettings::semitones, TuningSettings::cents);
  display.setTuningBypass(TuningSettings::bypass);

  spin_unlock(displaySpinlock, save);
}

//central
void __isr encoder1_switch_callback() {

  bool switchDownEvent = processSwitchEvent(0, enc1Debouncer, ENCODER1_SWITCH);
  if (switchDownEvent) {
    switch(controlMode) {
      case CONTROLMODES::METAOSCMODE:
      {
        uint32_t save = spin_lock_blocking(displaySpinlock);  

        switchToOSCMode();
  
        spin_unlock(displaySpinlock, save);

        break;
      }
      case CONTROLMODES::OSCMODE:
      {
        uint32_t save = spin_lock_blocking(displaySpinlock);  

        controlMode = CONTROLMODES::METAOSCMODE;
        display.setScreen(portal::SCREENMODES::METAOSCVIS);
  
        spin_unlock(displaySpinlock, save);
        break;
      }
      case CONTROLMODES::TUNINGMODE:
      {
        TuningSettings::save();
        uint32_t save = spin_lock_blocking(displaySpinlock);  
        controlMode = CONTROLMODES::QUANTISESETTINGSMODE;
        display.setQuant(TuningSettings::quantPull, TuningSettings::quantNotesPerOct);
        display.setScreen(portal::SCREENMODES::QUANTISE);
        spin_unlock(displaySpinlock, save);
        break;
      }
    }
  }
  if (controlMode == CONTROLMODES::CALIBRATEMODE)
  {
    CalibrationSettings::reset();
    display.setCalibEncoderSwitch(0, controls::encoderSwitches[0]);
  }

}

//left
void __isr encoder2_switch_callback() {
  bool switchDownEvent = processSwitchEvent(1, enc2Debouncer, ENCODER2_SWITCH);
  if (switchDownEvent) {
    switch(controlMode) {
      case CONTROLMODES::OSCMODE:
      {
        uint32_t save = spin_lock_blocking(displaySpinlock);  
        controlMode = CONTROLMODES::UTILITYMODE;
        display.setScreen(portal::SCREENMODES::UTILITY);
        spin_unlock(displaySpinlock, save);
        break;
      }
      case CONTROLMODES::UTILITYMODE:
      {
        uint32_t save = spin_lock_blocking(displaySpinlock);  
        controlMode = CONTROLMODES::OSCMODE;
        switchToOSCMode();
        spin_unlock(displaySpinlock, save);
        break;
      }
      case CONTROLMODES::QUANTISESETTINGSMODE:
      {
        TuningSettings::save();
        switchToTuningMode();
        break;
      }
      case CONTROLMODES::TUNINGMODE:
      {
        TuningSettings::bypass = !TuningSettings::bypass;
        display.setTuningBypass(TuningSettings::bypass);
        break;
      }
    }
  }
  if (controlMode == CONTROLMODES::CALIBRATEMODE)
  {
    display.setCalibEncoderSwitch(1, controls::encoderSwitches[1]);
  }
}

//right
void __isr encoder3_switch_callback() {
  bool switchDownEvent = processSwitchEvent(2, enc3Debouncer, ENCODER3_SWITCH);
  if (switchDownEvent) {
  // controls::encoderSwitches[2] = 1 - enc3Debouncer.debounce(ENCODER3_SWITCH); 
  // if (controls::encoderSwitches[2]) {
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
        switchToTuningMode();
        break;
      }
      case CONTROLMODES::CALIBRATEMODE:
      {
        break;
      }
      case CONTROLMODES::TUNINGMODE:
      {
        TuningSettings::save();
        controlMode = CONTROLMODES::OSCMODE;
        display.setScreen(portal::SCREENMODES::OSCBANKS);
        break;
      }
      case CONTROLMODES::QUANTISESETTINGSMODE:
      {
        TuningSettings::save();
        controlMode = CONTROLMODES::OSCMODE;
        display.setScreen(portal::SCREENMODES::OSCBANKS);
        break;
      }
      case CONTROLMODES::UTILITYMODE:
      {
        Serial.println("save");
        for(size_t i=0; i < 3; i++) {
          MyriadState::setOscBank(i, oscBankTypes[i]);
        }
        MyriadState::setMetaMod(currMetaMod);
        MyriadState::setMetaModDepth(metaOscsList.at(currMetaMod)->getDepth());
        MyriadState::setMetaModSpeed(metaOscsList.at(currMetaMod)->getSpeed());        
        MyriadState::setModTarget(modTarget);
        MyriadState::save();

        Serial.printf("save %f %f\n", MyriadState::getMetaModDepth(), MyriadState::getMetaModSpeed());
        
        uint32_t save = spin_lock_blocking(displaySpinlock);  
        controlMode = CONTROLMODES::OSCMODE;
        switchToOSCMode();
        spin_unlock(displaySpinlock, save);
        break;
      }

    }
  }
  if (controlMode == CONTROLMODES::CALIBRATEMODE)
  {
    display.setCalibEncoderSwitch(2, controls::encoderSwitches[2]);
  }

 
}

std::shared_ptr<metaOscMLP<N_OSCILLATORS>> isCurrentMetaOscMLP() {
    if (currMetaMod >= metaOscsList.size()) {
        return nullptr;
    }
    
    auto ptr = metaOscsList.at(currMetaMod);
    if (ptr->getType() == MetaOscType::MLP) {
        return std::static_pointer_cast<metaOscMLP<N_OSCILLATORS>>(ptr);
    }
    return nullptr;    
}

void calibrate_button_callback() {
  controls::calibrateButton = 1 - calibrateButtonDebouncer.debounce(CALIBRATE_BUTTON); 

  if (controls::calibrateButton == 1) {
    auto mlpPtr = isCurrentMetaOscMLP();
    if (mlpPtr != nullptr) {
      mlpPtr->randomise();
      Serial.println("Randomise");
    }else{
      if (controlMode != CONTROLMODES::CALIBRATEMODE) {
        controlMode = CONTROLMODES::CALIBRATEMODE;
        //get free memory
        int freeMem = rp2040.getFreeHeap();  
        display.setFreeHeap(freeMem);
        display.setScreen(portal::SCREENMODES::CALIBRATE);
      }else{
        CalibrationSettings::save();
        switchToOSCMode();
      }
    }
  }
}

struct repeating_timer timerAdcProcessor;
// struct repeating_timer timerDisplay;
//struct repeating_timer timerFreqReceiver;
// struct repeating_timer timerOscModeChangeMonitor;



void setup() {

  // This will show if you're in an exception/interrupt
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, []() {
      Serial.println("HARD FAULT in interrupt!");
      while(1);
  });

  //USB Serial
  Serial.begin();
  // while(!Serial) {}

  calcOscsSpinlock = spin_lock_init(spin_lock_claim_unused(true));
  displaySpinlock = spin_lock_init(spin_lock_claim_unused(true));
  adcSpinlock = spin_lock_init(spin_lock_claim_unused(true));


  CalibrationSettings::load();
  // CalibrationSettings::init();
  TuningSettings::load();
  MyriadState::load();


  tft.init();  
  tft.setRotation(3);

  //collect IDs from the oscillator models
  std::vector<String> oscModelIDs;
  for(size_t i=0; i < N_OSCILLATOR_MODELS; i++) {
    oscModelIDs.push_back(oscModelFactories[i]()->getIdentifier());
  }
  display.init(oscModelIDs);
  display.setCalibScreenTitle(MYRIAD_VERSION);

  display.setScreen(portal::SCREENMODES::OSCBANKS);
  // display.setMetaOsc(0, metaOscsList[0]);

  // Now turn display on
  digitalWrite(TFT_BL, HIGH);  


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

  // add_repeating_timer_us(adcProcessorDiv, adcProcessor, NULL, &timerAdcProcessor);
  // add_repeating_timer_ms(39, displayUpdate, NULL, &timerDisplay);
  // add_repeating_timer_ms(31, oscModeChangeMonitor, NULL, &timerOscModeChangeMonitor);

  //load stored state
  oscBankTypes[0] = MyriadState::getOscBank(0);
  oscBankTypes[1] = MyriadState::getOscBank(1);
  oscBankTypes[2] = MyriadState::getOscBank(2);
  Serial.println(oscBankTypes[2]);
  
  //on this unit
  assignOscModels(oscBankTypes[2]);

  //on unit B
  sendToMyriadB(0, oscBankTypes[0]);
  sendToMyriadB(1, oscBankTypes[1]);


  display.setOscBankModel(0, oscBankTypes[0]);
  display.setOscBankModel(1, oscBankTypes[1]);
  display.setOscBankModel(2, oscBankTypes[2]);

  setMetaOscMode(MyriadState::getMetaMod());
  metaOscsList.at(currMetaMod)->restoreDepth(MyriadState::getMetaModDepth());
  metaOscsList.at(currMetaMod)->restoreSpeed(MyriadState::getMetaModSpeed());
  modTarget = MyriadState::getModTarget();
  display.setModTarget(modTarget);
  display.setMetaModDepth(metaOscsList.at(currMetaMod)->moddepth.getNormalisedValue());
  display.setMetaModSpeed(metaOscsList.at(currMetaMod)->modspeed.getNormalisedValue());

  display.update();

  
}



size_t FAST_MEM displayTS = 0;
size_t FAST_MEM ocmTS = 0;
size_t FAST_MEM adcTS = 0;


void __not_in_flash_func(loop)() {

  auto now = millis();

  // if (now - adcTS >= 20) {
  //   adcProcessor();
  //   adcTS = now;
  //   Serial.println(".");
  // }
  if (adcReadyFlag) {
    processAdc();
    adcReadyFlag = false;
  }
  
  if (now - displayTS >= 39) {
    uint32_t save = spin_lock_blocking(displaySpinlock);  

    display.update();
    displayTS = now;
  
    spin_unlock(displaySpinlock, save);

  }

  if (now - ocmTS >= 31) {
    oscModeChangeMonitor();
    ocmTS = now;
  }

  delayMicroseconds(100);
}


void setup1() {

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


void __not_in_flash_func(loop1)() {
  uint32_t save = spin_lock_blocking(calcOscsSpinlock);  
  if (oscsRunning) {
    calculateOscBuffers();
  }
  spin_unlock(calcOscsSpinlock, save);
}
