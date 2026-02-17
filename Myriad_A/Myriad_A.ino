#define MYRIAD_VERSION "1.0.0"

#include <FS.h>
#include <LittleFS.h>

#include "myriad_setup.h"

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

#include "fixedlpf.hpp"

#include "octaves.hpp"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

#include "freqlookup.h"
#include "myriad_messages.h"
#include "SLIP.h"


#include "tuning.hpp"
#include "state.hpp"

#include "exp2Table.hpp"

#include "perf.hpp"

#include "streamMessaging.hpp"

#include "utils.hpp"

// #define SINGLEOSCILLATOR

#include "fixedpoint.hpp"
using namespace FixedPoint;

#include "fastExpFixed.hpp"

#include "ADCProfile.hpp"


#define FRAC_BITS 16
// using ADCCalibType = ADCCalibrator<12,121, int32_t, FRAC_BITS>;
// ADCCalibType __not_in_flash("pitchadclookup") pitchADCMap(CalibrationSettings::pitchCalPoints);


bool core1_separate_stack = true;

constexpr size_t N_OSCILLATORS=9;
constexpr size_t N_OSC_BANKS=3;

using portal = displayPortal<N_OSCILLATORS,N_OSC_BANKS,N_OSCILLATOR_MODELS>;

portal FAST_MEM display;

enum CONTROLMODES {OSCMODE, METAOSCMODE, CALIBRATEMODE, CALIBRATEPITCHMODE, TUNINGMODE, UTILITYMODE, QUANTISESETTINGSMODE} controlMode = CONTROLMODES::OSCMODE;

#define RUN_OSCS

std::array<oscModelPtr, 3> FAST_MEM currOscModels;

bool FAST_MEM oscsReadyToStart=false;
// volatile bool FAST_MEM restartOscsFlag=false;

metaOscNoneFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscNone;
metaOscSinesFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscSines;
metaOscSinesFMultipleFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscSinesFMultiple;
metaLorenzFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscLozenz;
metaRosslerFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscRossler;
metaOscBoidsFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscBoids;
metaDrunkenWalkersFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscDrunkenWalkers;
metaOscMLPFP<N_OSCILLATORS> __not_in_flash("mydata") staticMetaOscNN;


// Array of base class pointers
std::array<metaOscFP<N_OSCILLATORS>*, 8> __not_in_flash("mydata") metaOscsFPList = {
    &staticMetaOscNone,
    &staticMetaOscBoids,
    &staticMetaOscLozenz,
    &staticMetaOscRossler,
    &staticMetaOscSines,
    &staticMetaOscSinesFMultiple,
    &staticMetaOscNN,
    &staticMetaOscDrunkenWalkers
};

size_t FAST_MEM currMetaMod = 0;

DEFINE_TIMING_SWAPBUFFERS(0, __scratch_y("swap"))
DEFINE_TIMING_SWAPBUFFERS(1, __scratch_y("swap"))
DEFINE_TIMING_SWAPBUFFERS(2, __scratch_y("swap"))

#define CORE1_FAST_MEM __scratch_y("myriad")

uint32_t CORE1_FAST_MEM smOsc0_dma_chan;
uint32_t CORE1_FAST_MEM smOsc0_dma_chan_bit;

uint32_t CORE1_FAST_MEM smOsc1_dma_chan;
uint32_t CORE1_FAST_MEM smOsc1_dma_chan_bit;

uint32_t CORE1_FAST_MEM smOsc2_dma_chan;
uint32_t CORE1_FAST_MEM smOsc2_dma_chan_bit;

bool FAST_MEM bufSent0 = false;
bool FAST_MEM bufSent1 = false;
bool FAST_MEM bufSent2 = false;


struct repeating_timer FAST_MEM timerMetaModUpdate;

/////////////////// messaging to Myriad B ///////////////////////

// static FAST_MEM streamMessaging::msgpacket msg;
static spin_lock_t FAST_MEM *serialSpinlock;

void __force_inline __not_in_flash_func(sendToMyriadB) (streamMessaging::messageTypes msgType, float value) {
  uint32_t save = spin_lock_blocking(serialSpinlock);  
  streamMessaging::msgpacket msg;
  streamMessaging::createMessage(msg, value, msgType);
  streamMessaging::sendMessageWithDMA(msg);
  spin_unlock(serialSpinlock, save);
}

void __force_inline __not_in_flash_func(sendToMyriadB) (streamMessaging::messageTypes msgType, size_t value) {
  uint32_t save = spin_lock_blocking(serialSpinlock);  
  streamMessaging::msgpacket msg;
  streamMessaging::createMessage(msg, value, msgType);
  streamMessaging::sendMessageWithDMA(msg);
  spin_unlock(serialSpinlock, save);
}

void __force_inline __not_in_flash_func(sendToMyriadB) (streamMessaging::messageTypes msgType, int32_t value) {
  uint32_t save = spin_lock_blocking(serialSpinlock);  
  streamMessaging::msgpacket msg;
  streamMessaging::createMessage(msg, value, msgType);
  streamMessaging::sendMessageWithDMA(msg);
  spin_unlock(serialSpinlock, save);
}



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

bool FAST_MEM oscsRunning = false;
// static spin_lock_t FAST_MEM *calcOscsSpinlock;
static spin_lock_t FAST_MEM *displaySpinlock;
static spin_lock_t FAST_MEM *adcSpinlock;

#define DMACH_CORE1_OSC0 0
#define DMACH_CORE1_OSC1 1
#define DMACH_CORE1_OSC2 2
#define DMACH_ADC_A 3
#define DMACH_ADC_B 4
#define DMACH_SERIAL_TX 5

void startOscBankA() {

  // Serial.println("StartoscbankA");

  //reset the pio block
  pio_clear_instruction_memory(pio1);
  uint programOffset = currOscModels[0]->loadProg(pio1);
  pio_sm_config baseConfig = currOscModels[0]->getBaseConfig(programOffset);
  // Serial.printf("Offset %d\n", programOffset);

  const size_t modelClockDiv = currOscModels[0]->getClockDiv();

  smOsc0_dma_chan = smOsc0.init(pio1, 0, OSC7_PIN, programOffset, baseConfig, nextTimingBuffer0, dma_irh, modelClockDiv, currOscModels[0]->loopLength, DMA_IRQ_1, DMACH_CORE1_OSC0);
  // Serial.println("init");
  // if (smOsc0_dma_chan < 0) {
    // Serial.println("dma chan allocation error");
  // }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();
  
  #ifndef SINGLEOSCILLATOR

  smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, baseConfig, nextTimingBuffer1, dma_irh, modelClockDiv, currOscModels[1]->loopLength, DMA_IRQ_1, DMACH_CORE1_OSC1);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, baseConfig, nextTimingBuffer2, dma_irh, modelClockDiv, currOscModels[2]->loopLength, DMA_IRQ_1, DMACH_CORE1_OSC2);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();
  // Serial.println("started");
#endif

  oscsRunning = true;
}


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

static size_t __not_in_flash("adc") controlValues[4] = {0,0,0,0};

static uint16_t __not_in_flash("adc") capture_buf[16] __attribute__((aligned(2048)));
static uint16_t __not_in_flash("adc") capture_buf_a[16] __attribute__((aligned(2048)));
static uint16_t __not_in_flash("adc") capture_buf_b[16] __attribute__((aligned(2048)));


size_t FAST_MEM oscBankTypes[3] = {0,0,0}; 

uint __not_in_flash("adc") dma_chan;
uint __not_in_flash("adc") dma_chan2;

FixedLpf<18,2> FAST_MEM adcLpf0;
FixedLpf<12,6> FAST_MEM adcLpf1;
FixedLpf<12,6> FAST_MEM adcLpf2;    
FixedLpf<12,6> FAST_MEM adcLpf3;
using WvlenFPType = Fixed<20,11>;

WvlenFPType __not_in_flash("adc") new_base_frequency(0);
WvlenFPType __not_in_flash("adc") new_wavelen0_fixed(0);
WvlenFPType __not_in_flash("adc") new_wavelen1_fixed(0);
WvlenFPType __not_in_flash("adc") new_wavelen2_fixed(0);
WvlenFPType __not_in_flash("adc") detuneFixed;

Q16_16 __not_in_flash("adc") metaModWavelenMul0 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul1 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul2 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul3 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul4 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul5 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul6 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul7 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModWavelenMul8 = Q16_16(1);
Q16_16 __not_in_flash("adc") metaModCtrlMul = Q16_16(1);

bool __not_in_flash("adc") metaModReady = false;
bool __not_in_flash("adc") octReady = false;

size_t __not_in_flash("adc") lastOctaveIdx = 0;

constexpr size_t systemUpdateFreq = 8000; //kkkkkkkkkkk

size_t __not_in_flash("adc") metaUpdateCounter = 0;

bool __not_in_flash("adc") newFrequenciesReady = false;

static size_t __not_in_flash("adc") adcCount = 0;
static size_t __not_in_flash("adc") adcAccumulator0=0;
static size_t __not_in_flash("adc") adc0Oversample=0;

#define oversampleBits 3
#define oversampleFactor (1<<oversampleBits)

Fixed<16,16> __not_in_flash("adc") epsilon_fixed(0);

PERF_DECLARE(ADC);
PERF_DECLARE(METAMODS);
PERF_DECLARE(SERIALTX);

size_t __scratch_y("adc") pitchCVAccumulator=0;

Fixed<14,18> pitchCopy;
Q16_16 pitchVCopy;
Q16_16 wavelenScaleCopy;


void __not_in_flash_func(adcProcessor)(uint16_t adcReadings[]) {
    //oversampling pitch
    pitchCVAccumulator += adcReadings[0];

    adcCount++;

    if (adcCount == oversampleFactor) {
      adcCount=0;
      pitchCVAccumulator = pitchCVAccumulator >> oversampleBits;
      adcLpf0.play(pitchCVAccumulator); 
      pitchCVAccumulator=0;
    }else{
      return;
    }

    
    PERF_BEGIN(ADC);
    
    size_t pitchADCRaw = adcLpf0.raw(); // in Q14:18 format
    Fixed<14,18> pitchADCQ1418 = Fixed<14,18>::from_raw(pitchADCRaw);
    pitchCopy = pitchADCQ1418;
    
    //-----new calibration method
    size_t adcProfileIdx = pitchADCQ1418.to_int();
    Fixed<14,18> frac = pitchADCQ1418.frac();  // Mask lower bits
    if (adcProfileIdx >= 4095) {
      adcProfileIdx = 4094;
      frac = Fixed<14,18>(1);
    }
    // Look up adjacent entries
    Fixed<14,18> val0 = Fixed<14,18>(ADCProfile::cal_data.correction[adcProfileIdx]);
    Fixed<14,18> val1 = Fixed<14,18>(ADCProfile::cal_data.correction[adcProfileIdx + 1]);

    Fixed<14,18> diff = val1 - val0;
    Fixed<14,18> interpolated = diff * frac; 

    Fixed<14,18> correction = val0 + interpolated;

    // size_t correction = ADCProfile::cal_data.correction[adcProfileIdx];
    pitchADCQ1418 = pitchADCQ1418 + correction; // apply correction

    //18, 4078    
    // constexpr Q16_16 adcVRangeInv = Q16_16(10) / Q16_16(4078); 
    Q16_16 pitchCV_Q16 = (Q16_16(pitchADCQ1418) * ADCProfile::adcVRangeInv); // map to 0-10V range
    
    //old calibration method
    // auto pitchCV_Q16_raw = pitchADCMap.convertFixedInterpolated_Q14_18(pitchADCQ1418);
    // Q16_16 pitchCV_Q16 = Fixed<16,16,int32_t>::from_raw(pitchCV_Q16_raw);

    pitchVCopy = pitchCV_Q16;
    controlValues[0] = pitchADCQ1418.to_int();


    //quantise?
    // if (!TuningSettings::bypass && TuningSettings::quantPull > 0.f) {
    //   const float quantCV = std::round(pitchCV * TuningSettings::quantStepInv) * TuningSettings::quantStep;

    //   const float diff = quantCV - pitchCV;
    //   pitchCV = pitchCV + (diff * TuningSettings::quantAlpha);
    // }
    if (!TuningSettings::bypass && TuningSettings::quantPull > Q16_16(0)) {
      const Q16_16 quantCV = round(pitchCV_Q16 * TuningSettings::quantStepInv) * TuningSettings::quantStep;

      const Q16_16 diff = quantCV - pitchCV_Q16;
      pitchCV_Q16 = pitchCV_Q16 + (diff * TuningSettings::quantAlpha);
    }
    
    // Q16_16 freq_Q16 = exp_fast(pitchCV_Q16); 
    // pitchVExpCopy = freq_Q16;
    static Q16_16 minusOne_q16 = Q16_16(-1.0f);
    Q16_16 wavelenScale = exp2_fast(minusOne_q16 * pitchCV_Q16); //1/freq
    wavelenScaleCopy = wavelenScale;
      
    const WvlenFPType wvlenFixed = TuningSettings::bypass ? TuningSettings::wavelenC1Fixed : TuningSettings::baseWavelenFP; //Fixed point wavelength at C1

    new_base_frequency = wvlenFixed.mulWith(wavelenScale);


    ///////////////////////// detune 
    static Q16_16 mask4095 = Q16_16::from_raw(0x0FFF0000);
    static Q16_16 half_q16 = Q16_16(0.5f);

    adcLpf1.play(adcReadings[1]);
    Q16_16 filteredADC1_Q16 = Q16_16(adcLpf1.value()) + half_q16;
    filteredADC1_Q16  &= mask4095; 

    int correctionADC1 = ADCProfile::cal_data.correction[filteredADC1_Q16.to_int()];
    filteredADC1_Q16 = filteredADC1_Q16 + Q16_16(correctionADC1);

    controlValues[1] = filteredADC1_Q16.to_int();

    //detuning
    static Fixed<0,18> inv4096_q0_18 = Fixed<0,18>(1.f/4096.f);
    Fixed<0,18> ctrlValFixed = inv4096_q0_18.mulWith_split(filteredADC1_Q16);
    ctrlValFixed = ctrlValFixed * ctrlValFixed; //exponential mapping
    static Fixed<0,18> detuneScale = Fixed<0,18>(0.016f);
    Fixed<0,18> ctrlValScaled = ctrlValFixed.mul_fast(detuneScale);
    detuneFixed = new_base_frequency.mulWith(ctrlValScaled);

    ////////////////////////  epsilon

    adcLpf2.play(adcReadings[2]);
    Q16_16 filteredADC2_Q16 = Q16_16(adcLpf2.value()) + half_q16;
    filteredADC2_Q16  &= mask4095; 
    int correctionADC2 = ADCProfile::cal_data.correction[filteredADC2_Q16.to_int()];
    filteredADC2_Q16 = filteredADC2_Q16 + Q16_16(correctionADC2);
    controlValues[2] = filteredADC2_Q16.to_int();

    static Q16_16 inv4096_q16 = Q16_16(1.f/4096.f);
    epsilon_fixed = filteredADC2_Q16 * inv4096_q16;
    epsilon_fixed *= metaModCtrlMul;

    //octaves
    adcLpf3.play(adcReadings[3]);
    Q16_16 filteredADC3_Q16 = Q16_16(adcLpf3.value()); // + Q16_16(0.5f);
    filteredADC3_Q16  &= mask4095; 

    int correctionADC3 = ADCProfile::cal_data.correction[filteredADC3_Q16.to_int()];
    filteredADC3_Q16 = filteredADC3_Q16 + Q16_16(correctionADC3);
    controlValues[3] = filteredADC3_Q16.to_int();
    static Q16_16 octIdxScale = Q16_16(18.9999f/4095.f);
    size_t octaveIdx = (filteredADC3_Q16 * octIdxScale).to_int(); 

    static size_t octaveCtrlMap[19] = {
      0, 1, 2, 3, 4, 5, 6, 7, 
      8,8,8, 
      9,10,11,12, 13,14,15,16
    };

    octaveIdx = octaveCtrlMap[octaveIdx];

    if (octaveIdx != lastOctaveIdx) {
      lastOctaveIdx = octaveIdx;
      octReady = true;
      sendToMyriadB(streamMessaging::messageTypes::OCTSPREAD, octaveIdx);
      currentOctaveShifts = (int8_t *)octaveTableShift[octaveIdx];
    }

    oscsReadyToStart = true;
    newFrequenciesReady = true;

    PERF_BEGIN(SERIALTX);
    sendToMyriadB(streamMessaging::messageTypes::WAVELEN0, new_base_frequency.raw());
    PERF_END(SERIALTX);
    PERF_END(ADC);

}


__attribute__((hot, flatten))
void __not_in_flash_func(dma_irq_handler)() {
    // Check both channels in one go - likely to be compiled to efficient bit ops
    uint32_t irq_status = dma_hw->ints0;
    
    uint16_t *adcReadings = nullptr;
    
    // Branch 1: Check and handle dma_chan
    if (irq_status & (1u << dma_chan)) {
        dma_hw->ints0 = (1u << dma_chan);  // Clear IRQ directly
        adcReadings = capture_buf_a;
    }
    
    // Branch 2: Check and handle dma_chan2  
    if (irq_status & (1u << dma_chan2)) {
        dma_hw->ints0 = (1u << dma_chan2);  // Clear IRQ directly
        adcReadings = capture_buf_b;  // Will overwrite if both fired
    }
    
    // Process if either fired
    if (__builtin_expect(adcReadings != nullptr, 1)) {  // Likely hint
        adcProcessor(adcReadings);
    }
    
    // Stream messaging check - separate hardware
    // if (__builtin_expect(streamMessaging::dma_channel_tx >= 0, 0)) {  // Unlikely
    if (irq_status & (1u << streamMessaging::dma_channel_tx)) {
        dma_hw->ints0 = (1u << streamMessaging::dma_channel_tx);
    }
    // }
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
  constexpr size_t adcSystemFreq = systemUpdateFreq * 4 * oversampleFactor; //allow for 4 readings
  adc_set_clkdiv((48000000 / adcSystemFreq) - 1);

  //setup two dma channels in ping pong
  dma_chan = DMACH_ADC_A; // dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

  dma_chan2 = DMACH_ADC_B; // dma_claim_unused_channel(true);
  dma_channel_config cfg2 = dma_channel_get_default_config(dma_chan2);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_ring(&cfg, true, 3);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);
  channel_config_set_chain_to(&cfg, dma_chan2);


  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg2, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg2, false);
  channel_config_set_write_increment(&cfg2, true);
  channel_config_set_ring(&cfg2, true, 3);

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
  irq_add_shared_handler(DMA_IRQ_0, dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(DMA_IRQ_0, true);  

  adc_run(true);



  oscsReadyToStart = true;

}





// inline float __not_in_flash_func(adcMap)(const size_t adcIndex) {
//   //get the value
//   uint32_t save = spin_lock_blocking(adcSpinlock);  
//   const float val = controlValues[adcIndex];
//   spin_unlock(adcSpinlock, save);

//   //mapping
//   float mappedVal = (val - (CalibrationSettings::adcMins[adcIndex])) * CalibrationSettings::adcRangesInv[adcIndex];
//   if (mappedVal < 0.f) mappedVal = 0.f;

//   return mappedVal;
// }

void __not_in_flash_func(resetMetaMods)() {
  metaModWavelenMul0 = Q16_16(1);
  metaModWavelenMul1 = Q16_16(1);
  metaModWavelenMul2 = Q16_16(1);
  metaModWavelenMul3 = Q16_16(1);
  metaModWavelenMul4 = Q16_16(1);
  metaModWavelenMul5 = Q16_16(1);
  metaModWavelenMul6 = Q16_16(1);
  metaModWavelenMul7 = Q16_16(1);
  metaModWavelenMul8 = Q16_16(1);
}

bool __not_in_flash_func(metaModUpdate)(__unused struct repeating_timer *t) {
  PERF_BEGIN(METAMODS);
  if (currMetaMod > 0) {
    uint32_t save = spin_lock_blocking(adcSpinlock);
    auto metamods = metaOscsFPList.at(currMetaMod)->update(controlValues);
    spin_unlock(adcSpinlock, save);

    if (modTarget == MODTARGETS::PITCH_AND_EPSILON || modTarget == MODTARGETS::PITCH ) {
        metaModWavelenMul0 = (Q16_16(1) + (metamods[0]));
        metaModWavelenMul1 = (Q16_16(1) + (metamods[1]));
        metaModWavelenMul2 = (Q16_16(1) + (metamods[2]));
        metaModWavelenMul3 = (Q16_16(1) + (metamods[3]));
        metaModWavelenMul4 = (Q16_16(1) + (metamods[4]));
        metaModWavelenMul5 = (Q16_16(1) + (metamods[5]));
        metaModWavelenMul6 = (Q16_16(1) + (metamods[6]));
        metaModWavelenMul7 = (Q16_16(1) + (metamods[7]));
        metaModWavelenMul8 = (Q16_16(1) + (metamods[8]));
    }

    if (modTarget == MODTARGETS::PITCH_AND_EPSILON || modTarget == MODTARGETS::EPSILON ) {
        metaModCtrlMul = (
          Q16_16(1) 
            + (
                (
                  metamods[0]
                  + metamods[1]
                  + metamods[2]
                  + metamods[3]
                  + metamods[4]
                  + metamods[5]
                  + metamods[6]
                  + metamods[7]
                  + metamods[8]
                ) * Q16_16(7/9.f)
              )
          ); 
    }

    metaModReady = true;
  }
  PERF_END(METAMODS);
  return true;
}

// inline bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
//   display.update();
//   return true;
// }

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

bool FAST_MEM changeBankFlag = false;

// inline bool __not_in_flash_func(oscModeChangeMonitor)(__unused struct repeating_timer *t) {
inline bool __not_in_flash_func(oscModeChangeMonitor)() {
  for(size_t bank=0; bank < 3; bank++) {
    if (oscModeBankChangeTS[bank] > 0) {
      size_t elapsed = millis() - oscModeBankChangeTS[bank];
      if (elapsed > 80) {
        // Serial.printf("bank %d change: %d\n", bank, oscBankTypes[bank]);
        
        oscModeBankChangeTS[bank] = 0;

        if (bank ==2) {
          changeBankFlag = true;
        } else {

          sendToMyriadB(bank == 1 ? streamMessaging::messageTypes::BANK1 : streamMessaging::messageTypes::BANK0, oscBankTypes[bank]);
          // MyriadState::setOscBank(bank, oscBankTypes[bank]);
        
        }
      }
    }

  }

  return true;
}

void __not_in_flash_func(setMetaOscMode)(size_t mode) {
  currMetaMod = mode;
  display.setMetaOsc(currMetaMod, metaOscsFPList[currMetaMod]);

  cancel_repeating_timer(&timerMetaModUpdate);
  add_repeating_timer_ms(metaOscsFPList[currMetaMod]->getTimerMS(), metaModUpdate, NULL, &timerMetaModUpdate);

  if (mode ==0) {
    resetMetaMods();
    metaModCtrlMul = Q16_16(1);            
    metaModReady = true; //send to unit B
  }

}

void updateTuning() {
  TuningSettings::update();
  display.setTuning(TuningSettings::octaves, TuningSettings::semitones, TuningSettings::cents);
  display.setTuningBypass(TuningSettings::bypass);
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
        newMetaModMode = min(metaOscsFPList.size()-1, newMetaModMode);

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
        auto newVal = TuningSettings::quantNotesPerOct + Q16_16(change);
        if (newVal < Q16_16(1)) {
          newVal = Q16_16(1);
        }
        if (newVal > Q16_16(39)) {
          newVal = Q16_16(39);
        }
        TuningSettings::quantNotesPerOct = newVal;
        TuningSettings::updateQuant();
        display.setQuant(TuningSettings::quantPull.to_int(), TuningSettings::quantNotesPerOct.to_int());

        break;
      }
      // case CONTROLMODES::CALIBRATEPITCHMODE: 
      // {
      //   int newVal = CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex] + change;
      //   if (newVal >=0 && newVal < 5000) {
      //     CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex] = newVal;
      //     display.setPitchCalibValue(CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex]);
      //   }
      //   break;
      // }

    }
  }
}

//left
// struct encAccelData {
//   size_t prevChangeTS = 0;
//   float acc = 0.f;
// };

struct encAccelDataFP {
  size_t prevChangeTS = 0;
  Q16_16 acc = Q16_16(0);
};

// encAccelData FAST_MEM enc2Accel;
encAccelDataFP FAST_MEM enc2AccelFP;

// float __not_in_flash_func(calcAcceleration)(const int change, encAccelData &data) {
//   auto now = millis();
//   if (change != 0) {
//     auto gap = millis() - data.prevChangeTS;
//     data.acc += std::max(50.f-gap, -50.f)*0.012f;
//   }
//   data.acc *= 0.95f;
//   data.prevChangeTS = now;
//   float changeWithAcc = change * (1.f + data.acc);
//   if (change == 1 && changeWithAcc < 0.f) {
//     changeWithAcc = 0.f;
//   }
//   if (change == -1 && changeWithAcc > 0.f) {
//     changeWithAcc = 0.f;
//   }
//   // Serial.printf("change %d acc %f -> %f\n", change, accEnc2, changeWithAcc);
//   return changeWithAcc;
// }

Q16_16 __not_in_flash_func(calcAccelerationFP)(const int change, encAccelDataFP &data) {
  auto now = millis();
  if (change != 0) {
    long gap = millis() - data.prevChangeTS;
    data.acc += Q16_16(static_cast<int>(std::max(50L-gap, -50L)))*Q16_16(0.012f);
  }
  data.acc *= Q16_16(0.95f);
  data.prevChangeTS = now;
  Q16_16 changeWithAcc = Q16_16(change) * (Q16_16(1.f) + data.acc);
  if (change == 1 && changeWithAcc < Q16_16(0)) {
    changeWithAcc = Q16_16(0);
  }
  if (change == -1 && changeWithAcc > Q16_16(0)) {
    changeWithAcc = Q16_16(0);
  }
  return changeWithAcc;
}
//left encoder
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
            const Q16_16 changeWithAcc = calcAccelerationFP(change, enc2AccelFP);
            metaOscsFPList.at(currMetaMod)->setSpeed(changeWithAcc);
            display.setMetaModSpeed(metaOscsFPList.at(currMetaMod)->modspeed.getNormalisedValue());
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
          TuningSettings::octaves = max(-3, TuningSettings::octaves);
          TuningSettings::octaves = min(3, TuningSettings::octaves);
          updateTuning();
          break;
        }
        case CONTROLMODES::QUANTISESETTINGSMODE:
        {
          auto newVal = TuningSettings::quantPull + Q16_16(change);
          if (newVal < Q16_16(0)) {
            newVal = Q16_16(0);
          }
          if (newVal > Q16_16(100)) {
            newVal = Q16_16(100);
          }
          TuningSettings::quantPull = newVal;
          TuningSettings::updateQuant();
          display.setQuant(TuningSettings::quantPull.to_int(), TuningSettings::quantNotesPerOct.to_int());

          break;
        }
        // case CONTROLMODES::CALIBRATEPITCHMODE: 
        // {
        //   int newVal = PITCHCALSCREEEN::pointIndex + change;
        //   if (newVal >=0 && newVal < pitchADCMap.NPoints) {
        //     PITCHCALSCREEEN::pointIndex = newVal;
        //     display.setPitchCalibPoint(PITCHCALSCREEEN::pointIndex);
        //     display.setPitchCalibValue(CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex]);
        //   }
        //   break;
        // }

      }
    }
}

//right
// encAccelData FAST_MEM enc3Accel;
encAccelDataFP FAST_MEM enc3AccelFP;

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
          const Q16_16 changeWithAcc = calcAccelerationFP(change, enc3AccelFP);
          metaOscsFPList.at(currMetaMod)->setDepth(changeWithAcc);
          display.setMetaModDepth(metaOscsFPList.at(currMetaMod)->moddepth.getNormalisedValue());
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
      // Serial.printf("enc %d switch up ts %d\n", encoderIdx, controls::encoderSwitchUPTS[encoderIdx]);
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
        display.setQuant(TuningSettings::quantPull.to_int(), TuningSettings::quantNotesPerOct.to_int());
        display.setScreen(portal::SCREENMODES::QUANTISE);
        spin_unlock(displaySpinlock, save);
        break;
      }
    }
  }
  if (controlMode == CONTROLMODES::CALIBRATEMODE)
  {
    // CalibrationSettings::reset();
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
      case CONTROLMODES::CALIBRATEMODE:
      {
        // //assume 1v/oct input is grounded
        // Serial.println("Calibrate pitch");
        // CalibrationSettings::setADC0Mid(controlValues[0]);
        // pitchADCMap.rebuildFromThreePointEstimate(CalibrationSettings::adcMins[0], controlValues[0], CalibrationSettings::adcMaxs[0]);
        break;
      }
      // case CONTROLMODES::CALIBRATEPITCHMODE:
      // {
      //   pitchADCMap.rebuildLookupTable(CalibrationSettings::pitchCalPoints);
      //   break;
      // }

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
            metaModCtrlMul = Q16_16(1);            
            break;
          }
          case MODTARGETS::EPSILON: {
            modTarget = MODTARGETS::PITCH_AND_EPSILON;
            resetMetaMods();
            //clear out old metamod info
            break;
          }
          case MODTARGETS::PITCH_AND_EPSILON: {
            modTarget = MODTARGETS::PITCH;
            break;
          }
        }
        display.setModTarget(modTarget);
        metaModReady = true; //send any changes to unit B
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
        MyriadState::setMetaModDepth(metaOscsFPList.at(currMetaMod)->getDepth());
        MyriadState::setMetaModSpeed(metaOscsFPList.at(currMetaMod)->getSpeed());
        MyriadState::setModTarget(modTarget);
        MyriadState::save();

        Serial.printf("save %f %f\n", MyriadState::getMetaModDepth(), MyriadState::getMetaModSpeed());
        
        uint32_t save = spin_lock_blocking(displaySpinlock);  
        controlMode = CONTROLMODES::OSCMODE;
        switchToOSCMode();
        spin_unlock(displaySpinlock, save);
        break;
      }
      // case CONTROLMODES::CALIBRATEPITCHMODE:
      // {
      //   //rebuild pitch table
      //   if (!PITCHCALSCREEEN::calRunning) {
      //     PITCHCALSCREEEN::calRunning = true;
      //     PITCHCALSCREEEN::lastPitchADC = controlValues[0];
      //     //store current reading
      //     CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex] = controlValues[0];
      //     display.setPitchCalibValue(controlValues[0]);

      //   }else{
      //     PITCHCALSCREEEN::calRunning = false;
      //     pitchADCMap.rebuildLookupTable(CalibrationSettings::pitchCalPoints);
      //   }
      //   display.setPitchCalibRunning(PITCHCALSCREEEN::calRunning);
      //   break;
      // }

    }
  }
  if (controlMode == CONTROLMODES::CALIBRATEMODE)
  {
    display.setCalibEncoderSwitch(2, controls::encoderSwitches[2]);
  }

 
}

std::shared_ptr<metaOscMLP<N_OSCILLATORS>> isCurrentMetaOscMLP() {
    if (currMetaMod >= metaOscsFPList.size()) {
        return nullptr;
    }
    //TODO: restore
    // auto ptr = metaOscsFPList.at(currMetaMod);
    // if (ptr->getType() == MetaOscType::MLP) {
    //     return std::static_pointer_cast<metaOscMLP<N_OSCILLATORS>>(ptr);
    // }
    return nullptr;    
}

bool FAST_MEM calibButtonState = false;

void calibrate_button_callback() {
  controls::calibrateButton = 1 - calibrateButtonDebouncer.debounce(CALIBRATE_BUTTON); 
  auto currentState = calibButtonState;
  //gpio pull-up, so the value is inverted
  auto newState = controls::calibrateButton;
  bool switchDownEvent = false;
  if (newState != currentState) {
    calibButtonState = newState;
    if (newState == 1) {
      switchDownEvent = true;
    } 
  }  
  if (switchDownEvent) {
    auto mlpPtr = isCurrentMetaOscMLP();
    if (mlpPtr != nullptr) {
      mlpPtr->randomise();
      Serial.println("Randomise");
    }else{
      switch(controlMode) {
        case CONTROLMODES::CALIBRATEMODE: {
        //   controlMode = CONTROLMODES::CALIBRATEPITCHMODE;
        //   display.setScreen(portal::SCREENMODES::PITCHCALIBRATE);
        //   display.setPitchCalibPoint(PITCHCALSCREEEN::pointIndex);
        //   display.setPitchCalibValue(CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex]);

        //   break;
        // }
        // case CONTROLMODES::CALIBRATEPITCHMODE: {
        //   PITCHCALSCREEEN::calRunning = false;
        //   display.setPitchCalibRunning(false);          
          // CalibrationSettings::save();
          switchToOSCMode();
          break;
        }
        default: {
          controlMode = CONTROLMODES::CALIBRATEMODE;
          //get free memory
          int freeMem = rp2040.getFreeHeap();  
          display.setFreeHeap(freeMem);
          display.setScreen(portal::SCREENMODES::CALIBRATE);
        }
      }

    }
  }
}


volatile bool core0ReadyFlag = 0;

void dump_midscale_correction() {
    Serial.println("code, histogram, correction");
    for (int i = 2020; i < 2080; i++) {
        Serial.printf("%d, %lu, %d\n", i, ADCProfile::histogram[i], ADCProfile::cal_data.correction[i]);
    }
}

void check_correction_discontinuities() {
    Serial.println("Large jumps in correction table:");
    for (int i = 1; i < 4095; i++) {
        int16_t diff = ADCProfile::cal_data.correction[i] - ADCProfile::cal_data.correction[i-1];
        if (diff > 2 || diff < -2) {
            Serial.printf("  code %d: correction jumps from %d to %d (diff %d)\n",
                          i, 
                          ADCProfile::cal_data.correction[i-1],
                          ADCProfile::cal_data.correction[i],
                          diff);
        }
    }
}

void find_pitch_discontinuity() {
    Serial.println("Sweeping through correction table for large output jumps...\n");
    
    Q16_16 last_pitch = Q16_16(0);
    bool first = true;
    
    for (int adc = 100; adc < 4000; adc++) {
        // Simulate your pitch calculation
        Fixed<14,18> pitchADCQ1418 = Fixed<14,18>(adc);
        
        // Your DNL correction (simplified - no interpolation for this test)
        int16_t correction = ADCProfile::cal_data.correction[adc];
        pitchADCQ1418 = pitchADCQ1418 + Fixed<14,18>(correction);
        
        // Your endpoint calibration
        Q16_16 pitchCV_Q16 = Q16_16(pitchADCQ1418);
        Q16_16 pitch;
        
        constexpr Q16_16 tmpADCMin(0);
        constexpr Q16_16 tmpADCMid(2059);
        constexpr Q16_16 tmpADCMax(4095);
        
        if (pitchCV_Q16 < tmpADCMid) {
            pitch = (pitchCV_Q16 - tmpADCMin) / (tmpADCMid - tmpADCMin) * Q16_16(5);
        } else {
            pitch = (pitchCV_Q16 - tmpADCMid) / (tmpADCMax - tmpADCMid) * Q16_16(5) + Q16_16(5);
        }
        
        if (!first) {
            Q16_16 diff = pitch - last_pitch;
            // Normal step should be ~0.00244 (10V / 4096)
            // Flag anything > 0.01 (4x normal)
            if (diff > Q16_16(0.01) || diff < Q16_16(-0.01)) {
                float diff_mv = diff.to_float() * 1000.0f;
                float diff_cents = diff_mv / 0.833f;  // 83.3mV per semitone
                Serial.printf("ADC %d: jump of %.1f mV (%.1f cents)\n", 
                              adc, diff_mv, diff_cents);
            }
        }
        
        last_pitch = pitch;
        first = false;
    }
}

void setup() {

  // This will show if you're in an exception/interrupt
  // exception_set_exclusive_handler(HARDFAULT_EXCEPTION, []() {
  //     Serial.println("HARD FAULT in interrupt!");
  //     while(1);
  // });

  //claim dma channels before tft init
  dma_channel_claim(DMACH_CORE1_OSC0);
  dma_channel_claim(DMACH_CORE1_OSC1);
  dma_channel_claim(DMACH_CORE1_OSC2);
  dma_channel_claim(DMACH_ADC_A);
  dma_channel_claim(DMACH_ADC_B);
  dma_channel_claim(DMACH_SERIAL_TX);

  init_exp2_table();

  // //USB Serial
  Serial.begin();
  // while(!Serial) {}

  // calcOscsSpinlock = spin_lock_init(spin_lock_claim_unused(true));
  displaySpinlock = spin_lock_init(spin_lock_claim_unused(true));
  adcSpinlock = spin_lock_init(spin_lock_claim_unused(true));
  serialSpinlock = spin_lock_init(spin_lock_claim_unused(true));

  //set up serial tx to Myriad B
  bool serialTXOK = streamMessaging::setupTX(pio0, nullptr, 13, 12, DMACH_SERIAL_TX);
  if (!serialTXOK) {
    Serial.println("Error creating serial tx");
  }

  // while(!Serial) {}
  Serial.println("Myriad A starting...");

  ADCProfile::load_calibration_from_file();

  // dump_midscale_correction();
  // check_correction_discontinuities();
  find_pitch_discontinuity();
  // CalibrationSettings::load();

  // pitchADCMap.rebuildFromThreePointEstimate(CalibrationSettings::adcMins[0], CalibrationSettings::adc0Mid, CalibrationSettings::adcMaxs[0]);
  // CalibrationSettings::init();
  // pitchADCMap.rebuildLookupTable(CalibrationSettings::pitchCalPoints);

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
  // digitalWrite(TFT_BL, HIGH);  // TODO: TFT_BL not defined  

  gpio_set_drive_strength(12, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_slew_rate(12, GPIO_SLEW_RATE_FAST);
  // comms to Myriad B
  // Serial1.setRX(13);
  // Serial1.setTX(12);
  // Serial1.begin(SERIAL_CX_BAUD);

  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);


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


  //load stored state
  oscBankTypes[0] = MyriadState::getOscBank(0);
  oscBankTypes[1] = MyriadState::getOscBank(1);
  oscBankTypes[2] = MyriadState::getOscBank(2);
  // Serial.println(oscBankTypes[2]);
  
  //on this unit
  assignOscModels(oscBankTypes[2]);

  //on unit B
  sendToMyriadB(streamMessaging::messageTypes::BANK0, oscBankTypes[0]);
  sendToMyriadB(streamMessaging::messageTypes::BANK1, oscBankTypes[1]);


  display.setOscBankModel(0, oscBankTypes[0]);
  display.setOscBankModel(1, oscBankTypes[1]);
  display.setOscBankModel(2, oscBankTypes[2]);

  setMetaOscMode(MyriadState::getMetaMod());
  metaOscsFPList.at(currMetaMod)->restoreDepth(MyriadState::getMetaModDepth());
  metaOscsFPList.at(currMetaMod)->restoreSpeed(MyriadState::getMetaModSpeed());
  modTarget = MyriadState::getModTarget();
  display.setModTarget(modTarget);
  display.setMetaModDepth(metaOscsFPList.at(currMetaMod)->moddepth.getNormalisedValue());
  display.setMetaModSpeed(metaOscsFPList.at(currMetaMod)->modspeed.getNormalisedValue());

  constexpr WvlenFPType Fixed0(0);
  display.setDisplayWavelengths({Fixed0,Fixed0,Fixed0,Fixed0,Fixed0,Fixed0,Fixed0,Fixed0,Fixed0});

  core0ReadyFlag = 1;



  
}



size_t FAST_MEM displayTS = 0;
size_t FAST_MEM ocmTS = 0;
size_t FAST_MEM adcTS = 0;
size_t FAST_MEM ctrlTS = 0;
size_t FAST_MEM detuneTS = 0;
size_t FAST_MEM freqTS=0;
size_t FAST_MEM metaModSendIdx=0;

size_t FAST_MEM dotTS;

PERF_DECLARE(CALCOSCS);
PERF_DECLARE(DISPLAY);
size_t vu;

void __not_in_flash_func(loop)() {
    // sendToMyriadB(streamMessaging::messageTypes::CTRL, 17U);
    // delay(1);

  auto now = micros();

  // PERIODIC_RUN_US(
  //     auto metamods = metaOscsFPList.at(currMetaMod)->getValues();
  //   ,1000
  // );


  // if (newFrequenciesReady && now - freqTS >= 125) {

  //   // PERF_BEGIN(SERIALTX);
  //   // sendToMyriadB(streamMessaging::messageTypes::WAVELEN0, new_wavelen0);
  //   // PERF_END(SERIALTX);

  //   newFrequenciesReady = false;
  //   freqTS = now;
  // }else{
  //stagger meta mod sends
  if (metaModReady) {
    sendToMyriadB(streamMessaging::messageTypes::METAMOD8, metaModWavelenMul8.raw());
    metaModSendIdx=5;
    metaModReady=false;
  }
  switch(metaModSendIdx) {
    case 5:
    metaModSendIdx = 4;
    sendToMyriadB(streamMessaging::messageTypes::METAMOD3, metaModWavelenMul3.raw());
    break;
    case 4:
    metaModSendIdx = 3;
    sendToMyriadB(streamMessaging::messageTypes::METAMOD4, metaModWavelenMul4.raw());
    break;
    case 3:
    metaModSendIdx = 2;
    sendToMyriadB(streamMessaging::messageTypes::METAMOD5, metaModWavelenMul5.raw());
    break;
    case 2:
    metaModSendIdx = 1;
    sendToMyriadB(streamMessaging::messageTypes::METAMOD6, metaModWavelenMul6.raw());
    break;
    case 1:
    metaModSendIdx = 0;
    sendToMyriadB(streamMessaging::messageTypes::METAMOD7, metaModWavelenMul7.raw());
    break;
  }
    

  if (now - ctrlTS >= 1000) {
    sendToMyriadB(streamMessaging::messageTypes::CTRL0, epsilon_fixed.raw());
    ctrlTS = now;
  }
  if (now - detuneTS >= 1000) {
    sendToMyriadB(streamMessaging::messageTypes::DETUNE, detuneFixed.raw());
    detuneTS = now;
  }

  // if (octReady) {
  //   sendToMyriadB(streamMessaging::messageTypes::OCTSPREAD, lastOctaveIdx);
  //   octReady = false;
  // }  



  if (now - displayTS >= 1000000/20){
    PERF_BEGIN(DISPLAY);
    if (controlMode == CONTROLMODES::OSCMODE && oscsReadyToStart) {
      //same calc as on myriad B, but just for display
      WvlenFPType new_wavelen3_fixed = (new_wavelen0_fixed - detuneFixed - detuneFixed - detuneFixed);
      WvlenFPType new_wavelen4_fixed = (new_wavelen3_fixed - detuneFixed);
      WvlenFPType new_wavelen5_fixed = (new_wavelen4_fixed - detuneFixed);
      WvlenFPType new_wavelen6_fixed = (new_wavelen5_fixed - detuneFixed);
      WvlenFPType new_wavelen7_fixed = (new_wavelen6_fixed - detuneFixed);
      WvlenFPType new_wavelen8_fixed = (new_wavelen7_fixed - detuneFixed);

      // Serial.printf("%f %f %f\n", new_wavelen6_fixed.to_float(),   new_wavelen7_fixed.to_float(), new_wavelen8_fixed.to_float());
      new_wavelen3_fixed = new_wavelen3_fixed.mulWith(metaModWavelenMul3);
      new_wavelen4_fixed = new_wavelen4_fixed.mulWith(metaModWavelenMul4);
      new_wavelen5_fixed = new_wavelen5_fixed.mulWith(metaModWavelenMul5);
      new_wavelen6_fixed = new_wavelen6_fixed.mulWith(metaModWavelenMul6);
      new_wavelen7_fixed = new_wavelen7_fixed.mulWith(metaModWavelenMul7);
      new_wavelen8_fixed = new_wavelen8_fixed.mulWith(metaModWavelenMul8);

      new_wavelen3_fixed = currentOctaveShifts[1] > 0 ? new_wavelen3_fixed >> currentOctaveShifts[1] : new_wavelen3_fixed << -currentOctaveShifts[1];
      new_wavelen4_fixed = currentOctaveShifts[1] > 0 ? new_wavelen4_fixed >> currentOctaveShifts[1] : new_wavelen4_fixed << -currentOctaveShifts[1];
      new_wavelen5_fixed = currentOctaveShifts[1] > 0 ? new_wavelen5_fixed >> currentOctaveShifts[1] : new_wavelen5_fixed << -currentOctaveShifts[1];
      new_wavelen6_fixed = currentOctaveShifts[2] > 0 ? new_wavelen6_fixed >> currentOctaveShifts[2] : new_wavelen6_fixed << -currentOctaveShifts[2];
      new_wavelen7_fixed = currentOctaveShifts[2] > 0 ? new_wavelen7_fixed >> currentOctaveShifts[2] : new_wavelen7_fixed << -currentOctaveShifts[2];
      new_wavelen8_fixed = currentOctaveShifts[2] > 0 ? new_wavelen8_fixed >> currentOctaveShifts[2] : new_wavelen8_fixed << -currentOctaveShifts[2];

      uint32_t save = spin_lock_blocking(displaySpinlock);  

      display.setDisplayWavelengths({
              new_wavelen0_fixed,
              new_wavelen1_fixed,
              new_wavelen2_fixed,
              new_wavelen3_fixed,
              new_wavelen4_fixed,
              new_wavelen5_fixed,
              new_wavelen6_fixed,
              new_wavelen7_fixed,
              new_wavelen8_fixed});

      spin_unlock(displaySpinlock, save);

    }else if (controlMode == CONTROLMODES::CALIBRATEMODE) {
      // for(size_t i=1; i < 4; i++) {
      //   uint32_t save = spin_lock_blocking(adcSpinlock);  
      //   if (controlValues[i] < CalibrationSettings::adcMins[i] && controlValues[i] >= 0) {
      //     CalibrationSettings::adcMins[i] = controlValues[i];
      //   }
      //   if (controlValues[i] > CalibrationSettings::adcMaxs[i] && controlValues[i] < 4096) {
      //     CalibrationSettings::adcMaxs[i] = controlValues[i];
      //   }
      //   CalibrationSettings::adcRanges[i] = CalibrationSettings::adcMaxs[i] - CalibrationSettings::adcMins[i];
      //   if (CalibrationSettings::adcRanges[i]==0) CalibrationSettings::adcRanges[i]=1;
      //   CalibrationSettings::adcRangesInv[i] = 1.f / CalibrationSettings::adcRanges[i];
      //   spin_unlock(adcSpinlock, save);
      // }
      // display.setCalibADCValues(adcAccumulator0, adcAccumulator1, adcAccumulator2, adcAccumulator3);
      // display.setCalibADCValues(controlValues[0], controlValues[1], controlValues[2], controlValues[3]);
      display.setCalibWavelen0(new_wavelen0_fixed.to_int());
      display.setCalibADCFiltValues(controlValues[0], controlValues[1], controlValues[2], controlValues[3]);
      // display.setCalibADCMinMaxValues(CalibrationSettings::adcMins, CalibrationSettings::adcMaxs);
    }
    // else if (controlMode == CONTROLMODES::CALIBRATEPITCHMODE)
    // {
    //     display.setPitchCalibReading(controlValues[0]);
    //     int diff = controlValues[0] - PITCHCALSCREEEN::lastPitchADC;
    //     bool change=false;

    //     if (diff >15) {
    //       PITCHCALSCREEEN::lastPitchADC = controlValues[0];
    //       change=true;
    //     }

    //     if (PITCHCALSCREEEN::calRunning) {
    //       if (change) {
    //         int newVal = PITCHCALSCREEEN::pointIndex+1;
    //         if (newVal < pitchADCMap.NPoints) {
    //           PITCHCALSCREEEN::pointIndex = newVal;
    //           //set display
    //           display.setPitchCalibPoint(PITCHCALSCREEEN::pointIndex);
    //           display.setPitchCalibValue(CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex]);

    //           //record calibration
    //           CalibrationSettings::pitchCalPoints[PITCHCALSCREEEN::pointIndex] = controlValues[0];
    //         }else{
    //           // PITCHCALSCREEEN::calRunning = false;
    //           // display.setPitchCalibRunning(false);
    //         }
    //       }
    //     }
    // }

    uint32_t save = spin_lock_blocking(displaySpinlock);  
    display.update();
    spin_unlock(displaySpinlock, save);

    displayTS = now;
    PERF_END(DISPLAY);

  }

  if (now - ocmTS >= 31000) {
    oscModeChangeMonitor();
    ocmTS = now;
  }


  // if (now - dotTS > 500000) {
  //   Serial.printf("adc: %d\tstx: %d\tdsp:%d\tmod: %d\tf: %f\td: %d\ta:%d\tp: %f\twvs: %f\twv: %f\n", PERF_GET_MEAN(ADC), PERF_GET_MEAN(SERIALTX), PERF_GET_MEAN(CALCOSCS), PERF_GET_MEAN(METAMODS), PERF_GET_FREQ(ADC), PERF_GET_MEAN(DISPLAY), controlValues[0], pitchVCopy.to_float(), wavelenScaleCopy.to_float(), new_wavelen0_fixed.to_float());
  //   dotTS = now;
  // }
}


void setup1() {

  while(!core0ReadyFlag) {
    tight_loop_contents();
  }

  //start processing
  setup_adcs();  

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
  if (changeBankFlag) {
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

    dma_hw->ints1 = smOsc0_dma_chan_bit | smOsc1_dma_chan_bit | smOsc2_dma_chan_bit;

    memset(timing_swapbuffer_0_A, 0, sizeof(timing_swapbuffer_0_A));
    memset(timing_swapbuffer_0_B, 0, sizeof(timing_swapbuffer_0_B));
    memset(timing_swapbuffer_1_A, 0, sizeof(timing_swapbuffer_1_A));
    memset(timing_swapbuffer_1_B, 0, sizeof(timing_swapbuffer_1_B));
    memset(timing_swapbuffer_2_A, 0, sizeof(timing_swapbuffer_2_A));
    memset(timing_swapbuffer_2_B, 0, sizeof(timing_swapbuffer_2_B));

    // Reset buffer pointers to A buffers
    nextTimingBuffer0 = (io_rw_32)timing_swapbuffer_0_A;
    nextTimingBuffer1 = (io_rw_32)timing_swapbuffer_1_A;
    nextTimingBuffer2 = (io_rw_32)timing_swapbuffer_2_A;          

    auto w1 = currOscModels[0]->getWavelen();
    auto w2 = currOscModels[1]->getWavelen();
    auto w3 = currOscModels[2]->getWavelen();

    assignOscModels(oscBankTypes[2]);

    //refill from new oscillator
    //trigger buffer refills
    currOscModels[0]->reset();
    currOscModels[1]->reset();
    currOscModels[2]->reset();

    currOscModels[0]->setWavelen(w1);
    currOscModels[1]->setWavelen(w2);
    currOscModels[2]->setWavelen(w3);

    calculateOscBuffers();

    startOscBankA();

    changeBankFlag = false;
  }
  if (oscsRunning) {
    PERF_BEGIN(CALCOSCS);
    PERIODIC_RUN_US(
      currOscModels[0]->ctrl(epsilon_fixed);
      currOscModels[1]->ctrl(epsilon_fixed);
      currOscModels[2]->ctrl(epsilon_fixed);
    , 1000);
    if (newFrequenciesReady) {
      new_wavelen0_fixed = new_base_frequency;
      new_wavelen1_fixed = (new_wavelen0_fixed - detuneFixed);
      new_wavelen2_fixed = (new_wavelen1_fixed - detuneFixed);

      new_wavelen0_fixed = new_wavelen0_fixed.mulWith(metaModWavelenMul0);
      new_wavelen1_fixed = new_wavelen1_fixed.mulWith(metaModWavelenMul1);
      new_wavelen2_fixed = new_wavelen2_fixed.mulWith(metaModWavelenMul2);

      new_wavelen0_fixed = currentOctaveShifts[0] > 0 ? new_wavelen0_fixed >> currentOctaveShifts[0] : new_wavelen0_fixed << -currentOctaveShifts[0];
      new_wavelen1_fixed = currentOctaveShifts[0] > 0 ? new_wavelen1_fixed >> currentOctaveShifts[0] : new_wavelen1_fixed << -currentOctaveShifts[0];
      new_wavelen2_fixed = currentOctaveShifts[0] > 0 ? new_wavelen2_fixed >> currentOctaveShifts[0] : new_wavelen2_fixed << -currentOctaveShifts[0];
      currOscModels[0]->setWavelen(new_wavelen0_fixed.to_int());  
      currOscModels[1]->setWavelen(new_wavelen1_fixed.to_int());
      currOscModels[2]->setWavelen(new_wavelen2_fixed.to_int());
      newFrequenciesReady=false;
    }

    calculateOscBuffers();
    PERF_END(CALCOSCS);
  }
}
