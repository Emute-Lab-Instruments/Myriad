
#include <optional>

#include <TFT_eSPI.h>
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

#include "drawing.h"
#include "boids.h"

#define FAST_MEM __not_in_flash("mydata")

bool core1_separate_stack = true;

constexpr size_t N_OSCILLATORS=9;

#define RUN_OSCS
oscModelPtr FAST_MEM currOscModelBank0;
volatile bool FAST_MEM oscsReadyToStart=false;
volatile bool FAST_MEM restartOscsFlag=false;



// metaOscSines<N_OSCILLATORS> currMetaOsc;
metaOscMLP<N_OSCILLATORS> currMetaOsc;

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
  uint programOffset = currOscModelBank0->loadProg(pio1);
  // updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModelBank0, 50000);

  smOsc0_dma_chan = smOsc0.init(pio1, 0, OSC7_PIN, programOffset, nextTimingBuffer0, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_1);
  Serial.println("init");
  if (smOsc0_dma_chan < 0) {
    Serial.println("dma chan allocation error");
  }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();

  smOsc1_dma_chan = smOsc1.init(pio1, 1, OSC8_PIN, programOffset, nextTimingBuffer1, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_1);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  smOsc2_dma_chan = smOsc2.init(pio1, 2, OSC9_PIN, programOffset, nextTimingBuffer2, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_1);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();
}

void stopOscBankA() {
  smOsc0.stop();
  smOsc1.stop();
  smOsc2.stop();
}




// bool core1_separate_stack = true;

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

class displayPortal {
public:
  void drawOsc0(int osc) {
    tft.setCursor(50,120);
    tft.setTextColor(ELI_BLUE);
    tft.println(oldOsc0);
    tft.setCursor(50,120);
    tft.setTextColor(ELI_PINK);
    tft.println(osc);
    oldOsc0 = osc;
  }
  void drawOsc1(int osc) {
    tft.setCursor(120,120);
    tft.setTextColor(ELI_BLUE);
    tft.println(oldOsc1);
    tft.setCursor(120,120);
    tft.setTextColor(ELI_PINK);
    tft.println(osc);
    oldOsc1 = osc;
  }
  void drawOsc2(int osc) {
    tft.setCursor(190,120);
    tft.setTextColor(ELI_BLUE);
    tft.println(oldOsc2);
    tft.setCursor(190,120);
    tft.setTextColor(ELI_PINK);
    tft.println(osc);
    oldOsc2 = osc;
  }
private:
  int oldOsc0=0;
  int oldOsc1=0;
  int oldOsc2=0;
};

displayPortal display;

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

SerialPIO uartOut(13, SerialPIO::NOPIN, 64);


namespace controls {
  static int FAST_MEM encoderValues[3] = {0,0,0};
  static int FAST_MEM encoderAltValues[3] = {0,0,0};
  static bool FAST_MEM encoderSwitches[3] = {0,0,0};
};

enum METAMODMODES {NONE, BOIDS} metaModMode;
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
  int res = uartOut.write(reinterpret_cast<uint8_t*>(&slipBuffer), slipSize);
}

inline float __not_in_flash_func(adcMap)(const size_t adcIndex) {
  return (controlValues[adcIndex] - adcMins[adcIndex]) / adcRanges[adcIndex];
}

// void updateTimingBuffer(io_rw_32 &nextBuf,
//                                   uint32_t* bufferA, uint32_t* bufferB,
//                                   const std::vector<float>& sqrTemplate,
//                                   float oscWavelength) {
//     if (nextBuf == reinterpret_cast<io_rw_32>(bufferA)) {
//         for (size_t i = 0; i < sqrTemplate.size(); ++i) {
//             *(bufferB + i) = static_cast<uint32_t>(sqrTemplate[i] * oscWavelength);
//         }
//         nextBuf = reinterpret_cast<io_rw_32>(bufferB);
//     } else {
//         for (size_t i = 0; i < sqrTemplate.size(); ++i) {
//             *(bufferA + i) = static_cast<uint32_t>(sqrTemplate[i] * oscWavelength);
//         }
//         nextBuf = reinterpret_cast<io_rw_32>(bufferA);
//     }
// }


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
  // Serial.printf("%f \n", controlValues[0]);

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

  float acc =  1.f - (adcMap(2) * 0.02f);

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

  auto sines = currMetaOsc.update(controlValues);
  // for(int i=0; i < 6; i++) {
  //   Serial.print(sines[i]);
  //   Serial.print("\t");
  // }
  // Serial.println();

  new_wavelen0 *= (1.f + (sines[0]));
  new_wavelen1 *= (1.f + (sines[1]));
  new_wavelen2 *= (1.f + (sines[2]));
  new_wavelen3 *= (1.f + (sines[3]));
  new_wavelen4 *= (1.f + (sines[4]));
  new_wavelen5 *= (1.f + (sines[5]));
  new_wavelen6 *= (1.f + (sines[6]));
  new_wavelen7 *= (1.f + (sines[7]));
  new_wavelen8 *= (1.f + (sines[8]));


  //send new values

  // {
  // queueItem msg {0, new_wavelen0};
  // queue_try_add(&coreCommsQueue, &msg);
  // }
  // {
  // queueItem msg {1, new_wavelen1};
  // queue_try_add(&coreCommsQueue, &msg);
  // }
  // {  
  // queueItem msg {2, new_wavelen2};
  // queue_try_add(&coreCommsQueue, &msg);
  // }
  // wavelen0 = new_wavelen0;  
  // wavelen1 = new_wavelen1;  
  // wavelen2 = new_wavelen2;  
  // wavelen0 = 30000;  
  // wavelen1 = 30010;  
  // wavelen2 = 30020;  

  sendToMyriadB(messageTypes::WAVELEN0, new_wavelen0);
  sendToMyriadB(messageTypes::WAVELEN1, new_wavelen1);
  sendToMyriadB(messageTypes::WAVELEN2, new_wavelen2);
  sendToMyriadB(messageTypes::WAVELEN3, new_wavelen3);
  sendToMyriadB(messageTypes::WAVELEN4, new_wavelen4);
  sendToMyriadB(messageTypes::WAVELEN5, new_wavelen5);
  
  updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModelBank0, new_wavelen6);
  updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModelBank0, new_wavelen7);
  updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModelBank0, new_wavelen8);

  oscsReadyToStart = true;

  return true;
}
// bool __not_in_flash_func(core1FrequencyReceiver)(__unused struct repeating_timer *t) {
//   queueItem q;
//   bool itemWaiting = queue_try_remove(&coreCommsQueue, &q);
//   if (itemWaiting) {
//     switch(q.idx) {
//       case 0:
//         wavelen0 = q.value;
//         break;
//       case 1:
//         wavelen1 = q.value;
//         break;
//       case 2:
//         wavelen2 = q.value;
//         break;
//     }
//   }
//   return true;
// }


bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
  // tft.fillScreen(TFT_BLACK);
  // tft.setCursor(120,120);
  // tft.setTextColor(TFT_WHITE);
  // tft.println(controls::encoderValues[0]);

  // tft.setCursor(120,150);
  // tft.println(controlValues[0]);
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

void updateOscBank(int &currOscBank, int change, std::optional<messageTypes> OSCBANKMSG = std::nullopt) {
  int newOscTypeBank = currOscBank + change;
  //clip
  newOscTypeBank = max(0, newOscTypeBank);
  newOscTypeBank = min(maxOscBankType, newOscTypeBank);
  if (newOscTypeBank != currOscBank) {
    //send
    currOscBank = newOscTypeBank;
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
        currOscModelBank0 = oscModel1; //std::make_unique<squareOscillatorModel>();
        break;
        case 1:
        currOscModelBank0 = oscModel2; //std::make_unique<squareOscillatorModel2>();
        break;              
      }
      // Serial.println("Starting");
      startOscBankA();
      // restartOscsFlag = true;

    }
    Serial.println(currOscBank);  
  } 

}

void encoder1_callback() {
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  if (controls::encoderSwitches[0]) {
    controls::encoderAltValues[0] += change;
  }else{
    controls::encoderValues[0] += change;
    updateOscBank(oscTypeBank0, change, messageTypes::BANK0);
    display.drawOsc0(oscTypeBank0);
  }
}

void encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);

  if (controls::encoderSwitches[1]) {
    controls::encoderAltValues[1] += change;
    currMetaOsc.setSpeed(controls::encoderAltValues[1]);
  }else{
    controls::encoderValues[1] += change;
    updateOscBank(oscTypeBank1, change, messageTypes::BANK1);
    display.drawOsc1(oscTypeBank1);
  }

  // controls::encoderValues[1] += change;
  // // Serial.println(controls::encoderValues[1]);  
  // updateOscBank(oscTypeBank1, change, messageTypes::BANK1);
  // display.drawOsc1(oscTypeBank1);

}

void encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
  if (controls::encoderSwitches[2]) {
    controls::encoderAltValues[2] += change;
    currMetaOsc.setDepth(controls::encoderAltValues[2]);
  }else{
    controls::encoderValues[2] += change;
    updateOscBank(oscTypeBank2, change, std::nullopt);
    display.drawOsc2(oscTypeBank2);
  }
}

void encoder1_switch_callback() {
  //gpio pull-up, so the value is inverted
  controls::encoderSwitches[0] = 1 - digitalRead(ENCODER1_SWITCH);  
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


  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ELI_BLUE);
  tft.setFreeFont(&FreeSans18pt7b);

  for(size_t i=0; i < 4; i++) {
    adcRanges[i] = adcMaxs[i] - adcMins[i];
  }


  uartOut.begin(115200);
  Serial.begin(115200);
  metaModMode = METAMODMODES::NONE;
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

  // add_repeating_timer_ms(100, displayUpdate, NULL, &timerDisplay);
  

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



}


int count=0;
/* Fonction loop() */
void loop() {
  // __wfi();
  // tft.fillScreen(0x0007);
  // // tft.drawLine(100,100,140,80, ELI_PINK);
  // // tft.drawLine(140,80,140,100, ELI_PINK);

  

  // // tft.setCursor(120,150);
  // // tft.println(controlValues[0]);
  // switch(metaModMode) {
  //   case NONE:
  //   break;
  //   case BOIDS:
  //     boids.draw(tft);
  //     boids.update();
  //   break;
  // }

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
  delay(1);
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
  currOscModelBank0 = oscModel2;
#ifdef RUN_OSCS
//wait for the first ADC readings
  while(!oscsReadyToStart) {
    ;
  }
  Serial.println("Start");
  startOscBankA();
  Serial.println("Started");
#endif

}

void loop1() {
  // __wfi();
  // if (restartOscsFlag) {
  //   restartOscsFlag=false;
  //   startOscBankA();
  // }
  tight_loop_contents();
}
