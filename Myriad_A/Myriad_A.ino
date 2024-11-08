
#include <TFT_eSPI.h>
#include "myriad_pins.h"
// #include "myriad_setup.h"
#include "pio_expdec.h"
#include "smBitStreamOsc.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

#include "MedianFilter.h"
#include "MAFilter.h"

// #include "mode_saw.h"
// #include "mode_sqr.h"
// #include "mode_tri.h"
#include "freqlookup.h"
#include "myriad_messages.h"
#include "SLIP.h"

#include "drawing.h"
#include "boids.h"

#define FAST_MEM __not_in_flash("mydata")

float clockdiv = 10;
uint32_t clockHz = 15625000 / 80;
size_t cpuClock=125000000;
float sampleClock = cpuClock / clockdiv;
uint32_t mwavelen = 125000000/clockdiv /50;
uint32_t mwavelen2 = mwavelen * 1.01;
uint32_t mwavelen3 = mwavelen2 * 1.01;
// Ensure `timing_buffer` is aligned to 16-bytes so we can use DMA address
// wrapping
float sqrTemplate[8] {
  0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5
};
// uint32_t timing_buffer[8] __attribute__((aligned(16))) {clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1};
// uint32_t timing_buffer2[8] __attribute__((aligned(16))) {clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1};
  
uint32_t FAST_MEM timing_buffer[8] __attribute__((aligned(16))) {
  mwavelen,mwavelen>>1,mwavelen,mwavelen>>1,mwavelen,mwavelen>>1,mwavelen,mwavelen>>1
 
  };
uint32_t timing_buffer3[8] __attribute__((aligned(16))) {
  mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2
 
  };
uint32_t timing_buffer4[8] __attribute__((aligned(16))) {
  mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3,
 
  };

// uint32_t pio_dma_chan;

io_rw_32 nextTimingBuffer = (io_rw_32)timing_buffer;
io_rw_32  nextTimingBuffer2 = (io_rw_32)timing_buffer3;
io_rw_32  nextTimingBuffer3 = (io_rw_32)timing_buffer4;


uint programOffset = pio_add_program(pio0, &pin_ctrl_program);

uint32_t FAST_MEM smOsc0_dma_chan;
uint32_t FAST_MEM smOsc0_dma_chan_bit;

uint32_t FAST_MEM smOsc1_dma_chan;
uint32_t FAST_MEM smOsc1_dma_chan_bit;
// uint32_t FAST_MEM smOsc1_dma_chan_bit_inv;

uint32_t FAST_MEM smOsc2_dma_chan;
uint32_t FAST_MEM smOsc2_dma_chan_bit;

void __not_in_flash_func(dma_irh)() {
  uint32_t triggered_channels = dma_hw->ints1;
  if (triggered_channels & (1u << smOsc0_dma_chan)) {
    dma_hw->ints1 = (1u << smOsc0_dma_chan);  
  //   // Channel 0 triggered, handle it
    // dma_hw->ints0 = smOsc0_dma_chan_bit;
    dma_hw->ch[smOsc0_dma_chan].al3_read_addr_trig = nextTimingBuffer;
  }
  // else
  // if (triggered_channels & smOsc1_dma_chan_bit) {
  // //   // Channel 0 triggered, handle it
  //   dma_hw->ints0 = smOsc1_dma_chan_bit;
  // //   dma_hw->ints0 &= smOsc0_dma_chan_bit_inv;  // Clear interrupt 
  //     dma_hw->ch[smOsc1_dma_chan].al3_read_addr_trig = nextTimingBuffer2;
  // }
  // else
  // if (triggered_channels & smOsc2_dma_chan_bit) {
  // //   // Channel 0 triggered, handle it
  //   dma_hw->ints0 = smOsc2_dma_chan_bit;
  // //   dma_hw->ints0 &= smOsc0_dma_chan_bit_inv;  // Clear interrupt 
  //   dma_hw->ch[smOsc2_dma_chan].al3_read_addr_trig = nextTimingBuffer3;
  // }
}
// smBitStreamOsc smOsc0;
// smBitStreamOsc smOsc1;
// smBitStreamOsc smOsc2;



// bool core1_separate_stack = true;

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

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

SerialPIO uartOut(13,SerialPIO::NOPIN, 64);


namespace controls {
  static int encoderValues[3] = {0,0,0};
  static bool encoderSwitches[3] = {0,0,0};
};

enum METAMODMODES {NONE, BOIDS} metaModMode;
boidsSim boids;

MovingAverageFilter<int> adcFilters[4];
static int __not_in_flash("mydata") controlValues[4] = {0,0,0,0};


static uint16_t __not_in_flash("mydata") capture_buf[16] __attribute__((aligned(2048)));

static float __not_in_flash("mydata") octave0=0;
static float __not_in_flash("mydata") octave1=0;
static float __not_in_flash("mydata") octave2=0;
static float __not_in_flash("mydata") octave3=0;
static float __not_in_flash("mydata") octave4=0;
static float __not_in_flash("mydata") octave5=0;

int __not_in_flash("mydata") oscTypeBank0=0;
int __not_in_flash("mydata") oscTypeBank1=0;

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


void __not_in_flash_func(sendToMyriadB) (uint8_t msgType, float value) {
  static uint8_t __not_in_flash("mydata") slipBuffer[64];
  // if(spi_is_writable(spi1)) {
  spiMessage msg {msgType, value};
  unsigned int slipSize = SLIP::encode(reinterpret_cast<uint8_t*>(&msg), sizeof(spiMessage), &slipBuffer[0]);
    // for(int i=0; i < slipSize; i++) {
    //   Serial.print(slipBuffer[i]);
    //   Serial.print("\t");
    // }
    // Serial.println("");
    // int res = spi_write_blocking(spi1, reinterpret_cast<uint8_t*>(&slipBuffer), slipSize);
  int res = uartOut.write(reinterpret_cast<uint8_t*>(&slipBuffer), slipSize);


    // spiMessage decodeMsg;
    // SLIP::decode(slipBuffer, slipSize, reinterpret_cast<uint8_t*>(&decodeMsg));
    // Serial.println(decodeMsg.value);
    // Serial.print("r: ");
    // Serial.println(res);
  // }
}

bool __not_in_flash_func(adcProcessor)(__unused struct repeating_timer *t) {
  static size_t __not_in_flash("mydata") lastOctaveIdx = 0;
  controlValues[0] = capture_buf[0];
  controlValues[1] = capture_buf[1];
  controlValues[2] = capture_buf[2];
  controlValues[3] = capture_buf[3];

  // controlValues[0] = adcFilters[0].process(capture_buf[0]);
  // controlValues[1] = adcFilters[1].process(capture_buf[1]);
  // controlValues[2] = adcFilters[2].process(capture_buf[2]);
  // controlValues[3] = adcFilters[3].process(capture_buf[3]);

  size_t octaveIdx = controlValues[3] >> 8;  // div by 256 -> 16 divisions
  if (octaveIdx != lastOctaveIdx) {
    lastOctaveIdx = octaveIdx;
    switch(octaveIdx) {
      case 0:
      {
        octave0 = 0;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 0;
        octave5 = 0;
        break;
      }
      case 1:
      {
        octave0 = 1;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 0;
        octave5 = 0;
        break;
      }
      case 2:
      {
        octave0 = 2;
        octave1 = 1;
        octave2 = 0;
        octave3 = 0;
        octave4 = 0;
        octave5 = 0;
        break;
      }
      case 3:
      {
        octave0 = 3;
        octave1 = 2;
        octave2 = 1;
        octave3 = 0;
        octave4 = 0;
        octave5 = 0;
        break;
      }
      case 4:
      {
        octave0 = 3;
        octave1 = 2;
        octave2 = 1;
        octave3 = 0;
        octave4 = 0;
        octave5 = 1;
        break;
      }
      case 5:
      {
        octave0 = 3;
        octave1 = 2;
        octave2 = 1;
        octave3 = 0;
        octave4 = 1;
        octave5 = 2;
        break;
      }
      case 6:
      {
        octave0 = 3;
        octave1 = 2;
        octave2 = 1;
        octave3 = 1;
        octave4 = 2;
        octave5 = 3;
        break;
      }
      case 7:
      {
        octave0 = 2;
        octave1 = 2;
        octave2 = 0;
        octave3 = 0;
        octave4 = 2;
        octave5 = 2;
        break;
      }
      case 8:
      {
        octave0 = 2;
        octave1 = 1;
        octave2 = 0;
        octave3 = 0;
        octave4 = 1;
        octave5 = 2;
        break;
      }
      case 9:
      {
        octave0 = 2;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 0;
        octave5 = 1;
        break;
      }
      case 10:
      {
        octave0 = 1;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 0;
        octave5 = 2;
        break;
      }
      case 11:
      {
        octave0 = 0;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 2;
        octave5 = 2;
        break;
      }
      case 12:
      {
        octave0 = 0;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 2;
        octave5 = 3;
        break;
      }
      case 13:
      {
        octave0 = 0;
        octave1 = 0;
        octave2 = 0;
        octave3 = 1;
        octave4 = 2;
        octave5 = 3;
        break;
      }
      case 14:
      {
        octave0 = 0;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 1;
        octave5 = 2;
        break;
      }
      case 15:
      {
        octave0 = 0;
        octave1 = 0;
        octave2 = 0;
        octave3 = 0;
        octave4 = 0;
        octave5 = 1;
        break;
      }
      default:;
    }
  }

  // setFrequencies(freqtable[controlValues[0]], controlValues[1] >> 2, controlValues[2] >> 2);
  int detune = controlValues[1] >> 2;
  int acc = controlValues[2] >> 2;
  float new_wavelen0 = freqtable[controlValues[0]];
  float new_wavelen1 = new_wavelen0 + detune + acc;
  float new_wavelen2 = new_wavelen1 + detune + acc;
  float new_wavelen3 = new_wavelen2 + detune + acc;
  float new_wavelen4 = new_wavelen3 + detune + acc;
  float new_wavelen5 = new_wavelen4 + detune + acc;

  new_wavelen0 = new_wavelen0 * octave0;
  // new_wavelen1 = new_wavelen1 << octave1;
  // new_wavelen2 = new_wavelen2 << octave2;
  // new_wavelen3 = new_wavelen3 >> octave3;
  // new_wavelen4 = new_wavelen4 >> octave4;
  // new_wavelen5 = new_wavelen5 >> octave5;

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
  // sendToMyriadB(messageTypes::WAVELEN1, new_wavelen1);
  // sendToMyriadB(messageTypes::WAVELEN2, new_wavelen2);
  // sendToMyriadB(messageTypes::WAVELEN3, new_wavelen3);
  // sendToMyriadB(messageTypes::WAVELEN4, new_wavelen4);
  // sendToMyriadB(messageTypes::WAVELEN5, new_wavelen5);
  
  // Serial.print("1: ");
  // Serial.println(new_wavelen1);
  
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

void updateOscBank(int &currOscBank, messageTypes OSCBANKMSG, int change) {
  int newOscTypeBank = currOscBank + change;
  //clip
  newOscTypeBank = max(0, newOscTypeBank);
  newOscTypeBank = min(maxOscBankType, newOscTypeBank);
  if (newOscTypeBank != currOscBank) {
    //send
    currOscBank = newOscTypeBank;
    sendToMyriadB(OSCBANKMSG, currOscBank);
    Serial.println(currOscBank);  
  } 

}

void encoder1_callback() {
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  controls::encoderValues[0] += change;
  updateOscBank(oscTypeBank0, messageTypes::BANK0, change);
  Serial.println(controls::encoderValues[0]);  
  //update
  // int newOscTypeBank0 = oscTypeBank0 + change;
  // //clip
  // newOscTypeBank0 = max(0, newOscTypeBank0);
  // newOscTypeBank0 = min(maxOscBankType, newOscTypeBank0);
  // if (newOscTypeBank0 != oscTypeBank0) {
  //   //send
  //   oscTypeBank0 = newOscTypeBank0;
  //   sendToMyriadB(messageTypes::BANK0, oscTypeBank0);
  //   Serial.println(oscTypeBank0);  
  // } 
}

void encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  controls::encoderValues[1] += change;
  // Serial.println(controls::encoderValues[1]);  
  // updateOscBank(oscTypeBank1, messageTypes::BANK1, change);

}

void encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
  controls::encoderValues[2] += change;
  Serial.println(controls::encoderValues[2]);  
}

void encoder1_switch_callback() {
  controls::encoderSwitches[0] = digitalRead(ENCODER1_SWITCH);  
}

void encoder2_switch_callback() {
  controls::encoderSwitches[1] = digitalRead(ENCODER2_SWITCH);  
}

void encoder3_switch_callback() {
  controls::encoderSwitches[2] = digitalRead(ENCODER3_SWITCH);  
}


struct repeating_timer timerAdcProcessor;
struct repeating_timer timerDisplay;
struct repeating_timer timerFreqReceiver;



void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ELI_BLUE);
  tft.setFreeFont(&FreeMono9pt7b);


  uartOut.begin(115200);
  Serial.begin(115200);
  metaModMode = METAMODMODES::NONE;
  boids.init();

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
  
  //SPI 
  // spi_init(spi1, spiFrequency);
  // gpio_set_function(10, GPIO_FUNC_SPI); //SCK
  // gpio_set_function(11, GPIO_FUNC_SPI); //TX
  // gpio_set_function(12, GPIO_FUNC_SPI); //RX
  // gpio_set_function(13, GPIO_FUNC_SPI); //CS




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
  tft.fillScreen(0x0007);
  tft.setCursor(120,120);
  tft.setTextColor(ELI_PINK);
  tft.println(controls::encoderValues[0]);
  // // tft.drawLine(100,100,140,80, ELI_PINK);
  // // tft.drawLine(140,80,140,100, ELI_PINK);

  

  // // tft.setCursor(120,150);
  // // tft.println(controlValues[0]);
  switch(metaModMode) {
    case NONE:
    break;
    case BOIDS:
      boids.draw(tft);
      boids.update();
    break;
  }

  // static uint8_t __not_in_flash("mydata") slipBuffer[64];
  // spiMessage msg {0, count++};
  // unsigned int slipSize = SLIP::encode(reinterpret_cast<uint8_t*>(&msg), sizeof(spiMessage), &slipBuffer[0]);
  // int res = uartOut.write(reinterpret_cast<uint8_t*>(&slipBuffer), slipSize);
  // uartOut.print(count++ % 255);
  // Serial.println(count);
  delay(50);
  // __wfi();
}


void setup1() {
  // oscillator pins
  // setupOscPin(OSC1_PIN);
  // setupOscPin(OSC2_PIN);
  // setupOscPin(OSC3_PIN);

  // add_repeating_timer_ms(-25, core1FrequencyReceiver, NULL, &timerFreqReceiver);


  //oscillator clock state machine
  // uint offset = pio_add_program(pio1, &dsp_clock_program);
  // dsp_clock_forever(pio1, 0, offset, pdmFreq);

  //oscillator clock interrupt
  // irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw_coreA1);
  // irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_sqr_coreA1);
  // irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_tri_coreA1);
  
  // irq_set_enabled(PIO1_IRQ_0, true);
  // irq_set_priority(PIO1_IRQ_0, 10);
  // pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;
  // smOsc0_dma_chan = smOsc0.init(pio0, 0, 5, programOffset, timing_buffer, dma_irh, clockdiv);
  // smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;

  // smOsc1_dma_chan = smOsc1.init(pio0, 1, 6, programOffset, timing_buffer, dma_irh, clockdiv);
  // smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;

  // smOsc2_dma_chan = smOsc2.init(pio0, 2, 7, programOffset, timing_buffer, dma_irh, clockdiv);
  // smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;



}

void loop1() {
  // __wfi();
  tight_loop_contents();
}
