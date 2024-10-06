
#include <TFT_eSPI.h>
#include "myriad_pins.h"
#include "myriad_setup.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"

#include "MedianFilter.h"
#include "MAFilter.h"

#include "mode_saw.h"
#include "freqlookup.h"
#include "myriad_messages.h"

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

const uint16_t MAX_ITERATION = 300;  // Nombre de couleurs

const size_t BUF_LEN = 5;
uint8_t spi_out_buf[BUF_LEN];


#define SCREEN_WIDTH tft.width()    //
#define SCREEN_HEIGHT tft.height()  // Taille de l'écran

#define OSC1_PIN 5
#define OSC2_PIN 6
#define OSC3_PIN 7

#define ENCODER1_A_PIN 9
#define ENCODER1_B_PIN 14
#define ENCODER1_SWITCH 8

#define ENCODER2_A_PIN 16
#define ENCODER2_B_PIN 21
#define ENCODER2_SWITCH 15

#define ENCODER3_A_PIN 24
#define ENCODER3_B_PIN 25
#define ENCODER3_SWITCH 23

int encoderValues[3] = {0,0,0};

MovingAverageFilter<int> adcFilters[4];
int controlValues[4] = {0,0,0,0};


static uint16_t __not_in_flash("mydata") capture_buf[16] __attribute__((aligned(2048)));

static float zoom = 0.5;

void __not_in_flash_func(draw_Julia)(float c_r, float c_i, float zoom) {

  tft.setCursor(0, 0);
  float new_r = 0.0, new_i = 0.0, old_r = 0.0, old_i = 0.0;

  /* Pour chaque pixel en X */

  for (int16_t x = SCREEN_WIDTH / 2 - 1; x >= 0; x--) {  // Rely on inverted symmetry
    /* Pour chaque pixel en Y */
    for (uint16_t y = 0; y < SCREEN_HEIGHT; y++) {
      old_r = 1.5 * (x - SCREEN_WIDTH / 2) / (0.5 * zoom * SCREEN_WIDTH);
      old_i = (y - SCREEN_HEIGHT / 2) / (0.5 * zoom * SCREEN_HEIGHT);
      uint16_t i = 0;

      while ((old_r * old_r + old_i * old_i) < 4.0 && i < MAX_ITERATION) {
        new_r = old_r * old_r - old_i * old_i;
        new_i = 2.0 * old_r * old_i;

        old_r = new_r + c_r;
        old_i = new_i + c_i;

        i++;
      }
      /* Affiche le pixel */
      if (i < 100) {
        tft.drawPixel(x, y, tft.color565(255, 255, map(i, 0, 100, 255, 0)));
        tft.drawPixel(SCREEN_WIDTH - x - 1, SCREEN_HEIGHT - y - 1, tft.color565(255, 255, map(i, 0, 100, 255, 0)));
      }
      if (i < 200) {
        tft.drawPixel(x, y, tft.color565(255, map(i, 100, 200, 255, 0), 0));
        tft.drawPixel(SCREEN_WIDTH - x - 1, SCREEN_HEIGHT - y - 1, tft.color565(255, map(i, 100, 200, 255, 0), 0));
      } else {
        tft.drawPixel(x, y, tft.color565(map(i, 200, 300, 255, 0), 0, 0));
        tft.drawPixel(SCREEN_WIDTH - x - 1, SCREEN_HEIGHT - y - 1, tft.color565(map(i, 200, 300, 255, 0), 0, 0));
      }
    }
  }
}

static size_t __not_in_flash("mydata") octave0=0;
static size_t __not_in_flash("mydata") octave1=0;
static size_t __not_in_flash("mydata") octave2=0;
static size_t __not_in_flash("mydata") octave3=0;
static size_t __not_in_flash("mydata") octave4=0;
static size_t __not_in_flash("mydata") octave5=0;


void __not_in_flash_func(setFrequencies)(size_t base, const size_t detune, const size_t acc) {
  wavelen0 = base;
  wavelen1 = wavelen0 + detune + acc;
  wavelen2 = wavelen1 + detune + acc;
  wavelen3 = wavelen2 + detune + acc;
  wavelen4 = wavelen3 + detune + acc;
  wavelen5 = wavelen4 + detune + acc;
  wavelen6 = wavelen5 + detune + acc;
  wavelen7 = wavelen6 + detune + acc;
  wavelen8 = wavelen7 + detune + acc;
  wavelen0 = wavelen0 << octave0;
  wavelen1 = wavelen1 << octave1;
  wavelen2 = wavelen2 << octave2;
  wavelen3 = wavelen3 >> octave3;
  wavelen4 = wavelen4 >> octave4;
  wavelen5 = wavelen5 >> octave5;
  wavelen6 = wavelen6 >> octave3;
  wavelen7 = wavelen7 >> octave4;
  wavelen8 = wavelen8 >> octave5;
}


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


void sendToMyriadB (int msgType, size_t value) {
    //___encode to bytes
    //send message type
    spi_out_buf[0] = static_cast<uint8_t>(msgType);

    //decode and send val
    uint8_t* byteArray = reinterpret_cast<uint8_t*>(&value);
    spi_out_buf[1] = byteArray[0];
    spi_out_buf[2] = byteArray[1];
    spi_out_buf[3] = byteArray[2];
    spi_out_buf[4] = byteArray[3];

    spi_write_blocking(spi1, spi_out_buf, BUF_LEN);
}

size_t lastOctaveIdx = 0;
bool __not_in_flash_func(adcProcessor)(__unused struct repeating_timer *t) {
  controlValues[0] = adcFilters[0].process(capture_buf[0]);
  controlValues[1] = adcFilters[1].process(capture_buf[1]);
  controlValues[2] = adcFilters[2].process(capture_buf[2]);
  controlValues[3] = adcFilters[3].process(capture_buf[3]);

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

  setFrequencies(freqtable[controlValues[0]], controlValues[1] >> 2, controlValues[2] >> 2);

  
  sendToMyriadB(messageTypes::WAVELEN3, wavelen3);
  sendToMyriadB(messageTypes::WAVELEN4, wavelen4);
  sendToMyriadB(messageTypes::WAVELEN5, wavelen5);
  sendToMyriadB(messageTypes::WAVELEN6, wavelen6);
  sendToMyriadB(messageTypes::WAVELEN7, wavelen7);
  sendToMyriadB(messageTypes::WAVELEN8, wavelen8);

  
  return true;
}

bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(120,120);
  tft.setTextColor(TFT_WHITE);
  tft.println(encoderValues[0]);

  tft.setCursor(120,150);
  tft.println(controlValues[0]);
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

void encoder1_callback() {
  int change = read_rotary(enc1Code, enc1Store, ENCODER1_A_PIN, ENCODER1_B_PIN);
  encoderValues[0] += change;
  Serial.println(encoderValues[0]);  
}

void encoder2_callback() {
  int change = read_rotary(enc2Code, enc2Store, ENCODER2_A_PIN, ENCODER2_B_PIN);
  encoderValues[1] += change;
  Serial.println(encoderValues[1]);  
}

void encoder3_callback() {
  int change = read_rotary(enc3Code, enc3Store, ENCODER3_A_PIN, ENCODER3_B_PIN);
  encoderValues[2] += change;
  Serial.println(encoderValues[2]);  
}


struct repeating_timer timerAdcProcessor;
struct repeating_timer timerDisplay;

void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(&FreeMono9pt7b);

  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  //ADCs
  const size_t filterSize=5;
  for(auto &filter: adcFilters) {
    filter.init(filterSize);
  }
  setup_adcs();

  add_repeating_timer_ms(-10, adcProcessor, NULL, &timerAdcProcessor);
  add_repeating_timer_ms(-100, displayUpdate, NULL, &timerDisplay);

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
  spi_init(spi1, 1000 * 1000);
  gpio_set_function(10, GPIO_FUNC_SPI); //SCK
  gpio_set_function(11, GPIO_FUNC_SPI); //TX
  gpio_set_function(12, GPIO_FUNC_SPI); //RX
  gpio_set_function(13, GPIO_FUNC_SPI); //CS


  Serial.begin(115200);

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


}



/* Fonction loop() */
void loop() {
  __wfi();
}


void setup1() {
  //oscillator pins
  setupOscPin(OSC1_PIN);
  setupOscPin(OSC2_PIN);
  setupOscPin(OSC3_PIN);

  //oscillator clock state machine
  uint offset = pio_add_program(pio1, &dsp_clock_program);
  dsp_clock_forever(pio1, 0, offset, pdmFreq);

  //oscillator clock interrupt
  irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw_coreA1);
  irq_set_enabled(PIO1_IRQ_0, true);
  irq_set_priority(PIO1_IRQ_0, 40);
  pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;
}

void loop1() {
  __wfi();
}
