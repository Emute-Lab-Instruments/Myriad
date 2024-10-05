
#include <TFT_eSPI.h>
#include "myriad_pins.h"
#include "myriad_setup.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"




TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

const uint16_t MAX_ITERATION = 300;  // Nombre de couleurs

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

static int __not_in_flash("mydata") wavelen0 = 40000;
static int __not_in_flash("mydata") wavelen1 = 39400;
static int __not_in_flash("mydata") wavelen2 = 40600;
static int __not_in_flash("mydata") wavelen3 = 41100;
static int __not_in_flash("mydata") wavelen4 = 41500;
static int __not_in_flash("mydata") wavelen5 = 41900;

static int __not_in_flash("mydata") phase0 = 0;
static bool __not_in_flash("mydata") y_0 = 0;
static int __not_in_flash("mydata") err0 = 0;

static int __not_in_flash("mydata") phase1 = 0;
static bool __not_in_flash("mydata") y_1 = 0;
static int __not_in_flash("mydata") err1 = 0;

static int __not_in_flash("mydata") phase2 = 0;
static bool __not_in_flash("mydata") y_2 = 0;
static int __not_in_flash("mydata") err2 = 0;


static inline void __isr irq_handler_saw() {

  if (phase0 >= wavelen0) {
    phase0 = 0;
  }
  phase0++;


  y_0 = phase0 >= err0 ? 1 : 0;
  err0 = (y_0 ? wavelen0 : 0) - phase0 + err0;
  gpio_put(OSC1_PIN, y_0);


  ////// OSC 2


  if (phase1 >= wavelen1) {
    phase1 = 0;
  }
  phase1++;

  y_1 = phase1 >= err1 ? 1 : 0;
  err1 = (y_1 ? wavelen1 : 0) - phase1 + err1;
  gpio_put(OSC2_PIN, y_1);


  ////// OSC 3


  if (phase2 >= wavelen2) {
    phase2 = 0;
  }
  phase2++;

  y_2 = phase2 >= err2 ? 1 : 0;
  err2 = (y_2 ? wavelen2 : 0) - phase2 + err2;
  gpio_put(OSC3_PIN, y_2);

  pio_interrupt_clear(pio1, 0);
}

bool __not_in_flash_func(repeating_timer_callback)(__unused struct repeating_timer *t) {
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


void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(&FreeMono9pt7b);

  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  //ADCs
  setup_adcs();
  // struct repeating_timer timer;
  // add_repeating_timer_ms(10, repeating_timer_callback, NULL, &timer);

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

  /* Dessine la fractale */
  // draw_Julia(-0.8,+0.156,zoom);
  // tft.fillRect(0, 0, 150, 20, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(120,120);
  tft.setTextColor(TFT_WHITE);
  tft.println(encoderValues[0]);
  // delay(2000);
  // zoom *= 1.5;
  // if (zoom > 100) zoom = 0.5;
  // Serial.println(capture_buf[0]);
  // Serial.println(capture_buf[1]);
  // Serial.println(capture_buf[2]);
  // Serial.println(capture_buf[3]);


  delay(20);
}


void encoderSwitchInterrupt() {
  Serial.println("switch");
}
void setup1() {
  //oscillator pings
  // setupOscPin(OSC1_PIN);
  // setupOscPin(OSC2_PIN);
  // setupOscPin(OSC3_PIN);

  // //oscillator clock state machine
  // uint offset = pio_add_program(pio1, &dsp_clock_program);
  // dsp_clock_forever(pio1, 0, offset, pdmFreq);

  // //oscillator clock interrupt
  // irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw);
  // irq_set_enabled(PIO1_IRQ_0, true);
  // irq_set_priority(PIO1_IRQ_0, 40);
  // pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;
}

void loop1() {
  __wfi();
}
