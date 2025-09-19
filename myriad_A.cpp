extern "C" {
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "dsp_clock.pio.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include <stdlib.h>
}

#include "freqlookup.h"

#include "saw.h"


// #define RUNCORE0_OSCS
// #define RUNCORE1_OSCS 

//extra optimisations: https://stackoverflow.com/questions/14492436/g-optimization-beyond-o3-ofast
//https://forums.raspberrypi.com/viewtopic.php?t=323982 



#define CAPTURE_CHANNEL 0

// ADC smoothing variables - using same pattern as reference
volatile uint8_t mxPos = 0; // external multiplexer value
volatile int32_t knobssm[4] = {0,0,0,0};
volatile int32_t cvsm[2] = {0,0};
volatile uint16_t knobs[4] = {0,0,0,0}; // 0-4095
volatile uint16_t cv[2] = {0,0}; // -2047 - 2048
volatile uint16_t audio_in[2] = {2048, 2048};

// Keep capture_buf for compatibility but reduce size
static uint16_t __not_in_flash("mydata") capture_buf[4] __attribute__((aligned(2048)));


// const int pdmFreq = 44100 * 384;
const int pdmFreq = 44100 * 128;




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
  wavelen0 = wavelen0 << octave0;
  wavelen1 = wavelen1 << octave1;
  wavelen2 = wavelen2 << octave2;
  wavelen3 = wavelen3 >> octave3;
  wavelen4 = wavelen4 >> octave4;
  wavelen5 = wavelen5 >> octave5;
}

static inline void __isr irq_handler_flipTable() {
  static int __not_in_flash("mydata") count=0;
  // static const int counts[] = {28,38,19,59,23,12,40,124,92,29,19,459,19,49,29,19,2001,12,490,2000,12344};
  static int __not_in_flash("mydata") counts[] = {20000,20000,1000};
  // static const int counts[] = {100,100,100,100,100};
  //idea: move around an offset window within an array, moving two values each time, all values must always add up to wavelength
  static int __not_in_flash("mydata") bit=0;
  static int __not_in_flash("mydata") countPtr = 0;
  static int __not_in_flash("mydata") countTarget = 0;
  // static int __not_in_flash("mydata")bitOnCount=0;
  
  if (count>=countTarget) {
    bit = 1-bit;
    countTarget = counts[countPtr];
    count = 0;
    countPtr++;
    if (countPtr == 3 ) {
      countPtr=0;
    }
  }
  gpio_put(OSC1_PIN, bit);  
  gpio_put(OSC2_PIN, bit);  
  gpio_put(OSC3_PIN, bit);  
  gpio_put(OSC4_PIN, bit);  
  gpio_put(OSC5_PIN, bit);  
  gpio_put(OSC6_PIN, bit);  
  count++;

  pio_interrupt_clear(pio0, 0);

}


static inline void __isr irq_handler_test() {
  static int count=0;
  // static const int counts[] = {28,38,19,59,23,12,40,124,92,29,19,459,19,49,29,19,2001,12,490,2000,12344};
  // static const int counts[] = {20000,20000};
  // static const int counts[] = {100,100,100,100,100};
  static int bit=0;
  static int countTarget = 50000;
  
  if (count>=countTarget) {
    bit = 1-bit;
    countTarget >>=2;
    //create harmonics
    // countTarget += 89;
    // countTarget &=872128;
    count = 0;
    if (countTarget <= 0 ) {
      countTarget += wavelen0;
    }
  }
  gpio_put(OSC1_PIN, bit);  
  gpio_put(OSC2_PIN, bit);  
  gpio_put(OSC3_PIN, bit);  
  count++;

  pio_interrupt_clear(pio0, 0);

}


static inline void __isr irq_handler_bitosc() {
  static int bit=0;
  static int32_t oscx=18892;
  oscx *= 139;
  oscx -= 53;
  oscx >>= 3;
  bit = (oscx & 0b10) >> 1;
  gpio_put(OSC1_PIN, bit);  


  static int bit1=0;
  static int32_t oscx1=18892;
  oscx1 *= 138;
  oscx1 -= 53;
  oscx1 >>= 3;
  bit1 = (oscx1 & 0b1000) >> 3;
  gpio_put(OSC2_PIN, bit1);  

  pio_interrupt_clear(pio0, 0);

}


void dsp_clock_forever(PIO pio, uint sm, uint offset, uint freq) {
    dsp_clock_program_init(pio, sm, offset);
    pio_sm_set_enabled(pio, sm, true);

    pio->txf[sm] = clock_get_hz(clk_sys) / freq;
    // pio->txf[sm] = 1;
}

void dsp_clock_irq_1_forever(PIO pio, uint sm, uint offset, uint freq) {
    dsp_clock_irq_1_program_init(pio, sm, offset);
    pio_sm_set_enabled(pio, sm, true);
    pio->txf[sm] = clock_get_hz(clk_sys) / freq;
}


void __not_in_flash_func(setupOscPin)(int pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
  gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
  gpio_put(pin, 0);
}



void __not_in_flash_func(core1_main)()
{
  setupOscPin(OSC4_PIN);
  setupOscPin(OSC5_PIN);
  setupOscPin(OSC6_PIN);

  uint offset = pio_add_program(pio1, &dsp_clock_program);


  dsp_clock_forever(pio1, 0, offset, pdmFreq);



  // __uint32_t counter=0;

  irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw_core1);
  irq_set_enabled(PIO1_IRQ_0, true);
  irq_set_priority(PIO1_IRQ_0, 40);
  pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;  


    while (1) {
      __wfi();
      // printf("core 1\n");
      // sleep_ms(1000);
    }
}



// ADC interrupt handler with IIR smoothing
void __not_in_flash_func(adc_irq_handler)() {
    // Get ADC data from FIFO
    uint16_t adc = adc_fifo_get_blocking();

    switch(adc_get_selected_input()) {
        case 0:
            // Knob 0 - stronger smoothing for knobs
            knobssm[0] = (127 * (knobssm[0]) + 16 * adc) >> 7;
            knobs[0] = knobssm[0] >> 4;
            capture_buf[0] = knobs[0];
            break;
        case 1:
            // Knob 1
            knobssm[1] = (127 * (knobssm[1]) + 16 * adc) >> 7;
            knobs[1] = knobssm[1] >> 4;
            capture_buf[1] = knobs[1];
            break;
        case 2:
            // Knob 2
            knobssm[2] = (127 * (knobssm[2]) + 16 * adc) >> 7;
            knobs[2] = knobssm[2] >> 4;
            capture_buf[2] = knobs[2];
            break;
        case 3:
            // Knob 3
            knobssm[3] = (127 * (knobssm[3]) + 16 * adc) >> 7;
            knobs[3] = knobssm[3] >> 4;
            capture_buf[3] = knobs[3];
            break;
    }
}

size_t lastOctaveIdx = 0;
bool __not_in_flash_func(repeating_timer_callback)(__unused struct repeating_timer *t) {
    // printf("Repeat at %lld\n", time_us_64());

    size_t octaveIdx = knobs[3] >> 8;  // div by 256 -> 16 divisions
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

    setFrequencies(freqtable[knobs[0]], knobs[1] >> 2, knobs[2] >> 2);
    return true;
}

int __not_in_flash_func(main)() {
  stdio_init_all();
  printf("Hello from Myriad A");
  setupOscPin(LED_PIN);
  gpio_put(LED_PIN, 1);

  // bi_decl(bi_program_description("This is a test binary."));
  // bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
  // set_sys_clock_khz(250000, true);
  set_sys_clock_khz(270000, true);

  setFrequencies(22000,10, 0);

  // ADC initialization using interrupt-based approach from reference
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  adc_gpio_init(29);

  // Set round robin for all 4 channels
  adc_set_round_robin(0x0F);
  adc_fifo_setup(true, true, 1, false, false);

  // 96kHz sampling rate (48MHz / 500 = 96kHz)
  adc_set_clkdiv((48000000 / 96000) - 1);

  // Setup interrupt handler
  irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler);
  adc_irq_set_enabled(true);
  irq_set_enabled(ADC_IRQ_FIFO, true);

  // Start ADC
  adc_run(true);


  struct repeating_timer timer;
  add_repeating_timer_ms(10, repeating_timer_callback, NULL, &timer);


  //setup SPI 
  spi_init(spi1, 1000 * 1000);
  gpio_set_function(10, GPIO_FUNC_SPI); //SCK
  gpio_set_function(11, GPIO_FUNC_SPI); //TX
  gpio_set_function(12, GPIO_FUNC_SPI); //RX
  gpio_set_function(13, GPIO_FUNC_SPI); //CS
  const size_t BUF_LEN = 1;
  uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

#ifdef RUNCORE1_OSCS
  multicore_launch_core1(core1_main);
#endif


#ifdef RUNCORE0_OSCS
  //start core 0 oscillators

  setupOscPin(OSC1_PIN);
  setupOscPin(OSC2_PIN);
  setupOscPin(OSC3_PIN);

  PIO pio = pio0;
  uint offset = pio_add_program(pio, &dsp_clock_program);
  printf("Loaded program at %d\n", offset);

  // const int pdmFreq = 44100 * 512;

  dsp_clock_forever(pio, 0, offset, pdmFreq);
  // irq_set_exclusive_handler(PIO0_IRQ_0, irq_handler_saw);
  irq_set_exclusive_handler(PIO0_IRQ_0, irq_handler_flipTable);
  irq_set_enabled(PIO0_IRQ_0, true);
  irq_set_priority(PIO0_IRQ_0, 40);
  pio0->inte0 = PIO_IRQ0_INTE_SM0_BITS;  


#endif

  //adc reader
  //pio clocked interrupt generator
  // uint offset2 = pio_add_program(pio0, &dsp_clock_irq_1_program);
  // dsp_clock_irq_1_forever(pio0, 1, offset2, 10);

  //interrupt handler
  // irq_set_exclusive_handler(PIO0_IRQ_1, irq_handler_adc_reader);
  // irq_set_enabled(PIO0_IRQ_1, true);
  //lower priority than oscillators
  // irq_set_priority(PIO0_IRQ_1, 0);
  // pio0->inte1 = PIO_IRQ1_INTE_SM1_BITS;  

  // bool led=0;
  uint8_t count;
  while (1) {    
    // __wfi();
    // // for(int i=0; i < adc_fifo_get_level(); i++) {
    // for(int i=0; i < 4; i++) {
    //   // printf("%d\t", adc_fifo_get());
    //   printf("%d\t", knobs[i]);
    // }
    // printf("\n");
    // // gpio_put(LED_PIN, led);
    // // led = !led;
    printf("SPI test\n");
    out_buf[0] = count;
    spi_write_blocking(spi1, out_buf, BUF_LEN);
    count = ++count % 255;
    printf("SPI: sent: %d\n", out_buf[0]);
    sleep_ms(500);
  }

}



