
#include <TFT_eSPI.h>
#include "myriad_pins.h"
#include "myriad_setup.h"

#include "hardware/spi.h"





#define OSC1_PIN 2
#define OSC2_PIN 3
#define OSC3_PIN 5
#define OSC4_PIN 6
#define OSC5_PIN 7
#define OSC6_PIN 8

static int __not_in_flash("mydata") wavelen0=40000;
static int __not_in_flash("mydata") wavelen1=39400;
static int __not_in_flash("mydata") wavelen2=40600;
static int __not_in_flash("mydata") wavelen3=41100;
static int __not_in_flash("mydata") wavelen4=41500;
static int __not_in_flash("mydata") wavelen5=41900;

static int __not_in_flash("mydata") phase0=0;
static bool __not_in_flash("mydata") y_0=0;
static int __not_in_flash("mydata") err0=0;

static int __not_in_flash("mydata") phase1=0;
static bool __not_in_flash("mydata") y_1=0;
static int __not_in_flash("mydata") err1=0;

static int __not_in_flash("mydata") phase2=0;
static bool __not_in_flash("mydata") y_2=0;
static int __not_in_flash("mydata") err2=0;

static inline void __isr irq_handler_saw() {

  if (phase0>=wavelen0) {
    phase0 = 0;
  }
  phase0++;

  
  y_0 = phase0 >= err0 ? 1 : 0;
  err0 = (y_0 ? wavelen0 : 0) - phase0 + err0;
  gpio_put(OSC1_PIN, y_0);


  ////// OSC 2


  if (phase1>=wavelen1) {
    phase1 = 0;
  }
  phase1++;

  y_1 = phase1 >= err1 ? 1 : 0;
  err1 = (y_1 ? wavelen1 : 0) - phase1 + err1;
  gpio_put(OSC2_PIN, y_1);


  ////// OSC 3


  if (phase2>=wavelen2) {
    phase2 = 0;
  }
  phase2++;

  y_2 = phase2 >= err2 ? 1 : 0;
  err2 = (y_2 ? wavelen2 : 0) - phase2 + err2;
  gpio_put(OSC3_PIN, y_2);

  pio_interrupt_clear(pio0, 0);
}


static inline void __isr irq_handler_saw_core1() {

  //OSC 4
  static int __not_in_flash("mydata") phase3=0;
  static bool __not_in_flash("mydata") y_3=0;
  static int __not_in_flash("mydata") err3=0;

  if (phase3>=wavelen3) {
    phase3 = 0;
  }
  phase3++;

  
  y_3 = phase3 >= err3 ? 1 : 0;
  err3 = (y_3 ? wavelen3 : 0) - phase3 + err3;
  gpio_put(OSC4_PIN, y_3);

  //// OSC 5

  static int __not_in_flash("mydata") phase4=0;
  static bool __not_in_flash("mydata") y_4=0;
  static int __not_in_flash("mydata") err4=0;

  if (phase4>=wavelen4) {
    phase4 = 0;
  }
  phase4++;

  y_4 = phase4 >= err4 ? 1 : 0;
  err4 = (y_4 ? wavelen4 : 0) - phase4 + err4;
  gpio_put(OSC5_PIN, y_4);


  ////// OSC 6

  static int __not_in_flash("mydata") phase5=0;
  static bool __not_in_flash("mydata") y_5=0;
  static int __not_in_flash("mydata") err5=0;

  if (phase5>=wavelen5) {
    phase5 = 0;
  }
  phase5++;

  y_5 = phase5 >= err5 ? 1 : 0;
  err5 = (y_5 ? wavelen5 : 0) - phase5 + err5;
  gpio_put(OSC6_PIN, y_5);

  pio_interrupt_clear(pio1, 0);
}


#define RUNCORE0_OSCS
#define RUNCORE1_OSCS

void setup() {
  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);

#ifdef RUNCORE0_OSCS
  //start core 0 oscillators

  setupOscPin(OSC1_PIN);
  setupOscPin(OSC2_PIN);
  setupOscPin(OSC3_PIN);

  PIO pio = pio0;
  uint offset = pio_add_program(pio, &dsp_clock_program);
  printf("Loaded program at %d\n", offset);

  dsp_clock_forever(pio, 0, offset, pdmFreq);
  irq_set_exclusive_handler(PIO0_IRQ_0, irq_handler_saw);
  irq_set_enabled(PIO0_IRQ_0, true);
  irq_set_priority(PIO0_IRQ_0, 40);
  pio0->inte0 = PIO_IRQ0_INTE_SM0_BITS;  


#endif

}



/* Fonction loop() */
void loop() {
  __wfi();
}


void setup1() {
#ifdef RUNCORE1_OSCS
  setupOscPin(OSC4_PIN);
  setupOscPin(OSC5_PIN);
  setupOscPin(OSC6_PIN);

  uint offset = pio_add_program(pio1, &dsp_clock_program);

  dsp_clock_forever(pio1, 0, offset, pdmFreq);
  irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw_core1);
  irq_set_enabled(PIO1_IRQ_0, true);
  irq_set_priority(PIO1_IRQ_0, 40);
  pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;  
#endif
}

void loop1() {
  __wfi();
}
