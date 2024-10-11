#ifndef MODE_SQR_H
#define MODE_SQR_H

#include "myriad_pins.h"
#include "synthparams.h"




void mode_sqr_draw_icon() {

}


static inline void __isr irq_handler_sqr_coreB0() {
  static int __scratch_x("mydata") phase0 = 0;
  static int __scratch_x("mydata") phase1 = 0;
  static int __scratch_x("mydata") phase2 = 0;

  bool y_0 = phase0 < (wavelen0 >> 1) ? 1 : 0;
  phase0++;
  if (phase0 >= wavelen0)
    phase0=0;
  gpio_put(OSC1_PIN, y_0);

  bool y_1 = phase1 < (wavelen1 >> 1) ? 1 : 0;
  phase1++;
  if (phase1 >= wavelen1)
    phase1=0;
  gpio_put(OSC2_PIN, y_1);

  bool y_2 = phase2 < (wavelen2 >> 1) ? 1 : 0;
  phase2++;
  if (phase2 >= wavelen2)
    phase2=0;
  gpio_put(OSC3_PIN, 0);


  pio_interrupt_clear(pio0, 0);
}

static inline void __isr irq_handler_sqr_coreB1() {
  static int __scratch_y("mydata") phase3 = 0;
  static int __scratch_y("mydata") phase4 = 0;
  static int __scratch_y("mydata") phase5 = 0;

  bool y_3 = phase3 < (wavelen3 >> 1) ? 1 : 0;
  phase3++;
  if (phase3 >= wavelen3)
    phase3=0;
  gpio_put(OSC4_PIN, y_3);

  bool y_4 = phase4 < (wavelen4 >> 1) ? 1 : 0;
  phase4++;
  if (phase4 >= wavelen4)
    phase4=0;
  gpio_put(OSC5_PIN, y_4);

  bool y_5 = phase5 < (wavelen5 >> 1) ? 1 : 0;
  phase5++;
  if (phase5 >= wavelen5)
    phase5=0;
  gpio_put(OSC6_PIN, 0);

  pio_interrupt_clear(pio1, 0);
}

#endif