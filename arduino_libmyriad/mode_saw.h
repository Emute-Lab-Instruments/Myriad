#pragma once
#include "myriad_pins.h"

static int __not_in_flash("mydata") wavelen0 = 40000;
static int __not_in_flash("mydata") wavelen1 = 39400;
static int __not_in_flash("mydata") wavelen2 = 40600;
static int __not_in_flash("mydata") wavelen3 = 41100;
static int __not_in_flash("mydata") wavelen4 = 41500;
static int __not_in_flash("mydata") wavelen5 = 41900;
static int __not_in_flash("mydata") wavelen6 = 42600;
static int __not_in_flash("mydata") wavelen7 = 43500;
static int __not_in_flash("mydata") wavelen8 = 44900;


void mode_saw_draw_icon() {

}


static inline void __isr irq_handler_saw_coreA1() {
    static int __not_in_flash("mydata") phase0 = 0;
    static bool __not_in_flash("mydata") y_0 = 0;
    static int __not_in_flash("mydata") err0 = 0;
    static int __not_in_flash("mydata") phase1 = 0;
    static bool __not_in_flash("mydata") y_1 = 0;
    static int __not_in_flash("mydata") err1 = 0;
    static int __not_in_flash("mydata") phase2 = 0;
    static bool __not_in_flash("mydata") y_2 = 0;
    static int __not_in_flash("mydata") err2 = 0;

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

static inline void __isr irq_handler_saw_coreB0() {

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

  pio_interrupt_clear(pio0, 0);
}

static inline void __isr irq_handler_saw_coreB1() {

  //OSC 6
  static int __not_in_flash("mydata") phase6=0;
  static bool __not_in_flash("mydata") y_6=0;
  static int __not_in_flash("mydata") err6=0;

  if (phase6>=wavelen6) {
    phase6 = 0;
  }
  phase6++;

  
  y_6 = phase6 >= err6 ? 1 : 0;
  err6 = (y_6 ? wavelen6 : 0) - phase6 + err6;
  gpio_put(OSC7_PIN, y_6);

  //// OSC 7

  static int __not_in_flash("mydata") phase7=0;
  static bool __not_in_flash("mydata") y_7=0;
  static int __not_in_flash("mydata") err7=0;

  if (phase7>=wavelen7) {
    phase7 = 0;
  }
  phase7++;

  y_7 = phase7 >= err7 ? 1 : 0;
  err7 = (y_7 ? wavelen7 : 0) - phase7 + err7;
  gpio_put(OSC8_PIN, y_7);


  ////// OSC 8

  static int __not_in_flash("mydata") phase8=0;
  static bool __not_in_flash("mydata") y_8=0;
  static int __not_in_flash("mydata") err8=0;

  if (phase8>=wavelen8) {
    phase8 = 0;
  }
  phase8++;

  y_8 = phase8 >= err8 ? 1 : 0;
  err8 = (y_8 ? wavelen8 : 0) - phase8 + err8;
//   gpio_put(OSC9_PIN, y_8);

  pio_interrupt_clear(pio1, 0);
}

