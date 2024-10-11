#pragma once

#include "dsp_clock.pio.h"

const int pdmFreq = 44100 * 128;
const size_t spiFrequency = 400 * 1000;

void __not_in_flash_func(setupOscPin)(int pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
  gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
  gpio_put(pin, 0);
}

void dsp_clock_forever(PIO pio, uint sm, uint offset, uint freq) {
    dsp_clock_program_init(pio, sm, offset);
    pio_sm_set_enabled(pio, sm, true);

    pio->txf[sm] = clock_get_hz(clk_sys) / freq;
}
