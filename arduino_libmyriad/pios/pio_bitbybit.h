#ifndef PIO_BITBYBIT_H
#define PIO_BITBYBIT_H

// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#include "hardware/pio.h"

// -------- //
// bitbybit //
// -------- //

#define bitbybit_wrap_target 0
#define bitbybit_wrap 0

static const uint16_t bitbybit_program_instructions[] = {
            //     .wrap_target
    0x6001, //  0: out    pins, 1                    
            //     .wrap
};

static const struct pio_program bitbybit_program = {
    .instructions = bitbybit_program_instructions,
    .length = 1,
    .origin = -1,
};

static inline pio_sm_config bitbybit_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + bitbybit_wrap_target, offset + bitbybit_wrap);
    sm_config_set_sideset(&c, 2, true, false);

    //Enable auto-pull every 32 bits, shift right
    // This eliminates timing variations from manual pull/set

    sm_config_set_out_shift(&c, true, true, 32);
    return c;
}

#endif // PIO_BITBYBIT_H