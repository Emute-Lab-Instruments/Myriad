// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// -------- //
// pin_ctrl //
// -------- //

#define pin_ctrl_wrap_target 0
#define pin_ctrl_wrap 5

static const uint16_t pin_ctrl_program_instructions[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block                      
    0x6040, //  1: out    y, 32                      
    0xb822, //  2: mov    x, y            side 1     
    0x0043, //  3: jmp    x--, 3                     
    0xb022, //  4: mov    x, y            side 0     
    0x0045, //  5: jmp    x--, 5                     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program pin_ctrl_program = {
    .instructions = pin_ctrl_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config pin_ctrl_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + pin_ctrl_wrap_target, offset + pin_ctrl_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}
#endif



/*
.program pin_ctrl
.side_set 1 opt
begin:
    pull block  ; Grab next command word
    out y, 32   ; 
start:
    mov x, y side 1
wait_loop:
    jmp x--, wait_loop ; Delay for (x + 1) cycles
    mov x, y side 0   ; 
wait_loop2:
    jmp x--, wait_loop2 ; Delay for (x + 1) cycles
.wrap
*/