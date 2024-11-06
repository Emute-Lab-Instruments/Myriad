#pragma once

#include "myriad_pins.h"
#include "synthparams.h"



#include "mode_saw.h"
#include "mode_sqr.h"

static size_t __not_in_flash("mydata") osc0Mode=0;
static size_t __not_in_flash("mydata") osc1Mode=0;


static inline void __isr irq_handler_osc_coreB0() {
    switch(osc0Mode) {
        case 0:
            irq_handler_saw_coreB0();
        break;
        case 1:
            irq_handler_sqr_coreB0();
        break;
    }
    
}

static inline void __isr irq_handler_osc_coreB1() {
    switch(osc1Mode) {
        case 0:
            irq_handler_saw_coreB1();
        break;
        case 1:
            irq_handler_sqr_coreB1();
        break;
    }
    
}
