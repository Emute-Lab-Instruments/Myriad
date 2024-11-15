#ifndef __SM_BITSTREAM_OSC_H
#define __SM_BITSTREAM_OSC_H

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"


class smBitStreamOsc {
public:
  smBitStreamOsc() {}
  
  uint32_t init(PIO pio_, uint sm_, uint pin_, uint offset_, const void* firstTimingBuffer, irq_handler_t dma_irq_handler, size_t clockdiv, uint dmaIrq=DMA_IRQ_0) {
    pio = pio_;
    sm = sm_;
    pin = pin_;
    offset = offset_;

    // Run the pin_ctrl program
    return pin_ctrl_prepare(pio, sm, offset, pin, firstTimingBuffer, dma_irq_handler, clockdiv, dmaIrq);
  }


  void pin_ctrl_program_init(PIO pio, uint sm, uint offset, uint pin, size_t clockdiv) {
    // Setup pin to be accessible from the PIO
    pio_gpio_init(pio, pin);
    // pio_gpio_init(pio, pin + 1);

    // Setup PIO SM to control pin
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    // Configure PIO SM with pin_ctrl program
    pio_sm_config c = pin_ctrl_program_get_default_config(offset);

    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_clkdiv(&c, clockdiv);

    sm_config_set_in_shift(&c, true, false, 32);
    // don't join FIFOs together to get an 8 entry TX FIFO
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
    pio_sm_init(pio, sm, offset, &c);
  }

  uint32_t pin_ctrl_prepare(PIO pio, uint sm, uint offset, uint pin, const void* firstTimingBuffer, irq_handler_t dma_irq_handler, size_t clockdiv, uint dmaIrq) {
    // Allocate a DMA channel to feed the pin_ctrl SM its command words
    pio_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config pio_dma_chan_config = dma_channel_get_default_config(pio_dma_chan);
    // Transfer 32 bits each time
    channel_config_set_transfer_data_size(&pio_dma_chan_config, DMA_SIZE_32);
    // Increment read address (a different command word from `timing_buffer`
    // each time)
    channel_config_set_read_increment(&pio_dma_chan_config, true);
    // Write to the same address (the PIO SM TX FIFO)
    channel_config_set_write_increment(&pio_dma_chan_config, false);
    // Set read address to wrap on a 16-byte boundary
    channel_config_set_ring(&pio_dma_chan_config, false, 4);
    // Transfer when PIO SM TX FIFO has space
    channel_config_set_dreq(&pio_dma_chan_config, pio_get_dreq(pio, sm, true));

    // Setup the channel and set it going
    dma_channel_configure(
      pio_dma_chan,
      &pio_dma_chan_config,
      &pio->txf[sm],  // Write to PIO TX FIFO
      firstTimingBuffer,  // Read values from timing buffer
      2,             // transfer count
      false           // don't start yet
    );

    // Setup IRQ for DMA transfer end
    
    if (dmaIrq == DMA_IRQ_0) {
      dma_channel_set_irq0_enabled(pio_dma_chan, true);
    }else{
      dma_channel_set_irq1_enabled(pio_dma_chan, true);
    }

    irq_set_exclusive_handler(dmaIrq, dma_irq_handler);
    irq_set_enabled(dmaIrq, true);

    // Initialise PIO SM with pin_ctrl program
    pin_ctrl_program_init(pio, sm, offset, pin, clockdiv);
    return pio_dma_chan;
  }

  void go() {
    // Start the DMA (must do this after program init or DMA won't do anything)
    dma_channel_start(pio_dma_chan);
    // Start the PIO
    pio_sm_set_enabled(pio, sm, true);
  }

  void pause() {
    dma_channel_abort(pio_dma_chan);
  }

  uint32_t pio_dma_chan;
  uint offset;
  PIO pio;
  uint pin;
  uint sm;

};


#endif