
#include <TFT_eSPI.h>


#include "myriad_pins.h"
#include "myriad_setup.h"

#include "hardware/spi.h"

#include "mode_saw.h"

#include "myriad_messages.h"

const size_t BUF_LEN = 5;
uint8_t spi_in_buf[BUF_LEN];



#define RUNCORE0_OSCS
#define RUNCORE1_OSCS

void setup() {
  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);

  //setup SPI 
  spi_init(spi1, 1000 * 1000);
  spi_set_slave(spi1, true);
  gpio_set_function(10, GPIO_FUNC_SPI); //SCK
  gpio_set_function(11, GPIO_FUNC_SPI); //TX
  gpio_set_function(12, GPIO_FUNC_SPI); //RX
  gpio_set_function(13, GPIO_FUNC_SPI); //CS


#ifdef RUNCORE0_OSCS
  //start core 0 oscillators

  setupOscPin(OSC4_PIN);
  setupOscPin(OSC5_PIN);
  setupOscPin(OSC6_PIN);

  PIO pio = pio0;
  uint offset = pio_add_program(pio, &dsp_clock_program);
  printf("Loaded program at %d\n", offset);

  dsp_clock_forever(pio, 0, offset, pdmFreq);
  irq_set_exclusive_handler(PIO0_IRQ_0, irq_handler_saw_coreB0);
  irq_set_enabled(PIO0_IRQ_0, true);
  irq_set_priority(PIO0_IRQ_0, 40);
  pio0->inte0 = PIO_IRQ0_INTE_SM0_BITS;  

#endif

}



/* Fonction loop() */
void loop() {
  int bytesRead =spi_read_blocking(spi1, 0, spi_in_buf, BUF_LEN);

  if (bytesRead == BUF_LEN) {

    //decode message
    messageTypes msgType = static_cast<messageTypes>(spi_in_buf[0]);
    
    size_t *valueOut;
    valueOut = reinterpret_cast<size_t*>(&spi_in_buf[1]);

    switch(msgType) {
      case WAVELEN3:
        wavelen3 = *valueOut;
        break;
      case WAVELEN4:
        wavelen4 = *valueOut;
        break;
      case WAVELEN5:
        wavelen5 = *valueOut;
        break;
      case WAVELEN6:
        wavelen6 = *valueOut;
        break;
      case WAVELEN7:
        wavelen7 = *valueOut;
        break;
      case WAVELEN8:
        wavelen8 = *valueOut;
        // wavelen8 = wavelen7;
        break;
      default:
        break;
    }

  //   wavelen4 = *valueOut;
  //   wavelen5 = *valueOut;
  //   wavelen6 = *valueOut;
  //   wavelen7 = *valueOut;
  //   wavelen8 = *valueOut;
  }
 // __wfi();
}


void setup1() {
#ifdef RUNCORE1_OSCS
  setupOscPin(OSC7_PIN);
  setupOscPin(OSC8_PIN);
  setupOscPin(OSC9_PIN);

  uint offset = pio_add_program(pio1, &dsp_clock_program);

  dsp_clock_forever(pio1, 0, offset, pdmFreq);
  irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw_coreB1);
  irq_set_enabled(PIO1_IRQ_0, true);
  irq_set_priority(PIO1_IRQ_0, 40);
  pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;  
#endif
}

void loop1() {
  __wfi();
}
