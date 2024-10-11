
#include <TFT_eSPI.h>


#include "myriad_pins.h"
#include "myriad_setup.h"

#include "hardware/spi.h"
#include "hardware/sync.h"

#include "mode_saw.h"
#include "mode_sqr.h"

#include "myriad_messages.h"

#include "SLIP.h"

bool core1_separate_stack = true;

const size_t __not_in_flash("mydata") BUF_LEN = 10;
uint8_t __not_in_flash("mydata") spi_in_buf[BUF_LEN];


#define RUNCORE0_OSCS
#define RUNCORE1_OSCS

static uint8_t __not_in_flash("mydata") slipBuffer[64];

enum SPISTATES {WAITFOREND, ENDORBYTES, READBYTES};
static SPISTATES  __not_in_flash("mydata") spiState = SPISTATES::WAITFOREND;
static int __not_in_flash("mydata") spiIdx=0;

void readSpi() {
  uint8_t spiByte=0;
  int nBytes;
  if (spi_is_readable(spi1)) {
    switch(spiState) {
      case SPISTATES::WAITFOREND:
        nBytes = spi_read_blocking(spi1, 0, &spiByte, 1);
        if (nBytes == 1) {
          if (spiByte == SLIP::END) {
            // Serial.println("end");
            slipBuffer[0] = SLIP::END;
            spiState = SPISTATES::ENDORBYTES;
          }else{
            // Serial.println(spiByte);
          }
        }
        break;
      case ENDORBYTES:
        nBytes = spi_read_blocking(spi1, 0, &spiByte, 1);
        if (nBytes == 1) {
          if (spiByte == SLIP::END) {
            //this is the message start
            spiIdx = 1;

          }else{
            slipBuffer[1] = spiByte;
            spiIdx=2;
          }
          spiState = SPISTATES::READBYTES;
        }
        break;
      case READBYTES:
        nBytes = spi_read_blocking(spi1, 0, &spiByte, 1);
        slipBuffer[spiIdx++] = spiByte;
        if (nBytes == 1) {
          if (spiByte == SLIP::END) {
            // for(int i=0; i < spiIdx; i++) {
            //   Serial.print(slipBuffer[i]);
            //   Serial.print("\t");
            // }
            // Serial.println("");
            spiMessage decodeMsg;
            SLIP::decode(slipBuffer, spiIdx, reinterpret_cast<uint8_t*>(&decodeMsg));
            // Serial.println(decodeMsg.msg);
            // Serial.println(decodeMsg.value);
            switch(decodeMsg.msg) {
              case WAVELEN3:
              {
                wavelen3 = decodeMsg.value;
              }
                break;
              case WAVELEN4:
              {
                wavelen4 = decodeMsg.value;
              }
                break;
              case WAVELEN5:
              {
                wavelen5 = decodeMsg.value;
              }
              break;
              default:
                break;
            }


            spiState = SPISTATES::WAITFOREND;
          }
        }
        break;
    }
  }
}

void setup() {
  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);


  queue_init(&coreCommsQueue, sizeof(queueItem), 3);

  spi_init(spi1, 1000 * 1000);
  spi_set_slave(spi1, true);
  gpio_set_function(10, GPIO_FUNC_SPI); //SCK
  gpio_set_function(11, GPIO_FUNC_SPI); //TX
  gpio_set_function(12, GPIO_FUNC_SPI); //RX
  gpio_set_function(13, GPIO_FUNC_SPI); //CS


#ifdef RUNCORE0_OSCS
  //start core 0 oscillators
  setupOscPin(OSC1_PIN);
  setupOscPin(OSC2_PIN);
  setupOscPin(OSC3_PIN);

  PIO pio = pio0;
  uint offset = pio_add_program(pio, &dsp_clock_program);
  printf("Loaded program at %d\n", offset);

  dsp_clock_forever(pio, 0, offset, pdmFreq);
  irq_set_exclusive_handler(PIO0_IRQ_0, irq_handler_saw_coreB0);
  // irq_set_exclusive_handler(PIO0_IRQ_0, irq_handler_sqr_coreB0);
  irq_set_enabled(PIO0_IRQ_0, true);
  irq_set_priority(PIO0_IRQ_0, 40);
  pio0->inte0 = PIO_IRQ0_INTE_SM0_BITS;  

#endif

}


/* Fonction loop() */
void __not_in_flash_func(loop)() {
  //decode
  // readSpi();
 __wfi();
}


void setup1() {

#ifdef RUNCORE1_OSCS
  setupOscPin(OSC4_PIN);
  setupOscPin(OSC5_PIN);
  setupOscPin(OSC6_PIN);


  uint offset = pio_add_program(pio1, &dsp_clock_program);

  dsp_clock_forever(pio1, 0, offset, pdmFreq);
  irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw_coreB1);
  irq_set_enabled(PIO1_IRQ_0, true);
  irq_set_priority(PIO1_IRQ_0, 10);
  pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;  
#endif
}

void loop1() {
  __wfi();
}
