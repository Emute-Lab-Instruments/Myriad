


#include "myriad_pins.h"
#include "myriad_setup.h"

#include "hardware/sync.h"

#include "pio_expdec.h"
#include "smBitStreamOsc.h"


// #include "mode_saw.h"
// #include "mode_sqr.h"
// #include "mode_all.h"

#define RUNCORE0_OSCS
// #define RUNCORE1_OSCS



#include "myriad_messages.h"

#include "SLIP.h"


#define FAST_MEM __not_in_flash("mydata")

float clockdiv = 8;
uint32_t clockHz = 15625000 / 80;
size_t cpuClock=125000000;
float sampleClock = cpuClock / clockdiv;
uint32_t mwavelen = 125000000/clockdiv /50;
uint32_t mwavelen2 = mwavelen * 1.01;
uint32_t mwavelen3 = mwavelen2 * 1.01;
uint32_t mwavelen4 = mwavelen3 * 1.01;
uint32_t mwavelen5 = mwavelen4 * 1.01;
uint32_t mwavelen6 = mwavelen5 * 1.01;
// Ensure `timing_buffer` is aligned to 16-bytes so we can use DMA address
// wrapping
float sqrTemplate[8] {
  0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5
};
// uint32_t timing_buffer[8] __attribute__((aligned(16))) {clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1};
// uint32_t timing_buffer2[8] __attribute__((aligned(16))) {clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1};
  
uint32_t FAST_MEM timing_buffer[8] __attribute__((aligned(16))) {
  mwavelen,mwavelen>>1,mwavelen,mwavelen>>1,mwavelen,mwavelen>>1,mwavelen,mwavelen>>1
 
  };
uint32_t timing_buffer3[8] __attribute__((aligned(16))) {
  mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2,mwavelen2
 
  };
uint32_t timing_buffer4[8] __attribute__((aligned(16))) {
  mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3,mwavelen3, 
};

uint32_t timing_buffer5[8] __attribute__((aligned(16))) {
  mwavelen4,mwavelen4,mwavelen4,mwavelen4,mwavelen4,mwavelen4,mwavelen4,mwavelen4,
};
uint32_t timing_buffer6[8] __attribute__((aligned(16))) {
  mwavelen5,mwavelen5,mwavelen5,mwavelen5,mwavelen5,mwavelen5,mwavelen5,mwavelen5
};
uint32_t timing_buffer7[8] __attribute__((aligned(16))) {
  mwavelen6,mwavelen6,mwavelen6,mwavelen6,mwavelen6,mwavelen6,mwavelen6,mwavelen6,
};

// uint32_t pio_dma_chan;

io_rw_32 nextTimingBuffer = (io_rw_32)timing_buffer;
io_rw_32  nextTimingBuffer2 = (io_rw_32)timing_buffer3;
io_rw_32  nextTimingBuffer3 = (io_rw_32)timing_buffer4;

io_rw_32  nextTimingBuffer4 = (io_rw_32)timing_buffer5;
io_rw_32  nextTimingBuffer5 = (io_rw_32)timing_buffer6;
io_rw_32  nextTimingBuffer6 = (io_rw_32)timing_buffer7;


uint programOffset = pio_add_program(pio0, &pin_ctrl_program);
uint programOffset1 = pio_add_program(pio1, &pin_ctrl_program);

uint32_t FAST_MEM smOsc0_dma_chan;
uint32_t FAST_MEM smOsc0_dma_chan_bit;

uint32_t FAST_MEM smOsc1_dma_chan;
uint32_t FAST_MEM smOsc1_dma_chan_bit;

uint32_t FAST_MEM smOsc2_dma_chan;
uint32_t FAST_MEM smOsc2_dma_chan_bit;


/////////////////////////////////   IRQ 000000000000000000000000000000000000
void __isr dma_irh() {
  uint32_t triggered_channels = dma_hw->ints0;
  if (triggered_channels & smOsc0_dma_chan_bit) {
    dma_hw->ints0 = smOsc0_dma_chan_bit;  
    dma_hw->ch[smOsc0_dma_chan].al3_read_addr_trig = nextTimingBuffer;
  }
  else
  if (triggered_channels & smOsc1_dma_chan_bit) {
  //   // Channel 0 triggered, handle it
    dma_hw->ints0 = smOsc1_dma_chan_bit;
  //   dma_hw->ints0 &= smOsc0_dma_chan_bit_inv;  // Clear interrupt 
    dma_hw->ch[smOsc1_dma_chan].al3_read_addr_trig = nextTimingBuffer2;
  }
  else
  if (triggered_channels & smOsc2_dma_chan_bit) {
    dma_hw->ints0 = smOsc2_dma_chan_bit;
    dma_hw->ch[smOsc2_dma_chan].al3_read_addr_trig = nextTimingBuffer3;
  }
}
smBitStreamOsc smOsc0;
smBitStreamOsc smOsc1;
smBitStreamOsc smOsc2;


uint32_t volatile FAST_MEM smOsc3_dma_chan;
uint32_t volatile FAST_MEM smOsc3_dma_chan_bit;

uint32_t volatile FAST_MEM smOsc4_dma_chan;
uint32_t volatile FAST_MEM smOsc4_dma_chan_bit;

uint32_t volatile FAST_MEM smOsc5_dma_chan;
uint32_t volatile FAST_MEM smOsc5_dma_chan_bit;



/////////////////////////////////   IRQ 11111111111111111111111111111
void __not_in_flash_func(dma_irh1)() {
  uint32_t triggered_channels = dma_hw->ints1;
  if (triggered_channels & smOsc3_dma_chan_bit) {
    dma_hw->ints1 = smOsc3_dma_chan_bit;  
    dma_hw->ch[smOsc3_dma_chan].al3_read_addr_trig = nextTimingBuffer4;
  }
  else
  if (triggered_channels & smOsc4_dma_chan_bit) {
  //   // Channel 0 triggered, handle it
    dma_hw->ints1 = smOsc4_dma_chan_bit;
  //   dma_hw->ints0 &= smOsc0_dma_chan_bit_inv;  // Clear interrupt 
    dma_hw->ch[smOsc4_dma_chan].al3_read_addr_trig = nextTimingBuffer5;
  }
  else
  if (triggered_channels & smOsc5_dma_chan_bit) {
    dma_hw->ints1 = smOsc5_dma_chan_bit;
    dma_hw->ch[smOsc5_dma_chan].al3_read_addr_trig = nextTimingBuffer6;
  }
}


smBitStreamOsc smOsc3;
smBitStreamOsc smOsc4;
smBitStreamOsc smOsc5;


bool core1_separate_stack = true;

const size_t __not_in_flash("mydata") BUF_LEN = 10;
uint8_t __not_in_flash("mydata") spi_in_buf[BUF_LEN];



static uint8_t __not_in_flash("mydata") slipBuffer[64];

enum SPISTATES {WAITFOREND, ENDORBYTES, READBYTES};
static SPISTATES  __not_in_flash("mydata") spiState = SPISTATES::WAITFOREND;
static int __not_in_flash("mydata") spiIdx=0;

int oscBankChange=-1;

inline void __not_in_flash_func(readUart)() {
  uint8_t spiByte=0;
  int nBytes;
  // Serial.println("readuart");
  if (Serial1.available()) {
    switch(spiState) {
      case SPISTATES::WAITFOREND:
        spiByte = Serial1.read();
        if (spiByte != -1) {
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
        spiByte = Serial1.read();
        if (spiByte != -1) {
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
        spiByte = Serial1.read();
        if (spiByte != -1) {

          slipBuffer[spiIdx++] = spiByte;
          if (spiByte == SLIP::END) {
            // for(int i=0; i < spiIdx; i++) {
            //   Serial.print(slipBuffer[i]);
            //   Serial.print("\t");
            // }
            // Serial.println("");
            spiMessage decodeMsg;
            SLIP::decode(slipBuffer, spiIdx, reinterpret_cast<uint8_t*>(&decodeMsg));
            Serial.println(decodeMsg.msg);
            Serial.println(decodeMsg.value);
            // wavelen0 = 15000;
            switch(decodeMsg.msg) {
              case WAVELEN0:
              {
                // wavelen0 = decodeMsg.value;
                // Serial.println(decodeMsg.value);
              }
              break;        
              case WAVELEN1:
              {
                // wavelen1 = decodeMsg.value;
                // Serial.println(decodeMsg.value);
              }
              break;        
              case WAVELEN2:
              {
                // wavelen2 = decodeMsg.value;
                // Serial.println(decodeMsg.value);
              }
              break;        
              case WAVELEN3:
              {
                // wavelen3 = decodeMsg.value;
              }
                break;
              case WAVELEN4:
              {
                // wavelen4 = decodeMsg.value;
              }
                break;
              case WAVELEN5:
              {
                // wavelen5 = decodeMsg.value;
              }
              break;
              // case BANK0:
              // {
              //   osc0Mode = decodeMsg.value;                
              // }
              // break;
              // case BANK1:
              // {
              //   osc1Mode = decodeMsg.value;
              // }
              // break;
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

  Serial.begin(115200);

  Serial1.setRX(13); //uart 1
  Serial1.setTX(12);
  Serial1.begin(115200);

  // queue_init(&coreCommsQueue, sizeof(queueItem), 3);

#ifdef RUNCORE0_OSCS
  smOsc0_dma_chan = smOsc0.init(pio0, 0, OSC1_PIN, programOffset, timing_buffer, dma_irh, clockdiv, DMA_IRQ_0);
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();

  smOsc1_dma_chan = smOsc1.init(pio0, 1, OSC2_PIN, programOffset, timing_buffer, dma_irh, clockdiv, DMA_IRQ_0);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  smOsc2_dma_chan = smOsc2.init(pio0, 2, OSC3_PIN, programOffset, timing_buffer, dma_irh, clockdiv, DMA_IRQ_0);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();
#endif
}


/* Fonction loop() */
void __not_in_flash_func(loop)() {
  readUart();
}


void setup1() {

#ifdef RUNCORE1_OSCS
  smOsc3_dma_chan = smOsc3.init(pio1, 0, OSC4_PIN, programOffset1, timing_buffer, dma_irh1, clockdiv, DMA_IRQ_1);
  smOsc3_dma_chan_bit = 1u << smOsc3_dma_chan;
  smOsc3.go();

  smOsc4_dma_chan = smOsc4.init(pio1, 1, OSC5_PIN, programOffset1, timing_buffer, dma_irh1, clockdiv, DMA_IRQ_1);
  smOsc4_dma_chan_bit = 1u << smOsc4_dma_chan;
  smOsc4.go();

  smOsc5_dma_chan = smOsc5.init(pio1, 2, OSC6_PIN, programOffset1, timing_buffer, dma_irh1, clockdiv, DMA_IRQ_1);
  smOsc5_dma_chan_bit = 1u << smOsc5_dma_chan;
  smOsc5.go();

#endif
}

void loop1() {
  // if (oscBankChange != -1) {
  //   oscBankChange = -1;
  //   // irq_set_enabled(PIO1_IRQ_0, false);
  //   // irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_sqr_coreB1);
  //   // irq_set_enabled(PIO1_IRQ_0, true);
  //   irq_handler_t usb_handler = irq_get_exclusive_handler(PIO1_IRQ_0);
  //   irq_remove_handler(PIO1_IRQ_0, usb_handler);
  //   irq_set_enabled(PIO1_IRQ_0, false);
  //   irq_set_exclusive_handler(PIO1_IRQ_0, irq_handler_saw_coreB1);
  //   irq_set_enabled(PIO1_IRQ_0, true);
  //   irq_set_priority(PIO1_IRQ_0, 10);
  //   pio1->inte0 = PIO_IRQ0_INTE_SM0_BITS;  
  // }
  __wfi();
}