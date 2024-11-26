


#include "myriad_pins.h"
#include "myriad_setup.h"
#include "hardware/sync.h"
// #include "pio_expdec.h"
#include "pios/pio_sq.h"
#include "smBitStreamOsc.h"
#include "myriad_messages.h"
#include "SLIP.h"
#include <memory>


#define RUNCORE0_OSCS
// #define RUNCORE1_OSCS
#define FAST_MEM __not_in_flash("mydata")

bool core1_separate_stack = true;

//todo: try sliding window across template array
//todo: try feedback system to generate template values
//todo: perlin noise
//todo: time varying buffers

class oscillatorModel {
public:
  oscillatorModel() {};
  pio_program prog;
  virtual void fillBuffer(uint32_t* bufferA, size_t wavelen)=0;
  size_t loopLength;
};



class squareOscillatorModel : public virtual oscillatorModel {
public:
  squareOscillatorModel() : oscillatorModel(){
    loopLength=2;
    prog=pin_ctrl_program;
  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen);
    }
  }
  std::vector<float> oscTemplate {0.1,0.9};
};

class squareOscillatorModel2 : public oscillatorModel {
public:
  squareOscillatorModel2() : oscillatorModel() {
    loopLength=2;
    prog=pin_ctrl_program;
  }
  inline void fillBuffer(uint32_t* bufferA, size_t wavelen) {
    for (size_t i = 0; i < oscTemplate.size(); ++i) {
        *(bufferA + i) = static_cast<uint32_t>(oscTemplate[i] * wavelen);
    }
  }
  std::vector<float> oscTemplate {0.5,0.5};
};

using oscModelPtr = std::unique_ptr<oscillatorModel>;

oscModelPtr FAST_MEM currOscModelBank0;
oscModelPtr FAST_MEM currOscModelBank1;

float clockdiv = 8;
uint32_t clockHz = 15625000 / 80;
size_t cpuClock=125000000;
float sampleClock = cpuClock / clockdiv;
uint32_t mwavelen = 125000000/clockdiv /80;
uint32_t mwavelen2 = mwavelen * 1.01;
uint32_t mwavelen3 = mwavelen2 * 1.01;
uint32_t mwavelen4 = mwavelen3 * 1.01;
uint32_t mwavelen5 = mwavelen4 * 1.01;
uint32_t mwavelen6 = mwavelen5 * 1.01;
// Ensure `timing_buffer` is aligned to 16-bytes so we can use DMA address
// wrapping
// std::vector<float> sqrTemplate {0.1,1.9};

// uint32_t timing_buffer[8] __attribute__((aligned(16))) {clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1};
// uint32_t timing_buffer2[8] __attribute__((aligned(16))) {clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1,clockHz>>1};

#define DEFINE_TIMING_SWAPBUFFERS(name) \
uint32_t FAST_MEM timing_swapbuffer_##name##_A[24] __attribute__((aligned(16))) {mwavelen,mwavelen}; \
uint32_t FAST_MEM timing_swapbuffer_##name##_B[24] __attribute__((aligned(16))) {mwavelen,mwavelen}; \
io_rw_32 FAST_MEM nextTimingBuffer##name = (io_rw_32)timing_swapbuffer_##name##_A;


// io_rw_32 nextTimingBuffer1 = (io_rw_32)timing_swapbuffer_1_A;

// alignas(16) uint32_t FAST_MEM name[2] = {mwavelen, mwavelen};

DEFINE_TIMING_SWAPBUFFERS(0)
DEFINE_TIMING_SWAPBUFFERS(1)
DEFINE_TIMING_SWAPBUFFERS(2)
DEFINE_TIMING_SWAPBUFFERS(3)
DEFINE_TIMING_SWAPBUFFERS(4)
DEFINE_TIMING_SWAPBUFFERS(5)


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
    dma_hw->ch[smOsc0_dma_chan].al3_read_addr_trig = nextTimingBuffer0;
  }
  else
  if (triggered_channels & smOsc1_dma_chan_bit) {
  //   // Channel 0 triggered, handle it
    dma_hw->ints0 = smOsc1_dma_chan_bit;
  //   dma_hw->ints0 &= smOsc0_dma_chan_bit_inv;  // Clear interrupt 
    dma_hw->ch[smOsc1_dma_chan].al3_read_addr_trig = nextTimingBuffer1;
  }
  else
  if (triggered_channels & smOsc2_dma_chan_bit) {
    dma_hw->ints0 = smOsc2_dma_chan_bit;
    dma_hw->ch[smOsc2_dma_chan].al3_read_addr_trig = nextTimingBuffer2;
  }
}
smBitStreamOsc FAST_MEM smOsc0;
smBitStreamOsc FAST_MEM smOsc1;
smBitStreamOsc FAST_MEM smOsc2;


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
    dma_hw->ch[smOsc3_dma_chan].al3_read_addr_trig = nextTimingBuffer3;
  }
  else
  if (triggered_channels & smOsc4_dma_chan_bit) {
  //   // Channel 0 triggered, handle it
    dma_hw->ints1 = smOsc4_dma_chan_bit;
  //   dma_hw->ints0 &= smOsc0_dma_chan_bit_inv;  // Clear interrupt 
    dma_hw->ch[smOsc4_dma_chan].al3_read_addr_trig = nextTimingBuffer4;
  }
  else
  if (triggered_channels & smOsc5_dma_chan_bit) {
    dma_hw->ints1 = smOsc5_dma_chan_bit;
    dma_hw->ch[smOsc5_dma_chan].al3_read_addr_trig = nextTimingBuffer5;
  }
}


smBitStreamOsc FAST_MEM smOsc3;
smBitStreamOsc FAST_MEM smOsc4;
smBitStreamOsc FAST_MEM smOsc5;



const size_t __not_in_flash("mydata") BUF_LEN = 10;
uint8_t __not_in_flash("mydata") spi_in_buf[BUF_LEN];



static uint8_t __not_in_flash("mydata") slipBuffer[64];

enum SPISTATES {WAITFOREND, ENDORBYTES, READBYTES};
static SPISTATES  __not_in_flash("mydata") spiState = SPISTATES::WAITFOREND;
static int __not_in_flash("mydata") spiIdx=0;

int oscBankChange=-1;

void restart_sm(PIO pio, uint sm) {
  // Reset the state machine program counter to zero
  // pio_sm_exec(pio, sm, pio_encode_jmp(0));
  
  // Alternatively, you can use the RESTART flag directly:
  pio->ctrl = (1u << (PIO_CTRL_SM_RESTART_LSB + sm));
}



void updateTimingBuffer(io_rw_32 &nextBuf,
                                  uint32_t* bufferA, uint32_t* bufferB,
                                  oscModelPtr& oscModel,
                                  float oscWavelength) {
    if (nextBuf == reinterpret_cast<io_rw_32>(bufferA)) {

        // for (size_t i = 0; i < sqrTemplate.size(); ++i) {
        //     *(bufferB + i) = static_cast<uint32_t>(sqrTemplate[i] * oscWavelength);
        // }
        oscModel->fillBuffer(bufferB, oscWavelength);
        nextBuf = reinterpret_cast<io_rw_32>(bufferB);
    } else {
        // for (size_t i = 0; i < sqrTemplate.size(); ++i) {
        //     *(bufferA + i) = static_cast<uint32_t>(sqrTemplate[i] * oscWavelength);
        // }
        oscModel->fillBuffer(bufferA, oscWavelength);
        nextBuf = reinterpret_cast<io_rw_32>(bufferA);
    }
}

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
            // Serial.print(decodeMsg.msg);
            // Serial.print(": ");
            
            // Serial.println(decodeMsg.value);
            switch(decodeMsg.msg) {
              case WAVELEN0:
              {
                
                updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModelBank0, decodeMsg.value);
              }
              break;        
              case WAVELEN1:
              {
                updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModelBank0, decodeMsg.value);
              }
              break;        
              case WAVELEN2:
              {
                updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModelBank0, decodeMsg.value);
              }
              break;        
              case WAVELEN3:
              {
                updateTimingBuffer(nextTimingBuffer3, timing_swapbuffer_3_A, timing_swapbuffer_3_B, currOscModelBank1, decodeMsg.value);
              }
                break;
              case WAVELEN4:
              {
                updateTimingBuffer(nextTimingBuffer4, timing_swapbuffer_4_A, timing_swapbuffer_4_B, currOscModelBank1, decodeMsg.value);
              }
                break;
              case WAVELEN5:
              {
                updateTimingBuffer(nextTimingBuffer5, timing_swapbuffer_5_A, timing_swapbuffer_5_B, currOscModelBank1, decodeMsg.value);
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

uint programOffset;

void startOscBankA() {
  // pio_clear_instruction_memory(pio0);


  smOsc0_dma_chan = smOsc0.init(pio0, 0, OSC1_PIN, programOffset, nextTimingBuffer0, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_0);
  if (smOsc0_dma_chan < 0) {
    Serial.println("dma chan allocation error");
  }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();

  smOsc1_dma_chan = smOsc1.init(pio0, 1, OSC2_PIN, programOffset, nextTimingBuffer1, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_0);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  smOsc2_dma_chan = smOsc2.init(pio0, 2, OSC3_PIN, programOffset, nextTimingBuffer2, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_0);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();
}

void stopOscBankA() {
  smOsc0.stop();
  smOsc1.stop();
  smOsc2.stop();
}

void setup() {
  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  currOscModelBank0 = std::make_unique<squareOscillatorModel2>();
  programOffset = pio_add_program(pio0, &currOscModelBank0->prog);


  Serial.begin(115200);

  Serial1.setRX(13); //uart 1
  Serial1.setTX(12);
  Serial1.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  // queue_init(&coreCommsQueue, sizeof(queueItem), 3);
#ifdef RUNCORE0_OSCS
  // currOscModelBank0 = std::make_unique<squareOscillatorModel2>();
  // uint programOffset = pio_add_program(pio0, &currOscModelBank0->prog);

  // smOsc0_dma_chan = smOsc0.init(pio0, 0, OSC1_PIN, programOffset, timing_swapbuffer_0_A, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_0);
  // smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  // smOsc0.go();

  // smOsc1_dma_chan = smOsc1.init(pio0, 1, OSC2_PIN, programOffset, timing_swapbuffer_1_A, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_0);
  // smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  // smOsc1.go();

  // smOsc2_dma_chan = smOsc2.init(pio0, 2, OSC3_PIN, programOffset, timing_swapbuffer_2_A, dma_irh, clockdiv, currOscModelBank0->loopLength, DMA_IRQ_0);
  // smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  // smOsc2.go();


  Serial.println("Start");
  startOscBankA();
  // currOscModelBank0.release();

  // delay(1000);
  // Serial.println("Stop");
  // stopOscBankA();
  // delay(1000);
  // Serial.println("Start");
  // // currOscModelBank0 = std::make_unique<squareOscillatorModel2>();
  // startOscBankA();
#endif
}


/* Fonction loop() */
void __not_in_flash_func(loop)() {
  readUart();
}


void setup1() {
  currOscModelBank1 = std::make_unique<squareOscillatorModel>();
  uint programOffset1 = pio_add_program(pio1, &currOscModelBank1->prog);

#ifdef RUNCORE1_OSCS
  smOsc3_dma_chan = smOsc3.init(pio1, 0, OSC4_PIN, programOffset1, timing_swapbuffer_3_A, dma_irh1, clockdiv, currOscModelBank1->loopLength, DMA_IRQ_1);
  smOsc3_dma_chan_bit = 1u << smOsc3_dma_chan;
  smOsc3.go();

  smOsc4_dma_chan = smOsc4.init(pio1, 1, OSC5_PIN, programOffset1, timing_swapbuffer_4_A, dma_irh1, clockdiv, currOscModelBank1->loopLength, DMA_IRQ_1);
  smOsc4_dma_chan_bit = 1u << smOsc4_dma_chan;
  smOsc4.go();

  smOsc5_dma_chan = smOsc5.init(pio1, 2, OSC6_PIN, programOffset1, timing_swapbuffer_5_A, dma_irh1, clockdiv, currOscModelBank1->loopLength, DMA_IRQ_1);
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