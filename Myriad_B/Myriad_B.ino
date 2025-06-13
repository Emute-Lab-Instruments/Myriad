


#include "myriad_pins.h"
#include "hardware/sync.h"
#include "pios/pio_sq.h"
#include "pios/pio_pulse.h"
#include "pios/pio_expdec.h"
#include "pios/pio_bitbybit.h"
#include "smBitStreamOsc.h"
#include "myriad_messages.h"
#include "SLIP.h"
#include "oscillatorModels.hpp"
#include "bitstreams.hpp"
#include <memory>


#define RUNCORE0_OSCS
#define RUNCORE1_OSCS

bool core1_separate_stack = true;

// oscModelPtr FAST_MEM currOscModelBank0;
// oscModelPtr FAST_MEM currOscModelBank1;

std::array<oscModelPtr, 3> currOscModels0;
std::array<oscModelPtr, 3> currOscModels1;


bool oscillatorsAreRunning = false;
volatile bool FAST_MEM oscsReadyToStart=false;



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

volatile bool FAST_MEM bufSent0 = false;
volatile bool FAST_MEM bufSent1 = false;
volatile bool FAST_MEM bufSent2 = false;
volatile bool FAST_MEM bufSent3 = false;
volatile bool FAST_MEM bufSent4 = false;
volatile bool FAST_MEM bufSent5 = false;



/////////////////////////////////   IRQ 000000000000000000000000000000000000
void __isr dma_irh() {
  //TEST
  // Serial.println("dma");
  uint32_t triggered_channels = dma_hw->ints0;
  if (triggered_channels & smOsc0_dma_chan_bit) {
    dma_hw->ints0 = smOsc0_dma_chan_bit;  
    dma_hw->ch[smOsc0_dma_chan].al3_read_addr_trig = nextTimingBuffer0;
    bufSent0 = true;
  }
  else
  if (triggered_channels & smOsc1_dma_chan_bit) {
    dma_hw->ints0 = smOsc1_dma_chan_bit;
    dma_hw->ch[smOsc1_dma_chan].al3_read_addr_trig = nextTimingBuffer1;
    bufSent1 = true;
  }
  else
  if (triggered_channels & smOsc2_dma_chan_bit) {
    dma_hw->ints0 = smOsc2_dma_chan_bit;
    dma_hw->ch[smOsc2_dma_chan].al3_read_addr_trig = nextTimingBuffer2;
    bufSent2 = true;
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

volatile bool FAST_MEM oscsRunning0 = false;
volatile bool FAST_MEM oscsRunning1 = false;



/////////////////////////////////   IRQ 11111111111111111111111111111
void __not_in_flash_func(dma_irh1)() {
  uint32_t triggered_channels = dma_hw->ints1;
  if (triggered_channels & smOsc3_dma_chan_bit) {
    dma_hw->ints1 = smOsc3_dma_chan_bit;  
    dma_hw->ch[smOsc3_dma_chan].al3_read_addr_trig = nextTimingBuffer3;
    bufSent3 = true;
  }
  else
  if (triggered_channels & smOsc4_dma_chan_bit) {
  //   // Channel 0 triggered, handle it
    dma_hw->ints1 = smOsc4_dma_chan_bit;
    dma_hw->ch[smOsc4_dma_chan].al3_read_addr_trig = nextTimingBuffer4;
    bufSent4 = true;
  }
  else
  if (triggered_channels & smOsc5_dma_chan_bit) {
    dma_hw->ints1 = smOsc5_dma_chan_bit;
    dma_hw->ch[smOsc5_dma_chan].al3_read_addr_trig = nextTimingBuffer5;
    bufSent5 = true;
  }
}


smBitStreamOsc FAST_MEM smOsc3;
smBitStreamOsc FAST_MEM smOsc4;
smBitStreamOsc FAST_MEM smOsc5;


// const size_t __not_in_flash("mydata") BUF_LEN = 10;
// uint8_t __not_in_flash("mydata") spi_in_buf[BUF_LEN];

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

void assignOscModels0(size_t modelIdx) {
  for(auto &model: currOscModels0) {
    model = oscModelFactories[modelIdx](); 

  }
}
void assignOscModels1(size_t modelIdx) {
  for(auto &model: currOscModels1) {
    model = oscModelFactories[modelIdx](); 
  }
}

void __force_inline calculateOscBuffers0() {
  if ((currOscModels0[0]->updateBufferInSyncWithDMA && bufSent0) || (!currOscModels0[0]->updateBufferInSyncWithDMA && currOscModels0[0]->newFreq)) {
    currOscModels0[0]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModels0[0], currOscModels0[0]->wavelen);
    bufSent0 = false;
  }
  if ((currOscModels0[1]->updateBufferInSyncWithDMA && bufSent1) || (!currOscModels0[1]->updateBufferInSyncWithDMA && currOscModels0[1]->newFreq)) {
    currOscModels0[1]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModels0[1], currOscModels0[1]->wavelen);
    bufSent1 = false;
  }
  if ((currOscModels0[2]->updateBufferInSyncWithDMA && bufSent2) || (!currOscModels0[2]->updateBufferInSyncWithDMA && currOscModels0[2]->newFreq)) {
    currOscModels0[2]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModels0[2], currOscModels0[2]->wavelen);
    bufSent2 = false;
  }
}

void __force_inline calculateOscBuffers1() {
  if ((currOscModels1[0]->updateBufferInSyncWithDMA && bufSent3) || (!currOscModels1[0]->updateBufferInSyncWithDMA && currOscModels1[0]->newFreq)) {
    currOscModels1[0]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer3, timing_swapbuffer_3_A, timing_swapbuffer_3_B, currOscModels1[0], currOscModels1[0]->wavelen);
    bufSent3 = false;
  }
  if ((currOscModels1[1]->updateBufferInSyncWithDMA && bufSent4) || (!currOscModels1[1]->updateBufferInSyncWithDMA && currOscModels1[1]->newFreq)) {
    currOscModels1[1]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer4, timing_swapbuffer_4_A, timing_swapbuffer_4_B, currOscModels1[1], currOscModels1[1]->wavelen);
    bufSent4 = false;
  }
  if ((currOscModels1[2]->updateBufferInSyncWithDMA && bufSent5) || (!currOscModels1[2]->updateBufferInSyncWithDMA && currOscModels1[2]->newFreq)) {
    currOscModels1[2]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer5, timing_swapbuffer_5_A, timing_swapbuffer_5_B, currOscModels1[2], currOscModels1[2]->wavelen);
    bufSent5 = false;
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
                // updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModels0[0], decodeMsg.value);
                currOscModels0[0]->wavelen = decodeMsg.value;
                currOscModels0[0]->newFreq = true; //trigger buffer refill
              }
              break;        
              case WAVELEN1:
              {
                // updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModels0[1], decodeMsg.value);
                currOscModels0[1]->wavelen = decodeMsg.value;
                currOscModels0[1]->newFreq = true; //trigger buffer refill
              }
              break;        
              case WAVELEN2:
              {
                // updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModels0[2], decodeMsg.value);
                currOscModels0[2]->wavelen = decodeMsg.value;
                currOscModels0[2]->newFreq = true; //trigger buffer refill
              }
              break;        
              case WAVELEN3:
              {
                // updateTimingBuffer(nextTimingBuffer3, timing_swapbuffer_3_A, timing_swapbuffer_3_B, currOscModels1[0], decodeMsg.value);
                currOscModels1[0]->wavelen = decodeMsg.value;
                currOscModels1[0]->newFreq = true; //trigger buffer refill
              }
                break;
              case WAVELEN4:
              {
                // updateTimingBuffer(nextTimingBuffer4, timing_swapbuffer_4_A, timing_swapbuffer_4_B, currOscModels1[1], decodeMsg.value);
                currOscModels1[1]->wavelen = decodeMsg.value;
                currOscModels1[1]->newFreq = true; //trigger buffer refill
              }
                break;
              case WAVELEN5:
              {
                // updateTimingBuffer(nextTimingBuffer5, timing_swapbuffer_5_A, timing_swapbuffer_5_B, currOscModels1[2], decodeMsg.value);
                currOscModels1[2]->wavelen = decodeMsg.value;
                currOscModels1[2]->newFreq = true; //trigger buffer refill
                //freqs sent sequentially, so all oscs should now have a freq
                oscsReadyToStart = true;
              }
              break;
              case CTRL0:
              {
                const float v = decodeMsg.value;
                currOscModels0[0]->ctrl(v); 
                // Serial.println(v);
                break;
              }
              case CTRL1:
              {
                const float v = decodeMsg.value;
                currOscModels0[1]->ctrl(v); 
                break;
              }
              case CTRL2:
              {
                const float v = decodeMsg.value;
                currOscModels0[2]->ctrl(v); 
                break;
              }
              case CTRL3:
              {
                const float v = decodeMsg.value;
                currOscModels1[0]->ctrl(v); 
                break;
              }
              case CTRL4:
              {
                const float v = decodeMsg.value;
                currOscModels1[1]->ctrl(v); 
                break;
              }
              case CTRL5:
              {
                const float v = decodeMsg.value;
                currOscModels1[2]->ctrl(v); 
                break;
              }
              case BANK0:
              {
                // Serial.println("bank0");
                stopOscBankA();
                assignOscModels0(decodeMsg.value);
                //refill from new oscillator
                //trigger buffer refills
                currOscModels0[0]->newFreq=true;
                currOscModels0[1]->newFreq=true;
                currOscModels0[2]->newFreq=true;

                calculateOscBuffers0();

                startOscBankA();
              }
              break;
              case BANK1:
              {
                // Serial.println("bank1");
                // // Serial.println(decodeMsg.value);
                stopOscBankB();
                assignOscModels1(decodeMsg.value);
                //refill from new oscillator
                //trigger buffer refills
                currOscModels1[0]->newFreq=true;
                currOscModels1[1]->newFreq=true;
                currOscModels1[2]->newFreq=true;

                calculateOscBuffers1();

                startOscBankB();
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

void startOscBankA() {

  Serial.println("StartoscbankA");

  pio_clear_instruction_memory(pio0);
  uint programOffset = currOscModels0[0]->loadProg(pio0);
  pio_sm_config baseConfig = currOscModels0[0]->getBaseConfig(programOffset);

  smOsc0_dma_chan = smOsc0.init(pio0, 0, OSC1_PIN, programOffset, baseConfig, nextTimingBuffer0, dma_irh, clockdiv, currOscModels0[0]->loopLength, DMA_IRQ_0);
  Serial.println("init");
  if (smOsc0_dma_chan < 0) {
    Serial.println("dma chan allocation error");
  }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();

  smOsc1_dma_chan = smOsc1.init(pio0, 1, OSC2_PIN, programOffset, baseConfig, nextTimingBuffer1, dma_irh, clockdiv, currOscModels0[1]->loopLength, DMA_IRQ_0);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  smOsc2_dma_chan = smOsc2.init(pio0, 2, OSC3_PIN, programOffset, baseConfig, nextTimingBuffer2, dma_irh, clockdiv, currOscModels0[2]->loopLength, DMA_IRQ_0);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();

  oscsRunning0 = true;
}

void stopOscBankA() {
  oscsRunning0 = false;
  smOsc0.stop();
  smOsc1.stop();
  smOsc2.stop();
}

void startOscBankB() {

  Serial.println("StartoscbankB");
  pio_clear_instruction_memory(pio1);
  uint programOffset = currOscModels1[0]->loadProg(pio1);
  pio_sm_config baseConfig = currOscModels1[0]->getBaseConfig(programOffset);


  smOsc3_dma_chan = smOsc3.init(pio1, 0, OSC4_PIN, programOffset, baseConfig, nextTimingBuffer3, dma_irh1, clockdiv, currOscModels1[0]->loopLength, DMA_IRQ_1);
  smOsc3_dma_chan_bit = 1u << smOsc3_dma_chan;
  smOsc3.go();

  smOsc4_dma_chan = smOsc4.init(pio1, 1, OSC5_PIN, programOffset, baseConfig, nextTimingBuffer4, dma_irh1, clockdiv, currOscModels1[1]->loopLength, DMA_IRQ_1);
  smOsc4_dma_chan_bit = 1u << smOsc4_dma_chan;
  smOsc4.go();

  smOsc5_dma_chan = smOsc5.init(pio1, 2, OSC6_PIN, programOffset, baseConfig, nextTimingBuffer5, dma_irh1, clockdiv, currOscModels1[2]->loopLength, DMA_IRQ_1);
  smOsc5_dma_chan_bit = 1u << smOsc5_dma_chan;
  smOsc5.go();
  oscsRunning1 = true;


}

void stopOscBankB() {
  oscsRunning1 = false;
  smOsc3.stop();
  smOsc4.stop();
  smOsc5.stop();
}

const size_t oscStartDelay = 200;

void setup() {
  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  // currOscModelBank0 = oscModel2;
  assignOscModels0(0);


  Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB
  // }

  Serial1.setRX(13); //uart 1
  Serial1.setTX(12);
  Serial1.begin(115200);


  // queue_init(&coreCommsQueue, sizeof(queueItem), 3);
#ifdef RUNCORE0_OSCS
  delay(oscStartDelay);
  // while(!oscsReadyToStart) {
  //   ;
  // }
  Serial.println("Start");
  startOscBankA();
  Serial.println("Started");
#endif
}


/* Fonction loop() */
void __not_in_flash_func(loop)() {
  readUart();
  if (oscsRunning0) {
    calculateOscBuffers0();
  }

  // delay(1);
  // __wfi();
}


void setup1() {
//  currOscModelBank1 = oscModel2;
  assignOscModels1(0);

#ifdef RUNCORE1_OSCS
  // while(!oscsReadyToStart) {
  //   ;
  // }
  delay(oscStartDelay);
  Serial.println("StartB");
  startOscBankB();
  Serial.println("StartedB");
#endif
}

void loop1() {
  if (oscsRunning1) {
    calculateOscBuffers1();
  }
}