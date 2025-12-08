


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
#include "myriad_setup.h"
#include <memory>
#include "octaves.hpp"
#include "streamMessaging.hpp"
#include "fixedpoint.hpp"

using namespace FixedPoint;

#define RUNCORE0_OSCS
#define RUNCORE1_OSCS

bool core1_separate_stack = true;
#define CORE0_FAST_MEM __scratch_x("myriad")
#define CORE1_FAST_MEM __scratch_y("myriad")

#define DMACH_CORE0_OSC0 0
#define DMACH_CORE0_OSC1 2
#define DMACH_CORE0_OSC2 4
#define DMACH_CORE1_OSC0 1
#define DMACH_CORE1_OSC1 3
#define DMACH_CORE1_OSC2 5
#define DMACH_SERIAL_RX_A 6
#define DMACH_SERIAL_RX_B 7


// oscModelPtr FAST_MEM currOscModelBank0;
// oscModelPtr FAST_MEM currOscModelBank1;

std::array<oscModelPtr, 3> FAST_MEM currOscModels0;
std::array<oscModelPtr, 3> FAST_MEM currOscModels1;

static spin_lock_t FAST_MEM *calcOscsSpinlock0;
static spin_lock_t FAST_MEM *calcOscsSpinlock1;



volatile bool FAST_MEM waitingForFirstFrequency0=true;
volatile bool FAST_MEM oscsStartedAfterFirstFrequency0=false;
volatile bool FAST_MEM waitingForFirstFrequency1=true;
volatile bool FAST_MEM oscsStartedAfterFirstFrequency1=false;

// Track current bank types to prevent redundant changes
volatile size_t FAST_MEM currentBank0Type = 0;
volatile size_t FAST_MEM currentBank1Type = 0;
volatile size_t FAST_MEM requestedBank1=0;

// Track PIO program offsets for cleanup
static uint bank0_program_offset = 0;
static bool bank0_program_loaded = false;
static uint bank1_program_offset = 0;
static bool bank1_program_loaded = false;



DEFINE_TIMING_SWAPBUFFERS(0, __scratch_x("swap"))
DEFINE_TIMING_SWAPBUFFERS(1, __scratch_x("swap"))
DEFINE_TIMING_SWAPBUFFERS(2, __scratch_x("swap"))
DEFINE_TIMING_SWAPBUFFERS(3, __scratch_y("swap"))
DEFINE_TIMING_SWAPBUFFERS(4, __scratch_y("swap"))
DEFINE_TIMING_SWAPBUFFERS(5, __scratch_y("swap"))


uint32_t CORE0_FAST_MEM smOsc0_dma_chan;
uint32_t CORE0_FAST_MEM smOsc0_dma_chan_bit;

uint32_t CORE0_FAST_MEM smOsc1_dma_chan;
uint32_t CORE0_FAST_MEM smOsc1_dma_chan_bit;

uint32_t CORE0_FAST_MEM smOsc2_dma_chan;
uint32_t CORE0_FAST_MEM smOsc2_dma_chan_bit;

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


uint32_t CORE1_FAST_MEM smOsc3_dma_chan;
uint32_t CORE1_FAST_MEM smOsc3_dma_chan_bit;
uint32_t CORE1_FAST_MEM smOsc4_dma_chan;
uint32_t CORE1_FAST_MEM smOsc4_dma_chan_bit;
uint32_t CORE1_FAST_MEM smOsc5_dma_chan;
uint32_t CORE1_FAST_MEM smOsc5_dma_chan_bit;

volatile bool FAST_MEM oscsRunning0 = false;
volatile bool FAST_MEM oscsRunning1 = false;

// float ctrlVal = 0.f;
Fixed<16,16> FAST_MEM epsilon_fixed(0);




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
  // Serial.printf("assignOscModels0: modelIdx=%d\n", modelIdx);
  // Serial.flush();

  for(size_t i = 0; i < currOscModels0.size(); i++) {
    // Serial.printf("  Creating model %d...\n", i);
    // Serial.flush();

    currOscModels0[i] = oscModelFactories[modelIdx]();

    // Serial.printf("  Model %d created\n", i);
    // Serial.flush();
  }

  // Serial.println("assignOscModels0 complete");
  // Serial.flush();
}
void assignOscModels1(size_t modelIdx) {
  for(auto &model: currOscModels1) {
    model = oscModelFactories[modelIdx](); 
  }
}

void __force_inline calculateOscBuffers0() {
  if ((currOscModels0[0]->updateBufferInSyncWithDMA && bufSent0) || (!currOscModels0[0]->updateBufferInSyncWithDMA && currOscModels0[0]->newFreq)) {
    currOscModels0[0]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer0, timing_swapbuffer_0_A, timing_swapbuffer_0_B, currOscModels0[0]);
    bufSent0 = false;
  }
  if ((currOscModels0[1]->updateBufferInSyncWithDMA && bufSent1) || (!currOscModels0[1]->updateBufferInSyncWithDMA && currOscModels0[1]->newFreq)) {
    currOscModels0[1]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer1, timing_swapbuffer_1_A, timing_swapbuffer_1_B, currOscModels0[1]);
    bufSent1 = false;
  }
  if ((currOscModels0[2]->updateBufferInSyncWithDMA && bufSent2) || (!currOscModels0[2]->updateBufferInSyncWithDMA && currOscModels0[2]->newFreq)) {
    currOscModels0[2]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer2, timing_swapbuffer_2_A, timing_swapbuffer_2_B, currOscModels0[2]);
    bufSent2 = false;
  }
}

void __force_inline calculateOscBuffers1() {
  if ((currOscModels1[0]->updateBufferInSyncWithDMA && bufSent3) || (!currOscModels1[0]->updateBufferInSyncWithDMA && currOscModels1[0]->newFreq)) {
    currOscModels1[0]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer3, timing_swapbuffer_3_A, timing_swapbuffer_3_B, currOscModels1[0]);
    bufSent3 = false;
  }
  if ((currOscModels1[1]->updateBufferInSyncWithDMA && bufSent4) || (!currOscModels1[1]->updateBufferInSyncWithDMA && currOscModels1[1]->newFreq)) {
    currOscModels1[1]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer4, timing_swapbuffer_4_A, timing_swapbuffer_4_B, currOscModels1[1]);
    bufSent4 = false;
  }
  if ((currOscModels1[2]->updateBufferInSyncWithDMA && bufSent5) || (!currOscModels1[2]->updateBufferInSyncWithDMA && currOscModels1[2]->newFreq)) {
    currOscModels1[2]->newFreq = false;
    updateTimingBuffer(nextTimingBuffer5, timing_swapbuffer_5_A, timing_swapbuffer_5_B, currOscModels1[2]);
    bufSent5 = false;
  }
}

// inline void __not_in_flash_func(setCtrl)() {
//   currOscModels0[0]->ctrl(epsilon_fixed);
//   currOscModels0[1]->ctrl(epsilon_fixed);
//   currOscModels0[2]->ctrl(epsilon_fixed);
//   currOscModels1[0]->ctrl(epsilon_fixed);
//   currOscModels1[1]->ctrl(epsilon_fixed);
//   currOscModels1[2]->ctrl(epsilon_fixed);
// }

// float FAST_MEM detune = 0.f;
Fixed<20,12> FAST_MEM detuneFixed(0);
Q16_16 FAST_MEM metaModWavelenMul3(1);
Q16_16 FAST_MEM metaModWavelenMul4(1);
Q16_16 FAST_MEM metaModWavelenMul5(1);
Q16_16 FAST_MEM metaModWavelenMul6(1);
Q16_16 FAST_MEM metaModWavelenMul7(1);
Q16_16 FAST_MEM metaModWavelenMul8(1);

float FAST_MEM currOct3=1;
float FAST_MEM currOct4=1;
float FAST_MEM currOct5=1;
float FAST_MEM currOct6=1;
float FAST_MEM currOct7=1;
float FAST_MEM currOct8=1;

size_t FAST_MEM octaveIdx = 0;

volatile bool FAST_MEM newFrequenciesReady0 = false;
volatile bool FAST_MEM newFrequenciesReady1 = false;

// float new_wavelen0 = 10000;
Fixed<20,12> new_wavelen2_fixed;


// static uint8_t __scratch_x("mydata") slipBuffer[64];
volatile bool FAST_MEM newCtrlReady = false;


void startOscBankA() {

  uint programOffset = currOscModels0[0]->loadProg(pio0);


  pio_sm_config baseConfig = currOscModels0[0]->getBaseConfig(programOffset);
  const size_t modelClockDiv = currOscModels0[0]->getClockDiv();

  smOsc0_dma_chan = smOsc0.init(pio0, 0, OSC1_PIN, programOffset, baseConfig, nextTimingBuffer0, dma_irh, modelClockDiv, currOscModels0[0]->loopLength, DMA_IRQ_0, DMACH_CORE0_OSC0);
  if (smOsc0_dma_chan < 0) {
    Serial.println("dma chan allocation error");
  }
  smOsc0_dma_chan_bit = 1u << smOsc0_dma_chan;
  smOsc0.go();

  smOsc1_dma_chan = smOsc1.init(pio0, 1, OSC2_PIN, programOffset, baseConfig, nextTimingBuffer1, dma_irh, modelClockDiv, currOscModels0[1]->loopLength, DMA_IRQ_0,DMACH_CORE0_OSC1);
  smOsc1_dma_chan_bit = 1u << smOsc1_dma_chan;
  smOsc1.go();

  smOsc2_dma_chan = smOsc2.init(pio0, 2, OSC3_PIN, programOffset, baseConfig, nextTimingBuffer2, dma_irh, modelClockDiv, currOscModels0[2]->loopLength, DMA_IRQ_0, DMACH_CORE0_OSC2);
  smOsc2_dma_chan_bit = 1u << smOsc2_dma_chan;
  smOsc2.go();

  oscsRunning0 = true;
}

void stopOscBankA() {

  oscsRunning0 = false;

  smOsc0.stop();
  smOsc1.stop();
  smOsc2.stop();

  currOscModels0[0]->unloadProg(pio0);

  // Clear the buffers explicitly
  memset(timing_swapbuffer_0_A, 0, sizeof(timing_swapbuffer_0_A));
  memset(timing_swapbuffer_0_B, 0, sizeof(timing_swapbuffer_0_B));
  memset(timing_swapbuffer_1_A, 0, sizeof(timing_swapbuffer_1_A));
  memset(timing_swapbuffer_1_B, 0, sizeof(timing_swapbuffer_1_B));
  memset(timing_swapbuffer_2_A, 0, sizeof(timing_swapbuffer_2_A));
  memset(timing_swapbuffer_2_B, 0, sizeof(timing_swapbuffer_2_B));

  // Reset buffer pointers to A buffers
  nextTimingBuffer0 = (io_rw_32)timing_swapbuffer_0_A;
  nextTimingBuffer1 = (io_rw_32)timing_swapbuffer_1_A;
  nextTimingBuffer2 = (io_rw_32)timing_swapbuffer_2_A;

  bufSent0 = false;
  bufSent1 = false;
  bufSent2 = false;

}

void startOscBankB() {


  // Don't clear entire PIO - this would destroy the RX program
  // The oscillator loadProg() will add its program without clearing others
  // pio_clear_instruction_memory(pio1);

  uint programOffset = currOscModels1[0]->loadProg(pio1);
  pio_sm_config baseConfig = currOscModels1[0]->getBaseConfig(programOffset);

  const size_t modelClockDiv = currOscModels1[0]->getClockDiv();

  // Monitor PIO program placement
  // Check how much PIO memory is used (32 instructions max)

  smOsc3_dma_chan = smOsc3.init(pio1, 0, OSC4_PIN, programOffset, baseConfig, nextTimingBuffer3, dma_irh1, modelClockDiv, currOscModels1[0]->loopLength, DMA_IRQ_1, DMACH_CORE1_OSC0);
  smOsc3_dma_chan_bit = 1u << smOsc3_dma_chan;
  smOsc3.go();

  smOsc4_dma_chan = smOsc4.init(pio1, 1, OSC5_PIN, programOffset, baseConfig, nextTimingBuffer4, dma_irh1, modelClockDiv, currOscModels1[1]->loopLength, DMA_IRQ_1, DMACH_CORE1_OSC1);
  smOsc4_dma_chan_bit = 1u << smOsc4_dma_chan;
  smOsc4.go();

  smOsc5_dma_chan = smOsc5.init(pio1, 2, OSC6_PIN, programOffset, baseConfig, nextTimingBuffer5, dma_irh1, modelClockDiv, currOscModels1[2]->loopLength, DMA_IRQ_1), DMACH_CORE1_OSC2;
  smOsc5_dma_chan_bit = 1u << smOsc5_dma_chan;
  smOsc5.go();
  oscsRunning1 = true;


}

void stopOscBankB() {
  oscsRunning1 = false;

  smOsc3.stop();
  smOsc4.stop();
  smOsc5.stop();

  currOscModels1[0]->unloadProg(pio1);


  memset(timing_swapbuffer_3_A, 0, sizeof(timing_swapbuffer_3_A));
  memset(timing_swapbuffer_3_B, 0, sizeof(timing_swapbuffer_3_B));
  memset(timing_swapbuffer_4_A, 0, sizeof(timing_swapbuffer_4_A));
  memset(timing_swapbuffer_4_B, 0, sizeof(timing_swapbuffer_4_B));
  memset(timing_swapbuffer_5_A, 0, sizeof(timing_swapbuffer_5_A));
  memset(timing_swapbuffer_5_B, 0, sizeof(timing_swapbuffer_5_B));
  
  // Reset buffer pointers to A buffers
  nextTimingBuffer3 = (io_rw_32)timing_swapbuffer_3_A;
  nextTimingBuffer4 = (io_rw_32)timing_swapbuffer_4_A;
  nextTimingBuffer5 = (io_rw_32)timing_swapbuffer_5_A;  

  bufSent3 = false;
  bufSent4 = false;
  bufSent5 = false;

}

__force_inline void __not_in_flash_func(processSerialMessage)(streamMessaging::msgpacket &msg) {
  switch(msg.msgType) {
    case streamMessaging::messageTypes::WAVELEN0:
    {
      
      // uint32_t save = spin_lock_blocking(calcOscsSpinlock0);  
      // new_wavelen0 = msg.value.floatValue;
      Fixed<20,12> new_wavelen0_fixed = Fixed<20,12>::from_raw(msg.value.intValue);
      new_wavelen2_fixed = new_wavelen0_fixed - detuneFixed - detuneFixed;
      newFrequenciesReady0 = true;
      newFrequenciesReady1 = true;
      waitingForFirstFrequency0 = false;
      waitingForFirstFrequency1 = false;
      break;
    }
    case streamMessaging::messageTypes::DETUNE:
    {
      detuneFixed = Fixed<20,12>::from_raw(msg.value.intValue);
      break;
    }     
    case streamMessaging::messageTypes::METAMOD3:
    {
      metaModWavelenMul3 = Q16_16::from_raw(msg.value.intValue);
      break;
    }
    case streamMessaging::messageTypes::METAMOD4:
    {
      metaModWavelenMul4 = Q16_16::from_raw(msg.value.intValue);
      break;
    }
    case streamMessaging::messageTypes::METAMOD5:
    {
      metaModWavelenMul5 = Q16_16::from_raw(msg.value.intValue);
      break;
    }
    case streamMessaging::messageTypes::METAMOD6:
    {
      uint32_t save1 = spin_lock_blocking(calcOscsSpinlock1);  
      metaModWavelenMul6 = Q16_16::from_raw(msg.value.intValue);
      spin_unlock(calcOscsSpinlock1, save1);
      break;
    }
    case streamMessaging::messageTypes::METAMOD7:
    {
      uint32_t save1 = spin_lock_blocking(calcOscsSpinlock1);  
      metaModWavelenMul7 = Q16_16::from_raw(msg.value.intValue);
      spin_unlock(calcOscsSpinlock1, save1);
      break;
    }
    case streamMessaging::messageTypes::METAMOD8:
    {
      uint32_t save1 = spin_lock_blocking(calcOscsSpinlock1);  
      metaModWavelenMul8 = Q16_16::from_raw(msg.value.intValue);
      spin_unlock(calcOscsSpinlock1, save1);
      break;
    }
    case streamMessaging::messageTypes::OCTSPREAD:
    {
      octaveIdx = msg.value.uintValue;
      if (octaveIdx > 15) octaveIdx = 15;
      currentOctaveShifts = (int8_t *)octaveTableShift[octaveIdx];
      break;
    }
    case streamMessaging::messageTypes::CTRL0:
    {
      epsilon_fixed = Q16_16::from_raw(msg.value.intValue);
      newCtrlReady = true;

      currOscModels0[0]->ctrl(epsilon_fixed);
      currOscModels0[1]->ctrl(epsilon_fixed);
      currOscModels0[2]->ctrl(epsilon_fixed);

      break;
    }
    case streamMessaging::messageTypes::BANK0:
    {
      size_t requestedBank = msg.value.uintValue;

      // Skip if already at this bank (prevents redundant changes)
      if (requestedBank == currentBank0Type) {
        break;
      }


      // uint32_t save = spin_lock_blocking(calcOscsSpinlock0);

      stopOscBankA();
      dma_hw->ints0 = smOsc0_dma_chan_bit | smOsc1_dma_chan_bit | smOsc2_dma_chan_bit;

      auto w1 = currOscModels0[0]->getWavelen();
      auto w2 = currOscModels0[1]->getWavelen();
      auto w3 = currOscModels0[2]->getWavelen();


      assignOscModels0(requestedBank);

      currOscModels0[0]->reset();
      currOscModels0[1]->reset();
      currOscModels0[2]->reset();

      currOscModels0[0]->setWavelen(w1);
      currOscModels0[1]->setWavelen(w2);
      currOscModels0[2]->setWavelen(w3);

      currOscModels0[0]->ctrl(epsilon_fixed);
      currOscModels0[1]->ctrl(epsilon_fixed);
      currOscModels0[2]->ctrl(epsilon_fixed);

      calculateOscBuffers0();

      startOscBankA();

      // Update current bank type
      currentBank0Type = requestedBank;

      Serial.println("Bank0 changed, RX restarted");
      // streamMessaging::resumeReceiver();

    }
    break;
    case streamMessaging::messageTypes::BANK1:
    {
      uint32_t save = spin_lock_blocking(calcOscsSpinlock1);
      requestedBank1 = msg.value.uintValue;
      spin_unlock(calcOscsSpinlock1, save);

    }
    break;
    default:
      break;
  }
}  



const size_t oscStartDelay = 200;

void setup() {
  //show on board LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);

  calcOscsSpinlock0 = spin_lock_init(spin_lock_claim_unused(true));

  // currOscModelBank0 = oscModel2;
  assignOscModels0(0);


  Serial.begin();
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB
  // }
  // pinMode(13, INPUT_PULLUP);

  // Serial1.setRX(13); //uart 1
  // Serial1.setTX(12);
  // Serial1.setFIFOSize(16); //reduce latency
  // Serial1.begin(SERIAL_CX_BAUD);

    // while(!Serial) {}
    pinMode(22,OUTPUT);
    streamMessaging::setupRX(pio0, 22, 12, DMACH_SERIAL_RX_A, DMACH_SERIAL_RX_B);



  // queue_init(&coreCommsQueue, sizeof(queueItem), 3);
#ifdef RUNCORE0_OSCS
  // delay(oscStartDelay);
  // while(!oscsReadyToStart) {
  //   ;
  // }
  // Serial.println("Start");
  // currOscModels0[0]->ctrl(ctrlVal);
  // currOscModels0[1]->ctrl(ctrlVal);
  // currOscModels0[2]->ctrl(ctrlVal);

  // startOscBankA();
  // Serial.println("Started");
#endif
}

size_t counter=0;
const size_t checkevery=10000;
size_t errorCount=0;
size_t totalMessagesReceived=0;

size_t serialts=0;

void __not_in_flash_func(loop)() {
  digitalWrite(22,0);

  streamMessaging::msgpacket* msg;
  streamMessaging::RxStatus status = streamMessaging::receiveWithDMA(&msg);

  if (status != streamMessaging::RxStatus::NO_MESSAGE) {
      if (status == streamMessaging::RxStatus::MESSAGE_OK) {
          totalMessagesReceived++;
          digitalWrite(22,1);
          processSerialMessage(*msg);
      } else {
          errorCount++;
      }
      if (counter++ == checkevery) {
          Serial.printf("%d messages received, %d errors, %d total\n", checkevery, errorCount,totalMessagesReceived);
          Serial.printf("new oct %d\n ", octaveIdx);

          counter=0;
          errorCount=0;
      }
  }
  if (waitingForFirstFrequency0 == false && oscsStartedAfterFirstFrequency0 == false) {
    // currOscModels0[0]->ctrl(ctrlVal);
    // currOscModels0[1]->ctrl(ctrlVal);
    // currOscModels0[2]->ctrl(ctrlVal);

    startOscBankA();
    oscsStartedAfterFirstFrequency0 = true;
  }
  if (oscsRunning0) {
    if (newFrequenciesReady0) {
      Fixed<20,12> new_wavelen3_fixed = (new_wavelen2_fixed - detuneFixed);
      Fixed<20,12> new_wavelen4_fixed = (new_wavelen3_fixed - detuneFixed);
      Fixed<20,12> new_wavelen5_fixed = (new_wavelen4_fixed - detuneFixed);

      new_wavelen3_fixed = new_wavelen3_fixed.mulWith(metaModWavelenMul3);
      new_wavelen4_fixed = new_wavelen4_fixed.mulWith(metaModWavelenMul4);
      new_wavelen5_fixed = new_wavelen5_fixed.mulWith(metaModWavelenMul5);


      new_wavelen3_fixed = currentOctaveShifts[0] > 0 ? new_wavelen3_fixed >> currentOctaveShifts[0] : new_wavelen3_fixed << -currentOctaveShifts[0];
      new_wavelen4_fixed = currentOctaveShifts[1] > 0 ? new_wavelen4_fixed >> currentOctaveShifts[1] : new_wavelen4_fixed << -currentOctaveShifts[1];
      new_wavelen5_fixed = currentOctaveShifts[2] > 0 ? new_wavelen5_fixed >> currentOctaveShifts[2] : new_wavelen5_fixed << -currentOctaveShifts[2];

      currOscModels0[0]->setWavelen(new_wavelen3_fixed.to_int());
      currOscModels0[1]->setWavelen(new_wavelen4_fixed.to_int());
      currOscModels0[2]->setWavelen(new_wavelen5_fixed.to_int());
      newFrequenciesReady0 = false;
    }
    
    // uint32_t save = spin_lock_blocking(calcOscsSpinlock0);  
    calculateOscBuffers0();
    // spin_unlock(calcOscsSpinlock0, save);
  }

  auto now = millis();
  if (now - serialts > 500) {
    Serial.print(".");
    serialts=now;
  }
  // delay(1);
  // __wfi();
}


void setup1() {
  calcOscsSpinlock1 = spin_lock_init(spin_lock_claim_unused(true));

//  currOscModelBank1 = oscModel2;
  assignOscModels1(0);


}

void __not_in_flash_func(loop1)() {
  if (waitingForFirstFrequency1 == false && oscsStartedAfterFirstFrequency1 == false) {
    startOscBankB();
    oscsStartedAfterFirstFrequency1 = true;
  }
  if (oscsRunning1) {
    uint32_t save = spin_lock_blocking(calcOscsSpinlock1);
    const bool newBank = currentBank1Type != requestedBank1;
    spin_unlock(calcOscsSpinlock1, save);

    if (newBank) {
      // Serial.printf("bank1: changing %d -> %d\n", currentBank1Type, requestedBank);

      stopOscBankB();

      // busy_wait_us(100);

      dma_hw->ints1 = smOsc3_dma_chan_bit | smOsc4_dma_chan_bit | smOsc5_dma_chan_bit;

      auto w1 = currOscModels1[0]->getWavelen();
      auto w2 = currOscModels1[1]->getWavelen();
      auto w3 = currOscModels1[2]->getWavelen();

      assignOscModels1(requestedBank1);

      currOscModels1[0]->reset();
      currOscModels1[1]->reset();
      currOscModels1[2]->reset();

      currOscModels1[0]->setWavelen(w1);
      currOscModels1[1]->setWavelen(w2);
      currOscModels1[2]->setWavelen(w3);

      currOscModels1[0]->ctrl(epsilon_fixed);
      currOscModels1[1]->ctrl(epsilon_fixed);
      currOscModels1[2]->ctrl(epsilon_fixed);

      calculateOscBuffers1();

      startOscBankB();

      // Update current bank type
      uint32_t save = spin_lock_blocking(calcOscsSpinlock1);
      currentBank1Type = requestedBank1;
      spin_unlock(calcOscsSpinlock1, save);

    }    
    if (newCtrlReady) {
      currOscModels1[0]->ctrl(epsilon_fixed);
      currOscModels1[1]->ctrl(epsilon_fixed);
      currOscModels1[2]->ctrl(epsilon_fixed);
      newCtrlReady = false;
    }
    if (newFrequenciesReady1) {
      Fixed<20,12> new_wavelen6_fixed = (new_wavelen2_fixed - detuneFixed - detuneFixed - detuneFixed - detuneFixed );
      Fixed<20,12> new_wavelen7_fixed = (new_wavelen6_fixed - detuneFixed);
      Fixed<20,12> new_wavelen8_fixed = (new_wavelen7_fixed - detuneFixed);

      new_wavelen6_fixed = new_wavelen6_fixed.mulWith(metaModWavelenMul6);
      new_wavelen7_fixed = new_wavelen7_fixed.mulWith(metaModWavelenMul7);
      new_wavelen8_fixed = new_wavelen8_fixed.mulWith(metaModWavelenMul8);


      new_wavelen6_fixed = currentOctaveShifts[0] > 0 ? new_wavelen6_fixed >> currentOctaveShifts[0] : new_wavelen6_fixed << -currentOctaveShifts[0];
      new_wavelen7_fixed = currentOctaveShifts[1] > 0 ? new_wavelen7_fixed >> currentOctaveShifts[1] : new_wavelen7_fixed << -currentOctaveShifts[1];
      new_wavelen8_fixed = currentOctaveShifts[2] > 0 ? new_wavelen8_fixed >> currentOctaveShifts[2] : new_wavelen8_fixed << -currentOctaveShifts[2];

      currOscModels1[0]->setWavelen(new_wavelen6_fixed.to_int());
      currOscModels1[1]->setWavelen(new_wavelen7_fixed.to_int());
      currOscModels1[2]->setWavelen(new_wavelen8_fixed.to_int());
      newFrequenciesReady1 = false;
    }
    // uint32_t save = spin_lock_blocking(calcOscsSpinlock1);  
    calculateOscBuffers1();
    // spin_unlock(calcOscsSpinlock1, save);

  }
}