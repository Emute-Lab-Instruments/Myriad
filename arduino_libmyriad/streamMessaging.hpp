#ifndef __STREAMMESSAGING_H__
#define __STREAMMESSAGING_H__

#include <cstring>
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "stream_tx.pio.h"
#include "stream_rx.pio.h"


namespace streamMessaging {

  constexpr uint8_t MAGIC_BYTE = 0b10101010;

  enum messageTypes {WAVELEN0, BANK0, BANK1, CTRL,
                  CTRL0, CTRL1, CTRL2, CTRL3, CTRL4, CTRL5, DETUNE, OCTSPREAD,
                  METAMOD3, METAMOD4, METAMOD5, METAMOD6, METAMOD7, METAMOD8};

  // constexpr size_t RX_DATA_PIN = 12;
  // constexpr size_t RX_FRAME_PIN = 13;

  constexpr float BIT_RATE = 20000000.0f;

  #define DMA_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
  #define PIO_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY

  PIO pioTx;
  uint smTx;
  uint offsetTx;
  static __scratch_x("msg") uint dma_channel_tx;


  dma_channel_config config_tx;


  // void setupTX() {
  //   bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&stream_tx_program, &pioTx, &smTx, &offsetTx, TX_DATA_PIN, 2, true);
  //   stream_tx_program_init(pioTx, smTx, offsetTx, TX_DATA_PIN, TX_FRAME_PIN, BIT_RATE);
  //   Serial.printf("Tx prog: %d\n", success);

  //   irq_add_shared_handler(dma_get_irq_num(0), dma_irq_handler_tx, DMA_IRQ_PRIORITY);
  //   irq_set_enabled(dma_get_irq_num(0), true);

  //   // setup dma for write
  //   dma_channel_tx = dma_claim_unused_channel(false);
  //   if (dma_channel_tx < 0) {
  //       panic("No free dma channels");
  //   }
  //   config_tx = dma_channel_get_default_config(dma_channel_tx);
  //   channel_config_set_transfer_data_size(&config_tx, DMA_SIZE_32);
  //   channel_config_set_read_increment(&config_tx, true);
  //   channel_config_set_write_increment(&config_tx, false);
  //   // enable irq for tx
  //   dma_irqn_set_channel_enabled(0, dma_channel_tx, true);        

  //   //connect DMA with PIO Rx
  //   channel_config_set_dreq(&config_tx, pio_get_dreq(pioTx, smTx, true));

  // }

  bool setupTX(PIO pioNum, irq_handler_t dma_handler, int TX_DATA_PIN, int TX_FRAME_PIN, uint dmaChannel) {

    // bool success =
    // pio_claim_free_sm_and_add_program_for_gpio_range(&stream_tx_program,
    // &pioTx, &smTx, &offsetTx, TX_DATA_PIN, 2, true);

    pioTx = pioNum; 

    // Check if program fits and add it
    if (!pio_can_add_program(pioTx, &stream_tx_program)) {
      Serial.printf("Program doesn't fit in PIO%d\n", pio_get_index(pioTx));
      return false;
    }

    offsetTx = pio_add_program(pioTx, &stream_tx_program);

    // Claim an unused state machine
    int sm = pio_claim_unused_sm(pioTx, false); // false = don't require it
    if (sm < 0) {
      Serial.printf("No free state machine on PIO%d\n", pio_get_index(pioTx));
      return false;
    }
    smTx = sm;

    stream_tx_program_init(pioTx, smTx, offsetTx, TX_DATA_PIN, TX_FRAME_PIN, BIT_RATE); 
    Serial.printf("Tx prog: %d\n", sm);
    
    if (dma_handler != nullptr) {
      irq_add_shared_handler(dma_get_irq_num(0), dma_handler,
                            DMA_IRQ_PRIORITY);
      irq_set_enabled(dma_get_irq_num(0), true);
    }
    // setup dma for write
    dma_channel_tx = dmaChannel; // dma_claim_unused_channel(false);
    // if (dma_channel_tx < 0) {
    //   panic("No free dma channels");
    // }
    config_tx = dma_channel_get_default_config(dma_channel_tx);
    channel_config_set_transfer_data_size(&config_tx, DMA_SIZE_32);
    channel_config_set_read_increment(&config_tx, true);
    channel_config_set_write_increment(&config_tx, false);
    // enable irq for tx
    dma_irqn_set_channel_enabled(0, dma_channel_tx, true);

    // connect DMA with PIO Rx
    channel_config_set_dreq(&config_tx, pio_get_dreq(pioTx, smTx, true));
    //
    return true;
  }

  struct __attribute__((packed)) msgpacket {
      union {
        float floatValue;
        size_t uintValue;
        int32_t intValue;
      } value;
      uint8_t msgType;
      const uint8_t magicByte = MAGIC_BYTE;
      uint16_t checksum;
  };

  static_assert(sizeof(msgpacket) == 8, "msgpacket must be 8 bytes");

  __always_inline void calcCheckSum(msgpacket &msg) {
    uint32_t v = msg.value.uintValue;
    msg.checksum = (uint16_t)(v ^ (v >> 16) ^ msg.msgType);  
  }
  __always_inline void createMessage(msgpacket &msg, const float value, const messageTypes msgType) {
    msg.value.floatValue = value;
    msg.msgType = msgType;
    calcCheckSum(msg);
  };

  __always_inline void createMessage(msgpacket &msg, const size_t value, const messageTypes msgType) {
    msg.value.uintValue = value;
    msg.msgType = msgType;
    calcCheckSum(msg);
  };

  __always_inline void createMessage(msgpacket &msg, const int32_t value, const messageTypes msgType) {
    msg.value.intValue = value;
    msg.msgType = msgType;
    calcCheckSum(msg);
  };
  __always_inline void sendMessageWithDMA(msgpacket &msg) {
      while (dma_channel_is_busy(dma_channel_tx)) {
        tight_loop_contents();
      }
      dma_channel_configure(dma_channel_tx, &config_tx, &pioTx->txf[smTx], &msg, 2, true); // dma started    
  }


  __always_inline bool checksumIsOk(msgpacket *msg) {
    uint32_t v = msg->value.uintValue;
    return msg->checksum == (uint16_t)(v ^ (v >> 16) ^ msg->msgType);  
     
  }

  __always_inline bool magicByteOk(msgpacket *msg) {
    return msg->magicByte == MAGIC_BYTE;
  }


//RX
  PIO pioRx;
  uint smRx;
  uint offsetRx;
  #define RX_BUFFER_SIZE 256
  #define RX_BUFFER_SIZE_WORDS RX_BUFFER_SIZE / 4
  uint8_t rx_buffer_a[RX_BUFFER_SIZE] __attribute__((aligned(RX_BUFFER_SIZE)));
  uint8_t rx_buffer_b[RX_BUFFER_SIZE] __attribute__((aligned(RX_BUFFER_SIZE)));

  size_t* rx_buffer_a_word = (size_t*)rx_buffer_a;
  size_t* rx_buffer_b_word = (size_t*)rx_buffer_b;

  volatile uint32_t last_dma_pos = 0;

  size_t * curr_rx_buffer;

  static uint dma_channel_rx_a, dma_channel_rx_b, current_rx_dma;
  dma_channel_config config_rx_a,  config_rx_b;
  volatile bool receiver_paused = false;

  //also need RX_DATA_PIN+1 available for the packet frame signal
  bool setupRX(PIO pioNum, uint offset, int RX_DATA_PIN, uint dmaChA, uint dmaChB) {
    // bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&stream_rx_program, &pioRx, &smRx, &offsetRx, RX_DATA_PIN, 2, true);
    
    pioRx = pioNum; 

    // Check if program fits and add it
    if (!pio_can_add_program_at_offset(pioRx, &stream_rx_program, offset)) {
      Serial.printf("Program doesn't fit in PIO%d\n", pio_get_index(pioRx));
      return false;
    }

    offsetRx = offset;
    pio_add_program_at_offset(pioRx, &stream_rx_program, offsetRx);

    // Claim an unused state machine
    // int sm = pio_claim_unused_sm(pioRx, false); // false = don't require it
    // if (sm < 0) {
    //   Serial.printf("No free state machine on PIO%d\n", pio_get_index(pioRx));
    //   return false;
    // }
    smRx = 3;

    // Check if available
    if (pio_sm_is_claimed(pioRx, smRx)) {
        Serial.printf("State machine %d already claimed!", smRx);
        return false;
    }

    // Claim it
    pio_sm_claim(pioRx, smRx);



	  stream_rx_program_init(pioRx, smRx, offsetRx, RX_DATA_PIN, streamMessaging::BIT_RATE);
    Serial.printf("Rx prog: %d %d %d\n", pioRx==pio0 ? 0 : 1, smRx, offsetRx);

    dma_channel_rx_a = dmaChA; //dma_claim_unused_channel(false);
    // if (dma_channel_rx_a < 0) {
    //     panic("No free dma channels");
    // }
    dma_channel_rx_b = dmaChB; // dma_claim_unused_channel(false);
    // if (dma_channel_rx_b < 0) {
    //     panic("No free dma channels");
    // }

    config_rx_a = dma_channel_get_default_config(dma_channel_rx_a);
    channel_config_set_transfer_data_size(&config_rx_a, DMA_SIZE_32);
    channel_config_set_read_increment(&config_rx_a, false);
    channel_config_set_write_increment(&config_rx_a, true);
    channel_config_set_ring(&config_rx_a, true, __builtin_ctz(RX_BUFFER_SIZE));    

    channel_config_set_dreq(&config_rx_a, pio_get_dreq(pioRx, smRx, false));

    channel_config_set_chain_to(&config_rx_a, dma_channel_rx_b); 
    dma_channel_configure(dma_channel_rx_a, &config_rx_a, 
                         rx_buffer_a, 
                         &pioRx->rxf[smRx], 
                         RX_BUFFER_SIZE_WORDS, false);    


    //channel B
    config_rx_b = dma_channel_get_default_config(dma_channel_rx_b);
    channel_config_set_transfer_data_size(&config_rx_b, DMA_SIZE_32);
    channel_config_set_read_increment(&config_rx_b, false);
    channel_config_set_write_increment(&config_rx_b, true);
    channel_config_set_ring(&config_rx_b, true, __builtin_ctz(RX_BUFFER_SIZE));    

    channel_config_set_dreq(&config_rx_b, pio_get_dreq(pioRx, smRx, false));

    channel_config_set_chain_to(&config_rx_b, dma_channel_rx_a); 
    dma_channel_configure(dma_channel_rx_b, &config_rx_b, 
                         rx_buffer_b, 
                         &pioRx->rxf[smRx], 
                         RX_BUFFER_SIZE_WORDS, false);    
                         
    dma_channel_set_config(dma_channel_rx_a, &config_rx_a, false);
    dma_channel_set_config(dma_channel_rx_b, &config_rx_b, false);

    current_rx_dma = dma_channel_rx_a;  
    curr_rx_buffer = rx_buffer_a_word;

    dma_channel_start(dma_channel_rx_a);
    return true;
  }


  enum class RxStatus : uint8_t {
      NO_MESSAGE = 0,
      MESSAGE_OK = 1,
      CHECKSUM_ERROR = 2,
      MAGIC_ERROR = 4,
  };  

  // Enable bitwise operations
  inline RxStatus operator|(RxStatus a, RxStatus b) {
      return static_cast<RxStatus>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
  }

  inline RxStatus operator&(RxStatus a, RxStatus b) {
      return static_cast<RxStatus>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
  }

  inline RxStatus& operator|=(RxStatus& a, RxStatus b) {
      return a = a | b;
  }
  inline RxStatus receiveWithDMA(streamMessaging::msgpacket** msg_out) {
      // Determine which DMA channel is currently active (being written to)
      bool channel_a_busy = dma_channel_is_busy(dma_channel_rx_a);
      bool channel_b_busy = dma_channel_is_busy(dma_channel_rx_b);

      uint active_dma_channel;
      size_t* active_buffer;

      // Determine active channel and read from the INACTIVE buffer
      if (channel_a_busy && !channel_b_busy) {
          // Channel A is writing, read from buffer B
          active_dma_channel = dma_channel_rx_a;
          active_buffer = rx_buffer_a_word;
      } else if (channel_b_busy && !channel_a_busy) {
          // Channel B is writing, read from buffer A
          active_dma_channel = dma_channel_rx_b;
          active_buffer = rx_buffer_b_word;
      } else {
          // Both busy or both idle - use current tracking
          active_dma_channel = current_rx_dma;
          active_buffer = curr_rx_buffer;
      }

      // Check if we need to switch to the other buffer
      if (active_dma_channel != current_rx_dma) {
          // DMA switched buffers - switch our read buffer to the old one
          current_rx_dma = active_dma_channel;
          // Read from the NON-active buffer
          curr_rx_buffer = (active_dma_channel == dma_channel_rx_a) ? rx_buffer_b_word : rx_buffer_a_word;
          last_dma_pos = 0;
      }

      uint32_t remaining = dma_channel_hw_addr(current_rx_dma)->transfer_count;
      uint32_t current_dma_pos = RX_BUFFER_SIZE_WORDS - remaining;

      // Check if we've read all messages in current buffer
      if (last_dma_pos >= RX_BUFFER_SIZE_WORDS) {
          // Wrapped past end of buffer - try to switch
          last_dma_pos = 0;
          // Force re-check on next call
          return RxStatus::NO_MESSAGE;
      }

      // Fast path: no message available
      if (current_dma_pos - last_dma_pos < 2) {
          return RxStatus::NO_MESSAGE;
      }

      // Point directly to message in DMA buffer (zero-copy)
      *msg_out = reinterpret_cast<streamMessaging::msgpacket*>(&curr_rx_buffer[last_dma_pos]);

      // Validate and combine errors into single status byte
      uint8_t status = static_cast<uint8_t>(RxStatus::MESSAGE_OK);

      if (!streamMessaging::checksumIsOk(*msg_out)) {
          status |= static_cast<uint8_t>(RxStatus::CHECKSUM_ERROR);
      }
      if (!streamMessaging::magicByteOk(*msg_out)) {
          status |= static_cast<uint8_t>(RxStatus::MAGIC_ERROR);
      }

      // Advance position
      last_dma_pos += 2;

      return static_cast<RxStatus>(status);
  }

  // Pause the receiver - stops DMA and PIO but preserves buffer state
  void pauseReceiver() {
      if (receiver_paused) return; // Already paused

      // Abort both DMA channels to stop new data transfer
      dma_channel_abort(dma_channel_rx_a);
      dma_channel_abort(dma_channel_rx_b);

      // Disable the PIO state machine to stop hardware reception
      pio_sm_set_enabled(pioRx, smRx, false);

      receiver_paused = true;
  }

  // Resume the receiver - restarts DMA and PIO from current position
  void resumeReceiver() {
      if (!receiver_paused) return; // Not paused

      // Re-enable the PIO state machine
      pio_sm_set_enabled(pioRx, smRx, true);

      // Restart the current DMA channel
      dma_channel_start(current_rx_dma);

      receiver_paused = false;
  }

  // Clear both receive buffers - zeros out all data
  void clearBuffers() {
      memset(rx_buffer_a, 0, RX_BUFFER_SIZE);
      memset(rx_buffer_b, 0, RX_BUFFER_SIZE);
  }

  // Complete reset of receiver - stops everything, clears buffers, and restarts from initial state
  void resetReceiver() {
      // Stop DMA and PIO
      dma_channel_abort(dma_channel_rx_a);
      dma_channel_abort(dma_channel_rx_b);
      pio_sm_set_enabled(pioRx, smRx, false);

      // Drain PIO RX FIFO to clear any stale data
      pio_sm_clear_fifos(pioRx, smRx);

      // Clear both buffers
      clearBuffers();

      // Reset position tracking to initial state
      last_dma_pos = 0;
      current_rx_dma = dma_channel_rx_a;
      curr_rx_buffer = rx_buffer_a_word;

      // Reconfigure both DMA channels
      dma_channel_configure(dma_channel_rx_a, &config_rx_a,
                           rx_buffer_a,
                           &pioRx->rxf[smRx],
                           RX_BUFFER_SIZE_WORDS, false);

      dma_channel_configure(dma_channel_rx_b, &config_rx_b,
                           rx_buffer_b,
                           &pioRx->rxf[smRx],
                           RX_BUFFER_SIZE_WORDS, false);

      // Re-enable PIO state machine
      pio_sm_set_enabled(pioRx, smRx, true);

      // Start DMA channel A
      dma_channel_start(dma_channel_rx_a);

      receiver_paused = false;
  }

  // Query receiver status
  inline bool isReceiverPaused() {
      return receiver_paused;
  }
};



#endif