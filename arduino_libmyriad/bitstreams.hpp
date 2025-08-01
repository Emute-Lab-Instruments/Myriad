#pragma once

#include "clockfreq.h"

#define FAST_MEM __not_in_flash("mydata")

#define OSC_BUFFER_SIZE 512

#define DEFINE_TIMING_SWAPBUFFERS(name) \
uint32_t FAST_MEM timing_swapbuffer_##name##_A[OSC_BUFFER_SIZE] __attribute__((aligned(16))) {0}; \
uint32_t FAST_MEM timing_swapbuffer_##name##_B[OSC_BUFFER_SIZE] __attribute__((aligned(16))) {0}; \
io_rw_32 FAST_MEM nextTimingBuffer##name = (io_rw_32)timing_swapbuffer_##name##_A;

inline void __not_in_flash_func(updateTimingBuffer)(io_rw_32 &nextBuf,
                                  uint32_t* bufferA, uint32_t* bufferB,
                                  oscModelPtr& oscModel
                                  ) {
    if (nextBuf == reinterpret_cast<io_rw_32>(bufferA)) {
        oscModel->fillBuffer(bufferB);
        nextBuf = reinterpret_cast<io_rw_32>(bufferB);
    } else {
        oscModel->fillBuffer(bufferA);
        nextBuf = reinterpret_cast<io_rw_32>(bufferA);
    }
}

