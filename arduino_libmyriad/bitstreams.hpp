#pragma once

#include "clockfreq.h"

#define FAST_MEM __not_in_flash("mydata")


uint32_t mwavelen = cpuClock/clockdiv /80;
#define DEFINE_TIMING_SWAPBUFFERS(name) \
uint32_t FAST_MEM timing_swapbuffer_##name##_A[512] __attribute__((aligned(16))) {mwavelen,mwavelen}; \
uint32_t FAST_MEM timing_swapbuffer_##name##_B[512] __attribute__((aligned(16))) {mwavelen,mwavelen}; \
io_rw_32 FAST_MEM nextTimingBuffer##name = (io_rw_32)timing_swapbuffer_##name##_A;

inline void __not_in_flash_func(updateTimingBuffer)(io_rw_32 &nextBuf,
                                  uint32_t* bufferA, uint32_t* bufferB,
                                  oscModelPtr& oscModel,
                                  float oscWavelength) {
    if (nextBuf == reinterpret_cast<io_rw_32>(bufferA)) {
        oscModel->fillBuffer(bufferB, oscWavelength);
        nextBuf = reinterpret_cast<io_rw_32>(bufferB);
    } else {
        oscModel->fillBuffer(bufferA, oscWavelength);
        nextBuf = reinterpret_cast<io_rw_32>(bufferA);
    }
}

