#pragma once
#include <stddef.h>
#include "hardware/clocks.h"

constexpr float CLOCKDIV_BASE = 96.f; 
constexpr float clockdiv = CLOCKDIV_BASE;
constexpr size_t cpuClock=250000000;
constexpr float sampleClock = cpuClock / clockdiv;
constexpr float sampleClockInv = 1.f/sampleClock;
constexpr float minWavelen = sampleClock / 20000.f; //20kHz max
constexpr float maxWavelen = sampleClock / 20.f; //1Hz min
