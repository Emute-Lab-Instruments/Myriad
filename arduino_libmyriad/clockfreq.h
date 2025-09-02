#pragma once
#include <stddef.h>
#include "hardware/clocks.h"

constexpr float CLOCKDIV_BASE = 96.f; 
constexpr float clockdiv = CLOCKDIV_BASE;
constexpr size_t cpuClock=200000000;
constexpr float sampleClock = cpuClock / clockdiv;
constexpr float sampleClockInv = 1.f/sampleClock;
