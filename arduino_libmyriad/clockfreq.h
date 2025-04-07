#pragma once
#include <stddef.h>
#include "hardware/clocks.h"

constexpr float clockdiv = 32;
constexpr size_t cpuClock=200000000;
constexpr float sampleClock = cpuClock / clockdiv;
