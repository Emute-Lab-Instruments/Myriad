#pragma once
#include <stddef.h>
#include "hardware/clocks.h"
#include "fixedpoint.hpp"

using namespace FixedPoint;

constexpr size_t CLOCKDIV_BASE = 96; 
constexpr size_t clockdiv = CLOCKDIV_BASE;
constexpr size_t cpuClock=250000000;
constexpr float sampleClock = static_cast<float>(cpuClock) / clockdiv;
constexpr float sampleClockInv = 1.f/sampleClock;
constexpr float minWavelen = sampleClock / 20000.f; //20kHz max
constexpr float maxWavelen = sampleClock / 20.f; //1Hz min


constexpr Fixed<24,8> sampleClockFP = Fixed<24,8>(sampleClock);
constexpr Q16_16 sampleClockInvFP(sampleClockInv);
constexpr Fixed<20,12> minWavelenFP(minWavelen); //20kHz max
constexpr Fixed<20,12> maxWavelenFP(maxWavelen);
