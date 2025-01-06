#pragma once

constexpr float clockdiv = 8;
// uint32_t clockHz = 15625000 / 80;
constexpr size_t cpuClock=133000000;

constexpr float sampleClock = cpuClock / clockdiv;
