#pragma once
#include <vector>

struct oscDisplayModes {
    enum MODES {SPECTRAL,SILENCE,NOISE} mode;
    std::vector<size_t> data;
};