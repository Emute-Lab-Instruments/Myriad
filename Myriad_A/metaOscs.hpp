#pragma once

#include <stdlib.h>
#include <array>
#include "Arduino.h"

constexpr float TWOPI  = PI * 2;

class metaOsc {
public:
    metaOsc() {};
    ~metaOsc() {};
    float depth=0;
    float speed=0;
    virtual void setDepth(int newDepth) {
        depth=newDepth;
    }
    virtual void setSpeed(int newSpeed) {
        speed = newSpeed;
    }
};

template<size_t N>
class metaOscSines : public metaOsc {
public:
    metaOscSines() {
        //init with equally spread phase
        const float phaseGap = TWOPI/N;
        for(size_t i=0; i < N; i++) {
            phasors[i] = i*phaseGap;
        }
    }

    std::array<float, N> update() {
        for(size_t i=0; i < N; i++) {
            sines[i] = sinf(phasors[i]) * depth;
            phasors[i] += speed;
        }
        return sines;
    }

    void setDepth(int newDepth) {
        depth= newDepth * 0.01;
    }

    void setSpeed(int newSpeed) {
        speed = newSpeed * 0.005;
        
    }



private:
    std::array<float, N> phasors;
    std::array<float, N> sines;
};