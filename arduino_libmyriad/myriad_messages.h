#pragma once

enum messageTypes {WAVELEN0=0, WAVELEN1, WAVELEN2, WAVELEN3, WAVELEN4, WAVELEN5};

struct spiMessage {
    uint8_t msg;
    size_t value;
};