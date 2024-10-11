#pragma once

enum messageTypes {WAVELEN3=0, WAVELEN4, WAVELEN5, WAVELEN6, WAVELEN7, WAVELEN8};

struct spiMessage {
    uint8_t msg;
    size_t value;
};