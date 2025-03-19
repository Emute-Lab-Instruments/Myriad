#pragma once

enum messageTypes {WAVELEN0=0, WAVELEN1, WAVELEN2, WAVELEN3, WAVELEN4, WAVELEN5, BANK0, BANK1, CTRL,
                CTRL0, CTRL1, CTRL2, CTRL3, CTRL4, CTRL5};


// enum oscBankTypes {SAW=0, SQR};
// size_t maxOscBankType = 1;

struct spiMessage {
    uint8_t msg;
    float value;
};