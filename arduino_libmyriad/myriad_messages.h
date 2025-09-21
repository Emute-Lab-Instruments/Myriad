#pragma once

enum messageTypes {WAVELEN0=0, WAVELEN1, WAVELEN2, WAVELEN3, WAVELEN4, WAVELEN5, BANK0, BANK1, CTRL,
                CTRL0, CTRL1, CTRL2, CTRL3, CTRL4, CTRL5, DETUNE, OCTSPREAD,
                METAMOD3, METAMOD4, METAMOD5, METAMOD6, METAMOD7, METAMOD8};


// enum oscBankTypes {SAW=0, SQR};
// size_t maxOscBankType = 1;

struct spiMessage {
    uint8_t msg;
    union {
      size_t ivalue;
      float value;
    };
};