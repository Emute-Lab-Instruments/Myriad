#ifndef _DRAWING_H
#define _DRAWING_H

#define GFX_MEM __not_in_flash("gfx")

#include "fixedpoint.hpp"
using namespace FixedPoint;

constexpr uint16_t rgbTo565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#define ELI_BLUE 0x0007
#define ELI_PINK 0xFC9F
static uint16_t ELI_PINK2 = rgbTo565(216, 180, 250);
static uint16_t ELI_PINK3 = rgbTo565(167, 122, 222);
static uint16_t GREYED_OUT_COL = rgbTo565(64,64,64);


static size_t sqhalfwidth=80;
static size_t sqwidth = sqhalfwidth+sqhalfwidth; //dim of the largest square that fits in the circle
static size_t sqbound = (240 - sqwidth) / 2; //left/top of the largest square that fits in the circle
static size_t sqboundBR = sqbound + sqwidth; //right/bottom X of the largest square that fits in the circle


static Q16_16 sqhalfwidthFP=Q16_16(80);
static Q16_16 sqwidthFP = sqhalfwidthFP+sqhalfwidthFP; //dim of the largest square that fits in the circle
static Q16_16 sqboundFP = (Q16_16(240) - sqwidthFP) / Q16_16(2); //left/top of the largest square that fits in the circle
static Q16_16 sqboundBRFP = sqboundFP + sqwidthFP; //right/bottom X of the largest square that fits in the circle

// 64-color gradient array from dark green to light pink
// Using 16-bit RGB565 format for TFT_eSPI
// Format: RGB565 = RRRRR GGGGGG BBBBB

static uint16_t GFX_MEM gradient_colors[64] = {
    0x0200,  // Dark green
    0x0220,
    0x0240,
    0x0260,
    0x0280,
    0x02A0,
    0x02C0,
    0x02E0,
    0x0300,
    0x0320,
    0x0340,
    0x0360,
    0x0380,
    0x03A0,
    0x03C0,
    0x03E0,
    0x0400,
    0x0420,
    0x0440,
    0x0460,
    0x0480,
    0x04A0,
    0x04C0,
    0x04E0,
    0x0500,
    0x0520,
    0x0540,
    0x0560,
    0x0580,
    0x05A0,
    0x05C0,
    0x05E0,
    0x0600,  // Mid green
    0x2600,
    0x4600,
    0x6600,
    0x8600,
    0xA600,
    0xC600,
    0xE600,
    0xE620,
    0xE640,
    0xE660,
    0xE680,
    0xE6A0,
    0xE6C0,
    0xE6E0,
    0xE700,
    0xE720,
    0xE740,
    0xE760,
    0xE780,
    0xE7A0,
    0xE7C0,
    0xE7E0,
    0xE800,
    0xE820,
    0xE840,
    0xE860,
    0xE880,
    0xE8A0,
    0xE8C0,
    0xE8E0,
    0xF8E0   // Light pink
};



#endif