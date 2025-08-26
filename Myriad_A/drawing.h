#ifndef _DRAWING_H
#define _DRAWING_H

constexpr uint16_t rgbTo565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#define ELI_BLUE 0x0007
#define ELI_PINK 0xFC9F
constexpr uint16_t ELI_PINK2 = rgbTo565(216, 180, 250);
constexpr uint16_t ELI_PINK3 = rgbTo565(167, 122, 222);


constexpr size_t sqhalfwidth=84;
constexpr size_t sqwidth = sqhalfwidth+sqhalfwidth; //dim of the largest square that fits in the circle
constexpr size_t sqbound = (240 - sqwidth) / 2; //left/top of the largest square that fits in the circle
constexpr size_t sqboundBR = sqbound + sqwidth; //right/bottom X of the largest square that fits in the circle

#endif