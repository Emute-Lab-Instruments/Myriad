#ifndef _DRAWING_H
#define _DRAWING_H

#define ELI_BLUE 0x0007
#define ELI_PINK 0xFC9F

const size_t sqhalfwidth=84;
const size_t sqwidth = sqhalfwidth+sqhalfwidth; //dim of the largest square that fits in the circle
const size_t sqbound = (240 - sqwidth) / 2; //left/top of the largest square that fits in the circle

#endif