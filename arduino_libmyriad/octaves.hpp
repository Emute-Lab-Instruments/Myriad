#ifndef OCTAVES_HPP
#define OCTAVES_HPP

// static const __not_in_flash("octaves") float octaveTable[16][9] = {
//     {1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f},   // case 0
//     {0.5f,   1.0f,   1.0f,   0.5f,   1.0f,   1.0f,   0.5f,   1.0f,   1.0f},   // case 1
//     {0.5f,   0.5f,   1.0f,   0.5f,   0.5f,   1.0f,   0.5f,   0.5f,   1.0f},   // case 2
//     {0.25f,  0.5f,   1.0f,   0.25f,  0.5f,   1.0f,   0.25f,  0.5f,   1.0f},   // case 3
//     {0.25f,  0.5f,   2.0f,   0.25f,  0.5f,   2.0f,   0.25f,  0.5f,   2.0f},   // case 4
//     {0.25f,  0.5f,   4.0f,   0.25f,  0.5f,   4.0f,   0.25f,  0.5f,   4.0f},   // case 5
//     {0.25f,  0.25f,  4.0f,   0.25f,  0.25f,  4.0f,   0.25f,  0.25f,  4.0f},   // case 6
//     {0.5f,   0.25f,  4.0f,   0.5f,   0.25f,  4.0f,   0.5f,   0.25f,  4.0f},   // case 7
//     {0.25f,  2.0f,   4.0f,   0.25f,  2.0f,   4.0f,   0.25f,  2.0f,   4.0f},   // case 8
//     {0.25f,  4.0f,   4.0f,   0.25f,  4.0f,   4.0f,   0.25f,  4.0f,   4.0f},   // case 9
//     {0.5f,   4.0f,   4.0f,   0.5f,   4.0f,   4.0f,   0.25f,  4.0f,   4.0f},   // case 10
//     {1.0f,   4.0f,   4.0f,   1.0f,   4.0f,   4.0f,   1.0f,   4.0f,   4.0f},   // case 11
//     {2.0f,   4.0f,   4.0f,   2.0f,   4.0f,   4.0f,   2.0f,   4.0f,   4.0f},   // case 12
//     {2.0f,   2.0f,   4.0f,   2.0f,   2.0f,   4.0f,   2.0f,   2.0f,   4.0f},   // case 13
//     {1.0f,   2.0f,   4.0f,   1.0f,   2.0f,   4.0f,   1.0f,   2.0f,   4.0f},   // case 14
//     {1.0f,   2.0f,   2.0f,   1.0f,   2.0f,   2.0f,   1.0f,   2.0f,   2.0f}    // case 15
// };

static const __not_in_flash("octaves") int8_t octaveTableShift[17][3] = {
    {1, 1, 2},   // case 0
    {0, 1, 2},   // case 1
    {0, 1, 1},   // case 2
    {0, 0, 1},   //case 3
    {-1, -1, -2},   //case 4
    {0, -1, -2},   //case 5
    {0, -1, -1},   //case 6
    {0,0,-1},   //case 7
    {0, 0, 0},   //case 8
    {-1, 0, 1},   //case 9
    {-1, 0, 2},   //case 10
    {-2, 0, 1},   //case 11
    {-2, 0, 2},   //case 12
    {-1, 1, 2},   //case 13
    {-2, 1, 2},   //case 14
    {-2, -1, 1},   //case 15
    {-2, -1, 2}    //case 16
};
// static const __not_in_flash("octaves") int8_t octaveTableShift[16][3] = {
//     {0, 0, 0},   // case 0
//     {1, 0, 0},   // case 1
//     {1, 1, 0},   // case 2
//     {2, 1, 0},   //case 3
//     {2, 1, -1},   //case 4
//     {2, 1, -2},   //case 5
//     {2, 2, -2},   //case 6
//     {1, 2, -2},   //case 7
//     {2, -1, -2},   //case 8
//     {2, -2, -2},   //case 9
//     {1, -2, -2},   //case 10
//     {0, -2, -2},   //case 11
//     {-1, -2, -2},   //case 12
//     {-1, -1, -2},   //case 13
//     {0, -1, -2},   //case 14
//     {0, -1, -1}    //case 15
// };

// static float __not_in_flash("adc") *currentOctaves = (float *)octaveTable[0];
static int8_t __not_in_flash("adc") *currentOctaveShifts = (int8_t *)octaveTableShift[0];




#endif // OCTAVES_HPP