#ifndef OCTAVES_HPP
#define OCTAVES_HPP

static const __scratch_x("octaves") float octaveTable[16][9] = {
    {1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f},   // case 0
    {0.5f,   1.0f,   1.0f,   0.5f,   1.0f,   1.0f,   0.5f,   1.0f,   1.0f},   // case 1
    {0.5f,   0.5f,   1.0f,   0.5f,   0.5f,   1.0f,   0.5f,   0.5f,   1.0f},   // case 2
    {0.25f,  0.5f,   1.0f,   0.25f,  0.5f,   1.0f,   0.25f,  0.5f,   1.0f},   // case 3
    {0.25f,  0.5f,   2.0f,   0.25f,  0.5f,   2.0f,   0.25f,  0.5f,   2.0f},   // case 4
    {0.25f,  0.5f,   4.0f,   0.25f,  0.5f,   4.0f,   0.25f,  0.5f,   4.0f},   // case 5
    {0.25f,  0.25f,  4.0f,   0.25f,  0.25f,  4.0f,   0.25f,  0.25f,  4.0f},   // case 6
    {0.5f,   0.25f,  4.0f,   0.5f,   0.25f,  4.0f,   0.5f,   0.25f,  4.0f},   // case 7
    {0.25f,  2.0f,   4.0f,   0.25f,  2.0f,   4.0f,   0.25f,  2.0f,   4.0f},   // case 8
    {0.25f,  4.0f,   4.0f,   0.25f,  4.0f,   4.0f,   0.25f,  4.0f,   4.0f},   // case 9
    {0.5f,   4.0f,   4.0f,   0.5f,   4.0f,   4.0f,   0.25f,  4.0f,   4.0f},   // case 10
    {1.0f,   4.0f,   4.0f,   1.0f,   4.0f,   4.0f,   1.0f,   4.0f,   4.0f},   // case 11
    {2.0f,   4.0f,   4.0f,   2.0f,   4.0f,   4.0f,   2.0f,   4.0f,   4.0f},   // case 12
    {2.0f,   2.0f,   4.0f,   2.0f,   2.0f,   4.0f,   2.0f,   2.0f,   4.0f},   // case 13
    {1.0f,   2.0f,   4.0f,   1.0f,   2.0f,   4.0f,   1.0f,   2.0f,   4.0f},   // case 14
    {1.0f,   2.0f,   2.0f,   1.0f,   2.0f,   2.0f,   1.0f,   2.0f,   2.0f}    // case 15
};

static float __scratch_x("adc") *currentOctaves = (float *)octaveTable[0];

//   if (octaveIdx != lastOctaveIdx) {
//     lastOctaveIdx = octaveIdx;
//     switch(octaveIdx) {
//       case 0:   {octaves = {1.0f, 1.0f,  1.0f,    1.0f,  1.0f,  1.0f,    1.0f,  1.0f,  1.0f}; break;}
//       case 1:   {octaves = {0.5f, 1.0f,  1.0f,    0.5f,  1.0f,  1.0f,    0.5f,  1.0f,  1.0f}; break;}
//       case 2:   {octaves = {0.5f, 0.5f,  1.0f,    0.5f,  0.5f,  1.0f,    0.5f,  0.5f,  1.0f}; break;}
//       case 3:   {octaves = {0.25f, 0.5f,  1.0f,    0.25f,  0.5f,  1.0f,    0.25f,  0.5f,  1.0f}; break;}

//       case 4:   {octaves = {0.25f, 0.5f,  2.0f,    0.25f,  0.5f,  2.0f,    0.25f,  0.5f,  2.0f}; break;}
//       case 5:   {octaves = {0.25f, 0.5f,  4.0f,    0.25f,  0.5f,  4.0f,    0.25f,  0.5f,  4.0f}; break;}
//       case 6:   {octaves = {0.25f, 0.25f,  4.0f,    0.25f,  0.25f,  4.0f,    0.25f,  0.25f,  4.0f}; break;}
//       case 7:   {octaves = {0.5f, 0.25f,  4.0f,    0.5f,  0.25f,  4.0f,    0.5f,  0.25f,  4.0f}; break;}

//       case 8:   {octaves = {0.25f, 2.0f,  4.0f,    0.25f,  2.0f,  4.0f,    0.25f,  2.0f,  4.0f}; break;}
//       case 9:   {octaves = {0.25f, 4.0f,  4.0f,    0.25f,  4.0f,  4.0f,    0.25f,  4.0f,  4.0f}; break;}
//       case 10:  {octaves = {0.5f, 4.0f,  4.0f,    0.5f,  4.0f,  4.0f,    0.25f,  4.0f,  4.0f}; break;}
//       case 11:  {octaves = {1.0f, 4.0f,  4.0f,    1.0f,  4.0f,  4.0f,    1.0f,  4.0f,  4.0f}; break;}

//       case 12:  {octaves = {2.0f, 4.0f,  4.0f,    2.0f,  4.0f,  4.0f,    2.0f,  4.0f,  4.0f}; break;}
//       case 13:  {octaves = {2.0f, 2.0f,  4.0f,    2.0f,  2.0f,  4.0f,    2.0f,  2.0f,  4.0f}; break;}
//       case 14:  {octaves = {1.0f, 2.0f,  4.0f,    1.0f,  2.0f,  4.0f,    1.0f,  2.0f,  4.0f}; break;}
//       case 15:  {octaves = {1.0f, 2.0f,  2.0f,    1.0f,  2.0f,  2.0f,    1.0f,  2.0f,  2.0f}; break;}
//       default:;
//     }
//   }


#endif // OCTAVES_HPP