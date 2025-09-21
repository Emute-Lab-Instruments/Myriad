#ifndef EXP2TABLE_HPP
#define EXP2TABLE_HPP

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define TABLE_SIZE 16384
#define MAX_VALUE 10.0f
#define MIN_VALUE 0.0f
#define RANGE (MAX_VALUE - MIN_VALUE)
#define SCALE ((float)(TABLE_SIZE - 1) / RANGE)

static float __not_in_flash("exptable") exp2_table[TABLE_SIZE];

// Initialize the lookup table
void init_exp2_table() {
    // if (exp2_table != NULL) return;
    
    // exp2_table = malloc(TABLE_SIZE * sizeof(float));
    // if (!exp2_table) {
    //     fprintf(stderr, "Failed to allocate lookup table\n");
    //     exit(1);
    // }
    
    // printf("Building exp2f lookup table (%d entries)...\n", TABLE_SIZE);
    
    for (int i = 0; i < TABLE_SIZE; i++) {
        float x = MIN_VALUE + (float)i * RANGE / (TABLE_SIZE - 1);
        exp2_table[i] = exp2f(x);
    }
    
    printf("Table built successfully\n");
}

// Fast exp2f lookup with linear interpolation
float fast_exp2f(float x) {
    // Clamp to valid range
    if (x <= MIN_VALUE) return exp2_table[0];
    if (x >= MAX_VALUE) return exp2_table[TABLE_SIZE - 1];
    
    // Convert to table index (floating point)
    float idx_f = (x - MIN_VALUE) * SCALE;
    int idx = (int)idx_f;
    float frac = idx_f - idx;
    
    // Linear interpolation between adjacent entries
    if (idx >= TABLE_SIZE - 1) {
        return exp2_table[TABLE_SIZE - 1];
    }
    
    return exp2_table[idx] + frac * (exp2_table[idx + 1] - exp2_table[idx]);
}


#endif // EXP2TABLE_HPP