#ifndef MEDIANS_HPP
#define MEDIANS_HPP


typedef struct {
    size_t samples[3] = {0,0,0};    // Circular buffer for 3 samples
    uint8_t index=0;        // Current index (0-2)
} median_filter_t;


static inline size_t median3_fast(size_t a, size_t b, size_t c) {
    // Use conditional moves instead of branches for better performance
    size_t max_ab = (a > b) ? a : b;
    size_t min_ab = (a > b) ? b : a;
    size_t max_all = (max_ab > c) ? max_ab : c;
    size_t min_all = (min_ab < c) ? min_ab : c;
    
    // Median is the sum minus max minus min
    return (a + b + c) - max_all - min_all;
}

static inline size_t median_filter_3pt_update_fast(median_filter_t* filter, size_t new_sample) {
    // Store sample and advance index
    filter->samples[filter->index] = new_sample;
    filter->index = (filter->index + 1) % 3;
    
    // Always return median (assumes filter is pre-filled)
    return median3_fast(filter->samples[0], filter->samples[1], filter->samples[2]);
}

typedef struct {
    size_t samples[5] = {0,0,0,0,0};    // Circular buffer for 5 samples
    uint8_t index=0;        // Current index (0-4)
    uint8_t count=0;        // Number of samples collected (0-5)
    uint16_t padding;     // Ensure 32-bit alignment
} median_filter_5pt_t;

static inline void median_filter_5pt_init(median_filter_5pt_t* filter) {
    filter->index = 0;
    filter->count = 0;
}

// Optimized 5-point median using partial sorting network
// Minimizes comparisons and swaps for ARM Cortex-M0+
static inline size_t median5_fast(size_t a, size_t b, size_t c, size_t d, size_t e) {
    size_t temp;
    
    // Optimized sorting network for 5 elements (9 comparisons)
    // Variables a,b,c,d,e are sorted in-place using only registers
    
    // First pass - sort pairs to establish partial order
    if (a > b) { temp = a; a = b; b = temp; }  // a <= b
    if (d > e) { temp = d; d = e; e = temp; }  // d <= e
    if (a > c) { temp = a; a = c; c = temp; }  // a <= c, maintain a <= b
    if (b > c) { temp = b; b = c; c = temp; }  // a <= b <= c
    
    // Second pass - merge the two sorted pairs with the sorted triple
    if (a > d) { temp = a; a = d; d = temp; }  // ensure a is minimum
    if (c > e) { temp = c; c = e; e = temp; }  // ensure e is maximum
    if (b > d) { temp = b; b = d; d = temp; }  // position b and d
    if (c > d) { temp = c; c = d; d = temp; }  // position c and d
    if (b > c) { temp = b; b = c; c = temp; }  // final sort b and c
    
    return c; // c is guaranteed to be the median after this sorting network
}

static inline size_t median_filter_5pt_update_fast(median_filter_5pt_t* filter, size_t new_sample) {
    filter->samples[filter->index] = new_sample;
    filter->index = (filter->index + 1) % 5;
    
    return median5_fast(filter->samples[0], filter->samples[1], filter->samples[2],
                       filter->samples[3], filter->samples[4]);
}

#endif // MEDIANS_HPP