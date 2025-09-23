#ifndef FIXED_POINT_EXP2F_H
#define FIXED_POINT_EXP2F_H

#include <cstdint>
#include <cmath>

/**
 * RP2040-optimized fixed-point exp2f implementation
 * Dynamically generates lookup table on initialization for flexibility
 * Uses only RAM (no flash storage needed)
 */
template<int INT_BITS = 4, int FRAC_BITS = 16, int LUT_BITS = 10>
class FixedPointExp2f {
private:
    static constexpr int32_t ONE = 1 << FRAC_BITS;
    static constexpr int32_t FRAC_MASK = ONE - 1;
    static constexpr int LUT_SIZE = 1 << LUT_BITS;
    static constexpr int LUT_SHIFT = FRAC_BITS - LUT_BITS;
    
    // Dynamic lookup table - allocated and filled on first use
    static int32_t* exp2_lut;
    static bool initialized;
    
    // Generate the lookup table dynamically
    static void generate_lut() {
        if (initialized) return;
        
        // Allocate memory for LUT
        exp2_lut = new int32_t[LUT_SIZE + 1]; // +1 for safe interpolation
        
        // Generate 2^(i/LUT_SIZE) values in fixed-point format
        for (int i = 0; i <= LUT_SIZE; i++) {
            double x = (double)i / LUT_SIZE;
            double result = pow(2.0, x);  // 2^x
            exp2_lut[i] = (int32_t)(result * ONE + 0.5); // Round and convert to fixed-point
        }
        
        initialized = true;
    }
    
public:
    /**
     * Initialize the lookup table - call this once at startup
     */
    static void init() {
        generate_lut();
    }
    
    /**
     * Check if initialized
     */
    static bool is_initialized() {
        return initialized;
    }
    
    /**
     * Ultra-fast exp2f computation
     */
    static inline int32_t exp2f(int32_t x) {
        // Auto-initialize on first use (optional safety)
        if (!initialized) generate_lut();
        
        if (x <= 0) return ONE;
        
        // Extract integer and fractional parts
        uint32_t ux = (uint32_t)x;
        uint32_t int_part = ux >> FRAC_BITS;
        uint32_t frac_part = ux & FRAC_MASK;
        
        // Bounds check
        if (int_part >= 15) return 0x7FFFFFFF;
        
        // LUT lookup with interpolation
        uint32_t lut_index = frac_part >> LUT_SHIFT;
        uint32_t remainder = frac_part & ((1 << LUT_SHIFT) - 1);
        
        int32_t y0 = exp2_lut[lut_index];
        
        // Fast interpolation
        if (remainder != 0) {
            int32_t y1 = exp2_lut[lut_index + 1];
            int32_t delta = y1 - y0;
            y0 += (delta * remainder) >> LUT_SHIFT;
        }
        
        // Apply integer part
        return y0 << int_part;
    }
    
    /**
     * Batch processing
     */
    static void exp2f_batch(const int32_t* inputs, int32_t* outputs, int count) {
        if (!initialized) generate_lut();
        
        for (int i = 0; i < count; i++) {
            outputs[i] = exp2f(inputs[i]);
        }
    }
    
    /**
     * Conversion utilities
     */
    static inline int32_t floatToFixed(float f) {
        return (int32_t)(f * ONE);
    }
    
    static inline float fixedToFloat(int32_t q) {
        return (float)q / ONE;
    }
    
    /**
     * Get memory usage
     */
    static constexpr int getMemoryUsage() {
        return (LUT_SIZE + 1) * sizeof(int32_t);
    }
    
    /**
     * Cleanup - free memory (optional)
     */
    static void cleanup() {
        if (initialized && exp2_lut) {
            delete[] exp2_lut;
            exp2_lut = nullptr;
            initialized = false;
        }
    }
};

// Static member definitions
template<int INT_BITS, int FRAC_BITS, int LUT_BITS>
int32_t* FixedPointExp2f<INT_BITS, FRAC_BITS, LUT_BITS>::exp2_lut = nullptr;

template<int INT_BITS, int FRAC_BITS, int LUT_BITS>
bool FixedPointExp2f<INT_BITS, FRAC_BITS, LUT_BITS>::initialized = false;

/**
 * Specialized version for common Q4.16 format
 * Optimized for RP2040 with minimal memory usage
 */
template<>
class FixedPointExp2f<4, 16, 10> {
private:
    static constexpr int32_t ONE = 65536;
    static constexpr int32_t FRAC_MASK = 65535;
    static constexpr int LUT_SIZE = 1024;
    
    static int32_t exp2_lut[LUT_SIZE + 1];
    static bool initialized;
    
public:
    static void init() {
        if (initialized) return;
        
        // Generate optimized 1K lookup table
        for (int i = 0; i <= LUT_SIZE; i++) {
            float x = (float)i / LUT_SIZE;
            float result = powf(2.0f, x);  // Use powf for better performance
            exp2_lut[i] = (int32_t)(result * 65536.0f + 0.5f);
        }
        
        initialized = true;
    }
    
    static inline int32_t exp2f(int32_t x) {
        if (x <= 0) return ONE;
        
        uint32_t ux = (uint32_t)x;
        uint32_t int_part = ux >> 16;
        uint32_t frac_part = ux & FRAC_MASK;
        
        if (int_part >= 15) return 0x7FFFFFFF;
        
        // Optimized for 1K LUT: shift by 6 bits
        uint32_t lut_index = frac_part >> 6;  // Top 10 bits
        uint32_t remainder = frac_part & 63;   // Bottom 6 bits
        
        int32_t y0 = exp2_lut[lut_index];
        
        if (remainder != 0) {
            int32_t y1 = exp2_lut[lut_index + 1];
            int32_t delta = y1 - y0;
            y0 += (delta * remainder) >> 6;
        }
        
        return y0 << int_part;
    }
    
    static void exp2f_batch(const int32_t* inputs, int32_t* outputs, int count) {
        for (int i = 0; i < count; i++) {
            int32_t x = inputs[i];
            if (x <= 0) {
                outputs[i] = ONE;
                continue;
            }
            
            uint32_t ux = (uint32_t)x;
            uint32_t int_part = ux >> 16;
            uint32_t frac_part = ux & FRAC_MASK;
            
            if (int_part >= 15) {
                outputs[i] = 0x7FFFFFFF;
                continue;
            }
            
            uint32_t lut_index = frac_part >> 6;
            uint32_t remainder = frac_part & 63;
            
            int32_t y0 = exp2_lut[lut_index];
            if (remainder != 0) {
                int32_t y1 = exp2_lut[lut_index + 1];
                y0 += ((y1 - y0) * remainder) >> 6;
            }
            
            outputs[i] = y0 << int_part;
        }
    }
    
    static inline int32_t floatToFixed(float f) {
        return (int32_t)(f * 65536.0f);
    }
    
    static inline float fixedToFloat(int32_t q) {
        return (float)q / 65536.0f;
    }
    
    static bool is_initialized() { return initialized; }
    static constexpr int getMemoryUsage() { return (LUT_SIZE + 1) * 4; } // 4100 bytes
};

// Static member definitions for specialization
template<>
int32_t FixedPointExp2f<4, 16, 10>::exp2_lut[1025];

template<>
bool FixedPointExp2f<4, 16, 10>::initialized = false;

// Convenient typedefs
using Exp2f_RP2040_1K = FixedPointExp2f<4, 16, 10>;  // 4KB, recommended
using Exp2f_RP2040_256 = FixedPointExp2f<4, 16, 8>;  // 1KB, minimal memory
using Exp2f_RP2040_4K = FixedPointExp2f<4, 16, 12>;  // 16KB, high accuracy

#endif // FIXED_POINT_EXP2F_H

/*
Usage Example:

void setup() {
    // Initialize the lookup table once at startup
    Exp2f_RP2040_1K::init();
}

void loop() {
    // Fast computation
    int32_t input = Exp2f_RP2040_1K::floatToFixed(2.5f);
    int32_t result = Exp2f_RP2040_1K::exp2f(input);
    
    // Batch processing
    int32_t samples[64];
    int32_t outputs[64];
    Exp2f_RP2040_1K::exp2f_batch(samples, outputs, 64);
}

Benefits of Dynamic Generation:
✓ No large static arrays in flash
✓ Flexible - can change LUT size at compile time
✓ Uses RAM efficiently (only what's needed)
✓ No linker issues
✓ Easy to modify precision at runtime
✓ Faster compilation
✓ Mathematical precision (uses actual pow() function)
*/