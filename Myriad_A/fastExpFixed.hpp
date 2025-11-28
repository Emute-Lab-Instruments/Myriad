#pragma once

#include "fixedpoint.hpp"
#include <array>
#include <cmath>

namespace FixedPoint {

template<int IntBits, int FracBits, typename StorageT>
class FastExp {
public:
    using FixedT = Fixed<IntBits, FracBits, StorageT>;
    
    static constexpr int TOTAL_BITS = IntBits + FracBits;
    static_assert(TOTAL_BITS == 32, "FastExp optimized for 32-bit fixed point");
    
private:
    static constexpr int32_t ONE = 1 << FracBits;
    
    // More precise constants (using more decimal places)
    static constexpr int32_t LOG2_E = 94548;   // log2(e) = 1.44269504089
    static constexpr int32_t LN2 = 45426;      // ln(2) = 0.69314718056
    static constexpr int32_t INV_LN2 = 94548;  // 1/ln(2) = log2(e)
    
    static constexpr int32_t POLY_C0 = 65536;
    static constexpr int32_t POLY_C1 = 65536;
    static constexpr int32_t POLY_C2 = 32768;
    static constexpr int32_t POLY_C3 = 10922;
    static constexpr int32_t POLY_C4 = 2731;
    static constexpr int32_t POLY_C5 = 546;
    
    static constexpr size_t LUT_SIZE = 256;
    static constexpr size_t LUT_SHIFT = 8;
    
    alignas(4) std::array<int32_t, LUT_SIZE> exp2_lut;
    
public:
    FastExp() : exp2_lut{} {
        // Build table with better precision
        for (size_t i = 0; i < LUT_SIZE; i++) {
            double x = static_cast<double>(i) / LUT_SIZE;
            double result = std::exp2(x);
            // Round carefully
            exp2_lut[i] = static_cast<int32_t>(result * ONE + 0.5);
        }
    }
    
    // ========================================================================
    // IMPROVED LOOKUP WITH INTERPOLATION
    // ========================================================================
    
    inline FixedT exp_lut_interp(const FixedT& x) const {
        int32_t x_val = x.raw();
        
        // Handle extreme values first
        if (x_val > (11 << FracBits)) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (x_val < -(20 << FracBits)) {
            return FixedT::from_raw(1);  // Very small positive value, not zero
        }
        
        // Convert to log2 space with rounding
        // x_log2 = x * log2(e)
        int64_t x_log2_64 = static_cast<int64_t>(x_val) * LOG2_E;
        
        // Add rounding bias for positive values
        if (x_log2_64 >= 0) {
            x_log2_64 += (1LL << (FracBits - 1));
        } else {
            x_log2_64 -= (1LL << (FracBits - 1));
        }
        
        int64_t x_log2 = x_log2_64 >> FracBits;
        
        // Extract integer part (power of 2)
        int32_t n = static_cast<int32_t>(x_log2 >> FracBits);
        
        // Check bounds after extracting n
        if (n >= IntBits) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (n < -FracBits) {
            return FixedT::from_raw(1);
        }
        
        // Get fractional part for interpolation
        // Make sure it's positive
        uint32_t frac = static_cast<uint32_t>(x_log2) & ((1u << FracBits) - 1);
        
        // Calculate LUT index and sub-index
        uint32_t lut_index = frac >> (FracBits - LUT_SHIFT);
        uint32_t lut_frac = frac & ((1u << (FracBits - LUT_SHIFT)) - 1);
        
        // Bounds check
        if (lut_index >= LUT_SIZE) {
            lut_index = LUT_SIZE - 1;
        }
        
        // Get table values
        int32_t val0 = exp2_lut[lut_index];
        int32_t val1;
        
        if (lut_index < LUT_SIZE - 1) {
            val1 = exp2_lut[lut_index + 1];
        } else {
            // At boundary, extrapolate
            val1 = (val0 * 2) - exp2_lut[lut_index - 1];
        }
        
        // Linear interpolation with proper rounding
        int64_t diff = static_cast<int64_t>(val1) - val0;
        int64_t interp_shift = FracBits - LUT_SHIFT;
        int64_t interpolated = val0 + ((diff * lut_frac + (1LL << (interp_shift - 1))) >> interp_shift);
        
        // Clamp to valid range
        if (interpolated > std::numeric_limits<int32_t>::max()) {
            interpolated = std::numeric_limits<int32_t>::max();
        }
        if (interpolated < 0) {
            interpolated = 0;
        }
        
        int32_t result = static_cast<int32_t>(interpolated);
        
        // Apply power of 2 shift
        if (n >= 0) {
            if (n > IntBits - 1) {
                return FixedT::from_raw(std::numeric_limits<int32_t>::max());
            }
            // Check for overflow before shifting
            if (result > (std::numeric_limits<int32_t>::max() >> n)) {
                return FixedT::from_raw(std::numeric_limits<int32_t>::max());
            }
            return FixedT::from_raw(result << n);
        } else {
            int32_t shift_amount = -n;
            if (shift_amount >= 32) {
                return FixedT::from_raw(0);
            }
            // Add rounding
            int32_t rounded = (result + (1 << (shift_amount - 1))) >> shift_amount;
            return FixedT::from_raw(rounded);
        }
    }

    inline FixedT exp_lut_interp_range_0_10_safe(const FixedT& x) const {
        int32_t x_val = x.raw();
        
        // Clamp to [0, 10]
        if (x_val >= (10 << FracBits)) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (x_val < 0) {
            return FixedT::from_raw(ONE);
        }
        
        // Use full precision multiply (one int64 op)
        int32_t x_log2 = static_cast<int32_t>(
            (static_cast<int64_t>(x_val) * LOG2_E) >> FracBits
        );
        
        int32_t n = x_log2 >> FracBits;
        
        uint32_t frac = static_cast<uint32_t>(x_log2) & 0xFFFF;
        uint32_t lut_index = frac >> 8;
        
        // Safe index access
        lut_index = (lut_index < 255) ? lut_index : 254;
        
        int32_t val0 = exp2_lut[lut_index];
        int32_t val1 = exp2_lut[lut_index + 1];
        
        int32_t diff = val1 - val0;
        uint32_t lut_frac = frac & 0xFF;
        
        int32_t interpolated = val0 + ((diff * lut_frac) >> 8);
        
        // Overflow check before shift
        if (n >= 14 && interpolated > (std::numeric_limits<int32_t>::max() >> n)) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        
        return FixedT::from_raw(interpolated << n);
    }
    
    // ========================================================================
    // NON-INTERPOLATED VERSION (FOR COMPARISON)
    // ========================================================================
    
    inline FixedT exp_lut(const FixedT& x) const {
        int32_t x_val = x.raw();
        
        if (x_val > (11 << FracBits)) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (x_val < -(20 << FracBits)) {
            return FixedT::from_raw(1);
        }
        
        // Convert to log2 space
        int64_t x_log2 = (static_cast<int64_t>(x_val) * LOG2_E) >> FracBits;
        
        int32_t n = static_cast<int32_t>(x_log2 >> FracBits);
        
        if (n >= IntBits) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (n < -FracBits) {
            return FixedT::from_raw(0);
        }
        
        uint32_t frac = static_cast<uint32_t>(x_log2) & ((1u << FracBits) - 1);
        uint32_t lut_index = (frac >> (FracBits - LUT_SHIFT));
        
        if (lut_index >= LUT_SIZE) {
            lut_index = LUT_SIZE - 1;
        }
        
        int32_t table_val = exp2_lut[lut_index];
        
        if (n >= 0) {
            if (table_val > (std::numeric_limits<int32_t>::max() >> n)) {
                return FixedT::from_raw(std::numeric_limits<int32_t>::max());
            }
            return FixedT::from_raw(table_val << n);
        } else {
            int32_t shift_amount = -n;
            if (shift_amount >= 32) return FixedT::from_raw(0);
            return FixedT::from_raw((table_val + (1 << (shift_amount - 1))) >> shift_amount);
        }
    }
    
    // Keep other methods (exp_poly, exp_range_reduced, etc.) the same...
    
    inline FixedT exp_poly(const FixedT& x) const {
        int32_t x_val = x.raw();
        
        if (x_val > (IntBits << FracBits)) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (x_val < -(IntBits << FracBits)) {
            return FixedT::from_raw(0);
        }
        
        int64_t x2 = (static_cast<int64_t>(x_val) * x_val) >> FracBits;
        int64_t x3 = (x2 * x_val) >> FracBits;
        int64_t x4 = (x2 * x2) >> FracBits;
        int64_t x5 = (x4 * x_val) >> FracBits;
        
        int64_t result = POLY_C0;
        result += x_val;
        result += (x2 * POLY_C2) >> FracBits;
        result += (x3 * POLY_C3) >> FracBits;
        result += (x4 * POLY_C4) >> FracBits;
        result += (x5 * POLY_C5) >> FracBits;
        
        return FixedT::from_raw(static_cast<int32_t>(result));
    }
    
    inline FixedT exp_range_reduced(const FixedT& x) const {
        int32_t x_val = x.raw();
        
        if (x_val > (11 << FracBits)) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (x_val < -(20 << FracBits)) {
            return FixedT::from_raw(0);
        }
        
        int32_t n = static_cast<int32_t>(((static_cast<int64_t>(x_val) * LOG2_E) + (ONE >> 1)) >> FracBits);
        int32_t r = x_val - static_cast<int32_t>((static_cast<int64_t>(n) * LN2) >> FracBits);
        
        int64_t r2 = (static_cast<int64_t>(r) * r) >> FracBits;
        int64_t r3 = (r2 * r) >> FracBits;
        int64_t r4 = (r2 * r2) >> FracBits;
        
        int64_t exp_r = ONE;
        exp_r += r;
        exp_r += (r2 >> 1);
        exp_r += (r3 * 10922) >> FracBits;
        exp_r += (r4 * 2731) >> FracBits;
        
        if (n >= 0) {
            if (n >= IntBits) {
                return FixedT::from_raw(std::numeric_limits<int32_t>::max());
            }
            return FixedT::from_raw(static_cast<int32_t>(exp_r << n));
        } else {
            if (n <= -FracBits) {
                return FixedT::from_raw(0);
            }
            return FixedT::from_raw(static_cast<int32_t>(exp_r >> -n));
        }
    }
    
    inline FixedT exp2_fixed(const FixedT& x) const {
        int32_t n = x.to_int();
        
        if (n >= IntBits) {
            return FixedT::from_raw(std::numeric_limits<int32_t>::max());
        }
        if (n <= -FracBits) {
            return FixedT::from_raw(0);
        }
        
        uint32_t frac = static_cast<uint32_t>(x.raw()) & ((1u << FracBits) - 1);
        uint32_t lut_index = frac >> (FracBits - LUT_SHIFT);
        
        int32_t table_val = exp2_lut[lut_index];
        
        if (n >= 0) {
            return FixedT::from_raw(table_val << n);
        } else {
            return FixedT::from_raw(table_val >> -n);
        }
    }
    
    inline FixedT exp10_fixed(const FixedT& x) const {
        constexpr int32_t LN10 = 150902;
        int32_t arg = static_cast<int32_t>((static_cast<int64_t>(x.raw()) * LN10) >> FracBits);
        return exp_lut_interp(FixedT::from_raw(arg));
    }
};

// Global instance
inline FastExp<16, 16, int32_t> g_fast_exp_q16;

// Convenience wrappers
inline Q16_16 exp_fast(const Q16_16& x) {
    return g_fast_exp_q16.exp_lut_interp_range_0_10_safe(x);
}

inline Q16_16 exp_fast_poly(const Q16_16& x) {
    return g_fast_exp_q16.exp_poly(x);
}

inline Q16_16 exp_fast_reduced(const Q16_16& x) {
    return g_fast_exp_q16.exp_range_reduced(x);
}

inline Q16_16 exp2_fast(const Q16_16& x) {
    return g_fast_exp_q16.exp2_fixed(x);
}

inline Q16_16 exp10_fast(const Q16_16& x) {
    return g_fast_exp_q16.exp10_fixed(x);
}

} // namespace FixedPoint