#pragma once

#include "fixedpoint.hpp"
#include "sinetable_fixed.hpp"
#include <array>

using namespace FixedPoint;

/**
 * Fixed-Point Sine Wave Modulation Oscillator
 *
 * Generates N independent sine waves with configurable depth and speed.
 * Uses Q16.16 fixed-point arithmetic internally for better performance.
 *
 * Based on metaOscSines from metaOscs.hpp but with:
 * - Internal fixed-point computation
 * - Q16.16 output array
 * - High-performance sine table lookups
 *
 * @tparam N Number of oscillators
 */
template<size_t N>
class metaOscSinesFixed {
public:
    using FixedType = Q16_16;

    /**
     * Constructor - Initializes N oscillators with equally spaced phases
     */
    metaOscSinesFixed() {
        // Two PI in fixed-point
        constexpr FixedType TWOPI_FIXED = FixedType::from_float_ct(6.28318530718);

        // Initialize with equally spread phases (0, 2π/N, 4π/N, ...)
        const FixedType phaseGap = TWOPI_FIXED / FixedType(N);

        for(size_t i = 0; i < N; i++) {
            phasors[i] = phaseGap * FixedType(i);
        }

        // Initialize parameters (matching float version defaults)
        moddepth = FixedType(0.0f);    // Start at 0
        modspeed = FixedType(0.0f);    // Start at 0

        // Set reasonable ranges
        maxDepth = FixedType(0.1f);    // Max modulation depth
        maxSpeed = FixedType(0.1f);    // Max speed

        depthScale = FixedType(0.0009f);  // Encoder scale
        speedScale = FixedType(0.001f);   // Encoder scale
    }

    /**
     * Update all oscillators and return sine wave values
     *
     * @param adcs Array of 4 ADC values (unused in sine version but kept for interface compatibility)
     * @return Array of N sine wave values in Q16.16 format, range ≈ [-moddepth, +moddepth]
     */
    std::array<FixedType, N> update(const size_t adcs[4]) {
        // Use high-performance software interpolated sine lookup
        for(size_t i = 0; i < N; i++) {
            // Lookup sine value using optimized table
            sines[i] = sine_table.fast_sin_hw(phasors[i]) * moddepth;

            // Advance phase
            phasors[i] += modspeed;

            // Wrap phase (optional - sine table handles wraparound, but keeping phase bounded is cleaner)
            // The sine table supports -2π to 4π range, so we only wrap if way out of bounds
            if (phasors[i] > FixedType(12.566f)) {  // 2π * 2
                phasors[i] -= FixedType(6.283f);    // 2π
            }
        }

        return sines;
    }

    /**
     * Get current sine values without updating
     */
    const std::array<FixedType, N>& getValues() const {
        return sines;
    }

    /**
     * Set modulation depth (how much the sines modulate)
     * @param delta Change in depth (will be scaled and clamped)
     */
    void setDepth(float delta) {
        moddepth += FixedType(delta) * depthScale;
        moddepth = clamp(moddepth, FixedType(0.0f), maxDepth);
    }

    /**
     * Get current modulation depth as float
     */
    float getDepth() const {
        return moddepth.to_float();
    }

    /**
     * Restore depth to specific value (for state loading)
     */
    void restoreDepth(float v) {
        moddepth = FixedType(v);
        moddepth = clamp(moddepth, FixedType(0.0f), maxDepth);
    }

    /**
     * Set modulation speed (how fast the sines oscillate)
     * @param delta Change in speed (will be scaled and clamped)
     */
    void setSpeed(float delta) {
        modspeed += FixedType(delta) * speedScale;
        modspeed = clamp(modspeed, FixedType(0.0f), maxSpeed);
    }

    /**
     * Get current modulation speed as float
     */
    float getSpeed() const {
        return modspeed.to_float();
    }

    /**
     * Restore speed to specific value (for state loading)
     */
    void restoreSpeed(float v) {
        modspeed = FixedType(v);
        modspeed = clamp(modspeed, FixedType(0.0f), maxSpeed);
    }

    /**
     * Get name of oscillator type
     */
    const char* getName() const {
        return "sines (fixed-point)";
    }

    /**
     * Get current phasor values (for debugging/visualization)
     */
    const std::array<FixedType, N>& getPhasors() const {
        return phasors;
    }

private:
    // Clamp helper
    static FixedType clamp(const FixedType& val, const FixedType& minVal, const FixedType& maxVal) {
        if (val < minVal) return minVal;
        if (val > maxVal) return maxVal;
        return val;
    }

    // Sine table instance (shared across all oscillators)
    static SineTableQ16_16 sine_table;

    // Oscillator state
    std::array<FixedType, N> phasors;   // Current phase for each oscillator
    std::array<FixedType, N> sines;     // Current sine values

    // Parameters
    FixedType moddepth;     // Modulation depth (amplitude)
    FixedType modspeed;     // Modulation speed (phase increment per update)

    // Parameter limits
    FixedType maxDepth;
    FixedType maxSpeed;
    FixedType depthScale;   // Encoder scaling factor
    FixedType speedScale;   // Encoder scaling factor
};

// Define static sine table (shared across all instances)
template<size_t N>
FixedPoint::SineTableQ16_16 metaOscSinesFixed<N>::sine_table;

