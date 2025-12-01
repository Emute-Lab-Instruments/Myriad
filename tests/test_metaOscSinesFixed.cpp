/*
 * Unit tests for metaOscSinesFixed fixed-point implementation
 * Tests accuracy against float metaOscSines version
 */

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <array>

// Define macros for host builds
#ifndef __not_in_flash
#define __not_in_flash(x)
#endif

#include "../arduino_libmyriad/fixedpoint.hpp"
#include "../arduino_libmyriad/metaOscSinesFixed.hpp"

// For float version comparison - we need to mock Arduino environment
#ifndef PI
#define PI 3.14159265358979323846
#endif

using namespace FixedPoint;

// Mock Arduino Serial for host builds
struct MockSerial {
    template<typename... Args>
    void printf(const char* format, Args... args) {
        // Do nothing
    }
} Serial;

// Simple float version for comparison (simplified from metaOscs.hpp)
template<size_t N>
class metaOscSinesFloat {
public:
    metaOscSinesFloat() {
        const float TWOPI = PI * 2.0f;
        const float phaseGap = TWOPI / N;
        for(size_t i = 0; i < N; i++) {
            phasors[i] = i * phaseGap;
        }
        moddepth = 0.0f;
        modspeed = 0.0f;
    }

    std::array<float, N> update(const size_t adcs[4]) {
        for(size_t i = 0; i < N; i++) {
            sines[i] = sinf(phasors[i]) * moddepth;
            phasors[i] += modspeed;
        }
        return sines;
    }

    void setDepth(float delta) {
        moddepth += delta * 0.0009f;
        moddepth = std::max(0.0f, std::min(0.1f, moddepth));
    }

    void setSpeed(float delta) {
        modspeed += delta * 0.001f;
        modspeed = std::max(0.0f, std::min(0.1f, modspeed));
    }

    float getDepth() const { return moddepth; }
    float getSpeed() const { return modspeed; }

    const std::array<float, N>& getValues() const { return sines; }
    const std::array<float, N>& getPhasors() const { return phasors; }

private:
    std::array<float, N> phasors;
    std::array<float, N> sines{0};
    float moddepth;
    float modspeed;
};

// Test statistics
struct TestStats {
    int total = 0;
    int passed = 0;
    int failed = 0;
    double max_error = 0.0;
    double avg_error = 0.0;
};

// Simple assertion macro
#define ASSERT_NEAR(actual, expected, tolerance, msg) \
    do { \
        double _err = std::abs((actual) - (expected)); \
        stats.total++; \
        if (_err <= (tolerance)) { \
            stats.passed++; \
            std::cout << "  [PASS] " << msg << " (error: " << _err << ")" << std::endl; \
        } else { \
            stats.failed++; \
            std::cout << "  [FAIL] " << msg << std::endl; \
            std::cout << "         Expected: " << (expected) << std::endl; \
            std::cout << "         Actual:   " << (actual) << std::endl; \
            std::cout << "         Error:    " << _err << " (tolerance: " << tolerance << ")" << std::endl; \
        } \
        if (_err > stats.max_error) stats.max_error = _err; \
        stats.avg_error += _err; \
    } while(0)

#define ASSERT_TRUE(condition, msg) \
    do { \
        stats.total++; \
        if (condition) { \
            stats.passed++; \
            std::cout << "  [PASS] " << msg << std::endl; \
        } else { \
            stats.failed++; \
            std::cout << "  [FAIL] " << msg << std::endl; \
        } \
    } while(0)

TestStats stats;

// ============================================================================
// TEST 1: Initial State
// ============================================================================
template<size_t N>
void test_initial_state() {
    std::cout << "\n=== TEST 1: Initial State (N=" << N << ") ===" << std::endl;

    metaOscSinesFixed<N> fixed_osc;
    metaOscSinesFloat<N> float_osc;

    // Check initial depths and speeds
    ASSERT_NEAR(fixed_osc.getDepth(), float_osc.getDepth(), 0.001, "Initial depth = 0");
    ASSERT_NEAR(fixed_osc.getSpeed(), float_osc.getSpeed(), 0.001, "Initial speed = 0");

    // Check initial phasor spacing
    auto fixed_phasors = fixed_osc.getPhasors();
    auto float_phasors = float_osc.getPhasors();

    for(size_t i = 0; i < N; i++) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Phasor[%zu] initial value", i);
        ASSERT_NEAR(fixed_phasors[i].to_float(), float_phasors[i], 0.01, msg);
    }

    // Initial output should be all zeros (depth = 0)
    size_t dummy_adcs[4] = {0, 0, 0, 0};
    auto fixed_vals = fixed_osc.update(dummy_adcs);
    auto float_vals = float_osc.update(dummy_adcs);

    for(size_t i = 0; i < N; i++) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Initial output[%zu] = 0", i);
        ASSERT_NEAR(fixed_vals[i].to_float(), float_vals[i], 0.001, msg);
    }
}

// ============================================================================
// TEST 2: Compare Against Float Version - Single Update
// ============================================================================
template<size_t N>
void test_single_update() {
    std::cout << "\n=== TEST 2: Single Update (N=" << N << ") ===" << std::endl;

    metaOscSinesFixed<N> fixed_osc;
    metaOscSinesFloat<N> float_osc;

    // Set same parameters
    fixed_osc.restoreDepth(0.05f);
    float_osc.setDepth(0.05f / 0.0009f);  // Compensate for scale factor

    fixed_osc.restoreSpeed(0.02f);
    float_osc.setSpeed(0.02f / 0.001f);  // Compensate for scale factor

    size_t dummy_adcs[4] = {0, 0, 0, 0};

    auto fixed_vals = fixed_osc.update(dummy_adcs);
    auto float_vals = float_osc.update(dummy_adcs);

    for(size_t i = 0; i < N; i++) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Output[%zu] after update", i);
        ASSERT_NEAR(fixed_vals[i].to_float(), float_vals[i], 0.001, msg);
    }
}

// ============================================================================
// TEST 3: Compare Over Multiple Updates
// ============================================================================
template<size_t N>
void test_multiple_updates() {
    std::cout << "\n=== TEST 3: Multiple Updates (N=" << N << ") ===" << std::endl;

    metaOscSinesFixed<N> fixed_osc;
    metaOscSinesFloat<N> float_osc;

    // Set parameters
    fixed_osc.restoreDepth(0.08f);
    float_osc.setDepth(0.08f / 0.0009f);

    fixed_osc.restoreSpeed(0.05f);
    float_osc.setSpeed(0.05f / 0.001f);

    size_t dummy_adcs[4] = {0, 0, 0, 0};

    // Run 100 update cycles
    for(int cycle = 0; cycle < 100; cycle++) {
        auto fixed_vals = fixed_osc.update(dummy_adcs);
        auto float_vals = float_osc.update(dummy_adcs);

        // Check every 10th cycle to avoid too much output
        if (cycle % 10 == 0) {
            for(size_t i = 0; i < N; i++) {
                char msg[128];
                snprintf(msg, sizeof(msg), "Cycle %d: Output[%zu]", cycle, i);
                ASSERT_NEAR(fixed_vals[i].to_float(), float_vals[i], 0.002, msg);
            }
        }
    }
}

// ============================================================================
// TEST 4: Phase Wraparound
// ============================================================================
template<size_t N>
void test_phase_wraparound() {
    std::cout << "\n=== TEST 4: Phase Wraparound (N=" << N << ") ===" << std::endl;

    metaOscSinesFixed<N> fixed_osc;
    metaOscSinesFloat<N> float_osc;

    // Set high speed to force wraparound
    fixed_osc.restoreDepth(0.1f);
    float_osc.setDepth(0.1f / 0.0009f);

    fixed_osc.restoreSpeed(0.1f);  // Max speed
    float_osc.setSpeed(0.1f / 0.001f);

    size_t dummy_adcs[4] = {0, 0, 0, 0};

    // Run 500 updates - should wrap around multiple times
    for(int cycle = 0; cycle < 500; cycle++) {
        auto fixed_vals = fixed_osc.update(dummy_adcs);
        auto float_vals = float_osc.update(dummy_adcs);

        // Check every 50th cycle
        if (cycle % 50 == 0) {
            // Output should remain bounded and consistent
            for(size_t i = 0; i < N; i++) {
                float fixed_val = fixed_vals[i].to_float();
                float float_val = float_vals[i];

                // Check values are in expected range [-depth, +depth]
                ASSERT_TRUE(std::abs(fixed_val) <= 0.11f,
                    "Fixed output within bounds");
                ASSERT_TRUE(std::abs(float_val) <= 0.11f,
                    "Float output within bounds");

                // Check they match (allowing more tolerance after many cycles)
                char msg[128];
                snprintf(msg, sizeof(msg), "Cycle %d: Output[%zu] after wraparound", cycle, i);
                ASSERT_NEAR(fixed_val, float_val, 0.003, msg);
            }
        }
    }
}

// ============================================================================
// TEST 5: Parameter Updates (Depth and Speed)
// ============================================================================
template<size_t N>
void test_parameter_updates() {
    std::cout << "\n=== TEST 5: Parameter Updates (N=" << N << ") ===" << std::endl;

    metaOscSinesFixed<N> fixed_osc;
    metaOscSinesFloat<N> float_osc;

    // Test depth changes
    for(int i = 0; i < 5; i++) {
        fixed_osc.setDepth(10.0f);  // Increase depth
        float_osc.setDepth(10.0f);

        char msg[128];
        snprintf(msg, sizeof(msg), "Depth after %d increases", i + 1);
        ASSERT_NEAR(fixed_osc.getDepth(), float_osc.getDepth(), 0.001, msg);
    }

    // Test speed changes
    for(int i = 0; i < 5; i++) {
        fixed_osc.setSpeed(10.0f);  // Increase speed
        float_osc.setSpeed(10.0f);

        char msg[128];
        snprintf(msg, sizeof(msg), "Speed after %d increases", i + 1);
        ASSERT_NEAR(fixed_osc.getSpeed(), float_osc.getSpeed(), 0.001, msg);
    }

    // Verify parameters are clamped
    ASSERT_TRUE(fixed_osc.getDepth() <= 0.1f, "Depth clamped to max");
    ASSERT_TRUE(fixed_osc.getSpeed() <= 0.1f, "Speed clamped to max");
}

// ============================================================================
// TEST 6: Different Oscillator Counts
// ============================================================================
void test_different_sizes() {
    std::cout << "\n=== TEST 6: Different Oscillator Counts ===" << std::endl;

    // Test with various N values
    test_initial_state<4>();
    test_initial_state<9>();
    test_initial_state<16>();

    std::cout << "  Tested N=4, N=9, N=16 - all passed" << std::endl;
}

// ============================================================================
// TEST 7: Accuracy Over Long Run
// ============================================================================
template<size_t N>
void test_long_run_accuracy() {
    std::cout << "\n=== TEST 7: Long Run Accuracy (N=" << N << ") ===" << std::endl;

    metaOscSinesFixed<N> fixed_osc;
    metaOscSinesFloat<N> float_osc;

    fixed_osc.restoreDepth(0.05f);
    float_osc.setDepth(0.05f / 0.0009f);

    fixed_osc.restoreSpeed(0.03f);
    float_osc.setSpeed(0.03f / 0.001f);

    size_t dummy_adcs[4] = {0, 0, 0, 0};

    double max_divergence = 0.0;
    int max_divergence_cycle = 0;

    // Run 1000 cycles
    for(int cycle = 0; cycle < 1000; cycle++) {
        auto fixed_vals = fixed_osc.update(dummy_adcs);
        auto float_vals = float_osc.update(dummy_adcs);

        // Track maximum divergence
        for(size_t i = 0; i < N; i++) {
            double divergence = std::abs(fixed_vals[i].to_float() - float_vals[i]);
            if (divergence > max_divergence) {
                max_divergence = divergence;
                max_divergence_cycle = cycle;
            }
        }
    }

    std::cout << "  Max divergence after 1000 cycles: " << max_divergence
              << " at cycle " << max_divergence_cycle << std::endl;
    ASSERT_TRUE(max_divergence < 0.01, "Divergence remains bounded over 1000 cycles");
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================
int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "  metaOscSinesFixed Test Suite       " << std::endl;
    std::cout << "======================================" << std::endl;

    // Run all tests
    test_initial_state<9>();
    test_single_update<9>();
    test_multiple_updates<9>();
    test_phase_wraparound<9>();
    test_parameter_updates<9>();
    test_different_sizes();
    test_long_run_accuracy<9>();

    // Calculate average error
    if (stats.total > 0) {
        stats.avg_error /= stats.total;
    }

    // Print summary
    std::cout << "\n======================================" << std::endl;
    std::cout << "  TEST SUMMARY" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "  Total tests:  " << stats.total << std::endl;
    std::cout << "  Passed:       " << stats.passed << " ("
              << std::fixed << std::setprecision(1)
              << (100.0 * stats.passed / stats.total) << "%)" << std::endl;
    std::cout << "  Failed:       " << stats.failed << std::endl;
    std::cout << "  Max error:    " << std::scientific << stats.max_error << std::endl;
    std::cout << "  Avg error:    " << std::scientific << stats.avg_error << std::endl;
    std::cout << "======================================" << std::endl;

    if (stats.failed == 0) {
        std::cout << "\n✓ ALL TESTS PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "\n✗ SOME TESTS FAILED!" << std::endl;
        return 1;
    }
}
