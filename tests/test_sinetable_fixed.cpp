/*
 * Unit tests for fixed-point sine table implementation
 * Tests accuracy against float version and validates edge cases
 */

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <array>

// Define Pico SDK macro for host builds
#ifndef __not_in_flash
#define __not_in_flash(x)
#endif

#include "../Myriad_A/sinetable.h"
#include "../arduino_libmyriad/sinetable_fixed.hpp"

using namespace FixedPoint;

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
// TEST 1: Compare Against Float Version
// ============================================================================
void test_compare_against_float() {
    std::cout << "\n=== TEST 1: Compare Against Float Version ===" << std::endl;

    SineTableQ16_16 fixed_table;

    // Test 100 evenly-spaced points from -2π to 4π
    constexpr double pi = 3.14159265358979323846;
    constexpr int num_points = 100;

    for (int i = 0; i < num_points; i++) {
        double x = -2.0 * pi + (6.0 * pi * i / (num_points - 1));

        // Compare sine
        float expected_sin = sineTable::fast_sin(x);
        float actual_sin = fixed_table.fast_sin(Q16_16(x)).to_float();

        char msg[128];
        snprintf(msg, sizeof(msg), "sin(%.4f)", x);
        ASSERT_NEAR(actual_sin, expected_sin, 0.002, msg);

        // Compare cosine
        float expected_cos = sineTable::fast_cos(x);
        float actual_cos = fixed_table.fast_cos(Q16_16(x)).to_float();

        snprintf(msg, sizeof(msg), "cos(%.4f)", x);
        ASSERT_NEAR(actual_cos, expected_cos, 0.002, msg);
    }
}

// ============================================================================
// TEST 2: Edge Cases - Special Angles
// ============================================================================
void test_edge_cases() {
    std::cout << "\n=== TEST 2: Edge Cases - Special Angles ===" << std::endl;

    SineTableQ16_16 table;
    constexpr double pi = 3.14159265358979323846;

    // sin(0) ≈ 0
    float sin_0 = table.fast_sin(Q16_16(0.0)).to_float();
    ASSERT_NEAR(sin_0, 0.0, 0.01, "sin(0) = 0");

    // sin(π/2) ≈ 1
    float sin_pi2 = table.fast_sin(Q16_16(pi / 2)).to_float();
    ASSERT_NEAR(sin_pi2, 1.0, 0.01, "sin(π/2) = 1");

    // sin(π) ≈ 0
    float sin_pi = table.fast_sin(Q16_16(pi)).to_float();
    ASSERT_NEAR(sin_pi, 0.0, 0.01, "sin(π) = 0");

    // sin(3π/2) ≈ -1
    float sin_3pi2 = table.fast_sin(Q16_16(3 * pi / 2)).to_float();
    ASSERT_NEAR(sin_3pi2, -1.0, 0.01, "sin(3π/2) = -1");

    // sin(2π) ≈ 0
    float sin_2pi = table.fast_sin(Q16_16(2 * pi)).to_float();
    ASSERT_NEAR(sin_2pi, 0.0, 0.01, "sin(2π) = 0");

    // cos(0) ≈ 1
    float cos_0 = table.fast_cos(Q16_16(0.0)).to_float();
    ASSERT_NEAR(cos_0, 1.0, 0.01, "cos(0) = 1");

    // cos(π/2) ≈ 0
    float cos_pi2 = table.fast_cos(Q16_16(pi / 2)).to_float();
    ASSERT_NEAR(cos_pi2, 0.0, 0.01, "cos(π/2) = 0");

    // cos(π) ≈ -1
    float cos_pi = table.fast_cos(Q16_16(pi)).to_float();
    ASSERT_NEAR(cos_pi, -1.0, 0.01, "cos(π) = -1");

    // cos(3π/2) ≈ 0
    float cos_3pi2 = table.fast_cos(Q16_16(3 * pi / 2)).to_float();
    ASSERT_NEAR(cos_3pi2, 0.0, 0.01, "cos(3π/2) = 0");

    // cos(2π) ≈ 1
    float cos_2pi = table.fast_cos(Q16_16(2 * pi)).to_float();
    ASSERT_NEAR(cos_2pi, 1.0, 0.01, "cos(2π) = 1");
}

// ============================================================================
// TEST 3: Negative Values
// ============================================================================
void test_negative_values() {
    std::cout << "\n=== TEST 3: Negative Values ===" << std::endl;

    SineTableQ16_16 table;
    constexpr double pi = 3.14159265358979323846;

    // sin(-π/2) ≈ -1
    float sin_neg_pi2 = table.fast_sin(Q16_16(-pi / 2)).to_float();
    ASSERT_NEAR(sin_neg_pi2, -1.0, 0.01, "sin(-π/2) = -1");

    // sin(-π) ≈ 0
    float sin_neg_pi = table.fast_sin(Q16_16(-pi)).to_float();
    ASSERT_NEAR(sin_neg_pi, 0.0, 0.01, "sin(-π) = 0");

    // Test symmetry: sin(-x) = -sin(x)
    for (int i = 0; i < 10; i++) {
        double x = (pi * i / 10.0);
        float sin_x = table.fast_sin(Q16_16(x)).to_float();
        float sin_neg_x = table.fast_sin(Q16_16(-x)).to_float();

        char msg[128];
        snprintf(msg, sizeof(msg), "sin(-%.4f) = -sin(%.4f)", x, x);
        ASSERT_NEAR(sin_neg_x, -sin_x, 0.02, msg);
    }
}

// ============================================================================
// TEST 4: Wraparound Behavior
// ============================================================================
void test_wraparound() {
    std::cout << "\n=== TEST 4: Wraparound Behavior ===" << std::endl;

    SineTableQ16_16 table;
    constexpr double pi = 3.14159265358979323846;

    // sin(0) ≈ sin(2π)
    float sin_0 = table.fast_sin(Q16_16(0.0)).to_float();
    float sin_2pi = table.fast_sin(Q16_16(2 * pi)).to_float();
    ASSERT_NEAR(sin_0, sin_2pi, 0.01, "sin(0) = sin(2π)");

    // cos(0) ≈ cos(2π)
    float cos_0 = table.fast_cos(Q16_16(0.0)).to_float();
    float cos_2pi = table.fast_cos(Q16_16(2 * pi)).to_float();
    ASSERT_NEAR(cos_0, cos_2pi, 0.01, "cos(0) = cos(2π)");

    // Test equivalence at multiple points
    for (int i = 0; i < 5; i++) {
        double x = (pi * i / 5.0);
        double x_plus_2pi = x + 2 * pi;

        float sin_x = table.fast_sin(Q16_16(x)).to_float();
        float sin_x2 = table.fast_sin(Q16_16(x_plus_2pi)).to_float();

        char msg[128];
        snprintf(msg, sizeof(msg), "sin(%.4f) = sin(%.4f + 2π)", x, x);
        ASSERT_NEAR(sin_x, sin_x2, 0.02, msg);
    }
}

// ============================================================================
// TEST 5: Cosine-Sine Relationship
// ============================================================================
void test_cosine_sine_relationship() {
    std::cout << "\n=== TEST 5: Cosine-Sine Relationship ===" << std::endl;

    SineTableQ16_16 table;
    constexpr double pi = 3.14159265358979323846;

    // Test cos(x) = sin(x + π/2)
    for (int i = 0; i < 20; i++) {
        double x = -pi + (2 * pi * i / 19.0);

        float cos_x = table.fast_cos(Q16_16(x)).to_float();
        float sin_x_shifted = table.fast_sin(Q16_16(x + pi / 2)).to_float();

        char msg[128];
        snprintf(msg, sizeof(msg), "cos(%.4f) = sin(%.4f + π/2)", x, x);
        ASSERT_NEAR(cos_x, sin_x_shifted, 0.02, msg);
    }

    // Test sin²(x) + cos²(x) = 1
    for (int i = 0; i < 10; i++) {
        double x = (2 * pi * i / 10.0);

        float sin_x = table.fast_sin(Q16_16(x)).to_float();
        float cos_x = table.fast_cos(Q16_16(x)).to_float();
        float sum_squares = sin_x * sin_x + cos_x * cos_x;

        char msg[128];
        snprintf(msg, sizeof(msg), "sin²(%.4f) + cos²(%.4f) = 1", x, x);
        ASSERT_NEAR(sum_squares, 1.0, 0.02, msg);
    }
}

// ============================================================================
// TEST 6: Accuracy Across Full Range
// ============================================================================
void test_accuracy_full_range() {
    std::cout << "\n=== TEST 6: Accuracy Across Full Range ===" << std::endl;

    SineTableQ16_16 fixed_table;
    constexpr double pi = 3.14159265358979323846;

    // Test random points within supported range -2π to 4π
    std::srand(42);  // Fixed seed for reproducibility
    constexpr int num_samples = 100;

    for (int i = 0; i < num_samples; i++) {
        // Random value in range [-2π, 4π]
        double x = -2 * pi + (std::rand() / (double)RAND_MAX) * 6 * pi;

        // Compare against standard library sin/cos
        double expected_sin = std::sin(x);
        double expected_cos = std::cos(x);

        double actual_sin = fixed_table.fast_sin(Q16_16(x)).to_float();
        double actual_cos = fixed_table.fast_cos(Q16_16(x)).to_float();

        // Allow more error since we're comparing to std::sin, not the float table
        char msg[128];
        snprintf(msg, sizeof(msg), "sin(%.4f) vs std::sin", x);
        ASSERT_NEAR(actual_sin, expected_sin, 0.01, msg);

        snprintf(msg, sizeof(msg), "cos(%.4f) vs std::cos", x);
        ASSERT_NEAR(actual_cos, expected_cos, 0.01, msg);
    }
}

// ============================================================================
// TEST 7: Fixed-Point Resolution
// ============================================================================
void test_fixed_point_resolution() {
    std::cout << "\n=== TEST 7: Fixed-Point Resolution ===" << std::endl;

    // Test that Q16_16 can represent the sine table values
    constexpr double resolution = 1.0 / 65536.0;  // Q16_16 resolution

    std::cout << "  Q16_16 resolution: " << resolution << std::endl;
    std::cout << "  Table quantization: ~" << (2.0 * 3.14159 / 1024.0) << " radians/entry" << std::endl;

    // Ensure we can represent values near the limits
    Q16_16 max_val(1.0);
    Q16_16 min_val(-1.0);

    ASSERT_NEAR(max_val.to_float(), 1.0, 0.0001, "Q16_16 can represent 1.0");
    ASSERT_NEAR(min_val.to_float(), -1.0, 0.0001, "Q16_16 can represent -1.0");

    // Test small value representation
    Q16_16 small_val(0.0001);
    ASSERT_TRUE(small_val.to_float() > 0.0, "Q16_16 can represent small positive values");
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================
int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "  Fixed-Point Sine Table Test Suite  " << std::endl;
    std::cout << "======================================" << std::endl;

    // Run all tests
    test_compare_against_float();
    test_edge_cases();
    test_negative_values();
    test_wraparound();
    test_cosine_sine_relationship();
    test_accuracy_full_range();
    test_fixed_point_resolution();

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
