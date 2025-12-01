/*
 * RP2040 Hardware Test for Fixed-Point Sine Table
 * Tests hardware interpolator performance and accuracy
 *
 * Build: cd build && cmake .. && make
 * Flash: picotool load test_sinetable_rp2040.uf2 && picotool reboot
 * Monitor: minicom -b 115200 -D /dev/ttyACM0
 */

#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/interp.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "../arduino_libmyriad/sinetable_fixed.hpp"
#include "../Myriad_A/sinetable.h"

using namespace FixedPoint;

// Test statistics
struct TestResults {
    const char* name;
    uint32_t cycles;
    double max_error;
    double avg_error;
    int passed;
    int failed;
};

// Timing utilities - use hardware timer (M0+ doesn't have DWT)
// RP2040 timer runs at 1 MHz, so we multiply by 125 to get approximate cycles
static inline uint32_t get_cycles() {
    return time_us_64() * 125;  // 125 MHz clock, so µs * 125 = cycles
}

// ============================================================================
// TEST 1: Accuracy - Hardware Interpolator vs Software
// ============================================================================
void test_accuracy_hw_vs_sw() {
    printf("\n=== TEST 1: Hardware Interpolator Accuracy ===\n");

    SineTableQ16_16 table;
    constexpr double pi = 3.14159265358979323846;

    double max_error = 0.0;
    double sum_error = 0.0;
    int num_tests = 100;

    for (int i = 0; i < num_tests; i++) {
        double angle = -2 * pi + (6 * pi * i / (num_tests - 1));
        Q16_16 x(angle);

        // Software version
        Q16_16 sw_result = table.fast_sin(x);

        // Hardware version
        Q16_16 hw_result = table.fast_sin_hw(x);

        // Compare against std::sin
        double expected = std::sin(angle);
        double sw_val = sw_result.to_float();
        double hw_val = hw_result.to_float();

        double sw_error = std::abs(sw_val - expected);
        double hw_error = std::abs(hw_val - expected);

        sum_error += hw_error;
        if (hw_error > max_error) max_error = hw_error;

        // Print sample results
        if (i % 20 == 0) {
            printf("  Angle: %.4f  SW err: %.6f  HW err: %.6f\n",
                   angle, sw_error, hw_error);
        }
    }

    printf("  Hardware Max Error:  %.6f\n", max_error);
    printf("  Hardware Avg Error:  %.6f\n", sum_error / num_tests);
    printf("  Expected HW error:   < 0.0001 (10-100x better than SW)\n");
}

// ============================================================================
// TEST 2: Special Angles - Hardware Interpolator
// ============================================================================
void test_edge_cases_hw() {
    printf("\n=== TEST 2: Hardware Interpolator - Edge Cases ===\n");

    SineTableQ16_16 table;
    constexpr double pi = 3.14159265358979323846;

    struct TestCase {
        double angle;
        const char* name;
        double expected_sin;
        double expected_cos;
    };

    TestCase cases[] = {
        {0.0,      "sin(0)",      0.0,  1.0},
        {pi/2,     "sin(π/2)",    1.0,  0.0},
        {pi,       "sin(π)",      0.0, -1.0},
        {3*pi/2,   "sin(3π/2)",  -1.0,  0.0},
        {2*pi,     "sin(2π)",     0.0,  1.0},
        {pi/4,     "sin(π/4)",    0.707, 0.707},
        {pi/6,     "sin(π/6)",    0.5,   0.866},
    };

    for (const auto& tc : cases) {
        Q16_16 x(tc.angle);
        Q16_16 sin_hw = table.fast_sin_hw(x);
        Q16_16 cos_hw = table.fast_cos_hw(x);

        double sin_val = sin_hw.to_float();
        double cos_val = cos_hw.to_float();
        double sin_err = std::abs(sin_val - tc.expected_sin);
        double cos_err = std::abs(cos_val - tc.expected_cos);

        const char* sin_status = (sin_err < 0.01) ? "PASS" : "FAIL";
        const char* cos_status = (cos_err < 0.01) ? "PASS" : "FAIL";

        printf("  [%s] %-12s = %.4f (err: %.6f)\n",
               sin_status, tc.name, sin_val, sin_err);
        printf("  [%s] c%-11s = %.4f (err: %.6f)\n",
               cos_status, tc.name, cos_val, cos_err);
    }
}

// ============================================================================
// TEST 3: Performance Benchmarking
// ============================================================================
void test_performance() {
    printf("\n=== TEST 3: Performance Benchmarking ===\n");

    SineTableQ16_16 table;
    constexpr int ITERATIONS = 1000;
    constexpr double angle = 1.234;

    // Warmup
    for (int i = 0; i < 100; i++) {
        volatile Q16_16 result = table.fast_sin(Q16_16(angle));
    }

    // Benchmark 1: Software floor lookup (no interpolation)
    uint32_t start_floor = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        volatile Q16_16 result = table.fast_sin(Q16_16(angle));
    }
    uint32_t end_floor = get_cycles();
    uint32_t time_floor = end_floor - start_floor;

    // Benchmark 2: Software linear interpolation
    uint32_t start_interp = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        volatile Q16_16 result = table.fast_sin_hw(Q16_16(angle));
    }
    uint32_t end_interp = get_cycles();
    uint32_t time_interp = end_interp - start_interp;

    // Benchmark 3: Hardware-accelerated interpolation
    uint32_t start_hw_accel = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        volatile Q16_16 result = table.fast_sin_hw_accel(Q16_16(angle));
    }
    uint32_t end_hw_accel = get_cycles();
    uint32_t time_hw_accel = end_hw_accel - start_hw_accel;

    // Benchmark 4: Symmetry-optimized version
    uint32_t start_sym = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        volatile Q16_16 result = table.fast_sin_sym(Q16_16(angle));
    }
    uint32_t end_sym = get_cycles();
    uint32_t time_sym = end_sym - start_sym;

    // Benchmark 5: Float version for comparison
    uint32_t start_float = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        volatile float result = sineTable::fast_sin(angle);
    }
    uint32_t end_float = get_cycles();
    uint32_t time_float = end_float - start_float;

    // Benchmark 6: std::sin as baseline
    uint32_t start_std = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        volatile double result = std::sin(angle);
    }
    uint32_t end_std = get_cycles();
    uint32_t time_std = end_std - start_std;

    printf("  Iterations: %d\n", ITERATIONS);
    printf("\n  Fixed-Point Implementations:\n");
    printf("  1. Floor lookup:      %6lu cycles (%.1f cycles/call)\n",
           time_floor, (float)time_floor / ITERATIONS);
    printf("  2. SW Interpolation:  %6lu cycles (%.1f cycles/call)\n",
           time_interp, (float)time_interp / ITERATIONS);
    printf("  3. HW Interpolation:  %6lu cycles (%.1f cycles/call)\n",
           time_hw_accel, (float)time_hw_accel / ITERATIONS);
    printf("  4. Symmetry-optimized:%6lu cycles (%.1f cycles/call)\n",
           time_sym, (float)time_sym / ITERATIONS);

    printf("\n  Reference Implementations:\n");
    printf("  Float Lookup:         %6lu cycles (%.1f cycles/call)\n",
           time_float, (float)time_float / ITERATIONS);
    printf("  std::sin (baseline):  %6lu cycles (%.1f cycles/call)\n",
           time_std, (float)time_std / ITERATIONS);

    printf("\n  Best speedup vs std::sin: %.2fx\n",
           (float)time_std / ((time_hw_accel < time_interp && time_hw_accel < time_sym) ? time_hw_accel :
                              (time_interp < time_sym ? time_interp : time_sym)));
}

// ============================================================================
// TEST 4: Dual Hardware Interpolator
// ============================================================================
void test_dual_hw() {
    printf("\n=== TEST 4: Dual Hardware Interpolator ===\n");

    SineTableQ16_16 table;
    constexpr double pi = 3.14159265358979323846;

    // Test simultaneous processing
    Q16_16 x0(pi/4);
    Q16_16 x1(pi/3);
    Q16_16 out0, out1;

    table.fast_sin_dual_hw(x0, x1, out0, out1);

    double expected0 = std::sin(pi/4);
    double expected1 = std::sin(pi/3);

    double actual0 = out0.to_float();
    double actual1 = out1.to_float();

    double error0 = std::abs(actual0 - expected0);
    double error1 = std::abs(actual1 - expected1);

    printf("  sin(π/4): %.4f (expected: %.4f, error: %.6f) %s\n",
           actual0, expected0, error0, error0 < 0.01 ? "PASS" : "FAIL");
    printf("  sin(π/3): %.4f (expected: %.4f, error: %.6f) %s\n",
           actual1, expected1, error1, error1 < 0.01 ? "PASS" : "FAIL");

    // Benchmark dual processing
    constexpr int ITERATIONS = 1000;

    uint32_t start_single = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        volatile Q16_16 r0 = table.fast_sin_hw(x0);
        volatile Q16_16 r1 = table.fast_sin_hw(x1);
    }
    uint32_t end_single = get_cycles();

    uint32_t start_dual = get_cycles();
    for (int i = 0; i < ITERATIONS; i++) {
        table.fast_sin_dual_hw(x0, x1, out0, out1);
    }
    uint32_t end_dual = get_cycles();

    uint32_t time_single = end_single - start_single;
    uint32_t time_dual = end_dual - start_dual;

    printf("\n  Performance (2 values, %d iterations):\n", ITERATIONS);
    printf("  Sequential HW: %lu cycles (%.1f cycles/call)\n", time_single, (float)time_single / ITERATIONS);
    printf("  Dual HW:       %lu cycles (%.1f cycles/call)\n", time_dual, (float)time_dual / ITERATIONS);
    printf("  Speedup:       %.2fx\n", (float)time_single / time_dual);
}

// ============================================================================
// TEST 5: Interpolation Quality
// ============================================================================
void test_interpolation_quality() {
    printf("\n=== TEST 5: Interpolation Quality ===\n");

    SineTableQ16_16 table;

    // Test angles between table entries to verify interpolation
    // Table has 1024 entries over 2π, so ~0.00614 radians per entry
    constexpr double entry_spacing = 6.283185307179586 / 1024.0;

    printf("  Testing interpolation between table entries...\n");
    printf("  Entry spacing: %.6f radians\n", entry_spacing);

    double max_sw_error = 0.0;
    double max_hw_error = 0.0;

    for (int i = 0; i < 100; i++) {
        // Test midpoint between entries
        double angle = (i * entry_spacing) + (entry_spacing / 2);
        Q16_16 x(angle);

        double expected = std::sin(angle);
        double sw_val = table.fast_sin(x).to_float();
        double hw_val = table.fast_sin_hw(x).to_float();

        double sw_error = std::abs(sw_val - expected);
        double hw_error = std::abs(hw_val - expected);

        if (sw_error > max_sw_error) max_sw_error = sw_error;
        if (hw_error > max_hw_error) max_hw_error = hw_error;
    }

    printf("  Software (floor):       max error = %.6f\n", max_sw_error);
    printf("  Hardware (linear):      max error = %.6f\n", max_hw_error);
    printf("  Interpolation benefit:  %.1fx better\n", max_sw_error / max_hw_error);
}

// ============================================================================
// MAIN
// ============================================================================
int main() {
    // Initialize stdio for USB output
    stdio_init_all();

    // Wait for serial connection (like Arduino's while(!Serial){})
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);  // Extra delay for terminal to be ready

    printf("\n");
    printf("======================================\n");
    printf("  RP2040 Sine Table Hardware Test\n");
    printf("  Q16_16 Fixed-Point Implementation\n");
    printf("======================================\n");
    printf("\n");
    printf("Board: RP2040\n");
    printf("Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
    printf("SDK Version: %s\n", PICO_SDK_VERSION_STRING);
    printf("Timing: Hardware timer (1 µs resolution)\n");
    printf("\n");

    // Run all tests
    test_accuracy_hw_vs_sw();
    test_edge_cases_hw();
    test_performance();
    test_dual_hw();
    test_interpolation_quality();

    printf("\n");
    printf("======================================\n");
    printf("  All Tests Complete!\n");
    printf("======================================\n");

    // Blink LED to indicate completion
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}
