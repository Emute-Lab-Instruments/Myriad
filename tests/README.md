# Myriad Test Suite

This directory contains unit tests for the Myriad project, focusing on validating the fixed-point implementations against their floating-point counterparts.

## Building Tests

The tests are built for the host system (not for RP2040) to enable rapid development and validation.

```bash
cd tests
mkdir build
cd build
cmake ..
make
```

## Running Tests

After building:

```bash
./test_sinetable_fixed
```

Or using CTest:

```bash
ctest --verbose
```

## Test Coverage

### test_sinetable_fixed

Tests the fixed-point sine table implementation (`arduino_libmyriad/sinetable_fixed.hpp`):

1. **Accuracy Tests**: Compares Q16_16 fixed-point output against float version
2. **Edge Cases**: Tests special angles (0, π/2, π, 3π/2, 2π)
3. **Negative Values**: Tests negative inputs and symmetry
4. **Wraparound**: Tests periodic behavior (sin(x) = sin(x + 2π))
5. **Trigonometric Identities**:
   - cos(x) = sin(x + π/2)
   - sin²(x) + cos²(x) = 1
6. **Full Range**: Random sampling across supported range (-2π to 4π)
7. **Resolution**: Validates Q16_16 numerical precision

## Requirements

- C++17 compiler (GCC, Clang)
- CMake 3.13 or higher
- Math library (libm)

## Test Output

The tests produce detailed output showing:
- Individual test pass/fail status
- Error values for accuracy tests
- Summary statistics (total, passed, failed, max error, average error)

Example output:
```
======================================
  Fixed-Point Sine Table Test Suite
======================================

=== TEST 1: Compare Against Float Version ===
  [PASS] sin(-6.2832) (error: 0.000123)
  [PASS] cos(-6.2832) (error: 0.000098)
  ...

======================================
  TEST SUMMARY
======================================
  Total tests:  500
  Passed:       500 (100.0%)
  Failed:       0
  Max error:    1.234e-03
  Avg error:    3.456e-04
======================================

✓ ALL TESTS PASSED!
```

## Adding New Tests

To add new tests:

1. Add test function to `test_sinetable_fixed.cpp`
2. Call the function from `main()`
3. Use `ASSERT_NEAR()` or `ASSERT_TRUE()` macros
4. Rebuild and run

## Notes

- Tests use a fixed random seed (42) for reproducibility
- Tolerance values are chosen based on:
  - Q16_16 resolution: ±0.0000153
  - Table quantization: ±0.003 (360°/1024 ≈ 0.35° per bin)
  - Expected total error: < 0.01 for most cases
