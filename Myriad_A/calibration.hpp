#include <LittleFS.h>
#include <ArduinoJson.h>

#define CALIBMEM __scratch_x("calib")

#include "fixedpoint.hpp"

using namespace FixedPoint;

//adc setup
//oversampling by 3 bits
//https://www.ti.com/lit/an/sprad55a/sprad55a.pdf?ts=1758242355517
// constexpr size_t accumulatorBits = 4; 
// constexpr size_t accumulatorCount = 1 << accumulatorBits; 
// constexpr size_t controlUpdateFreq = 100; //Hz
// constexpr size_t adcFreq = accumulatorCount * controlUpdateFreq; //Hz
// constexpr size_t adcClockDiv = 48000000 / (adcFreq * 4); //round robin of 4 channels
// constexpr size_t adcProcessorDiv = 1000000/adcFreq;
// constexpr size_t adcResBits = 12;//12 + accumulatorBits;
// constexpr size_t adcDivisor = accumulatorBits + 12 - adcResBits; 
constexpr size_t adcScaleMax = 4096;
constexpr size_t adcInitMin = adcScaleMax * 0.1;
constexpr size_t adcInitMax = adcScaleMax * 0.9;
constexpr Fixed<16,16> adcInitMinFP(adcInitMin);
constexpr Fixed<16,16> adcInitMaxFP(adcInitMax);

#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>

#include <type_traits>

namespace PITCHCALSCREEEN {
  int pointIndex=0;
  bool calRunning = false;
  size_t lastPitchADC = 0;

}

// Template parameters:
// - BitDepth: ADC bit resolution (e.g., 12 for 12-bit ADC)
// - CalPoints: Number of calibration points
// - FixedPointT: Integer type for fixed-point storage (uint32_t, int32_t, etc.)
// - FracBits: Number of fractional bits in fixed-point representation
template<int BitDepth, int CalPoints, typename FixedPointT = uint32_t, int FracBits = 16>
class ADCCalibrator {
private:
    static constexpr int ADC_MAX_VALUE = (1 << BitDepth) - 1;
    static constexpr int TABLE_SIZE = ADC_MAX_VALUE + 1;
    static constexpr FixedPointT SCALE_FACTOR = FixedPointT(1) << FracBits;
    static constexpr float SCALE_FACTOR_F = static_cast<float>(SCALE_FACTOR);
    static constexpr bool IS_SIGNED = std::is_signed<FixedPointT>::value;
    
    // Validate template parameters at compile time
    static_assert(std::is_integral<FixedPointT>::value, "FixedPointT must be an integer type");
    static_assert(FracBits > 0 && FracBits < (sizeof(FixedPointT) * 8 - (IS_SIGNED ? 1 : 0)), 
                  "FracBits must leave room for integer part (and sign bit if signed)");
    
    std::array<FixedPointT, TABLE_SIZE> lookupTable;
    float minVoltage;
    float maxVoltage;
    float voltageStep;
    float outputMin;
    float outputMax;
    
public:
    // Type aliases for clarity
    using fixed_t = FixedPointT;
    static constexpr int fractional_bits = FracBits;
    static constexpr int integer_bits = sizeof(FixedPointT) * 8 - FracBits - (IS_SIGNED ? 1 : 0);
    
    // Constructor with output range rescaling (default 0-10)
    constexpr ADCCalibrator(const std::array<int, CalPoints>& calReadings,
                            float vMin = -5.0f,
                            float vMax = 5.0f,
                            float outMin = 0.0f,
                            float outMax = 10.0f)
        : minVoltage(vMin),
          maxVoltage(vMax),
          voltageStep((vMax - vMin) / (CalPoints - 1)),
          outputMin(outMin),
          outputMax(outMax) {
        
        buildLookupTable(calReadings);
    }
    
    // Alternative constructor with C-array
    constexpr ADCCalibrator(const int (&calReadings)[CalPoints],
                            float vMin = -5.0f,
                            float vMax = 5.0f,
                            float outMin = 0.0f,
                            float outMax = 10.0f)
        : minVoltage(vMin),
          maxVoltage(vMax),
          voltageStep((vMax - vMin) / (CalPoints - 1)),
          outputMin(outMin),
          outputMax(outMax) {
        
        std::array<int, CalPoints> calArray;
        for (int i = 0; i < CalPoints; ++i) {
            calArray[i] = calReadings[i];
        }
        buildLookupTable(calArray);
    }


  void rebuildFromThreePointEstimate(size_t minReading, size_t midReading, size_t maxReading) {
      std::array<int, CalPoints> calReadings;
      size_t midIdx = CalPoints / 2;
      calReadings[0] = static_cast<int>(minReading);
      calReadings[CalPoints - 1] = static_cast<int>(maxReading);
      calReadings[midIdx] = static_cast<int>(midReading);
      float lowRange = static_cast<float>(midReading - minReading);
      float highRange = static_cast<float>(maxReading - midReading);
      float nDivisions = midIdx;
      for(size_t i=1; i < midIdx; i++) {
          calReadings[i] = static_cast<int>((minReading + (i * lowRange / nDivisions)) + 0.5f);
          calReadings[midIdx + i] = static_cast<int>((midReading + (i * highRange / nDivisions)) + 0.5f);
      }
      for(size_t i=0; i < CalPoints; i++) {
          Serial.println(calReadings[i]);
      }
      buildLookupTable(calReadings);
  }

  void rebuildLookupTable(const std::array<int, CalPoints>& calReadings) {
    buildLookupTable(calReadings);
  }

    // ========================================================================
    // FIXED-POINT INTERPOLATED LOOKUP
    // ========================================================================
    
    // Interpolated lookup with compile-time fixed-point format
    template<FixedPointType InputFixed>
    inline FixedPointT convertFixedInterpolated(const InputFixed& adcFixed) const {
        using input_storage = typename InputFixed::storage_type;
        
        // Extract integer part (table index)
        int32_t index = adcFixed.to_int();
        
        // Clamp to valid range
        if (index < 0) return lookupTable[0];
        if (index >= ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
        
        // Get fractional part for interpolation (0.0 to 1.0 in fixed point)
        // This is the remainder after extracting the integer part
        input_storage frac = adcFixed.raw() - (static_cast<input_storage>(index) << InputFixed::FRACTIONAL_BITS);
        
        // Look up adjacent table entries
        FixedPointT val0 = lookupTable[index];
        FixedPointT val1 = lookupTable[index + 1];
        
        // Linear interpolation: result = val0 + (val1 - val0) * frac
        // We need to multiply by frac and then shift back
        int64_t diff = static_cast<int64_t>(val1) - val0;
        int64_t scaled_diff = (diff * frac) >> InputFixed::FRACTIONAL_BITS;
        
        return val0 + static_cast<FixedPointT>(scaled_diff);
    }
    
    // Specialized version for common Q19.13 format (optimized for your use case)
    inline FixedPointT convertFixedInterpolated_Q19_13(Q19_13 adcFixed) const {
        // Extract integer part
        int32_t index = adcFixed.to_int();
        
        // Clamp to valid range
        if (index < 0) return lookupTable[0];
        if (index >= ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
        
        // Extract fractional part (13 bits)
        int32_t frac = adcFixed.raw() & ((1 << 13) - 1);  // Mask lower 13 bits
        
        // Look up adjacent entries
        FixedPointT val0 = lookupTable[index];
        FixedPointT val1 = lookupTable[index + 1];
        
        // Interpolate: val0 + (val1 - val0) * (frac / 8192)
        int64_t diff = static_cast<int64_t>(val1) - val0;
        int64_t interpolated = (diff * frac) >> 13;  // Divide by 2^13 = 8192
        
        return val0 + static_cast<FixedPointT>(interpolated);
    }

    // Specialized version for common Q14.18
    inline FixedPointT convertFixedInterpolated_Q14_18(Fixed<14,18> adcFixed) const {
        // Extract integer part
        int32_t index = adcFixed.to_int();
        
        // Clamp to valid range
        if (index < 0) return lookupTable[0];
        if (index >= ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
        
        // Extract fractional part (18 bits)
        int32_t frac = adcFixed.raw() & ((1 << 18) - 1);  // Mask lower bits
        
        // Look up adjacent entries
        FixedPointT val0 = lookupTable[index];
        FixedPointT val1 = lookupTable[index + 1];
        
        // Interpolate: val0 + (val1 - val0) * (frac / x)
        int64_t diff = static_cast<int64_t>(val1) - val0;
        int64_t interpolated = (diff * frac) >> 18;  // 
        
        return val0 + static_cast<FixedPointT>(interpolated);
    }
    
    // Ultra-optimized version for RP2040 with hardware interpolator
    #ifdef PICO_SDK_VERSION_MAJOR
    // inline FixedPointT convertFixedInterpolated_HW(Q19_13 adcFixed) const {
    //     int32_t index = adcFixed.to_int();
        
    //     if (index < 0) return lookupTable[0];
    //     if (index >= ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
        
    //     int32_t frac = adcFixed.raw() & ((1 << 13) - 1);
        
    //     FixedPointT val0 = lookupTable[index];
    //     FixedPointT val1 = lookupTable[index + 1];
        
    //     // Use RP2040 hardware interpolator for the lerp
    //     interp_config cfg = interp_default_config();
    //     interp_config_set_shift(&cfg, 13);  // Divide by 2^13
    //     interp_config_set_signed(&cfg, true);
    //     interp_set_config(interp0, 0, &cfg);
        
    //     int32_t diff = static_cast<int32_t>(val1 - val0);
    //     interp0->accum[0] = diff;
    //     interp0->base[0] = frac;
        
    //     return val0 + static_cast<FixedPointT>(interp0->peek[0]);
    // }
inline FixedPointT convertFixedInterpolated_Q14_18_HW(FixedPoint::Fixed<14, 18, int32_t> adcFixed) const {
    int32_t index = adcFixed.to_int();
    
    if (index < 0) return lookupTable[0];
    if (index >= ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
    
    // Extract fractional part (18 bits) - this is our interpolation factor (0 to 1)
    uint32_t frac = adcFixed.raw() & 0x3FFFF;
    
    FixedPointT val0 = lookupTable[index];
    FixedPointT val1 = lookupTable[index + 1];
    
    // Calculate difference (can be negative)
    int32_t diff = static_cast<int32_t>(val1) - static_cast<int32_t>(val0);
    
    // Configure hardware interpolator for: result = (diff * frac) >> 18
    interp_config cfg = interp_default_config();
    interp_config_set_shift(&cfg, 18);        // Shift right by 18 bits
    interp_config_set_signed(&cfg, true);     // Handle signed arithmetic
    interp_config_set_mask(&cfg, 0, 31);      // Use all 32 bits
    interp_set_config(interp0, 0, &cfg);
    
    // Set base (multiplier) first, then accumulator
    interp0->base[0] = frac;                  // Interpolation factor
    interp0->accum[0] = diff;                 // Value to multiply
    
    // Read result: (diff * frac) >> 18
    int32_t interpolated = static_cast<int32_t>(interp0->peek[0]);
    
    return val0 + static_cast<FixedPointT>(interpolated);
}
    #endif
    
    // ========================================================================
    // CONVENIENCE FUNCTIONS FOR DIFFERENT FORMATS
    // ========================================================================
    inline float convertFixedInterpolatedToFloat_Q14_18(const Fixed<14,18>& adcFixed) const {
        return fixedToFloat(convertFixedInterpolated_Q14_18(adcFixed));
    }
    
    // Convert from any fixed-point format and return as float
    template<FixedPointType InputFixed>
    inline float convertFixedInterpolatedToFloat(const InputFixed& adcFixed) const {
        return fixedToFloat(convertFixedInterpolated(adcFixed));
    }
    
    // Convert with automatic format detection (returns same format as lookup table)
    template<FixedPointType InputFixed>
    inline auto convertAndMatchFormat(const InputFixed& adcFixed) const {
        FixedPointT result = convertFixedInterpolated(adcFixed);
        return Fixed<integer_bits, FracBits, FixedPointT>::from_raw(result);
    }
    
    
    // O(1) lookup - returns fixed-point integer
    inline FixedPointT convertFixed(int adcReading) const {
        if (adcReading < 0) return lookupTable[0];
        if (adcReading > ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
        return lookupTable[adcReading];
    }
    
    // Convert and return as float
    inline float convertFloat(int adcReading) const {
        return fixedToFloat(convertFixed(adcReading));
    }
    
    // Operator[] returns fixed-point value
    inline FixedPointT operator[](int adcReading) const {
        return convertFixed(adcReading);
    }
    
    const int NPoints = CalPoints;
    // Static conversion utilities - C++14 compatible
private:
    // Pre-computed reciprocal for faster division
    static constexpr float SCALE_RECIPROCAL = 1.0f / SCALE_FACTOR_F;
    
    template<bool Signed>
    static constexpr typename std::enable_if<Signed, FixedPointT>::type
    floatToFixedImpl(float value) {
        // Round to nearest integer in fixed-point representation
        return static_cast<FixedPointT>(value * SCALE_FACTOR_F + (value >= 0 ? 0.5f : -0.5f));
    }
    
    template<bool Signed>
    static constexpr typename std::enable_if<!Signed, FixedPointT>::type
    floatToFixedImpl(float value) {
        // For unsigned, ensure we don't go negative
        if (value < 0) return 0;
        // Round to nearest integer in fixed-point representation
        return static_cast<FixedPointT>(value * SCALE_FACTOR_F + 0.5f);
    }
    
public:
    static constexpr FixedPointT floatToFixed(float value) {
        return floatToFixedImpl<IS_SIGNED>(value);
    }
    
    // Optimized fixed-to-float conversion
    static constexpr float fixedToFloat(FixedPointT value) {
        // Use multiplication by reciprocal instead of division
        // This is faster on most MCUs, especially those without hardware division
        return static_cast<float>(value) * SCALE_RECIPROCAL;
    }
    
    // Alternative for MCUs without FPU - returns integer and fractional parts separately
    // This avoids floating point entirely until the final step
    static void fixedToIntFrac(FixedPointT value, int& intPart, int& fracPart1000) {
        intPart = value >> FracBits;
        FixedPointT fracBits = value & ((FixedPointT(1) << FracBits) - 1);
        // Convert fraction to thousandths (0-999) for display without float math
        fracPart1000 = (static_cast<int64_t>(fracBits) * 1000) >> FracBits;
    }
    
    // Fixed-point arithmetic helpers - C++14 compatible using SFINAE
private:
    template<typename T>
    static constexpr typename std::enable_if<(sizeof(T) <= 2), T>::type
    fixedMultiplyImpl(T a, T b, int fracBits) {
        int32_t result = static_cast<int32_t>(a) * static_cast<int32_t>(b);
        return static_cast<T>(result >> fracBits);
    }
    
    template<typename T>
    static constexpr typename std::enable_if<((sizeof(T) > 2) && (sizeof(T) <= 4)), T>::type
    fixedMultiplyImpl(T a, T b, int fracBits) {
        int64_t result = static_cast<int64_t>(a) * static_cast<int64_t>(b);
        return static_cast<T>(result >> fracBits);
    }
    
    template<typename T>
    static constexpr typename std::enable_if<(sizeof(T) > 4), T>::type
    fixedMultiplyImpl(T a, T b, int fracBits) {
        T a_int = a >> fracBits;
        T a_frac = a & ((T(1) << fracBits) - 1);
        T b_int = b >> fracBits;
        T b_frac = b & ((T(1) << fracBits) - 1);
        
        T result = a_int * b_int;
        result += ((a_int * b_frac) >> fracBits) + ((a_frac * b_int) >> fracBits);
        result += (a_frac * b_frac) >> (2 * fracBits);
        return result;
    }
    
    template<typename T>
    static constexpr typename std::enable_if<(sizeof(T) <= 2), T>::type
    fixedDivideImpl(T a, T b, int fracBits) {
        int32_t temp = (static_cast<int32_t>(a) << fracBits) / static_cast<int32_t>(b);
        return static_cast<T>(temp);
    }
    
    template<typename T>
    static constexpr typename std::enable_if<((sizeof(T) > 2) && (sizeof(T) <= 4)), T>::type
    fixedDivideImpl(T a, T b, int fracBits) {
        int64_t temp = (static_cast<int64_t>(a) << fracBits) / static_cast<int64_t>(b);
        return static_cast<T>(temp);
    }
    
    template<typename T>
    static constexpr typename std::enable_if<(sizeof(T) > 4), T>::type
    fixedDivideImpl(T a, T b, int fracBits) {
        return static_cast<T>((a << fracBits) / b);
    }
    
public:
    static constexpr FixedPointT fixedMultiply(FixedPointT a, FixedPointT b) {
        return fixedMultiplyImpl(a, b, FracBits);
    }
    
    static constexpr FixedPointT fixedDivide(FixedPointT a, FixedPointT b) {
        return fixedDivideImpl(a, b, FracBits);
    }
    
    // Get configuration
    static constexpr int getBitDepth() { return BitDepth; }
    static constexpr int getTableSize() { return TABLE_SIZE; }
    static constexpr int getCalibrationPoints() { return CalPoints; }
    static constexpr size_t getMemoryUsage() { return TABLE_SIZE * sizeof(FixedPointT); }
    static constexpr int getFractionalBits() { return FracBits; }
    static constexpr int getIntegerBits() { return integer_bits; }
    static constexpr float getResolution() { return 1.0f / SCALE_FACTOR_F; }
    
    float getMinVoltage() const { return minVoltage; }
    float getMaxVoltage() const { return maxVoltage; }
    float getVoltageStep() const { return voltageStep; }
    float getOutputMin() const { return outputMin; }
    float getOutputMax() const { return outputMax; }
    
    // Debug function to get raw fixed-point value
    FixedPointT getRawFixed(int adcReading) const {
        if (adcReading < 0) return lookupTable[0];
        if (adcReading > ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
        return lookupTable[adcReading];
    }
    
    // Debug function to check intermediate voltage before rescaling
    float getUnscaledVoltage(int adcReading) const {
        std::array<int, CalPoints> calReadings;
        std::array<float, CalPoints> calVoltages;
        
        // This is just for debugging - rebuild calibration arrays
        // In production, you might want to store these
        float currentVoltage = minVoltage;
        for (int i = 0; i < CalPoints; ++i) {
            calVoltages[i] = currentVoltage;
            currentVoltage += voltageStep;
        }
        
        // You'd need to store calReadings or pass them in for this to work
        // For now, returning a placeholder
        return 0.0f; // Placeholder
    }
    
private:
    // Rescale voltage from input range to output range
    float rescaleVoltage(float voltage) const {
        float normalized = (voltage - minVoltage) / (maxVoltage - minVoltage);
        return outputMin + normalized * (outputMax - outputMin);
    }
    
    void buildLookupTable(const std::array<int, CalPoints>& calReadings) {
        // Build voltage array for calibration points
        std::array<float, CalPoints> calVoltages;
        float currentVoltage = minVoltage;
        for (int i = 0; i < CalPoints; ++i) {
            calVoltages[i] = currentVoltage;
            currentVoltage += voltageStep;
        }
        
        // Fill lookup table with rescaled fixed-point values
        for (int adc = 0; adc <= ADC_MAX_VALUE; ++adc) {
            // Get the interpolated voltage in the original range
            float voltage = interpolateVoltage(adc, calReadings, calVoltages);
            // Rescale to output range (e.g., 0-10)
            float rescaled = rescaleVoltage(voltage);
            // Convert to fixed-point
            lookupTable[adc] = floatToFixed(rescaled);
        }
    }
    
    float interpolateVoltage(int adcReading,
                            const std::array<int, CalPoints>& calReadings,
                            const std::array<float, CalPoints>& calVoltages) const {
        
        // Below minimum calibration point
        if (adcReading <= calReadings[0]) {
            if (CalPoints < 2) return calVoltages[0];
            
            float slope = (calVoltages[1] - calVoltages[0]) / 
                         (calReadings[1] - calReadings[0]);
            return calVoltages[0] + slope * (adcReading - calReadings[0]);
        }
        
        // Above maximum calibration point
        if (adcReading >= calReadings[CalPoints - 1]) {
            if (CalPoints < 2) return calVoltages[CalPoints - 1];
            
            float slope = (calVoltages[CalPoints-1] - calVoltages[CalPoints-2]) / 
                         (calReadings[CalPoints-1] - calReadings[CalPoints-2]);
            return calVoltages[CalPoints-1] + slope * (adcReading - calReadings[CalPoints-1]);
        }
        
        // Binary search for interpolation points
        auto it = std::lower_bound(calReadings.begin(), calReadings.end(), adcReading);
        int upperIdx = std::distance(calReadings.begin(), it);
        int lowerIdx = upperIdx - 1;
        
        // Linear interpolation
        float ratio = static_cast<float>(adcReading - calReadings[lowerIdx]) / 
                     (calReadings[upperIdx] - calReadings[lowerIdx]);
        return calVoltages[lowerIdx] + ratio * (calVoltages[upperIdx] - calVoltages[lowerIdx]);
    }
};


// Helper type aliases for common configurations
template<int CalPoints>
using ADC12Bit = ADCCalibrator<12, CalPoints, float>;



// constexpr int cal_12bit_bipolar[11] = {
//     10,    // -5V
//     420,   // -4V
//     835,   // -3V
//     1250,  // -2V
//     1665,  // -1V
//     2048,  //  0V
//     2430,  //  1V
//     2845,  //  2V
//     3260,  //  3V
//     3675,  //  4V
//     4090   //  5V
// };



namespace CalibrationSettings {
  static CALIBMEM size_t adcMins[4] = {adcInitMin,adcInitMin,adcInitMin,adcInitMin};
  static CALIBMEM size_t adcMaxs[4] = {adcInitMax,adcInitMax,adcInitMax,adcInitMax};
  static CALIBMEM size_t adcRanges[4];
  static CALIBMEM float adcRangesInv[4];

  static CALIBMEM Fixed<16,16> adcMinsFP[4] = {adcInitMinFP,adcInitMinFP,adcInitMinFP,adcInitMinFP};
  static CALIBMEM Fixed<16,16> adcMaxsFP[4] = {adcInitMaxFP,adcInitMaxFP,adcInitMaxFP,adcInitMaxFP};
  static CALIBMEM Fixed<16,16> adcRangesFP[4];
  static CALIBMEM Fixed<0,16> adcRangesInvFP[4];

  // static __not_in_flash("calib") std::array<int,9> pitchCalPoints = {
  //   0,  //-5V C-1
  //   512, //-3.75V DNL peak 1, d#0
  //   1024, //-2.5V f#1
  //   1536, //-1.25V DNL peak 2 A2
  //   2048, //C3
  //   2560, //1.25V, DNL peak 3 d#4
  //   3072, //2.5V f#5
  //   3584, //3.75V, DNL peak 4, a6
  //   4095 //5V, C8
  // };

  static __not_in_flash("calib") std::array<int,121> pitchCalPointsInit = {10, 44, 78, 112, 146, 180, 214, 248, 282, 316, 350, 384, 418, 452, 486, 520, 554, 588, 622, 656, 690, 724, 758, 792, 826, 860, 894, 928, 962, 996, 1030, 1064, 1098, 1132, 1166, 1200, 1234, 1268, 1302, 1336, 1370, 1404, 1438, 1472, 1506, 1540, 1574, 1608, 1642, 1676, 1710, 1744, 1778, 1812, 1846, 1880, 1914, 1948, 1982, 2016, 2050, 2084, 2118, 2152, 2186, 2220, 2254, 2288, 2322, 2356, 2390, 2424, 2458, 2492, 2526, 2560, 2594, 2628, 2662, 2696, 2730, 2764, 2798, 2832, 2866, 2900, 2934, 2968, 3002, 3036, 3070, 3104, 3138, 3172, 3206, 3240, 3274, 3308, 3342, 3376, 3410, 3444, 3478, 3512, 3546, 3580, 3614, 3648, 3682, 3716, 3750, 3784, 3818, 3852, 3886, 3920, 3954, 3988, 4022, 4056, 4090};
  static __not_in_flash("calib") std::array<int,121> pitchCalPoints = pitchCalPointsInit;
  
  const char* CALIB_FILE = "/calibration.json";

  void init() {
    for(size_t i=0; i < 4; i++) {
      CalibrationSettings::adcRanges[i] = CalibrationSettings::adcMaxs[i] - CalibrationSettings::adcMins[i];
      CalibrationSettings::adcRangesInv[i] = 1.f / CalibrationSettings::adcRanges[i];
      CalibrationSettings::adcMinsFP[i] = Fixed<16,16>(CalibrationSettings::adcMins[i]);
      CalibrationSettings::adcRangesFP[i] = Fixed<16,16>(CalibrationSettings::adcRanges[i]);
      CalibrationSettings::adcRangesInvFP[i] = Fixed<0,16>(CalibrationSettings::adcRangesInv[i]);
    }
  }

  void reset() {
    adcMins[0] = adcInitMin; adcMins[1] = adcInitMin; adcMins[2] = adcInitMin; adcMins[3] = adcInitMin;
    adcMaxs[0] = adcInitMax; adcMaxs[1] = adcInitMax; adcMaxs[2] = adcInitMax; adcMaxs[3] = adcInitMax;
    pitchCalPoints = pitchCalPointsInit;
    init();
  }

  // void setADC0Mid(size_t mid) {
  //   adc0Mid = mid;
  // }

  bool load() {
    if (!LittleFS.begin()) {
      Serial.println("Failed to mount LittleFS");
      CalibrationSettings::init(); // Initialize with default values
      return false;
    }

    if (!LittleFS.exists(CALIB_FILE)) {
      Serial.println("Calibration file not found, using defaults");
      CalibrationSettings::init(); // Initialize with default values
      return false;
    }

    File file = LittleFS.open(CALIB_FILE, "r");
    if (!file) {
      Serial.println("Failed to open calibration file");
      CalibrationSettings::init(); // Initialize with default values
      return false;
    }

    // Create JSON document
    JsonDocument doc;
    
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      Serial.print("Failed to parse calibration file: ");
      Serial.println(error.c_str());
      CalibrationSettings::init(); // Initialize with default values
      return false;
    }

    // Load values from JSON, using defaults if keys don't exist
    JsonArray mins = doc["adcMins"];
    JsonArray maxs = doc["adcMaxs"];
    JsonArray pitchcal = doc["pcal"];

    // adc0Mid = doc["adc0Mid"] | adc0Mid;

    if (pitchcal.size() == pitchCalPoints.size()) {
      for(size_t i =0; i < pitchCalPoints.size(); i++) {
        pitchCalPoints[i] = pitchcal[i] | pitchCalPoints[i];
      }
    }

    if (mins.size() == 4 && maxs.size() == 4) {
      for (size_t i = 0; i < 4; i++) {
        adcMins[i] = mins[i] | adcMins[i]; // Use default if null
        adcMaxs[i] = maxs[i] | adcMaxs[i]; // Use default if null
      }
      CalibrationSettings::init(); // Calculate ranges and inverse ranges
      Serial.println("Calibration loaded successfully");
      return true;
    } else {
      Serial.println("Invalid calibration data format");
      CalibrationSettings::init(); // Initialize with default values
      return false;
    }
  }

  void save() {
    if (!LittleFS.begin()) {
      Serial.println("Failed to mount LittleFS");
      return;
    }

    // Create JSON document
    JsonDocument doc;
    
    // Create arrays for mins and maxs
    JsonArray mins = doc["adcMins"].to<JsonArray>();
    JsonArray maxs = doc["adcMaxs"].to<JsonArray>();
    JsonArray pitchcal = doc["pcal"].to<JsonArray>();

    // Populate arrays
    for (size_t i = 0; i < 4; i++) {
      mins.add(adcMins[i]);
      maxs.add(adcMaxs[i]);
    }

    for(size_t i=0; i < pitchCalPoints.size(); i++) {
      pitchcal.add(pitchCalPoints[i]);
    }

    // doc["adc0Mid"] = adc0Mid;

    // Open file for writing
    File file = LittleFS.open(CALIB_FILE, "w");
    if (!file) {
      Serial.println("Failed to create calibration file");
      return;
    }

    // Serialize JSON to file
    size_t bytesWritten = serializeJson(doc, file);
    file.close();

    if (bytesWritten > 0) {
      Serial.print("Calibration saved successfully (");
      Serial.print(bytesWritten);
      Serial.println(" bytes)");
    } else {
      Serial.println("Failed to write calibration file");
    }
  }
}