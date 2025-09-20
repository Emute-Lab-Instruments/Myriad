#include <LittleFS.h>
#include <ArduinoJson.h>

#define CALIBMEM __scratch_x("calib")


//adc setup
//oversampling by 3 bits
//https://www.ti.com/lit/an/sprad55a/sprad55a.pdf?ts=1758242355517
constexpr size_t accumulatorBits = 4; 
constexpr size_t accumulatorCount = 1 << accumulatorBits; 
constexpr size_t controlUpdateFreq = 100; //Hz
constexpr size_t adcFreq = accumulatorCount * controlUpdateFreq; //Hz
constexpr size_t adcClockDiv = 48000000 / (adcFreq * 4); //round robin of 4 channels
constexpr size_t adcProcessorDiv = 1000000/adcFreq;
constexpr size_t adcResBits = 12;//12 + accumulatorBits;
constexpr size_t adcDivisor = accumulatorBits + 12 - adcResBits; 
constexpr size_t adcScaleMax = 1<<adcResBits;
constexpr size_t adcInitMin = adcScaleMax * 0.1;
constexpr size_t adcInitMax = adcScaleMax * 0.9;

#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>

// Template parameters:
// - BitDepth: ADC bit resolution (e.g., 12 for 12-bit ADC)
// - CalPoints: Number of calibration points
// - T: Data type for lookup table (float, int16_t, etc.)
template<int BitDepth, int CalPoints, typename T = float>
class ADCCalibrator {
private:
    static constexpr int ADC_MAX_VALUE = (1 << BitDepth) - 1;
    static constexpr int TABLE_SIZE = ADC_MAX_VALUE + 1;
    
    std::array<T, TABLE_SIZE> lookupTable;
    float minVoltage;
    float maxVoltage;
    float voltageStep;
    
public:
    // Constructor takes calibration readings and voltage range
    constexpr ADCCalibrator(const std::array<int, CalPoints>& calReadings,
                            float vMin = -5.0f,
                            float vMax = 5.0f)
        : minVoltage(vMin),
          maxVoltage(vMax),
          voltageStep((vMax - vMin) / (CalPoints - 1)) {
        
        buildLookupTable(calReadings);
    }
    
    // Alternative constructor with C-array for easier initialization
    constexpr ADCCalibrator(const int (&calReadings)[CalPoints],
                            float vMin = -5.0f,
                            float vMax = 5.0f)
        : minVoltage(vMin),
          maxVoltage(vMax),
          voltageStep((vMax - vMin) / (CalPoints - 1)) {
        
        std::array<int, CalPoints> calArray;
        for (int i = 0; i < CalPoints; ++i) {
            calArray[i] = calReadings[i];
        }
        buildLookupTable(calArray);
    }
    
    // O(1) lookup - force inline for MCU performance
    inline T convert(int adcReading) const {
        if (adcReading < 0) return lookupTable[0];
        if (adcReading > ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE];
        return lookupTable[adcReading];
    }
    
    // Operator[] for convenient access
    inline T operator[](int adcReading) const {
        return convert(adcReading);
    }
    
    // Get configuration at compile time
    static constexpr int getBitDepth() { return BitDepth; }
    static constexpr int getTableSize() { return TABLE_SIZE; }
    static constexpr int getCalibrationPoints() { return CalPoints; }
    static constexpr size_t getMemoryUsage() { return TABLE_SIZE * sizeof(T); }
    
    float getMinVoltage() const { return minVoltage; }
    float getMaxVoltage() const { return maxVoltage; }
    float getVoltageStep() const { return voltageStep; }
    
private:
    void buildLookupTable(const std::array<int, CalPoints>& calReadings) {
        // Build voltage array for calibration points
        std::array<float, CalPoints> calVoltages;
        float currentVoltage = minVoltage;
        for (int i = 0; i < CalPoints; ++i) {
            calVoltages[i] = currentVoltage;
            currentVoltage += voltageStep;
        }
        
        // Fill lookup table
        for (int adc = 0; adc <= ADC_MAX_VALUE; ++adc) {
            float voltage = interpolateVoltage(adc, calReadings, calVoltages);
            lookupTable[adc] = static_cast<T>(voltage);
        }
    }
    
    float interpolateVoltage(int adcReading,
                            const std::array<int, CalPoints>& calReadings,
                            const std::array<float, CalPoints>& calVoltages) const {
        
        // Below minimum calibration point
        if (adcReading <= calReadings[0]) {
            if constexpr (CalPoints < 2) return calVoltages[0];
            
            float slope = (calVoltages[1] - calVoltages[0]) / 
                         (calReadings[1] - calReadings[0]);
            return calVoltages[0] + slope * (adcReading - calReadings[0]);
        }
        
        // Above maximum calibration point
        if (adcReading >= calReadings[CalPoints - 1]) {
            if constexpr (CalPoints < 2) return calVoltages[CalPoints - 1];
            
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

// // Specialization for compressed storage using fixed-point arithmetic
// template<int BitDepth, int CalPoints>
// class CompressedADCCalibrator {
// private:
//     static constexpr int ADC_MAX_VALUE = (1 << BitDepth) - 1;
//     static constexpr int TABLE_SIZE = ADC_MAX_VALUE + 1;
//     static constexpr float SCALE_FACTOR = 1000.0f;  // millivolt resolution
    
//     std::array<int16_t, TABLE_SIZE> lookupTable;
//     float minVoltage;
//     float maxVoltage;
    
// public:
//     constexpr CompressedADCCalibrator(const int (&calReadings)[CalPoints],
//                                       float vMin = -5.0f,
//                                       float vMax = 5.0f)
//         : minVoltage(vMin), maxVoltage(vMax) {
        
//         // Use float calibrator to build, then compress
//         ADCCalibrator<BitDepth, CalPoints, float> floatCalib(calReadings, vMin, vMax);
        
//         for (int adc = 0; adc <= ADC_MAX_VALUE; ++adc) {
//             float voltage = floatCalib[adc];
//             lookupTable[adc] = static_cast<int16_t>(
//                 voltage * SCALE_FACTOR + (voltage >= 0 ? 0.5f : -0.5f)
//             );
//         }
//     }
    
//     inline float convert(int adcReading) const {
//         if (adcReading < 0) return lookupTable[0] / SCALE_FACTOR;
//         if (adcReading > ADC_MAX_VALUE) return lookupTable[ADC_MAX_VALUE] / SCALE_FACTOR;
//         return lookupTable[adcReading] / SCALE_FACTOR;
//     }
    
//     inline float operator[](int adcReading) const {
//         return convert(adcReading);
//     }
    
//     static constexpr size_t getMemoryUsage() { 
//         return TABLE_SIZE * sizeof(int16_t); 
//     }
// };

// Helper type aliases for common configurations
template<int CalPoints>
using ADC12Bit = ADCCalibrator<12, CalPoints, float>;


// Example usage
#include <iostream>
#include <iomanip>

// Global constant calibration data (stored in flash on MCU)
constexpr int cal_12bit_bipolar[11] = {
    10,    // -5V
    420,   // -4V
    835,   // -3V
    1250,  // -2V
    1665,  // -1V
    2048,  //  0V
    2430,  //  1V
    2845,  //  2V
    3260,  //  3V
    3675,  //  4V
    4090   //  5V
};



namespace CalibrationSettings {
  static CALIBMEM size_t adcMins[4] = {adcInitMin,adcInitMin,adcInitMin,adcInitMin};
  static CALIBMEM size_t adcMaxs[4] = {adcInitMax,adcInitMax,adcInitMax,adcInitMax};
  static CALIBMEM size_t adcRanges[4];
  static CALIBMEM float adcRangesInv[4];

  const char* CALIB_FILE = "/calibration.json";

  void init() {
    for(size_t i=0; i < 4; i++) {
      CalibrationSettings::adcRanges[i] = CalibrationSettings::adcMaxs[i] - CalibrationSettings::adcMins[i];
      CalibrationSettings::adcRangesInv[i] = 1.f / CalibrationSettings::adcRanges[i];
    }
  }

  void reset() {
    adcMins[0] = adcInitMin; adcMins[1] = adcInitMin; adcMins[2] = adcInitMin; adcMins[3] = adcInitMin;
    adcMaxs[0] = adcInitMax; adcMaxs[1] = adcInitMax; adcMaxs[2] = adcInitMax; adcMaxs[3] = adcInitMax;
    init();
  }

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

    // Populate arrays
    for (size_t i = 0; i < 4; i++) {
      mins.add(adcMins[i]);
      maxs.add(adcMaxs[i]);
    }

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