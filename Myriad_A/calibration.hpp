#include <LittleFS.h>
#include <ArduinoJson.h>

#define CALIBMEM __scratch_x("calib")


//adc setup
//oversampling by 3 bits
//https://www.ti.com/lit/an/sprad55a/sprad55a.pdf?ts=1758242355517
constexpr size_t accumulatorBits = 4; 
constexpr size_t accumulatorCount = 1 << accumulatorBits; 
constexpr size_t controlUpdateFreq = 150; //Hz
constexpr size_t adcFreq = accumulatorCount * controlUpdateFreq; //Hz
constexpr size_t adcClockDiv = 48000000 / (adcFreq * 4); //round robin of 4 channels
constexpr size_t adcProcessorDiv = 1000000/adcFreq;
constexpr size_t adcResBits = 13;//12 + accumulatorBits;
constexpr size_t adcDivisor = accumulatorBits + 12 - adcResBits; 
constexpr size_t adcScaleMax = 1<<adcResBits;
constexpr size_t adcInitMin = adcScaleMax * 0.1;
constexpr size_t adcInitMax = adcScaleMax * 0.9;


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