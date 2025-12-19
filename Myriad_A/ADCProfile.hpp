#ifndef ADC_PROFILE_HPP
#define ADC_PROFILE_HPP

namespace ADCProfile {

typedef struct {
    uint32_t magic;             // 0xCAL1B8ED to validate
    int16_t correction[4096];   // correction[raw_code] = offset to add
    uint16_t adc_min;           // ADC reading at 0V
    uint16_t adc_max;           // ADC reading at full scale
} CalibrationData;

static CalibrationData cal_data;
static uint32_t histogram[4096];

// -----------------------------------------------------------------------------
// Load calibration from LittleFS
// -----------------------------------------------------------------------------
bool load_calibration_from_file(void) {
    if (!LittleFS.begin()) {
        Serial.printf("LittleFS mount failed\n");
        return false;
    }
    
    // Try binary file first (faster)
    File f = LittleFS.open("/histogram.bin", "r");
    if (f) {
        size_t hist_size = f.read((uint8_t*)histogram, sizeof(histogram));
        size_t cal_size = f.read((uint8_t*)&cal_data, sizeof(cal_data));
        f.close();
        
        if (hist_size == sizeof(histogram) && 
            cal_size == sizeof(cal_data) && 
            cal_data.magic == 0xCA11B8ED) {
            Serial.printf("Calibration loaded from LittleFS (range %d-%d)\n", 
                   cal_data.adc_min, cal_data.adc_max);
            return true;
        }
        Serial.printf("Binary file corrupt or incomplete\n");
    }
    
    Serial.printf("No valid calibration in LittleFS\n");
    return false;
}
}


#endif // ADC_PROFILE_HPP