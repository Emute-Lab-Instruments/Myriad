#ifndef ADC_PROFILE_HPP
#define ADC_PROFILE_HPP

#include "fixedpoint.hpp"
using namespace FixedPoint;
namespace ADCProfile {

typedef struct {
    uint32_t magic;             // 0xCAL1B8ED to validate
    int16_t correction[4096];   // correction[raw_code] = offset to add
    uint16_t adc_min;           // ADC reading at 0V
    uint16_t adc_max;           // ADC reading at full scale
} CalibrationData;

Q16_16 adcVRangeInv; 


static CalibrationData cal_data;
static uint32_t histogram[4096];

void smooth_correction_table() {
    Serial.printf("Smoothing correction table across bit-9 boundaries...\n");
    
    const int boundaries[] = {512, 1536, 2560, 3584};
    const int radius = 16;
    
    for (int b = 0; b < 4; b++) {
        int center = boundaries[b];
        
        int left_idx = center - radius - 1;
        int right_idx = center + radius;
        
        if (left_idx < 0) left_idx = 0;
        if (right_idx > 4095) right_idx = 4095;
        
        // Read values BEFORE modifying
        int16_t left_val = cal_data.correction[left_idx];
        int16_t right_val = cal_data.correction[right_idx];
        
        // Now overwrite in place
        for (int i = left_idx + 1; i < right_idx; i++) {
            float t = (float)(i - left_idx) / (right_idx - left_idx);
            cal_data.correction[i] = left_val + (int16_t)(t * (right_val - left_val));
        }
        
        Serial.printf("  Boundary %d: smoothed codes %d-%d\n", center, left_idx, right_idx);
    }
    
    Serial.printf("  Done\n\n");
}

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
            smooth_correction_table();
            adcVRangeInv = Q16_16(10) / Q16_16(cal_data.adc_max); 
            return true;
        }
        Serial.printf("Binary file corrupt or incomplete\n");
    }
    
    Serial.printf("No valid calibration in LittleFS\n");
    return false;
}
}


#endif // ADC_PROFILE_HPP