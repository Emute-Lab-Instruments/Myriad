
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
#define CAL_PWM_PIN         2
#define CAL_ADC_PIN         29
#define CAL_ADC_CHAN        3

#define PWM_WRAP            256       // 11-bit
#define SETTLE_US           1000           // RC filter settling (assumes ~100µs τ, 20× margin)
#define SAMPLES_PER_LEVEL   32         // Heavy averaging at each PWM level
#define NUM_SWEEPS          4         // Multiple sweeps (2 up, 2 down) to average hysteresis

// Flash storage for calibration
#define CAL_FLASH_OFFSET    (1024 * 1024)  // 1MB into flash (adjust for your binary size)

// -----------------------------------------------------------------------------
// Calibration data structures
// -----------------------------------------------------------------------------
typedef struct {
    uint32_t magic;             // 0xCAL1B8ED to validate
    int16_t correction[4096];   // correction[raw_code] = offset to add
    uint16_t adc_min;           // ADC reading at 0V
    uint16_t adc_max;           // ADC reading at full scale
} CalibrationData;

static CalibrationData cal_data;
static uint32_t histogram[4096];

// -----------------------------------------------------------------------------
// Hardware init
// -----------------------------------------------------------------------------

#include <LittleFS.h>

// -----------------------------------------------------------------------------
// Save histogram to LittleFS
// -----------------------------------------------------------------------------
void save_histogram_to_file(void) {
    Serial.printf("Saving histogram to LittleFS...\n");
    
    if (!LittleFS.begin()) {
        Serial.printf("  LittleFS mount failed, formatting...\n");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.printf("  LittleFS failed after format!\n");
            return;
        }
    }
    
    // Save as CSV for easy Jupyter import
    File f = LittleFS.open("/adc_cal.csv", "w");
    if (!f) {
        Serial.printf("  Failed to open file for writing\n");
        return;
    }
    
    f.println("code,count,correction");
    for (int i = 0; i < 4096; i++) {
        f.printf("%d,%lu,%d\n", i, histogram[i], cal_data.correction[i]);
    }
    f.close();
    
    // Also save binary for fast loading
    save_histogram_binary();
    
    // Verify
    File check = LittleFS.open("/adc_cal.csv", "r");  // Fixed filename
    if (check) {
        Serial.printf("  Saved adc_cal.csv (%d bytes)\n", check.size());
        check.close();
    }
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
            return true;
        }
        Serial.printf("Binary file corrupt or incomplete\n");
    }
    
    Serial.printf("No valid calibration in LittleFS\n");
    return false;
}


// Also save raw binary for faster loading if needed
void save_histogram_binary(void) {
    File f = LittleFS.open("/histogram.bin", "w");
    if (!f) return;
    
    f.write((uint8_t*)histogram, sizeof(histogram));
    f.write((uint8_t*)&cal_data, sizeof(cal_data));
    f.close();
    
    Serial.printf("  Saved histogram.bin\n");
}

void dump_csv_to_serial(void) {
    Serial.println("CSV_START");
    Serial.println("code,count,correction");
    for (int i = 0; i < 4096; i++) {
        Serial.printf("%d,", histogram[i]);
    }
    Serial.println();
    Serial.println("CSV_END");
    Serial.flush();
    Serial.println("CORR_START");
    Serial.println("code,count,correction");
    for (int i = 0; i < 4096; i++) {
        Serial.printf("%d,", cal_data.correction[i]);
    }
    Serial.println();
    Serial.println("CORR_END");
    Serial.flush();

    
}

static uint pwm_slice;

void cal_pwm_init(void) {
    gpio_set_function(CAL_PWM_PIN, GPIO_FUNC_PWM);
    pwm_slice = pwm_gpio_to_slice_num(CAL_PWM_PIN);
    pwm_set_wrap(pwm_slice, PWM_WRAP);
    pwm_set_clkdiv(pwm_slice, 1.0f);
    pwm_set_gpio_level(CAL_PWM_PIN, 0);
    pwm_set_enabled(pwm_slice, true);
}

void cal_adc_init(void) {
    adc_init();
    adc_gpio_init(CAL_ADC_PIN);
    adc_select_input(CAL_ADC_CHAN);
}

// -----------------------------------------------------------------------------
// ADC reading with heavy averaging
// -----------------------------------------------------------------------------
uint16_t adc_read_averaged(uint32_t num_samples) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < num_samples; i++) {
        sum += adc_read();
    }
    return (sum + num_samples / 2) / num_samples;  // rounded
}

// -----------------------------------------------------------------------------
// Histogram collection - sweep PWM through full range
// -----------------------------------------------------------------------------
void collect_histogram_sweep(bool ascending, size_t settleUS) {
    Serial.printf("  Sweep %s...\n", ascending ? "UP" : "DOWN");
    
    int start = ascending ? 0 : PWM_WRAP;
    int end = ascending ? PWM_WRAP : 0;
    int step = ascending ? 1 : -1;  
    
    for (int pwm_level = start; ascending ? (pwm_level <= end) : (pwm_level >= end); pwm_level += step) {
        pwm_set_gpio_level(CAL_PWM_PIN, pwm_level);
        sleep_us(settleUS);
        
        // Collect samples at this level
        for (int s = 0; s < SAMPLES_PER_LEVEL; s++) {
            uint16_t code = adc_read();
            if (code < 4096) {
                histogram[code]++;
            }
            sleep_us(100);
        }
        if (pwm_level % 100 == 0) {
            Serial.printf("%d,", pwm_level);
        }
    }
    Serial.println();
}

void collect_histogram(size_t settleUS) {
    memset(histogram, 0, sizeof(histogram));
    
    Serial.printf("Collecting histogram (%d sweeps)...\n", NUM_SWEEPS);
    
    for (int sweep = 0; sweep < NUM_SWEEPS; sweep++) {
        collect_histogram_sweep(sweep % 2 == 0, settleUS);  // alternate up/down
        Serial.printf("  Sweep %d/%d complete\n", sweep + 1, NUM_SWEEPS);
    }
    
    // Return PWM to zero
    pwm_set_gpio_level(CAL_PWM_PIN, 0);
}

// -----------------------------------------------------------------------------
// Build correction LUT from histogram
// -----------------------------------------------------------------------------
void build_correction_lut(void) {
    Serial.printf("Building correction LUT...\n");
    
    // Find the actual range used (ignore empty bins at extremes)
    uint16_t code_min = 0, code_max = 4095;
    while (code_min < 4095 && histogram[code_min] == 0) code_min++;
    while (code_max > 0 && histogram[code_max] == 0) code_max--;
    
    cal_data.adc_min = code_min;
    cal_data.adc_max = code_max;
    
    Serial.printf("  ADC range: %d to %d\n", code_min, code_max);
    
    // Calculate total samples in valid range
    uint64_t total_samples = 0;
    for (int i = code_min; i <= code_max; i++) {
        total_samples += histogram[i];
    }
    
    // Expected samples per code for a perfectly linear ADC
    double expected_per_code = (double)total_samples / (code_max - code_min + 1);
    
    Serial.printf("  Total samples: %llu, expected/code: %.1f\n", total_samples, expected_per_code);
    
    // Build cumulative distribution and derive correction
    // Key insight: cumulative count at code i tells us what "ideal" code should be
    double cumulative = 0;
    
    for (int i = 0; i < 4096; i++) {
        if (i < code_min) {
            // Below range - no correction
            cal_data.correction[i] = cal_data.correction[code_min];
        } else if (i > code_max) {
            // Above range - use last valid correction
            cal_data.correction[i] = cal_data.correction[code_max];
        } else {
            // Add half this bin's count (center of bin)
            cumulative += histogram[i] / 2.0;
            
            // Ideal code based on cumulative distribution
            double ideal_code = code_min + (cumulative / expected_per_code);
            
            // Correction is the difference
            cal_data.correction[i] = (int16_t)round(ideal_code) - i;
            
            // Add second half of bin for next iteration
            cumulative += histogram[i] / 2.0;
        }
        Serial.printf("Code: %d, val: %d\n", i, histogram[i]);
    }

    // Validate corrections aren't crazy (should be within ±16 LSBs typically)
    int16_t max_corr = 0, min_corr = 0;
    for (int i = code_min; i <= code_max; i++) {
        if (cal_data.correction[i] > max_corr) max_corr = cal_data.correction[i];
        if (cal_data.correction[i] < min_corr) min_corr = cal_data.correction[i];
    }
    Serial.printf("  Correction range: %d to %+d LSBs\n", min_corr, max_corr);
    
    cal_data.magic = 0xCA11B8ED;
}

// -----------------------------------------------------------------------------
// Flash storage
// -----------------------------------------------------------------------------
// void save_calibration(void) {
//     Serial.printf("Saving calibration to flash...\n");
    
//     uint32_t ints = save_and_disable_interrupts();
    
//     // Erase sector (4KB minimum)
//     flash_range_erase(CAL_FLASH_OFFSET, 4096);
    
//     // Write calibration data
//     flash_range_program(CAL_FLASH_OFFSET, (uint8_t*)&cal_data, sizeof(cal_data));
    
//     restore_interrupts(ints);
    
//     Serial.printf("  Saved %d bytes at flash offset 0x%X\n", sizeof(cal_data), CAL_FLASH_OFFSET);
// }

// bool load_calibration(void) {
//     const CalibrationData* flash_cal = (const CalibrationData*)(XIP_BASE + CAL_FLASH_OFFSET);
    
//     if (flash_cal->magic == 0xCA11B8ED) {
//         memcpy(&cal_data, flash_cal, sizeof(cal_data));
//         Serial.printf("Calibration loaded from flash (range %d-%d)\n", 
//                cal_data.adc_min, cal_data.adc_max);
//         return true;
//     }
    
//     Serial.printf("No valid calibration in flash\n");
//     return false;
// }

// -----------------------------------------------------------------------------
// Apply correction
// -----------------------------------------------------------------------------
static inline uint16_t adc_corrected(uint16_t raw) {
    int32_t corrected = raw + cal_data.correction[raw];
    if (corrected < 0) corrected = 0;
    if (corrected > 4095) corrected = 4095;
    return corrected;
}

// Full pipeline: oversample + correct
uint16_t adc_read_calibrated(uint32_t oversample) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < oversample; i++) {
        uint16_t raw = adc_read();
        sum += adc_corrected(raw);
    }
    return sum / oversample;
}

// -----------------------------------------------------------------------------
// Run calibration
// -----------------------------------------------------------------------------
void run_calibration(size_t settleUS) {
    Serial.printf("\n=== ADC Calibration ===\n");
    Serial.printf("Ensure PWM pin %d is connected to ADC pin %d via RC filter\n", 
           CAL_PWM_PIN, CAL_ADC_PIN);
    Serial.printf("Recommended: 1k series + 1µF to ground\n\n");
    
    cal_pwm_init();
    cal_adc_init();
    
    sleep_ms(100);
    
    collect_histogram(settleUS);
    build_correction_lut();
    // save_calibration();
    save_histogram_to_file();  
    dump_csv_to_serial();  

    
    Serial.printf("\n=== Calibration complete ===\n");
}

// -----------------------------------------------------------------------------
// Verification - test a few points
// -----------------------------------------------------------------------------
void verify_calibration(size_t settleUS) {
    Serial.printf("\n=== Verification ===\n");
    
    cal_pwm_init();
    cal_adc_init();
    
    // Test at 10%, 25%, 50%, 75%, 90% of range
    const int test_points[] = {10, 20, 25,25, 50,60, 75,83, 90};
    
    for (int i = 0; i < 8; i++) {
        uint32_t pwm_level = (test_points[i] * PWM_WRAP) / 100;
        uint32_t expected_code = (test_points[i] * 4095) / 100;
        
        pwm_set_gpio_level(CAL_PWM_PIN, pwm_level);
        sleep_us(settleUS * 500);
        
        uint16_t raw = adc_read_averaged(256);
        uint16_t corrected = adc_corrected(raw);
        
        int32_t raw_error = (int32_t)raw - expected_code;
        int32_t corr_error = (int32_t)corrected - expected_code;
        
        Serial.printf("  %2d%%: expected=%4d, raw=%4d (err %+3d), corrected=%4d (err %+3d)\n",
               test_points[i], expected_code, raw, raw_error, corrected, corr_error);
    }
    
    pwm_set_gpio_level(CAL_PWM_PIN, 0);
}

    
void measure_step_response(void) {
    Serial.printf("\n=== Step Response Test ===\n");
    
    cal_pwm_init();
    cal_adc_init();
    
    // Start at 0
    pwm_set_gpio_level(CAL_PWM_PIN, 0);
    sleep_ms(100);
    uint16_t baseline = adc_read_averaged(64);
    
    // Step to 50%
    uint32_t step_target = PWM_WRAP / 2;
    pwm_set_gpio_level(CAL_PWM_PIN, step_target);
    
    // Sample rapidly after step
    Serial.printf("time_us,adc_code\n");
    uint32_t start = time_us_32();
    for (int i = 0; i < 200; i++) {
        uint32_t t = time_us_32() - start;
        uint16_t code = adc_read();
        Serial.printf("%lu,%d\n", t, code);
        sleep_us(10);
    }
    
    uint16_t final_val = adc_read_averaged(64);
    Serial.printf("\nBaseline: %d, Final: %d, Delta: %d codes\n", 
                  baseline, final_val, final_val - baseline);
    
    pwm_set_gpio_level(CAL_PWM_PIN, 0);
}
                            
void setup() {
  Serial.begin();
  while (!Serial) {}

  Serial.printf("\nRP2040 ADC Calibration\n");
  Serial.printf("==================================\n");
    
    // for (auto &settle : {500}) {
    //     Serial.printf("Settle time: %d\n", settle);
    //     // load_calibration_from_file();
    //     run_calibration(settle);
    //     // // }
        
    //     verify_calibration(settle);
    //     verify_calibration(settle);
    //     verify_calibration(settle);
    //     verify_calibration(settle>>1);
    //     verify_calibration(settle>>2);
    // }
    measure_step_response();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
