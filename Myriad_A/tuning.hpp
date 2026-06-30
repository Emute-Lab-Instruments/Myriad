#ifndef MYRIAD_A_TUNING_HPP
#define MYRIAD_A_TUNING_HPP

#define TUNING_MEM __scratch_x("tuningdata")

#include "clockfreq.h"

#include "fixedpoint.hpp"
#include "file_utils.hpp"
using namespace FixedPoint;
using WvlenFPType = Fixed<20,11>;

namespace TuningSettings {
    constexpr float wavelen20hz = sampleClock/20.f;
    constexpr float freqC0 = 8.1759375f;
    constexpr float freqC1 = freqC0*2.f;
    constexpr float wavelenC1 = sampleClock/freqC1; //start from C1
    constexpr float wavelenC1Inv = 1.f/freqC1; //inverse for fast calculations
    // constexpr float wavelenC2 = wavelenC1 * 0.5f;

    static TUNING_MEM WvlenFPType wavelenC1Fixed = WvlenFPType(wavelenC1);
    static TUNING_MEM  WvlenFPType wavelenC1InvFixed(wavelenC1Inv); 

    static TUNING_MEM WvlenFPType wavelenCMinus1Fixed = WvlenFPType(wavelenC1 * 8);


    static TUNING_MEM int octaves=0;
    static TUNING_MEM int semitones=0;
    static TUNING_MEM int cents=0;
    static TUNING_MEM float adjustment = 0.0f;
    static TUNING_MEM float adjustmentPow2=1.f;
    static TUNING_MEM Q16_16 adjustmentPow2InvFP(1.f);
    static TUNING_MEM Q16_16 maxWaveLenScaleFP(1.f);

    //base frequency
    float TUNING_MEM baseFrequency = freqC1 * powf(2,TuningSettings::adjustment);
    float TUNING_MEM baseWavelen = sampleClock /baseFrequency;
    float TUNING_MEM baseWavelenInv = 1.f/baseWavelen;

    WvlenFPType TUNING_MEM baseFrequencyFP = WvlenFPType(baseFrequency);
    WvlenFPType TUNING_MEM baseWavelenFP = WvlenFPType(sampleClockFP.divWith(baseFrequencyFP));
    WvlenFPType TUNING_MEM baseWavelenInvFP = WvlenFPType(1)/baseWavelenFP;

    Q16_16 TUNING_MEM quantNotesPerOct = Q16_16(12);
    Q16_16 TUNING_MEM quantPull = Q16_16(0);

    
    Q16_16 TUNING_MEM quantStep = Q16_16(1) / quantNotesPerOct;    
    Q16_16 TUNING_MEM quantStepInv = Q16_16(1)/quantStep;
    Q16_16 TUNING_MEM quantAlpha = quantPull * Q16_16(0.01f);

    bool TUNING_MEM bypass = false;

    
    const char* TUNING_FILE = "/tune.json";

    /**
     * @brief Update the tuning settings based on the current values.
     * This method recalculates the adjustment value based on octaves, semitones, and cents.
     */
    void update() {
        TuningSettings::adjustment = ((TuningSettings::octaves) + (TuningSettings::semitones * 1.f/12.f) + (TuningSettings::cents * 1.f/1200.f)); // 10 octaves
        TuningSettings::adjustmentPow2 = powf(2,TuningSettings::adjustment);
        TuningSettings::adjustmentPow2InvFP = Q16_16(1.0f / TuningSettings::adjustmentPow2);
        // Pre-multiply clamp: wavelenScale is clamped here, then multiplied by adjustmentPow2InvFP.
        // Limit must be 8 * adjustmentPow2 so that after the multiply the effective scale stays ≤ 8
        // (= 3 octaves below C1), keeping the final WvlenFPType mulWith within int32_t range.
        // This also stays well within Q16_16 range (max 8 * 2^4 = 128 at +4 oct).
        TuningSettings::maxWaveLenScaleFP = Q16_16(8.f * TuningSettings::adjustmentPow2);
        baseFrequency = freqC1 * TuningSettings::adjustmentPow2;
        baseFrequencyFP = WvlenFPType(baseFrequency);
        baseWavelen = sampleClock  / baseFrequency;
        baseWavelenInv = 1.f/baseWavelen;
        // Pin fixed-point base to C1 so it never overflows WvlenFPType.
        // The tuning offset is applied per-sample in the audio loop via adjustmentPow2InvFP.
        baseWavelenFP = wavelenC1Fixed;
        baseWavelenInvFP = wavelenC1InvFixed;

    }

    void updateQuant() {
        quantStep = Q16_16(1) / quantNotesPerOct;    
        quantStepInv = Q16_16(1)/quantStep;
        quantAlpha = quantPull * Q16_16(0.01f);
    }
    
    static bool load() {
        if (!LittleFS.begin()) {
            Serial.println("Failed to mount LittleFS");
            return false;
        }

        if (!LittleFS.exists(TUNING_FILE)) {
            Serial.println("Tuning file not found, using defaults");
            return false;
        }

        File file = LittleFS.open(TUNING_FILE, "r");
        if (!file) {
            Serial.println("Failed to open tuning file");
            return false;
        }

        // Create JSON document
        JsonDocument doc;
        
        // Deserialize the JSON document
        DeserializationError error = deserializeJson(doc, file);
        
        file.close();

        if (error) {
            Serial.print("Failed to parse tuning file: ");
            Serial.println(error.c_str());
            return false;
        }

        // Load values from JSON, using defaults if keys don't exist
        octaves = doc["octaves"] | octaves;
        semitones = doc["semitones"] | semitones;
        cents = doc["cents"] | cents;
        quantNotesPerOct = Q16_16::from_raw(doc["quantNotesPerOct"]) | quantNotesPerOct;
        quantPull = Q16_16::from_raw(doc["quantPull"]) | quantPull;
        bypass = doc["bypass"] | bypass;

        // Bounds validation — clamp to sane musical ranges
        octaves  = constrain(octaves,  -4, 4);
        semitones = constrain(semitones, -12, 12);
        cents    = constrain(cents,    -100, 100);
        if (quantNotesPerOct < Q16_16(1) || quantNotesPerOct > Q16_16(24))
            quantNotesPerOct = Q16_16(12);
        if (quantPull < Q16_16(0) || quantPull > Q16_16(1))
            quantPull = Q16_16(0);

        Serial.println("Tuning loaded successfully");
        update();
        updateQuant();
        return true;
    }

    void save() {
        if (!LittleFS.begin()) {
            Serial.println("Failed to mount LittleFS");
            return;
        }

        JsonDocument doc;
        doc["octaves"] = octaves;
        doc["semitones"] = semitones;
        doc["cents"] = cents;
        doc["quantNotesPerOct"] = quantNotesPerOct.raw();
        doc["quantPull"] = quantPull.raw();
        doc["bypass"] = bypass;

        // Serialise to a String buffer first, then write atomically
        String buf;
        size_t bytesWritten = serializeJson(doc, buf);
        if (bytesWritten == 0) {
            Serial.println("Failed to serialise tuning");
            return;
        }

        if (FileUtils::atomicWrite(TUNING_FILE,
                                   reinterpret_cast<const uint8_t*>(buf.c_str()),
                                   buf.length())) {
            Serial.print("Tuning saved (");
            Serial.print(bytesWritten);
            Serial.println(" bytes)");
        } else {
            Serial.println("Failed to write tuning file");
        }
    }    
};

#endif // MYRIAD_A_TUNING_HPP