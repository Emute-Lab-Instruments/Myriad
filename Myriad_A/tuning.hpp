#ifndef MYRIAD_A_TUNING_HPP
#define MYRIAD_A_TUNING_HPP

#define TUNING_MEM __scratch_x("tuningdata")

#include "clockfreq.h"


namespace TuningSettings {
    constexpr float wavelen20hz = sampleClock/20.f;
    constexpr float freqC0 = 8.1759375f;
    constexpr float freqC1 = freqC0*2.f;
    constexpr float wavelenC1 = sampleClock/freqC1; //start from C1
    constexpr float wavelenC1Inv = 1.f/freqC1; //start from C1
    constexpr float wavelenC2 = wavelenC1 * 0.5f;

    static TUNING_MEM int octaves=0;
    static TUNING_MEM int semitones=0;
    static TUNING_MEM int cents=0;
    static TUNING_MEM float adjustment = 0.0f;

    //base frequency
    float TUNING_MEM baseFrequency = freqC1 * powf(2,TuningSettings::adjustment);
    float TUNING_MEM baseWavelen = sampleClock /baseFrequency;
    float TUNING_MEM baseWavelenInv = 1.f/baseWavelen;

    float TUNING_MEM quantNotesPerOct = 12.f;
    float TUNING_MEM quantPull = 0.f;

    
    float TUNING_MEM quantStep = 1.0f / quantNotesPerOct;    
    float TUNING_MEM quantStepInv = 1.f/quantStep;
    float TUNING_MEM quantAlpha = quantPull * 0.01f;

    bool TUNING_MEM bypass = false;

    
    const char* TUNING_FILE = "/tuning.json";

    /**
     * @brief Update the tuning settings based on the current values.
     * This method recalculates the adjustment value based on octaves, semitones, and cents.
     */
    void update() {
        TuningSettings::adjustment = ((TuningSettings::octaves) + (TuningSettings::semitones * 1.f/12.f) + (TuningSettings::cents * 1.f/1200.f)); // 10 octaves
        baseFrequency = freqC1 * powf(2,TuningSettings::adjustment);
        baseWavelen = sampleClock  / baseFrequency;
        baseWavelenInv = 1.f/baseWavelen;

    }

    void updateQuant() {
        quantStep = 1.f / quantNotesPerOct;    
        quantStepInv = 1.f/quantStep;
        quantAlpha = quantPull * 0.01f;
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
        quantNotesPerOct = doc["quantNotesPerOct"] | quantNotesPerOct;
        quantPull = doc["quantPull"] | quantPull;
        bypass = doc["bypass"] | bypass;

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

        // Create JSON document
        JsonDocument doc;
        
        // Set values
        doc["octaves"] = octaves;
        doc["semitones"] = semitones;
        doc["cents"] = cents;
        doc["quantNotesPerOct"] = quantNotesPerOct;
        doc["quantPull"] = quantPull;
        doc["bypass"] = bypass;

        // Open file for writing
        File file = LittleFS.open(TUNING_FILE, "w");
        if (!file) {
            Serial.println("Failed to create tuning file");
            return;
        }

        // Serialize JSON to file
        size_t bytesWritten = serializeJson(doc, file);
        file.close();

        if (bytesWritten > 0) {
            Serial.print("Tuning saved successfully (");
            Serial.print(bytesWritten);
            Serial.println(" bytes)");
        } else {
            Serial.println("Failed to write tuning file");
        }
    }    
};

#endif // MYRIAD_A_TUNING_HPP