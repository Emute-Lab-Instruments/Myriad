#ifndef MYRIAD_A_TUNING_HPP
#define MYRIAD_A_TUNING_HPP

#define TUNING_MEM __not_in_flash("tuningdata")

namespace TuningSettings {
    static TUNING_MEM int octaves=0;
    static TUNING_MEM int semitones=0;
    static TUNING_MEM int cents=0;
    static TUNING_MEM float adjustment = 0.0f;
    
    const char* TUNING_FILE = "/tuning.json";

    /**
     * @brief Update the tuning settings based on the current values.
     * This method recalculates the adjustment value based on octaves, semitones, and cents.
     */
    void update() {
      TuningSettings::adjustment = ((TuningSettings::octaves * 0.1f) + (TuningSettings::semitones * 0.1f * 1/12.f) + (TuningSettings::cents * 0.0001f)) * 10.f; // 10 octaves
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

        Serial.println("Tuning loaded successfully");
        update();
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