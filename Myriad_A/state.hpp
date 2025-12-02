#ifndef MYRIAD_A_STATE_HPP
#define MYRIAD_A_STATE_HPP

#define STATE_MEM __not_in_flash("state")

#include "metaOscs.hpp"
#include "fixedpoint.hpp"

class MyriadState {
public:
    // State snapshot for batch operations
    struct StateSnapshot {
        size_t oscBanks[3];
        size_t metaMod;
        Q16_16 metaModDepth;
        Q16_16 metaModSpeed;
        MODTARGETS modTarget;
    };

    // Initialize with defaults
    static void __not_in_flash_func(load)() {
        MyriadState::oscBanks[0] = 0;
        MyriadState::oscBanks[1] = 0;
        MyriadState::oscBanks[2] = 0;
        MyriadState::metaMod = 0;
        MyriadState::metaModDepth = Q16_16(0.0f);
        MyriadState::metaModSpeed = Q16_16(0.0f);
        MyriadState::modTarget = MODTARGETS::PITCH;
        MyriadState::changeFlag = false;
        MyriadState::loadFromFlash();
    }

    // Inline setters
    static inline void __not_in_flash_func(setOscBank)(const size_t bankIdx, const size_t modelIdx) {
        if (bankIdx < 3) {
            MyriadState::oscBanks[bankIdx] = modelIdx;
        }
        MyriadState::changeFlag = true;
    }

    static inline void __not_in_flash_func(setMetaMod)(const size_t value) {
        MyriadState::metaMod = value;
        MyriadState::changeFlag = true;
    }

    static inline void __not_in_flash_func(setMetaModDepth)(const Q16_16 value) {
        MyriadState::metaModDepth = value;
        MyriadState::changeFlag = true;
    }

    static inline void __not_in_flash_func(setMetaModSpeed)(const Q16_16 value) {
        MyriadState::metaModSpeed = value;
        MyriadState::changeFlag = true;
    }

    static inline void __not_in_flash_func(setModTarget)(const MODTARGETS value) {
        MyriadState::modTarget = value;
        MyriadState::changeFlag = true;
    }

    static inline void __not_in_flash_func(clearChangeFlag)() {
        MyriadState::changeFlag = false;
    }

    static size_t __not_in_flash_func(getOscBank)(const size_t bankIdx) {
        if (bankIdx < 3) {
            return MyriadState::oscBanks[bankIdx];
        }
        return 0;
    }

    static size_t __not_in_flash_func(getMetaMod)() {
        return MyriadState::metaMod;
    }

    static Q16_16 __not_in_flash_func(getMetaModDepth)() {
        return MyriadState::metaModDepth;
    }

    static Q16_16 __not_in_flash_func(getMetaModSpeed)() {
        return MyriadState::metaModSpeed;
    }

    static MODTARGETS __not_in_flash_func(getModTarget)() {
        return MyriadState::modTarget;
    }

    static inline bool __not_in_flash_func(getChangeFlag)() {
        return MyriadState::changeFlag;
    }

    // Binary flash persistence - much faster than JSON
    static bool __not_in_flash_func(loadFromFlash)() {
        if (!LittleFS.begin()) {
            Serial.println("Failed to mount LittleFS");
            return false;
        }

        if (!LittleFS.exists(STATE_FILE)) {
            Serial.println("State file not found, using defaults");
            return false;
        }

        File file = LittleFS.open(STATE_FILE, "r");
        if (!file) {
            Serial.println("Failed to open state file");
            return false;
        }

        // Check file size
        if (file.size() != sizeof(StateBinary)) {
            file.close();
            return false;
        }

        StateBinary binary;
        size_t bytesRead = file.read(reinterpret_cast<uint8_t*>(&binary), sizeof(StateBinary));
        file.close();

        if (bytesRead != sizeof(StateBinary)) return false;

        // Validate magic number and version
        if (binary.magic != MAGIC_NUMBER || binary.version != FORMAT_VERSION) {
            return false;
        }

        // Verify checksum
        if (!validateChecksum(binary)) {
            return false;
        }

        // Load data atomically
        oscBanks[0] = binary.oscBank0;
        oscBanks[1] = binary.oscBank1;
        oscBanks[2] = binary.oscBank2;
        metaMod = binary.metaMod;
        metaModDepth = binary.metaModDepth;
        metaModSpeed = binary.metaModSpeed;
        modTarget = static_cast<MODTARGETS>(binary.modTarget);
        changeFlag = false;

        return true;
    }

    static void __not_in_flash_func(saveIfChanged)() {
        if (MyriadState::changeFlag) {
            MyriadState::saveToFlash();
            MyriadState::clearChangeFlag();
        }
    }

    static void __not_in_flash_func(save)() {
        MyriadState::saveToFlash();
    }

private:
    // Binary format structure 
    struct StateBinary {
        uint32_t magic;           // Magic number for validation
        uint16_t version;         // Format version
        size_t oscBank0;          // Individual oscillator banks
        size_t oscBank1;
        size_t oscBank2;
        size_t metaMod;           // Meta modulation
        Q16_16 metaModDepth;       // Meta mod depth
        Q16_16 metaModSpeed;       // Meta mod speed
        uint8_t modTarget;        // Modulation target (stored as uint8_t)
        uint32_t checksum;        // Simple checksum
    };

    // Constants
    static const uint32_t MAGIC_NUMBER = 0x4D59524E;  // "MYRN" in hex
    static const uint16_t FORMAT_VERSION = 1;

    // Memory layout optimized for cache efficiency
    static size_t STATE_MEM oscBanks[3];     // Hot data - accessed frequently
    static bool STATE_MEM changeFlag;        // Hot data - checked often
    static size_t STATE_MEM metaMod;         // Warm data
    static Q16_16 STATE_MEM metaModDepth;     // Cold data
    static Q16_16 STATE_MEM metaModSpeed;     // Cold data
    static MODTARGETS STATE_MEM modTarget;   // Cold data
    static const char* STATE_FILE;

    // Simple checksum calculation
    static uint32_t __not_in_flash_func(calculateChecksum)(const StateBinary& binary) {
        uint32_t checksum = 0;
        const uint32_t* data = reinterpret_cast<const uint32_t*>(&binary);
        const size_t words = (sizeof(StateBinary) - sizeof(uint32_t)) / sizeof(uint32_t);
        
        for (size_t i = 0; i < words; i++) {
            checksum ^= data[i];
        }
        return checksum;
    }

    static bool __not_in_flash_func(validateChecksum)(const StateBinary& binary) {
        StateBinary temp = binary;
        temp.checksum = 0;  // Zero out checksum field
        return calculateChecksum(temp) == binary.checksum;
    }

    static void __not_in_flash_func(saveToFlash)() {
        StateBinary binary = {
            .magic = MAGIC_NUMBER,
            .version = FORMAT_VERSION,
            .oscBank0 = oscBanks[0],
            .oscBank1 = oscBanks[1],
            .oscBank2 = oscBanks[2],
            .metaMod = metaMod,
            .metaModDepth = metaModDepth,
            .metaModSpeed = metaModSpeed,
            .modTarget = static_cast<uint8_t>(modTarget),
            .checksum = 0  // Will be calculated below
        };

        // Calculate and set checksum
        binary.checksum = calculateChecksum(binary);

        File file = LittleFS.open(STATE_FILE, "w");
        if (!file) return;

        size_t bytesWritten = file.write(reinterpret_cast<const uint8_t*>(&binary), sizeof(StateBinary));
        file.close();

        // Optional: verify write success
        if (bytesWritten == sizeof(StateBinary)) {
            // Success - could add logging here
        }
    }
};

// Static member definitions
size_t MyriadState::oscBanks[3];
bool MyriadState::changeFlag;
size_t MyriadState::metaMod;
Q16_16 MyriadState::metaModDepth;
Q16_16 MyriadState::metaModSpeed;
MODTARGETS MyriadState::modTarget;
const char* MyriadState::STATE_FILE = "/st.bin";

#endif // MYRIAD_A_STATE_HPP