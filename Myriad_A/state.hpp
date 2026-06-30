#ifndef MYRIAD_A_STATE_HPP
#define MYRIAD_A_STATE_HPP

#define STATE_MEM __not_in_flash("state")

#include "metaOscs.hpp"
#include "fixedpoint.hpp"
#include "file_utils.hpp"

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

    // Initialize with defaults then attempt to load from flash
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

    // Try primary file first; on failure fall back to backup.
    // On successful primary load, refresh the backup.
    static bool __not_in_flash_func(loadFromFlash)() {
        if (!LittleFS.begin()) {
            Serial.println("Failed to mount LittleFS");
            return false;
        }

        if (loadFromPath(STATE_FILE)) {
            saveToPath(STATE_FILE_BAK);
            return true;
        }

        Serial.println("Primary state failed, trying backup");
        return loadFromPath(STATE_FILE_BAK);
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
    struct StateBinary {
        uint32_t magic;
        uint16_t version;
        size_t oscBank0;
        size_t oscBank1;
        size_t oscBank2;
        size_t metaMod;
        Q16_16 metaModDepth;
        Q16_16 metaModSpeed;
        uint8_t modTarget;
        uint32_t checksum;    // must remain the last field
    };

    static const uint32_t MAGIC_NUMBER  = 0x4D59524E;  // "MYRN"
    static const uint16_t FORMAT_VERSION = 2;           // v2: CRC32 checksum
    static const size_t   MAX_OSC_MODELS = 64;          // generous upper bound for bank index
    static const size_t   MAX_META_OSCS  = 16;          // generous upper bound for metaMod index

    static size_t STATE_MEM oscBanks[3];
    static bool STATE_MEM changeFlag;
    static size_t STATE_MEM metaMod;
    static Q16_16 STATE_MEM metaModDepth;
    static Q16_16 STATE_MEM metaModSpeed;
    static MODTARGETS STATE_MEM modTarget;
    static const char* STATE_FILE;
    static const char* STATE_FILE_BAK;

    // CRC32 over all bytes before the checksum field.
    // checksum must be the last field in StateBinary.
    static uint32_t __not_in_flash_func(calculateChecksum)(const StateBinary& binary) {
        return FileUtils::crc32(reinterpret_cast<const uint8_t*>(&binary),
                                sizeof(StateBinary) - sizeof(uint32_t));
    }

    static bool __not_in_flash_func(validateChecksum)(const StateBinary& binary) {
        return calculateChecksum(binary) == binary.checksum;
    }

    static bool loadFromPath(const char* path) {
        if (!LittleFS.exists(path)) return false;

        File file = LittleFS.open(path, "r");
        if (!file) return false;

        if (file.size() != sizeof(StateBinary)) {
            file.close();
            return false;
        }

        StateBinary binary;
        size_t bytesRead = file.read(reinterpret_cast<uint8_t*>(&binary), sizeof(StateBinary));
        file.close();

        if (bytesRead != sizeof(StateBinary)) return false;
        if (binary.magic != MAGIC_NUMBER || binary.version != FORMAT_VERSION) return false;
        if (!validateChecksum(binary)) return false;

        // Bounds validation — reject nonsense values that survived checksum
        if (binary.oscBank0 >= MAX_OSC_MODELS ||
            binary.oscBank1 >= MAX_OSC_MODELS ||
            binary.oscBank2 >= MAX_OSC_MODELS) return false;
        if (binary.metaMod >= MAX_META_OSCS) return false;
        if (binary.modTarget >= 3) return false;

        oscBanks[0]  = binary.oscBank0;
        oscBanks[1]  = binary.oscBank1;
        oscBanks[2]  = binary.oscBank2;
        metaMod      = binary.metaMod;
        metaModDepth = binary.metaModDepth;
        metaModSpeed = binary.metaModSpeed;
        modTarget    = static_cast<MODTARGETS>(binary.modTarget);
        changeFlag   = false;
        return true;
    }

    static void saveToPath(const char* path) {
        // Zero-initialise so padding bytes are deterministic for CRC32
        StateBinary binary = {};
        binary.magic       = MAGIC_NUMBER;
        binary.version     = FORMAT_VERSION;
        binary.oscBank0    = oscBanks[0];
        binary.oscBank1    = oscBanks[1];
        binary.oscBank2    = oscBanks[2];
        binary.metaMod     = metaMod;
        binary.metaModDepth = metaModDepth;
        binary.metaModSpeed = metaModSpeed;
        binary.modTarget   = static_cast<uint8_t>(modTarget);
        binary.checksum    = calculateChecksum(binary);

        FileUtils::atomicWrite(path,
                               reinterpret_cast<const uint8_t*>(&binary),
                               sizeof(binary));
    }

    static void __not_in_flash_func(saveToFlash)() {
        saveToPath(STATE_FILE);
    }
};

// Static member definitions
size_t MyriadState::oscBanks[3];
bool MyriadState::changeFlag;
size_t MyriadState::metaMod;
Q16_16 MyriadState::metaModDepth;
Q16_16 MyriadState::metaModSpeed;
MODTARGETS MyriadState::modTarget;
const char* MyriadState::STATE_FILE     = "/state.bin";
const char* MyriadState::STATE_FILE_BAK = "/state.bin.bak";

#endif // MYRIAD_A_STATE_HPP
