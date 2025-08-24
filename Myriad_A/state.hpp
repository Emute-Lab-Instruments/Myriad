#ifndef MYRIAD_A_STATE_HPP
#define MYRIAD_A_STATE_HPP

#include "hardware/flash.h"
#include "hardware/sync.h"

#define STATE_MEM __not_in_flash("state")

class MyriadState {
public:
    // Binary format structure - aligned to flash page requirements (256 bytes)
    struct __attribute__((packed, aligned(4))) StateBinary {
        uint32_t magic;           // Magic number for validation
        uint16_t version;         // Format version
        uint16_t sequence;        // Sequence number for wear leveling
        uint32_t writeCount;      // Total number of writes to this sector
        size_t oscBanks[3];       // Oscillator banks
        size_t metaMod;           // Meta modulation
        float metaModDepth;       // Meta mod depth
        float metaModSpeed;       // Meta mod speed
        uint32_t checksum;        // Simple checksum
        uint8_t padding[256 - 48]; // Pad to exactly 256 bytes (FLASH_PAGE_SIZE)
    };

    // State snapshot for batch operations
    struct StateSnapshot {
        size_t oscBanks[3];
        size_t metaMod;
        float metaModDepth;
        float metaModSpeed;
    };

    // Initialize with defaults
    static void __not_in_flash_func(load)() {
        oscBanks[0] = 0;
        oscBanks[1] = 0;
        oscBanks[2] = 0;
        metaMod = 0;
        metaModDepth = 0.0f;
        metaModSpeed = 0.0f;
        changeFlag = false;
        currentSequence = 0;
        totalWrites = 0;
    }

    // Hot path setters - inline and optimized
    static inline void setOscBank(const size_t bankIdx, const size_t modelIdx) {
        if (bankIdx < 3 && oscBanks[bankIdx] != modelIdx) {
            oscBanks[bankIdx] = modelIdx;
            changeFlag = true;
        }
    }

    static inline void setMetaMod(const size_t value) {
        if (metaMod != value) {
            metaMod = value;
            changeFlag = true;
        }
    }

    static inline void setMetaModDepth(const float value) {
        if (metaModDepth != value) {
            metaModDepth = value;
            changeFlag = true;
        }
    }

    static inline void setMetaModSpeed(const float value) {
        if (metaModSpeed != value) {
            metaModSpeed = value;
            changeFlag = true;
        }
    }

    static inline void clearChangeFlag() {
        changeFlag = false;
    }

    // Template versions for compile-time bounds checking
    template<size_t BANK_IDX>
    static inline void setOscBank(const size_t modelIdx) {
        static_assert(BANK_IDX < 3, "Invalid bank index");
        if (oscBanks[BANK_IDX] != modelIdx) {
            oscBanks[BANK_IDX] = modelIdx;
            changeFlag = true;
        }
    }

    template<size_t BANK_IDX>
    static inline size_t getOscBank() {
        static_assert(BANK_IDX < 3, "Invalid bank index");
        return oscBanks[BANK_IDX];
    }

    // Hot path getters - inline for speed
    static inline size_t getOscBank(const size_t bankIdx) {
        return (bankIdx < 3) ? oscBanks[bankIdx] : 0;
    }

    static inline size_t getMetaMod() { return metaMod; }
    static inline float getMetaModDepth() { return metaModDepth; }
    static inline float getMetaModSpeed() { return metaModSpeed; }
    static inline bool getChangeFlag() { return changeFlag; }

    // Debug/monitoring functions
    static inline uint32_t getTotalWrites() { return totalWrites; }
    static inline uint16_t getCurrentSequence() { return currentSequence; }
    static inline uint16_t getCurrentSector() { return currentSector; }

    // Batch operations for efficiency
    static inline void loadSnapshot(const StateSnapshot& snapshot) {
        oscBanks[0] = snapshot.oscBanks[0];
        oscBanks[1] = snapshot.oscBanks[1];
        oscBanks[2] = snapshot.oscBanks[2];
        metaMod = snapshot.metaMod;
        metaModDepth = snapshot.metaModDepth;
        metaModSpeed = snapshot.metaModSpeed;
        changeFlag = true;
    }

    static inline StateSnapshot getSnapshot() {
        return StateSnapshot{
            {oscBanks[0], oscBanks[1], oscBanks[2]},
            metaMod,
            metaModDepth,
            metaModSpeed
        };
    }

    // Raw flash persistence with wear leveling
    static bool __not_in_flash_func(loadFromFlash)() {
        uint16_t latestSequence = 0;
        uint16_t latestSector = 0;
        bool foundValid = false;

        // Scan all sectors to find the one with highest sequence number
        for (uint16_t sector = 0; sector < NUM_SECTORS; sector++) {
            const StateBinary* stored = getSectorAddress(sector);
            
            // Check if sector contains valid data
            if (stored->magic == MAGIC_NUMBER && 
                stored->version == FORMAT_VERSION &&
                validateChecksum(*stored)) {
                
                // Check if this is the latest version
                if (!foundValid || isSequenceNewer(stored->sequence, latestSequence)) {
                    latestSequence = stored->sequence;
                    latestSector = sector;
                    foundValid = true;
                }
            }
        }

        if (!foundValid) {
            return false;
        }

        // Load from the latest valid sector
        const StateBinary* latest = getSectorAddress(latestSector);
        oscBanks[0] = latest->oscBanks[0];
        oscBanks[1] = latest->oscBanks[1];
        oscBanks[2] = latest->oscBanks[2];
        metaMod = latest->metaMod;
        metaModDepth = latest->metaModDepth;
        metaModSpeed = latest->metaModSpeed;
        
        // Update wear leveling state
        currentSequence = latestSequence;
        currentSector = latestSector;
        totalWrites = latest->writeCount;
        changeFlag = false;

        return true;
    }

    // Non-blocking save check
    static inline bool canSaveNow() {
        return !saveInProgress && changeFlag;
    }

    // Fast save with wear leveling - respects RP2040 flash page/sector requirements
    static void __not_in_flash_func(saveIfChanged)() {
        if (!changeFlag || saveInProgress) return;

        saveInProgress = true;
        
        // Move to next sector for wear leveling
        uint16_t targetSector = (currentSector + 1) % NUM_SECTORS;
        
        // Increment sequence number (with rollover handling)
        uint16_t newSequence = currentSequence + 1;
        if (newSequence == 0) newSequence = 1; // Skip 0 as it indicates uninitialized
        
        // Prepare data structure in RAM - exactly 256 bytes (one page)
        binary.magic = MAGIC_NUMBER;
        binary.version = FORMAT_VERSION;
        binary.sequence = newSequence;
        binary.writeCount = totalWrites + 1;
        binary.oscBanks[0] = oscBanks[0];
        binary.oscBanks[1] = oscBanks[1];
        binary.oscBanks[2] = oscBanks[2];
        binary.metaMod = metaMod;
        binary.metaModDepth = metaModDepth;
        binary.metaModSpeed = metaModSpeed;
        binary.checksum = 0;
        
        // Clear padding to ensure clean data
        memset(binary.padding, 0, sizeof(binary.padding));

        // Calculate checksum
        binary.checksum = calculateChecksum(binary);

        // Validate sector bounds
        if (targetSector >= NUM_SECTORS) {
            saveInProgress = false;
            return;
        }

        // Calculate sector offset - must be sector-aligned (4096 bytes)
        uint32_t sectorOffset = STORAGE_START_OFFSET + (targetSector * FLASH_SECTOR_SIZE);
        
        // Bounds check
        if (sectorOffset + FLASH_SECTOR_SIZE > STORAGE_END_OFFSET) {
            saveInProgress = false;
            return;
        }

        // Critical section: disable interrupts for flash operation
        uint32_t interrupts = save_and_disable_interrupts();
        
        //Step 1: Erase entire sector (4096 bytes) - this is required before any programming
        flash_range_erase(sectorOffset, FLASH_SECTOR_SIZE);
        
        // // Step 2: Program one page (256 bytes) at the start of the sector
        // // Data size must be exactly 256 bytes and page-aligned
        // flash_range_program(sectorOffset, 
        //                    reinterpret_cast<const uint8_t*>(&binary), 
        //                    FLASH_PAGE_SIZE);
        
        restore_interrupts(interrupts);

        Serial.println("sectorOffset: " + String(sectorOffset));
        
        // Update state only if flash operations succeeded
        currentSector = targetSector;
        currentSequence = newSequence;
        totalWrites++;
        changeFlag = false;
        saveInProgress = false;
    }

    // Maintenance function - call occasionally to check wear
    static void __not_in_flash_func(printWearInfo)() {
        Serial.print("Total writes: ");
        Serial.print(totalWrites);
        Serial.print(", Current sector: ");
        Serial.print(currentSector);
        Serial.print(", Writes per sector: ~");
        Serial.println(totalWrites / NUM_SECTORS);
    }

private:
    // Flash storage configuration - using 12MB (2MB-14MB) for wear leveling
    // Use Pico SDK constants: FLASH_PAGE_SIZE (256) and FLASH_SECTOR_SIZE (4096)
    static const uint32_t STORAGE_START_OFFSET = 2 * 1024 * 1024;           // 2MB (after code)
    static const uint32_t STORAGE_END_OFFSET = 14 * 1024 * 1024;            // 14MB (before LittleFS)
    static const uint32_t STORAGE_SIZE = STORAGE_END_OFFSET - STORAGE_START_OFFSET;  // 12MB
    static const uint32_t NUM_SECTORS = STORAGE_SIZE / FLASH_SECTOR_SIZE;             // 3072 sectors
    static const uintptr_t FLASH_STORAGE_BASE = XIP_BASE + STORAGE_START_OFFSET;
    static const uint32_t MAGIC_NUMBER = 0x4D59524E;  // "MYRN"
    static const uint16_t FORMAT_VERSION = 1;

    // Memory layout optimized for cache efficiency
    static size_t STATE_MEM oscBanks[3];        // Hot data
    static volatile bool STATE_MEM changeFlag;           // Hot data
    static volatile bool STATE_MEM saveInProgress;       // Prevent concurrent saves
    static uint16_t STATE_MEM currentSector;    // Current active sector (16-bit for 3072 sectors)
    static uint16_t STATE_MEM currentSequence;  // Current sequence number
    static uint32_t STATE_MEM totalWrites;      // Total write operations
    static size_t STATE_MEM metaMod;            // Warm data
    static float STATE_MEM metaModDepth;        // Cold data
    static float STATE_MEM metaModSpeed;        // Cold data
    static STATE_MEM StateBinary binary; 


    // Helper functions
    static inline const StateBinary* getSectorAddress(uint16_t sector) {
        return reinterpret_cast<const StateBinary*>(
            FLASH_STORAGE_BASE + (sector * FLASH_SECTOR_SIZE)
        );
    }

    // Handle sequence number rollover
    static inline bool isSequenceNewer(uint16_t seq1, uint16_t seq2) {
        // Handle wraparound by checking if difference is reasonable
        uint16_t diff = seq1 - seq2;
        return diff < 32768;  // Assume sequences don't differ by more than 32K
    }

    // Simple checksum calculation
    static uint32_t __not_in_flash_func(calculateChecksum)(const StateBinary& binary) {
        uint32_t checksum = 0;
        const uint32_t* data = reinterpret_cast<const uint32_t*>(&binary);
        const size_t words = (offsetof(StateBinary, checksum)) / sizeof(uint32_t);
        
        for (size_t i = 0; i < words; i++) {
            checksum ^= data[i];
        }
        return checksum;
    }

    static bool __not_in_flash_func(validateChecksum)(const StateBinary& binary) {
        StateBinary temp = binary;
        temp.checksum = 0;
        return calculateChecksum(temp) == binary.checksum;
    }
};

// Static member definitions
size_t MyriadState::oscBanks[3];
volatile bool MyriadState::changeFlag;
volatile bool MyriadState::saveInProgress;
uint16_t MyriadState::currentSector;
uint16_t MyriadState::currentSequence;
uint32_t MyriadState::totalWrites;
size_t MyriadState::metaMod;
float MyriadState::metaModDepth;
float MyriadState::metaModSpeed;
MyriadState::StateBinary MyriadState::binary; 


#endif // MYRIAD_A_STATE_HPP