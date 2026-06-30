#ifndef FILE_UTILS_HPP
#define FILE_UTILS_HPP

#include <LittleFS.h>

namespace FileUtils {

// Bit-by-bit CRC32 (ISO 3309 / zlib polynomial). Suitable for small payloads.
inline uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFF;
}

// Write buf to path atomically via a .tmp rename. Assumes LittleFS is already mounted.
// Returns true on success; the original file is untouched if the write fails.
inline bool atomicWrite(const char* path, const uint8_t* buf, size_t len) {
    String tmp = String(path) + ".tmp";
    File f = LittleFS.open(tmp.c_str(), "w");
    if (!f) return false;
    size_t written = f.write(buf, len);
    f.close();
    if (written != len) {
        LittleFS.remove(tmp.c_str());
        return false;
    }
    LittleFS.remove(path);
    return LittleFS.rename(tmp.c_str(), path);
}

} // namespace FileUtils

#endif // FILE_UTILS_HPP
