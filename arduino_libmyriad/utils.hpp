#pragma once

#define PERIODIC_RUN(code, freq_ms) \
{ \
    static size_t lastUpdate = 0; \
    size_t now = millis(); \
    if (now - lastUpdate > (freq_ms)) { \
        lastUpdate = now; \
        code; \
    } \
}  

#define PERIODIC_RUN_US(code, freq_us) \
{ \
    static __not_in_flash("periodicupdate") size_t lastUpdate = 0; \
    size_t now = micros(); \
    if (now - lastUpdate > (freq_us)) { \
        lastUpdate = now; \
        code; \
    } \
}  