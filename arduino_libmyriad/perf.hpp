#ifndef PERF_MEASURE_HPP
#define PERF_MEASURE_HPP

#include "pico/stdlib.h"
#include "hardware/timer.h"

// Configuration
#define PERF_ENABLED 1  // Set to 0 to completely disable (zero overhead)
#define PERF_WINDOW_SIZE 64  // Number of samples to average (must be power of 2)

// Performance measurement structure
struct PerfCounter {
    uint32_t accumulator;      // Running sum for mean calculation
    uint32_t sample_count;     // Total samples (wraps at PERF_WINDOW_SIZE)
    uint32_t mean_cycles;      // Rolling mean
    uint32_t last_cycles;      // Most recent measurement
    const char* name;
};

// Storage for performance counters (define in your .cpp file)
#define MAX_PERF_COUNTERS 16
extern PerfCounter perf_counters[MAX_PERF_COUNTERS];
extern uint8_t perf_counter_count;

// Initialize a performance counter
inline void perf_init_counter(PerfCounter* counter, const char* name) {
    counter->accumulator = 0;
    counter->sample_count = 0;
    counter->mean_cycles = 0;
    counter->last_cycles = 0;
    counter->name = name;
}

// Fast update using exponential moving average
inline void perf_update_stats(PerfCounter* counter, uint32_t cycles) {
    counter->last_cycles = cycles;
    counter->sample_count++;
    
    // Simple rolling average: accumulate until window full, then reset
    counter->accumulator += cycles;
    
    if ((counter->sample_count & (PERF_WINDOW_SIZE - 1)) == 0) {
        // Window complete - calculate mean and reset
        counter->mean_cycles = counter->accumulator >> __builtin_ctz(PERF_WINDOW_SIZE);
        counter->accumulator = 0;
    }
}

#if PERF_ENABLED

// Declare a performance counter
#define PERF_DECLARE(name) \
    static PerfCounter perf_##name __attribute__((section(".uninitialized_data"))); \
    static bool perf_##name##_initialized = false;

// Initialize counter (call once, e.g., in init())
#define PERF_INIT(name) \
    do { \
        if (!perf_##name##_initialized) { \
            perf_init_counter(&perf_##name, #name); \
            if (perf_counter_count < MAX_PERF_COUNTERS) { \
                perf_counters[perf_counter_count++] = perf_##name; \
            } \
            perf_##name##_initialized = true; \
        } \
    } while(0)

// Start timing a segment
#define PERF_BEGIN(name) \
    uint32_t perf_start_##name = time_us_32();

// End timing and update statistics
#define PERF_END(name) \
    do { \
        uint32_t perf_end_##name = time_us_32(); \
        uint32_t perf_cycles_##name = perf_end_##name - perf_start_##name; \
        perf_update_stats(&perf_##name, perf_cycles_##name); \
    } while(0)


// Get performance data (thread-safe read)
#define PERF_GET_MEAN(name) (perf_##name.mean_cycles)
#define PERF_GET_LAST(name) (perf_##name.last_cycles)
#define PERF_GET_COUNT(name) (perf_##name.sample_count)

#else  // PERF_ENABLED == 0

// No-op versions (zero overhead when disabled)
#define PERF_DECLARE(name)
#define PERF_INIT(name)
#define PERF_BEGIN(name)
#define PERF_END(name)
#define PERF_SCOPE(name)
#define PERF_GET_MEAN(name) (0)
#define PERF_GET_LAST(name) (0)
#define PERF_GET_COUNT(name) (0)

#endif // PERF_ENABLED

// // Print all counters (call from monitoring thread/process)
// inline void perf_print_all() {
//     printf("\n=== Performance Statistics (window=%d) ===\n", PERF_WINDOW_SIZE);
//     printf("%-20s %10s %10s %10s\n", "Name", "Mean(us)", "Last(us)", "Count");
//     printf("-------------------------------------------------------\n");
    
//     for (uint8_t i = 0; i < perf_counter_count; i++) {
//         PerfCounter* c = &perf_counters[i];
//         if (c->sample_count > 0) {
//             printf("%-20s %10u %10u %10u\n", 
//                    c->name, 
//                    c->mean_cycles,
//                    c->last_cycles,
//                    c->sample_count);
//         }
//     }
//     printf("=======================================================\n\n");
// }

// // Print single counter (lightweight)
// inline void perf_print(const char* name, PerfCounter* counter) {
//     printf("%s: mean=%uus last=%uus count=%u\n",
//            name, counter->mean_cycles, counter->last_cycles, counter->sample_count);
// }

// // Reset all counters
// inline void perf_reset_all() {
//     for (uint8_t i = 0; i < perf_counter_count; i++) {
//         perf_init_counter(&perf_counters[i], perf_counters[i].name);
//     }
// }

// // JSON output (lightweight)
// inline void perf_print_json() {
//     printf("{\"perf\":[");
//     for (uint8_t i = 0; i < perf_counter_count; i++) {
//         PerfCounter* c = &perf_counters[i];
//         printf("{\"n\":\"%s\",\"m\":%u,\"l\":%u,\"c\":%u}",
//                c->name, c->mean_cycles, c->last_cycles, c->sample_count);
//         if (i < perf_counter_count - 1) printf(",");
//     }
//     printf("]}\n");
// }

#endif // PERF_MEASURE_HPP