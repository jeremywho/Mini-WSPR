#include "wspr_sched.h"

int64_t wspr_even_minute_index(int64_t utc_ms) {
    int64_t minute = utc_ms / 60000;     // floor for >=0
    return minute / 2;                    // even-minute index (odd minutes share the prior even idx)
}
bool wspr_is_tx_slot(int64_t i, const wspr_duty_t* d) {
    if (!d || d->period <= 0) return true;
    int64_t m = i % d->period; if (m < 0) m += d->period;
    return m == d->phase;
}
int64_t wspr_next_tx_anchor_ms(int64_t from_ms, const wspr_duty_t* d) {
    // even-minute boundaries are at minute = 0,2,4,... -> ms = idx * 120000
    int64_t idx = from_ms <= 1000 ? 0 : ((from_ms - 1000) + 120000 - 1) / 120000; // ceil((from-1000)/120000)
    for (;; ++idx) {
        int64_t anchor = idx * 120000 + 1000;
        if (anchor >= from_ms && wspr_is_tx_slot(idx, d)) return anchor;
    }
}
int wspr_duty_phase_for_call(const char* call, int period) {
    if (period <= 0) return 0;
    uint32_t h = 2166136261u;                 // FNV-1a
    for (const char* p = call; p && *p; ++p) { h ^= (uint8_t)*p; h *= 16777619u; }
    return (int)(h % (uint32_t)period);
}
