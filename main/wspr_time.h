#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// "YYYY-MM-DD" + "HH:MM:SS" (UTC) -> Unix epoch milliseconds. false on malformed/out-of-range.
bool wspr_gps_to_epoch_ms(const char* date_ymd, const char* time_hms, int64_t* out_ms);

typedef struct {
    bool    have_valid_rmc;    // last RMC had status 'A'
    int64_t rmc_mono_ms;       // monotonic time when that RMC was parsed
    int64_t epoch_at_rmc_ms;   // UTC epoch from that RMC
    bool    have_grid;         // a 4-char grid is available
} wspr_fix_src_t;
typedef struct { bool usable; int64_t utc_ms; } wspr_fix_t;

// usable only if valid RMC + grid + age (now-rmc) <= max_age; utc projected on the mono clock.
wspr_fix_t wspr_fix_now(const wspr_fix_src_t* src, int64_t now_mono_ms, int64_t max_age_ms);
#ifdef __cplusplus
}
#endif
