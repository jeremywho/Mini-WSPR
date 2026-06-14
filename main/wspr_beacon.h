#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "wspr_sched.h"   // wspr_duty_t
#include "wspr_time.h"    // wspr_fix_src_t
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    char        callsign[12];
    char        grid_override[5];   // "" -> use the live GPS grid
    int         power_dbm;
    const char* band;               // e.g. "20m"
    wspr_duty_t duty;
    double      base_hz_desired;    // ~1500 (tone-0 audio)
} wspr_cfg_t;

typedef enum { WSPR_HOLD = 0, WSPR_WAIT = 1, WSPR_TX = 2 } wspr_action_t;

typedef struct {
    wspr_action_t action;
    int64_t  anchor_ms;             // WAIT/TX: epoch ms of the next TX (boundary+1000)
    int64_t  dial_hz;               // TX: radio dial
    double   base_hz;               // TX: clamped tone-0 audio frequency
    char     grid[5];               // TX: grid actually used
    uint8_t  symbols[162];          // TX: encoded channel symbols
} wspr_plan_t;

// Pure decision. HOLD (no fresh GPS / unencodable config / bad band), WAIT (countdown to
// anchor), or TX (plan filled). prekey_lead_ms = how early before the anchor to report TX
// so the device can pre-key and wait for the exact start; pass a small bounded positive
// value (the device's fixed TX setup lead, e.g. ~1500 ms). A negative value yields a
// permanent WAIT; a very large one keeps reporting TX for most of the inter-slot gap.
// No hardware, no I/O.
wspr_action_t wspr_beacon_decide(const wspr_cfg_t* cfg, const wspr_fix_src_t* src,
                                 const char* gps_grid4, int64_t now_mono_ms,
                                 int64_t max_age_ms, int64_t prekey_lead_ms,
                                 wspr_plan_t* out);
#ifdef __cplusplus
}
#endif
