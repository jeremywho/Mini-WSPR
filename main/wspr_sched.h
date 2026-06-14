#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { int period; int phase; } wspr_duty_t;  // TX when even_min_index % period == phase
// Precondition: utc_ms / from_ms are non-negative Unix-epoch milliseconds (GPS-sourced),
// far below INT64 limits; these functions are not defined for negative or near-overflow inputs.
// `phase` is normalized mod `period`, so any phase value selects a valid slot (never hangs).

int64_t wspr_even_minute_index(int64_t utc_ms);                 // index over EVEN minutes only
bool    wspr_is_tx_slot(int64_t even_minute_index, const wspr_duty_t* d);
int64_t wspr_next_tx_anchor_ms(int64_t from_ms, const wspr_duty_t* d); // boundary+1000, >= from_ms
int     wspr_duty_phase_for_call(const char* call, int period); // deterministic per-station spread
#ifdef __cplusplus
}
#endif
