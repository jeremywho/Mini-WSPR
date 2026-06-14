#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { int period; int phase; } wspr_duty_t;  // TX when even_min_index % period == phase

int64_t wspr_even_minute_index(int64_t utc_ms);                 // index over EVEN minutes only
bool    wspr_is_tx_slot(int64_t even_minute_index, const wspr_duty_t* d);
int64_t wspr_next_tx_anchor_ms(int64_t from_ms, const wspr_duty_t* d); // boundary+1000, >= from_ms
int     wspr_duty_phase_for_call(const char* call, int period); // deterministic per-station spread
#ifdef __cplusplus
}
#endif
