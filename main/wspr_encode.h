#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WSPR_SYMBOL_COUNT 162

// Encode a standard (Type 1) WSPR message into 162 four-FSK channel symbols (0..3).
//   callsign  : standard call, <= 6 chars, exactly one positionable digit; no '/'.
//   grid4     : 4-char Maidenhead locator (e.g. "FN42"); letters A-R, digits 0-9.
//   power_dbm : 0..60, last decimal digit must be 0, 3, or 7.
// Returns true and fills out_symbols on success.
// Returns false (out_symbols untouched) if the inputs are not Type-1 encodable.
bool wspr_encode_type1(const char* callsign, const char* grid4,
                       int power_dbm, uint8_t out_symbols[WSPR_SYMBOL_COUNT]);

#ifdef __cplusplus
}
#endif
