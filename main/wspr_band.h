#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { const char* name; int64_t dial_hz; } wspr_band_t;
const wspr_band_t* wspr_band_by_name(const char* name);   // NULL if unknown
// Clamp the tone-0 audio base so every FSK tone stays within [win_low, win_high].
// tone_count = number of FSK tones (4 for WSPR 4-FSK), so the highest tone sits at
// base + (tone_count-1)*tone_spacing.
// Precondition: the window is at least as wide as the tone span ((tone_count-1)*spacing);
// for WSPR that span is ~4.4 Hz in a 200 Hz window. If the window is narrower, the result
// is best-effort (returns win_low) and high tones may exceed win_high.
double wspr_clamp_base_hz(double desired, double tone_spacing, int tone_count,
                          double win_low, double win_high);
#ifdef __cplusplus
}
#endif
