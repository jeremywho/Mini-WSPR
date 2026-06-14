#include "wspr_band.h"
#include <string.h>
static const wspr_band_t BANDS[] = {
    {"160m", 1836600LL}, {"80m", 3568600LL}, {"60m", 5287200LL}, {"40m", 7038600LL},
    {"30m", 10138700LL}, {"20m", 14095600LL}, {"17m", 18104600LL}, {"15m", 21094600LL},
    {"12m", 24924600LL}, {"10m", 28124600LL}, {"6m", 50293000LL},
};
const wspr_band_t* wspr_band_by_name(const char* name) {
    if (!name) return 0;
    for (unsigned i = 0; i < sizeof(BANDS) / sizeof(BANDS[0]); ++i)
        if (strcmp(BANDS[i].name, name) == 0) return &BANDS[i];
    return 0;
}
double wspr_clamp_base_hz(double desired, double tone_spacing, int tone_count,
                          double win_low, double win_high) {
    double span = (tone_count - 1) * tone_spacing;
    double hi = win_high - span;
    if (hi < win_low) hi = win_low;
    if (desired < win_low) return win_low;
    if (desired > hi) return hi;
    return desired;
}
