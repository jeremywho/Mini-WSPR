#include "tx_synth.h"
#include <math.h>

static const double TX_TWO_PI = 6.28318530717958647692;
static inline uint8_t clamp_u8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v); }

void tx_synth_init(tx_synth_t* s, const tx_synth_plan_t* plan, int64_t start_sample) {
    s->plan = *plan;
    s->samples_per_symbol = plan->symbol_seconds * (double)plan->sample_rate;
    s->total_samples = (int64_t)ceil(s->samples_per_symbol * (double)plan->count);
    if (start_sample < 0) start_sample = 0;
    if (start_sample > s->total_samples) start_sample = s->total_samples;
    s->sample_pos = start_sample;
    s->phase = 0.0;
}

int64_t tx_synth_total_samples(const tx_synth_t* s) { return s->total_samples; }
bool tx_synth_done(const tx_synth_t* s) { return s->sample_pos >= s->total_samples; }

int tx_synth_pull(tx_synth_t* s, uint8_t* out, int max) {
    const double sps = s->samples_per_symbol;
    int i = 0;
    for (; i < max && s->sample_pos < s->total_samples; ++i) {
        int idx = (int)floor((double)s->sample_pos / sps);
        if (idx < 0) idx = 0;
        if (idx >= s->plan.count) idx = s->plan.count - 1;
        float tone_hz = (float)s->plan.base_hz
                      + (float)s->plan.tone_spacing_hz * (float)s->plan.symbols[idx];
        s->phase += TX_TWO_PI * (double)tone_hz / (double)s->plan.sample_rate;
        if (s->phase > TX_TWO_PI) s->phase = fmod(s->phase, TX_TWO_PI);
        int v = 128 + (int)lrintf((float)s->plan.amplitude * sinf((float)s->phase));
        uint8_t b = clamp_u8(v);
        if (s->plan.byte_stuff && b == 0x3B) b = 0x3A;
        out[i] = b;
        s->sample_pos++;
    }
    return i;
}
