#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// A symbol-stream TX plan. Frequencies/durations are ABSOLUTE (Hz / seconds);
// sample_rate is only the discretization. base_hz is the audio frequency of tone 0.
typedef struct {
    const uint8_t* symbols;       // values 0..(tones-1)
    int      count;               // 162 (WSPR) / 79 (FT8)
    double   symbol_seconds;      // 0.682667 (WSPR) / 0.160 (FT8)
    double   tone_spacing_hz;     // 12000.0/8192.0 (WSPR) / 6.25 (FT8)
    double   base_hz;             // tone-0 audio frequency (e.g. 1500)
    int      sample_rate;         // 11520 device / 12000 for wsprd host check
    double   amplitude;           // 88.0
    bool     byte_stuff;          // map sample 0x3B -> 0x3A (truSDX CAT delimiter)
} tx_synth_plan_t;

typedef struct {
    tx_synth_plan_t plan;
    double  samples_per_symbol;   // symbol_seconds * sample_rate
    int64_t total_samples;        // ceil(samples_per_symbol * count)
    int64_t sample_pos;           // next sample index to emit
    double  phase;                // continuous-phase accumulator (radians)
} tx_synth_t;

// start_sample: index to begin emitting at (>0 = late start; phase is NOT advanced
// for skipped samples, matching the proven FT8 tx_task). Clamped to [0,total].
void    tx_synth_init(tx_synth_t* s, const tx_synth_plan_t* plan, int64_t start_sample);
int64_t tx_synth_total_samples(const tx_synth_t* s);
int     tx_synth_pull(tx_synth_t* s, uint8_t* out, int max);  // count emitted; 0 at end
bool    tx_synth_done(const tx_synth_t* s);

#ifdef __cplusplus
}
#endif
