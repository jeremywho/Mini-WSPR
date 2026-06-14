# WSPR Device Logic (host-tested) Implementation Plan — Plan 2 of 3

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans or subagent-driven-development. Steps use `- [ ]` checkboxes.

**Goal:** Implement the pure, dependency-free, host-testable logic of the WSPR beacon — the TX synth, the scheduler, the GPS time/freshness contract, and the band/frequency plan — each verified on the host (MinGW gcc), with the synth additionally round-tripped through `wsprd` at both 12000 Hz and the device's 11520 Hz.

**Architecture:** Four pure C11 modules under `main/` (no ESP-IDF deps, so the same objects compile on host and ESP32-S3 later): `tx_synth` (generalized symbol→PCM renderer extracted byte-for-byte from the proven FT8 `tx_task`), `wspr_sched` (even-UTC-minute + duty scheduler), `wspr_time` (GPS RMC freshness gate + epoch conversion), `wspr_band` (dial-freq table + audio-offset clamp). Host tests live in `host_mock/`. The synth is verified against a frozen copy of the inline FT8 math (must be byte-identical) and via `wsprd` decode of audio it renders.

**Tech Stack:** C11, MinGW-W64 gcc 15.2, Python 3.13 (`/c/Python313/python`, for WAV pad + 25/24 resample), `wsprd` at `C:\WSJT\wsjtx\bin\wsprd.exe`. Reuses the merged `main/wspr_encode.{c,h}` from Plan 1.

**Codex review of this design (high effort): "Sound-with-changes."** Refinements folded in: (1) FT8 byte-parity test guards the extraction; (2) dual-rate wsprd round-trip (12000 + 11520→12000 resample) + per-symbol boundary assertions; (3) anchor accuracy / pre-key-then-wait + skip-late is a Plan 3 device concern, noted; (4) freshness-gate GPS; (5) per-call duty phase to avoid slot collisions; (6) `base_hz` = tone-0 audio frequency, clamp tone (count−1) in-band.

---

## File Structure

| File | Responsibility |
|---|---|
| `main/tx_synth.h` / `.c` | Generalized stateful symbol→8-bit-PCM renderer. Byte-identical to the FT8 `tx_task` math. The device `tx_task` will later call this (Plan 3). |
| `main/wspr_sched.h` / `.c` | Even-UTC-minute index, duty selection, next-TX-anchor (boundary+1000ms), per-call duty phase. |
| `main/wspr_time.h` / `.c` | `YYYY-MM-DD`+`HH:MM:SS` → Unix epoch ms; RMC-freshness-gated fix with monotonic projection. |
| `main/wspr_band.h` / `.c` | WSPR dial-frequency table; tone-0 audio base clamp. |
| `host_mock/test_device_logic.c` | Unit tests for all four modules incl. the FT8 byte-parity oracle. |
| `host_mock/wspr_render.c` | CLI: `wspr_render <call> <grid> <dbm> <rate> <out.wav>` — encode (Plan 1) + `tx_synth` → 16-bit mono WAV. Drives the `wsprd` round-trips. |
| `host_mock/roundtrip_wspr.sh` | Renders at 12000 and at 11520 (resample 25/24), pads to 120 s, runs `wsprd`, asserts the message. |
| `host_mock/build_device.sh` | Compiles + runs `test_device_logic`; builds `wspr_render`. |

---

## Task 1: `tx_synth` — extracted, byte-identical renderer

**Files:** Create `main/tx_synth.h`, `main/tx_synth.c`, append to `host_mock/test_device_logic.c`, create `host_mock/build_device.sh`.

- [ ] **Step 1: Header**

`main/tx_synth.h`:
```c
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
```

- [ ] **Step 2: Failing tests** — append to `host_mock/test_device_logic.c`. This includes the FROZEN FT8 inline reference (a verbatim copy of `tx_task`'s math) and asserts byte-identity.

```c
#include <math.h>
#include "tx_synth.h"

// Frozen copy of the proven FT8 tx_task synth math (audio_trusdx_serial.cpp:966-999),
// the extraction oracle. Do NOT "improve" — it defines byte-exact correctness.
static void ft8_inline_reference(const uint8_t* tones, int count, double base_hz,
                                 int64_t start, int rate, double amp, uint8_t* out, int n) {
    const double TWO_PI = 6.28318530717958647692;
    const double sps = 0.160 * rate;
    double phase = 0.0; int64_t pos = start;
    for (int i = 0; i < n; ++i) {
        int idx = (int)floor((double)pos / sps);
        if (idx < 0) idx = 0; if (idx >= count) idx = count - 1;
        float tone_hz = (float)base_hz + 6.25f * (float)tones[idx];
        phase += TWO_PI * (double)tone_hz / (double)rate;
        if (phase > TWO_PI) phase = fmod(phase, TWO_PI);
        int v = 128 + (int)lrintf((float)amp * sinf((float)phase));
        if (v < 0) v = 0; if (v > 255) v = 255;
        uint8_t b = (uint8_t)v; if (b == 0x3B) b = 0x3A;
        out[i] = b; pos++;
    }
}

static void test_tx_synth_ft8_parity(void) {
    uint8_t tones[79]; for (int i = 0; i < 79; ++i) tones[i] = (uint8_t)(i % 8);
    for (int start = 0; start <= 4000; start += 2000) {
        tx_synth_plan_t p = { tones, 79, 0.160, 6.25, 1500.0, 11520, 88.0, true };
        tx_synth_t s; tx_synth_init(&s, &p, start);
        int64_t total = tx_synth_total_samples(&s);
        int n = (int)(total - start); if (n > 20000) n = 20000;
        uint8_t a[20000], b[20000];
        int got = tx_synth_pull(&s, a, n);
        ft8_inline_reference(tones, 79, 1500.0, start, 11520, 88.0, b, n);
        CHECK(got == n && memcmp(a, b, n) == 0, "tx_synth == frozen FT8 inline math");
    }
}

static void test_tx_synth_totals_and_stuff(void) {
    uint8_t wsym[162]; for (int i = 0; i < 162; ++i) wsym[i] = (uint8_t)(i % 4);
    tx_synth_plan_t wp = { wsym, 162, 8192.0/12000.0, 12000.0/8192.0, 1500.0, 11520, 88.0, true };
    tx_synth_t s; tx_synth_init(&s, &wp, 0);
    CHECK(tx_synth_total_samples(&s) == 1274020, "WSPR total samples @11520 = 1274020");
    // pull all; assert no raw 0x3B survives stuffing and range is 0..255
    int no3b = 1;
    uint8_t buf[4096];
    for (;;) { int k = tx_synth_pull(&s, buf, 4096); if (k == 0) break;
               for (int i = 0; i < k; ++i) if (buf[i] == 0x3B) no3b = 0; }
    CHECK(no3b, "byte-stuffing leaves no 0x3B in WSPR stream");
    CHECK(tx_synth_done(&s), "synth done after draining");
}
```

- [ ] **Step 3: Run — expect failure** (no `tx_synth.c`).
Run: `sh host_mock/build_device.sh` (created in Step 5). Expected: undefined `tx_synth_*`.

- [ ] **Step 4: Implement** `main/tx_synth.c`:
```c
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
```
(Byte-exactness vs FT8: for FT8 params, `(float)base + (float)6.25*(float)sym` == `(float)base + 6.25f*(float)sym`, `(float)88.0`==`88.0f`, same TWO_PI/fmod/lrintf/clamp/stuff order.)

- [ ] **Step 5: Build script** `host_mock/build_device.sh`:
```sh
#!/bin/sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../main"
gcc -std=c11 -g -Wall -Wextra -I"$SRC" -I"$HERE" \
    "$SRC/wspr_encode.c" "$SRC/tx_synth.c" "$SRC/wspr_sched.c" "$SRC/wspr_time.c" "$SRC/wspr_band.c" \
    "$HERE/test_device_logic.c" -lm -o "$HERE/test_device_logic.exe"
"$HERE/test_device_logic.exe"
gcc -std=c11 -g -Wall -Wextra -I"$SRC" \
    "$SRC/wspr_encode.c" "$SRC/tx_synth.c" "$HERE/wspr_render.c" -lm -o "$HERE/wspr_render.exe"
echo "built wspr_render.exe"
```
(Modules from later tasks are referenced; create empty stubs for `wspr_sched.c`/`wspr_time.c`/`wspr_band.c` + their headers now so Task 1 links — Tasks 2–4 fill them. Also create the `test_device_logic.c` top matter: includes `<stdio.h>`,`<string.h>`,`wspr_encode.h`, the `CHECK` macro + `g_fail`, and a `main()` that calls the test fns. And a minimal `wspr_render.c` stub printing usage, fleshed out in Task 1 Step 7.)

- [ ] **Step 6: Run — expect pass** (parity + totals).
Run: `sh host_mock/build_device.sh`. Expected: `ALL TESTS PASSED`.

- [ ] **Step 7: `wspr_render.c` + the `wsprd` round-trips.**

`host_mock/wspr_render.c`:
```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wspr_encode.h"
#include "tx_synth.h"
static void w16(FILE* f, int x){ unsigned char b[2]={(unsigned char)(x&0xff),(unsigned char)((x>>8)&0xff)}; fwrite(b,1,2,f);}
static void w32(FILE* f, long x){ for(int i=0;i<4;i++){ fputc((int)(x&0xff),f); x>>=8;} }
int main(int argc, char** argv){
    if (argc != 6){ fprintf(stderr,"usage: wspr_render <call> <grid> <dbm> <rate> <out.wav>\n"); return 2; }
    int dbm = atoi(argv[3]), rate = atoi(argv[4]);
    uint8_t sym[WSPR_SYMBOL_COUNT];
    if (!wspr_encode_type1(argv[1], argv[2], dbm, sym)){ fprintf(stderr,"encode failed\n"); return 1; }
    tx_synth_plan_t p = { sym, 162, 8192.0/12000.0, 12000.0/8192.0, 1500.0, rate, 88.0, true };
    tx_synth_t s; tx_synth_init(&s, &p, 0);
    long n = (long)tx_synth_total_samples(&s);
    FILE* f = fopen(argv[5],"wb"); if(!f){ perror("open"); return 1; }
    fwrite("RIFF",1,4,f); w32(f,36+n*2); fwrite("WAVE",1,4,f);
    fwrite("fmt ",1,4,f); w32(f,16); w16(f,1); w16(f,1); w32(f,rate); w32(f,(long)rate*2); w16(f,2); w16(f,16);
    fwrite("data",1,4,f); w32(f,n*2);
    uint8_t buf[4096];
    for(;;){ int k = tx_synth_pull(&s, buf, 4096); if(k==0) break;
             for(int i=0;i<k;i++) w16(f, ((int)buf[i]-128)*256); }
    fclose(f); fprintf(stderr,"rendered %ld samples @%d\n", n, rate); return 0;
}
```

`host_mock/roundtrip_wspr.sh`:
```sh
#!/bin/sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PY=/c/Python313/python
WSPRD="C:/WSJT/wsjtx/bin/wsprd.exe"
"$HERE/wspr_render.exe" K1ABC FN42 37 12000 "$HERE/r12000.wav"
"$HERE/wspr_render.exe" K1ABC FN42 37 11520 "$HERE/r11520.wav"
# 11520 -> 12000 via exact 25/24 polyphase; both padded to 120 s @12000
"$PY" - "$HERE" <<'PY'
import sys, wave, numpy as np
from scipy.signal import resample_poly
here = sys.argv[1]
def load(p):
    w=wave.open(p); fr=w.readframes(w.getnframes()); w.close()
    return np.frombuffer(fr, dtype='<i2').astype(np.float64)
def save_pad(x, p, fs=12000, secs=120):
    x=np.clip(np.round(x),-32768,32767).astype('<i2')
    pad=np.zeros(fs*secs - len(x), dtype='<i2') if len(x)<fs*secs else np.zeros(0,dtype='<i2')
    out=wave.open(p,'wb'); out.setnchannels(1); out.setsampwidth(2); out.setframerate(fs)
    out.writeframes(x.tobytes()+pad.tobytes()); out.close()
save_pad(load(here+'/r12000.wav'), here+'/r12000_pad.wav')
save_pad(resample_poly(load(here+'/r11520.wav'), 25, 24), here+'/r11520_pad.wav')
print("padded both")
PY
for tag in r12000 r11520; do
  echo "=== wsprd $tag ==="
  "$WSPRD" -f 14.0971 "$HERE/${tag}_pad.wav" 2>&1 | tee "$HERE/.${tag}.txt"
  grep -iq "K1ABC FN42 37" "$HERE/.${tag}.txt" && echo "${tag} OK" || { echo "${tag} FAIL"; exit 1; }
done
rm -f "$HERE"/r12000*.wav "$HERE"/r11520*.wav "$HERE"/.r12000.txt "$HERE"/.r11520.txt
echo "ROUNDTRIP OK (both rates)"
```

- [ ] **Step 8: Run the round-trips** — Run: `sh host_mock/build_device.sh && sh host_mock/roundtrip_wspr.sh`. Expected: `r12000 OK`, `r11520 OK`, `ROUNDTRIP OK (both rates)`. (If `scipy` is absent: `/c/Python313/python -m pip install scipy`.)

- [ ] **Step 9: Commit**
```bash
git add main/tx_synth.* host_mock/test_device_logic.c host_mock/build_device.sh host_mock/wspr_render.c host_mock/roundtrip_wspr.sh
git commit -m "feat: tx_synth (extracted, FT8 byte-parity + WSPR wsprd round-trip @12000/11520)"
```

---

## Task 2: `wspr_sched` — even-minute / duty scheduler

**Files:** Create `main/wspr_sched.h` / `.c`; append tests.

- [ ] **Step 1: Header** `main/wspr_sched.h`:
```c
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
```

- [ ] **Step 2: Failing tests** (append):
```c
#include "wspr_sched.h"
static void test_sched(void) {
    // 00:00:00 UTC = minute 0 (even, index 0); 00:02:00 = index 1; 00:04:00 = index 2
    CHECK(wspr_even_minute_index(0) == 0, "min0 -> idx0");
    CHECK(wspr_even_minute_index(120000) == 1, "min2 -> idx1");
    CHECK(wspr_even_minute_index(60000) == 0, "min1(odd) floors to even idx0");
    wspr_duty_t d = { 5, 0 };
    CHECK(wspr_is_tx_slot(0, &d) && !wspr_is_tx_slot(1, &d) && wspr_is_tx_slot(5, &d), "1-in-5 duty");
    // from just after 00:00:01 anchor -> next tx anchor is 00:10:01 (idx 5) for phase 0
    CHECK(wspr_next_tx_anchor_ms(2000, &d) == 600000 + 1000, "next anchor skips to idx5");
    CHECK(wspr_next_tx_anchor_ms(0, &d) == 1000, "anchor at boundary0 + 1s");
    // hour/day rollover sanity: a from_ms near a day boundary returns a valid future anchor
    int64_t day = 86400000LL;
    CHECK(wspr_next_tx_anchor_ms(day - 5000, &d) >= day, "rolls into next day");
    // per-call phase deterministic + in range
    int ph = wspr_duty_phase_for_call("K1ABC", 5);
    CHECK(ph >= 0 && ph < 5 && ph == wspr_duty_phase_for_call("K1ABC", 5), "call phase stable+in-range");
}
```

- [ ] **Step 3: Run — expect failure.** Run: `sh host_mock/build_device.sh`.

- [ ] **Step 4: Implement** `main/wspr_sched.c`:
```c
#include "wspr_sched.h"
#include <string.h>

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
```

- [ ] **Step 5: Run — expect pass.** **Step 6: Commit** `feat: wspr_sched even-minute/duty scheduler + per-call phase`.

---

## Task 3: `wspr_time` — GPS epoch + RMC freshness gate

**Files:** Create `main/wspr_time.h` / `.c`; append tests.

- [ ] **Step 1: Header** `main/wspr_time.h`:
```c
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
```

- [ ] **Step 2: Failing tests** (append):
```c
#include "wspr_time.h"
static void test_time(void) {
    int64_t ms = 0;
    CHECK(wspr_gps_to_epoch_ms("1970-01-01","00:00:00",&ms) && ms == 0, "epoch 1970");
    CHECK(wspr_gps_to_epoch_ms("2000-01-01","00:00:00",&ms) && ms == 946684800000LL, "epoch 2000");
    CHECK(wspr_gps_to_epoch_ms("2026-06-14","12:00:00",&ms) && ms == 1781784000000LL, "epoch 2026-06-14T12");
    CHECK(!wspr_gps_to_epoch_ms("2026-13-01","00:00:00",&ms), "reject bad month");
    CHECK(!wspr_gps_to_epoch_ms("2026-06-14","24:00:00",&ms), "reject bad hour");

    wspr_fix_src_t src = { true, 1000, 1781784000000LL, true };
    wspr_fix_t f = wspr_fix_now(&src, 5000, 10000);   // 4 s old, limit 10 s
    CHECK(f.usable && f.utc_ms == 1781784000000LL + 4000, "fresh fix projects +4s");
    f = wspr_fix_now(&src, 20000, 10000);             // 19 s old
    CHECK(!f.usable, "stale fix rejected");
    src.have_grid = false;
    CHECK(!wspr_fix_now(&src, 5000, 10000).usable, "no grid -> unusable");
    src.have_grid = true; src.have_valid_rmc = false;
    CHECK(!wspr_fix_now(&src, 5000, 10000).usable, "no valid RMC -> unusable");
}
```
(Expected epoch for 2026-06-14T12:00:00Z is `1781784000000` — confirm at implement time with `/c/Python313/python -c "import datetime;print(int(datetime.datetime(2026,6,14,12,tzinfo=datetime.timezone.utc).timestamp())*1000)"` and adjust the literal if needed.)

- [ ] **Step 3: Run — expect failure.**

- [ ] **Step 4: Implement** `main/wspr_time.c`:
```c
#include "wspr_time.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// days from civil 1970-01-01 (Howard Hinnant), valid for Gregorian dates.
static int64_t days_from_civil(int y, int m, int d) {
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int64_t yoe = (int64_t)(y - era * 400);
    int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + (d - 1);
    int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}
static bool digits(const char* p, int n) { for (int i=0;i<n;i++) if(!isdigit((unsigned char)p[i])) return false; return true; }

bool wspr_gps_to_epoch_ms(const char* date_ymd, const char* time_hms, int64_t* out_ms) {
    if (!date_ymd || !time_hms || !out_ms) return false;
    if (strlen(date_ymd)!=10 || date_ymd[4]!='-' || date_ymd[7]!='-') return false;
    if (strlen(time_hms)!=8 || time_hms[2]!=':' || time_hms[5]!=':') return false;
    if (!digits(date_ymd,4) || !digits(date_ymd+5,2) || !digits(date_ymd+8,2)) return false;
    if (!digits(time_hms,2) || !digits(time_hms+3,2) || !digits(time_hms+6,2)) return false;
    int y=atoi(date_ymd), mo=atoi(date_ymd+5), d=atoi(date_ymd+8);
    int h=atoi(time_hms), mi=atoi(time_hms+3), s=atoi(time_hms+6);
    if (mo<1||mo>12||d<1||d>31||h>23||mi>59||s>60) return false;
    int64_t days = days_from_civil(y, mo, d);
    int64_t secs = days*86400 + h*3600 + mi*60 + s;
    *out_ms = secs * 1000;
    return true;
}
wspr_fix_t wspr_fix_now(const wspr_fix_src_t* src, int64_t now_mono_ms, int64_t max_age_ms) {
    wspr_fix_t f = { false, 0 };
    if (!src || !src->have_valid_rmc || !src->have_grid) return f;
    int64_t age = now_mono_ms - src->rmc_mono_ms;
    if (age < 0 || age > max_age_ms) return f;
    f.usable = true;
    f.utc_ms = src->epoch_at_rmc_ms + age;
    return f;
}
```

- [ ] **Step 5: Run — expect pass.** **Step 6: Commit** `feat: wspr_time GPS epoch + RMC freshness gate`.

---

## Task 4: `wspr_band` — dial table + audio clamp

**Files:** Create `main/wspr_band.h` / `.c`; append tests.

- [ ] **Step 1: Header** `main/wspr_band.h`:
```c
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { const char* name; int64_t dial_hz; } wspr_band_t;
const wspr_band_t* wspr_band_by_name(const char* name);   // NULL if unknown
// Clamp tone-0 audio base so [base, base+(count-1)*spacing] stays within [win_low, win_high].
double wspr_clamp_base_hz(double desired, double tone_spacing, int tone_count,
                          double win_low, double win_high);
#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Failing tests** (append):
```c
#include "wspr_band.h"
static void test_band(void) {
    const wspr_band_t* b = wspr_band_by_name("20m");
    CHECK(b && b->dial_hz == 14095600LL, "20m dial");
    CHECK(wspr_band_by_name("40m")->dial_hz == 7038600LL, "40m dial");
    CHECK(wspr_band_by_name("nope") == 0, "unknown band -> NULL");
    double sp = 12000.0/8192.0;
    CHECK(wspr_clamp_base_hz(1500, sp, 162, 1400, 1600) == 1500.0, "in-range base unchanged");
    CHECK(wspr_clamp_base_hz(1300, sp, 162, 1400, 1600) == 1400.0, "below window -> low");
    double hi = wspr_clamp_base_hz(1599, sp, 162, 1400, 1600);
    CHECK(hi + (162-1)*sp <= 1600.0 + 1e-9, "tone (count-1) kept inside window");
}
```

- [ ] **Step 3: Run — expect failure.**

- [ ] **Step 4: Implement** `main/wspr_band.c`:
```c
#include "wspr_band.h"
#include <string.h>
static const wspr_band_t BANDS[] = {
    {"160m", 1836600LL}, {"80m", 3568600LL}, {"60m", 5287200LL}, {"40m", 7038600LL},
    {"30m", 10138700LL}, {"20m", 14095600LL}, {"17m", 18104600LL}, {"15m", 21094600LL},
    {"12m", 24924600LL}, {"10m", 28124600LL}, {"6m", 50293000LL},
};
const wspr_band_t* wspr_band_by_name(const char* name) {
    if (!name) return 0;
    for (unsigned i = 0; i < sizeof(BANDS)/sizeof(BANDS[0]); ++i)
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
```

- [ ] **Step 5: Run — expect pass.** **Step 6: Commit** `feat: wspr_band dial table + audio-base clamp`.

---

## Self-Review (against spec + Codex)

- **Spec §6.2 synth** (11520, fractional SPS, double phase, tone-0 base, 0x3A stuffing, anchor) → Task 1. ✓ Anchor-at-start handled via `start_sample` + the device-side pre-key/skip-late noted for Plan 3.
- **Spec §6.3 scheduler** (even-minute+1s, duty) → Task 2. ✓ Per-call phase added (Codex #7).
- **Spec §6.5 / §8 GPS contract** (freshness + epoch) → Task 3. ✓
- **Spec §9 frequency plan** (dial table, tone-0 clamp) → Task 4. ✓
- **Codex #1 FT8 parity** → Task 1 Step 2 frozen oracle. ✓ **#2 dual-rate wsprd + boundaries** → Task 1 Steps 7–8 (+ totals/boundary in Step 2). ✓ **#4 freshness** → Task 3. ✓ **#6/#8 band semantics** → Task 4. ✓
- **Placeholders:** none — every step has complete code. The only computed literal (epoch for 2026-06-14) has an explicit verification command.
- **Type consistency:** module names/signatures consistent across headers, build script, and tests.

**Deferred to Plan 3 (needs hardware):** wiring `tx_synth` into the device `tx_task` (pre-key → wait-for-anchor → stream; skip-if-late), the GPS module producing `wspr_fix_src_t`, `wspr_config` (NVS), the UI, ESP-IDF bootstrap, and the on-hardware RF loopback + 110.6 s serial-pacing/drift soak (Codex #3, #5).

---

## Next: Plan 3 (device integration + RF verification — hardware-gated)
Bootstrap the Mini-WSPR ESP-IDF repo from the Mini-FT8 skeleton; strip FT8 decode; refactor `tx_task` to call `tx_synth` via `trusdx_begin_tx_plan` (pre-key, wait-for-anchor, skip-late); feed `wspr_fix_src_t` from `gps.cpp`; add `wspr_config` (NVS) + status UI; `idf.py build`; then on-hardware RF loopback decode + ≥10-TX dummy-load soak with pacing/drift logging + on-air WSPRnet spot. The pure modules from this plan drop in unchanged.
