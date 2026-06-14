#include <stdio.h>
#include <string.h>
#include <math.h>
#include "wspr_encode.h"
#include "tx_synth.h"
#include "wspr_sched.h"
#include "wspr_time.h"
#include "wspr_band.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); g_fail++; } \
} while (0)

// --- tx_synth -------------------------------------------------------------
// Frozen copy of the proven FT8 tx_task synth math (audio_trusdx_serial.cpp:966-999),
// the extraction oracle. Do NOT "improve" — it defines byte-exact correctness.
static void ft8_inline_reference(const uint8_t* tones, int count, double base_hz,
                                 int64_t start, int rate, double amp, uint8_t* out, int n) {
    const double TWO_PI = 6.28318530717958647692;
    const double sps = 0.160 * rate;
    double phase = 0.0; int64_t pos = start;
    for (int i = 0; i < n; ++i) {
        int idx = (int)floor((double)pos / sps);
        if (idx < 0) idx = 0;
        if (idx >= count) idx = count - 1;
        float tone_hz = (float)base_hz + 6.25f * (float)tones[idx];
        phase += TWO_PI * (double)tone_hz / (double)rate;
        if (phase > TWO_PI) phase = fmod(phase, TWO_PI);
        int v = 128 + (int)lrintf((float)amp * sinf((float)phase));
        if (v < 0) v = 0;
        if (v > 255) v = 255;
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
        static uint8_t a[20000], b[20000];
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
    int no3b = 1;
    uint8_t buf[4096];
    for (;;) { int k = tx_synth_pull(&s, buf, 4096); if (k == 0) break;
               for (int i = 0; i < k; ++i) if (buf[i] == 0x3B) no3b = 0; }
    CHECK(no3b, "byte-stuffing leaves no 0x3B in WSPR stream");
    CHECK(tx_synth_done(&s), "synth done after draining");
}

// --- wspr_sched -----------------------------------------------------------
static void test_sched(void) {
    CHECK(wspr_even_minute_index(0) == 0, "min0 -> idx0");
    CHECK(wspr_even_minute_index(120000) == 1, "min2 -> idx1");
    CHECK(wspr_even_minute_index(60000) == 0, "min1(odd) floors to even idx0");
    wspr_duty_t d = { 5, 0 };
    CHECK(wspr_is_tx_slot(0, &d) && !wspr_is_tx_slot(1, &d) && wspr_is_tx_slot(5, &d), "1-in-5 duty");
    CHECK(wspr_next_tx_anchor_ms(2000, &d) == 600000 + 1000, "next anchor skips to idx5");
    CHECK(wspr_next_tx_anchor_ms(0, &d) == 1000, "anchor at boundary0 + 1s");
    int64_t day = 86400000LL;
    CHECK(wspr_next_tx_anchor_ms(day - 5000, &d) >= day, "rolls into next day");
    int ph = wspr_duty_phase_for_call("K1ABC", 5);
    CHECK(ph >= 0 && ph < 5 && ph == wspr_duty_phase_for_call("K1ABC", 5), "call phase stable+in-range");
}

// --- wspr_time ------------------------------------------------------------
static void test_time(void) {
    int64_t ms = 0;
    CHECK(wspr_gps_to_epoch_ms("1970-01-01","00:00:00",&ms) && ms == 0, "epoch 1970");
    CHECK(wspr_gps_to_epoch_ms("2000-01-01","00:00:00",&ms) && ms == 946684800000LL, "epoch 2000");
    CHECK(wspr_gps_to_epoch_ms("2026-06-14","12:00:00",&ms) && ms == 1781438400000LL, "epoch 2026-06-14T12");
    CHECK(!wspr_gps_to_epoch_ms("2026-13-01","00:00:00",&ms), "reject bad month");
    CHECK(!wspr_gps_to_epoch_ms("2026-06-14","24:00:00",&ms), "reject bad hour");

    wspr_fix_src_t src = { true, 1000, 1781438400000LL, true };
    wspr_fix_t f = wspr_fix_now(&src, 5000, 10000);   // 4 s old, limit 10 s
    CHECK(f.usable && f.utc_ms == 1781438400000LL + 4000, "fresh fix projects +4s");
    f = wspr_fix_now(&src, 20000, 10000);             // 19 s old
    CHECK(!f.usable, "stale fix rejected");
    src.have_grid = false;
    CHECK(!wspr_fix_now(&src, 5000, 10000).usable, "no grid -> unusable");
    src.have_grid = true; src.have_valid_rmc = false;
    CHECK(!wspr_fix_now(&src, 5000, 10000).usable, "no valid RMC -> unusable");
}

// --- wspr_band ------------------------------------------------------------
static void test_band(void) {
    const wspr_band_t* b = wspr_band_by_name("20m");
    CHECK(b && b->dial_hz == 14095600LL, "20m dial");
    CHECK(wspr_band_by_name("40m")->dial_hz == 7038600LL, "40m dial");
    CHECK(wspr_band_by_name("nope") == 0, "unknown band -> NULL");
    double sp = 12000.0/8192.0;
    // WSPR is 4-FSK: 4 distinct tones (0..3), so the clamp uses tone_count = 4.
    CHECK(wspr_clamp_base_hz(1500, sp, 4, 1400, 1600) == 1500.0, "in-range base unchanged");
    CHECK(wspr_clamp_base_hz(1300, sp, 4, 1400, 1600) == 1400.0, "below window -> low");
    double hi = wspr_clamp_base_hz(1599, sp, 4, 1400, 1600);
    CHECK(hi + (4-1)*sp <= 1600.0 + 1e-9, "highest FSK tone kept inside window");
}

int main(void) {
    test_tx_synth_ft8_parity();
    test_tx_synth_totals_and_stuff();
    test_sched();
    test_time();
    test_band();
    if (g_fail == 0) printf("ALL TESTS PASSED\n");
    else printf("%d CHECK(S) FAILED\n", g_fail);
    return g_fail ? 1 : 0;
}
