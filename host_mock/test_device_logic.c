#include <stdio.h>
#include <string.h>
#include <math.h>
#include "wspr_encode.h"
#include "tx_synth.h"
#include "wspr_sched.h"
#include "wspr_time.h"
#include "wspr_band.h"
#include "wspr_beacon.h"

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

static void test_tx_synth_chunked(void) {
    uint8_t wsym[162]; for (int i = 0; i < 162; ++i) wsym[i] = (uint8_t)((i * 7) % 4);
    tx_synth_plan_t wp = { wsym, 162, 8192.0/12000.0, 12000.0/8192.0, 1473.0, 11520, 88.0, true };
    tx_synth_t s1; tx_synth_init(&s1, &wp, 0);
    int64_t total = tx_synth_total_samples(&s1);
    static uint8_t big[1300000];
    int64_t got1 = 0;
    for (;;) { int want = (int)(total - got1 > 65536 ? 65536 : total - got1);
               int k = tx_synth_pull(&s1, big + got1, want); if (!k) break; got1 += k; }
    tx_synth_t s2; tx_synth_init(&s2, &wp, 0);
    int mism = 0; int64_t pos = 0; uint8_t c[64];
    for (;;) { int k = tx_synth_pull(&s2, c, 64); if (!k) break;
               for (int i = 0; i < k; ++i) if (pos + i < got1 && c[i] != big[pos + i]) mism = 1;
               pos += k; }
    CHECK(got1 == total && pos == total && !mism, "chunked 64B pulls == one-shot (state continuity)");
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
    wspr_duty_t bad = { 5, 7 };   // phase 7 normalizes to 2 -> a slot must still exist (no infinite loop)
    CHECK(wspr_is_tx_slot(2, &bad) && wspr_next_tx_anchor_ms(0, &bad) == 2*120000 + 1000,
          "out-of-range duty phase normalized, no hang");
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
    CHECK(!wspr_gps_to_epoch_ms("2026-02-30","00:00:00",&ms), "reject Feb 30");
    CHECK(!wspr_gps_to_epoch_ms("2026-04-31","00:00:00",&ms), "reject Apr 31");
    CHECK( wspr_gps_to_epoch_ms("2024-02-29","00:00:00",&ms), "accept leap Feb 29 (2024)");
    CHECK(!wspr_gps_to_epoch_ms("2026-02-29","00:00:00",&ms), "reject Feb 29 in non-leap 2026");
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

// --- wspr_beacon ----------------------------------------------------------
static void test_beacon(void) {
    wspr_cfg_t cfg; memset(&cfg, 0, sizeof cfg);
    strcpy(cfg.callsign, "K1ABC"); cfg.power_dbm = 37; cfg.band = "20m";
    cfg.duty.period = 1; cfg.duty.phase = 0; cfg.base_hz_desired = 1500;
    wspr_plan_t out;
    int64_t e30, e159;
    wspr_gps_to_epoch_ms("2026-06-14","12:00:30",&e30);
    wspr_gps_to_epoch_ms("2026-06-14","12:01:59",&e159);

    wspr_fix_src_t stale = { true, 1000, e30, true };
    CHECK(wspr_beacon_decide(&cfg,&stale,"FN42", 999999, 10000, 2000, &out)==WSPR_HOLD, "stale->HOLD");

    wspr_fix_src_t s30 = { true, 1000, e30, true };
    CHECK(wspr_beacon_decide(&cfg,&s30,"FN42", 1000, 10000, 2000, &out)==WSPR_WAIT, "mid-slot->WAIT");
    CHECK(out.anchor_ms > e30, "wait anchor in future");

    wspr_fix_src_t s159 = { true, 1000, e159, true };
    CHECK(wspr_beacon_decide(&cfg,&s159,"FN42", 1000, 10000, 3000, &out)==WSPR_TX, "near anchor->TX");
    CHECK(out.dial_hz==14095600LL, "TX dial 20m");
    CHECK(out.base_hz>=1400 && out.base_hz<=1600, "TX base in window");
    CHECK(out.symbols[0]<=3 && out.symbols[161]<=3, "TX symbols valid");

    wspr_cfg_t bad; memset(&bad, 0, sizeof bad);
    strcpy(bad.callsign,"ABCDEF"); bad.power_dbm=37; bad.band="20m"; bad.duty.period=1; bad.base_hz_desired=1500;
    CHECK(wspr_beacon_decide(&bad,&s30,"FN42",1000,10000,2000,&out)==WSPR_HOLD, "unencodable call -> HOLD not WAIT");

    strcpy(cfg.grid_override,"IO90");
    wspr_beacon_decide(&cfg,&s159,"FN42",1000,10000,3000,&out);
    CHECK(strcmp(out.grid,"IO90")==0, "grid override used");

    cfg.band="99m";
    CHECK(wspr_beacon_decide(&cfg,&s159,"FN42",1000,10000,3000,&out)==WSPR_HOLD, "bad band->HOLD");
}

int main(void) {
    test_tx_synth_ft8_parity();
    test_tx_synth_totals_and_stuff();
    test_tx_synth_chunked();
    test_sched();
    test_time();
    test_band();
    test_beacon();
    if (g_fail == 0) printf("ALL TESTS PASSED\n");
    else printf("%d CHECK(S) FAILED\n", g_fail);
    return g_fail ? 1 : 0;
}
