# WSPR Device Integration + RF Verification — Plan 3 of 3

> Task 1 is pure host-testable logic (implement + verify off-hardware). Tasks 2–7 are the
> on-hardware integration runbook — code is written here, but VERIFICATION needs the physical
> Cardputer ADV + (tr)uSDX + GPS rig and is done on the bench.

**Goal:** Turn the host-verified WSPR modules (Plans 1–2) into a working beacon on hardware:
one pure decision function that drives the whole beacon, then the ESP-IDF integration that
keys the truSDX from it, plus on-air verification.

**Architecture:** `wspr_beacon_decide()` (pure) folds `wspr_time` + `wspr_sched` + `wspr_band` +
`wspr_encode` into a single `HOLD / WAIT / TX(plan)` decision. The device loop calls it each
tick; on `TX` it pre-keys the truSDX, waits for the exact anchor, and streams `tx_synth` via a
generalized `trusdx_begin_tx_plan`. GPS feeds a freshness-gated `wspr_fix_src_t`; config lives
in NVS; a status screen shows countdown / TX progress.

**Tech Stack:** ESP-IDF v5.4 (`C:\esp\esp-idf`, `. C:\esp\esp-idf\export.ps1; idf.py build`),
the Mini-FT8 skeleton (M5 libs, USB-host, `cdc_acm`==2.2.0), MinGW gcc for the host test.

---

## Task 1 (HOST-TESTABLE): `wspr_beacon_decide` — the beacon brain

**Files:** Create `main/wspr_beacon.h` / `.c`; extend `host_mock/build_device.sh` + `host_mock/test_device_logic.c`.

The decision logic is pure (no hardware) and so is verified on the host now.

- [ ] **Step 1: Header** `main/wspr_beacon.h`:
```c
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "wspr_sched.h"   // wspr_duty_t
#include "wspr_time.h"    // wspr_fix_src_t
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    char        callsign[12];
    char        grid_override[5];   // "" -> use the live GPS grid
    int         power_dbm;
    const char* band;               // e.g. "20m"
    wspr_duty_t duty;
    double      base_hz_desired;    // ~1500 (tone-0 audio)
} wspr_cfg_t;

typedef enum { WSPR_HOLD = 0, WSPR_WAIT = 1, WSPR_TX = 2 } wspr_action_t;

typedef struct {
    wspr_action_t action;
    int64_t  anchor_ms;             // WAIT/TX: epoch ms of the next TX (boundary+1000)
    int64_t  dial_hz;               // TX: radio dial
    double   base_hz;               // TX: clamped tone-0 audio frequency
    char     grid[5];               // TX: grid actually used
    uint8_t  symbols[162];          // TX: encoded channel symbols
} wspr_plan_t;

// Pure decision. HOLD (no fresh GPS / unencodable config / bad band), WAIT (countdown to
// anchor), or TX (plan filled). prekey_lead_ms = how early before the anchor to report TX
// so the device can pre-key and wait for the exact start. No hardware, no I/O.
wspr_action_t wspr_beacon_decide(const wspr_cfg_t* cfg, const wspr_fix_src_t* src,
                                 const char* gps_grid4, int64_t now_mono_ms,
                                 int64_t max_age_ms, int64_t prekey_lead_ms,
                                 wspr_plan_t* out);
#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Failing tests** — append `test_beacon()` to `host_mock/test_device_logic.c`, `#include "wspr_beacon.h"`, and call it from `main`:
```c
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

    strcpy(cfg.grid_override,"IO90");
    wspr_beacon_decide(&cfg,&s159,"FN42",1000,10000,3000,&out);
    CHECK(strcmp(out.grid,"IO90")==0, "grid override used");

    cfg.band="99m";
    CHECK(wspr_beacon_decide(&cfg,&s159,"FN42",1000,10000,3000,&out)==WSPR_HOLD, "bad band->HOLD");
}
```
Add `wspr_beacon.c` to the `gcc` line in `host_mock/build_device.sh`.

- [ ] **Step 3: Run — expect failure.** `sh host_mock/build_device.sh`.

- [ ] **Step 4: Implement** `main/wspr_beacon.c`:
```c
#include "wspr_beacon.h"
#include "wspr_band.h"
#include "wspr_encode.h"
#include <string.h>

wspr_action_t wspr_beacon_decide(const wspr_cfg_t* cfg, const wspr_fix_src_t* src,
                                 const char* gps_grid4, int64_t now_mono_ms,
                                 int64_t max_age_ms, int64_t prekey_lead_ms,
                                 wspr_plan_t* out) {
    memset(out, 0, sizeof(*out));
    out->action = WSPR_HOLD;
    if (!cfg) return WSPR_HOLD;

    wspr_fix_t f = wspr_fix_now(src, now_mono_ms, max_age_ms);
    if (!f.usable) return WSPR_HOLD;                          // no fresh GPS time

    const char* grid = (cfg->grid_override[0]) ? cfg->grid_override : gps_grid4;
    const wspr_band_t* band = wspr_band_by_name(cfg->band);
    if (!band || !grid) return WSPR_HOLD;

    int64_t anchor = wspr_next_tx_anchor_ms(f.utc_ms, &cfg->duty);
    out->anchor_ms = anchor;

    if (anchor - f.utc_ms > prekey_lead_ms) { out->action = WSPR_WAIT; return WSPR_WAIT; }

    if (!wspr_encode_type1(cfg->callsign, grid, cfg->power_dbm, out->symbols))
        return WSPR_HOLD;                                     // unencodable config
    out->base_hz = wspr_clamp_base_hz(cfg->base_hz_desired, 12000.0/8192.0, 4, 1400.0, 1600.0);
    out->dial_hz = band->dial_hz;
    strncpy(out->grid, grid, 4); out->grid[4] = 0;
    out->action = WSPR_TX;
    return WSPR_TX;
}
```

- [ ] **Step 5: Run — expect pass.** **Step 6: Commit** `feat: wspr_beacon_decide (host-tested HOLD/WAIT/TX brain)`.

---

## Task 2 (HARDWARE): Bootstrap the Mini-WSPR ESP-IDF project

Copy the Mini-FT8 ESP-IDF skeleton into Mini-WSPR (build system, `components/` for M5 + USB-host
+ `ch340_usb_serial`, `main/` infra, partition table, `sdkconfig.defaults`, `idf_component.yml`
pinned `cdc_acm ==2.2.0`). **Strip FT8:** delete `components/ft8_lib`, `ft8_audio_pipeline.*`,
`autoseq.*`, the waterfall/RX-list UI, `resample.*`, and the FT8 decode path in `main.cpp`.
Keep `audio_trusdx_serial`, `radio_trusdx`, `radio_control*`, `gps`, `stream_*`, the M5/USB-host
stack. Add the Plan 1–2 modules + `wspr_beacon` to `main/CMakeLists.txt`.
**Verify:** `. C:\esp\esp-idf\export.ps1; idf.py build` is clean. **Gotchas** (from Mini-FT8 notes):
`idf.py fullclean` after any managed-component version change; keep `cdc_acm` pinned `==2.2.0`;
mixed-EOL `idf_component.yml`.

## Task 3 (HARDWARE): Generalize the TX synth path — `trusdx_begin_tx_plan`

Refactor `audio_trusdx_serial.cpp`'s `tx_task` to drive `tx_synth` instead of the inline FT8 math.
New entry `trusdx_begin_tx_plan(symbols, count, symbol_seconds, tone_spacing_hz, base_hz, anchor_ms)`.
**Keep `trusdx_serial_begin_ft8_tx` working** (build a 79/0.160/6.25 plan internally) — the FT8 path
must stay byte-identical (Plan 2's parity test is the spec).

**WSPR keying/timing (Codex Part B — refined):**
- Use a **fixed, bounded `TX_SETUP_LEAD_MS`** device constant (~1500 ms), NOT the scheduler's
  `prekey_lead`. Do not let the key-up time float from config (prekeying 30–120 s early
  needlessly stress-tests the PA).
- Sequence: set dial freq + USB mode **early**; at `anchor_ms − TX_SETUP_LEAD_MS` send
  `UA1;TX0;` (key up); confirm you are **still before the anchor**; then start audio at the
  target time. (Waiting *before* `TX0;` lets CAT/USB latency eat into the anchor — wait *after*.)
- **First-audio-byte-write time ≠ RF audio onset** (USB host + CDC + ATmega buffering + synth add
  latency). Add a calibrated `TX_AUDIO_ADVANCE_MS` measured from RF loopback (tune to drive
  `wsprd` `DT`→0); start the first byte at `anchor − TX_AUDIO_ADVANCE_MS`.
- Option to evaluate on a dummy load: stream **centered silence (0x80)** during the short
  prekey window, then switch to WSPR samples at the anchor — keeps the audio/serial path alive
  and makes timing sample-driven (SSB should emit little/no RF for DC-centered silence).
- Define the **late threshold numerically** (start ~tens of ms; if the first byte cannot be
  queued within it, abort the slot via `;RX;` — never partial-start a WSPR frame). Tune from
  `wsprd` `DT`.
- Keep `sent_samples`-driven pacing, the 100 ms drain + `;RX;`×3 key-off, and `0x3B→0x3A`
  stuffing **outside** the pure `tx_synth`.

**Verify (host first):** Plan 2 FT8 parity test still passes. **Then hardware:** RF loopback (below).

## Task 4 (HARDWARE): GPS freshness source + scheduler wiring

In `gps.cpp`, track the monotonic timestamp of the last *valid RMC* (status `A`) and expose a
`wspr_fix_src_t` snapshot (`have_valid_rmc`, `rmc_mono_ms`, `epoch_at_rmc_ms` via
`wspr_gps_to_epoch_ms(date_utc, time_utc)`, `have_grid`) + the live `grid_square`. Drive the main
loop from `wspr_beacon_decide(cfg, &src, grid, esp_timer_ms, MAX_AGE, PREKEY_LEAD, &plan)`:
`HOLD`→show status; `WAIT`→countdown; `TX`→`trusdx_begin_tx_plan(plan.symbols, 162, 0.682667,
12000.0/8192.0, plan.base_hz, plan.anchor_ms)` after setting dial `plan.dial_hz` + USB mode.

## Task 5 (HARDWARE): `wspr_config` (NVS) + status UI

NVS-persisted callsign / grid-override / power-dBm / band / duty-period / base-hz (reuse Mini-FT8's
station-data NVS + STATUS-menu edit pattern); default `duty.phase = wspr_duty_phase_for_call(call,
period)`. Status screen: callsign · grid · band/dial · power; big `NEXT TX mm:ss` (from
`plan.anchor_ms`) / `● TX nn/110 s`; session TX counter; "check wsprnet.org" line. Reuse the `G`
GPS screen.

## Task 6 (HARDWARE): Build + flash

`idf.py build` clean; flash via download mode (hold BOOT, tap Reset ~2 s → COM port; `idf.py -p
COMx flash`; plain Reset to boot). Pre-flight: truSDX **menu 8.3 reference calibration**, correct
**LPF + resonant antenna**, conservative power (~1–2 W).

## Task 7 (HARDWARE): On-air verification (the real gates)

1. **RF loopback:** truSDX → dummy load → RTL-SDR → `wsprd` decodes the live beacon. **Measure
   absolute carrier frequency and drift across the full 110.6 s frame, and `wsprd` `DT`** (use it
   to calibrate `TX_AUDIO_ADVANCE_MS` from Task 3 so `DT`→0).
2. **Serial-pacing soak:** ≥10 back-to-back transmissions into a dummy load. Log, per slot:
   timestamps before/after `UA1`, before/after `TX0`, first audio write, last audio write;
   total bytes + stuffed-byte count; underruns / blocking-write stalls; actual wall-clock stream
   duration vs the expected 110.592 s; key-off time; measured RF start offset; `wsprd` `DT`,
   frequency error, and drift per frame. Confirm no carrier drop.
3. **Forced abort/stall test:** trigger a cancel and a simulated stall mid-frame; prove the
   `;RX;` cleanup runs and the DTR-pulse recovery revives a forced stuck-TX.
4. **Thermal/power sanity** at the configured power and 20 % duty.
5. **On-air:** real antenna → transmit → your spots appear on **wsprnet.org** with distance + SNR.

---

## Self-Review
- Task 1 is pure + host-tested (HOLD/WAIT/TX, override, bad-band, freshness, anchor) — verified off-hardware.
- Tasks 2–7 preserve the RF-proven FT8 path (parity test), bake in Codex's pre-key/wait-for-anchor + skip-late, freshness-gate GPS, per-call duty phase, and the long-burst soak. All hardware verification is explicit and gated on the physical rig.
- The Plan 1–2 modules drop in unchanged; no re-implementation.
