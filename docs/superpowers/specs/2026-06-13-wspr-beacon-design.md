# WSPR Beacon for M5 Cardputer ADV + (tr)uSDX — v1 Design Spec

**Date:** 2026-06-13
**Status:** Design approved (pending user spec review) → next step: implementation plan
**Sibling project:** Mini-FT8 (`C:\Data\Repos\Mini-FT8`) — reused as the code skeleton

---

## 1. Goal

A portable, GPS-disciplined **WSPR transmit beacon** running on the M5 Cardputer ADV
(ESP32-S3), driving a (tr)uSDX QRP radio over a single USB cable via PE1NNZ
"CAT streaming" (CAT commands + 8-bit PCM audio multiplexed on one USB-CDC link).

The operating goal: go out portable, transmit WSPR on one band, and read how far the
signal reaches on **wsprnet.org** from a phone. WSPR's global receiver network does the
receiving and distance/SNR mapping — the device only needs to transmit.

## 2. Scope

**In (v1):**
- Transmit a standard (Type 1) WSPR message: `callsign + 4-char grid + power(dBm)`.
- Single band, etiquette-aware duty cycle (~20%, configurable).
- GPS-disciplined timing (TX at even-UTC-minute + 1.0 s) and GPS-sourced grid.
- On-device config (callsign / power / band / duty) persisted to NVS.
- Status screen + a host/bench/RF test suite.

**Out (deferred to v2):**
- On-device WSPR **receive/decode** (re-evaluated after field use).
- Multi-band cycling.
- WiFi convenience (e.g. auto-pulling your own spots from wsprnet).

## 3. Locked decisions

| Decision | Choice | Rationale |
|---|---|---|
| Scope | TX-only beacon, RX deferred | WSPRnet does the receiving; nails the goal with a fraction of the work and RAM. |
| Repo | New standalone repo from the Mini-FT8 skeleton, FT8 stripped | Reuses the proven ESP-IDF/M5/USB-host build + flash infra; keeps Mini-FT8's fork diff untouched. |
| Operating model | Single band, ~20% duty, randomized audio offset | Antenna is single-band in the field anyway; plays nice on shared WSPR frequencies. |
| Encoder | Clean-room from the WSPR protocol + canonical test vector | Verifiable byte-for-byte; keeps the repo permissively licensed (MIT), no GPL entanglement. |

This design was reviewed by an independent model (Codex, xhigh) which verified the
reuse claims against the real Mini-FT8 source. Its corrections are folded in below
(see §5 and §7 especially).

## 4. WSPR protocol facts (the numbers we must hit)

- **Message:** 28-bit callsign + 15-bit grid + 7-bit power = **50 source bits** (Type 1).
- **FEC:** convolutional code, constraint length **K=32, rate 1/2**, generator polynomials
  `0xf2d05351` and `0xe4613c47`; 50 bits + 31 zero tail → 162 encoded bits.
- **Interleave:** bit-reversal permutation over the 162 bit positions.
- **Sync:** fixed 162-bit pseudo-random sync vector. Channel symbol `i` =
  `sync[i] + 2 * data_interleaved[i]` → value in **0..3** (4-FSK).
- **162 symbols**, 4-FSK. **Tone spacing = symbol rate = 12000 / 8192 = 1.46484 Hz**
  (MFSK: spacing equals baud). Symbol period = 8192 / 12000 = **0.682667 s**.
- **Duration ≈ 110.6 s** (162 × 0.682667).
- **TX window:** start **1.0 s after an even UTC minute** (tolerance ~1 s).
- **Power:** reported in dBm; **last digit must be ∈ {0, 3, 7}** (…27, 30, 33, 37…).
- **Sensitivity:** roughly −28 to −31 dB SNR (2500 Hz ref), ~7–10 dB better than FT8.

## 5. Hardware & corrected reuse map

Hardware: M5 Cardputer ADV (ESP32-S3, native USB host) + M5 LoRa+GPS Cap (ATGM336H GPS
on G13/G15) + (tr)uSDX (CH340 USB, R2.00x beta firmware, CAT streaming @ 115200).

**Reused from Mini-FT8 (proven, compiled, on-hardware):**
| Source | Reused for | Note |
|---|---|---|
| `main/audio_trusdx_serial.cpp/.h` | truSDX CAT streaming, TX keying (`UA1;TX0;` → `;RX;`), byte-stuffing, USB-host plumbing | **The synth inside it is hardcoded FT8 — must be generalized (see §7).** |
| `main/radio_trusdx.cpp`, `main/radio_control*.cpp` | Set dial frequency + USB mode via CAT | |
| `main/gps.cpp/.h` | UTC time + 4-char Maidenhead grid | **Needs a freshness-gated timing contract (see §8).** |
| ESP-IDF / M5 / USB-host project skeleton | Build + download-mode flash; `cdc_acm` pinned `==2.2.0`; DRAM budget check | |

**⚠ Correction (verified against code):** `main/ft8_tx_synth.cpp` — which exposes a clean
streaming synth API — is **not in `main/CMakeLists.txt`** and is **not compiled into the
firmware**. It is leftover host-test code. The RF-proven TX synthesis is **inline in
`audio_trusdx_serial.cpp`** and is hardcoded for FT8:
`TRUSDX_FT8_TONE_COUNT = 79`, `TRUSDX_FT8_TONE_SECONDS = 0.160`, tone spacing `6.25f`,
`TRUSDX_TX_SAMPLE_RATE 11520`, byte-stuffing `0x3B → 0x3A`, plus a slot-compensation term.
`ft8_tx_synth.cpp` may be used only as a **structural template**; anything built from it
must earn its own RF verification.

**Stripped from the skeleton:** `ft8_lib` decode + `ldpc.c` + `monitor.c` (the 80 KB static
waterfall), `ft8_audio_pipeline`, `autoseq`, RX list / waterfall UI, `resample` (RX-only).
This deletion buys back the RAM headroom (the FT8 decode path ran at ~8 KB free heap, no
PSRAM). TX-only WSPR has no comparable memory pressure.

## 6. Architecture (modules + interfaces)

Five well-bounded units. Each has one job and a narrow interface.

### 6.1 `wspr_encode` — message → symbols (new, clean-room)
```c
// Pack a standard (Type 1) WSPR message into 162 four-FSK symbols (values 0..3).
// Returns false if the inputs cannot be represented as a Type 1 message
// (compound/oversized callsign, malformed grid, or invalid dBm). On false, DO NOT
// transmit. v1 supports Type 1 ONLY (no /P, no compound, no Type 2/3 hashing).
bool wspr_encode_type1(const char* callsign, const char* grid4,
                       int power_dbm, uint8_t out_symbols[162]);
```
Internals: pack call (28b) + grid (15b) + power (7b) → convolutional encode (K=32, r=1/2)
→ interleave → merge sync vector → 162 symbols. Constants (generator polys, sync vector)
are public protocol facts. Power validated against the {0,3,7}-last-digit rule.

### 6.2 Generalized TX synth — symbols → streamed audio (generalize the truSDX path)
Replace the FT8-hardcoded `begin_ft8_tx` with a parameterized plan (or add a parallel
WSPR path that shares the keying/streaming/byte-stuffing):
```c
typedef struct {
    const uint8_t* symbols;     // values 0..(tones-1)
    int      count;             // 162 (WSPR) / 79 (FT8)
    double   symbol_seconds;    // 0.682667 (WSPR) / 0.160 (FT8)
    double   tone_spacing_hz;   // 1.46484  (WSPR) / 6.25  (FT8)
    double   base_hz;           // frequency of tone 0 (lowest tone)
    int64_t  start_anchor_ms;   // absolute intended start; waveform plays from symbol 0
} tx_symbol_plan_t;

void trusdx_begin_tx_plan(const tx_symbol_plan_t* plan);
```
Synth rules (corrected):
- **Sample rate = 11520 Hz** (the active path's `TRUSDX_TX_SAMPLE_RATE`), 8-bit unsigned PCM.
- **Fractional symbol length**: `samples_per_symbol = 11520 * 8192 / 12000 = 7864.32`.
  Derive the current symbol index from the **fractional sample position**, never a rounded
  integer SPS (avoids cumulative symbol-timing drift over 162 symbols).
- **`double` phase accumulator**, continuous across symbol boundaries (keeps the ~6 Hz
  occupied bandwidth). `phase += 2π * tone_hz / 11520` per sample.
- **Anchor at `start_anchor_ms`** (the intended WSPR start instant) and play from symbol 0.
  Do **not** reuse FT8's `(start - slot_start) * rate / 1000` slot-skip — for WSPR it would
  drop the first second of audio.
- **Byte-stuffing matches the proven path: `0x3B → 0x3A`** (not `0x3C`).

### 6.3 `wspr_beacon` — scheduler / state machine (new)
```
WAIT_FIX  → no usable GPS time/grid: hold, show "waiting for GPS".
WAIT_SLOT → have a usable fix: compute the next even-UTC-minute boundary.
DECIDE    → at the boundary, apply duty cycle (e.g. TX 1 of every 5 even minutes).
TX        → set band/mode via CAT → key (UA1;TX0;) → stream 162 symbols anchored at
            boundary+1.0s (~110.6 s) → key off (;RX;) → WAIT_SLOT.
SKIP      → show "next TX in mm:ss" countdown → WAIT_SLOT.
```
Duty is a deterministic fraction of even minutes, not random keying, so transmissions stay
aligned to the WSPR grid. A "force TX now" key triggers a one-shot at the next boundary.

### 6.4 `wspr_config` — settings (new, reuse Mini-FT8 NVS/STATUS-menu pattern)
Persisted: `callsign`, `power_dbm`, `band` (selects dial freq + LPF reminder), `duty_pct`,
optional manual `grid` override (default: live GPS grid). On-device edit reuses Mini-FT8's
in-place digit/char STATUS-menu editor.

### 6.5 `gps` timing contract (extend Mini-FT8 `gps`)
See §8 — add a freshness-gated `gps_get_time_fix()` returning a UTC↔monotonic anchor.

### 6.6 UI (new, minimal)
Main screen: callsign · grid · band/dial freq · power. Big **`NEXT TX mm:ss`** or
**`● TX 47 / 110 s`**. A session TX counter. A "check wsprnet.org" reminder line.
Keys: force-TX-now, config, GPS screen (reuse `G`).

## 7. TX signal chain (precise)

```
callsign+grid+dBm ─ wspr_encode_type1 ─► 162 symbols (0..3)
                                          │
   GPS UTC+1.0s anchor ─────────────────► trusdx_begin_tx_plan
                                          │  (11520 Hz, fractional SPS, double phase,
                                          │   base_hz=tone0, spacing 1.46484 Hz, 0x3B→0x3A)
                                          ▼
   8-bit PCM @ 11520 B/s ─ CAT-stream ─► (tr)uSDX ─ USB key UA1;TX0; … ;RX;
```

## 8. Timing & scheduling (corrected — GPS is data, not a contract)

The existing `gps` exposes `valid_fix`, a 4-char grid, and `"HH:MM:SS"` strings — but
`valid_fix` is **not** invalidated when NMEA stops, and `last_rx_ms` ticks on *any*
checksum-valid sentence. Scheduling a transmission off that is unsafe (TX in the wrong
WSPR window → nobody decodes, or worse, off-grid).

Add a freshness-gated snapshot:
```c
typedef struct {
    bool    usable;        // recent valid RMC AND fresh grid within a max-age window
    int64_t utc_unix_ms;   // UTC at the moment of capture
    int64_t mono_ms;       // esp_timer_get_time()/1000 at capture (the anchor clock)
    char    grid4[5];
} gps_time_fix_t;

gps_time_fix_t gps_get_time_fix(void);   // usable=false when stale
```
The scheduler computes the even-minute boundary from `utc_unix_ms` projected forward on the
monotonic `mono_ms` clock, and anchors TX at `boundary + 1000 ms`. `usable == false` → hold
TX. (If the system clock is already disciplined from GPS via `settimeofday`, the same
freshness gate applies before trusting it.)

## 9. Frequency plan & calibration

- Dial = standard **WSPR USB dial frequency** per band, e.g. 40m `7.038600`, 30m
  `10.138700`, 20m `14.095600`, 17m `18.104600`, 15m `21.094600` MHz. Mode USB.
- Audio: `base_hz` = **tone 0** (lowest tone). Random per-TX offset within the WSPR window;
  clamp so the highest tone (tone 3 = base + 3 × 1.46484 ≈ base + 4.4 Hz) stays **≤ ~1595 Hz**
  (i.e. base roughly in 1400–1590 Hz).
- **Calibration matters:** 100 Hz at 14 MHz ≈ 7 ppm. The truSDX Si5351 reference (menu 8.3)
  must be trimmed so the signal lands inside the 200 Hz WSPR sub-band with margin for drift.
- **Drift:** WSPR is more drift-sensitive than the 200 Hz window implies. Verify carrier
  drift over a full 110.6 s frame (see §12), not just "does it decode once".

## 10. Power & thermal

WSPR is ~7–10 dB more sensitive than FT8; transcontinental spots happen at milliwatts.
**Default power conservative (~1–2 W), configurable** — running the truSDX at 5 W key-down
for 110.6 s, repeatedly, is genuine continuous-duty stress on a QRP PA. Lower power is both
kinder to the rig and a more representative QRP propagation test. Power is reported honestly
in dBm (WSPRnet's reach-vs-power stats depend on it).

## 11. Reliability: the long-burst risk (first-class)

TX audio is **11,520 bytes/s**, and the truSDX CAT link is **115,200 baud 8N1 = 11,520
bytes/s** — i.e. the stream runs at the **raw serial line rate with zero headroom**,
sustained for ~110.6 s (vs FT8's proven ~13 s burst, ~8.5× shorter exposure). The active
path sends 64-byte chunks through blocking CH340 writes (256-byte OUT buffer). Any sustained
stall (USB-host task preemption, GPS UART ISR, display repaint) can underflow the rig and
drop carrier mid-frame → corrupt WSPR transmission.

Mitigations (required, not optional):
- Keep the TX feeder task high-priority and lean; minimize contending work during a frame.
- **Dummy-load soak: ≥10 consecutive transmissions** without carrier drop or stuck TX.
- **Stuck-TX recovery**: detect a wedged transmission and recover via the proven **DTR-pulse
  reset** (DTR↔ATmega RESET, ~80 ms high → low, wait ~3.5 s) already documented for Mini-FT8.
- Watch for underflow (feeder starved) and abort cleanly to `;RX;` rather than half-keying.

## 12. Test plan

1. **Encoder unit test (host):** encode the canonical `K1ABC FN42 37`; diff the 162 symbols
   **byte-for-byte** against the published WSPR reference vector. Add negative cases:
   lowercase/invalid callsign, malformed grid, invalid dBm (last digit ∉ {0,3,7}), `/P` and
   compound calls (must return `false`), and edge audio offsets.
2. **Synth bench test (host/loopback):** render a frame to a 12 kHz WAV; decode with WSJT-X
   / `wsprd`; must recover call+grid+power. Verify continuous phase / occupied bandwidth.
3. **RF loopback (mandatory gate):** truSDX → dummy load → RTL-SDR → `wsprd` decodes the live
   beacon (the same chain that proved FT8 TX). **Also measure absolute carrier frequency and
   drift across the full 110.6 s frame**, not just decode success.
4. **Reliability soak:** ≥10 back-to-back transmissions into a dummy load; confirm no carrier
   drop, and confirm DTR-pulse recovery from a forced stuck-TX.
5. **Thermal/SWR/power sanity:** confirm the PA tolerates the configured power at sustained
   duty without bias shift or excess heat; validate the conservative default.
6. **On-air:** real resonant antenna → transmit → spots appear on wsprnet.org.

## 13. Pre-flight checklist (field)

- [ ] truSDX reference frequency calibrated (menu 8.3) — the #1 cause of "nobody hears me".
- [ ] Correct **LPF** selected for the band; **resonant antenna or tuner** connected.
- [ ] Callsign / grid-source / power / band / duty confirmed on-device.
- [ ] GPS fix acquired (scheduler shows a usable fix, not "waiting for GPS").
- [ ] Power set conservatively for sustained duty.

## 14. Open risks / notes

- truSDX firmware fragility to unexpected CAT tokens (documented in Mini-FT8) — keep the
  WSPR keying sequence identical to the RF-proven FT8 sequence apart from the audio payload.
- The generalized `trusdx_begin_tx_plan` must not regress Mini-FT8's FT8 TX if any shared
  code is later merged back (it won't be in v1, but keep the interface FT8-compatible).
- Repo name `Mini-WSPR` and MIT license are defaults — easy to change before first push.

## 15. Repo bootstrap (post-approval, via the implementation plan)

New standalone repo (not a fork). Copy the Mini-FT8 ESP-IDF/M5/USB-host skeleton, strip the
FT8 decode/waterfall/autoseq modules (§5), add the five new units (§6). Initial feature work
happens on a branch/worktree per the standard workflow; only the design artifact lives on
the initial main commit.
