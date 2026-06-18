# CLAUDE.md — Mini-WSPR

Session/dev guide. See **[README.md](README.md)** for the overview and
**[docs/superpowers/specs/2026-06-13-wspr-beacon-design.md](docs/superpowers/specs/2026-06-13-wspr-beacon-design.md)**
for the design rationale; this file is the durable how-to for working on the code.

## What it is

A GPS-disciplined **WSPR TX-only beacon** for the M5 Cardputer ADV (ESP32-S3) driving a (tr)uSDX
over a single USB cable (PE1NNZ CAT streaming). Bootstrapped from the Mini-FT8 skeleton; reuses its
truSDX/GPS/USB-host backend. Built + hardware-verified — decoded off-air as `K7DYX EL09 33`.

## Module map — device path vs host-test-only

The host-verifiable logic was written and tested on the host FIRST, then wired into firmware.
**Two synth files are NOT the device TX path — know this before editing:**

| File | Role |
|---|---|
| `main/wspr_app.cpp` | beacon app (`app_main`): M5 + GPS + truSDX connect + `wspr_beacon_decide()` loop + keying + TX UI |
| `main/wspr_beacon.c` | `wspr_beacon_decide()` — pure HOLD/WAIT/TX decision (time freshness + sched + band + encode) |
| `main/wspr_encode.c` | clean-room WSPR Type-1 encoder (162 symbols) |
| `main/wspr_{sched,time,band}.c` | even-minute/duty scheduler · GPS-epoch + RMC freshness · dial table |
| `main/audio_trusdx_serial.cpp` | **the REAL device TX synth** (inline in `tx_task`, below) + truSDX backend |
| `main/tx_synth.c` | ⚠ compiled but **ZERO device callers** — host `wsprd` round-trip only. Editing it does NOT change device TX |
| `main/ft8_tx_synth.cpp` | ⚠ **DEAD** — not in `main/CMakeLists.txt`, never compiled. Leftover skeleton |

### The real TX synth (verified by reading the code)

Device WSPR/FT8 audio is generated **inline in `audio_trusdx_serial.cpp`'s `tx_task`** — a
continuous-phase MFSK NCO: `tone_hz = base_hz + tone_spacing × symbol[idx]`,
`phase += 2π·tone_hz/11520`, `sample = 128 + AMP·sinf(phase)` (8-bit PCM @ 11520 Hz), byte-stuffed
`0x3B→0x3A` (truSDX uses `;` as its CAT delimiter). It DEFAULTS to FT8 (6.25 Hz, FT8 tone-seconds);
`trusdx_begin_tx_plan(symbols, count, symbol_seconds, tone_spacing, …)` overrides those for WSPR
(162 sym, ~1.465 Hz). **To change the on-air waveform edit `tx_task` — NOT `tx_synth.c` / `ft8_tx_synth.cpp`.**

## Build / flash / test

```bash
# Host tests (no idf, no make — pure C):
sh host_mock/build_and_test.sh     # encoder unit tests + golden vectors
sh host_mock/build_device.sh       # device-logic tests
sh host_mock/roundtrip_wspr.sh     # render audio -> wsprd decodes it back (the RF-proof gate)
```
```powershell
# Device firmware:
. C:\esp\esp-idf\export.ps1        # ESP-IDF 5.4.x
idf.py build                       # -> build/mini_wspr.bin
idf.py -p <COM> flash              # download mode: hold BOOT/G0, tap RESET, release; plain RESET to boot
```

`wsprd` (WSJT-X, `C:\WSJT\wsjtx\bin\wsprd.exe`) is the independent decode gate. RTL-SDR live decode:
`Ft8DotNet.Cli.exe --live --protocol wspr --device 0` (kill stale instances first).

## Changing config / band / timing

- **Callsign / grid / power / band:** `g_cfg` in `main/wspr_app.cpp` (compiled-in; NVS edit UI is a
  TODO). Set honest power on the rig to match the reported dBm.
- **TX control / duty:** boots QUIET; **Enter** = one TX next even-minute, **`a`** = continuous ~20%.
  The `auto_mode` / `tx_armed` gating is in the `wspr_app.cpp` loop.
- **Dial frequencies:** `wspr_band.c` table.
- **Start timing:** the frame can start up to `PREKEY_LEAD_MS` (250 ms) early — `tx_task` has no
  in-task wait-for-anchor yet. Calibrate from the `wsprd` DT (refine `TX_SETUP_LEAD` / add advance).

## Gotchas (hard-won)

- **Long-burst risk:** TX audio = 11,520 B/s over 115,200-baud 8N1 = the raw line rate, ZERO
  headroom, sustained ~111 s. A PA-safety watchdog forces key-off past ~118 s. Dummy-load soak
  before on-air. (A stuck key in testing was a BROKEN DUMMY LOAD, not a firmware stall.)
- **GPS is data, not a timing contract:** `valid_fix` isn't cleared when NMEA stops. Timing uses an
  RMC-specific `esp_timer` stamp captured when `gps.time_utc` changes (freshness-gated) — don't
  schedule off the `HH:MM:SS` strings.
- **truSDX:** DTR is wired to the ATmega RESET (keep deasserted); `UA2;` crashes its firmware (use
  `UA1;`); needs R2.00x beta firmware. Full truSDX protocol facts live in the
  [Mini-FT8](https://github.com/jeremywho/Mini-FT8) `CLAUDE.md` / `TRUSDX_FACTS.md`.

## State

Working + hardware-verified; merged to `master`. **Only remaining = the real-antenna on-air WSPRnet
reach test** (swap dummy load → 20 m antenna → `a`, check wsprnet.org for K7DYX). v2 = on-device RX
(deferred — needs a ~2-min buffer + Fano decoder, memory-bound on this chip).
