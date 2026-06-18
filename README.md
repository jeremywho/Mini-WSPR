# Mini-WSPR

Portable, GPS-disciplined **WSPR transmit beacon** for the M5 Cardputer ADV (ESP32-S3)
driving a (tr)uSDX QRP radio over a single USB cable (PE1NNZ CAT streaming).

Transmit WSPR on one band in the field, then read how far your signal reached on
[wsprnet.org](https://wsprnet.org). Sibling project to
[Mini-FT8](https://github.com/jeremywho/Mini-FT8) — reuses its truSDX CAT-streaming, GPS,
and USB-host plumbing.

**Status: working and hardware-verified.** The beacon boots, connects the rig, GPS-locks,
and keys a clean WSPR frame; an independent RTL-SDR + `wsprd` decoded its own transmission
off-air as **`K7DYX EL09 33`** (callsign / grid / dBm), in-window. The remaining goal is a
real-antenna on-air reach test reported back by WSPRnet.

## v1 at a glance

- TX-only beacon (callsign + 4-char grid + power). Receiving is done by WSPRnet.
- Single band, etiquette-aware (~20% duty), GPS time + grid, randomized in-window offset.
- Clean-room WSPR encoder (MIT-licensed, no GPL dependency), host-tested round-trip vs `wsprd`.
- **Manual / auto TX control** (like WSJT-X): boots **quiet**; **Enter** arms one transmit on
  the next even-minute slot; **`a`** toggles continuous ~20%-duty beaconing.
- **PA-safety watchdog**: a frame is ~110.6 s; if the rig stays keyed past ~118 s the firmware
  forces key-off so a stalled TX can't sit on the air or cook the PA.
- **G0/BOOT** returns to the [cardputer-launcher](https://github.com/jeremywho/cardputer-launcher)
  menu when flashed as part of that OTA suite; a no-op flashed standalone.

## Build & flash

```bash
. C:\esp\esp-idf\export.ps1        # ESP-IDF 5.4.x
idf.py build                       # -> build/mini_wspr.bin
idf.py -p <PORT> flash monitor     # standalone
```

As part of the launcher suite the same `build/mini_wspr.bin` is flashed into the `ota_1`
slot — see the launcher's
[flash-image bundle](https://github.com/jeremywho/cardputer-launcher/tree/master/flash-image).

Operating: set callsign/grid/power/band in `main/wspr_app.cpp` (`g_cfg`), connect the
(tr)uSDX over USB, let GPS lock, then **Enter** (one TX) or **`a`** (continuous). Set honest
power on the rig to match the reported dBm.

## How it works

Host-verified modules (`main/wspr_*`) — encoder, scheduler, time/anchor, band plan, and the
`wspr_beacon_decide()` brain — drive the truSDX backend reused from Mini-FT8. The TX-path
internals, timing tolerances, and design rationale are documented in
[`docs/superpowers/specs/2026-06-13-wspr-beacon-design.md`](docs/superpowers/specs/2026-06-13-wspr-beacon-design.md)
and the plans alongside it — **read those before changing the synth or TX timing**, the
audio path has non-obvious sharing with the FT8 backend.

**TX synth lives inline in `audio_trusdx_serial.cpp` (`tx_task`)** — a continuous-phase MFSK NCO
(11520 Hz, 8-bit PCM, `0x3B→0x3A` CAT-stuffing), parameterized by `trusdx_begin_tx_plan`. Two
look-alike files are **not** the device path: `ft8_tx_synth.cpp` (uncompiled, dead) and `tx_synth.c`
(compiled but host-test-only — it validates the encoder against `wsprd`, it does not drive TX).
Full module map + dev workflow: **[CLAUDE.md](CLAUDE.md)**.

## License

MIT — see [LICENSE](LICENSE).
