# Mini-WSPR

Portable, GPS-disciplined **WSPR transmit beacon** for the M5 Cardputer ADV (ESP32-S3)
driving a (tr)uSDX QRP radio over a single USB cable (PE1NNZ CAT streaming).

Transmit WSPR on one band in the field, then read how far your signal reached on
[wsprnet.org](https://wsprnet.org). Sibling project to
[Mini-FT8](https://github.com/wcheng95/Mini-FT8) — reuses its truSDX CAT-streaming,
GPS, and USB-host plumbing.

**Status:** Design phase. See
[`docs/superpowers/specs/2026-06-13-wspr-beacon-design.md`](docs/superpowers/specs/2026-06-13-wspr-beacon-design.md).

## v1 at a glance

- TX-only beacon (callsign + 4-char grid + power). Receiving is done by WSPRnet.
- Single band, etiquette-aware (~20% duty), GPS time + grid, randomized in-window offset.
- Clean-room WSPR encoder (MIT-licensed, no GPL dependency).

## License

MIT — see [LICENSE](LICENSE).
