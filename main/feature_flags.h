#pragma once

// ---------------------------------------------------------------------------
// Build-time feature flags for Mini-FT8.
//
// Override at cmake time:
//   idf.py build -DENABLE_FT4=OFF    # FT8-only, no mode-switch menu item
//   idf.py build -DENABLE_BLE=OFF    # disable BLE regardless of protocol
// ---------------------------------------------------------------------------

// ── 1. Protocol support ───────────────────────────────────────────────────────
//
// ENABLE_FT4 — include FT4 protocol code and the runtime Mode menu item.
// Default ON (set by CMakeLists.txt).  When ON, the user can select FT8 or
// FT4 from Settings → Mode; the choice is saved to Station.txt and applied
// at next boot.
//
// When OFF, only FT8 is available and no Mode menu item is shown.
#ifndef ENABLE_FT4
#define ENABLE_FT4 1
#endif

// ── 2. Optional features ─────────────────────────────────────────────────────

// ENABLE_BLE — compile in NimBLE controller + GATT UI service.
// Both BLE and FT4 code are compiled into the same binary by default.
// Runtime enforcement: if the user has selected FT4 mode in Station.txt,
// BLE initialisation is skipped at boot (NimBLE consumes ~50 KB DRAM that
// the FT4 LDPC decoder needs).  See apply_ble_enabled_policy() in main.cpp.
#ifndef ENABLE_BLE
#define ENABLE_BLE 1
#endif

