// Mini-WSPR beacon application entry.
//
// A minimal TX-only WSPR beacon loop built on the proven Mini-FT8 truSDX backend
// (connect + CAT-streaming TX) and the host-verified WSPR modules. It replaces the
// FT8 app_main. The precise pre-key/wait-for-anchor TX timing (Plan 3 Task 3) and
// NVS config + UI (Task 5) are tuned on hardware; this compiles, wires the real APIs
// and the verified wspr_beacon_decide() brain, and keeps the early-start bounded
// within WSPR's start tolerance.
#include <cstdio>
#include <cstring>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "M5Cardputer.h"

#include "gps.h"               // C++ header (std::string in gps_state_t)
#include "audio_trusdx_serial.h"
#include "radio_control.h"
#include "wspr_beacon.h"
#include "wspr_sched.h"
#include "wspr_time.h"

static const char* TAG = "wspr_app";

// v1 config is compiled in; NVS + on-device editing is Plan 3 Task 5 (hardware).
static wspr_cfg_t g_cfg;

// Connect to the truSDX off the UI thread so a slow/absent rig never freezes the
// display or GPS. Retries until ready, so plug order doesn't matter; once connected
// trusdx_serial_start() is a no-op. The beacon loop gates TX on readiness.
static void connect_task(void*) {
    radio_control_set_backend(RADIO_CONTROL_TRUSDX_CAT);
    for (;;) {
        if (!trusdx_serial_is_ready()) trusdx_serial_start();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

static void beacon_task(void*) {
    auto m5cfg = M5.config();
    M5Cardputer.begin(m5cfg);
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 0);
    M5.Display.print("Mini-WSPR");

    nvs_flash_init();
    gps_start(115200);

    xTaskCreatePinnedToCore(connect_task, "wspr_connect", 4096, nullptr, 4, nullptr, 0);

    std::snprintf(g_cfg.callsign, sizeof(g_cfg.callsign), "%s", "K7DYX");
    g_cfg.grid_override[0] = '\0';
    g_cfg.power_dbm = 33;            // ~2 W; honest power set on the rig
    g_cfg.band = "20m";
    g_cfg.duty.period = 5;           // ~20% duty: 1 TX per 5 even minutes (~every 10 min), polite WSPR
    g_cfg.duty.phase = wspr_duty_phase_for_call(g_cfg.callsign, g_cfg.duty.period);
    g_cfg.base_hz_desired = 1500.0;

    const int64_t MAX_AGE_MS = 5000;     // GPS fix freshness limit
    // Small lead: tx_task has no in-task wait-for-anchor yet (Plan 3 Task 3), so the
    // frame starts up to PREKEY_LEAD_MS early — kept well inside WSPR's ~1-2 s tolerance.
    const int64_t PREKEY_LEAD_MS = 250;
    uint32_t tx_count = 0;
    int64_t  last_tx_anchor = -1;
    int64_t  tx_started_mono = 0;   // PA-safety watchdog: when the current TX keyed (0 = idle)
    bool     auto_mode = false;     // false = manual (Enter fires one TX); true = continuous ~20%
    bool     tx_armed = false;      // manual: a TX is armed for the next even-minute slot
    char     last_key = 0;          // keyboard debounce

    // RMC-specific monotonic anchor: gps.time_utc only changes on a valid RMC, so a
    // change marks a fresh RMC. Stamp esp_timer then (same clock as now_mono) and keep
    // the matching epoch — this is the freshness source wspr_fix_now() needs.
    std::string last_rmc_time;
    int64_t rmc_mono_ms = 0;
    int64_t rmc_epoch_ms = 0;

    ESP_LOGI(TAG, "beacon loop start (%s %s p=%d)", g_cfg.callsign, g_cfg.band, g_cfg.duty.phase);

    while (true) {
        gps_tick();
        gps_state_t gs = gps_get_state();
        int64_t now_mono = esp_timer_get_time() / 1000;

        // Keyboard control: Enter arms ONE transmit (next cycle); 'a' toggles continuous ~20% auto.
        M5Cardputer.update();
        M5Cardputer.Keyboard.updateKeysState();
        auto &kb = M5Cardputer.Keyboard.keysState();
        char key = kb.enter ? '\n' : (!kb.word.empty() ? kb.word.back() : 0);
        if (key && key != last_key) {
            if (key == '\n') tx_armed = true;
            else if (key == 'a' || key == 'A') auto_mode = !auto_mode;
        }
        last_key = key;
        // Manual uses every even minute as a candidate (the arm gates it); auto = polite 1-in-5.
        g_cfg.duty.period = auto_mode ? 5 : 1;
        g_cfg.duty.phase  = auto_mode ? wspr_duty_phase_for_call(g_cfg.callsign, 5) : 0;

        int64_t e = 0;
        if (gs.valid_fix && !gs.time_utc.empty() && gs.time_utc != last_rmc_time &&
            wspr_gps_to_epoch_ms(gs.date_utc.c_str(), gs.time_utc.c_str(), &e)) {
            last_rmc_time = gs.time_utc;
            rmc_mono_ms   = now_mono;
            rmc_epoch_ms  = e;
        }

        wspr_fix_src_t src;
        src.have_valid_rmc  = (rmc_mono_ms != 0) && gs.valid_fix;
        src.rmc_mono_ms     = rmc_mono_ms;
        src.epoch_at_rmc_ms = rmc_epoch_ms;
        src.have_grid       = !gs.grid_square.empty();

        wspr_plan_t plan;
        wspr_action_t act = wspr_beacon_decide(&g_cfg, &src, gs.grid_square.c_str(),
                                               now_mono, MAX_AGE_MS, PREKEY_LEAD_MS, &plan);

        // current UTC projected on the same monotonic clock the fix was stamped with.
        int64_t utc_now = (rmc_mono_ms != 0) ? rmc_epoch_ms + (now_mono - rmc_mono_ms) : 0;

        if (act == WSPR_TX && (auto_mode || tx_armed) &&
            plan.anchor_ms != last_tx_anchor && !trusdx_serial_ft8_tx_active()) {
            if (radio_control_sync_frequency_mode((int)plan.dial_hz) == ESP_OK &&
                trusdx_begin_tx_plan(plan.symbols, 162, 8192.0 / 12000.0, 12000.0 / 8192.0,
                                     (int)plan.base_hz, plan.anchor_ms, utc_now) == ESP_OK) {
                last_tx_anchor = plan.anchor_ms;   // latch: do not re-key this slot
                tx_armed = false;                  // manual one-shot: disarm after firing
                tx_count++;
            }
        }

        // PA-safety watchdog: a WSPR frame is 110.6 s; if the rig stays keyed past ~118 s
        // something stalled — force a key-off so a stuck TX can't sit on the air / cook the PA.
        if (trusdx_serial_ft8_tx_active()) {
            if (tx_started_mono == 0) tx_started_mono = now_mono;
            else if (now_mono - tx_started_mono > 118000) {
                trusdx_serial_cancel_ft8_tx();
                ESP_LOGW(TAG, "TX watchdog: keyed >118s, forcing key-off");
            }
        } else {
            tx_started_mono = 0;
        }

        // Status, redrawn ONLY when it changes. Bg-colored text + fixed-width padding
        // overwrite in place (no clear-to-black), so the display does not flicker.
        char tmp[24], l1[24], l2[24], l3[24];
        std::snprintf(tmp, sizeof tmp, "%s %s", g_cfg.callsign,
                      src.have_grid ? gs.grid_square.c_str() : "----");
        std::snprintf(l1, sizeof l1, "%-18s", tmp);
        std::snprintf(tmp, sizeof tmp, "%s rig:%s TX:%u", g_cfg.band,
                      trusdx_serial_is_ready() ? "OK" : "..", (unsigned)tx_count);
        std::snprintf(l2, sizeof l2, "%-18s", tmp);
        long secs = (long)((plan.anchor_ms - utc_now) / 1000);
        if (trusdx_serial_ft8_tx_active()) std::snprintf(tmp, sizeof tmp, "** TX **");
        else if (act == WSPR_HOLD)         std::snprintf(tmp, sizeof tmp, "waiting GPS");
        else if (tx_armed)                 std::snprintf(tmp, sizeof tmp, "ARMED %lds", secs);
        else if (auto_mode)                std::snprintf(tmp, sizeof tmp, "AUTO %lds", secs);
        else                               std::snprintf(tmp, sizeof tmp, "Enter=TX a=auto");
        std::snprintf(l3, sizeof l3, "%-18s", tmp);

        static char p1[24] = {0}, p2[24] = {0}, p3[24] = {0};
        if (std::strcmp(l1, p1) || std::strcmp(l2, p2) || std::strcmp(l3, p3)) {
            M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
            M5.Display.setCursor(0, 40); M5.Display.print(l1);
            M5.Display.setCursor(0, 62); M5.Display.print(l2);
            M5.Display.setCursor(0, 84); M5.Display.print(l3);
            std::strcpy(p1, l1); std::strcpy(p2, l2); std::strcpy(p3, l3);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(beacon_task, "wspr_beacon", 8192, nullptr, 5, nullptr, 0);
}
