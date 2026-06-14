// Mini-WSPR beacon application entry.
//
// A minimal TX-only WSPR beacon loop built on the proven Mini-FT8 truSDX backend
// (connect + CAT-streaming TX) and the host-verified WSPR modules. It replaces the
// FT8 app_main. The precise pre-key/wait-for-anchor TX timing (Plan 3 Task 3) and
// NVS config + UI (Task 5) are tuned on hardware; this compiles, wires the real APIs
// and the verified wspr_beacon_decide() brain, and keeps the early-start bounded
// within WSPR's start tolerance.
#include <cstdio>
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

    radio_control_set_backend(RADIO_CONTROL_TRUSDX_CAT);
    trusdx_serial_start();

    std::snprintf(g_cfg.callsign, sizeof(g_cfg.callsign), "%s", "N0CALL");
    g_cfg.grid_override[0] = '\0';
    g_cfg.power_dbm = 33;            // ~2 W; honest power set on the rig
    g_cfg.band = "20m";
    g_cfg.duty.period = 5;           // ~20% duty
    g_cfg.duty.phase = wspr_duty_phase_for_call(g_cfg.callsign, g_cfg.duty.period);
    g_cfg.base_hz_desired = 1500.0;

    const int64_t MAX_AGE_MS = 5000;     // GPS fix freshness limit
    // Small lead: tx_task has no in-task wait-for-anchor yet (Plan 3 Task 3), so the
    // frame starts up to PREKEY_LEAD_MS early — kept well inside WSPR's ~1-2 s tolerance.
    const int64_t PREKEY_LEAD_MS = 250;
    uint32_t tx_count = 0;
    int64_t  last_tx_anchor = -1;

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

        if (act == WSPR_TX && plan.anchor_ms != last_tx_anchor && !trusdx_serial_ft8_tx_active()) {
            if (radio_control_sync_frequency_mode((int)plan.dial_hz) == ESP_OK &&
                trusdx_begin_tx_plan(plan.symbols, 162, 8192.0 / 12000.0, 12000.0 / 8192.0,
                                     (int)plan.base_hz, plan.anchor_ms, utc_now) == ESP_OK) {
                last_tx_anchor = plan.anchor_ms;   // latch: do not re-key this slot
                tx_count++;
            }
        }

        M5.Display.fillRect(0, 40, 240, 90, TFT_BLACK);
        M5.Display.setCursor(0, 40);
        M5.Display.printf("%s %s\n", g_cfg.callsign,
                          src.have_grid ? gs.grid_square.c_str() : "----");
        M5.Display.printf("%s  TX:%u\n", g_cfg.band, (unsigned)tx_count);
        if (act == WSPR_HOLD) {
            M5.Display.print("waiting GPS");
        } else if (act == WSPR_WAIT) {
            long secs = (long)((plan.anchor_ms - utc_now) / 1000);
            M5.Display.printf("next TX %lds", secs);
        } else {
            M5.Display.print("** TX **");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(beacon_task, "wspr_beacon", 8192, nullptr, 5, nullptr, 0);
}
