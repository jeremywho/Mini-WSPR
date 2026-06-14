// Mini-WSPR beacon application entry.
//
// A minimal TX-only WSPR beacon loop built on the proven Mini-FT8 truSDX backend
// (connect + CAT-streaming TX) and the host-verified WSPR modules. It replaces the
// FT8 app_main. Behavioural details (M5 init specifics, connect timing, the pre-key/
// wait-for-anchor TX timing per Plan 3 Task 3) are tuned on hardware; this compiles
// and wires the real APIs + the verified wspr_beacon_decide() brain.
#include <cstdio>
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
    const int64_t PREKEY_LEAD_MS = 1500; // TX setup lead (Plan 3 Task 3)
    uint32_t tx_count = 0;

    ESP_LOGI(TAG, "beacon loop start (%s %s p=%d)", g_cfg.callsign, g_cfg.band, g_cfg.duty.phase);

    while (true) {
        gps_tick();
        gps_state_t gs = gps_get_state();
        int64_t now_mono = esp_timer_get_time() / 1000;

        int64_t epoch = 0;
        wspr_fix_src_t src;
        src.have_valid_rmc = gs.valid_fix &&
            wspr_gps_to_epoch_ms(gs.date_utc.c_str(), gs.time_utc.c_str(), &epoch);
        src.rmc_mono_ms = (int64_t)gs.last_rx_ms;
        src.epoch_at_rmc_ms = epoch;
        src.have_grid = !gs.grid_square.empty();

        wspr_plan_t plan;
        wspr_action_t act = wspr_beacon_decide(&g_cfg, &src, gs.grid_square.c_str(),
                                               now_mono, MAX_AGE_MS, PREKEY_LEAD_MS, &plan);

        if (act == WSPR_TX && !trusdx_serial_ft8_tx_active()) {
            radio_control_sync_frequency_mode((int)plan.dial_hz);
            trusdx_begin_tx_plan(plan.symbols, 162, 8192.0 / 12000.0, 12000.0 / 8192.0,
                                 (int)plan.base_hz, plan.anchor_ms, plan.anchor_ms);
            tx_count++;
        }

        M5.Display.fillRect(0, 40, 240, 90, TFT_BLACK);
        M5.Display.setCursor(0, 40);
        M5.Display.printf("%s %s\n", g_cfg.callsign,
                          src.have_grid ? gs.grid_square.c_str() : "----");
        M5.Display.printf("%s  TX:%u\n", g_cfg.band, (unsigned)tx_count);
        if (act == WSPR_HOLD) {
            M5.Display.print("waiting GPS");
        } else if (act == WSPR_WAIT) {
            long secs = (long)((plan.anchor_ms - epoch) / 1000);
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
