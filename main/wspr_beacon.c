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
