#include "wspr_time.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// days from civil 1970-01-01 (Howard Hinnant), valid for the Gregorian calendar.
static int64_t days_from_civil(int y, int m, int d) {
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int64_t yoe = (int64_t)(y - era * 400);
    int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + (d - 1);
    int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}
static bool digits(const char* p, int n) {
    for (int i = 0; i < n; i++) if (!isdigit((unsigned char)p[i])) return false;
    return true;
}
static bool valid_dom(int y, int mo, int d) {
    static const int dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (mo < 1 || mo > 12 || d < 1) return false;
    int max = dim[mo - 1];
    if (mo == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) max = 29;  // leap February
    return d <= max;
}

bool wspr_gps_to_epoch_ms(const char* date_ymd, const char* time_hms, int64_t* out_ms) {
    if (!date_ymd || !time_hms || !out_ms) return false;
    if (strlen(date_ymd) != 10 || date_ymd[4] != '-' || date_ymd[7] != '-') return false;
    if (strlen(time_hms) != 8 || time_hms[2] != ':' || time_hms[5] != ':') return false;
    if (!digits(date_ymd, 4) || !digits(date_ymd + 5, 2) || !digits(date_ymd + 8, 2)) return false;
    if (!digits(time_hms, 2) || !digits(time_hms + 3, 2) || !digits(time_hms + 6, 2)) return false;
    int y = atoi(date_ymd), mo = atoi(date_ymd + 5), d = atoi(date_ymd + 8);
    int h = atoi(time_hms), mi = atoi(time_hms + 3), s = atoi(time_hms + 6);
    if (!valid_dom(y, mo, d) || h > 23 || mi > 59 || s > 60) return false;  // s==60 allows a leap second
    int64_t days = days_from_civil(y, mo, d);
    int64_t secs = days * 86400 + h * 3600 + mi * 60 + s;
    *out_ms = secs * 1000;
    return true;
}
wspr_fix_t wspr_fix_now(const wspr_fix_src_t* src, int64_t now_mono_ms, int64_t max_age_ms) {
    wspr_fix_t f = { false, 0 };
    if (!src || !src->have_valid_rmc || !src->have_grid) return f;
    int64_t age = now_mono_ms - src->rmc_mono_ms;
    if (age < 0 || age > max_age_ms) return f;
    f.usable = true;
    f.utc_ms = src->epoch_at_rmc_ms + age;
    return f;
}
