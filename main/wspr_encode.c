#include "wspr_encode.h"
#include "wspr_encode_internal.h"
#include "wspr_sync.h"
#include <string.h>
#include <ctype.h>

int wspr_char_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'Z') return c - 'A' + 10;
    if (c == ' ') return 36;
    return -1;
}

bool wspr_normalize_call(const char* in, char out_field[6]) {
    if (!in) return false;
    char up[8]; int n = 0;
    for (const char* p = in; *p; ++p) {
        if (n >= 7) return false;                 // longer than 6 (+1) -> reject
        char c = (char)toupper((unsigned char)*p);
        if (c == '/') return false;               // compound/portable -> not Type 1
        up[n++] = c;
    }
    if (n < 3 || n > 6) return false;
    int shift = 0;
    if (!(up[2] >= '0' && up[2] <= '9')) {
        if (n >= 2 && up[1] >= '0' && up[1] <= '9') shift = 1;
        else return false;
    }
    for (int i = 0; i < 6; ++i) out_field[i] = ' ';
    for (int i = 0; i < n; ++i) {
        if (shift + i >= 6) return false;
        out_field[shift + i] = up[i];
    }
    if (wspr_char_val(out_field[0]) < 0) return false;
    char c1 = out_field[1];
    if (!((c1 >= '0' && c1 <= '9') || (c1 >= 'A' && c1 <= 'Z'))) return false;
    if (!(out_field[2] >= '0' && out_field[2] <= '9')) return false;
    for (int i = 3; i < 6; ++i)
        if (!(out_field[i] == ' ' || (out_field[i] >= 'A' && out_field[i] <= 'Z'))) return false;
    return true;
}

uint32_t wspr_pack_call(const char field[6]) {
    uint32_t n = (uint32_t)wspr_char_val(field[0]);
    n = n * 36 + (uint32_t)wspr_char_val(field[1]);
    n = n * 10 + (uint32_t)(field[2] - '0');
    n = n * 27 + (uint32_t)(wspr_char_val(field[3]) - 10);
    n = n * 27 + (uint32_t)(wspr_char_val(field[4]) - 10);
    n = n * 27 + (uint32_t)(wspr_char_val(field[5]) - 10);
    return n;  // 28 bits
}

bool wspr_pack_grid_power(const char* grid4, int dbm, uint32_t* out_m) {
    if (!grid4 || strlen(grid4) != 4) return false;
    char g[4];
    for (int i = 0; i < 4; ++i) g[i] = (char)toupper((unsigned char)grid4[i]);
    int g1 = g[0] - 'A', g2 = g[1] - 'A', g3 = g[2] - '0', g4 = g[3] - '0';
    if (g1 < 0 || g1 > 17 || g2 < 0 || g2 > 17) return false;   // field A..R
    if (g3 < 0 || g3 > 9  || g4 < 0 || g4 > 9)  return false;   // square 0..9
    if (dbm < 0 || dbm > 60) return false;
    int last = dbm % 10;
    if (!(last == 0 || last == 3 || last == 7)) return false;
    uint32_t m1 = (uint32_t)((179 - 10 * g1 - g3) * 180 + 10 * g2 + g4);
    *out_m = m1 * 128u + (uint32_t)(dbm + 64);
    return true;  // 22 bits
}

uint8_t wspr_parity32(uint32_t x) {
    x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
    return (uint8_t)(x & 1u);
}

void wspr_conv_encode(const uint8_t bits[50], uint8_t enc[162]) {
    uint32_t reg = 0;
    int k = 0;
    for (int i = 0; i < 81; ++i) {
        uint32_t b = (i < 50) ? (uint32_t)(bits[i] & 1u) : 0u;
        reg = (reg << 1) | b;
        enc[k++] = wspr_parity32(reg & 0xf2d05351u);
        enc[k++] = wspr_parity32(reg & 0xe4613c47u);
    }
}

uint8_t wspr_bitrev8(uint8_t b) {
    b = (uint8_t)((b >> 4) | (b << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

void wspr_interleave(const uint8_t enc[162], uint8_t out[162]) {
    int i = 0;
    for (int p = 0; p < 256 && i < 162; ++p) {
        uint8_t j = wspr_bitrev8((uint8_t)p);
        if (j < 162) { out[j] = enc[i]; ++i; }
    }
}

bool wspr_encode_type1(const char* callsign, const char* grid4,
                       int power_dbm, uint8_t out_symbols[WSPR_SYMBOL_COUNT]) {
    char field[6];
    uint32_t n, m;
    if (!callsign || !grid4) return false;
    if (!wspr_normalize_call(callsign, field)) return false;
    if (!wspr_pack_grid_power(grid4, power_dbm, &m)) return false;
    n = wspr_pack_call(field);

    uint8_t bits[50];
    for (int i = 0; i < 28; ++i) bits[i]      = (uint8_t)((n >> (27 - i)) & 1u);
    for (int i = 0; i < 22; ++i) bits[28 + i] = (uint8_t)((m >> (21 - i)) & 1u);

    uint8_t enc[162], inter[162];
    wspr_conv_encode(bits, enc);
    wspr_interleave(enc, inter);
    for (int i = 0; i < 162; ++i)
        out_symbols[i] = (uint8_t)(WSPR_SYNC[i] + 2 * inter[i]);
    return true;
}
