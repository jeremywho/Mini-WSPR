#pragma once
#include <stdbool.h>
#include <stdint.h>
// Internal helpers exposed for host unit tests. Not a public API.
int      wspr_char_val(char c);                 // '0'-'9'->0..9, 'A'-'Z'->10..35, ' '->36, else -1
bool     wspr_normalize_call(const char* in, char out_field[6]);
uint32_t wspr_pack_call(const char field[6]);
bool     wspr_pack_grid_power(const char* grid4, int dbm, uint32_t* out_m);
uint8_t  wspr_parity32(uint32_t x);
void     wspr_conv_encode(const uint8_t bits[50], uint8_t enc[162]);
uint8_t  wspr_bitrev8(uint8_t b);
void     wspr_interleave(const uint8_t enc[162], uint8_t out[162]);
