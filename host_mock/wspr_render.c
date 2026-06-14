#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wspr_encode.h"
#include "tx_synth.h"

static void w16(FILE* f, int x) { unsigned char b[2] = {(unsigned char)(x & 0xff), (unsigned char)((x >> 8) & 0xff)}; fwrite(b, 1, 2, f); }
static void w32(FILE* f, long x) { for (int i = 0; i < 4; i++) { fputc((int)(x & 0xff), f); x >>= 8; } }

int main(int argc, char** argv) {
    if (argc != 6) { fprintf(stderr, "usage: wspr_render <call> <grid> <dbm> <rate> <out.wav>\n"); return 2; }
    int dbm = atoi(argv[3]), rate = atoi(argv[4]);
    uint8_t sym[WSPR_SYMBOL_COUNT];
    if (!wspr_encode_type1(argv[1], argv[2], dbm, sym)) { fprintf(stderr, "encode failed\n"); return 1; }
    tx_synth_plan_t p = { sym, 162, 8192.0/12000.0, 12000.0/8192.0, 1500.0, rate, 88.0, true };
    tx_synth_t s; tx_synth_init(&s, &p, 0);
    long n = (long)tx_synth_total_samples(&s);
    FILE* f = fopen(argv[5], "wb"); if (!f) { perror("open"); return 1; }
    fwrite("RIFF", 1, 4, f); w32(f, 36 + n * 2); fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f); w32(f, 16); w16(f, 1); w16(f, 1); w32(f, rate); w32(f, (long)rate * 2); w16(f, 2); w16(f, 16);
    fwrite("data", 1, 4, f); w32(f, n * 2);
    uint8_t buf[4096];
    for (;;) { int k = tx_synth_pull(&s, buf, 4096); if (k == 0) break;
               for (int i = 0; i < k; i++) w16(f, ((int)buf[i] - 128) * 256); }
    fclose(f); fprintf(stderr, "rendered %ld samples @%d\n", n, rate); return 0;
}
