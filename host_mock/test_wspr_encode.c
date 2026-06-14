#include <stdio.h>
#include <string.h>
#include "wspr_encode.h"
#include "wspr_encode_internal.h"
#include "wspr_golden.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); g_fail++; } \
} while (0)

static void test_normalize(void) {
    char f[6];
    CHECK(wspr_normalize_call("K1ABC", f) && memcmp(f, " K1ABC", 6) == 0, "K1ABC -> ' K1ABC'");
    CHECK(wspr_normalize_call("PA0XYZ", f) && memcmp(f, "PA0XYZ", 6) == 0, "PA0XYZ unchanged");
    CHECK(wspr_normalize_call("g4jnt", f) && memcmp(f, " G4JNT", 6) == 0, "lowercase upcased+shifted");
    CHECK(!wspr_normalize_call("K1ABC/P", f), "reject /P");
    CHECK(!wspr_normalize_call("ABCDEF", f), "reject no-digit");
    CHECK(!wspr_normalize_call("VE3ABCD", f), "reject too-long");
}

static void test_pack_call(void) {
    char f[6];
    wspr_normalize_call("K1ABC", f);
    // 36,20,1 then (A,B,C)->0,1,2 : 36 -> *36+20=1316 -> *10+1=13161
    //  -> *27+0=355347 -> *27+1=9594370 -> *27+2=259047992
    CHECK(wspr_pack_call(f) == 259047992u, "pack_call(' K1ABC')");
}

static void test_pack_grid_power(void) {
    uint32_t m = 0;
    // FN42: g1=5,g2=13,g3=4,g4=2 -> M1=(179-50-4)*180+130+2=22632 -> *128+37+64=2896997
    CHECK(wspr_pack_grid_power("FN42", 37, &m) && m == 2896997u, "pack FN42/37");
    CHECK(wspr_pack_grid_power("fn42", 37, &m) && m == 2896997u, "grid lowercased");
    CHECK(!wspr_pack_grid_power("FN42", 35, &m), "reject dBm last-digit 5");
    CHECK(!wspr_pack_grid_power("FN4", 37, &m),  "reject 3-char grid");
    CHECK(!wspr_pack_grid_power("SN42", 37, &m), "reject field letter > R");
}

static void test_conv(void) {
    CHECK(wspr_parity32(0) == 0, "parity 0");
    CHECK(wspr_parity32(1) == 1, "parity 1");
    CHECK(wspr_parity32(0x3) == 0, "parity 0b11");
    CHECK(wspr_parity32(0x7) == 1, "parity 0b111");

    uint8_t zeros[50] = {0}, enc[162];
    wspr_conv_encode(zeros, enc);
    int allzero = 1; for (int i = 0; i < 162; ++i) if (enc[i]) allzero = 0;
    CHECK(allzero, "zero message -> zero codeword");

    uint8_t imp[50] = {0}; imp[0] = 1;
    wspr_conv_encode(imp, enc);
    int ok = 1;
    for (int k = 0; k < 32; ++k) {
        uint8_t p1 = (uint8_t)((0xf2d05351u >> k) & 1u);
        uint8_t p2 = (uint8_t)((0xe4613c47u >> k) & 1u);
        if (enc[2*k] != p1 || enc[2*k+1] != p2) ok = 0;
    }
    CHECK(ok, "impulse response matches generator polynomials");
}

static void test_interleave(void) {
    CHECK(wspr_bitrev8(0x00) == 0x00, "bitrev 0");
    CHECK(wspr_bitrev8(0x01) == 0x80, "bitrev 1");
    CHECK(wspr_bitrev8(0x80) == 0x01, "bitrev 0x80");
    CHECK(wspr_bitrev8(0xFF) == 0xFF, "bitrev 0xFF");
    CHECK(wspr_bitrev8(0x02) == 0x40, "bitrev 0x02");

    uint8_t in[162], out[162];
    for (int i = 0; i < 162; ++i) in[i] = (uint8_t)i;
    wspr_interleave(in, out);
    int seen[162] = {0}, perm_ok = 1;
    for (int i = 0; i < 162; ++i) { if (seen[out[i]]) perm_ok = 0; seen[out[i]] = 1; }
    CHECK(perm_ok, "interleave is a permutation of all 162 positions");
}

static void test_golden(void) {
    for (size_t c = 0; c < sizeof(WSPR_GOLDEN)/sizeof(WSPR_GOLDEN[0]); ++c) {
        const wspr_golden_t* g = &WSPR_GOLDEN[c];
        uint8_t sym[162];
        bool ok = wspr_encode_type1(g->call, g->grid, g->dbm, sym);
        CHECK(ok, "encode returns true for golden case");
        int match = ok ? (memcmp(sym, g->sym, 162) == 0) : 0;
        CHECK(match, g->call);
        if (ok && !match) {
            for (int i = 0; i < 162; ++i)
                if (sym[i] != g->sym[i]) { printf("  first diff at %d: got %d want %d\n",
                                                   i, sym[i], g->sym[i]); break; }
        }
    }
}

static void test_reject(void) {
    uint8_t sym[162];
    CHECK( wspr_encode_type1("K1ABC", "FN42", 37, sym), "valid accepted");
    CHECK(!wspr_encode_type1("K1ABC/P", "FN42", 37, sym), "reject /P");
    CHECK(!wspr_encode_type1("ABCDEF", "FN42", 37, sym), "reject no-digit call");
    CHECK(!wspr_encode_type1("VE3ABCD", "FN42", 37, sym), "reject too-long call");
    CHECK(!wspr_encode_type1("K1ABC", "FN4", 37, sym), "reject short grid");
    CHECK(!wspr_encode_type1("K1ABC", "FN42", 35, sym), "reject bad dBm");
    CHECK(!wspr_encode_type1(NULL, "FN42", 37, sym), "reject NULL call");
}

int main(void) {
    test_normalize();
    test_pack_call();
    test_pack_grid_power();
    test_conv();
    test_interleave();
    test_golden();
    test_reject();
    if (g_fail == 0) printf("ALL TESTS PASSED\n");
    else printf("%d CHECK(S) FAILED\n", g_fail);
    return g_fail ? 1 : 0;
}
