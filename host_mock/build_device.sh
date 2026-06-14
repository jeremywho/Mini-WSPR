#!/bin/sh
# Host build + run for the WSPR device-logic unit tests; also builds wspr_render.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../main"
gcc -std=c11 -g -Wall -Wextra -I"$SRC" -I"$HERE" \
    "$SRC/wspr_encode.c" "$SRC/tx_synth.c" "$SRC/wspr_sched.c" "$SRC/wspr_time.c" "$SRC/wspr_band.c" \
    "$HERE/test_device_logic.c" -lm -o "$HERE/test_device_logic.exe"
"$HERE/test_device_logic.exe"
gcc -std=c11 -g -Wall -Wextra -I"$SRC" \
    "$SRC/wspr_encode.c" "$SRC/tx_synth.c" "$HERE/wspr_render.c" -lm -o "$HERE/wspr_render.exe"
echo "built wspr_render.exe"
