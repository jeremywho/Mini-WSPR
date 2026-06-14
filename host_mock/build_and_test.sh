#!/bin/sh
# Host build + run for the WSPR encoder unit tests.
# Canonical entry point on the Windows dev box (Git Bash + MinGW gcc), where
# `make` is not installed. On a make-equipped system the Makefile also works.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
gcc -std=c11 -g -Wall -Wextra -I"$HERE/../main" -I"$HERE" \
    "$HERE/../main/wspr_encode.c" "$HERE/test_wspr_encode.c" \
    -o "$HERE/test_wspr_encode.exe"
"$HERE/test_wspr_encode.exe"
