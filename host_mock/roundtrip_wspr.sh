#!/bin/sh
# Render the C tx_synth output and decode it with wsprd at both 12000 Hz and the
# device's 11520 Hz (resampled 25/24 -> 12000). Asserts the message round-trips.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PY=/c/Python313/python
WSPRD="C:/WSJT/wsjtx/bin/wsprd.exe"
"$HERE/wspr_render.exe" K1ABC FN42 37 12000 "$HERE/r12000.wav"
"$HERE/wspr_render.exe" K1ABC FN42 37 11520 "$HERE/r11520.wav"
"$PY" - "$HERE" <<'PY'
import sys, wave, numpy as np
from scipy.signal import resample_poly
here = sys.argv[1]
def load(p):
    w=wave.open(p); fr=w.readframes(w.getnframes()); w.close()
    return np.frombuffer(fr, dtype='<i2').astype(np.float64)
def save_pad(x, p, fs=12000, secs=120):
    x=np.clip(np.round(x),-32768,32767).astype('<i2')
    need=fs*secs-len(x)
    pad=np.zeros(need, dtype='<i2') if need>0 else np.zeros(0,dtype='<i2')
    out=wave.open(p,'wb'); out.setnchannels(1); out.setsampwidth(2); out.setframerate(fs)
    out.writeframes(x.tobytes()+pad.tobytes()); out.close()
save_pad(load(here+'/r12000.wav'), here+'/r12000_pad.wav')
save_pad(resample_poly(load(here+'/r11520.wav'), 25, 24), here+'/r11520_pad.wav')
print("padded both")
PY
rc=0
for tag in r12000 r11520; do
  echo "=== wsprd $tag ==="
  "$WSPRD" -f 14.0971 "$HERE/${tag}_pad.wav" 2>&1 | tee "$HERE/.${tag}.txt"
  if grep -iq "K1ABC FN42 37" "$HERE/.${tag}.txt"; then echo "${tag} OK"; else echo "${tag} FAIL"; rc=1; fi
done
rm -f "$HERE"/r12000*.wav "$HERE"/r11520*.wav "$HERE"/.r12000.txt "$HERE"/.r11520.txt
[ "$rc" = 0 ] && echo "ROUNDTRIP OK (both rates)" || echo "ROUNDTRIP FAILED"
exit $rc
