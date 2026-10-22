#!/usr/bin/env python3
"""
make_tone.py - write a continuous sine WAV for the beacon speaker (stdlib only).

  python make_tone.py --f0 4000 --seconds 600 --amp 0.5      ->  tone_4000Hz.wav

f0 must be an integer number of Hz and equal to F0_HZ in the firmware. Play it from a phone/laptop through
the Bluetooth speaker; write down the phone/laptop volume and the speaker volume and do not change them
during the calibration campaign (K depends on them).
"""
import argparse, math, struct, wave

ap = argparse.ArgumentParser()
ap.add_argument("--f0", type=int, default=1000)
ap.add_argument("--seconds", type=int, default=600)
ap.add_argument("--amp", type=float, default=0.5, help="fraction of full scale (0-1)")
ap.add_argument("--fs", type=int, default=44100)
a = ap.parse_args()

# one second holds an integer number of cycles, so repeating the block is phase-continuous
block = b"".join(struct.pack("<h", int(32767 * a.amp * math.sin(2 * math.pi * a.f0 * i / a.fs))) for i in range(a.fs))
out = f"tone_{a.f0}Hz.wav"
with wave.open(out, "wb") as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(a.fs)
    for _ in range(a.seconds):
        w.writeframes(block)
print(f"wrote {out}: {a.f0} Hz, {a.seconds} s, amp {a.amp}")
