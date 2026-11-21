#!/usr/bin/env python3
"""
make_tone.py - write a continuous beacon WAV for the speaker (stdlib only).

Single tone (legacy single-bin firmware):
  python Tools/make_tone.py --f0 4000 --seconds 600 --amp 0.5
      ->  Data/stimuli/tone_4000Hz.wav

Multi-tone (frequency diversity; matches TONE_K in firmware/esp32_acoustic_v2):
  python Tools/make_tone.py --multitone --seconds 600 --amp 0.9
      ->  Data/stimuli/multitone_2500-6250Hz_16.wav   (16 tones, 2500..6250 Hz every 250 Hz)

The multi-tone uses Schroeder phases, phi_m = -pi m(m-1)/M, which keep the crest factor
near that of one sine, so each tone gets as much level as the speaker allows without
clipping. --amp is the peak as a fraction of full scale. Every tone is an integer number
of Hz, so a 1 s block holds whole cycles and repeating it is phase-continuous.

Pre-emphasised multi-tone from a spec (Tools/beacon_spec.py writes it from measured runs):
  python Tools/make_tone.py --spec Data/stimuli/horn12_spec.json --seconds 600 --amp 0.9
      ->  Data/stimuli/horn12.wav     (tones may be half-Hz: the block is then 2 s long)

The tones must equal the firmware bins: k*31.25 Hz for fs = 16 kHz, N = 512 (the defaults
are k = 80..200, step 8). Play it from a phone/laptop through the Bluetooth speaker; write
down the phone/laptop volume and the speaker volume and do not change them during a
calibration campaign (K depends on them).
"""
import argparse
import json
import math
import struct
import wave
from fractions import Fraction


def tone_list(start, step, count):
    return [start + i * step for i in range(count)]


def block_seconds(freqs, limit=10):
    """Shortest block (s) holding a whole number of cycles of every tone (1 s for integer Hz)."""
    for t in range(1, limit + 1):
        if all((Fraction(str(f)) * t).denominator == 1 for f in freqs):
            return t
    raise ValueError("tones need a block longer than %d s; use bin-centred frequencies" % limit)


def schroeder_block(freqs, fs, peak, gains_db=None):
    """Whole-cycle block of a Schroeder-phase multi-tone, scaled to the given peak.

    gains_db sets each tone's relative level (pre-emphasis). Returns (samples, scale), where
    scale is the amplitude (fraction of full scale) of a 0 dB tone.
    """
    m = len(freqs)
    g = [1.0] * m if gains_db is None else [10 ** (d / 20) for d in gains_db]
    phases = [-math.pi * i * (i - 1) / m for i in range(1, m + 1)]
    n = fs * block_seconds(freqs)
    x = [sum(a * math.sin(2 * math.pi * f * k / fs + p) for f, a, p in zip(freqs, g, phases)) for k in range(n)]
    scale = peak / max(abs(v) for v in x)
    return [v * scale for v in x], scale


def write_wav(path, samples, fs, seconds):
    """Repeat the whole-cycle block to fill at least `seconds`."""
    block = b"".join(struct.pack("<h", int(round(32767 * v))) for v in samples)
    repeats = max(1, math.ceil(seconds * fs / len(samples)))
    with wave.open(path, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(fs)
        for _ in range(repeats):
            w.writeframes(block)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--f0", type=int, default=4000, help="single-tone frequency (Hz)")
    ap.add_argument("--multitone", action="store_true", help="write the multi-tone beacon instead")
    ap.add_argument("--spec", help="JSON with tone_hz and gains_db (Tools/beacon_spec.py)")
    ap.add_argument("--start", type=int, default=2500, help="first tone (Hz), multi-tone")
    ap.add_argument("--step", type=int, default=250, help="tone spacing (Hz), multi-tone")
    ap.add_argument("--count", type=int, default=16, help="number of tones, multi-tone")
    ap.add_argument("--seconds", type=int, default=600)
    ap.add_argument("--amp", type=float, default=0.5, help="peak as a fraction of full scale (0-1)")
    ap.add_argument("--fs", type=int, default=44100)
    ap.add_argument("--out", help="output path (default under Data/stimuli/)")
    a = ap.parse_args(argv)
    if not 0 < a.amp <= 1:
        ap.error("--amp must be in (0, 1]")
    if a.spec:
        spec = json.load(open(a.spec, encoding="utf-8"))
        freqs, gains = [float(f) for f in spec["tone_hz"]], [float(g) for g in spec["gains_db"]]
        if len(freqs) != len(gains) or max(freqs) >= a.fs / 2:
            ap.error("spec needs one gain per tone, all tones below the WAV Nyquist frequency")
        samples, scale = schroeder_block(freqs, a.fs, a.amp, gains)
        rms = math.sqrt(sum(v * v for v in samples) / len(samples))
        out = a.out or a.spec.replace("_spec.json", ".wav")
        write_wav(out, samples, a.fs, a.seconds)
        print(f"wrote {out}: {len(freqs)} tones {freqs[0]:g}..{freqs[-1]:g} Hz, gains {min(gains):+.1f}..{max(gains):+.1f} dB, "
              f"peak {a.amp}, 0 dB tone amplitude {scale:.3f} FS, crest factor {a.amp / rms:.2f}, "
              f"block {block_seconds(freqs)} s, {a.seconds} s")
    elif a.multitone:
        freqs = tone_list(a.start, a.step, a.count)
        if a.count < 1 or freqs[-1] >= a.fs / 2:
            ap.error("tones must be below the WAV Nyquist frequency")
        samples, scale = schroeder_block(freqs, a.fs, a.amp)
        rms = math.sqrt(sum(v * v for v in samples) / len(samples))
        out = a.out or f"Data/stimuli/multitone_{freqs[0]}-{freqs[-1]}Hz_{a.count}.wav"
        write_wav(out, samples, a.fs, a.seconds)
        print(f"wrote {out}: {a.count} tones {freqs[0]}..{freqs[-1]} Hz step {a.step}, peak {a.amp}, "
              f"per-tone amplitude {scale:.3f} FS, crest factor {a.amp / rms:.2f}, {a.seconds} s")
    else:
        samples = [a.amp * math.sin(2 * math.pi * a.f0 * n / a.fs) for n in range(a.fs)]
        out = a.out or f"Data/stimuli/tone_{a.f0}Hz.wav"
        write_wav(out, samples, a.fs, a.seconds)
        print(f"wrote {out}: {a.f0} Hz, {a.seconds} s, amp {a.amp}")
    return out


if __name__ == "__main__":
    main()
