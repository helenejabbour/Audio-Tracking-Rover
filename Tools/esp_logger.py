#!/usr/bin/env python3
"""
esp_logger.py - log ESP32 acoustic packets to CSV, with ground truth passed on the command line.

  pip install pyserial
  python esp_logger.py --port COM6 --label E0_static --rho 1.0 --beta 0 --phi 0 --duration 60
  (Pi/Linux: --port /dev/ttyUSB0 or /dev/ttyACM0)

Writes Data/raw/<label>_<timestamp>.csv (+ .json with args, firmware config, summary) and prints a quick look:
packet period (~T_m), valid/clip/late rates, noise estimate s_n, ln(A+/A-) stats, A_bar*rho.
Close the Arduino Serial Monitor first (the port is exclusive).

Packets measured before the firmware confirms the commanded settings (its 'C' reply to '?') are skipped,
so the first rows of a run never use the previous run's dither or arm angle. Ctrl+C stops early and still saves.
Needs esp32_acoustic_frontend v2 (Oct 28) for the settings check and the capture-order column 'pfirst'.
"""
import argparse, csv, json, math, os, statistics as st, time
import serial

RAW = ["t_pi", "seq", "t_ms", "dt_pair_ms", "phi_c", "dlt", "A_m", "A_p", "Aref_m", "Aref_p", "pp_m", "pp_p", "flags"]


def parse(line):
    f = line.strip().split(",")
    if len(f) != 13 or f[0] != "M":
        return None
    try:
        return [int(f[1]), int(f[2]), int(f[3])] + [float(x) for x in f[4:10]] + [int(x) for x in f[10:13]]
    except ValueError:
        return None


def sd(v):
    return st.stdev(v) if len(v) > 1 else float("nan")


def summarize(rows, rho):
    n = len(rows)
    if n < 3:
        return {"n": n}
    seq, tms = [r["seq"] for r in rows], [r["t_ms"] for r in rows]
    steps = [(b - a) / 1000 for a, b, s0, s1 in zip(tms, tms[1:], seq, seq[1:]) if s1 == s0 + 1]
    gaps = sum(max(b - a - 1, 0) for a, b in zip(seq, seq[1:]))
    resets = sum(1 for a, b in zip(seq, seq[1:]) if b < a)
    good = [r for r in rows if r["valid"]]
    s = {"n": n, "T_m_median_s": st.median(steps) if steps else None, "dropped": gaps, "resets": resets,
         "valid_pct": 100 * len(good) / n, "clip_pct": 100 * sum(r["clip"] for r in rows) / n,
         "late_pct": 100 * sum(r["late"] for r in rows) / n}
    ref_all = [0.5 * (r["Aref_m"] + r["Aref_p"]) for r in rows]          # all rows: speaker-off run gives the noise floor
    s.update({"Aref_mean_all": st.mean(ref_all), "s_n_est": st.mean(ref_all) / math.sqrt(math.pi / 2)})  # Rayleigh -> per-quadrature std
    if len(good) >= 3:
        Am, Ap = [r["A_m"] for r in good], [r["A_p"] for r in good]
        ref = [0.5 * (r["Aref_m"] + r["Aref_p"]) for r in good]
        lr = [math.log(r["A_p"] / r["A_m"]) for r in good]
        geo = [math.sqrt(r["A_m"] * r["A_p"]) for r in good]
        s.update({"A_m_mean": st.mean(Am), "A_m_std": sd(Am), "A_p_mean": st.mean(Ap), "A_p_std": sd(Ap),
                  "snr_db": 20 * math.log10(st.mean(geo) / max(st.mean(ref), 1e-9)),
                  "ln_ratio_mean": st.mean(lr), "ln_ratio_std": sd(lr), "A_geo_mean": st.mean(geo),
                  "A_geo_x_rho": st.mean(geo) * rho})
        # Same ratio split by capture order: if the two means differ, the capture order biases the bearing.
        for name, pf in (("mfirst", 0), ("pfirst", 1)):
            sub = [math.log(r["A_p"] / r["A_m"]) for r in good if r["pfirst"] == pf]
            if sub:
                s[f"ln_ratio_mean_{name}"] = st.mean(sub)
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--label", default="test")
    ap.add_argument("--rho", type=float, required=True, help="true distance servo axis -> speaker [m]")
    ap.add_argument("--beta", type=float, required=True, help="true bearing of the speaker [deg, + = left]")
    ap.add_argument("--phi", type=float, default=0.0, help="arm centre phi_c to command [deg]")
    ap.add_argument("--delta", type=float, default=None, help="dither delta [deg]; inspect the selected legacy firmware default")
    ap.add_argument("--settle", type=int, default=None, help="servo settle [ms]; inspect the selected legacy firmware default")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--outdir", default="Data/raw")
    ap.add_argument("--notes", default="", help="e.g. gain trimmer mark, speaker volume, room")
    a = ap.parse_args()

    os.makedirs(a.outdir, exist_ok=True)
    base = os.path.join(a.outdir, f"{a.label}_{time.strftime('%Y%m%d_%H%M%S')}")
    ser = serial.serial_for_url(a.port, a.baud, timeout=0.2)
    time.sleep(2.0)                                   # most boards reset when the port opens
    ser.reset_input_buffer()
    cmds = [f"P{a.phi}"] + ([f"D{a.delta}"] if a.delta is not None else []) + \
           ([f"S{a.settle}"] if a.settle is not None else []) + ["?"]
    for c in cmds:
        ser.write((c + "\n").encode())
        time.sleep(0.05)

    # What the firmware should report once our commands are in force (it clamps them the same way).
    want = {"phi": max(-80.0, min(80.0, a.phi)),
            "dlt": None if a.delta is None else max(1.0, min(60.0, a.delta)),
            "settle": None if a.settle is None else max(20, min(1000, a.settle))}

    def matches(c):                      # c = fields of a v2 'C' line
        return (abs(float(c[7]) - want["phi"]) < 0.05
                and (want["dlt"] is None or abs(float(c[8]) - want["dlt"]) < 0.05)
                and (want["settle"] is None or int(c[5]) == want["settle"]))

    cfg, rows, bad, skipped, ready, n_cfg = None, [], 0, 0, False, 0
    t_cmd = time.monotonic()
    t_end, t_stat = t_cmd + a.duration, 0.0
    cols = RAW + ["valid", "clip", "late", "pfirst", "rho_true", "beta_true", "label"]
    try:
        with open(base + ".csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            while time.monotonic() < t_end:
                raw = ser.readline().decode("ascii", errors="ignore")
                if raw.startswith("C,"):
                    cfg, n_cfg = raw.strip(), n_cfg + 1
                    print("\n" + cfg)
                    c = cfg.split(",")
                    if not ready:
                        if len(c) < 9:
                            print("# old firmware (no settings echo): cannot confirm settings, logging from here on")
                            ready = True
                        elif matches(c):
                            ready = True
                        elif n_cfg >= 2:         # the reply to '?' always comes after P/D/S were applied
                            print(f"# WARN firmware runs phi_c={c[7]} dlt={c[8]} settle={c[5]}, not what was asked")
                            ready = True
                    continue
                if raw.startswith("#"):
                    print("\n" + raw.strip()); continue
                p = parse(raw)
                if p is None:
                    bad += raw.startswith("M"); continue
                if not ready:
                    if time.monotonic() - t_cmd < 10.0:
                        skipped += 1; continue
                    print("\n# WARN no settings confirmation after 10 s, logging anyway")
                    ready = True
                fl = p[11]
                row = dict(zip(RAW, [time.time()] + p), valid=int((fl & 7) == 1), clip=(fl >> 1) & 1, late=(fl >> 2) & 1)
                rows.append(row)
                w.writerow([row[k] for k in RAW] + [row["valid"], row["clip"], row["late"], row["pfirst"],
                                                    a.rho, a.beta, a.label])
                fh.flush()
                if time.monotonic() - t_stat > 1.0:
                    t_stat = time.monotonic()
                    print(f"\rn={len(rows):4d} valid={100*sum(r['valid'] for r in rows)/len(rows):3.0f}% "
                          f"A-={row['A_m']:7.1f} A+={row['A_p']:7.1f} Aref={row['Aref_m']:5.2f} "
                          f"pp={row['pp_m']}/{row['pp_p']} flags={fl}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    s = summarize(rows, a.rho)
    s["bad_lines"] = bad
    s["skipped_stale"] = skipped
    print("\n\n--- summary ---")
    for k, v in s.items():
        print(f"{k:16s} {v:.4g}" if isinstance(v, float) else f"{k:16s} {v}")
    with open(base + ".json", "w") as fh:
        json.dump({"args": vars(a), "firmware_config": cfg, "summary": s}, fh, indent=1)
    print(f"\nsaved {base}.csv / .json")


if __name__ == "__main__":
    main()
