#!/usr/bin/env python3
"""
plot_directivity.py - plot horn directivity (experiment 1) from esp_logger.py runs.

  pip install numpy matplotlib
  python plot_directivity.py Experiment1_out2.zip
  python plot_directivity.py Experiment1_4.zip Experiment1_out2.zip --labels "indoor" "outdoor"
  python plot_directivity.py data                      (a folder works too)

Each zip file or folder is one sweep (one line on the plot). It reads every E1_*.csv inside, keeps only
valid packets (flags bit0), and reduces each run to one level: the median of sqrt(A- * A+).

A run is left out, and listed as such, when
  - fewer than --min-valid of its packets are valid (e.g. the servo-noise clipping at +60 deg), or
  - its level wandered by more than --max-sd (std of ln A over its valid packets; 0.15 = about 15 %),
    which is what someone walking past, a cat or a door looks like.
If a sweep has two clean runs at 0 deg (start and end), slow drift is removed linearly in time
(--drift none turns this off). A large drift is reported, because a sudden change (the speaker being
nudged) is not linear and makes that sweep unreliable.

The plot is in dB relative to each sweep's peak (--ref zero: relative to its 0-deg level). Error bars are
the standard error from the run's 1-second means, so slow wandering inside a run shows up in them.
--fit adds the best Gaussian (a parabola in ln A), the model the report uses, with its sigma and centre.
Saves <first input>_directivity.png (or --out) and opens the plot window (unless --no-show).
"""
import argparse, csv, io, math, os, sys, zipfile
from statistics import median

try:
    import numpy as np
    import matplotlib
except ImportError:
    sys.exit("needs numpy and matplotlib:  pip install numpy matplotlib")

DB = 20 / math.log(10)                     # ln amplitude -> dB


def read_runs(src, prefix):
    """Yield (name, rows) for every <prefix>*.csv in a zip file or folder (searched recursively)."""
    if zipfile.is_zipfile(src):
        with zipfile.ZipFile(src) as z:
            for n in sorted(z.namelist()):
                b = os.path.basename(n)
                if b.startswith(prefix) and b.endswith(".csv") and not n.startswith("__MACOSX"):
                    with z.open(n) as fh:
                        yield b, list(csv.DictReader(io.TextIOWrapper(fh, encoding="utf-8", newline="")))
    elif os.path.isdir(src):
        for root, _, files in os.walk(src):
            for b in sorted(files):
                if b.startswith(prefix) and b.endswith(".csv"):
                    with open(os.path.join(root, b), newline="", encoding="utf-8") as fh:
                        yield b, list(csv.DictReader(fh))
    else:
        sys.exit(f"{src}: not a zip file or a folder")


def summarize_run(name, rows):
    if not rows:
        return None
    phi = float(rows[0]["phi_c"])
    t = np.array([float(r["t_pi"]) for r in rows])
    ok = np.array([int(r["flags"]) & 1 for r in rows], bool)
    lnA = np.log(np.maximum([math.sqrt(float(r["A_m"]) * float(r["A_p"])) for r in rows], 1e-9))
    s = dict(name=name, phi=phi, t=float(t[0]), n=len(rows), valid=float(ok.mean()), lnA=float("nan"),
             sd=float("nan"), se=float("nan"))
    if ok.sum() >= 3:
        v, tv = lnA[ok], t[ok] - t[0]
        s["lnA"] = float(median(v))
        s["sd"] = float(np.std(v, ddof=1))
        blocks = [v[(tv >= k) & (tv < k + 1)].mean() for k in range(int(tv.max()) + 1) if ((tv >= k) & (tv < k + 1)).any()]
        s["se"] = float(np.std(blocks, ddof=1) / math.sqrt(len(blocks))) if len(blocks) > 1 else float(np.std(v, ddof=1))
    return s


def analyse(src, a):
    runs = [s for name, rows in read_runs(src, a.prefix) if (s := summarize_run(name, rows))]
    if not runs:
        sys.exit(f"{src}: no {a.prefix}*.csv files found")
    runs.sort(key=lambda s: s["t"])
    for s in runs:
        s["why"] = ("few valid packets (%.0f %%)" % (100 * s["valid"]) if s["valid"] < a.min_valid or math.isnan(s["lnA"]) else
                    "level wandered (sd %.2f)" % s["sd"] if s["sd"] > a.max_sd else "")
    good = [s for s in runs if not s["why"]]
    zeros = [s for s in good if abs(s["phi"]) < 0.5]
    drift_note = "no drift correction (needs two clean 0-deg runs)"
    if a.drift == "linear" and len(zeros) >= 2 and zeros[-1]["t"] > zeros[0]["t"]:
        z0, z1 = zeros[0], zeros[-1]
        k = (z1["lnA"] - z0["lnA"]) / (z1["t"] - z0["t"])
        for s in good:
            s["lnA"] -= k * (s["t"] - z0["t"])
        d = math.exp(z1["lnA"] + k * (z1["t"] - z0["t"]) - z0["lnA"]) - 1      # drift before correction
        drift_note = f"drift {d:+.0%} between first and last 0-deg run, removed linearly"
        if abs(d) > 0.15:
            drift_note += "  <-- large: check for a disturbance (speaker nudged?) before trusting this sweep"
    elif a.drift == "none":
        drift_note = "drift correction off"
    # one point per angle (repeated angles, e.g. the two 0-deg runs, are averaged after drift correction)
    pts = {}
    for s in good:
        pts.setdefault(round(s["phi"], 1), []).append(s)
    P = np.array(sorted(pts))
    L = np.array([np.mean([s["lnA"] for s in pts[p]]) for p in P])
    E = np.array([math.sqrt(sum(s["se"] ** 2 for s in pts[p])) / len(pts[p]) for p in P])
    return dict(src=src, runs=runs, P=P, L=L, E=E, drift=drift_note)


def gauss_fit(P, L, E, lim):
    m = np.abs(P) <= lim
    if m.sum() < 4:
        return None
    w = 1 / np.maximum(E[m], 0.005)                    # floor so one very quiet run does not dominate
    c2, c1, c0 = np.polyfit(P[m], L[m], 2, w=w)
    if c2 >= 0:
        return None                                    # not peaked: no Gaussian
    sigma, centre = math.sqrt(-1 / (2 * c2)), -c1 / (2 * c2)
    res = L[m] - np.polyval([c2, c1, c0], P[m])
    return dict(c=(c2, c1, c0), sigma=sigma, centre=centre, rms_db=DB * float(np.sqrt(np.mean(res ** 2))),
                max_db=DB * float(np.max(np.abs(res))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="zip files and/or folders, one sweep each")
    ap.add_argument("--labels", nargs="+", help="legend names, one per input (default: file name)")
    ap.add_argument("--prefix", default="E1_", help="only files starting with this (default E1_)")
    ap.add_argument("--min-valid", type=float, default=0.5, help="min fraction of valid packets per run (default 0.5)")
    ap.add_argument("--max-sd", type=float, default=0.15, help="max std of ln A inside a run (default 0.15)")
    ap.add_argument("--drift", choices=["linear", "none"], default="linear")
    ap.add_argument("--ref", choices=["peak", "zero"], default="peak", help="0 dB = sweep peak or its 0-deg level")
    ap.add_argument("--fit", action="store_true", help="overlay the best Gaussian for each sweep")
    ap.add_argument("--fit-range", type=float, default=80, help="fit only |angle| <= this (default 80)")
    ap.add_argument("--out", help="output image (default: <first input>_directivity.png)")
    ap.add_argument("--no-show", action="store_true", help="save the image without opening a window")
    a = ap.parse_args()
    if a.labels and len(a.labels) != len(a.inputs):
        sys.exit("give one --labels entry per input")
    if a.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for i, src in enumerate(a.inputs):
        r = analyse(src, a)
        label = a.labels[i] if a.labels else os.path.splitext(os.path.basename(os.path.normpath(src)))[0]
        if len(r["P"]) == 0:
            print(f"\n=== {label}: no usable runs"); continue
        if a.ref == "zero" and np.any(np.abs(r["P"]) < 0.5):
            ref = float(r["L"][np.abs(r["P"]) < 0.5][0])
        else:
            ref = float(r["L"].max())
        dB, eB = DB * (r["L"] - ref), DB * r["E"]

        print(f"\n=== {label}  ({src})")
        print("  " + r["drift"])
        print(f"  {'angle':>6} {'level':>7} {'dB':>6} {'+-dB':>5}")
        for p, l, d, e in zip(r["P"], r["L"], dB, eB):
            print(f"  {p:+6.1f} {math.exp(l):7.1f} {d:+6.2f} {e:5.2f}")
        left = [s for s in r["runs"] if s["why"]]
        for s in left:
            print(f"  left out: {s['name']}  ({s['why']})")

        line = ax.errorbar(r["P"], dB, yerr=eB, marker="o", ms=4, capsize=2, lw=1.5, label=label)
        if a.fit:
            g = gauss_fit(r["P"], r["L"], r["E"], a.fit_range)
            if g:
                x = np.linspace(max(r["P"].min(), -a.fit_range), min(r["P"].max(), a.fit_range), 200)
                ax.plot(x, DB * (np.polyval(g["c"], x) - ref), "--", lw=1, color=line[0].get_color(),
                        label=f"{label}: Gaussian, sigma {g['sigma']:.0f} deg, centre {g['centre']:+.0f} deg")
                print(f"  Gaussian fit: sigma {g['sigma']:.0f} deg, centre {g['centre']:+.1f} deg, "
                      f"misfit {g['rms_db']:.2f} dB rms / {g['max_db']:.2f} dB max")
            else:
                print("  Gaussian fit: not possible (curve not peaked or too few points)")

    ax.set_xlabel("horn angle (deg, + = left; speaker at 0)")
    ax.set_ylabel("level re peak (dB)" if a.ref == "peak" else "level re 0 deg (dB)")
    ax.set_title("Horn directivity")
    ax.set_xticks(range(-80, 81, 20))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    out = a.out or os.path.splitext(os.path.normpath(a.inputs[0]))[0] + "_directivity.png"
    fig.savefig(out, dpi=150)
    print(f"\nsaved {out}")
    if not a.no_show:
        plt.show()


if __name__ == "__main__":
    main()
