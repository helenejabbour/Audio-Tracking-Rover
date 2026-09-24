"""
Directivity & range POC — analysis.

Reads sweep_trials.csv (and optionally stability_log.csv) produced by
collect_sweep_data.py and produces every metric/plot from the POC design:

  1. Amplitude-vs-distance: monotonicity, decay fit, R^2, per-point CV.
  2. Amplitude-vs-angle: polar directivity pattern, angular resolution
     (smallest distinguishable Delta-theta), front-back ambiguity check.
  3. SNR tables at every (theta, d) point, with a suggested confidence
     gating threshold.
  4. Repeatability: correlation between two sessions of the same sweep,
     if more than one `session` tag is present in the CSV.
  5. Temporal stability: drift/CV over time, if stability_log.csv exists.

Run this after collect_sweep_data.py has produced data. Safe to re-run
any time - it only reads, never writes to, the trial CSVs.
"""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt

TRIALS_LOG_PATH = "sweep_trials.csv"
STABILITY_LOG_PATH = "stability_log.csv"

SNR_CONFIDENCE_THRESHOLD_DB = 6.0  # suggested default; POC data should refine this


def load_trials(path):
    if not os.path.exists(path):
        return []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    for r in rows:
        for k in ["theta_deg", "distance_cm", "peak_ac_mv", "rms_ac_mv",
                  "noise_floor_mv", "snr_db", "onset_delay_ms", "n_samples", "wall_time_s"]:
            if k in r and r[k] not in ("", "nan"):
                try:
                    r[k] = float(r[k])
                except ValueError:
                    pass
    return rows


def group_by(rows, key):
    groups = {}
    for r in rows:
        groups.setdefault(r[key], []).append(r)
    return dict(sorted(groups.items()))


# ============================================================
# 1. Distance sweep analysis
# ============================================================
def analyze_distance_sweep(rows, session=None):
    dist_rows = [r for r in rows if r["mode"] == "distance_cm" and (session is None or r["session"] == session)]
    if not dist_rows:
        print("[distance sweep] No data found.")
        return None

    by_dist = group_by(dist_rows, "distance_cm")
    dists, means, stds, cvs, snr_means = [], [], [], [], []

    print("\n" + "=" * 70)
    print(f"DISTANCE SWEEP — RMS Amplitude vs Distance  (session={session or 'all'})")
    print("=" * 70)
    print(f"{'Dist [cm]':>10} {'n':>4} {'Mean RMS [mV]':>15} {'Std [mV]':>10} {'CV [%]':>8} {'Mean SNR [dB]':>14}")

    for d, group in by_dist.items():
        vals = np.array([g["rms_ac_mv"] for g in group])
        snr_vals = np.array([g["snr_db"] for g in group if not np.isnan(g.get("snr_db", np.nan))])
        mean_v, std_v = vals.mean(), vals.std()
        cv = (std_v / mean_v * 100) if mean_v != 0 else float("nan")
        snr_mean = snr_vals.mean() if len(snr_vals) else float("nan")

        print(f"{d:>10.1f} {len(vals):>4} {mean_v:>15.2f} {std_v:>10.2f} {cv:>8.1f} {snr_mean:>14.1f}")

        dists.append(d); means.append(mean_v); stds.append(std_v); cvs.append(cv); snr_means.append(snr_mean)

    dists_arr, means_arr, stds_arr = np.array(dists), np.array(means), np.array(stds)

    # Monotonicity check
    diffs = np.diff(means_arr)
    is_monotonic_decreasing = np.all(diffs <= 0)
    n_violations = np.sum(diffs > 0)
    print(f"\nMonotonic decrease across full range: {is_monotonic_decreasing}"
          f"  ({n_violations} non-monotonic step(s) if False)")

    # Fit inverse-distance law: A = k / d^n  =>  log(A) = log(k) - n*log(d)
    valid = means_arr > 0
    log_d = np.log(dists_arr[valid])
    log_a = np.log(means_arr[valid])
    if len(log_d) >= 2:
        n_fit, log_k = np.polyfit(log_d, log_a, 1)
        n_fit = -n_fit  # A ~ 1/d^n convention
        fit_a = np.exp(log_k) / dists_arr[valid] ** n_fit
        ss_res = np.sum((means_arr[valid] - fit_a) ** 2)
        ss_tot = np.sum((means_arr[valid] - means_arr[valid].mean()) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
        print(f"Power-law fit: RMS_amplitude ~ 1 / distance^{n_fit:.2f}   (R^2 = {r_squared:.3f})")
        print(f"  (free-field point source predicts exponent ~1.0; indoor multipath typically deviates)")
    else:
        n_fit, r_squared = float("nan"), float("nan")

    # Usable range: where CV stays below a working threshold (e.g. 25%)
    cv_arr = np.array(cvs)
    usable_mask = cv_arr < 25.0
    if usable_mask.any():
        max_usable_dist = dists_arr[usable_mask].max()
        print(f"\nUsable sensing range (CV < 25%): up to ~{max_usable_dist:.0f} cm")
    else:
        print("\n[WARNING] No distance point has CV < 25% — noise dominates across your entire tested range.")

    # Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    ax1.errorbar(dists_arr, means_arr, yerr=stds_arr, fmt="o-", color="#007acc", capsize=4)
    if len(log_d) >= 2:
        fit_x = np.linspace(dists_arr.min(), dists_arr.max(), 100)
        ax1.plot(fit_x, np.exp(log_k) / fit_x ** n_fit, "--", color="#d9534f",
                  label=f"fit: 1/d^{n_fit:.2f} (R²={r_squared:.2f})")
        ax1.legend()
    ax1.set_xlabel("Distance [cm]")
    ax1.set_ylabel("RMS AC Amplitude [mV]")
    ax1.set_title("Amplitude vs Distance")
    ax1.grid(True, linestyle=":", alpha=0.6)

    ax2.plot(dists_arr, cv_arr, "o-", color="#5cb85c")
    ax2.axhline(25.0, color="gray", linestyle="--", linewidth=0.8, label="25% CV reference")
    ax2.set_xlabel("Distance [cm]")
    ax2.set_ylabel("Coefficient of Variation [%]")
    ax2.set_title("Measurement Noise vs Distance")
    ax2.legend()
    ax2.grid(True, linestyle=":", alpha=0.6)

    plt.tight_layout()
    plt.savefig("distance_sweep_analysis.png", dpi=150)
    print("\nSaved: distance_sweep_analysis.png")

    return {"dists": dists_arr, "means": means_arr, "stds": stds_arr, "cv": cv_arr,
            "n_fit": n_fit, "r_squared": r_squared}


# ============================================================
# 2. Angle sweep analysis
# ============================================================
def analyze_angle_sweep(rows, session=None):
    angle_rows = [r for r in rows if r["mode"] == "theta_deg" and (session is None or r["session"] == session)]
    if not angle_rows:
        print("[angle sweep] No data found.")
        return None

    by_angle = group_by(angle_rows, "theta_deg")
    angles, means, stds, snr_means = [], [], [], []

    print("\n" + "=" * 70)
    print(f"ANGLE SWEEP — RMS Amplitude vs Bearing  (session={session or 'all'})")
    print("=" * 70)
    print(f"{'Angle [deg]':>12} {'n':>4} {'Mean RMS [mV]':>15} {'Std [mV]':>10} {'Mean SNR [dB]':>14}")

    for a, group in by_angle.items():
        vals = np.array([g["rms_ac_mv"] for g in group])
        snr_vals = np.array([g["snr_db"] for g in group if not np.isnan(g.get("snr_db", np.nan))])
        mean_v, std_v = vals.mean(), vals.std()
        snr_mean = snr_vals.mean() if len(snr_vals) else float("nan")
        print(f"{a:>12.1f} {len(vals):>4} {mean_v:>15.2f} {std_v:>10.2f} {snr_mean:>14.1f}")
        angles.append(a); means.append(mean_v); stds.append(std_v); snr_means.append(snr_mean)

    angles_arr, means_arr, stds_arr = np.array(angles), np.array(means), np.array(stds)

    # Angular resolution: smallest |theta_i - theta_j| where mean difference
    # exceeds 2x the combined stddev (i.e. statistically distinguishable)
    print("\nPairwise distinguishability (|mean diff| > 2*combined std):")
    min_resolvable_gap = None
    for i in range(len(angles_arr)):
        for j in range(i + 1, len(angles_arr)):
            gap = abs(angles_arr[j] - angles_arr[i])
            mean_diff = abs(means_arr[j] - means_arr[i])
            combined_std = np.sqrt(stds_arr[i] ** 2 + stds_arr[j] ** 2)
            distinguishable = mean_diff > 2 * combined_std if combined_std > 0 else mean_diff > 0
            if distinguishable and (min_resolvable_gap is None or gap < min_resolvable_gap):
                min_resolvable_gap = gap

    if min_resolvable_gap is not None:
        print(f"  Smallest statistically distinguishable angular gap: ~{min_resolvable_gap:.1f} deg")
        print("  (this is your ACHIEVABLE bearing resolution — not the sweep step size)")
    else:
        print("  [WARNING] No pair of tested angles was statistically distinguishable.")
        print("  Amplitude does not currently carry usable bearing information at this noise level.")

    # Front-back / symmetric ambiguity check (compares theta vs 180-theta if both present)
    print("\nSymmetry / front-back ambiguity check (theta vs 180-theta):")
    for a in angles_arr:
        mirror = 180.0 - a
        if mirror in by_angle and a < mirror:
            m_a = means_arr[list(angles_arr).index(a)]
            m_mirror = np.mean([g["rms_ac_mv"] for g in by_angle[mirror]])
            print(f"  theta={a:.0f} (RMS={m_a:.1f}mV)  vs  theta={mirror:.0f} (RMS={m_mirror:.1f}mV)"
                  f"  -> {'AMBIGUOUS (indistinguishable)' if abs(m_a - m_mirror) < 2*stds_arr[list(angles_arr).index(a)] else 'distinguishable'}")

    # Polar plot
    fig = plt.figure(figsize=(12, 5.5))
    ax_polar = fig.add_subplot(121, projection="polar")
    theta_rad = np.deg2rad(angles_arr)
    ax_polar.plot(theta_rad, means_arr, "o-", color="#007acc")
    ax_polar.fill(theta_rad, means_arr, color="#007acc", alpha=0.15)
    ax_polar.set_title("Directivity Pattern (RMS Amplitude)", pad=20)

    ax_cart = fig.add_subplot(122)
    ax_cart.errorbar(angles_arr, means_arr, yerr=stds_arr, fmt="o-", color="#d9534f", capsize=4)
    ax_cart.set_xlabel("Angle [deg]")
    ax_cart.set_ylabel("RMS AC Amplitude [mV]")
    ax_cart.set_title("Amplitude vs Angle (linear view)")
    ax_cart.grid(True, linestyle=":", alpha=0.6)

    plt.tight_layout()
    plt.savefig("angle_sweep_analysis.png", dpi=150)
    print("\nSaved: angle_sweep_analysis.png")

    return {"angles": angles_arr, "means": means_arr, "stds": stds_arr,
            "min_resolvable_gap_deg": min_resolvable_gap}


# ============================================================
# 3. Repeatability across sessions
# ============================================================
def analyze_repeatability(rows, mode, value_key):
    sessions = sorted(set(r["session"] for r in rows if r["mode"] == mode))
    if len(sessions) < 2:
        print(f"\n[repeatability: {mode}] Only one session found ({sessions}) — "
              f"run a second sweep under a different SESSION_TAG to check repeatability.")
        return None

    print("\n" + "=" * 70)
    print(f"REPEATABILITY CHECK — {mode}  (comparing sessions: {sessions[:2]})")
    print("=" * 70)

    s1_rows = [r for r in rows if r["mode"] == mode and r["session"] == sessions[0]]
    s2_rows = [r for r in rows if r["mode"] == mode and r["session"] == sessions[1]]

    by1 = {k: np.mean([g["rms_ac_mv"] for g in v]) for k, v in group_by(s1_rows, value_key).items()}
    by2 = {k: np.mean([g["rms_ac_mv"] for g in v]) for k, v in group_by(s2_rows, value_key).items()}

    common_keys = sorted(set(by1.keys()) & set(by2.keys()))
    if len(common_keys) < 2:
        print("  Not enough overlapping points between sessions to correlate.")
        return None

    v1 = np.array([by1[k] for k in common_keys])
    v2 = np.array([by2[k] for k in common_keys])
    corr = np.corrcoef(v1, v2)[0, 1]

    print(f"  Correlation between session 1 and session 2: r = {corr:.3f}")
    print(f"  ({'good repeatability' if corr > 0.9 else 'MARGINAL/POOR — investigate mount slop, drift, or environment change' if corr < 0.7 else 'moderate repeatability'})")

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(v1, v2, color="#007acc")
    lims = [min(v1.min(), v2.min()), max(v1.max(), v2.max())]
    ax.plot(lims, lims, "--", color="gray", label="perfect agreement")
    ax.set_xlabel(f"Session 1 mean RMS [mV]")
    ax.set_ylabel(f"Session 2 mean RMS [mV]")
    ax.set_title(f"Repeatability ({mode}): r={corr:.3f}")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.6)
    plt.tight_layout()
    plt.savefig(f"repeatability_{mode}.png", dpi=150)
    print(f"  Saved: repeatability_{mode}.png")

    return corr


# ============================================================
# 4. Temporal stability
# ============================================================
def analyze_stability():
    if not os.path.exists(STABILITY_LOG_PATH):
        print(f"\n[stability] No {STABILITY_LOG_PATH} found — run collect_sweep_data.py with SWEEP_MODE='stability' first.")
        return None

    rows = load_trials(STABILITY_LOG_PATH)
    if not rows:
        print("[stability] File exists but is empty.")
        return None

    t = np.array([r["wall_time_s"] for r in rows])
    rms = np.array([r["rms_ac_mv"] for r in rows])

    mean_rms, std_rms = rms.mean(), rms.std()
    cv = std_rms / mean_rms * 100 if mean_rms != 0 else float("nan")

    # Linear drift fit
    slope, intercept = np.polyfit(t, rms, 1)

    print("\n" + "=" * 70)
    print("TEMPORAL STABILITY")
    print("=" * 70)
    print(f"  Duration: {t.max():.0f} s   n={len(rms)} samples")
    print(f"  Mean RMS: {mean_rms:.2f} mV   Std: {std_rms:.2f} mV   CV: {cv:.1f}%")
    print(f"  Linear drift: {slope*60:.3f} mV/min "
          f"({'negligible' if abs(slope*60) < 0.05*mean_rms else 'NOTABLE DRIFT — check thermal/mechanical stability'})")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, rms, "o-", color="#007acc", markersize=3, linewidth=0.8)
    fit_line = slope * t + intercept
    ax.plot(t, fit_line, "--", color="#d9534f", label=f"drift: {slope*60:.3f} mV/min")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("RMS AC Amplitude [mV]")
    ax.set_title("Temporal Stability at Fixed Point")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.6)
    plt.tight_layout()
    plt.savefig("stability_analysis.png", dpi=150)
    print("  Saved: stability_analysis.png")

    return {"mean": mean_rms, "std": std_rms, "cv": cv, "drift_per_min": slope * 60}


# ============================================================
# 5. SNR confidence-gating summary (across all data)
# ============================================================
def summarize_snr_gating(rows):
    snr_vals = np.array([r["snr_db"] for r in rows if isinstance(r.get("snr_db"), float) and not np.isnan(r["snr_db"])])
    if len(snr_vals) == 0:
        print("\n[SNR gating] No valid SNR data found.")
        return

    print("\n" + "=" * 70)
    print("SNR SUMMARY & SUGGESTED CONFIDENCE GATE")
    print("=" * 70)
    print(f"  Overall SNR: mean={snr_vals.mean():.1f} dB, min={snr_vals.min():.1f} dB, max={snr_vals.max():.1f} dB")
    frac_below = np.mean(snr_vals < SNR_CONFIDENCE_THRESHOLD_DB) * 100
    print(f"  Fraction of trials below {SNR_CONFIDENCE_THRESHOLD_DB} dB threshold: {frac_below:.1f}%")
    print(f"  --> A controller gating on SNR >= {SNR_CONFIDENCE_THRESHOLD_DB} dB would reject "
          f"{frac_below:.1f}% of this dataset's trials as untrustworthy.")
    print("  Adjust SNR_CONFIDENCE_THRESHOLD_DB above once you've seen where your real")
    print("  distinguishability breaks down (cross-reference with angle/distance tables).")


if __name__ == "__main__":
    all_rows = load_trials(TRIALS_LOG_PATH)
    if not all_rows:
        print(f"No data found at {TRIALS_LOG_PATH}. Run collect_sweep_data.py first.")
    else:
        sessions_present = sorted(set(r["session"] for r in all_rows))
        print(f"Loaded {len(all_rows)} trials across sessions: {sessions_present}")

        analyze_distance_sweep(all_rows)
        analyze_angle_sweep(all_rows)
        analyze_repeatability(all_rows, "theta_deg", "theta_deg")
        analyze_repeatability(all_rows, "distance_cm", "distance_cm")
        summarize_snr_gating(all_rows)

    analyze_stability()

    plt.show()
