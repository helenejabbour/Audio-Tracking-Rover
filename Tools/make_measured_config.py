#!/usr/bin/env python3
"""Build Config/measured_config.json: Config/phase_e.json (design choices) + measured values.

    python Tools/make_measured_config.py --speed-table <proposed_vehicle.json>          # motors off
    python Tools/make_measured_config.py --speed-table <proposed_vehicle.json> --reported-noise 7.9 \
        --out Config/measured_config_motor.json                                          # drive at PWM 45

Sources (28 Nov 2026):
  sensor   K, n, sigma: pooled per-pair fit, s1 (20 Nov) + A2 (28 Nov), 1.0-1.5 m (Local/Docs/A2_RANGE_REFERENCE.md)
  bearing  bin_model: Runs/2026-11-28/20261128T062919_226852Z_bearing_cal (s1 B0 -> E3 held out 27.6 deg RMS)
  vehicle  steering table, transport delay: Config/rover1_hardware.json (27 Nov identify runs)
"""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BEARING_CAL = 'Runs/2026-11-28/20261128T062919_226852Z_bearing_cal'


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--amplitude-noise', type=float, default=1.9,
                   help='true per-amplitude noise sd [counts]; 1.9 = median per-pair A sd at 1.0 m, volume 80 %%, motors off '
                        'and at PWM 45/80 alike (28 Nov): the motors do not raise it')
    p.add_argument('--noise-to-amplitude-sd', type=float, default=0.30,
                   help='scale from firmware in-band N to the amplitude sd used in R (28 Nov: added bearing scatter at '
                        'PWM 45, volume 40 %%, N 7.0 -> 0.29; physics 1/sqrt(2 M_eff) = 0.18-0.22)')
    p.add_argument('--reported-noise', type=float,
                   help='simulator only: in-band N the firmware reports [counts] (motor variant: 7.9 = PWM 45 median)')
    p.add_argument('--amplitude-constant', type=float, default=67.4,
                   help='K at sigma 77: 30.9 (volume 20 %%, morning trimmer) x 2.18 (1.0 m median A ratio, '
                        'volume 80 %%, trimmer mid, refs 154247/154404 vs A2 0957-0958)')
    p.add_argument('--speed-table', help='proposed_vehicle.json from rover.identify fit (fills hardware.speed_table)')
    p.add_argument('--out', default='Config/measured_config.json')
    a = p.parse_args()

    cfg = json.loads((ROOT/'Config/phase_e.json').read_text(encoding='utf-8'))
    hw = json.loads((ROOT/'Config/rover1_hardware.json').read_text(encoding='utf-8'))
    cal = json.loads((ROOT/BEARING_CAL/'proposed_config.json').read_text(encoding='utf-8'))

    sensor = {
        'bin_model': cal['bin_model'], 'bin_weights': cal.get('bin_weights'),
        'bearing_gain_correction': 1.0, 'boresight_deg': 0.0,   # the per-tone model holds the in-room correction
        # sigma stays 77: it is also the simulator's bearing scale h = sigma^2/(2 delta), in which the
        # multipath levels (fringe_ln_sd) are calibrated. K refitted with sigma fixed at 77 (range near beta = 0 unchanged).
        'amplitude_constant': a.amplitude_constant, 'range_exponent': 1.0, 'sigma_deg': 77.0,
        'range_noise_floor_rel': 0.15,
        'range_domain': [0.75, 2.5],                            # hard gate on rho_hat; must contain desired_range
        'amplitude_noise': a.amplitude_noise,
        'noise_to_amplitude_sd': a.noise_to_amplitude_sd,
        # fringe_ln_sd / room_bearing_sd_deg stay 0, as in phase_e.json: rover.phase_e applies its multipath levels
        # (none / horn12 target / s1 measured 0.144, 23 deg) on top, and 'none' is an empty override.
        'measurement_period': 0.483,                            # T_m median, wide16, delta 50, 28 Nov captures
    }
    vehicle = {
        'transport_delay': hw['vehicle']['transport_delay'],    # 0.0254 s, identify latency 27 Nov
        'max_steer_deg': hw['vehicle']['max_steer_deg'],        # 45 deg, steering table 27 Nov
        'wheelbase': 0.138,                                     # SunFounder manual: R_min = 138 mm at 45 deg
    }
    # mission / desired_range stay at the phase_e design point (follow, d* = 1.4 m): rover.phase_e designs there
    # and overrides per tier; Tier 1 on hardware passes --mission approach --desired-range 1.0 on the command line.
    cfg.update(sensor); cfg.update(vehicle); cfg.update(controller='lqi', hardware_ready=False)
    if a.reported_noise is not None:
        cfg['sim_reported_noise'] = a.reported_noise

    block = dict(hw['hardware'])
    # hardware.py checks Path(p).is_file(): point at files, not run folders.
    block['evidence_files'] = [str(Path(e)/'manifest.json').replace('\\', '/') for e in block['evidence_files']]
    if a.speed_table:
        fit = json.loads(Path(a.speed_table).read_text(encoding='utf-8'))
        table = fit.get('speed_table') or fit.get('hardware', {}).get('speed_table')
        if not table:
            raise SystemExit(f'{a.speed_table}: no speed_table found')
        block['speed_table'] = table
        cfg['min_speed'], cfg['max_speed'] = float(table[0][0]), float(table[-1][0])
        cfg['nominal_speed'] = min(max(cfg['nominal_speed'], cfg['min_speed']), cfg['max_speed'])
    cfg['hardware'] = block

    cfg['provenance'] = ('Measured config, 28 Nov 2026: phase_e.json design choices + measured sensor (pooled s1 + A2 range fit, '
                         's1 bearing_cal bin_model), rover1 steering table and transport delay. '
                         + ('Speed table measured. ' if a.speed_table else 'Speed limits, tau_speed, tau_steer, braking PROVISIONAL. ')
                         + f'amplitude_noise {a.amplitude_noise} counts'
                         + (f', simulated in-band N {a.reported_noise} (drive on).' if a.reported_noise is not None else '.'))
    cfg['sources_measured'] = {
        'amplitude_constant / range_exponent / sigma_deg': 'pooled per-pair fit (rover.calibrate model), s1 20 Nov + A2 28 Nov, '
            '1.0-1.5 m, 3168 amplitudes, sigma fixed at 77 deg (simulator bearing scale), b 0, n 1: K 30.9 at volume 20 %, '
            'log-RMSE 0.122 (n free: 0.975 +- 0.026; sigma free: K 27.3, sigma 117, n 1.01, same range near beta = 0). '
            'Rescaled to the live setup (volume 80 %, gain trimmer mid, blanking firmware) by the 1.0 m median-A ratio 2.18: '
            f'K {a.amplitude_constant}. The bearing bin_model is a per-tone log ratio, so a flat gain/volume change cancels',
        'range_noise_floor_rel': '0.15 = per-pair ~10 % (+) pose-to-pose room bias ~+-10 % at 1.0-1.5 m',
        'range_domain': '0.75-2.5 m gate on rho_hat (the in-room rho_hat stays <= ~1.6 m for true ranges <= 2.0 m; '
            'the 2.5 m edge avoids artificial rejections at the simulated 2.0 m Tier 1 start). '
            'Accurate only on 1.0-1.5 m: 0.75 m reads -20 %, 2.0 m reads as 1.5 m',
        'bin_model': f'{BEARING_CAL} (s1 B0 training, E3 held out 27.6 deg RMS)',
        'amplitude_noise': f'{a.amplitude_noise} counts. Measured per-pair A sd at 1.0 m, volume 80 %: 1.9 motors off and at '
            'PWM 45/80 (per segment 1.2-1.5 with the drive on vs 1.3-2.8 off: motors do not raise the amplitude scatter); '
            'in-band N rises 1.2 -> 7.9 (PWM 45) / 5.6 (PWM 80). The motor variant keeps the true noise and sets '
            'sim_reported_noise to the motor N, which is what the live estimator sees',
        'noise_to_amplitude_sd': f'{a.noise_to_amplitude_sd}: R uses max(amplitude_noise, scale*N). Raw N overstates the '
            'scatter of a 16-tone amplitude ~sqrt(2 M) times; calibrated on the 28 Nov PWM 45 segment at volume 40 % '
            '(bearing sd 4.2 -> 11.0 deg with A 16.6, N 7.0 -> 0.29)',
        'speed_table': 'rover.identify fit, Runs/2026-11-28/20261128T155805_660561Z_identify_speed: PWM 25/45/80 -> '
            '0.041/0.100/0.203 m/s; listening while driving validated at PWM 45 and 80 (100 % valid, 0 % clip, volume 80 %)',
        'fringe_ln_sd / room_bearing_sd_deg': '0 in the config (phase_e.json convention); the s1 measured level (0.144, 23 deg) '
            'is applied per scenario by rover.phase_e. For other simulations pass it in the scenario.',
        'transport_delay / max_steer_deg': 'Config/rover1_hardware.json (identify latency / steer, 27 Nov)',
        'wheelbase': 'SunFounder PiCar-V manual, minimum turning radius 138 mm at 45 deg',
    }
    out = ROOT/a.out
    out.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
    print(out)


if __name__ == '__main__':
    main()
