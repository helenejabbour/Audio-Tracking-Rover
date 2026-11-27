"""Phase B multi-tone session runner.

Walks through E0 silence, E0 tone, B0 sweep, B2 fringe test and the E3 held-out poses,
calling `python -m rover.collect` for each step and prompting for the physical setup.
Every run folder it creates is recorded in Runs/<date>/phase_b_<tag>_manifest.json, so the
analysis steps (rover.bearing_cal, rover.calibrate) know which folders belong to which step.

    python Tools/phase_b_session.py --port COM6 --tag s1 --volume "laptop 60 %, speaker 50 %" run
    python Tools/phase_b_session.py --port COM6 --tag s1 --only B0 run     # one step
    python Tools/phase_b_session.py --tag s1 analyze                       # after the capture
    python Tools/phase_b_session.py --port COM6 --tag s1 --dry-run run     # print commands only

Steps: E0S, E0T, B0, B2, E3 (default: all, in this order).

horn12 (Tier 1 session, on the rover Pi):
    python3 Tools/phase_b_session.py --profile h12 --port /dev/ttyUSB0 --tag h12a --volume "..." --room "..." run
"""
import argparse
import json
import subprocess
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CFG = 'Config/measured_multitone.json'
DELTA = 50.0

# Horn centre sweep, speaker fixed. rover.collect needs phi_c + delta <= 70 (servo end stop), so at
# delta = 50 the sweep ends at +20, not +30 (use --delta 40 for +/-30).
B0_PHI = list(range(-30, 21, 5))
B2_RHO = [1.000, 1.015, 1.030, 1.045, 1.060, 1.075]   # radial shifts of 1.5 cm
# Held-out E3 poses (rho, beta) taken from the Nov-3 grid so the old single-tone data are comparable.
E3_POSES = [(0.75, -30), (0.75, 15), (1.0, 0), (1.5, -30), (1.5, 0), (1.5, 30)]


# horn12 session (NEXT_STEPS "Next session (horn12)", trimmed for Tier 1): B0 training sweeps span the
# approach ranges and one off-axis line; E3 holds out new floor positions (none on a training line).
H12_B0_LINES = [(0.75, 0), (1.0, 0), (1.5, 0), (2.0, 0), (1.0, -20)]   # (rho, beta) per phi_c sweep
H12_E3_POSES = [(1.2, -30), (1.2, -15), (1.2, 0), (1.2, 15), (1.2, 30),
                (0.75, -15), (0.75, 15), (1.5, -15), (1.5, 15), (2.0, 0)]
PROFILES = {'mt': 'Config/measured_multitone.json', 'h12': 'Config/measured_horn12.json'}


def steps_h12():
    """horn12: yield (step, label, rho, beta, phi, duration, setup prompt)."""
    yield ('E0S', 'E0_silence_h12', 1.0, 0.0, 0.0, 60, 'Speaker OFF / muted (verify no sound in the room)')
    yield ('E0T', 'E0_tone_h12', 1.0, 0.0, 0.0, 60,
           'Speaker PLAYING horn12.wav (loop) at 1.0 m, beta 0, aimed at the horn; target median pp 1000-1500')
    for rho, beta in H12_B0_LINES:
        for i, phi in enumerate(B0_PHI):
            move = (f'MOVE speaker to rho={rho:.2f} m, beta={beta:+d}, aimed at the horn pivot. ' if i == 0
                    else 'Speaker unchanged. ')
            yield ('B0', f'B0_h12_r{rho:.2f}_b{beta:+d}_phi{phi:+d}', rho, float(beta), float(phi), 15,
                   move + f'Horn centre phi_c={phi:+d} deg (sent by the script)')
    for rho in B2_RHO:
        yield ('B2', f'B2_h12_rho{rho:.3f}', rho, 0.0, 0.0, 15,
               f'Move speaker radially to rho={rho:.3f} m (beta=0, aim and height unchanged)')
    for rho, beta in H12_E3_POSES:
        yield ('E3', f'E3_h12_r{rho:.2f}_b{beta:+d}', rho, float(beta), 0.0, 20,
               f'Place speaker at rho={rho:.2f} m, beta={beta:+d} deg, aimed at the horn pivot')


def steps(b0_rho=1.0):
    """Yield (step, label, rho, beta, phi, duration, setup prompt)."""
    yield ('E0S', 'E0_silence_mt', 1.0, 0.0, 0.0, 60, 'Speaker OFF / muted (verify no sound in the room)')
    yield ('E0T', 'E0_tone_mt', 1.0, 0.0, 0.0, 60, 'Speaker PLAYING the multi-tone WAV at 1.0 m, beta 0')
    for phi in B0_PHI:
        tag = '' if b0_rho == 1.0 else f'_r{b0_rho:.3f}'
        yield ('B0', f'B0_mt_phi{phi:+d}{tag}', b0_rho, 0.0, float(phi), 15,
               f'Speaker fixed at rho={b0_rho:.3f} m, beta=0, playing. Horn centre phi_c={phi:+d} deg (sent by the script)')
    for rho in B2_RHO:
        yield ('B2', f'B2_mt_rho{rho:.3f}', rho, 0.0, 0.0, 15,
               f'Move speaker radially to rho={rho:.3f} m (beta=0, aim and height unchanged)')
    for rho, beta in E3_POSES:
        yield ('E3', f'E3_mt_r{rho:.2f}_b{beta:+d}', rho, float(beta), 0.0, 20,
               f'Place speaker at rho={rho:.2f} m, beta={beta:+d} deg, aimed at the horn pivot')


def run_dir():
    d = ROOT / 'Runs' / date.today().isoformat()
    d.mkdir(parents=True, exist_ok=True)
    return d


def manifest_path(tag):
    return run_dir() / f'phase_b_{tag}_manifest.json'


def load_manifest(tag):
    p = manifest_path(tag)
    return json.loads(p.read_text()) if p.exists() else {'session': {}, 'runs': []}


def do_run(a):
    if not a.port and not a.dry_run:
        sys.exit('--port required')
    wanted = set(a.only.split(',')) if a.only else None
    man = load_manifest(a.tag)
    cfg = PROFILES[a.profile]
    man['session'] = {'volume': a.volume, 'room': a.room, 'config': cfg, 'delta': DELTA, 'profile': a.profile}
    done = {r['label'] for r in man['runs']}
    plan = steps_h12() if a.profile == 'h12' else steps(a.b0_rho)
    for step, label, rho, beta, phi, dur, prompt in plan:
        if wanted and step not in wanted:
            continue
        if label in done and not a.redo:
            print(f'skip {label} (already captured; use --redo)')
            continue
        notes = f'{step} {a.profile} {a.tag}; vol {a.volume}; room {a.room}'
        cmd = [sys.executable, '-m', 'rover.collect', '--port', a.port or 'COMX', '--config', cfg,
               '--label', label, '--rho', str(rho), '--beta', str(beta), '--phi', str(phi),
               '--delta', str(DELTA), '--duration', str(dur), '--notes', notes]
        print(f'\n[{step}] {prompt}\n    {" ".join(cmd[2:])}')
        if a.dry_run:
            continue
        ans = input('    Enter = capture, s = skip, q = quit > ').strip().lower()
        if ans == 'q':
            break
        if ans == 's':
            continue
        before = {p.name for p in run_dir().glob('*_capture')}
        subprocess.run(cmd, cwd=ROOT, check=False)
        new = sorted({p.name for p in run_dir().glob('*_capture')} - before)
        if not new or not (run_dir() / new[-1] / 'samples.csv').exists():
            print('    !! no data captured (empty run); not recorded - re-run this step with --redo')
            continue
        man['runs'] = [r for r in man['runs'] if r['label'] != label]
        man['runs'].append({'step': step, 'label': label, 'rho': rho, 'beta': beta, 'phi': phi,
                            'folder': str((run_dir() / new[-1]).relative_to(ROOT)).replace('\\', '/')})
        manifest_path(a.tag).write_text(json.dumps(man, indent=1))
        print(f'    recorded {new[-1]}')


def folders(man, *names):
    return [r['folder'] for r in man['runs'] if r['step'] in names
            and (ROOT / r['folder'] / 'samples.csv').exists()]


def py(*args):
    cmd = [sys.executable, '-m', *args]
    print('\n$', ' '.join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=False)


def do_analyze(a):
    man = load_manifest(a.tag)
    if not man['runs']:
        sys.exit(f'no runs recorded for tag {a.tag}')
    b0, b2, e3 = folders(man, 'B0'), folders(man, 'B2'), folders(man, 'E3')
    if man['session'].get('profile') == 'h12':
        cfg = PROFILES['h12']
        if b0:
            print('\n== B0_h12 per-tone BLUE model, validated on held-out E3_h12 ==')
            py('rover.bearing_cal', '--config', cfg, '--per-tone', *(['--validate', *e3] if e3 else []), '--', *b0)
        if b0 and b2:
            print('\n== same model on the B2_h12 fringe positions ==')
            py('rover.bearing_cal', '--config', cfg, '--per-tone', '--validate', *b2, '--', *b0)
        train = [f"{r['folder']}/samples.csv" for r in man['runs'] if r['step'] == 'B0' and r['phi'] == 0 and r['beta'] == 0]
        if train and e3:
            print('\n== Range (K): B0 phi_c=0 lines (beta=0) -> held-out E3 ==')
            py('rover.calibrate', '--config', cfg, '--train', *train, '--validation', *[f'{f}/samples.csv' for f in e3])
        print('\nE0 folders:', folders(man, 'E0S', 'E0T'))
        return
    if b0:
        print('\n== B0: multi-tone, equalised (kappa, offset, bin_weights) ==')
        py('rover.bearing_cal', '--config', CFG, '--equalize', '--', *b0)
        print('\n== B0: multi-tone, equal weights ==')
        py('rover.bearing_cal', '--config', CFG, '--', *b0)
        print('\n== B0: same captures, 4 kHz bin only (single-tone comparison) ==')
        py('rover.bearing_cal', '--config', CFG, '--tones', '4000', '--', *b0)
    if b2:
        print('\n== B2: per-position (read the swing), then spatially pooled ==')
        py('rover.bearing_cal', '--config', CFG, '--equalize', '--', *b2)
        py('rover.bearing_cal', '--config', CFG, '--equalize', '--pool', '--', *b2)
        print('\n== B2: 4 kHz only, for the swing comparison ==')
        py('rover.bearing_cal', '--config', CFG, '--tones', '4000', '--', *b2)
    if e3:
        print('\n== E3 held-out: bias and per-pose RMS ==')
        py('rover.bearing_cal', '--config', CFG, '--equalize', '--', *e3)
        print('\n== E3 held-out: 4 kHz only ==')
        py('rover.bearing_cal', '--config', CFG, '--tones', '4000', '--', *e3)
        print('\nRange (K, n): run rover.calibrate on the E3 folders (see its --help); '
              'K must be re-fit for the multi-tone level.')
    print('\nE0 folders:', folders(man, 'E0S', 'E0T'),
          '\n  -> summary.json: silence 0 accepted; tone T_m p95; noise level for amplitude_noise.')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=['run', 'analyze'])
    p.add_argument('--port')
    p.add_argument('--tag', required=True, help='session id, e.g. s1')
    p.add_argument('--profile', choices=sorted(PROFILES), default='mt',
                   help='mt: multitone (s1); h12: horn12 grid, Config/measured_horn12.json, _h12 labels')
    p.add_argument('--volume', default='UNLOGGED', help='playback + speaker volume, recorded in the notes')
    p.add_argument('--room', default='UNLOGGED')
    p.add_argument('--only', help='comma list of steps: E0S,E0T,B0,B2,E3')
    p.add_argument('--b0-rho', type=float, default=1.0, help='speaker distance for the B0 sweep (repeat B0 at shifted positions)')
    p.add_argument('--redo', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    (do_run if a.mode == 'run' else do_analyze)(a)


if __name__ == '__main__':
    main()
