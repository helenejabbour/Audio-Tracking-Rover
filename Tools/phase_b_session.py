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
    man['session'] = {'volume': a.volume, 'room': a.room, 'config': CFG, 'delta': DELTA}
    done = {r['label'] for r in man['runs']}
    for step, label, rho, beta, phi, dur, prompt in steps(a.b0_rho):
        if wanted and step not in wanted:
            continue
        if label in done and not a.redo:
            print(f'skip {label} (already captured; use --redo)')
            continue
        notes = f'{step} multitone {a.tag}; vol {a.volume}; room {a.room}'
        cmd = [sys.executable, '-m', 'rover.collect', '--port', a.port or 'COMX', '--config', CFG,
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
