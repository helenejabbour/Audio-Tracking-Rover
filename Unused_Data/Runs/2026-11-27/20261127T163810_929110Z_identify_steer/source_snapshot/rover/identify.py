"""Phase D vehicle identification on the follower Pi (SunFounder PiCar-S), plus offline fits.

Run modes (on the Pi; each writes Runs/<date>/<stamp>_identify_<mode>/):

  steer    Servo sweep with the motors off; the operator types the measured road-wheel
           angle (deg, LEFT POSITIVE) at each servo command. -> hardware.steering_table
  speed    Timed straight runs at several motor commands (0..100) and TWO OR MORE durations.
           The operator types the distance travelled. For each command the distance is fitted
           as d = v*T + b: the slope v is the steady speed, independent of the start-up
           transient and of the coast-down after the stop (both sit in b). -> speed_table
  step     Video aid. A sync flick of the front wheels (90 -> 70 -> 90) marks t = 0 in the
           video; then a speed step (forward at --pwm for --duration, then stop: gives the
           speed lag AND the braking) or a steering step (--servo-from -> --servo-to ->
           back). All command times are logged relative to the flick.
  latency  ESP32 on the Pi's USB: round-trip of '?' -> 'C2' and the R2 arrival jitter.
           transport_delay ~ min(RTT)/2 + p95(arrival jitter). No motors.

Offline:

  fit-step RUN --video CSV --sync-video-s T
           Video positions (columns t_s and x_m, or t_s and wheel_deg) from any tracker
           (e.g. the Tracker app, or frame-by-frame). Fits a first-order lag with dead time
           to the step, and constant deceleration to the stop. Writes step_fit.json.
  fit RUN [RUN ...]
           Combines speed/steer/latency runs and any step_fit.json into a proposed
           `hardware` block plus vehicle keys (proposed_vehicle.json). Never edits configs.

Motion needs --drive, a typed confirmation and the calibrated SunFounder config
(--picar-db, absolute path). Motor commands are capped by --max-pwm (default 80) and each
run by 6 s; Ctrl-C or any error stops the motors.
"""
import argparse
import csv
import json
import random
import time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from .config import create_run, load_config, write_json
from .hardware import PICAR_STRAIGHT, PICAR_TURNING_MAX, open_picar, read_picar_database

MAX_RUN_S = 6.0
SYNC_FLICK = 70  # servo command for the video sync flick


# ----------------------------------------------------------------------------- operator I/O

def ask_float(ask, prompt, allow_skip=True):
    """Ask until a finite number (or 's' to skip -> None)."""
    while True:
        text = ask(prompt).strip().lower()
        if allow_skip and text in ('s', 'skip'):
            return None
        try:
            value = float(text)
            if np.isfinite(value):
                return value
        except ValueError:
            pass
        print('  enter a number' + (" or 's' to skip" if allow_skip else ''))


def confirm(ask, what):
    if ask(f'{what}\nType "go" to continue: ').strip().lower() != 'go':
        raise SystemExit('not confirmed; nothing moved')


def write_rows(path, rows):
    if not rows:
        return
    with Path(path).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)


def read_rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


# ----------------------------------------------------------------------------- run modes

def run_steer(front, back, servos, repeats, settle, ask, sleep=time.sleep):
    """Servo sweep, motors off. Alternates sweep direction per repeat (exposes backlash)."""
    back.speed = 0; back.stop()
    rows = []
    for r in range(repeats):
        order = servos if r % 2 == 0 else servos[::-1]
        for s in order:
            front.turn(int(s)); sleep(settle)
            angle = ask_float(ask, f'[rep {r+1}] servo {s}: road-wheel angle deg (left +, s=skip): ')
            if angle is not None:
                rows.append({'repeat': r+1, 'direction': 'up' if r % 2 == 0 else 'down', 'servo': int(s), 'wheel_deg': angle})
    front.turn(PICAR_STRAIGHT)
    return rows


def run_speed(front, back, pwms, durations, repeats, ask, seed=0, sleep=time.sleep, clock=time.monotonic):
    """Timed straight runs in random order (battery drift averages out)."""
    plan = [(p, d, r+1) for p in pwms for d in durations for r in range(repeats)]
    random.Random(seed).shuffle(plan)
    rows = []
    front.turn(PICAR_STRAIGHT)
    for i, (pwm, duration, rep) in enumerate(plan):
        text = ask(f'[{i+1}/{len(plan)}] pwm {pwm}, {duration:.1f} s: rover on the start mark, Enter to drive (q = quit): ')
        if text.strip().lower() == 'q':
            break
        battery = ask('  battery LEDs lit (0/1/2, Enter = same as before): ').strip() or (rows[-1]['battery_leds'] if rows else '')
        try:
            back.forward(); t0 = clock(); back.speed = int(pwm)
            sleep(duration)
        finally:
            back.speed = 0; back.stop()
        driven = clock() - t0
        dist = ask_float(ask, '  distance travelled, start mark to the same point on the rover (m, s=skip): ')
        if dist is not None:
            rows.append({'order': i+1, 'pwm': int(pwm), 'duration_cmd_s': duration, 'duration_s': round(driven, 4),
                         'repeat': rep, 'distance_m': dist, 'battery_leds': battery})
    return rows


def run_step(front, back, profile, pwm, duration, servo_from, servo_to, dwell, sleep=time.sleep, clock=time.monotonic):
    """Command sequence for a video step test; returns [(event, t_rel_s)] relative to the sync flick."""
    events = []
    front.turn(PICAR_STRAIGHT); back.speed = 0; back.stop(); sleep(1.0)
    t0 = clock(); front.turn(SYNC_FLICK); events.append(('sync_flick', 0.)); sleep(.15)
    front.turn(PICAR_STRAIGHT); events.append(('sync_back', clock()-t0)); sleep(1.0)
    try:
        if profile == 'speed':
            back.forward(); back.speed = int(pwm); events.append((f'speed_step_pwm_{pwm}', clock()-t0))
            sleep(duration)
            back.speed = 0; back.stop(); events.append(('stop', clock()-t0)); sleep(1.5)
        else:
            front.turn(int(servo_from)); events.append((f'servo_{servo_from}', clock()-t0)); sleep(dwell)
            front.turn(int(servo_to)); events.append((f'servo_step_{servo_from}_to_{servo_to}', clock()-t0)); sleep(dwell)
            front.turn(int(servo_from)); events.append((f'servo_step_{servo_to}_to_{servo_from}', clock()-t0)); sleep(dwell)
            front.turn(PICAR_STRAIGHT); events.append(('straight', clock()-t0))
    finally:
        back.speed = 0; back.stop()
    return [{'event': e, 't_rel_s': round(t, 4)} for e, t in events]


def run_latency(ser, duration, ping_period=.25, clock=time.monotonic):
    """'?' round trips and R2 arrival jitter on an open serial port (ESP32 V2 firmware)."""
    from .sensing import parse_v2
    rtts, arrivals = [], []
    buffer = b''; pending = None; next_ping = clock()+1.; start = clock(); t_dev0 = None; unwrapped = 0.; prev = None
    while clock()-start < duration:
        now = clock()
        if pending is None and now >= next_ping:
            ser.write(b'?\n'); pending = now; next_ping = now+ping_period
        chunk = ser.read(ser.in_waiting or 1)
        now = clock()
        buffer += chunk
        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)
            text = line.decode('ascii', errors='replace').strip()
            if text.startswith('C2,') and pending is not None:
                rtts.append(now-pending); pending = None
            elif text.startswith('R2,'):
                f = parse_v2(text)
                if f is None:
                    continue
                te = f[3]
                if prev is not None:
                    unwrapped += ((te-prev) & 0xffffffff)*1e-6
                prev = te
                arrivals.append((now, unwrapped))
        if pending is not None and now-pending > 1.:
            pending = None  # lost reply; ping again
    return rtts, arrivals


def latency_summary(rtts, arrivals):
    rtt = np.asarray(rtts, float)
    a = np.asarray(arrivals, float)
    jitter = (a[:, 0]-a[:, 1]) if len(a) else np.array([])
    jitter = jitter-jitter.min() if len(jitter) else jitter
    out = {'pings': int(len(rtt)), 'packets': int(len(a))}
    if len(rtt):
        out.update(rtt_min_s=float(rtt.min()), rtt_median_s=float(np.median(rtt)), rtt_p95_s=float(np.percentile(rtt, 95)))
    if len(jitter):
        out.update(jitter_p50_s=float(np.percentile(jitter, 50)), jitter_p95_s=float(np.percentile(jitter, 95)),
                   jitter_max_s=float(jitter.max()))
    if len(rtt) and len(jitter):
        out['transport_delay_s'] = float(rtt.min()/2+np.percentile(jitter, 95))
        out['note'] = ('one-way ~ min RTT/2 (the firmware only polls commands between captures, so the median '
                       'RTT includes up to one 32 ms capture); plus the p95 of extra buffering seen on R2 arrivals')
    return out


# ----------------------------------------------------------------------------- fits

def fit_speed(rows):
    """Per motor command: d = v*T + b over all durations/repeats. Returns table rows."""
    by = {}
    for r in rows:
        by.setdefault(int(r['pwm']), []).append((float(r['duration_s']), float(r['distance_m'])))
    out = []
    for pwm, pts in sorted(by.items()):
        T, d = np.array(pts).T
        if len(np.unique(np.round(T, 2))) < 2 or len(T) < 3:
            raise ValueError(f'pwm {pwm}: need >= 2 durations and >= 3 runs to separate speed from start/stop transients')
        X = np.column_stack([T, np.ones_like(T)])
        coef, *_ = np.linalg.lstsq(X, d, rcond=None)
        res = d-X @ coef; dof = len(d)-2
        s2 = float(res @ res/dof) if dof > 0 else 0.
        cov = s2*np.linalg.inv(X.T @ X)
        out.append({'pwm': pwm, 'speed_mps': float(coef[0]), 'speed_se_mps': float(np.sqrt(cov[0, 0])),
                    'intercept_m': float(coef[1]), 'runs': int(len(d)), 'residual_sd_m': float(np.sqrt(s2))})
    return out


def fit_steering(rows):
    """Mean road-wheel angle per servo command, and the backlash (up minus down sweep)."""
    by = {}
    for r in rows:
        by.setdefault(int(r['servo']), []).append((float(r['wheel_deg']), r.get('direction', '')))
    out = []
    for servo, pts in sorted(by.items()):
        a = np.array([p[0] for p in pts])
        up = [p[0] for p in pts if p[1] == 'up']; down = [p[0] for p in pts if p[1] == 'down']
        out.append({'servo': servo, 'wheel_deg': float(a.mean()), 'sd_deg': float(a.std(ddof=1)) if len(a) > 1 else None,
                    'backlash_deg': float(np.mean(up)-np.mean(down)) if up and down else None, 'n': int(len(a))})
    return out


def tables(speed_fit, steer_fit):
    """hardware.speed_table [[m/s, pwm]] and steering_table [[deg, servo]], with checks."""
    proposed, problems = {}, []
    if speed_fit:
        sp = sorted(([r['speed_mps'], r['pwm']] for r in speed_fit), key=lambda x: x[0])
        if any(b[1] <= a[1] or b[0] <= a[0] for a, b in zip(sp, sp[1:])):
            problems.append('speed is not strictly increasing with pwm: repeat the runs or drop a command')
        if sp[0][0] <= 0:
            problems.append(f'pwm {sp[0][1]} does not move the rover: start the table above the stall command')
        proposed['speed_table'] = sp
        proposed['min_speed'] = sp[0][0]; proposed['max_speed'] = sp[-1][0]
    if steer_fit:
        st = sorted(([r['wheel_deg'], r['servo']] for r in steer_fit), key=lambda x: x[0])
        servo = np.array([s[1] for s in st])
        if not (np.all(np.diff(servo) > 0) or np.all(np.diff(servo) < 0)):
            problems.append('road-wheel angle is not monotonic in the servo command')
        by_servo = sorted(st, key=lambda x: x[1])
        wheel_at = lambda s: np.interp(s, [x[1] for x in by_servo], [x[0] for x in by_servo])
        if wheel_at(PICAR_STRAIGHT-10) <= wheel_at(PICAR_STRAIGHT+10):
            problems.append('servo below 90 does not steer LEFT (positive angle): check the sign you entered')
        proposed['steering_table'] = st
        proposed['max_steer_deg'] = float(min(abs(st[0][0]), abs(st[-1][0])))
    return proposed, problems


def first_order_position(t, v, tau, theta):
    """x(t) after a speed step at t=0 from rest: v*(s - tau*(1-exp(-s/tau))), s = t-theta >= 0."""
    s = np.maximum(t-theta, 0.)
    return v*(s-tau*(1-np.exp(-s/tau)))


def fit_speed_step(t, x, t_step, t_stop):
    """First-order lag + dead time on [t_step, t_stop]; constant deceleration after t_stop."""
    t, x = np.asarray(t, float), np.asarray(x, float)
    on = (t >= t_step-.2) & (t <= t_stop)
    tt, xx = t[on]-t_step, x[on]
    x0 = float(np.median(x[(t < t_step) & (t > t_step-.5)])) if np.any((t < t_step) & (t > t_step-.5)) else xx[0]
    v0 = max((xx[-1]-xx[0])/max(tt[-1]-tt[0], 1e-3), 1e-3)
    res = least_squares(lambda q: x0+first_order_position(tt, q[0], q[1], q[2])-xx, [v0, .2, .05],
                        bounds=([0, .01, 0], [5, 3, .5]))
    v, tau, theta = res.x
    out = {'speed_mps': float(v), 'tau_speed_s': float(tau), 'dead_time_s': float(theta),
           'rise_rmse_m': float(np.sqrt(np.mean(res.fun**2)))}
    after = t > t_stop
    if after.sum() >= 3:
        xa = x[after]; ta = t[after]-t_stop
        x_stop = float(np.interp(t_stop, t, x)); x_end = float(np.median(xa[-3:]))
        d = x_end-x_stop
        out.update(stop_distance_m=d, brake_decel_mps2=float(v*v/(2*d)) if d > 0 else None,
                   note='brake_decel = v^2/(2 d_stop) includes the command latency: conservative')
    return out


def fit_steer_step(t, angle, t_step):
    """delta(t) = a0 + da*(1-exp(-(t-t_step-theta)/tau)) on the samples after the step."""
    t, a = np.asarray(t, float), np.asarray(angle, float)
    pre = (t < t_step) & (t > t_step-.5); a0 = float(np.median(a[pre])) if pre.any() else float(a[0])
    m = t >= t_step; tt = t[m]-t_step; aa = a[m]
    da0 = float(aa[-1]-a0)
    res = least_squares(lambda q: a0+q[0]*(1-np.exp(-np.maximum(tt-q[2], 0)/q[1]))-aa, [da0, .1, .03],
                        bounds=([-90, .005, 0], [90, 2, .5]))
    da, tau, theta = res.x
    return {'step_deg': float(da), 'tau_steer_s': float(tau), 'dead_time_s': float(theta),
            'rate_deg_s_peak': float(abs(da)/tau), 'rmse_deg': float(np.sqrt(np.mean(res.fun**2)))}


def combine(run_dirs):
    """Proposed hardware/vehicle values from identify runs (speed, steer, latency, step_fit.json)."""
    speed_rows, steer_rows, lat, steps, sources = [], [], [], [], []
    for d in map(Path, run_dirs):
        sources.append(str(d))
        if (d/'speed_runs.csv').exists():
            speed_rows += read_rows(d/'speed_runs.csv')
        if (d/'steer_points.csv').exists():
            steer_rows += read_rows(d/'steer_points.csv')
        if (d/'latency.json').exists():
            lat.append(json.loads((d/'latency.json').read_text(encoding='utf-8')))
        if (d/'step_fit.json').exists():
            steps.append(json.loads((d/'step_fit.json').read_text(encoding='utf-8')))
    speed_fit = fit_speed(speed_rows) if speed_rows else []
    steer_fit = fit_steering(steer_rows) if steer_rows else []
    hw, problems = tables(speed_fit, steer_fit)
    vehicle = {k: hw.pop(k) for k in ('min_speed', 'max_speed', 'max_steer_deg') if k in hw}
    sp = [s for s in steps if 'tau_speed_s' in s]; st = [s for s in steps if 'tau_steer_s' in s]
    if sp:
        vehicle['tau_speed'] = float(np.median([s['tau_speed_s'] for s in sp]))
        dec = [s['brake_decel_mps2'] for s in sp if s.get('brake_decel_mps2')]
        if dec:
            vehicle['braking_deceleration'] = float(min(dec))  # worst case
    if st:
        vehicle['tau_steer'] = float(np.median([s['tau_steer_s'] for s in st]))
        vehicle['steer_rate_deg_s'] = float(min(s['rate_deg_s_peak'] for s in st))
    delays = [x['transport_delay_s'] for x in lat if 'transport_delay_s' in x]
    if delays:
        vehicle['transport_delay'] = float(max(delays))
    return {'hardware': hw, 'vehicle': vehicle, 'speed_fit': speed_fit, 'steer_fit': steer_fit,
            'step_fits': steps, 'latency': lat, 'problems': problems, 'sources': sources,
            'note': 'Proposed values only. Copy into Config/measured_config.json after review; '
                    'wheelbase, clearance and turning_offset_at_calibration are entered by hand.'}


# ----------------------------------------------------------------------------- CLI

def main(argv=None, ask=input):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='mode', required=True)

    def motion(sp):
        sp.add_argument('--picar-db', required=True, help='ABSOLUTE path to the calibrated SunFounder config')
        sp.add_argument('--drive', action='store_true', help='required for any motor or servo motion')
        sp.add_argument('--notes', required=True, help='rover id, floor, battery, load')
    s = sub.add_parser('steer'); motion(s)
    s.add_argument('--servos', type=int, nargs='+', default=list(range(45, 136, 10)))
    s.add_argument('--repeats', type=int, default=2); s.add_argument('--settle', type=float, default=.5)
    s = sub.add_parser('speed'); motion(s)
    s.add_argument('--pwm', type=int, nargs='+', default=[25, 35, 45, 60, 80])
    s.add_argument('--durations', type=float, nargs='+', default=[2., 4.])
    s.add_argument('--repeats', type=int, default=3); s.add_argument('--max-pwm', type=int, default=80)
    s.add_argument('--seed', type=int, default=0)
    s = sub.add_parser('step'); motion(s)
    s.add_argument('--profile', choices=['speed', 'steer'], required=True)
    s.add_argument('--pwm', type=int, default=45); s.add_argument('--duration', type=float, default=3.)
    s.add_argument('--max-pwm', type=int, default=80)
    s.add_argument('--servo-from', type=int, default=90); s.add_argument('--servo-to', type=int, default=70)
    s.add_argument('--dwell', type=float, default=1.)
    s = sub.add_parser('latency'); s.add_argument('--port', required=True); s.add_argument('--duration', type=float, default=300.)
    s.add_argument('--delta', type=float, default=50.); s.add_argument('--notes', required=True)
    s = sub.add_parser('fit-step'); s.add_argument('run'); s.add_argument('--video', required=True)
    s.add_argument('--sync-video-s', type=float, required=True, help='video time of the sync flick')
    s.add_argument('--event', help='event name to fit (default: the speed or servo step in events.csv)')
    s = sub.add_parser('fit'); s.add_argument('runs', nargs='+'); s.add_argument('--out')
    for sp in sub.choices.values():
        sp.add_argument('--config')
    a = p.parse_args(argv)
    cfg = load_config(a.config)

    if a.mode == 'fit':
        result = combine(a.runs)
        out = Path(a.out) if a.out else create_run('identify_fit', cfg)/'proposed_vehicle.json'
        write_json(out, result)
        print(json.dumps({'hardware': result['hardware'], 'vehicle': result['vehicle'], 'problems': result['problems']}, indent=1))
        print(out)
        return result
    if a.mode == 'fit-step':
        run = Path(a.run); events = {r['event']: float(r['t_rel_s']) for r in read_rows(run/'events.csv')}
        video = read_rows(a.video); t = np.array([float(r['t_s']) for r in video])-a.sync_video_s
        name = a.event or next(e for e in events if e.startswith('speed_step') or e.startswith('servo_step'))
        if 'x_m' in video[0]:
            fit = fit_speed_step(t, [float(r['x_m']) for r in video], events[name], events.get('stop', t.max()))
        else:
            fit = fit_steer_step(t, [float(r['wheel_deg']) for r in video], events[name])
        fit.update(event=name, video=str(a.video), sync_video_s=a.sync_video_s)
        write_json(run/'step_fit.json', fit); print(json.dumps(fit, indent=1))
        return fit
    if a.mode == 'latency':
        import serial
        out = create_run('identify_latency', cfg)
        with serial.Serial(a.port, 115200, timeout=.05) as ser:
            time.sleep(2); ser.reset_input_buffer(); ser.write(f'P0\nD{a.delta}\n'.encode())
            rtts, arrivals = run_latency(ser, a.duration)
        summary = latency_summary(rtts, arrivals); summary.update(port=a.port, notes=a.notes)
        write_rows(out/'rtt.csv', [{'rtt_s': r} for r in rtts])
        write_rows(out/'arrivals.csv', [{'received_s': r, 'device_s': d} for r, d in arrivals])
        write_json(out/'latency.json', summary); print(json.dumps(summary, indent=1)); print(out)
        return summary

    # motion modes
    read_picar_database(a.picar_db)  # fail before anything moves
    if not a.drive:
        p.error('motion modes need --drive')
    if a.mode in ('speed', 'step'):
        pwms = a.pwm if isinstance(a.pwm, list) else [a.pwm]
        if not all(0 < x <= min(a.max_pwm, 100) for x in pwms):
            p.error(f'motor commands must lie in (0, --max-pwm={a.max_pwm}]')
    if a.mode == 'speed' and (max(a.durations) > MAX_RUN_S or len(set(a.durations)) < 2):
        p.error(f'speed needs >= 2 distinct durations, each <= {MAX_RUN_S} s')
    if a.mode == 'step' and a.duration > MAX_RUN_S:
        p.error(f'--duration must be <= {MAX_RUN_S} s')
    servos = a.servos if a.mode == 'steer' else ([a.servo_from, a.servo_to] if a.mode == 'step' else [])
    lo, hi = PICAR_STRAIGHT-PICAR_TURNING_MAX, PICAR_STRAIGHT+PICAR_TURNING_MAX
    if any(not lo <= s <= hi for s in servos):
        p.error(f'servo commands must lie in [{lo}, {hi}] (Front_Wheels.turn clamp)')
    floor = a.mode == 'speed' or (a.mode == 'step' and a.profile == 'speed')
    confirm(ask, 'The rover will DRIVE on the floor: clear 3 m ahead, hand near the power switch.' if floor
            else 'Front wheels will steer; motors stay off. Wheels lifted or rover free to steer.')
    out = create_run(f'identify_{a.mode}', cfg)
    session = dict(vars(a)); session['picar_database_values'] = read_picar_database(a.picar_db)
    write_json(out/'session.json', session)
    front = back = None
    try:
        front, back = open_picar(a.picar_db)
        if a.mode == 'steer':
            rows = run_steer(front, back, a.servos, a.repeats, a.settle, ask)
            write_rows(out/'steer_points.csv', rows); print(fit_steering(rows))
        elif a.mode == 'speed':
            rows = run_speed(front, back, a.pwm, a.durations, a.repeats, ask, a.seed)
            write_rows(out/'speed_runs.csv', rows)
            if rows:
                try:
                    print(json.dumps(fit_speed(rows), indent=1))
                except ValueError as exc:
                    print(f'fit not possible yet: {exc}')
        else:
            rows = run_step(front, back, a.profile, a.pwm, a.duration, a.servo_from, a.servo_to, a.dwell)
            write_rows(out/'events.csv', rows); print(rows)
    except KeyboardInterrupt:
        print('interrupted: motors stopped')
    finally:
        if back is not None:
            back.speed = 0; back.stop()
        if front is not None:
            front.turn(PICAR_STRAIGHT)
    print(out)
    return out


if __name__ == '__main__':
    main()
