"""Independent global-pose bicycle plant with sequential noisy acoustic captures."""
from dataclasses import replace
import heapq
import numpy as np
from .controller import Tracker
from .model import wrap
from .sensing import Pair, amplitude


SCENARIOS = {
    'nominal': {},
    'speed_step': {'speed_step': .05},
    'gentle_turn': {'target_yaw': .025},
    'noise_drops_reflections': {'noise_scale': 2., 'drop': .15, 'reflection': .12, 'jitter': .15},
    'outage': {'outage': [18., 23.]},
    'gain_bias_20pct': {'gain_scale': 1.2},
    'slow_sensor': {'period': 1.2, 'gap': .55},
    'legacy_12s_sensor': {'period': 12.84, 'gap': 6.42},
    'actuator_mismatch': {'speed_scale': .8, 'lag_scale': 1.7},
    'stationary_target': {'target_speed': 0., 'mission':'approach'},
    'no_beacon': {'drop': 1.},
    'unreachable_target': {'target_speed': .5},
}


def geometry(follower, target):
    delta = target[:2]-follower[:2]
    return float(np.linalg.norm(delta)), float(wrap(np.arctan2(delta[1], delta[0])-follower[2]))


def pose_step(pose, speed, yaw, dt):
    """Exact constant-twist planar pose update, independent of observer dynamics."""
    theta = pose[2]
    if abs(yaw) < 1e-9:
        pose[:2] += speed*dt*np.array([np.cos(theta), np.sin(theta)])
    else:
        pose[:2] += speed/yaw*np.array([np.sin(theta+yaw*dt)-np.sin(theta),
                                       np.cos(theta)-np.cos(theta+yaw*dt)])
    pose[2] = wrap(theta+yaw*dt)


def run_simulation(cfg, scenario='nominal', seed=0, record=True):
    spec = SCENARIOS[scenario] if isinstance(scenario, str) else scenario
    cfg = dict(cfg,mission=spec.get('mission',cfg.get('mission','follow')))
    rng = np.random.default_rng(seed)
    tracker = Tracker(cfg)
    dt, duration = cfg['dt'], cfg['duration']
    initial_r = cfg['desired_range']+.35 + rng.uniform(-.1, .1)
    initial_beta = np.deg2rad(rng.uniform(-15, 15))
    follower = np.zeros(3)
    target = np.array([initial_r*np.cos(initial_beta), initial_r*np.sin(initial_beta),
                       np.deg2rad(rng.uniform(-5, 5))])
    vf = steer = 0.
    seq = 0; next_minus = 0.; pending = None; arrivals = []
    rows = []; rejected = {}; accepted = 0
    period = spec.get('period', cfg['measurement_period'])
    gap = spec.get('gap', cfg['pair_gap'])
    period = max(period, gap+dt)
    times = np.arange(0, duration+dt/2, dt)
    for t in times:
        rho, beta = geometry(follower, target)
        tracker.advance(t)
        if pending is not None and t >= pending['plus_at']-1e-8:
            p = pending
            plus = amplitude(max(rho, .01), beta, p['pp'], cfg)*p['gain_p'] + p['noise_p']
            flags = 1 if p['am'] > 0 and plus > 0 else 0
            packet = Pair(seq, (p['tm']+t)/2, t-p['tm'], p['pm'], p['pp'],
                          max(.001, p['am']), max(.001, plus), 1., 1., flags)
            if not p['drop']:
                arrival = t+cfg['transport_delay']+p['extra_delay']
                heapq.heappush(arrivals, (arrival, seq, packet))
            seq += 1; pending = None
        if t >= next_minus-1e-8 and pending is None:
            phi = np.deg2rad(tracker.horn_command(t))
            delta = np.deg2rad(cfg['dither_deg'])
            ga = spec.get('gain_scale', 1.)
            reflected = rng.random() < spec.get('reflection', 0.)
            gain_m, gain_p = (np.exp(rng.normal(0, .8, 2)) if reflected else np.ones(2))*ga
            nm, np_ = rng.normal(0, cfg['amplitude_noise']*spec.get('noise_scale', 1.), 2)
            drop = rng.random() < spec.get('drop', 0.)
            outage = spec.get('outage', [-1., -1.])
            drop = drop or outage[0] <= t <= outage[1]
            pending = {'tm': t, 'plus_at': t+gap, 'pm': phi-delta, 'pp': phi+delta,
                       'am': amplitude(max(rho, .01), beta, phi-delta, cfg)*gain_m+nm,
                       'gain_p': gain_p, 'noise_p': np_, 'drop': drop,
                       'extra_delay': rng.uniform(0, spec.get('extra_delay', 0.))}
            next_minus = t + period*(1+rng.uniform(-1, 1)*spec.get('jitter', 0.))
        while arrivals and arrivals[0][0] <= t+1e-9:
            _, _, packet = heapq.heappop(arrivals)
            ok, reason = tracker.ingest(packet, t)
            if ok: accepted += 1
            else: rejected[reason] = rejected.get(reason, 0)+1
        speed_cmd, steer_cmd = tracker.command(t)
        if tracker.observer.initialized:
            x = tracker.observer.state
            rhat, bhat = np.hypot(x[0], x[1]), np.arctan2(x[1], x[0])
        else:
            rhat = bhat = np.nan
        rows.append([t, rho, beta, rhat, bhat, vf, steer, speed_cmd, steer_cmd,
                     follower[0], follower[1], target[0], target[1], tracker.mode,
                     tracker.reason, t-tracker.last_accepted if np.isfinite(tracker.last_accepted) else -1.])
        # Actuator limits, deadband, lags and independent global pose propagation.
        dv = (speed_cmd*spec.get('speed_scale', 1.)-vf)/ (cfg['tau_speed']*spec.get('lag_scale', 1.))
        vf = max(0., vf+np.clip(dv, -cfg['braking_deceleration'], cfg['accel_limit'])*dt)
        ds = (steer_cmd-steer)/(cfg['tau_steer']*spec.get('lag_scale', 1.))
        steer += np.clip(ds, -np.deg2rad(cfg['steer_rate_deg_s']), np.deg2rad(cfg['steer_rate_deg_s']))*dt
        yaw = vf/cfg['wheelbase']*np.tan(steer)
        pose_step(follower, vf, yaw, dt)
        vt = spec.get('target_speed', cfg['nominal_speed']) + (spec.get('speed_step', 0.) if t >= 20 else 0.)
        wt = spec.get('target_yaw', 0.) if 15 < t < 40 else 0.
        pose_step(target, vt, wt, dt)
    metrics = summarize(rows, cfg)
    metrics.update({'seed': seed, 'accepted': accepted, 'rejections': rejected})
    return metrics, rows if record else None


COLUMNS = ['time_s','true_range_m','true_bearing_rad','estimated_range_m','estimated_bearing_rad',
           'actual_speed_mps','actual_steering_rad','command_speed_mps','command_steering_rad',
           'follower_x_m','follower_y_m','target_x_m','target_y_m','mode','reason','measurement_age_s']


def summarize(rows, cfg):
    a = np.array([r[:13] for r in rows], float)
    t, rho, beta = a[:, 0], a[:, 1], a[:, 2]
    e = rho-cfg['desired_range']; tail = t >= t[-1]-10
    band = .02*abs(e[0])
    outside = np.flatnonzero(abs(e) > band)
    settling = None if len(outside) and outside[-1] == len(t)-1 else \
        float(t[outside[-1]+1]) if len(outside) else 0.
    overshoot = max(0., float(-np.min(e)*np.sign(e[0])/abs(e[0])*100)) if abs(e[0]) > 1e-8 else None
    ss = float(np.mean(e[tail])); brms = float(np.rad2deg(np.sqrt(np.mean(beta[tail]**2))))
    tracking_fraction = sum(r[13] == 'TRACK' for r in rows)/len(rows)
    safe = bool(np.min(rho) >= cfg['clearance'])
    return {'steady_range_error_m': ss, 'tail_range_rmse_m': float(np.sqrt(np.mean(e[tail]**2))),
            'tail_bearing_rms_deg': brms, 'settling_2pct_s': settling, 'overshoot_pct': overshoot,
            'minimum_range_m': float(np.min(rho)), 'safe_clearance': safe,
            'tracking_fraction': tracking_fraction, 'max_command_speed': float(a[:, 7].max()),
            'practical_tracking_pass': bool(abs(ss) <= .15 and brms <= 10 and safe and tracking_fraction >= .7),
            'original_O3_pass': bool(settling is not None and overshoot is not None and settling <= 6 and overshoot <= 15)}
