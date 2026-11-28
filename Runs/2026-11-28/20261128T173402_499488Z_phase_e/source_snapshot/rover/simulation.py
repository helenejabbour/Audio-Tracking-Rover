"""Independent global-pose bicycle plant with sequential noisy acoustic captures."""
from dataclasses import replace
import heapq
import numpy as np
from .controller import Tracker
from .model import wrap
from .sensing import Pair, amplitude, ORDER_BIT


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
    # Multipath fringes and room bias as measured in Phase B s1 (per-tone BLUE, held out):
    # ~12 deg bearing scatter over a 7.5 cm shift (B2) and ~23 deg slowly varying room bias (E3).
    'fringe_room_s1': {'fringe_ln_sd': .144, 'room_bearing_sd_deg': 23.},
}

FRINGE_DEFAULTS = {'fringe_ln_sd': 0., 'fringe_length_m': .043, 'room_bearing_sd_deg': 0., 'room_length_m': 1.,
                   'room_range_ln_sd': 0., 'room_range_length_m': 1.}

# Living-room range law as measured on 28 Nov (29 battery captures, beta = 0, residuals against the
# pooled n = 1 fit): ln(A_measured/A_model) at the true range. 0.75 m reads ~20 % low (near reflection),
# 2.0 m ~32 % high (reverberant floor: rho_hat ~ 1.5 m). np.interp holds the end values outside the span.
S1_RANGE_BIAS_TABLE = [[.75, -.221], [1.0, .026], [1.3, -.020], [1.5, -.031], [2.0, .281]]


class RandomField:
    """Unit-variance stationary random field over R^dim (random Fourier features).

    Used for position-dependent multipath errors: the argument is the 4-vector
    (follower x, y, target x, y), because the path differences of reflections depend on
    both ends. Wavenumbers 2*pi/length*U(0.5, 1) in random directions, so the field
    decorrelates over roughly `length` of combined motion.
    """
    def __init__(self, rng, length, n=64, dim=4):
        d = rng.normal(size=(n, dim)); d /= np.linalg.norm(d, axis=1, keepdims=True)
        self.k = d*(2*np.pi/length)*rng.uniform(.5, 1., n)[:, None]
        self.phase = rng.uniform(0, 2*np.pi, n); self.scale = np.sqrt(2./n)

    def __call__(self, p):
        return float(self.scale*np.cos(self.k @ np.asarray(p, float)+self.phase).sum())


class MultipathModel:
    """Multiplicative capture error exp(e_side), e = fast fringe + antisymmetric room bias.

    fast: independent per horn side (the two captures see decorrelated fringe phase, measured
          corr ~ 0), sd fringe_ln_sd in ln-amplitude, period ~ fringe_length_m (lambda/2 at 4 kHz).
          It moves both the ratio (bearing) and the geometric mean (range).
    room: one slow field, +b/(2h) on A+ and -b/(2h) on A-, so it biases bearing by b deg
          (sd room_bearing_sd_deg) and leaves the range untouched (s1: range held out within ~8 %
          while bearing carried ~23 deg of room bias). h = sigma^2/(2 delta) in degrees.
    range: optional terms that act on both sides alike, so they bias range and not bearing:
          a slow pose-to-pose field (sd room_range_ln_sd; s1 E3 + 28 Nov: ~8 %) and the measured
          systematic range-law deviation range_bias_table [[rho, ln factor], ...] at the true range.
    A separate random stream keeps every other random draw of the simulation unchanged.
    """
    def __init__(self, params, cfg, seed, range_table=None):
        rng = np.random.default_rng([int(seed), 7919])
        self.p = params
        self.fast = {s: RandomField(rng, params['fringe_length_m']) for s in (-1, 1)}
        self.room = RandomField(rng, params['room_length_m'])
        # Drawn after the original fields, so runs without range terms are unchanged.
        self.room_range = RandomField(rng, params['room_range_length_m']) if params['room_range_ln_sd'] > 0 else None
        self.table = np.asarray(range_table, float) if range_table is not None else None
        self.h_deg = cfg['sigma_deg']**2/(2*cfg['dither_deg'])

    def gain(self, side, follower, target):
        pos = np.r_[follower[:2], target[:2]]
        e = self.p['fringe_ln_sd']*self.fast[side](pos)
        e += side*self.p['room_bearing_sd_deg']*self.room(pos)/(2*self.h_deg)
        if self.room_range is not None:
            e += self.p['room_range_ln_sd']*self.room_range(pos)
        if self.table is not None:
            e += float(np.interp(np.linalg.norm(target[:2]-follower[:2]), self.table[:, 0], self.table[:, 1]))
        return float(np.exp(e))


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
    # True sensor may differ from the calibrated model: constant boresight error = bearing bias.
    sensor = dict(cfg, boresight_deg=cfg['boresight_deg']+spec.get('boresight_error_deg', 0.))
    mp_params = {k: float(spec.get(k, cfg.get(k, v))) for k, v in FRINGE_DEFAULTS.items()}
    range_table = spec.get('range_bias_table', cfg.get('range_bias_table'))
    multipath = MultipathModel(mp_params, cfg, seed, range_table) \
        if mp_params['fringe_ln_sd'] > 0 or mp_params['room_bearing_sd_deg'] > 0 or \
        mp_params['room_range_ln_sd'] > 0 or range_table is not None else None
    # N the firmware would report in each capture (the live R reads it): 1 count = quiet bench;
    # set it to the motor-on in-band N to reproduce the live estimator with the drive running.
    reported_noise = float(cfg.get('sim_reported_noise', 1.))
    dt, duration = cfg['dt'], cfg['duration']
    initial_r = cfg['desired_range']+spec.get('initial_range_offset', .35) + rng.uniform(-.1, .1)
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
        if pending is not None and t >= pending['second_at']-1e-8:
            p = pending
            mp = multipath.gain(-1 if p['plus_first'] else 1, follower, target) if multipath else 1.
            second = amplitude(max(rho, .01), beta, p['phi2'], sensor)*p['gain2']*mp + p['noise2']
            am, ap = (second, p['a1']) if p['plus_first'] else (p['a1'], second)
            flags = (1 if am > 0 and ap > 0 else 0) | (ORDER_BIT if p['plus_first'] else 0)
            packet = Pair(seq, (p['t1']+t)/2, t-p['t1'], p['pm'], p['pp'],
                          max(.001, am), max(.001, ap), reported_noise, reported_noise, flags)
            if not p['drop']:
                arrival = t+cfg['transport_delay']+p['extra_delay']
                heapq.heappush(arrivals, (arrival, seq, packet))
            seq += 1; pending = None
        if t >= next_minus-1e-8 and pending is None:
            phi = np.deg2rad(tracker.horn_command(t))
            delta = np.deg2rad(cfg['dither_deg'])
            # Firmware clamps each capture angle to the physical horn span and reports the actual angle.
            pm, pp = np.clip([phi-delta, phi+delta], -np.deg2rad(cfg['horn_limit_deg']),
                             np.deg2rad(cfg.get('horn_limit_left_deg', cfg['horn_limit_deg'])))
            plus_first = bool(cfg.get('alternate_capture_order', False) and seq % 2 == 1)
            phi1, phi2 = (pp, pm) if plus_first else (pm, pp)
            ga = spec.get('gain_scale', 1.)
            reflected = rng.random() < spec.get('reflection', 0.)
            gain_m, gain_p = (np.exp(rng.normal(0, .8, 2)) if reflected else np.ones(2))*ga
            nm, np_ = rng.normal(0, cfg['amplitude_noise']*spec.get('noise_scale', 1.), 2)
            drop = rng.random() < spec.get('drop', 0.)
            outage = spec.get('outage', [-1., -1.])
            drop = drop or outage[0] <= t <= outage[1]
            pending = {'t1': t, 'second_at': t+gap, 'pm': pm, 'pp': pp, 'phi2': phi2,
                       'plus_first': plus_first,
                       'a1': amplitude(max(rho, .01), beta, phi1, sensor)*gain_m *
                             (multipath.gain(1 if plus_first else -1, follower, target) if multipath else 1.)+nm,
                       'gain2': gain_p, 'noise2': np_, 'drop': drop,
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
