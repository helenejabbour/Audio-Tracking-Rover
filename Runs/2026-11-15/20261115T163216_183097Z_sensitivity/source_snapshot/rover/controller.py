"""Hybrid acquisition/track/hold/stop supervisor with realizable car commands."""
import numpy as np
from .model import design, polar_output
from .observer import Observer
from .sensing import estimate


class Tracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.observer = Observer(cfg)
        self.K = design(cfg)['K']
        self.integral = 0.
        self.mode = 'ACQUIRE'
        self.reason = 'waiting_for_measurements'
        self.consecutive = 0
        self.last_accepted = -np.inf
        self.last_seq = None
        self.last_control_time = None
        self.speed_est = 0.
        self.steer_est = 0.
        self.speed_command = 0.
        self.steer_command = 0.
        self.stop_latched = False
        self.stop_reason = 'near_target_latched'
        self.horn_center = 0.
        self.scan_start = 0.
        self.accepted = 0
        self.rejected = 0

    def advance(self, now):
        if self.last_control_time is None:
            self.last_control_time = now
            return
        dt = now-self.last_control_time
        if dt < -1e-9:
            raise ValueError('nonmonotonic runtime clock')
        if dt > 1e-9:
            # An actuator model is a prediction, not measured odometry.
            self.speed_est += (1-np.exp(-dt/self.cfg['tau_speed']))*(self.speed_command-self.speed_est)
            self.steer_est += (1-np.exp(-dt/self.cfg['tau_steer']))*(self.steer_command-self.steer_est)
            yaw = self.speed_est/self.cfg['wheelbase']*np.tan(self.steer_est)
            self.observer.predict(now, self.speed_est, yaw)
        self.last_control_time = now

    def ingest(self, pair, now):
        self.advance(now)
        if self.last_seq is not None and not 0 < ((pair.seq-self.last_seq) & 0xffffffff) < 0x80000000:
            self.rejected += 1
            return False, 'duplicate_or_reset'
        self.last_seq = pair.seq
        measurement, reason = estimate(pair, self.cfg)
        age = now-pair.time
        if measurement is None or age < -.001 or age > self.cfg['max_measurement_age']:
            self.rejected += 1
            return False, reason if measurement is None else 'stale_measurement'
        was_initialized = self.observer.initialized
        ok, reason = self.observer.correct(measurement)
        if ok:
            if not was_initialized:
                self.observer.predict(now, self.speed_est, 0.)
            self.last_accepted = pair.time
            self.horn_center = float(np.rad2deg(measurement.y[1]))
            self.consecutive = min(self.cfg['acquire_pairs'], self.consecutive + 1)
            self.accepted += 1
        else:
            self.rejected += 1
        return ok, reason

    def command(self, now):
        self.advance(now)
        cfg = self.cfg
        if self.stop_latched:
            return self._stop('STOP', self.stop_reason)
        if not self.observer.initialized:
            return self._stop('ACQUIRE', 'waiting_for_measurements')
        if now-self.last_accepted > cfg['timeout']:
            # Clear stale target belief; reacquisition requires fresh consecutive pairs.
            self.observer = Observer(cfg)
            self.consecutive = 0
            self.scan_start = now
            return self._stop('HOLD', 'measurement_timeout')
        y, R, _ = polar_output(self.observer.state, self.observer.covariance)
        rho, beta = y
        sigma_r, sigma_b = np.sqrt(np.maximum(0, np.diag(R)))
        if self.consecutive < cfg['acquire_pairs'] and self.mode != 'TRACK':
            return self._stop('ACQUIRE', 'confirming_target')
        if sigma_r > cfg['max_range_std'] or sigma_b > np.deg2rad(cfg['max_bearing_std_deg']):
            return self._stop('HOLD', 'uncertain_state')
        if abs(beta) > np.deg2rad(55):
            return self._stop('HOLD', 'outside_forward_tracking_cone')
        # This bound assumes the calibrated range error model; acoustic fading can violate it.
        delay = cfg['timeout'] + cfg['tau_speed']
        available = rho-3*sigma_r-cfg['clearance']-cfg['stop_margin']
        if available <= 0:
            if rho <= cfg['clearance']+cfg['stop_margin']+.1:
                self.stop_latched = True
                return self._stop('STOP', 'near_target_latched')
            # A widened uncertainty interval is not evidence of physical proximity.
            # Stop, but allow fresh observations to recover before permanently latching.
            return self._stop('HOLD', 'clearance_uncertain')
        decel = cfg['braking_deceleration']
        safe_speed = max(0., -decel*delay+np.sqrt((decel*delay)**2+2*decel*available))
        x = self.observer.state
        target_speed = np.hypot(x[2], x[3])
        terminal_braking = self.speed_est**2/(2*cfg['braking_deceleration'])+self.speed_est*cfg['dt']+.02
        terminal_band = max(terminal_braking,cfg['min_speed']/cfg['approach_range_gain']+.02)
        if cfg.get('mission','follow') == 'approach' and rho <= cfg['desired_range']+terminal_band:
            self.stop_latched = True
            self.stop_reason = 'stationary_target_standoff'
            return self._stop('STOP', self.stop_reason)
        eps = float(np.arctan2(x[3], x[2])) if target_speed > .05 else 0.
        # The heading gain is used only in its local design region.
        eps = float(np.clip(eps, -np.deg2rad(45), np.deg2rad(45)))
        error = rho-cfg['desired_range']
        u = -self.K @ np.array([error, beta, eps, self.integral])
        desired_speed = cfg['baseline_speed'] if cfg['controller'] == 'bearing_only' else cfg['nominal_speed']+u[0]
        if cfg.get('mission','follow') == 'approach':
            desired_speed = cfg['approach_range_gain']*error
            u[1] = cfg['approach_bearing_gain']*beta
        speed = float(np.clip(desired_speed, 0, min(cfg['max_speed'], safe_speed)))
        # Large bearing => reduce forward speed while retaining feasible curvature.
        speed *= max(.25, np.cos(beta)**2)
        if 0 < speed < cfg['min_speed']:
            speed = 0.
        max_yaw = speed/cfg['wheelbase']*np.tan(np.deg2rad(cfg['max_steer_deg']))
        yaw = float(np.clip(u[1], -max_yaw, max_yaw))
        steering = float(np.arctan2(cfg['wheelbase']*yaw, speed)) if speed > 0 else 0.
        if cfg['controller'] != 'bearing_only' and cfg.get('mission','follow') == 'follow':
            # Back calculation tracks the actually realizable speed, with a 1 s unwind time.
            ki = -self.K[0, 3]
            unwind = (speed-desired_speed)/ki if abs(ki) > 1e-8 else 0.
            self.integral = float(np.clip(self.integral+cfg['dt']*(error+unwind),
                                         -cfg['integral_limit'], cfg['integral_limit']))
        self.mode, self.reason = 'TRACK', 'tracking'
        self.speed_command, self.steer_command = speed, steering
        return speed, steering

    def _stop(self, mode, reason):
        self.mode, self.reason = mode, reason
        self.speed_command = self.steer_command = self.integral = 0.
        return 0., 0.

    def horn_command(self, now):
        limit = self.cfg['horn_limit_deg']-self.cfg['dither_deg']
        if self.mode == 'TRACK' and self.observer.initialized:
            self.horn_center = float(np.rad2deg(np.arctan2(self.observer.state[1], self.observer.state[0])))
        if self.mode == 'TRACK' or now-self.last_accepted < 3.:
            return float(np.clip(self.horn_center, -limit, limit))
        # Stationary scan; never rotate the chassis for acquisition.
        centers = [float(np.clip(self.horn_center, -limit, limit)), -limit, 0., limit]
        return centers[int((now-self.scan_start)/(self.cfg['measurement_period']*(self.cfg['acquire_pairs']+1))) % len(centers)]
