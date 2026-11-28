"""Cartesian constant-velocity EKF with timestamped fixed-lag corrections.

    x = [relative x, relative y, target inertial vx, target inertial vy],
    all vectors resolved in the follower frame. No target state is supplied.
"""
from dataclasses import dataclass
import bisect
import numpy as np
from .model import cartesian_transition, polar_output, wrap


@dataclass
class Node:
    time: float
    x: np.ndarray
    P: np.ndarray
    control: tuple = (0., 0.)


class Observer:
    def __init__(self, cfg):
        self.cfg = cfg
        self.nodes = []
        self.last_measurement = -np.inf
        self.last_nis = None

    @property
    def initialized(self):
        return bool(self.nodes)

    @property
    def state(self):
        return self.nodes[-1].x

    @property
    def covariance(self):
        return self.nodes[-1].P

    def initialize(self, measurement):
        rho, beta = measurement.y
        c, s = np.cos(beta), np.sin(beta)
        J = np.array([[c, -rho*s], [s, rho*c]])
        initial_speed = 0. if self.cfg.get('mission','follow') == 'approach' else self.cfg['nominal_speed']
        x = np.array([rho*c, rho*s, initial_speed, 0.])
        P = np.zeros((4, 4)); P[:2, :2] = J @ measurement.covariance @ J.T
        P[2:, 2:] = .15**2*np.eye(2)
        self.nodes = [Node(measurement.time, x, P)]
        self.last_measurement = measurement.time

    def _predict(self, x, P, dt, u):
        return cartesian_transition(x, P, dt, *u, self.cfg['process_acceleration_noise'])

    def predict(self, now, speed, yaw):
        if not self.nodes:
            return
        old = self.nodes[-1]
        if now < old.time-1e-9:
            raise ValueError('prediction time must be monotonic')
        if now <= old.time + 1e-9:
            return
        old.control = (speed, yaw)
        x, P = self._predict(old.x, old.P, now-old.time, old.control)
        self.nodes.append(Node(now, x, P))
        while len(self.nodes) > 2 and self.nodes[1].time < now-self.cfg['history_seconds']:
            self.nodes.pop(0)

    def correct(self, measurement):
        if not self.initialized:
            self.initialize(measurement)
            return True, 'initialized'
        t = measurement.time
        if t <= self.last_measurement + 1e-9:
            return False, 'out_of_order'
        if t < self.nodes[0].time or t > self.nodes[-1].time+1e-8:
            return False, 'outside_history'
        i = bisect.bisect_right([n.time for n in self.nodes], t)-1
        node = self.nodes[i]
        x, P = self._predict(node.x, node.P, max(0., t-node.time), node.control)
        y, _, H = polar_output(x, P)
        residual = measurement.y-y; residual[1] = wrap(residual[1])
        S = H@P@H.T + measurement.covariance
        self.last_nis = float(residual @ np.linalg.solve(S, residual))
        if self.last_nis > self.cfg['innovation_gate']:
            return False, 'innovation_gate'
        K = np.linalg.solve(S, H@P).T
        x += K@residual
        I = np.eye(4)-K@H
        P = I@P@I.T + K@measurement.covariance@K.T  # Joseph form
        P = (P+P.T)/2
        if abs(t-node.time) < 1e-8:
            node.x, node.P = x.copy(), P.copy()
        else:
            # Preserve a fractional-time correction for the next delayed update.
            i += 1
            self.nodes.insert(i, Node(t, x.copy(), P.copy(), node.control))
        clock = t
        # Serial capture order is enforced, so no later accepted corrections are lost.
        for j in range(i+1, len(self.nodes)):
            x, P = self._predict(x, P, self.nodes[j].time-clock, self.nodes[j-1].control)
            self.nodes[j].x, self.nodes[j].P = x.copy(), P.copy()
            clock = self.nodes[j].time
        if i == len(self.nodes)-1:
            self.nodes[i].x, self.nodes[i].P = x, P
        self.last_measurement = t
        return True, 'accepted'
