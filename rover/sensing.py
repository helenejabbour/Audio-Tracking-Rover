"""Versioned wire format and calibrated paired-amplitude measurement model."""
from dataclasses import dataclass
import binascii
import math
import numpy as np


@dataclass(frozen=True)
class Pair:
    seq: int
    time: float
    gap: float
    phi_minus: float
    phi_plus: float
    amplitude_minus: float
    amplitude_plus: float
    noise_minus: float
    noise_plus: float
    flags: int = 1


@dataclass(frozen=True)
class Measurement:
    seq: int
    time: float
    y: np.ndarray
    covariance: np.ndarray


def encode_v2(fields):
    payload = ','.join(map(str, ['R2'] + list(fields)))
    crc = binascii.crc_hqx(payload.encode('ascii'), 0xffff)
    return f'{payload}*{crc:04X}'


def parse_v2(line):
    """Return typed fields. Reject nonfinite data, malformed CRC and impossible clocks."""
    try:
        payload, checksum = line.strip().split('*')
        if len(checksum) != 4 or binascii.crc_hqx(payload.encode('ascii'), 0xffff) != int(checksum, 16):
            return None
        p = payload.split(',')
        if len(p) != 14 or p[0] != 'R2':
            return None
        values = [int(v) for v in p[1:5]] + [float(v) for v in p[5:11]] + [int(v) for v in p[11:14]]
        if not all(math.isfinite(v) for v in values):
            return None
        if any(not 0 <= v <= 0xffffffff for v in values[:4]):
            return None
        if any(v < 0 for v in values[6:10]) or not 0 <= values[-1] <= 7:
            return None
        if any(not 0 <= v <= 4095 for v in values[10:12]):
            return None
        if not (-85 <= values[4] < values[5] <= 85):
            return None
        return values
    except (ValueError, UnicodeError, OverflowError):
        return None


class DeviceClock:
    """Map device emission time to monotonic host time using a minimum-offset envelope.

    A calibrated fixed transport delay is removed. This detects additional buffering,
    not an unknown constant transport delay. Reboots/out-of-order packets are rejected;
    restart the run after a reboot. uint32 micros and sequence wraps are supported.
    """
    def __init__(self, transport_delay):
        self.transport = transport_delay
        self.previous = None
        self.previous_seq = None
        self.unwrapped = 0.
        self.offset = None

    def convert(self, fields, received):
        seq, tm, tp, te, pm, pp, am, ap, nm, np_, _, _, flags = fields
        if self.previous is not None:
            ds = (seq-self.previous_seq) & 0xffffffff
            dt = (te-self.previous) & 0xffffffff
            if not 0 < ds < 0x80000000 or not 0 < dt < 0x80000000:
                raise ValueError('duplicate, out-of-order packet or device reset')
            self.unwrapped += dt*1e-6
        gap = ((tp-tm) & 0xffffffff)*1e-6
        after = ((te-tp) & 0xffffffff)*1e-6
        if gap <= 0 or gap > 5 or after > 1:
            raise ValueError('invalid capture timestamps')
        self.previous, self.previous_seq = te, seq
        candidate = received-self.transport-self.unwrapped
        self.offset = candidate if self.offset is None else min(self.offset, candidate)
        midpoint = self.unwrapped+self.offset-after-gap/2
        return Pair(seq, midpoint, gap, np.deg2rad(pm), np.deg2rad(pp), am, ap, nm, np_, flags)


def legacy_row(row, time_s):
    phi, delta = np.deg2rad([float(row['phi_c']), float(row['dlt'])])
    return Pair(int(row['seq']), time_s, float(row['dt_pair_ms'])*.001,
                phi-delta, phi+delta, float(row['A_m']), float(row['A_p']),
                float(row['Aref_m']), float(row['Aref_p']), int(row['flags']))


def estimate(pair, cfg):
    vals = [pair.time, pair.gap, pair.phi_minus, pair.phi_plus,
            pair.amplitude_minus, pair.amplitude_plus, pair.noise_minus, pair.noise_plus]
    if not np.isfinite(vals).all():
        return None, 'nonfinite'
    if pair.flags != 1:
        return None, 'firmware_quality'
    if pair.gap <= 0 or pair.gap > 2*cfg['max_measurement_age']:
        return None, 'pair_too_slow'
    amps = np.array([pair.amplitude_minus, pair.amplitude_plus])
    noise = np.array([pair.noise_minus, pair.noise_plus])
    if (noise < 0).any() or (amps < cfg['amplitude_min']).any() or \
            (amps < cfg['snr_ratio_min']*noise).any():
        return None, 'low_signal'
    phi = (pair.phi_minus + pair.phi_plus)/2
    delta = (pair.phi_plus - pair.phi_minus)/2
    if not np.deg2rad(.5) <= delta <= np.deg2rad(cfg['horn_limit_deg']):
        return None, 'dither'
    sigma = np.deg2rad(cfg['sigma_deg']); offset = np.deg2rad(cfg['boresight_deg'])
    h = sigma*sigma/(2*delta)
    beta = phi + offset + h*np.log(amps[1]/amps[0])
    error = phi + offset - beta
    if abs(error) > np.deg2rad(cfg['bearing_domain_deg']):
        return None, 'outside_angular_calibration'
    n = cfg['range_exponent']
    log_rho = (np.log(cfg['amplitude_constant']) - np.log(amps).mean() -
               (error*error+delta*delta)/(2*sigma*sigma))/n
    rho = float(np.exp(np.clip(log_rho, -30, 30)))
    if not cfg['range_domain'][0] <= rho <= cfg['range_domain'][1]:
        return None, 'outside_range_calibration'
    J = np.array([[rho/n*(-.5-error/(2*delta)), rho/n*(-.5+error/(2*delta))], [-h, h]])
    log_variance = (np.maximum(cfg['amplitude_noise'], noise)/amps)**2
    R = (J*log_variance) @ J.T
    # Bounded within-pair change is uncertainty, not a claim of simultaneity.
    R += np.diag([cfg['range_noise_floor']**2 + (cfg['pair_range_rate_bound']*pair.gap/2)**2,
                  np.deg2rad(cfg['bearing_noise_floor_deg'])**2 +
                  (cfg['pair_bearing_rate_bound']*pair.gap/2)**2])
    return Measurement(pair.seq, pair.time, np.array([rho, beta]), R), 'accepted'


def amplitude(rho, beta, phi, cfg):
    error = phi + np.deg2rad(cfg['boresight_deg']) - beta
    return cfg['amplitude_constant']/rho**cfg['range_exponent'] * \
        np.exp(-error*error/(2*np.deg2rad(cfg['sigma_deg'])**2))
