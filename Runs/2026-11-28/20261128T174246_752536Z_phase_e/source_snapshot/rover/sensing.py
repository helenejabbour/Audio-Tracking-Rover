"""Versioned wire format and calibrated paired-amplitude measurement model."""
from dataclasses import dataclass
import binascii
import math
import numpy as np


QUALITY_MASK = 0b0111  # bit0 valid, bit1 clipped, bit2 sample-timing failure
ORDER_BIT = 0b1000     # bit3 capture order: 0 = A- captured first, 1 = A+ captured first


def quality_ok(flags):
    """True when the firmware marked the pair valid with no clip/timing fault.

    The capture-order bit (wired v2 and V2 firmware alternate it every pair) is
    informational and must not reject the pair.
    """
    return (int(flags) & QUALITY_MASK) == 1


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
        if any(v < 0 for v in values[6:10]) or not 0 <= values[-1] <= 15:
            return None
        if any(not 0 <= v <= 4095 for v in values[10:12]):
            return None
        if not (-85 <= values[4] < values[5] <= 85):
            return None
        return values
    except (ValueError, UnicodeError, OverflowError):
        return None


def encode_bins(seq, minus, plus, noise_minus, noise_plus):
    """B2 line: per-tone amplitudes of one pair (same seq as its R2 line)."""
    fields = ['B2', seq, len(minus)] + [f'{v:.2f}' for v in minus] + [f'{v:.2f}' for v in plus] +         [f'{noise_minus:.3f}', f'{noise_plus:.3f}']
    payload = ','.join(map(str, fields))
    return f'{payload}*{binascii.crc_hqx(payload.encode("ascii"), 0xffff):04X}'


def parse_bins(line):
    """Return (seq, minus[M], plus[M], noise_minus, noise_plus) or None.

    Amplitudes are raw per-tone Goertzel values (not noise-subtracted); the noise values
    are sqrt(mean power) of the in-band noise bins, as in R2.
    """
    try:
        payload, checksum = line.strip().split('*')
        if len(checksum) != 4 or binascii.crc_hqx(payload.encode('ascii'), 0xffff) != int(checksum, 16):
            return None
        p = payload.split(',')
        if p[0] != 'B2':
            return None
        seq, m = int(p[1]), int(p[2])
        if not 1 <= m <= 64 or len(p) != 5 + 2*m or not 0 <= seq <= 0xffffffff:
            return None
        values = np.array([float(v) for v in p[3:]])
        if not np.isfinite(values).all() or (values < 0).any():
            return None
        return seq, values[:m], values[m:2*m], float(values[-2]), float(values[-1])
    except (ValueError, UnicodeError, OverflowError):
        return None


def tone_frequencies(config_line):
    """Tone frequencies (Hz) from a multi-tone C2 line, or None for a legacy firmware line."""
    fields = dict(f.split('=', 1) for f in config_line.strip().split(',')[1:] if '=' in f)
    try:
        fs, n, m = int(fields['fs']), int(fields['N']), int(fields['tones'])
        k0, k1 = (int(v) for v in fields['tone_k'].split(':'))
    except (KeyError, ValueError):
        return None
    if m == 1:
        return [k0*fs/n]
    if m < 1 or (k1-k0) % (m-1):
        return None
    step = (k1-k0)//(m-1)
    return [(k0+i*step)*fs/n for i in range(m)]


def combine_bins(amplitudes, noise, weights=None):
    """Band-combined, noise-subtracted amplitude sqrt(max(0, sum w a^2 / sum w - N^2)).

    Power (not log or amplitude) averaging: across bins with decorrelated fringe phase the
    mean power converges to the incoherent sum |D|^2 + sum |R_i|^2 instead of a fringe sample.
    weights equalise the speaker/microphone response (e.g. 1/mean power per bin from a
    calibration session) so one loud bin does not dominate; None means equal weights,
    which reproduces the firmware's Am/Ap.
    """
    a = np.asarray(amplitudes, float)
    w = np.ones_like(a) if weights is None else np.asarray(weights, float)
    if w.shape != a.shape or (w < 0).any() or w.sum() <= 0:
        raise ValueError('bin weights must be non-negative, one per tone, not all zero')
    power = (w*a*a).sum()/w.sum() - noise*noise
    return float(np.sqrt(max(0., power)))


def apply_bins(fields, bins, weights):
    """Replace Am/Ap of parsed R2 fields by a weighted recombination of its B2 bins."""
    seq, minus, plus, nm, np_ = bins
    if seq != fields[0]:
        raise ValueError('B2 and R2 sequence numbers differ')
    out = list(fields)
    out[6], out[7] = combine_bins(minus, nm, weights), combine_bins(plus, np_, weights)
    return out


def tone_log_ratios(minus, plus, noise_minus, noise_plus):
    """Per-tone 1/2 ln(P+/P-) with noise power removed and floored at (N/2)^2."""
    m, p = np.asarray(minus, float), np.asarray(plus, float)
    pm = np.maximum(m*m - noise_minus**2, max(noise_minus**2/4, 1e-6))
    pp = np.maximum(p*p - noise_plus**2, max(noise_plus**2/4, 1e-6))
    return 0.5*np.log(pp/pm)


def per_tone_bearing(minus, plus, noise_minus, noise_plus, model):
    """Bearing offset e = beta - phi_c (deg) from a fitted per-tone model (rover.bearing_cal).

    Each tone k has its own linear map L_k = s_k*e + c_k, fitted in the room at the working
    dither; the estimates (L_k - c_k)/s_k are combined with inverse-variance weights w_k
    (BLUE). Tones where the horn has no bearing slope get no weight, which is what the
    equal-power combination could not do.
    """
    s = np.asarray(model['slope_per_deg'], float); c = np.asarray(model['intercept'], float)
    w = np.asarray(model['weight'], float)
    L = tone_log_ratios(minus, plus, noise_minus, noise_plus)
    if L.shape != s.shape:
        raise ValueError(f'bin_model has {len(s)} tones but the capture has {len(L)}')
    use = w > 0
    return float((w[use]*(L[use]-c[use])/s[use]).sum()/w[use].sum())


def apply_bin_model(fields, bins, model, cfg, weights=None):
    """R2 fields with Am/Ap replaced so that estimate() returns the per-tone (BLUE) bearing.

    The geometric mean sqrt(Am*Ap) stays the power-combined amplitude (range), and the
    ratio is set to exp(2*L) with L = e_hat/h, h = sigma^2/(2*delta) in degrees, so the
    standard estimator reproduces e_hat. Use with bearing_gain_correction = 1 and
    boresight_deg = 0: the per-tone model already contains the in-room correction.
    """
    seq, minus, plus, nm, np_ = bins
    if seq != fields[0]:
        raise ValueError('B2 and R2 sequence numbers differ')
    delta = (fields[5]-fields[4])/2
    if abs(delta-model['delta_deg']) > .6:
        raise ValueError(f"bin_model was fitted at delta={model['delta_deg']}, pair has {delta:.1f}")
    e_hat = per_tone_bearing(minus, plus, nm, np_, model)
    am, ap = combine_bins(minus, nm, weights), combine_bins(plus, np_, weights)
    out = list(fields)
    if am > 0 and ap > 0:
        L = e_hat/(cfg['sigma_deg']**2/(2*delta))
        g = np.sqrt(am*ap)
        out[6], out[7] = float(g*np.exp(-L/2)), float(g*np.exp(L/2))
    else:
        out[6], out[7] = am, ap
    return out


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
        # tm/tp are the A-/A+ capture midpoints; with alternating capture order
        # either may come first, so use the signed uint32 difference.
        d = ((tp-tm+0x80000000) & 0xffffffff)-0x80000000
        gap = abs(d)*1e-6
        later = tp if d >= 0 else tm
        after = ((te-later) & 0xffffffff)*1e-6
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
    if not quality_ok(pair.flags):
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
    # In-room slope correction kappa (rover.bearing_cal): reverberant power that is not
    # from the source direction compresses ln(A+/A-), so beta_hat = kappa*beta + b.
    # boresight_deg carries -b; kappa = 1 is the free-field Gaussian model.
    h = sigma*sigma/(2*delta)/cfg.get('bearing_gain_correction', 1.)
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
    g = error*h/(sigma*sigma)  # = error/(2*delta) when kappa = 1
    J = np.array([[rho/n*(-.5-g), rho/n*(-.5+g)], [-h, h]])
    # noise = firmware in-band N (rms amplitude of one noise bin). For a band-combined amplitude over M
    # tones the scatter it causes is sd(A) ~ N/sqrt(2M); 28 Nov motor runs: N 1.2 -> 7.9 at PWM 45 while
    # sd(A) stays 1.2-1.5 counts. noise_to_amplitude_sd = 1/sqrt(2M) encodes that (default 1 = raw N).
    sd_from_noise = cfg.get('noise_to_amplitude_sd', 1.)*noise
    log_variance = (np.maximum(cfg['amplitude_noise'], sd_from_noise)/amps)**2
    R = (J*log_variance) @ J.T
    # Bounded within-pair change is uncertainty, not a claim of simultaneity.
    # Optional relative range floor (measured: 0.20*rho in horn_model_parameters.txt).
    range_floor2 = cfg['range_noise_floor']**2 + (cfg.get('range_noise_floor_rel', 0.)*rho)**2
    R += np.diag([range_floor2 + (cfg['pair_range_rate_bound']*pair.gap/2)**2,
                  np.deg2rad(cfg['bearing_noise_floor_deg'])**2 +
                  (cfg['pair_bearing_rate_bound']*pair.gap/2)**2])
    return Measurement(pair.seq, pair.time, np.array([rho, beta]), R), 'accepted'


def amplitude(rho, beta, phi, cfg):
    error = phi + np.deg2rad(cfg['boresight_deg']) - beta
    return cfg['amplitude_constant']/rho**cfg['range_exponent'] * \
        np.exp(-error*error/(2*np.deg2rad(cfg['sigma_deg'])**2))
