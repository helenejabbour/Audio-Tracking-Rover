from __future__ import annotations

import hashlib
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_config(path=None):
    path = Path(path) if path else ROOT / 'Config/simulation.json'
    cfg = json.loads(path.read_text(encoding='utf-8'))
    import math
    for key,value in cfg.items():
        if isinstance(value,(int,float)) and not isinstance(value,bool) and not math.isfinite(value):
            raise ValueError(f'{key} must be finite')
    positive = ('dt', 'desired_range', 'nominal_speed', 'wheelbase', 'max_speed',
                'tau_speed', 'tau_steer', 'measurement_period', 'sigma_deg',
                'amplitude_constant', 'range_exponent', 'dither_deg', 'timeout',
                'braking_deceleration', 'history_seconds')
    for key in positive:
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if not 0 <= cfg['min_speed'] <= cfg['nominal_speed'] <= cfg['max_speed']:
        raise ValueError('speed limits/nominal speed are inconsistent')
    if cfg['desired_range'] <= cfg['clearance'] + cfg['stop_margin']:
        raise ValueError('desired range must exceed clearance plus stop margin')
    if cfg['pair_gap'] / 2 + cfg['transport_delay'] > cfg['max_measurement_age']:
        raise ValueError('normal measurement age exceeds its acceptance limit')
    if cfg['history_seconds'] <= cfg['max_measurement_age']:
        raise ValueError('observer history is shorter than maximum accepted delay')
    if cfg['dither_deg'] >= cfg['horn_limit_deg']:
        raise ValueError('dither must leave room for horn pointing')
    if cfg.get('mission','follow') not in ('follow','approach'):
        raise ValueError('mission must be follow or approach')
    if cfg.get('approach_range_gain',0)<=0 or cfg.get('approach_bearing_gain',0)<=0:
        raise ValueError('approach gains must be positive')
    if not (0 < cfg['max_steer_deg'] < 60 and 0 < cfg['horn_limit_deg'] <= 75 and
            cfg['clearance'] > 0 and cfg['stop_margin'] >= 0 and cfg['transport_delay'] >= 0 and
            cfg['pair_gap'] > 0 and cfg['amplitude_noise'] >= 0 and cfg['range_noise_floor'] > 0 and
            cfg['bearing_noise_floor_deg'] > 0 and cfg['process_acceleration_noise'] >= 0 and
            cfg['acquire_pairs'] >= 2 and int(cfg['acquire_pairs']) == cfg['acquire_pairs']):
        raise ValueError('invalid physical, uncertainty or acquisition limits')
    return cfg


def create_run(label, config, root=None):
    now = datetime.now(timezone.utc)
    parent = Path(root) if root else ROOT / 'Runs'
    path = parent / now.strftime('%Y-%m-%d') / (now.strftime('%Y%m%dT%H%M%S_%fZ_') + label)
    path.mkdir(parents=True, exist_ok=False)
    (path / 'config.json').write_text(json.dumps(config, indent=2), encoding='utf-8')
    hashes = {}
    for directory in ('rover', 'Tests', 'Config', 'Firmware/esp32_acoustic_v2', 'Matlab'):
        for p in (ROOT / directory).rglob('*'):
            if p.is_file() and p.suffix in ('.py', '.json', '.ino', '.m') and '__pycache__' not in str(p):
                relative=p.relative_to(ROOT)
                hashes[relative.as_posix()] = hashlib.sha256(p.read_bytes()).hexdigest()
                snapshot=path/'source_snapshot'/relative
                snapshot.parent.mkdir(parents=True,exist_ok=True)
                snapshot.write_bytes(p.read_bytes())
    try:
        head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        head = None
    manifest = {'created_utc': now.isoformat(), 'python': platform.python_version(),
                'platform': platform.platform(), 'git_head': head, 'source_sha256': hashes}
    manifest['input_sha256']={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (ROOT/'Data').glob('*.csv')}
    (path / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return path


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
