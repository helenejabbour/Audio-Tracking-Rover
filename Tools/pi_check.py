#!/usr/bin/env python3
"""Non-moving readiness check for the follower Pi (run from the repo root, before Phase D).

  python3 Tools/pi_check.py --picar-db /home/pi/SunFounder_PiCar/picar/config --port /dev/ttyUSB0

Checks, read-only: Python version and Pi model; numpy/scipy/pyserial/smbus/RPi.GPIO/picar
imports; /dev/i2c-1 with the PCA9685 (0x40) and PCF8591 (0x48) answering a read; the
SunFounder config (rover.hardware.read_picar_database: absolute path, keys present, none
on the never-read last line); USB serial ports; and, with --port, the ESP32 V2 firmware
(C2 profile, valid CRC on R2/B2 lines). It never calls picar.setup() (that writes the
PCA9685) and never creates wheel objects, so nothing on the rover moves. The ESP32 horn
servo does dither once the ESP32 is powered, as always. Exit code 0 = all required checks pass.
"""
import argparse
import glob
import importlib
import json
import os
import platform
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# picar's PCA9685 driver imports smbus (apt: python3-smbus) and its TB6612 driver RPi.GPIO.
REQUIRED_IMPORTS = ['numpy', 'scipy', 'serial', 'smbus', 'RPi.GPIO', 'picar']


def pi_model():
    try:
        with open('/proc/device-tree/model', 'rb') as f:
            return f.read().decode(errors='replace').strip('\x00 \n')
    except OSError:
        return None


def check_imports():
    out = {}
    for name in REQUIRED_IMPORTS:
        try:
            m = importlib.import_module(name)
            out[name] = {'ok': True, 'version': getattr(m, '__version__', None), 'file': getattr(m, '__file__', None)}
        except Exception as exc:  # RPi.GPIO raises RuntimeError off-Pi or on a Pi 5
            out[name] = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
    return out


def check_i2c(bus_number=1, addresses=(0x40, 0x48)):
    if not os.path.exists(f'/dev/i2c-{bus_number}'):
        return {'ok': False, 'error': f'/dev/i2c-{bus_number} missing: enable I2C (raspi-config > Interface Options)'}
    try:
        import smbus
        bus = smbus.SMBus(bus_number)
    except Exception as exc:
        return {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
    found = {}
    for addr in addresses:
        try:
            bus.read_byte(addr); found[hex(addr)] = True
        except OSError:
            found[hex(addr)] = False
    return {'ok': found.get('0x40', False), 'devices': found,
            'note': '0x40 = PCA9685 (servo + motor PWM, required); 0x48 = PCF8591 ADC on the Robot HAT'}


def check_picar_db(path):
    from rover.hardware import read_picar_database
    try:
        return {'ok': True, 'values': read_picar_database(path)}
    except ValueError as exc:
        return {'ok': False, 'error': str(exc)}


def parse_esp32_lines(lines):
    """Summary of a few seconds of ESP32 output: C2 profile and CRC-valid R2/B2 counts."""
    from rover.sensing import parse_v2, parse_bins, tone_frequencies
    c2 = [l for l in lines if l.startswith('C2,')]
    r2 = [l for l in lines if l.startswith('R2,')]; b2 = [l for l in lines if l.startswith('B2,')]
    good_r2 = sum(parse_v2(l) is not None for l in r2); good_b2 = sum(parse_bins(l) is not None for l in b2)
    fields = dict(f.split('=', 1) for f in c2[-1].split(',')[1:] if '=' in f) if c2 else {}
    out = {'config_line': c2[-1] if c2 else None, 'profile': fields.get('profile'),
           'tone_hz': tone_frequencies(c2[-1]) if c2 else None,
           'r2': len(r2), 'r2_crc_ok': good_r2, 'b2': len(b2), 'b2_crc_ok': good_b2}
    out['ok'] = bool(c2) and good_r2 > 0 and good_r2 == len(r2) and good_b2 == len(b2)
    if out['profile'] not in (None, 'horn12'):
        out['warning'] = f"firmware profile {out['profile']}, Config/measured_horn12.json expects horn12"
    return out


def check_esp32(port, seconds=4.):
    try:
        import serial
        with serial.Serial(port, 115200, timeout=.2) as ser:
            # Leading newline: the port-open reset leaves garbage in the firmware's line buffer.
            time.sleep(2); ser.reset_input_buffer(); ser.write(b'\n?\n')
            lines, end = [], time.monotonic()+seconds
            while time.monotonic() < end:
                text = ser.readline().decode('ascii', errors='replace').strip()
                if text:
                    lines.append(text)
    except Exception as exc:
        return {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
    return parse_esp32_lines(lines)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--picar-db', help='absolute path to the calibrated SunFounder config')
    p.add_argument('--port', help='ESP32 serial port, e.g. /dev/ttyUSB0')
    a = p.parse_args(argv)
    model = pi_model()
    report = {'python': platform.python_version(), 'platform': platform.platform(), 'pi_model': model,
              'python_ok': sys.version_info >= (3, 9)}
    if model and 'Raspberry Pi 5' in model:
        report['pi5_warning'] = ('RPi.GPIO does not drive a Pi 5 (RP1); SunFounder_PiCar needs it for the TB6612 '
                                 'direction pins. Install rpi-lgpio (drop-in RPi.GPIO) or use a Pi 3B/3B+/4B.')
    report['imports'] = check_imports()
    report['i2c'] = check_i2c()
    report['serial_ports'] = sorted(glob.glob('/dev/ttyUSB*')+glob.glob('/dev/ttyACM*'))
    if a.picar_db:
        report['picar_db'] = check_picar_db(a.picar_db)
    if a.port:
        report['esp32'] = check_esp32(a.port)
    required = [report['python_ok'], report['i2c']['ok']] + [report['imports'][n]['ok'] for n in REQUIRED_IMPORTS]
    required += [report[k]['ok'] for k in ('picar_db', 'esp32') if k in report]
    report['all_required_ok'] = all(required)
    print(json.dumps(report, indent=1, default=str))
    for name, info in report['imports'].items():
        if not info['ok'] and name in REQUIRED_IMPORTS:
            print(f'MISSING {name}: {info["error"]}', file=sys.stderr)
    return 0 if report['all_required_ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
