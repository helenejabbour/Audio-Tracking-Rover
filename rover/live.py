"""Fixed-rate serial/replay runner. Dry-run is the default; never drives on replay."""
import argparse
import csv
import json
from pathlib import Path
import signal
import time
from .config import create_run,load_config,write_json
from .controller import Tracker
from .hardware import MotorProcess
from .link import open_link
from .sensing import DeviceClock,parse_v2,parse_bins,apply_bins,apply_bin_model,tone_frequencies,legacy_row
from .model import polar_output
import numpy as np


def replay(path,cfg,out):
    tracker=Tracker(cfg)
    with Path(path).open(newline='') as source:
        rows=list(csv.DictReader(source))
    results=[];t=0.;previous_ms=None;origin=None
    for row in rows:
        device=float(row['t_ms'])*.001
        if origin is None:origin=device
        received=device-origin
        if previous_ms is not None and device<=previous_ms:
            raise ValueError('legacy replay contains a device reset; split it into sessions')
        previous_ms=device
        while t < received:
            tracker.command(t);t+=cfg['dt']
        age=float(row['dt_pair_ms'])*.0005+cfg['transport_delay']
        pair=legacy_row(row,received-age)
        ok,reason=tracker.ingest(pair,t)
        speed,steer=tracker.command(t)
        results.append({'seq':pair.seq,'time_s':t,'accepted':ok,'reason':reason,
                        'mode':tracker.mode,'speed_mps':speed,'steering_rad':steer})
    with (out/'replay.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0]) if results else ['seq']);w.writeheader();w.writerows(results)
    summary={'input':str(Path(path).resolve()),'rows':len(rows),'accepted':sum(r['accepted'] for r in results),
             'nonzero_commands':sum(r['speed_mps']>0 for r in results),'hardware_driven':False}
    write_json(out/'summary.json',summary)
    return summary


def check_firmware_tones(text,model):
    """The per-tone bin_model is only valid for the beacon profile it was fitted on."""
    if model is None or not text.startswith('C2,'):return
    freqs=tone_frequencies(text)
    if freqs is None or len(freqs)!=len(model['tone_hz']) or not np.allclose(freqs,model['tone_hz']):
        raise RuntimeError(f"bin_model tones {model['tone_hz']} do not match firmware {freqs}")


def run_serial(cfg,port,duration,out,drive=False,baud=115200):
    tracker=Tracker(cfg);clock=DeviceClock(cfg['transport_delay']);motors=MotorProcess(cfg,drive)
    stop=False
    def request_stop(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    ser=None;count=0;invalid=0
    try:
        # Boot settling and the link handshake happen while the motors remain stopped.
        ser,early=open_link(port,f"P0\nD{cfg['dither_deg']}",baud)
        ser.timeout=0;ser.write_timeout=.05
        start=time.monotonic();tracker.scan_start=start;next_tick=start;buffer=b'';last_horn=None
        # Optional host-side bin weighting (cfg bin_weights): B2 precedes its R2 with the same seq.
        weights=cfg.get('bin_weights');model=cfg.get('bin_model');last_bins=None
        with (out/'raw_serial.jsonl').open('w',encoding='utf-8') as raw, \
             (out/'commands.csv').open('w',newline='') as commands, \
             (out/'measurement_decisions.csv').open('w',newline='') as decisions:
            # Handshake lines precede the control clock: log them, check the beacon profile, drop their pairs.
            for received,text in early:
                raw.write(json.dumps({'received_monotonic':received,'line':text,'handshake':True})+'\n')
                check_firmware_tones(text,model)
            writer=csv.writer(commands);writer.writerow(['elapsed_s','monotonic_s','mode','reason','speed_mps','steering_rad','accepted','rejected',
                'estimated_range_m','estimated_bearing_rad','range_std_m','bearing_std_rad','measurement_age_s','estimated_speed_mps','estimated_steering_rad','horn_center_deg'])
            decision_writer=csv.writer(decisions);decision_writer.writerow(['seq','received_monotonic','capture_monotonic','age_s','accepted','reason'])
            while not stop and time.monotonic()-start<duration:
                now=time.monotonic()
                if now<next_tick:
                    time.sleep(min(.005,next_tick-now));continue
                if now-next_tick > 2*cfg['dt']:
                    motors.send(0.,0.)
                    raise RuntimeError('control deadline missed by more than two periods')
                next_tick += cfg['dt'];tracker.advance(now)
                if ser.in_waiting>4096:
                    raise RuntimeError('serial backlog exceeds bounded buffer')
                buffer+=ser.read(ser.in_waiting)
                if len(buffer)>8192:raise RuntimeError('serial line overflow')
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1)
                    text=line.decode('ascii',errors='replace').strip()
                    raw.write(json.dumps({'received_monotonic':now,'line':text})+'\n')
                    check_firmware_tones(text,model)
                    if text.startswith('B2,'):
                        last_bins=parse_bins(text)
                        if last_bins is None:invalid+=1
                        continue
                    if not text.startswith('R2,'):continue
                    fields=parse_v2(text)
                    if fields is None:invalid+=1;continue
                    if (weights is not None or model is not None) and last_bins is not None and last_bins[0]==fields[0]:
                        try:
                            fields=apply_bin_model(fields,last_bins,model,cfg,weights) if model is not None                                 else apply_bins(fields,last_bins,weights)
                        except ValueError as exc:raise RuntimeError(f'bin_weights/bin_model: {exc}') from exc
                    elif model is not None:
                        invalid+=1;continue   # no matching B2: the firmware ratio is not the calibrated estimator
                    try:pair=clock.convert(fields,now)
                    except ValueError as exc:raise RuntimeError(str(exc)) from exc
                    ok,why=tracker.ingest(pair,now);count+=1
                    decision_writer.writerow([pair.seq,now,pair.time,now-pair.time,ok,why])
                speed,steering=tracker.command(now);motors.send(speed,steering)
                horn=round(tracker.horn_command(now),1)
                if horn!=last_horn:
                    ser.write(f'P{horn}\n'.encode());last_horn=horn
                if tracker.observer.initialized:
                    y,R,_=polar_output(tracker.observer.state,tracker.observer.covariance)
                    estimate_fields=[*y,*np.sqrt(np.maximum(0,np.diag(R)))]
                else:estimate_fields=['','','','']
                age=now-tracker.last_accepted if np.isfinite(tracker.last_accepted) else ''
                writer.writerow([now-start,now,tracker.mode,tracker.reason,speed,steering,tracker.accepted,tracker.rejected,
                                 *estimate_fields,age,tracker.speed_est,tracker.steer_est,horn])
                commands.flush();raw.flush();decisions.flush()
    finally:
        try:
            motors.close()
        finally:
            if ser is not None:ser.close()
            write_json(out/'summary.json',{'packets':count,'bad_crc_or_format':invalid,'accepted':tracker.accepted,
                                        'rejected':tracker.rejected,'hardware_driven':drive})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config');p.add_argument('--port');p.add_argument('--replay')
    p.add_argument('--duration',type=float,default=60);p.add_argument('--drive',action='store_true')
    p.add_argument('--mission',choices=['follow','approach'],help='approach is for an operator-declared stationary beacon')
    p.add_argument('--desired-range',type=float,help='separation / stand-off override in m')
    a=p.parse_args();cfg=load_config(a.config)
    if a.mission:cfg['mission']=a.mission
    if a.desired_range is not None:
        lo=cfg['clearance']+cfg['stop_margin']+3*cfg.get('range_noise_floor_rel',0.)*a.desired_range
        if not (lo < a.desired_range < cfg['range_domain'][1]):
            p.error(f"--desired-range must exceed clearance + stop margin + 3 sigma_rho ({lo:.2f} m) and stay inside the calibrated range")
        cfg['desired_range']=a.desired_range
    if bool(a.port)==bool(a.replay):p.error('choose exactly one of --port or --replay')
    if a.replay and a.drive:p.error('replay never drives hardware')
    if a.duration<=0:p.error('duration must be positive')
    out=create_run('live' if a.port else 'replay',cfg)
    if a.replay:print(replay(a.replay,cfg,out))
    else:run_serial(cfg,a.port,a.duration,out,a.drive)
    print(out)


if __name__=='__main__':main()
