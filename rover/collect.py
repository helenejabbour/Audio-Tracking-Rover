"""Stationary V2 calibration capture; records ground truth and never imports motors."""
import argparse
import csv
import json
import time
from .config import create_run,load_config,write_json
from .sensing import parse_v2,parse_bins,tone_frequencies,DeviceClock,quality_ok,ORDER_BIT


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--label',required=True)
    p.add_argument('--rho',type=float,required=True);p.add_argument('--beta',type=float,required=True)
    p.add_argument('--phi',type=float,default=0);p.add_argument('--delta',type=float,default=50)
    p.add_argument('--settle-ms',type=int,default=40);p.add_argument('--duration',type=float,default=30)
    p.add_argument('--notes',required=True);p.add_argument('--config')
    a=p.parse_args();cfg=load_config(a.config)
    right,left=cfg['horn_limit_deg'],cfg.get('horn_limit_left_deg',cfg['horn_limit_deg'])
    # Firmware clamps to [-right, +left]; a clamped pair would be silently filtered below.
    if not (a.rho>0 and a.duration>0 and 1<=a.delta<=60 and a.phi-a.delta>=-right and a.phi+a.delta<=left
            and 20<=a.settle_ms<=1000):
        p.error(f'invalid geometry, duration or servo settings (horn span is [-{right}, +{left}] deg)')
    import serial
    out=create_run('capture',cfg)
    write_json(out/'session.json',vars(a))
    clock=DeviceClock(cfg['transport_delay']);n=good=bad=0;firmware=[]
    # Multi-tone firmware sends B2 (per-tone amplitudes) just before each R2 with the same seq.
    pending={};bins_file=bins_writer=None;n_bins=0;freqs=None
    cols=['t_pi','seq','t_ms','dt_pair_ms','phi_c','dlt','A_m','A_p','Aref_m','Aref_p','pp_m','pp_p',
          'flags','valid','clip','late','pfirst','rho_true','beta_true','label','capture_monotonic_s']
    try:
        with serial.Serial(a.port,115200,timeout=.2) as ser, (out/'samples.csv').open('w',newline='') as f, \
                (out/'raw_serial.jsonl').open('w',encoding='utf-8') as raw:
            time.sleep(2);ser.reset_input_buffer()
            ser.write(f'P{a.phi}\nD{a.delta}\nS{a.settle_ms}\n?\n'.encode())
            writer=csv.writer(f);writer.writerow(cols);start=time.monotonic()
            while time.monotonic()-start<a.duration:
                line=ser.readline().decode('ascii',errors='replace').strip();now=time.monotonic()
                if not line:continue
                raw.write(json.dumps({'received_monotonic':now,'line':line})+'\n')
                if line.startswith('C2,'):firmware.append(line);freqs=tone_frequencies(line) or freqs
                if line.startswith('B2,'):
                    bins=parse_bins(line)
                    if bins is None:bad+=1
                    else:pending={bins[0]:bins}   # only the latest pair is ever needed
                    continue
                if not line.startswith('R2,'):continue
                fields=parse_v2(line)
                if fields is None:bad+=1;continue
                pair=clock.convert(fields,now)
                seq,tm,tp,te,pm,pp,am,ap,nm,np_,ppm,ppp,flags=fields
                # Ignore the in-flight pair captured before command settings took effect.
                if abs((pm+pp)/2-a.phi)>.6 or abs((pp-pm)/2-a.delta)>.6:continue
                writer.writerow([time.time(),seq,clock.unwrapped*1000,pair.gap*1000,(pm+pp)/2,(pp-pm)/2,
                                 am,ap,nm,np_,ppm,ppp,flags,int(quality_ok(flags)),int(bool(flags&2)),int(bool(flags&4)),int(bool(flags&ORDER_BIT)),
                                 a.rho,a.beta,a.label,pair.time])
                bins=pending.pop(seq,None)
                if bins is not None:
                    _,bm,bp,bnm,bnp=bins;m=len(bm)
                    if bins_writer is None:
                        names=[f'{x:g}' for x in freqs] if freqs and len(freqs)==m else [str(i) for i in range(m)]
                        bins_file=(out/'bins.csv').open('w',newline='');bins_writer=csv.writer(bins_file)
                        bins_writer.writerow(['seq']+[f'am_{x}' for x in names]+[f'ap_{x}' for x in names]+['N_m','N_p'])
                    bins_writer.writerow([seq,*bm,*bp,bnm,bnp]);bins_file.flush();n_bins+=1
                f.flush();raw.flush();n+=1;good+=quality_ok(flags)
    except KeyboardInterrupt:pass
    finally:
        if bins_file is not None:bins_file.close()
        write_json(out/'summary.json',{'n':n,'valid':good,'bins_rows':n_bins,'tone_hz':freqs,'bad_crc_or_format':bad,
                                       'firmware':firmware,'hardware_motion':False})
        print(out)


if __name__=='__main__':main()
