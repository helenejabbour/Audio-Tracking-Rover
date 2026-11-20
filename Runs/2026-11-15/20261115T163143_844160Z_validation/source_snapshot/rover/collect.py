"""Stationary V2 calibration capture; records ground truth and never imports motors."""
import argparse
import csv
import json
import time
from .config import create_run,load_config,write_json
from .sensing import parse_v2,DeviceClock


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--label',required=True)
    p.add_argument('--rho',type=float,required=True);p.add_argument('--beta',type=float,required=True)
    p.add_argument('--phi',type=float,default=0);p.add_argument('--delta',type=float,default=20)
    p.add_argument('--settle-ms',type=int,default=40);p.add_argument('--duration',type=float,default=30)
    p.add_argument('--notes',required=True);p.add_argument('--config')
    a=p.parse_args()
    if not (a.rho>0 and a.duration>0 and 2<=a.delta<=40 and abs(a.phi)+a.delta<=65 and 20<=a.settle_ms<=1000):
        p.error('invalid geometry, duration or servo settings')
    import serial
    cfg=load_config(a.config);out=create_run('capture',cfg)
    write_json(out/'session.json',vars(a))
    clock=DeviceClock(cfg['transport_delay']);n=good=bad=0;firmware=[]
    cols=['t_pi','seq','t_ms','dt_pair_ms','phi_c','dlt','A_m','A_p','Aref_m','Aref_p','pp_m','pp_p',
          'flags','valid','clip','late','rho_true','beta_true','label','capture_monotonic_s']
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
                if line.startswith('C2,'):firmware.append(line)
                if not line.startswith('R2,'):continue
                fields=parse_v2(line)
                if fields is None:bad+=1;continue
                pair=clock.convert(fields,now)
                seq,tm,tp,te,pm,pp,am,ap,nm,np_,ppm,ppp,flags=fields
                # Ignore the in-flight pair captured before command settings took effect.
                if abs((pm+pp)/2-a.phi)>.6 or abs((pp-pm)/2-a.delta)>.6:continue
                writer.writerow([time.time(),seq,clock.unwrapped*1000,pair.gap*1000,(pm+pp)/2,(pp-pm)/2,
                                 am,ap,nm,np_,ppm,ppp,flags,int(flags==1),int(bool(flags&2)),int(bool(flags&4)),
                                 a.rho,a.beta,a.label,pair.time])
                f.flush();raw.flush();n+=1;good+=flags==1
    except KeyboardInterrupt:pass
    finally:
        write_json(out/'summary.json',{'n':n,'valid':good,'bad_crc_or_format':bad,'firmware':firmware,'hardware_motion':False})
        print(out)


if __name__=='__main__':main()
