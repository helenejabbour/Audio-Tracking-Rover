"""Repeatable target motion profile. Commands are open loop; logs are not ground truth."""
import argparse
import csv
import time
from .config import load_config,create_run
from .hardware import MotorProcess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config');p.add_argument('--speed',type=float,default=.18)
    p.add_argument('--duration',type=float,default=15);p.add_argument('--steering-deg',type=float,default=0)
    p.add_argument('--drive',action='store_true');a=p.parse_args();cfg=load_config(a.config)
    import numpy as np
    if not (0<=a.speed<=cfg['max_speed'] and abs(a.steering_deg)<=cfg['max_steer_deg'] and a.duration>0):
        p.error('profile exceeds configured limits')
    out=create_run('target_profile',cfg);motor=MotorProcess(cfg,a.drive)
    try:
        with (out/'commands.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['elapsed_s','speed_mps','steering_deg'])
            start=time.monotonic()
            while time.monotonic()-start<a.duration:
                t=time.monotonic()-start;motor.send(a.speed,np.deg2rad(a.steering_deg));w.writerow([t,a.speed,a.steering_deg]);f.flush()
                time.sleep(cfg['dt'])
    except KeyboardInterrupt:pass
    finally:motor.close()
    print(out)


if __name__=='__main__':main()
