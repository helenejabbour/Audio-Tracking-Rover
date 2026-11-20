"""PiCar adapter isolated in a process that stops on a missing heartbeat."""
import multiprocessing as mp
from pathlib import Path
import queue
import time
import numpy as np


def validate_hardware(cfg):
    if cfg.get('hardware_ready') is not True:
        raise ValueError('hardware_ready is false: complete the calibration and commissioning guide')
    hw = cfg.get('hardware', {})
    for name in ['speed_table', 'steering_table']:
        table = np.asarray(hw.get(name, []), float)
        if table.ndim != 2 or table.shape[1] != 2 or len(table) < 3 or not np.isfinite(table).all():
            raise ValueError(f'hardware.{name} requires at least three finite measured pairs')
        if np.any(np.diff(table[:, 0]) <= 0):
            raise ValueError(f'hardware.{name} first column must strictly increase')
    speed = np.asarray(hw['speed_table'])
    if speed[0,0] > cfg['min_speed'] or speed[-1,0] < cfg['max_speed'] or \
            np.any((speed[:,1] < 0) | (speed[:,1] > 100)) or np.any(np.diff(speed[:,1]) <= 0):
        raise ValueError('speed table must cover calibrated speed range with monotonic PWM 0..100')
    steering=np.asarray(hw['steering_table'])
    if steering[0,0] > -cfg['max_steer_deg'] or steering[-1,0] < cfg['max_steer_deg'] or \
            np.any((steering[:,1]<0)|(steering[:,1]>180)):
        raise ValueError('steering table does not cover steering limits or uses invalid servo commands')
    change=np.diff(steering[:,1])
    if not (np.all(change>0) or np.all(change<0)):
        raise ValueError('steering servo calibration must be strictly monotonic in either direction')
    for flag in ['sensor_holdout_passed','actuator_calibration_passed','stop_test_passed','wheels_lifted_passed']:
        if hw.get(flag) is not True:
            raise ValueError(f'hardware.{flag} is not confirmed')
    evidence=hw.get('evidence_files', [])
    if not evidence or not all(Path(p).is_file() for p in evidence):
        raise ValueError('hardware.evidence_files must identify the completed measured calibration records')
    if cfg['measurement_period'] > .5 or cfg['transport_delay'] > .15:
        raise ValueError('timing is outside the initial commissioning envelope; redesign and revalidate first')
    if not cfg['range_domain'][0] < cfg['desired_range'] < cfg['range_domain'][1]:
        raise ValueError('operating point is outside calibrated range')


def map_actuators(cfg, speed, steering):
    if not np.isfinite([speed, steering]).all():
        raise ValueError('nonfinite actuator command')
    hw=cfg['hardware']; speeds=np.array(hw['speed_table']); angles=np.array(hw['steering_table'])
    speed=float(np.clip(speed,0,cfg['max_speed']))
    pwm=0 if speed < cfg['min_speed'] else int(round(np.interp(speed,speeds[:,0],speeds[:,1])))
    angle=float(np.clip(np.rad2deg(steering),-cfg['max_steer_deg'],cfg['max_steer_deg']))
    return pwm, int(round(np.interp(angle,angles[:,0],angles[:,1])))


def motor_worker(commands, status, cfg):
    back=None
    try:
        import picar
        from picar import front_wheels, back_wheels
        picar.setup()
        front=front_wheels.Front_Wheels(db=cfg['hardware'].get('picar_database','config'))
        back=back_wheels.Back_Wheels(db=cfg['hardware'].get('picar_database','config'))
        back.speed=0;back.stop()
        status.put(('ready',None))
        while True:
            try:
                item=commands.get(timeout=.1)
            except queue.Empty:
                back.speed=0;back.stop();continue
            if item is None: break
            stamp,speed,steer=item
            if time.monotonic()-stamp > .25:
                back.speed=0;back.stop();continue
            pwm,angle=map_actuators(cfg,speed,steer)
            front.turn(angle)
            if pwm:
                back.forward();back.speed=pwm
            else:
                back.speed=0;back.stop()
    except BaseException as exc:
        status.put(('error',repr(exc)))
    finally:
        if back is not None:
            back.speed=0;back.stop()


class MotorProcess:
    def __init__(self,cfg,drive=False):
        self.drive=drive;self.process=None
        if drive:
            validate_hardware(cfg)
            context=mp.get_context('spawn')
            self.commands=context.Queue(maxsize=2);self.status=context.Queue()
            self.process=context.Process(target=motor_worker,args=(self.commands,self.status,cfg),daemon=True)
            self.process.start()
            kind,detail=self.status.get(timeout=15)
            if kind!='ready': raise RuntimeError(detail)

    def send(self,speed,steering):
        if not self.drive:return
        if not self.process.is_alive():raise RuntimeError('motor process stopped')
        while True:
            try:self.commands.get_nowait()
            except queue.Empty:break
        try:self.commands.put_nowait((time.monotonic(),speed,steering))
        except queue.Full:raise RuntimeError('motor command queue stalled')

    def close(self):
        if not self.drive or self.process is None:return
        try:
            self.send(0.,0.)
            self.commands.put(None,timeout=.5)
        except (RuntimeError,queue.Full):pass
        self.process.join(timeout=2)
        # Do not forcibly kill an unresponsive driver; an independent power cut is required.
        if self.process.is_alive():
            raise RuntimeError('motor process is unresponsive: use the physical motor power cut')
