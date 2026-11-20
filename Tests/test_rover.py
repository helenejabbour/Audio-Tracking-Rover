import csv
from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
from rover.config import load_config
from rover.model import linear_model,relative_dynamics,design,controllability,observability,cartesian_transition
from rover.sensing import Pair,Measurement,estimate,amplitude,encode_v2,parse_v2,DeviceClock
from rover.observer import Observer
from rover.controller import Tracker
from rover.hardware import validate_hardware,map_actuators
from rover.simulation import run_simulation
from rover.calibrate import fit_model,evaluate


def perfect_pair(cfg,seq=1,t=0.,rho=1.5,beta=.1,phi=0.,gap=.18):
    delta=np.deg2rad(cfg['dither_deg'])
    return Pair(seq,t,gap,phi-delta,phi+delta,amplitude(rho,beta,phi-delta,cfg),
                amplitude(rho,beta,phi+delta,cfg),.1,.1,1)


class ModelTests(unittest.TestCase):
    def setUp(self):self.cfg=load_config()

    def test_numeric_linearization(self):
        c=self.cfg;A,B,_,_=linear_model(c['nominal_speed'],c['desired_range']);x=np.array([c['desired_range'],0.,0.]);v=c['nominal_speed'];h=1e-6
        numeric=np.column_stack([(relative_dynamics(x+np.eye(3)[i]*h,v,0,v,0)-relative_dynamics(x-np.eye(3)[i]*h,v,0,v,0))/(2*h) for i in range(3)])
        np.testing.assert_allclose(A,numeric,atol=1e-8)

    def test_structure_and_stationary_degeneracy(self):
        A,B,_,C=linear_model(.18,1.2)
        self.assertEqual(np.linalg.matrix_rank(controllability(A,B)),3)
        self.assertEqual(np.linalg.matrix_rank(observability(A,C)),3)
        self.assertEqual(np.linalg.matrix_rank(observability(A*0,C)),2)

    def test_both_designs_are_stable_and_steer_left(self):
        for method in ['lqi','place']:
            d=design(self.cfg,method)
            self.assertLess(max(abs(np.linalg.eigvals(d['Ad']-d['Bd']@d['K']))),1)
            self.assertGreater((-d['K']@np.array([0,.1,0,0]))[1],0)

    def test_cartesian_exact_motion(self):
        x=np.array([2.,0.,.1,0.]);P=np.eye(4)
        y,_=cartesian_transition(x,P,1.,.2,0.,0.)
        np.testing.assert_allclose(y,[1.9,0,.1,0])
        y,_=cartesian_transition(x,P,1.,0.,np.pi/2,0.)
        np.testing.assert_allclose(y,[0,-2.1,0,-.1],atol=1e-12)


class SensingTests(unittest.TestCase):
    def setUp(self):self.cfg=load_config()

    def test_closed_form_roundtrip_nonunit_exponent_and_boresight(self):
        self.cfg.update(range_exponent=1.3,boresight_deg=4.)
        for beta in [-.3,0,.3]:
            m,why=estimate(perfect_pair(self.cfg,beta=beta),self.cfg)
            self.assertEqual(why,'accepted');np.testing.assert_allclose(m.y,[1.5,beta],atol=1e-10)
            self.assertGreater(np.linalg.eigvalsh(m.covariance).min(),0)

    def test_bad_quality_and_nonfinite_rejected(self):
        p=perfect_pair(self.cfg)
        for bad in [replace(p,flags=5),replace(p,flags=3),replace(p,flags=0),replace(p,amplitude_plus=np.nan),replace(p,gap=6.42)]:
            self.assertIsNone(estimate(bad,self.cfg)[0])

    def test_crc_corruption_and_nan(self):
        line=encode_v2([1,100000,280000,300000,-20,20,100,100,1,1,200,200,1])
        self.assertIsNotNone(parse_v2(line))
        self.assertIsNone(parse_v2(line.replace('100.','101.')) if '100.' in line else parse_v2(line[:-1]+'Z'))
        self.assertIsNone(parse_v2(encode_v2([1,1,2,3,-20,20,'nan',100,1,1,200,200,1])))

    def test_clock_wrap_duplicate_and_extra_buffer_delay(self):
        clock=DeviceClock(.04)
        a=[0xfffffffe,0xffffff00-180000,0xffffff00,0xffffff80,-20,20,100,100,1,1,200,200,1]
        p=clock.convert(a,10.)
        b=[0xffffffff,100000,280000,300000,-20,20,100,100,1,1,200,200,1]
        q=clock.convert(b,10.6)
        self.assertGreater(q.time,p.time)
        self.assertGreater(10.6-q.time,.3)
        with self.assertRaises(ValueError):clock.convert(b,11.)


class ObserverTests(unittest.TestCase):
    def setUp(self):self.cfg=load_config()

    def test_delayed_update_matches_in_order_replay(self):
        initial=Measurement(0,0.,np.array([2.,.1]),np.diag([.05**2,.03**2]))
        obs1=Observer(self.cfg);obs2=Observer(self.cfg)
        obs1.initialize(initial);obs2.initialize(initial)
        m=Measurement(1,.225,np.array([1.97,.08]),np.diag([.05**2,.03**2]))
        for t in [.05,.1,.15,.2]:obs1.predict(t,.2,.04);obs2.predict(t,.2,.04)
        obs1.predict(.225,.2,.04);self.assertTrue(obs1.correct(m)[0])
        for t in [.25,.3,.35,.4]:obs1.predict(t,.2,.04);obs2.predict(t,.2,.04)
        self.assertTrue(obs2.correct(m)[0])
        np.testing.assert_allclose(obs1.state,obs2.state,atol=1e-10)
        np.testing.assert_allclose(obs1.covariance,obs2.covariance,atol=1e-10)

    def test_gate_does_not_refresh_measurement_time(self):
        o=Observer(self.cfg);o.initialize(Measurement(0,0.,np.array([1.5,0.]),np.eye(2)*.001));o.predict(.4,0,0)
        ok,why=o.correct(Measurement(1,.3,np.array([3.,1.]),np.eye(2)*.001))
        self.assertFalse(ok);self.assertEqual(why,'innovation_gate');self.assertEqual(o.last_measurement,0.)
        self.assertGreater(np.linalg.eigvalsh(o.covariance).min(),0)

    def test_reordered_update_rejected(self):
        o=Observer(self.cfg);m=Measurement(0,0.,np.array([1.5,0.]),np.eye(2)*.01);o.initialize(m);o.predict(.5,0,0)
        self.assertFalse(o.correct(m)[0])


class SafetyTests(unittest.TestCase):
    def setUp(self):self.cfg=load_config()

    def test_default_cannot_drive(self):
        with self.assertRaises(ValueError):validate_hardware(self.cfg)

    def test_no_signal_no_motion(self):
        tr=Tracker(self.cfg)
        for t in np.arange(0,3,.05):self.assertEqual(tr.command(float(t)),(0.,0.))

    def test_duplicates_do_not_acquire(self):
        tr=Tracker(self.cfg);p=perfect_pair(self.cfg,t=0)
        tr.ingest(p,.1)
        for _ in range(10):self.assertFalse(tr.ingest(p,.2)[0])
        self.assertEqual(tr.command(.2),(0.,0.))

    def test_watchdog_stops_after_previously_valid_data(self):
        tr=Tracker(self.cfg)
        for i,t in enumerate([.1,.5,.9]):tr.ingest(perfect_pair(self.cfg,seq=i,t=t-.1),t);tr.command(t)
        self.assertGreater(tr.command(1.)[0],0.)
        self.assertEqual(tr.command(2.5),(0.,0.));self.assertEqual(tr.mode,'HOLD')

    def test_stale_valid_packet_does_not_acquire(self):
        tr=Tracker(self.cfg)
        ok,why=tr.ingest(perfect_pair(self.cfg,t=0),1.)
        self.assertFalse(ok);self.assertEqual(why,'stale_measurement');self.assertEqual(tr.command(1.),(0.,0.))

    def test_rejected_packet_neither_confirms_nor_erases_consistent_observations(self):
        tr=Tracker(self.cfg)
        tr.ingest(perfect_pair(self.cfg,seq=0,t=0),.1)
        tr.ingest(replace(perfect_pair(self.cfg,seq=1,t=.4),flags=5),.5)
        self.assertEqual(tr.command(.5),(0.,0.));self.assertEqual(tr.consecutive,1)
        tr.ingest(perfect_pair(self.cfg,seq=2,t=.8),.9)
        self.assertEqual(tr.command(.9),(0.,0.))
        tr.ingest(perfect_pair(self.cfg,seq=3,t=1.2),1.3)
        self.assertGreater(tr.command(1.3)[0],0.)

    def test_motor_worker_stops_on_missing_heartbeat(self):
        import queue,sys,threading,time,types
        from unittest.mock import patch
        from rover.hardware import motor_worker
        calls=[]
        class Back:
            def __init__(self,**_):self.speed=0
            def forward(self):calls.append('forward')
            def stop(self):calls.append('stop')
        class Front:
            def __init__(self,**_):pass
            def turn(self,angle):pass
        fake=types.SimpleNamespace(setup=lambda:None,front_wheels=types.SimpleNamespace(Front_Wheels=Front),back_wheels=types.SimpleNamespace(Back_Wheels=Back))
        cfg=dict(self.cfg,hardware={'speed_table':[[.06,20],[.2,40],[.35,60]],'steering_table':[[-25,65],[0,90],[25,115]]})
        commands,status=queue.Queue(),queue.Queue()
        with patch.dict(sys.modules,{'picar':fake}):
            thread=threading.Thread(target=motor_worker,args=(commands,status,cfg));thread.start()
            self.assertEqual(status.get(timeout=1)[0],'ready')
            commands.put((time.monotonic(),.2,0));time.sleep(.03)
            self.assertIn('forward',calls)
            time.sleep(.15);self.assertEqual(calls[-1],'stop')
            commands.put(None);thread.join(timeout=1);self.assertFalse(thread.is_alive())

    def test_uncertain_clearance_stops_without_permanent_latch(self):
        tr=Tracker(self.cfg)
        tr.observer.initialize(Measurement(0,0.,np.array([1.,0.]),np.diag([.2**2,.02**2])))
        tr.last_accepted=0.;tr.consecutive=3
        self.assertEqual(tr.command(0.),(0.,0.))
        self.assertEqual(tr.reason,'clearance_uncertain');self.assertFalse(tr.stop_latched)

    def test_explicit_stationary_mission_uses_range_approach_and_latches(self):
        cfg=dict(self.cfg,mission='approach')
        tr=Tracker(cfg)
        for i,t in enumerate([.1,.5,.9]):tr.ingest(perfect_pair(cfg,seq=i,t=t-.1,rho=cfg['desired_range']+.05,beta=0),t)
        self.assertEqual(tr.command(.9),(0.,0.))
        self.assertTrue(tr.stop_latched);self.assertEqual(tr.reason,'stationary_target_standoff')

    def test_steering_and_speed_constraints(self):
        c=dict(self.cfg,duration=12.)
        _,rows=run_simulation(c,'nominal',2)
        for r in rows:
            self.assertLessEqual(r[7],c['max_speed']);self.assertGreaterEqual(r[7],0)
            self.assertLessEqual(abs(r[8]),np.deg2rad(c['max_steer_deg'])+1e-10)
            if r[7]==0:self.assertEqual(r[8],0.)

    def test_existing_logs_produce_no_motion(self):
        from rover.live import replay
        with tempfile.TemporaryDirectory() as tmp:
            summary=replay('Data/E0_static_20261022_134646.csv',self.cfg,Path(tmp))
        self.assertEqual(summary['accepted'],0);self.assertEqual(summary['nonzero_commands'],0)


class CalibrationTests(unittest.TestCase):
    def test_identifiable_synthetic_fit_and_holdout(self):
        cfg=load_config();true=dict(cfg,sigma_deg=48,amplitude_constant=125,range_exponent=1.15,boresight_deg=3.)
        rows=[]
        for rho in [.8,1.2,2.]:
            for beta in [-45,-30,-15,0,15,30,45]:
                for i in range(3):
                    pair=perfect_pair(true,rho=rho,beta=np.deg2rad(beta))
                    rows.append({'flags':'1','phi_c':'0','dlt':'20','rho_true':str(rho),'beta_true':str(beta),
                                 'A_m':str(pair.amplitude_minus),'A_p':str(pair.amplitude_plus),'Aref_m':'.1','Aref_p':'.1',
                                 'seq':str(i),'dt_pair_ms':'180','t_ms':str(i*400),'_file':f'{rho}_{beta}'})
        fitted,info=fit_model(rows,cfg)
        for key in ['sigma_deg','amplitude_constant','range_exponent','boresight_deg']:
            self.assertAlmostEqual(fitted[key],true[key],places=3)
        self.assertTrue(evaluate([r for r in rows if abs(float(r['beta_true']))<=30],fitted)['passed'])
        self.assertFalse(fitted['hardware_ready'])

    def test_inadequate_data_cannot_fit(self):
        with self.assertRaises(ValueError):fit_model([],load_config())


class TrialEvaluationTests(unittest.TestCase):
    def test_no_initial_step_has_no_overshoot_claim(self):
        from rover.simulation import summarize
        cfg=load_config()
        rows=[[t,cfg['desired_range'],0.,cfg['desired_range'],0.,0.,0.,0.,0.,0.,0.,cfg['desired_range'],0.,'TRACK','tracking',0.] for t in np.arange(0,15,.1)]
        m=summarize(rows,cfg)
        self.assertIsNone(m['overshoot_pct']);self.assertFalse(m['original_O3_pass'])

    def test_independent_reference_alignment(self):
        from rover.evaluate_trial import evaluate
        cfg=load_config();commands=[];truth=[]
        for t in np.arange(0,15.01,.1):
            truth.append({'time_s':t,'follower_x_m':.18*t,'follower_y_m':0,'follower_heading_rad':0,
                          'target_x_m':.18*t+cfg['desired_range'],'target_y_m':0})
            commands.append({'elapsed_s':t+.3,'estimated_range_m':cfg['desired_range'],'estimated_bearing_rad':0,
                             'speed_mps':.18,'steering_rad':0,'mode':'TRACK','reason':'tracking'})
        summary,_=evaluate(commands,truth,cfg,.3)
        self.assertLess(summary['range_estimation_rmse_m'],1e-10)
        self.assertEqual(summary['bearing_estimation_rmse_deg'],0.)
        self.assertTrue(summary['initial_transient_covered'])


if __name__=='__main__':unittest.main()
