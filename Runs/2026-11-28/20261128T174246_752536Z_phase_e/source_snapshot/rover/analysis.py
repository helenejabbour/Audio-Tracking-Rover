"""Auditable course-objective checks, including sampled observer/controller stability."""
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm, solve_discrete_are, solve_discrete_lyapunov, block_diag
from .model import linear_model, relative_dynamics, controllability, observability, design, zoh


def structure(A, B, C):
    return {'n': len(A), 'controllability_rank': int(np.linalg.matrix_rank(controllability(A, B))),
            'observability_rank': int(np.linalg.matrix_rank(observability(A, C)))}


def run_analysis(cfg):
    d = design(cfg); A, B, E, C = (d[k] for k in ['A', 'B', 'E', 'C'])
    n = len(A); x0 = np.array([cfg['desired_range'], 0., 0.]); h = 1e-6
    v = cfg['nominal_speed']
    Aj = np.column_stack([(relative_dynamics(x0+np.eye(n)[i]*h,v,0,v,0)-
                           relative_dynamics(x0-np.eye(n)[i]*h,v,0,v,0))/(2*h) for i in range(n)])
    def f_u(u): return relative_dynamics(x0,u[0],u[1],v,0)
    u0 = np.array([v,0.])
    Bj = np.column_stack([(f_u(u0+np.eye(2)[i]*h)-f_u(u0-np.eye(2)[i]*h))/(2*h) for i in range(2)])
    A5 = np.block([[A, E], [np.zeros((2,5))]])
    B5 = np.vstack([B, np.zeros((2,2))]); C5 = np.hstack([C, np.zeros((2,2))])
    cl = d['Ad']-d['Bd']@d['K']
    P = solve_discrete_lyapunov(cl.T, np.eye(4))
    Ad3, Bd3 = zoh(A, B, cfg['dt'])
    R = np.diag([cfg['range_noise_floor']**2,np.deg2rad(cfg['bearing_noise_floor_deg'])**2])
    Pk = solve_discrete_are(Ad3.T, C.T, np.eye(3)*1e-4, R)
    L = Pk@C.T@np.linalg.inv(C@Pk@C.T+R)
    F = (np.eye(3)-L@C)@Ad3
    # Homogeneous error coordinates [x; z; x-xhat]. Integral uses estimated range.
    Kx, Ki = d['K'][:,:3], d['K'][:,3:]
    exact_augmented = np.block([[d['Ad']-d['Bd']@d['K'],
                                 d['Bd']@Kx-np.vstack([np.zeros((3,3)), cfg['dt']*C[0]])],
                                [np.zeros((3,4)), F]])
    # Block upper triangular => eigenvalue union; this is an ideal synchronous reference.
    eig_union = np.r_[np.linalg.eigvals(cl),np.linalg.eigvals(F)]
    eig_error = np.max(abs(np.sort_complex(np.linalg.eigvals(exact_augmented))-np.sort_complex(eig_union)))
    A0=np.zeros((3,3))
    # At a=0: controllable subspace spans e1 and [0,1,1]; unobservable span e3.
    # Nilpotent nonzero A has a Jordan block at zero: Lyapunov UNSTABLE, not marginally stable.
    checks = {'jacobian_A_max_error': float(np.max(abs(A-Aj))),
              'jacobian_B_max_error': float(np.max(abs(B-Bj))),
              'nominal': structure(A,B,C), 'stopped': structure(A0,B,C),
              'disturbance_augmented': structure(A5,B5,C5),
              'open_loop_lyapunov_stable': False, 'open_loop_bibo_stable': False,
              'closed_loop_poles': [[float(z.real),float(z.imag)] for z in np.linalg.eigvals(cl)],
              'closed_loop_spectral_radius': float(max(abs(np.linalg.eigvals(cl)))),
              'lyapunov_min_eigenvalue': float(np.linalg.eigvalsh(P).min()),
              'lyapunov_residual': float(np.max(abs(cl.T@P@cl-P+np.eye(4)))),
              'synchronous_separation_error': float(eig_error),
              'K': d['K'].tolist(), 'Ad': d['Ad'].tolist(), 'Bd': d['Bd'].tolist()}
    # LTV A(t)=a(t)N, N^2=0: Phi(t,s)=I+N integral_s^t a(tau)dtau.
    N = np.zeros((3,3)); N[1,2]=1
    a0=v/cfg['desired_range']; T=3.
    phi = np.eye(3)+N*a0*(T+.2*(1-np.cos(T)))
    num = solve_ivp(lambda t,y: ((a0*(1+.2*np.sin(t))*N)@y.reshape(3,3)).ravel(),
                    [0,T], np.eye(3).ravel(), rtol=1e-10,atol=1e-12).y[:,-1].reshape(3,3)
    checks['ltv_transition_error'] = float(np.max(abs(phi-num)))
    # Nonzero input and initial state: exp(A t)=I+A t; zero-order hold convolution.
    init=np.array([.1,.05,-.02]); forcing=np.array([.02,.01]); Ad,Bd=zoh(A,B,T)
    predicted=Ad@init+Bd@forcing
    numeric=solve_ivp(lambda t,x:A@x+B@forcing,[0,T],init,rtol=1e-10,atol=1e-12).y[:,-1]
    checks['forced_lti_trajectory_error']=float(np.max(abs(predicted-numeric)))
    # Similarity transformation preserves input-output response.
    S=np.array([[1,.2,0],[0,1,.3],[.1,0,1]])
    At=np.linalg.solve(S,A@S); Bt=np.linalg.solve(S,B); Ct=C@S
    checks['similarity_transfer_error']=float(np.max(abs(C@np.linalg.solve(1j*np.eye(3)-A,B)-Ct@np.linalg.solve(1j*np.eye(3)-At,Bt))))
    checks['linearization_region']=[]
    for scale in [.25,.5,1.,2.]:
        dx=scale*np.array([.15,.1,.1]); tgrid=np.linspace(0,3,101)
        nl=solve_ivp(lambda t,x:relative_dynamics(x,v,0,v,0),[0,3],x0+dx,t_eval=tgrid,rtol=1e-10,atol=1e-12).y.T-x0
        lin=np.array([expm(A*t)@dx for t in tgrid])
        checks['linearization_region'].append({'scale':scale,'max_abs_error':np.max(abs(nl-lin),axis=0).tolist()})
    checks['sampled_loop_sweep'] = [sampled_loop(cfg,period,delay,gain) for period,delay,gain in
                                   [(p,d,g) for p in [.2,.4,.8,1.2] for d in [0.,.1,.3] for g in [.5,1.,2.]]]
    return checks


def sampled_loop(cfg,period,delay,gain=1.):
    """Monodromy of the LOCAL periodic loop with actuator lag and delayed observer.

    This is a separate linear reference, not a stability theorem for the EKF,
    saturation or supervisor. Measurement period and delay are grid-aligned.
    It uses a three-state steady-state KF and exact replay for delay < period.
    """
    d=design(cfg);A,B,C=d['A'],d['B'],d['C'];dt=cfg['dt']
    m=int(round(period/dt));lag=int(round(delay/dt))
    if lag>=m:return {'period_s':period,'delay_s':delay,'gain_scale':gain,'spectral_radius':None,'valid':False}
    Aplant=np.block([[A,B],[np.zeros((2,3)),-np.diag([1/cfg['tau_speed'],1/cfg['tau_steer']])]])
    Bplant=np.vstack([np.zeros((3,2)),np.diag([1/cfg['tau_speed'],1/cfg['tau_steer']])])
    Fp,Gp=zoh(Aplant,Bplant,dt);Fa=expm(A*dt);Fm=expm(A*period)
    R=np.diag([cfg['range_noise_floor']**2,np.deg2rad(cfg['bearing_noise_floor_deg'])**2])
    P=solve_discrete_are(Fm.T,C.T,np.eye(3)*.002*period,R)
    L=P@C.T@np.linalg.inv(C@P@C.T+R);replay=expm(A*delay)
    K=d['K']*gain
    # state = plant(5), integral(1), current estimate(3), delayed [plant3,estimate3] blocks.
    dim=9+6*lag
    def step(z,correct):
        plant=z[:5].copy();integral=z[5];hat=z[6:9].copy()
        if correct:
            if lag:
                past=z[9+6*(lag-1):9+6*lag];res=C@(past[:3]-past[3:])
            else:res=C@(plant[:3]-hat)
            hat += replay@L@res
        cmd=-K@np.r_[hat,integral]
        nxtplant=Fp@plant+Gp@cmd
        # Same known actuator forcing, so the state-prediction error evolves with exp(A dt).
        nxthat=Fa@hat+(nxtplant[:3]-Fa@plant[:3])
        nxt=np.r_[nxtplant,integral+dt*hat[0],nxthat,np.zeros(6*lag)]
        if lag:
            nxt[9:15]=np.r_[plant[:3],hat]
            if lag>1:nxt[15:]=z[9:-6]
        return nxt
    M0=np.column_stack([step(v,False) for v in np.eye(dim)])
    M1=np.column_stack([step(v,True) for v in np.eye(dim)])
    period_map=np.linalg.matrix_power(M0,m-1)@M1
    radius=float(max(abs(np.linalg.eigvals(period_map))))
    return {'period_s':period,'delay_s':delay,'gain_scale':gain,'spectral_radius':radius,'stable':radius<1.,'valid':True}
