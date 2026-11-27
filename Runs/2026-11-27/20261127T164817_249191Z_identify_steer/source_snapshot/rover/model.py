"""Kinematics and independent small-signal design; no hardware imports."""
import numpy as np
from scipy.linalg import expm, solve_discrete_are
from scipy.signal import place_poles


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def relative_dynamics(x, vf, wf, vt, wt):
    rho, beta, eps = x
    return np.array([vt*np.cos(eps-beta)-vf*np.cos(beta),
                     (vt*np.sin(eps-beta)+vf*np.sin(beta))/rho-wf, wt-wf])


def linear_model(speed, distance):
    A = np.zeros((3, 3)); A[1, 2] = speed / distance
    B = np.array([[-1., 0.], [0., -1.], [0., -1.]])
    E = np.array([[1., 0.], [0., 0.], [0., 1.]])
    C = np.array([[1., 0., 0.], [0., 1., 0.]])
    return A, B, E, C


def zoh(A, B, dt):
    n, m = B.shape
    M = np.zeros((n+m, n+m)); M[:n, :n] = A; M[:n, n:] = B
    M = expm(M * dt)
    return M[:n, :n], M[:n, n:]


def controllability(A, B):
    return np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(len(A))])


def observability(A, C):
    return np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(len(A))])


def design(cfg, method=None):
    A, B, E, C = linear_model(cfg['nominal_speed'], cfg['desired_range'])
    Aa = np.zeros((4, 4)); Aa[:3, :3] = A; Aa[3, 0] = 1
    Ba = np.vstack((B, np.zeros((1, 2))))
    Ad, Bd = zoh(Aa, Ba, cfg['dt'])
    method = method or cfg['controller']
    if method == 'place':
        K = np.zeros((2, 4))
        for row, idx, poles in [(0, [0, 3], cfg['range_poles']),
                                (1, [1, 2], cfg['bearing_poles'])]:
            K[row, idx] = place_poles(Ad[np.ix_(idx, idx)], Bd[idx, row:row+1],
                                      np.exp(np.array(poles)*cfg['dt'])).gain_matrix[0]
    elif method == 'lqi':
        Q, R = np.diag(cfg['lqr_q']), np.diag(cfg['lqr_r'])
        P = solve_discrete_are(Ad, Bd, Q, R)
        K = np.linalg.solve(R+Bd.T@P@Bd, Bd.T@P@Ad)
    elif method == 'bearing_only':
        K = np.zeros((2, 4)); K[1, 1] = -cfg['baseline_bearing_gain']
    else:
        raise ValueError(f'unknown controller: {method}')
    return {'A': A, 'B': B, 'E': E, 'C': C, 'Aa': Aa, 'Ba': Ba,
            'Ad': Ad, 'Bd': Bd, 'K': K}


def cartesian_transition(x, P, dt, speed, yaw, noise):
    """Exact constant-input CV transition in the rotating follower frame."""
    angle = yaw * dt
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, s], [-s, c]])
    disp = np.array([speed*dt, 0.]) if abs(yaw) < 1e-8 else \
        speed/yaw*np.array([s, 1-c])
    F = np.block([[R, dt*R], [np.zeros((2, 2)), R]])
    new_x = F @ x; new_x[:2] -= R @ disp
    I = np.eye(2)
    Q = noise * np.block([[dt**3/3*I, dt**2/2*I], [dt**2/2*I, dt*I]])
    new_P = F @ P @ F.T + Q
    return new_x, (new_P + new_P.T)/2


def polar_output(x, P):
    px, py = x[:2]; r2 = max(px*px + py*py, 1e-8); rho = np.sqrt(r2)
    H = np.array([[px/rho, py/rho, 0, 0], [-py/r2, px/r2, 0, 0]])
    return np.array([rho, np.arctan2(py, px)]), H @ P @ H.T, H
