% PHASE E VERSION (20 Nov 2026), promoted from Matlab/archive/acoustic_pursuit_design.m.
%   Every measured value and design knob is read from Config/phase_e.json (shared with the Python runtime, rover.phase_e).
%   Changes vs the archived draft: observer replay to the pair midpoint (Td_obs = Td + Tdw/2, as the Python runtime);
%   O5 margins with the information delay T_eff = Td_obs + Tm/2 + Ts/2 plus delay and disk margins; sensor with the
%   measured multipath model (fast fringe = white ln-amplitude noise per capture, room bias = AR(1) bearing bias,
%   rover.simulation.MultipathModel); S3 adds two multipath scenarios; S1 and the Simulink check are saved as figures;
%   Chapter 6 figures c1..c5 (LQR effect of R, root locus vs rho, LQR vs LQI, LQE tuning, LQG) via course_plots().
%   Run from the repo root:  addpath('Matlab'); R = acoustic_pursuit_design();   (figures + log in Runs/<date>/..._matlab_phase_e)
%
%   Commands for running
%   R = acoustic_pursuit_design();                                     % defaults: LQR + integral action, Kalman observer
%   R = acoustic_pursuit_design(struct('controller','place','observer','luenberger'));
%   R = acoustic_pursuit_design(struct('P',struct('Td',0.5,'Qd',[25 10 3 2])));   % override any value of user_settings()
%
% OPTIONS (fields of `opt`, all optional)
%   controller   'lqr' (default) | 'place' (pole placement) | 'lqr_noint' (LQR without integral action)
%                | 'bearing_only' (baseline of the report, weeks 7-8: v_f = v0, w_f = k_bearing*beta_hat)
%   observer     'kalman' (default) | 'luenberger' (pole placement) | 'ideal' (true state: a cheat, to isolate the controller)
%   dist_states  true (default): the observer also estimates the constant target disturbances [dv_t; w_t]
%   run_S1, run_mc, plot, export   true/false switches for the linearization check, Monte Carlo, figures, gains.json
%   P            struct overriding fields of user_settings()
%
% WHAT THE FILE DOES (follows the class method)
%   1. Identify the system: nonlinear -> LTI design model, MIMO/SISO, poles, transfer matrix, decoupling.
%   2. Controllability / observability: Kalman rank tests, state transformation to the Kalman canonical form
%      (co, c-no, nc-o, nc-no parts), stabilizability / detectability verdicts and how each part is treated.
%      Done for (i) the plant, (ii) the degenerate case v0 -> 0, (iii) the plant + target-disturbance states.
%   3. Design decisions: state feedback with integral action (controller), observer (estimator) incl. disturbance states.
%   4. Controller and observer synthesis (selectable types), discretization, delay compensation, margins (O5), peak sensitivity.
%   5. S1 (linear vs nonlinear), S2 (conditioning vs v0/d*), S3 (closed-loop Monte Carlo on the nonlinear plant + sensor model).
%   6. Export of matrices (struct R, optional gains.json for the Raspberry Pi).
%   7. Robustness layer (section F of user_settings): soft innovation gate (clipping near lock), watchdog and
%      hard stop (lost samples, minimum range), burst-timing stress (period jitter, dropped captures, outage window) and
%      reflection injection in the sensor model; S3 scenarios 3-4 compare the loop with and without the gate.
%   8. Course-objective checks: realizations and canonical forms, analytic LTI / LTV trajectories (2b), Lyapunov and BIBO (4d),
%      MIMO pole placement incl. partial controllability (4e).
% Hard-coded numbers you can change are collected in user_settings() below and flagged with  <== EXPERIMENT.
% Conventions: x = [d_rho; beta; eps] (range error, bearing, relative heading), u = [dv_f; w_f] (follower speed/yaw-rate
% deviations), w = [dv_t; w_t] (target disturbance), y = [d_rho_hat; beta_hat] (acoustic estimates).

function R = acoustic_pursuit_design(opt)
if nargin < 1, opt = struct(); end
P = user_settings();
if isfield(opt,'P'), fn = fieldnames(opt.P); for i = 1:numel(fn), P.(fn{i}) = opt.P.(fn{i}); end, end
P.out_dir = getopt(opt, 'out_dir', default_out_dir());
if ~exist(P.out_dir, 'dir'), mkdir(P.out_dir); end
diary(fullfile(P.out_dir, 'matlab_log.txt'));  cleanupDiary = onCleanup(@() diary('off'));
fprintf('Phase E MATLAB design, config %s, output %s\n', P.config, P.out_dir);
ctype = getopt(opt,'controller','lqr');   otype = getopt(opt,'observer','kalman');   dist = getopt(opt,'dist_states',true);
Ts = P.Ts;  Tm = P.Tm;  Td = P.Td;
yn = {'no','yes'};
if abs(Tm/Ts-round(Tm/Ts))>1e-9 || abs(Td/Ts-round(Td/Ts))>1e-9 || abs(P.Tdw/Ts-round(P.Tdw/Ts))>1e-9
    warning('Tm, Td and Tdw should be multiples of Ts (they are rounded in the simulation)');
end
if P.Td_obs > Tm, warning('Td_obs > Tm: the delay-compensated observer is no longer exact (several samples in flight)'); end

%% ===================================================================================================================
%% 1. SYSTEM IDENTIFICATION
%% ===================================================================================================================
% Nonlinear relative kinematics (report Eq. nl) linearised about straight-line pursuit at d*, v_f = v_t = v0:
%   d_rho' = dv_t - dv_f         beta' = a*eps - w_f         eps' = w_t - w_f          a = v0/d*
a = P.v0/P.d;
A = [0 0 0; 
    0 0 a; 
    0 0 0];   
B = [-1 0; 
    0 -1; 
    0 -1];   
E = [1 0; 
    0 0; 
    0 1];   
C = [1 0 0; 
    0 1 0];
n = 3;  m = 2;  q = 2;
hdr('1. System identification');
fprintf('  Design model: LTI, continuous-time, n = %d states, m = %d controlled inputs, q = %d outputs, 2 disturbance inputs.\n', n, m, q);
fprintf('  => MIMO (%d x %d). It is also sampled-data (control Ts = %.2f s, measurements every Tm = %.2f s) and delayed (Td = %.2f s,\n', q, m, Ts, Tm, Td);
fprintf('     pair gap Tdw = %.2f s, replay to the pair midpoint Td_obs = %.2f s, information delay T_eff = %.2f s).\n', P.Tdw, P.Td_obs, P.T_eff);
fprintf('  Poles of A: '); fprintf('%g ', eig(A)); fprintf('  -> all at the origin: for a > 0, the nontrivial nilpotent Jordan block makes the free response UNSTABLE.\n');
fprintf('  Classification: the plant is NONLINEAR (Eq. nl); the design model is its linearization about straight-line pursuit: LTI at fixed v0\n');
fprintf('    (valid for small d_rho, beta, eps: see S1), dynamic (integrators), causal, deterministic. If v0 varies, A(t) is time-varying: see 2b.\n');
fprintf('    Hybrid structure: continuous plant, discrete controller / observer (Ts), intermittent measurements (Tm), causal delay Td (handled by the\n');
fprintf('    exact replay Phi(Td), so the design model stays finite-dimensional).\n');
Gss = ss(A,B,C,zeros(2,2));
Gn = squeeze(freqresp(Gss, 1));  Ga = diag([-1/1j, -(1j+a)/(1j)^2]);
fprintf('  Transfer matrix G(s) = C(sI-A)^-1 B = diag( -1/s , -(s+a)/s^2 ),  a = %.3g   (check vs numeric: %.1e)\n', a, max(abs(Gn(:)-Ga(:))));
fprintf('  => the 2x2 MIMO plant is input-output DECOUPLED: range loop (dv_f -> d_rho, SISO) and bearing loop (w_f -> beta, SISO,\n');
fprintf('     with eps as a hidden state and a zero at s = -a). A block-diagonal gain therefore gives two SISO designs.\n');

%% ===================================================================================================================
%% 2. CONTROLLABILITY / OBSERVABILITY (class method)
%% ===================================================================================================================
hdr('2. Structural analysis: Kalman rank tests, Kalman canonical form');
AN.plant = report_structure('(i) plant, nominal a', A, B, C);
AN.zero  = report_structure('(ii) degenerate case v0 -> 0 (a = 0): shows what hidden parts look like', zeros(3), B, C);
Ao5 = [A E; zeros(2,5)];  Bo5 = [B; zeros(2,2)];  Co5 = [C zeros(2,2)];
AN.dist  = report_structure('(iii) plant + constant target-disturbance states [dv_t; w_t]', Ao5, Bo5, Co5);
fprintf('\n  HOW EACH PART IS TREATED\n');
fprintf('   - plant (i): fully controllable and observable -> no hidden modes: state feedback can place all poles, an observer can\n');
fprintf('     estimate all states (eps is not measured but is observable through beta'', the disturbances through rho'').\n');
fprintf('   - case (ii): the uncontrollable/unobservable parts are marginal (eigenvalue 0): NOT stabilizable/detectable.\n');
fprintf('     Meaning: if the rovers stop (v0 = 0) the heading eps can no longer be inferred nor corrected: keep v0 > 0.\n');
fprintf('   - case (iii): the 2 disturbance states are UNCONTROLLABLE by u (by nature) but OBSERVABLE. They are marginal (constant),\n');
fprintf('     so they cannot be stabilized, and need not be: they are ESTIMATED by the observer and REJECTED by integral action\n');
fprintf('     (internal-model principle). Without them in the observer, a constant dv_t biases the estimate and the integrator.\n');

hdr('S2. Conditioning of the Gramians vs a = v0/d*  (finite horizon, tolerance-scaled coordinates)');
Sx = diag(P.S2.tol_x);  Su = diag(P.S2.tol_u);  Sy = diag(P.S2.tol_y);
for aa = P.S2.a_list
    A_ = [0 0 0; 0 0 aa; 0 0 0];
    As = Sx\(A_*Sx);  Bs = Sx\(B*Su);  Cs = Sy\(C*Sx);
    fprintf('  a = %5.2f  rank(ctrb) %d  rank(obsv) %d   cond(Wc) %.1e   cond(Wo) %.1e\n', aa, rank(ctrb(A_,B)), rank(obsv(A_,C)), ...
            cond(gram_fh(As,Bs,P.S2.T)), cond(gram_fh(As',Cs',P.S2.T)));
end

hdr('2b. Non-unique state models, analytic trajectories (LTI and LTV), state transition matrices');
realization_report(a);
analytic_checks(A, B, E, P);

%% ===================================================================================================================
%% 3. DESIGN DECISIONS (what is designed and why)
%% ===================================================================================================================
hdr('3. Design decisions');
fprintf('  Controller: state feedback u = -Kx*xhat - Kz*z, z = integral of d_rho_hat.  Why:\n');
fprintf('    * the plant is controllable, so state feedback can set the dynamics; LQR gives guaranteed margins for the state-feedback loop\n');
fprintf('      and a single knob (Q/R) per channel; pole placement (option) lets you dictate the poles directly.\n');
fprintf('    * O1 needs zero steady-state range error for a constant target speed offset: dv_t is a constant disturbance acting at the\n');
fprintf('      range integrator, so integral action is required (internal model).\n');
fprintf('  Estimator: needed because only y = [rho_hat; beta_hat] is measured, intermittently (every Tm), delayed (Td) and noisy.\n');
fprintf('    * eps and the disturbances are unmeasured but observable -> an observer on [x; dv_t; w_t] reconstructs them.\n');
fprintf('    * Kalman (default): the measurement noise is known (App. B variances) and varies with range -> optimal weighting.\n');
fprintf('      Luenberger (option): pole placement, no noise model; ideal (option): true state, to see the cost of estimation.\n');
fprintf('    * Delay: the correction is applied as Phi(Td)*L*(y - C*xhat(t-Td)): exact replay for an LTI model (needs Td <= Tm).\n');
fprintf('  Selected here: controller = %s, observer = %s, disturbance states = %s.\n', ctype, otype, yn{dist+1});

%% ===================================================================================================================
%% 4. SYNTHESIS
%% ===================================================================================================================
hdr('4a. Controller synthesis');
Cz = C(1,:);
Aa = [A zeros(3,1); Cz 0];  Ba = [B; 0 0];                 % augmented with z' = d_rho  (fixed layout for the implementation)
[Ad, Bd] = zoh_mat(Aa, Ba, Ts);
if any(strcmp(ctype, {'lqr_noint','bearing_only'}))
    Aloop = A;  Bloop = B;  nz = 0;
else
    Aloop = Aa; Bloop = Ba; nz = 1;
    tz = rank([A B; Cz zeros(1,m)]);
    fprintf('  Integral action feasible? rank[A B; Cz 0] = %d = n + 1 = %d -> %s (no transmission zero at s = 0 on the range channel)\n', ...
            tz, n+1, yn{(tz==n+1)+1});
    fprintf('  Augmented pair (Aa, Ba): rank(ctrb) = %d of %d.\n', rank(ctrb(Aa,Ba)), n+1);
end
Rd = diag(P.Rd);
switch ctype
    case 'lqr'
        Qd = diag(P.Qd);
        Kc = lqr(Aloop, Bloop, Qd, Rd);   Kd = dlqr(Ad, Bd, Qd, Rd);
    case 'lqr_noint'
        Qd = diag(P.Qd(1:3));
        Kc = lqr(A, B, Qd, Rd);   [A3, B3] = zoh_mat(A, B, Ts);   Kd = [dlqr(A3, B3, Qd, Rd) zeros(2,1)];
    case 'bearing_only'
        % Baseline: no range feedback, constant speed v0; steer on the estimated bearing only (same observer).
        Kc = [0 0 0; 0 -P.k_bearing 0];   Kd = [0 0 0 0; 0 -P.k_bearing 0 0];
    case 'place'
        % The plant is decoupled (section 1), so each channel is placed on its own SISO subsystem (MIMO place() would assign
        % the poles to channels arbitrarily). Range subsystem [d_rho; z] driven by dv_f, bearing subsystem [beta; eps] by w_f.
        [pr, pb] = pole_set(P, nz);
        if nz, Ar = [0 0; 1 0]; Br = [-1; 0]; else, Ar = 0; Br = -1; end
        Ab = [0 a; 0 0];  Bb = [-1; -1];
        [Ard, Brd] = zoh_mat(Ar, Br, Ts);  [Abd, Bbd] = zoh_mat(Ab, Bb, Ts);
        Kc = assemble_K(place(Ar, Br, pr), place(Ab, Bb, pb), nz);
        Kd = [assemble_K(place(Ard, Brd, exp(pr*Ts)), place(Abd, Bbd, exp(pb*Ts)), nz), zeros(2, 1-nz)];
        fprintf('  Pole placement: range poles = %s, bearing poles = %s (continuous)\n', mat2str(pr.', 3), mat2str(pb.', 3));
    otherwise, error('unknown controller type: %s', ctype);
end
Acl = Ad - Bd*Kd;  if nz == 0, Acl = Acl(1:3,1:3); end
fprintf('  Kd (rows dv_f, w_f; cols d_rho, beta, eps, int d_rho):\n'); disp(Kd);
fprintf('  closed-loop |eig| (discrete): '); fprintf('%.3f ', sort(abs(eig(Acl)))); fprintf('\n');
fprintf('  K block-diagonal (range loop / bearing loop decoupled)? %s\n', yn{(max(abs(Kd(1,[2 3]))) + max(abs(Kd(2,[1 4]))) < 1e-9) + 1});

hdr('4b. Observer synthesis');
if dist, Ao = Ao5;  Bo = Bo5;  Co = Co5;  nx = 5; else, Ao = A;  Bo = B;  Co = C;  nx = 3; end
[Phi, Gam] = zoh_mat(Ao, Bo, Ts);
Phim = expm(Ao*Tm);  PhiTd = expm(Ao*P.Td_obs);                % replay to the pair midpoint
Apk = P.K/P.d*exp(-P.dlt^2/(2*P.sig^2));                        % A+/- with the arm centred on the target, rho = d*
s2  = 2*(P.sn/Apk)^2;                                           % var(ln A+) + var(ln A-)   (App. B)
sr  = sqrt(P.d^2*s2/4 + P.sr_floor^2);   sb = sqrt((P.sig^2/(2*P.dlt))^2*s2 + P.sb_floor^2);
Rm  = diag([sr^2 sb^2]);                                        % per-measurement covariance
fprintf('  measurement std at d*: rho_hat %.3f m (%.1f%%), beta_hat %.2f deg\n', sr, 100*sr/P.d, r2d_(sb));
fprintf('  observable pair (Phi(Tm), Co): rank(obsv) = %d of %d\n', rank(obsv(Phim, Co)), nx);
switch otype
    case 'kalman'
        Qm = diag(P.Qw(1:nx))*Tm;                               % process noise accumulated over one measurement period
        Pp = dare(Phim', Co', Qm, Rm);   L = Pp*Co'/(Co*Pp*Co' + Rm);
    case 'luenberger'
        po = P.obs_poles(1:nx);                                 % continuous observer poles
        Ko = place(Phim', Co', exp(po*Tm))';   L = Phim\Ko;     % error dynamics e+ = Phim (I - L Co) e
    case 'ideal'
        L = zeros(nx, 2);
    otherwise, error('unknown observer type: %s', otype);
end
if ~strcmp(otype,'ideal')
    fprintf('  observer error poles, |z| per measurement: '); fprintf('%.3f ', sort(abs(eig(Phim*(eye(nx) - L*Co))))); fprintf('\n');
    fprintf('  (compare with the controller: the observer should be faster, but not so fast that it amplifies measurement noise)\n');
end

% Innovation gate (reflections, outliers). nu = y - Co*xhat(t-Td) has steady-state covariance S = Co*Pp*Co' + Rm, where Pp solves
%   Pp = F*Pp*F' + Phim*L*Rm*L'*Phim' + Qm,  F = Phim*(I - L*Co)     (valid for ANY gain L; equals the dare solution for the Kalman gain).
if strcmp(otype,'ideal')
    Sg = eye(2);
else
    Fo = Phim*(eye(nx) - L*Co);
    Pg = dlyap(Fo, Phim*L*Rm*L'*Phim' + diag(P.Qw(1:nx))*Tm);
    Sg = P.gate_infl*(Co*Pg*Co' + Rm);
    if P.gate
        fprintf('  innovation gate: nu''*inv(S)*nu <= %.2f (chi2, 2 dof), S inflated x%g, sqrt(diag(S)) = [%.3f m, %.2f deg]; larger innovations are clipped (not discarded), only near lock\n', ...
                P.gate_chi2, P.gate_infl, sqrt(Sg(1,1)), r2d_(sqrt(Sg(2,2))));
    else, fprintf('  innovation gate: DISABLED (P.gate = false)\n'); end
end
gate_chi2 = P.gate_chi2;  if ~P.gate, gate_chi2 = Inf; end

hdr('4c. Robustness O5 (revised): state-feedback loop at the plant input, actuator lags, information delay T_eff (Pade)');
% Assumes the delay-compensated observer delivers the current state, but counts the information delay T_eff in the loop
% (conservative): T_eff = Td_obs + Tm/2 + Ts/2 = age of the newest information on average, plus the ZOH.
fprintf('  T_eff = %.3f s. O5 (revised): delay margin beyond T_eff >= %.2f s and balanced disk margin >= %.2f per loop.\n', P.T_eff, P.DM_min, P.disk_min);
Gact = ss(diag([-1/P.tau_v -1/P.tau_w]), diag([1/P.tau_v 1/P.tau_w]), eye(2), zeros(2,2));
[nn, dd] = pade_tf(P.T_eff, P.pade_order);  Dl = ss(tf(nn, dd));
nl = size(Aloop,1);
Lsf = Kc*ss(Aloop, Bloop, eye(nl), zeros(nl,2))*Gact*append(Dl, Dl);
chname = {'range', 'bearing'};  verdict = {'FAIL', 'PASS'};
for ch = 1:2
    if norm(Kc(ch,:)) < eps
        MG.(chname{ch}) = [NaN NaN NaN];  fprintf('  %-8s no feedback on this channel (open loop)\n', chname{ch});  continue;
    end
    Lc = minreal(Lsf(ch,ch), 1e-6);                                 % minreal: drop cancelling integrator states (else margin() can return PM = 0)
    [Gm, Pm, ~, Wp] = margin(Lc);
    Lw = squeeze(freqresp(Lc, logspace(-3, 1.5, P.n_freq)));
    alpha = 1/max(abs((1 - Lw)./(2*(1 + Lw))));                     % balanced disk margin (skew 0)
    DM = Pm*pi/180/Wp;  ok = (DM >= P.DM_min) && (alpha >= P.disk_min);
    MG.(chname{ch}) = [Pm, 20*log10(Gm), Wp, DM, alpha];
    fprintf('  %-8s PM %5.1f deg @ %.2f rad/s, GM %5.1f dB | delay margin %.2f s beyond T_eff, disk alpha %.2f (+-%.1f dB, +-%.0f deg) -> %s\n', ...
            chname{ch}, Pm, Wp, 20*log10(Gm), DM, alpha, 20*log10((1+alpha/2)/(1-alpha/2)), 2*atand(alpha/2), verdict{ok+1});
end
w = logspace(-2, 1.3, P.n_freq);  Ms = 0;  cpl = 0;
for i = 1:numel(w)
    F = squeeze(freqresp(Lsf, w(i)));
    Ms = max(Ms, max(svd(inv(eye(2) + F))));
    cpl = max(cpl, max(abs(F(1,2)), abs(F(2,1)))/max(min(abs(F(1,1)), abs(F(2,2))), eps));
end
MG.Ms = Ms;
fprintf('  MIMO check: peak sensitivity max sigma(S) = %.2f (%.1f dB); loop coupling max|Lij|/|Lii| = %.2g (0 => loop-at-a-time margins exact)\n', ...
        Ms, 20*log10(Ms), cpl);

hdr('4d. Lyapunov and BIBO characterisation');
nl_ = size(Aloop,1);
fprintf('  open loop: for a > 0, A has a nontrivial zero Jordan block -> Lyapunov UNSTABLE; G(s) has poles at s = 0 -> not BIBO stable.\n');
Acc = Aloop - Bloop*Kc;
if max(real(eig(Acc))) < 0
    Pc_ = lyap(Acc', eye(nl_));
    fprintf('  continuous loop: Acl''P + P Acl = -I -> P = P'' > 0 (eig %.3g .. %.3g): V = x''Px decreases along trajectories -> asymptotically stable (max Re(eig) = %.3f)\n', ...
            min(eig((Pc_+Pc_')/2)), max(eig((Pc_+Pc_')/2)), max(real(eig(Acc))));
else
    fprintf('  continuous loop: max Re(eig) = %.3f >= 0 -> no Lyapunov function (range channel not stabilised by this controller type)\n', max(real(eig(Acc))));
end
if max(abs(eig(Acl))) < 1
    Pd_ = dlyap(Acl', eye(size(Acl,1)));
    fprintf('  discrete loop (Ts): Acl''P Acl - P = -I -> P > 0 (eig %.3g .. %.3g): asymptotically stable (spectral radius %.3f)\n', ...
            min(eig((Pd_+Pd_')/2)), max(eig((Pd_+Pd_')/2)), max(abs(eig(Acl))));
else
    fprintf('  discrete loop (Ts): spectral radius %.3f >= 1 -> no Lyapunov function\n', max(abs(eig(Acl))));
end
fprintf('  separation principle: with the exact Phi(Td) replay the observer error obeys e+ = Phi_m (I - L Co) e independently of K, so spec(loop) =\n');
fprintf('    spec(Acl) U spec(observer) for the LTI loop. It is lost with saturation, actuator lags (not in the observer) and the nonlinear plant: see S3.\n');

hdr('4e. MIMO pole placement: non-uniqueness and partial controllability');
mimo_placement_demo(A, B, C, P);

D = struct('Kd',Kd, 'Ad',Ad, 'Bd',Bd, 'Ad4',Ad(4,1:3), 'Bd4',Bd(4,:), 'integral',(nz==1), 'Phi',Phi, 'Gam',Gam, 'Co',Co, 'L',L, ...
           'PhiTd',PhiTd, 'ideal',strcmp(otype,'ideal'), 'nx',nx, 'S',Sg, 'gate_chi2',gate_chi2, 'gate_zone',[P.gate_zone_rho P.gate_zone_beta]);

%% ===================================================================================================================
%% 5. VALIDATION: S1, S3
%% ===================================================================================================================
x0n = [P.d; 0; 0];
if getopt(opt,'run_S1',true)
    hdr('S1. Linear vs nonlinear model');
    f  = @(x,u,w_) f_nl(x, u(1), u(2), w_(1), w_(2));
    Aj = jac_fd(@(z) f(z,[P.v0;0],[P.v0;0]), x0n);  Bj = jac_fd(@(z) f(x0n,z,[P.v0;0]), [P.v0;0]);  Ej = jac_fd(@(z) f(x0n,[P.v0;0],z), [P.v0;0]);
    fprintf('  max |numeric Jacobian - (A,B,E)| = %.2e\n', max([max(abs(Aj(:)-A(:))) max(abs(Bj(:)-B(:))) max(abs(Ej(:)-E(:)))]));
    fprintf('  normalized RMS error, nonlinear plant vs LTI model under the same discrete state feedback (offsets = scale * base):\n');
    base = P.S1.base;  nS = round(P.S1.T/Ts);  S1fig = {};
    for s = P.S1.scales
        xn = base*s;  zn = 0;  xa = [base*s; 0];  XN = zeros(3,nS);  XL = XN;
        for k = 1:nS
            un = -Kd(:,1:3)*xn - Kd(:,4)*zn;
            zn = zn + Ad(4,1:3)*xn + Bd(4,:)*un;
            xn = rk4_step(x0n + xn, [P.v0 + un(1); un(2)], [P.v0; 0], Ts) - x0n;
            xa = (Ad - Bd*Kd)*xa;  XN(:,k) = xn;  XL(:,k) = xa(1:3);
        end
        nr = sqrt(mean((XN-XL).^2, 2))./sqrt(mean(XL.^2, 2));
        if any(abs(s - [0.5 1 1.5]) < 1e-9), S1fig(end+1,:) = {s, XN, XL}; end %#ok<AGROW>
        fprintf('    scale %.2f (d_rho0 %.2f m, beta0 %.0f deg, eps0 %.0f deg): d_rho %.3f  beta %.3f  eps %.3f\n', s, base(1)*s, ...
                r2d_(base(2)*s), r2d_(base(3)*s), nr);
    end
    if getopt(opt,'plot',true), plot_S1(S1fig, P); end
end

S3 = struct();
if getopt(opt,'run_mc',true)
    hdr('S3. Closed loop: nonlinear plant + acoustic sensor model + delays + observer + controller');
    x0 = P.x0_demo;
    out = run_closed_loop(P, D, x0, @(t) [P.v0 + P.dvt_demo; 0], P.T);
    S3.demo = loop_metrics(out, P);
    fprintf('  demo run (start %.2f m too far, %.0f deg bearing, target %+.2f m/s): settling %.1f s, overshoot %.1f %%, RMS beta %.2f deg, yss %.1f mm\n', ...
            x0(1)-P.d, r2d_(x0(2)), P.dvt_demo, S3.demo.ts, S3.demo.ov, S3.demo.rms_beta_deg, 1e3*S3.demo.yss);
    fprintf('  demo run: gate clipped %d of %d valid samples; watchdog stopped the follower for %.1f s; min range %.3f m (rho_safe %.3f m)\n', ...
            out.n_rej, out.n_valid, out.t_stop, S3.demo.rho_min, P.rho_safe);
    if getopt(opt,'plot',true)
        try, plot_run(out, P); catch, disp('  (no graphics available: plot skipped)'); end
    end
    scn = {'transient, no disturbance (O3)', 'constant target speed offset (O1)', ...
           'stress: reflections 15%, drops 20%, period jitter +-20%, speed offset (gate ON)', 'same stress, gate OFF', ...
           'multipath at the horn12 target (fringe 0.07, room bias 10 deg), speed offset', ...
           'multipath at the s1 level (fringe 0.144, room bias 23 deg), speed offset'};
    names = {'2% settling [s]', 'overshoot [%] (|d_rho0|>=0.25 m)', 'RMS beta, last 10 s [deg]', 'steady-state range error [m]', 'min range [m]'};
    rng(P.seed);
    for sc = 1:6
        Ps_ = P;  Dg = D;
        if sc == 5, Ps_.fringe = P.mp_horn12(1);  Ps_.room_sd = P.mp_horn12(2); end
        if sc == 6, Ps_.fringe = P.mp_s1(1);  Ps_.room_sd = P.mp_s1(2); end
        if sc == 3 || sc == 4
            rng(P.seed + 100);                                      % identical random draws in scenarios 3 and 4 (paired comparison)
            Ps_.refl_p = max(P.refl_p, 0.15);  Ps_.p_drop = max(P.p_drop, 0.2);  Ps_.Tm_jitter = max(P.Tm_jitter, 0.2);
        end
        if sc == 4, Dg.gate_chi2 = Inf; end
        Rr = zeros(P.N_mc, 5);
        for i = 1:P.N_mc
            xr = [P.d + P.d0_range(1) + diff(P.d0_range)*rand;  d2r_(P.beta0_range_deg(1) + diff(P.beta0_range_deg)*rand); ...
                  d2r_(P.eps0_range_deg(1) + diff(P.eps0_range_deg)*rand)];
            if sc == 1, dvt = 0; else, dvt = P.dvt_test*(2*randi(2)-3); end
            mm = loop_metrics(run_closed_loop(Ps_, Dg, xr, @(t) [P.v0 + dvt; 0], P.T), Ps_);
            Rr(i,:) = [mm.ts mm.ov mm.rms_beta_deg mm.yss mm.rho_min];
        end
        S3.(sprintf('scenario%d', sc)) = Rr;
        fprintf('\n  %s, %d runs (mean +- std, worst)\n', scn{sc}, P.N_mc);
        for j = 1:5
            v = Rr(~isnan(Rr(:,j)), j);
            if j == 5, wv = min(v); elseif j == 4, wv = max(abs(v)); else, wv = max(v); end
            fprintf('    %-34s %7.3f +- %6.3f   worst %7.3f\n', names{j}, mean(v), std(v), wv);
        end
        fprintf('    %-34s %d of %d\n', 'runs with min range < rho_safe', sum(Rr(:,5) < P.rho_safe), P.N_mc);
    end
end

%% ===================================================================================================================
%% 6. EXPORT
%% ===================================================================================================================
R = struct('P',P, 'D',D, 'analysis',AN, 'margins',MG, 'S3',S3, 'opt',struct('controller',ctype,'observer',otype,'dist_states',dist));
R.simulate = @(x0, wfun, T, sn) run_closed_loop(with_field_(P,'sn',sn), D, x0, wfun, T);   % reference simulation (used to validate Simulink)
if getopt(opt,'export',true) && exist('jsonencode') > 0 %#ok<EXIST>
    G = struct('Ts',Ts, 'Tm',Tm, 'Td',Td, 'd',P.d, 'v0',P.v0, 'controller',ctype, 'observer',otype, 'nx',nx, 'integral',D.integral, ...
               'Kd',Kd, 'Ad4',D.Ad4, 'Bd4',D.Bd4, 'Phi',Phi, 'Gam',Gam, 'Co',Co, 'L',L, 'PhiTd',PhiTd, ...
               'S',Sg, 'gate_chi2',min(gate_chi2,1e9), 'gate_zone',[P.gate_zone_rho P.gate_zone_beta], 't_lost',P.t_lost, 'rho_stop',P.rho_stop, 'v_max',P.v_max);
    fid = fopen(fullfile(P.out_dir, 'gains.json'),'w');  fwrite(fid, jsonencode(G));  fclose(fid);
    fprintf('\nsaved %s (controller and observer matrices)\n', fullfile(P.out_dir, 'gains.json'));
end
if getopt(opt,'run_simulink',true) && D.nx == 5 && ~D.ideal
    hdr('S4. Simulink implementation check (Matlab/acoustic_pursuit_loop.slx) against the reference simulation');
    R.simulink = simulink_check(P, D);
end
if getopt(opt,'course_plots',true)
    R.course = course_plots(P);                                    % Ch. 6 figures c1..c5 (LQR, root locus, LQI, LQE, LQG)
end
end

%% =====================================================================================================================
%% USER SETTINGS  -- every hard-coded number of the design lives here
%% =====================================================================================================================
function P = user_settings()
% Phase E: every measured value comes from Config/phase_e.json (shared with the Python runtime and rover.phase_e); the design
% knobs are the same keys (lqr_q/lqr_r, range_poles/bearing_poles, observer_qw). Values not in the JSON are commented.
root = fileparts(fileparts(mfilename('fullpath')));
P.config = fullfile(root, 'Config', 'phase_e.json');
c = jsondecode(fileread(P.config));
% ---- (A) MEASURED / PLATFORM VALUES (vehicle values are PROVISIONAL until Phase D) ------------------------------------
P.v0 = c.nominal_speed;  P.d = c.desired_range;                    % target speed [m/s], desired separation d* [m]
P.L = c.wheelbase;  P.ds_max = d2r_(c.max_steer_deg);              % wheelbase [m], steering limit [rad]           (provisional)
P.v_max = c.max_speed;  P.tau_v = c.tau_speed;  P.tau_w = c.tau_steer;   % speed limit, speed / yaw lags [s]      (provisional)
P.K = c.amplitude_constant;  P.Kerr = 1.0;                         % K [counts m]; Kerr = K_true/K_model
P.sig = d2r_(c.sigma_deg);  P.dlt = d2r_(c.dither_deg);            % horn Gaussian sigma 77 deg, dither 50 deg (measured)
P.sn = c.amplitude_noise;  P.A_thr = c.amplitude_min;              % amplitude noise std, detection threshold [counts]
P.phi_max = d2r_(c.horn_limit_deg);                                % servo limit [rad]
P.sr_floor = c.range_noise_floor_rel*c.desired_range;              % range floor 0.10 rho at d* [m] (multi-tone, s1)
P.sb_floor = d2r_(c.bearing_noise_floor_deg);                      % bearing floor 13 deg: per-pair noise + fast fringe (s1)
P.Ts = c.dt;                                                       % control period [s]
P.Tm = P.Ts*round(c.measurement_period/P.Ts);                      % 0.478 s -> 0.50 s (multiple of Ts, conservative)
P.Tdw = P.Ts*round(c.pair_gap/P.Ts);                               % pair gap 0.405 s -> 0.40 s
P.Td = P.Ts*max(1, round(c.transport_delay/P.Ts));                 % A+ capture -> controller (transport, provisional)
P.Td_obs = P.Td + P.Tdw/2;                                         % observer replay to the pair midpoint
P.T_eff = P.Td_obs + P.Tm/2 + P.Ts/2;                              % information delay for the O5 margins
% ---- (B) CONTROLLER KNOBS (same keys as the Python runtime) ---------------------------------------------------------
P.Qd = c.lqr_q(:).';  P.Rd = c.lqr_r(:).';                         % Bryson weights (see design_phase_e in the JSON)
P.pp.range_poles = c.range_poles(:);  P.pp.bearing_poles = c.bearing_poles(:);
P.pp.wn_r = -max(c.range_poles);  P.pp.zeta_r = 1;  P.pp.wn_b = -max(c.bearing_poles);  P.pp.zeta_b = 1;   % used by 4e only
P.zband = 0.15;                                                    % integrate only when |d_rho_hat| < zband [m]
P.zmax = c.integral_limit;                                         % integrator clamp [m s]
P.k_bearing = c.baseline_bearing_gain;                             % bearing-only baseline gain
% ---- (C) OBSERVER KNOBS --------------------------------------------------------------------------------------------
P.Qw = c.observer_qw(:).';                                         % [d_rho beta eps dv_t w_t] process-noise intensities
P.obs_poles = -[1.0 1.25 1.5 1.75 2.0];                            % 'luenberger' option only
% ---- (D) SPECIFICATION AND ANALYSIS --------------------------------------------------------------------------------
P.PM_min = 45;  P.GM_min_dB = 6;                                   % classical O5 (reported, not used as the gate)
P.DM_min = P.T_eff;  P.disk_min = 0.5;                             % O5 revised: tolerate +T_eff, disk alpha >= 0.5
P.pade_order = 3;  P.n_freq = 600;
a0 = P.v0/P.d;  P.S2.a_list = [0 0.05 a0 0.25 0.5 1.0 2.5];  P.S2.T = 6;
P.S2.tol_x = [0.15 d2r_(10) d2r_(20)];  P.S2.tol_u = [0.1 0.3];  P.S2.tol_y = [P.sr_floor P.sb_floor];
P.S1.base = [0.3; d2r_(20); d2r_(20)];  P.S1.scales = [0.25 0.5 1 1.5 2];  P.S1.T = 40;
% ---- (E) SIMULATION -------------------------------------------------------------------------------------------------
P.T = 60;  P.seed = 1;  P.N_mc = 10;
P.x0_demo = [P.d + 0.5; d2r_(10); d2r_(0)];  P.dvt_demo = 0.03;
P.d0_range = [-0.10 0.50];  P.beta0_range_deg = [-15 15];  P.eps0_range_deg = [-10 10];
P.dvt_test = 0.03;
% ---- (F) ROBUSTNESS LAYER --------------------------------------------------------------------------------------------
P.gate = true;  P.gate_chi2 = 9.21;  P.gate_infl = 1.0;
P.gate_zone_rho = 0.30;  P.gate_zone_beta = d2r_(20);
P.t_lost = c.timeout;  P.rho_stop = c.clearance;  P.rho_safe = c.clearance;
P.Tm_jitter = 0;  P.p_drop = 0;  P.outage = [];
P.refl_p = 0;  P.refl_g = 0.5;
% ---- (G) MULTIPATH (Phase B s1; rover.simulation.MultipathModel) -----------------------------------------------------
P.fringe = 0;  P.room_sd = 0;  P.room_tau = c.room_length_m/P.v0;  % off by default; set per scenario (S3 5-6)
P.mp_horn12 = [0.07 d2r_(10)];  P.mp_s1 = [0.144 d2r_(23)];
end

function CP = course_plots(P)
% Chapter 6 (LQR / LQI / LQE / LQG) results in the lecture's format, for the linear design model at the operating point.
% Continuous-time designs with the lecture's MATLAB commands (lqr, lqi, lqe); the Kalman simulation uses the discrete
% 8-step algorithm at the measurement period Tm. Figures c1..c5 in P.out_dir; gains printed to the log.
a = P.v0/P.d;
A = [0 0 0; 0 0 a; 0 0 0];  B = [-1 0; 0 -1; 0 -1];  E = [1 0; 0 0; 0 1];  C = [1 0 0; 0 1 0];
Q = diag(P.Qd);  R0 = diag(P.Rd);                 % [e beta eps int(e)], [dv w]  (same weights as the runtime LQI)
sysr = ss(A, B, [1 0 0], zeros(1,2));              % lqi integrates the range-error output
Aa = [A zeros(3,1); -[1 0 0] 0];  Ba = [B; 0 0];   % z' = r - y = -e (lqi convention; z = -int(e))
col = {[42 120 214]/255, [235 104 52]/255, [27 175 122]/255};  ink = [.32 .32 .31];  grey = [.6 .6 .58];
dvmax = P.v_max - P.v0;  wmax = P.v0*tan(P.ds_max)/P.L;
t = (0:0.02:40)';  x0 = [0.3; d2r_(15); 0; 0];
hdr('C. Chapter 6 results (LQR / LQI / LQE / LQG), continuous-time design model');
[K, S, ecl] = lqi(sysr, Q, R0);
fprintf('  LQI (lqi):  K = [Kx | Kz] =\n'); disp(K);
fprintf('  ARE solution P = S (symmetric, positive definite: eig %s)\n', mat2str(round(eig(S).'*1e3)/1e3));
fprintf('  closed-loop poles: %s\n', mat2str(round(ecl.'*1e3)/1e3));

% ---- C1: effect of R (aggressive / design / conservative) -------------------------------------------------------
sc = [0.1 1 10];  lab = {'R x 0.1 (aggressive)', 'R (design)', 'R x 10 (conservative)'};
f = figure('Visible','off', 'Color',[252 252 251]/255, 'Position',[100 100 980 520]);
tl = tiledlayout(f, 2, 2, 'TileSpacing','compact');
ax = gobjects(4,1); for k = 1:4, ax(k) = nexttile(tl); hold(ax(k),'on'); grid(ax(k),'on'); box(ax(k),'off'); end
fprintf('  C1 effect of R (lecture slides 18-21):\n');
for i = 1:3
    Ki = lqi(sysr, Q, sc(i)*R0);  X = lsim(ss(Aa - Ba*Ki, zeros(4,1), eye(4), 0), zeros(size(t)), t, x0);  U = -X*Ki.';
    plot(ax(1), t, X(:,1), 'Color', col{i}, 'LineWidth', 1.8);  plot(ax(2), t, r2d_(X(:,2)), 'Color', col{i}, 'LineWidth', 1.8);
    plot(ax(3), t, U(:,1), 'Color', col{i}, 'LineWidth', 1.8);  plot(ax(4), t, U(:,2), 'Color', col{i}, 'LineWidth', 1.8);
    fprintf('    %-24s K = %s   peak |dv| %.2f m/s, peak |w| %.2f rad/s\n', lab{i}, mat2str(round(Ki*1e3)/1e3), max(abs(U(:,1))), max(abs(U(:,2))));
end
yline(ax(3), [dvmax -P.v0], '--', 'Color', grey);  yline(ax(4), [wmax -wmax], '--', 'Color', grey);
ylabel(ax(1), 'range error \delta\rho [m]');  ylabel(ax(2), 'bearing \beta [deg]');
ylabel(ax(3), 'speed command \deltav_f [m/s]');  ylabel(ax(4), 'yaw-rate command \omega_f [rad/s]');
xlabel(ax(3), 't [s]');  xlabel(ax(4), 't [s]');
lg = legend(ax(1), lab, 'Orientation','horizontal', 'Box','off');  lg.Layout.Tile = 'north';
title(tl, 'C1  LQR trade-off: small R = fast but saturates the actuators (dashed limits); large R = slow', 'FontSize',10, 'FontWeight','bold');
exportgraphics(f, fullfile(P.out_dir, 'c1_lqr_effect_of_R.png'), 'Resolution', 160);  close(f);

% ---- C2: closed-loop pole locus vs rho (R = rho * R_design) -------------------------------------------------------
rho = logspace(-2, 2, 81);  PR = zeros(2, numel(rho));  PB = PR;
for i = 1:numel(rho)
    Ki = lqi(sysr, Q, rho(i)*R0);  Acl = Aa - Ba*Ki;
    PR(:,i) = eig(Acl([1 4],[1 4]));  PB(:,i) = eig(Acl([2 3],[2 3]));       % decoupled range / bearing blocks
end
f = figure('Visible','off', 'Color',[252 252 251]/255, 'Position',[100 100 820 420]);  hold on; grid on; box off;
plot(real(PR(:)), imag(PR(:)), '.', 'Color', col{1}, 'MarkerSize', 8);  plot(real(PB(:)), imag(PB(:)), '.', 'Color', col{2}, 'MarkerSize', 8);
Kd = lqi(sysr, Q, R0);  pd = eig(Aa - Ba*Kd);
plot(real(pd), imag(pd), 'kd', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(0, 0, 'kx', 'MarkerSize', 12, 'LineWidth', 2);  plot(-a, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
xline(0, 'Color', grey);
legend({'range loop poles (\delta\rho, \int\delta\rho)', 'bearing loop poles (\beta, \epsilon)', 'design \rho = 1', ...
        'open-loop poles (all at 0)', 'bearing-channel zero at -a'}, 'Location','southwest', 'Box','off');
xlim([-2 0.05]);  ylim([-0.15 0.15]);
text(-1.95, 0.12, {'\rho \downarrow (cheap control): poles move left, gains grow', '(\rho \rightarrow 0: one pole per loop \rightarrow -\infty, the other \rightarrow -a or a finite limit)'}, 'Color', ink, 'FontSize', 8);
text(-0.08, -0.13, '\rho \uparrow: back to 0', 'Color', ink, 'FontSize', 8, 'HorizontalAlignment','right');
xlabel('Re(s) [1/s]');  ylabel('Im(s) [rad/s]');
title(sprintf('C2  LQR root locus vs \\rho (R = \\rho R_{design}, \\rho = 10^{-2}..10^{2}); \\rho \\rightarrow \\infty returns the poles to 0'), 'FontSize', 10);
exportgraphics(f, fullfile(P.out_dir, 'c2_lqr_root_locus.png'), 'Resolution', 160);  close(f);

% ---- C3: LQR vs LQI under a constant target-speed offset (internal-model principle) -------------------------------
K3 = lqr(A, B, Q(1:3,1:3), R0);  dvt = 0.05;  w = dvt*(t >= 5);
X3 = lsim(ss(A - B*K3, E(:,1), eye(3), 0), w, t);                 U3 = -X3*K3.';
XI = lsim(ss(Aa - Ba*K, [E(:,1); 0], eye(4), 0), w, t);           UI = -XI*K.';
ess = dvt/K3(1,1)*(-1);                                           % steady state of e' = dvt + K3(1,1) e (K3(1,1) < 0 with B = -1)
fprintf('  C3 LQR K = %s: steady range error under dv_t = %.2f m/s: %.3f m (analytic dv_t/|k_e| = %.3f m); LQI: %.1e m\n', ...
        mat2str(round(K3*1e3)/1e3), dvt, X3(end,1), abs(ess), XI(end,1));
f = figure('Visible','off', 'Color',[252 252 251]/255, 'Position',[100 100 980 340]);
tl = tiledlayout(f, 1, 2, 'TileSpacing','compact');
nexttile(tl); hold on; grid on; box off;
plot(t, X3(:,1), 'Color', col{2}, 'LineWidth', 1.8);  plot(t, XI(:,1), 'Color', col{1}, 'LineWidth', 1.8);
yline(X3(end,1), ':', sprintf('LQR offset %.2f m', X3(end,1)), 'Color', ink, 'LabelHorizontalAlignment','left');
ylabel('range error \delta\rho [m]');  xlabel('t [s]');
nexttile(tl); hold on; grid on; box off;
plot(t, U3(:,1), 'Color', col{2}, 'LineWidth', 1.8);  plot(t, UI(:,1), 'Color', col{1}, 'LineWidth', 1.8);
yline(dvt, ':', 'target offset 0.05 m/s', 'Color', ink, 'LabelHorizontalAlignment','left');
ylabel('speed command \deltav_f [m/s]');  xlabel('t [s]');
lg = legend({'LQR (no integral)', 'LQI'}, 'Orientation','horizontal', 'Box','off');  lg.Layout.Tile = 'north';
title(tl, 'C3  Target speeds up by 0.05 m/s at t = 5 s: LQR keeps an offset, LQI removes it (O1)', 'FontSize',10, 'FontWeight','bold');
exportgraphics(f, fullfile(P.out_dir, 'c3_lqr_vs_lqi.png'), 'Resolution', 160);  close(f);

% ---- C4: LQE (Kalman) tuning: sensor-reliant vs model-reliant ------------------------------------------------------
Ao = [A E; zeros(2,5)];  Co = [C zeros(2,2)];  W = diag(P.Qw);  sr = P.sr_floor;  sb = P.sb_floor;
V = diag([sr^2 sb^2])*P.Tm;                                        % sampled noise as a continuous PSD
vs = [0.01 1 100];  vlab = {'V x 0.01 (trusts the sensor)', 'V (measured noise)', 'V x 100 (trusts the model)'};
fprintf('  C4 LQE (lqe, continuous, G = I, W = diag(observer_qw), V = diag(sigma^2) Tm): Kalman gain rows [d_rho beta eps dv_t w_t]\n');
for i = 1:3
    Le = lqe(Ao, eye(5), Co, W, vs(i)*V);
    fprintf('    %-30s Ke = %s\n', vlab{i}, mat2str(round(Le*1e3)/1e3));
end
rng(7);  dt = P.Ts;  tt = (0:dt:60)';  nm = round(P.Tm/dt);
wt = [0.05*(tt >= 10 & tt < 25), 0.03*(tt >= 30 & tt < 45)];       % target speeds up, then turns; follower holds v0 (u = 0)
[Ph, Gd] = zoh_mat(Ao(1:3,1:3), E, dt);  X = zeros(numel(tt), 3);
for k = 1:numel(tt)-1, X(k+1,:) = (Ph*X(k,:).' + Gd*wt(k,:).').'; end
km = 1:nm:numel(tt);  Y = X(km,1:2) + [sr*randn(numel(km),1), sb*randn(numel(km),1)];
XH = zeros(numel(tt), 5, 3);
for i = 1:3
    XH(:,:,i) = kf_run(Ao, Co, W, vs(i)*diag([sr^2 sb^2]), dt, km, Y, numel(tt), [sr sb]);   % lecture's 8-step KF
    e = squeeze(XH(:,1:3,i)) - X;
    fprintf('    %-30s RMS estimate error: d_rho %.3f m, beta %.1f deg, eps %.1f deg\n', vlab{i}, rms(e(:,1)), r2d_(rms(e(:,2))), r2d_(rms(e(:,3))));
end
vsw = logspace(-2, 3, 21);  ER = zeros(numel(vsw), 3);
for i = 1:numel(vsw)
    xe = kf_run(Ao, Co, W, vsw(i)*diag([sr^2 sb^2]), dt, km, Y, numel(tt), [sr sb]);  e = xe(:,1:3) - X;
    ER(i,:) = [rms(e(:,1)) r2d_(rms(e(:,2))) r2d_(rms(e(:,3)))];
end
[~, iopt] = min(ER(:,3));  CPv = vsw(iopt);
fprintf('    RMS-optimal V scale for this target (eps): %.3g x the measured noise (W = observer_qw assumes faster manoeuvres)\n', CPv);
f = figure('Visible','off', 'Color',[252 252 251]/255, 'Position',[100 100 1000 780]);
tl = tiledlayout(f, 4, 1, 'TileSpacing','compact');  names = {'\delta\rho [m]', '\beta [deg]', '\epsilon [deg] (hidden)'};
for r = 1:3
    nexttile(tl); hold on; grid on; box off;  s = 1;  if r > 1, s = 180/pi; end
    if r < 3, plot(tt(km), s*Y(:,r), '.', 'Color', grey, 'MarkerSize', 7); end
    plot(tt, s*X(:,r), 'Color', ink, 'LineWidth', 2.4);
    for i = 1:3, plot(tt, s*XH(:,r,i), 'Color', col{i}, 'LineWidth', 1.3); end
    ylabel(names{r});  if r == 3, xlabel('t [s]  (follower holds v_0; target speeds up at 10-25 s, turns at 30-45 s)'); end
    if r == 1, axl = gca; end
end
nexttile(tl); hold on; grid on; box off;
plot(vsw, ER(:,2), '-o', 'Color', col{1}, 'LineWidth', 1.6, 'MarkerSize', 3);  plot(vsw, ER(:,3), '-o', 'Color', col{2}, 'LineWidth', 1.6, 'MarkerSize', 3);
set(gca, 'XScale', 'log');  xline(1, ':', 'measured V', 'Color', ink);  xlabel('V scale (x measured noise)');  ylabel('RMS estimate error [deg]');
legend({'\beta', '\epsilon'}, 'Location','northeast', 'Box','off');
lg = legend(axl, {'measurements (every T_m)', 'true state', vlab{:}}, 'Orientation','horizontal', 'Box','off', 'NumColumns', 3);  lg.Layout.Tile = 'north';
title(tl, 'C4  LQE tuning: small V follows the noise, large V lags; \epsilon is only inferred through \beta'' = a\epsilon (bottom: error vs V)', 'FontSize',10, 'FontWeight','bold');
exportgraphics(f, fullfile(P.out_dir, 'c4_lqe_tuning.png'), 'Resolution', 160);  close(f);

% ---- C5: LQG = LQI + Kalman (separation principle) -------------------------------------------------------------------
Phd = expm(Ao*P.Tm);  Pp = dare(Phd.', Co.', W*P.Tm, diag([sr^2 sb^2]));  Lk = Pp*Co.'/(Co*Pp*Co.' + diag([sr^2 sb^2]));
eo = eig(Phd*(eye(5) - Lk*Co));
fprintf('  C5 separation: controller poles %s (continuous) U observer |z| per Tm %s\n', mat2str(round(ecl.'*1e3)/1e3), mat2str(round(abs(eo).'*1e3)/1e3));
rng(8);  [PhB, GdB] = zoh_mat(A, [B E], dt);  X = zeros(numel(tt),3);  X(1,:) = [0.3 d2r_(15) 0];  z = 0;
xh = zeros(5,1);  Pk = diag([sr^2 sb^2 d2r_(20)^2 0.05^2 0.03^2]);  XH = zeros(numel(tt),5);  U = zeros(numel(tt),2);  init = false;
PhiO = expm(Ao*dt);  [~, GoB] = zoh_mat(Ao, [B; zeros(2,2)], dt);
for k = 1:numel(tt)
    if mod(k-1, nm) == 0
        y = X(k,1:2).' + [sr*randn; sb*randn];
        if ~init, xh = [y; 0; 0; 0]; init = true;
        else, Sk = Co*Pk*Co.' + diag([sr^2 sb^2]);  Kk = Pk*Co.'/Sk;  xh = xh + Kk*(y - Co*xh);  Pk = (eye(5) - Kk*Co)*Pk; end
    end
    XH(k,:) = xh.';
    u = -K*[xh(1:3); z];  u(1) = min(max(u(1), -P.v0), dvmax);  u(2) = min(max(u(2), -wmax), wmax);  U(k,:) = u.';
    z = z - xh(1)*dt;                                                 % z' = -e (lqi convention)
    if k < numel(tt)
        X(k+1,:) = (PhB*X(k,:).' + GdB*[u; wt(k,:).']).';
        xh = PhiO*xh + GoB*u;  Pk = PhiO*Pk*PhiO.' + W*dt;
    end
end
f = figure('Visible','off', 'Color',[252 252 251]/255, 'Position',[100 100 980 560]);
tl = tiledlayout(f, 3, 1, 'TileSpacing','compact');
for r = 1:2
    axr = nexttile(tl); hold on; grid on; box off;  s = 1;  if r > 1, s = 180/pi; end
    plot(tt, s*X(:,r), 'Color', ink, 'LineWidth', 2.2);  plot(tt, s*XH(:,r), 'Color', col{1}, 'LineWidth', 1.4);  ylabel(names{r});
    if r == 1, ax1 = axr; end
end
nexttile(tl); hold on; grid on; box off;
plot(tt, U(:,1), 'Color', col{2}, 'LineWidth', 1.4);  plot(tt, U(:,2), 'Color', col{3}, 'LineWidth', 1.4);  ylabel('commands');  xlabel('t [s]');
legend({'\deltav_f [m/s]', '\omega_f [rad/s]'}, 'Location','eastoutside', 'Box','off');
lg = legend(ax1, {'true', 'Kalman estimate'}, 'Orientation','horizontal', 'Box','off');  lg.Layout.Tile = 'north';
title(tl, sprintf('C5  LQG (LQI + Kalman at T_m, linear model): after 10 s range within %.2f m, bearing within %.0f deg; each T_m update steps the commands', max(abs(X(tt > 10,1))), r2d_(max(abs(X(tt > 10,2))))), 'FontSize',10, 'FontWeight','bold');
exportgraphics(f, fullfile(P.out_dir, 'c5_lqg.png'), 'Resolution', 160);  close(f);
CP = struct('K_lqi', K, 'P_are', S, 'poles', ecl, 'K_lqr', K3, 'L_kalman', Lk, 'observer_absz', abs(eo), 'V_scale_opt', CPv);
end

function XH = kf_run(Ao, Co, W, Rd, dt, km, Y, n, s0)
% Discrete Kalman filter, lecture's 8-step algorithm: predict every dt (model only), correct at each measurement (every Tm).
Phi = expm(Ao*dt);  Qd = W*dt;  xh = zeros(5,1);  Pk = diag([s0(1)^2 s0(2)^2 (20*pi/180)^2 0.05^2 0.03^2]);  j = 1;  XH = zeros(n,5);
for k = 1:n
    if k > 1, xh = Phi*xh;  Pk = Phi*Pk*Phi.' + Qd; end
    if j <= numel(km) && k == km(j)
        Sk = Co*Pk*Co.' + Rd;  Kk = Pk*Co.'/Sk;  xh = xh + Kk*(Y(j,:).' - Co*xh);  Pk = (eye(5) - Kk*Co)*Pk;  j = j + 1;
    end
    XH(k,:) = xh.';
end
end

function d = default_out_dir()
root = fileparts(fileparts(mfilename('fullpath')));
t = datetime('now', 'TimeZone', 'UTC');
d = fullfile(root, 'Runs', char(t, 'yyyy-MM-dd'), [char(t, 'yyyyMMdd''T''HHmmss') '_matlab_phase_e']);
end

function plot_S1(S1fig, P)
% Linear (LTI design model) vs nonlinear plant under the same discrete state feedback, three offset sizes.
cols = {[42 120 214]/255, [235 104 52]/255, [27 175 122]/255};
f = figure('Visible','off', 'Color',[252 252 251]/255, 'Position',[100 100 900 330]);
t = (1:size(S1fig{1,2},2))*P.Ts;
for k = 1:2
    subplot(1,2,k); hold on; grid on; box off;
    for i = 1:size(S1fig,1)
        sc = 1; if k == 2, sc = 180/pi; end
        plot(t, sc*S1fig{i,2}(k,:), '-', 'Color', cols{i}, 'LineWidth', 1.8);
        plot(t, sc*S1fig{i,3}(k,:), '--', 'Color', cols{i}, 'LineWidth', 1.2, 'HandleVisibility','off');
    end
    xlabel('t [s]');
    if k == 1, ylabel('range error \delta\rho [m]'); else, ylabel('bearing \beta [deg]'); end
end
subplot(1,2,1); legend(arrayfun(@(i) sprintf('offset x%.1f (\\beta_0 = %.0f deg)', S1fig{i,1}, S1fig{i,1}*20), 1:size(S1fig,1), 'UniformOutput', false), 'Location','northeast', 'Box','off');
sgtitle('M1  Linear model (dashed) vs nonlinear plant (solid), same LQI: valid to |\beta|, |\epsilon| \approx 20 deg', 'FontSize', 10, 'FontWeight', 'bold');
exportgraphics(f, fullfile(P.out_dir, 'm1_linear_vs_nonlinear.png'), 'Resolution', 160);  close(f);
end

function res = simulink_check(P, D)
% Runs Matlab/acoustic_pursuit_loop.slx (MATLAB Function blocks: plant, sensor, observer, controller) with the Phase E
% parameters and compares it with the script's reference simulation run_closed_loop() for the same initial condition.
mdl = 'acoustic_pursuit_loop';  root = fileparts(mfilename('fullpath'));
load_system(fullfile(root, [mdl '.slx']));  cl = onCleanup(@() close_system(mdl, 0));
x0 = [P.d + 0.5; d2r_(10); d2r_(5)];  wt = [P.v0; 0];  T = 40;
Pq = P;  Pq.sn = 1e-9;  Pq.fringe = 0;  Pq.room_sd = 0;  Pq.refl_p = 0;               % noise-free implementation check
ref = run_closed_loop(Pq, D, x0, @(t) wt, T);
sim_noise_free = run_mdl(mdl, Pq, D, x0, wt, T);
Pn = P;  Pn.fringe = P.mp_horn12(1);  Pn.room_sd = P.mp_horn12(2);                   % measured noise + horn12-target multipath
sim_noisy = run_mdl(mdl, Pn, D, x0, wt, T);
xr = interp1(ref.t, ref.X.', sim_noise_free.t);
res = struct('max_range_diff_m', max(abs(xr(:,1) - sim_noise_free.X(:,1))), 'max_bearing_diff_deg', r2d_(max(abs(xr(:,2) - sim_noise_free.X(:,2)))));
fprintf('  noise-free: Simulink vs reference simulation: max |range diff| %.4f m, max |bearing diff| %.3f deg over %g s\n', ...
        res.max_range_diff_m, res.max_bearing_diff_deg, T);
yss = mean(sim_noisy.X(sim_noisy.t > T-10, 1)) - P.d;  brms = r2d_(sqrt(mean(sim_noisy.X(sim_noisy.t > T-10, 2).^2)));
fprintf('  with measured noise + horn12-target multipath: last-10-s range error %+.3f m, bearing RMS %.1f deg\n', yss, brms);
res.noisy_range_error_m = yss;  res.noisy_bearing_rms_deg = brms;
cols = {[42 120 214]/255, [235 104 52]/255, [27 175 122]/255};
f = figure('Visible','off', 'Color',[252 252 251]/255, 'Position',[100 100 900 380]);
tl = tiledlayout(f, 1, 2, 'TileSpacing','compact');
for k = 1:2
    nexttile(tl); hold on; grid on; box off;  sc = 1;  if k == 2, sc = 180/pi; end
    plot(ref.t, sc*ref.X(k,:), '-', 'Color', cols{1}, 'LineWidth', 2.2);
    plot(sim_noise_free.t, sc*sim_noise_free.X(:,k), '--', 'Color', cols{2}, 'LineWidth', 1.6);
    plot(sim_noisy.t, sc*sim_noisy.X(:,k), '-', 'Color', cols{3}, 'LineWidth', 1.0);
    if k == 1, yline(P.d, ':', 'Color', [.54 .54 .52]); ylabel('true range [m]'); else, ylabel('true bearing [deg]'); end
    xlabel('t [s]');
end
lg = legend({'script reference (noise-free)', 'Simulink (noise-free)', 'Simulink, measured noise + horn12-target multipath'}, 'Orientation','horizontal', 'Box','off');
lg.Layout.Tile = 'south';
title(tl, sprintf('M2  Simulink loop reproduces the reference design (max diff %.3f m, %.2f deg)', res.max_range_diff_m, res.max_bearing_diff_deg), 'FontSize', 10, 'FontWeight', 'bold');
exportgraphics(f, fullfile(P.out_dir, 'm2_simulink_check.png'), 'Resolution', 160);  close(f);
end

function out = run_mdl(mdl, P, D, x0, wt, T)
Kd = D.Kd;
Pc = [reshape(Kd(:,1:3), [], 1); Kd(:,4); D.Ad4(:); D.Bd4(:); P.v0; P.v_max; P.ds_max; P.L; P.zmax; P.phi_max; P.zband; ...
      double(D.integral); P.Ts; P.t_lost; P.rho_stop; P.d];
Po = [D.Phi(:); D.Gam(:); D.Co(:); D.L(:); D.PhiTd(:); D.S(:); min(D.gate_chi2, 1e9); D.gate_zone(:)];
Ps = [P.K; P.Kerr; P.sig; P.dlt; P.sn; P.A_thr; P.d; P.refl_p; P.refl_g; P.fringe; P.room_sd; P.room_tau; P.Tm];
w = get_param(mdl, 'ModelWorkspace');
vals = {'Pc',Pc; 'Po',Po; 'Ps',Ps; 'Td',P.Td; 'Td_obs',P.Td_obs; 'Tdw',P.Tdw; 'Tm',P.Tm; 'Ts',P.Ts; 'tau_v',P.tau_v; ...
        'tau_w',P.tau_w; 'v0',P.v0; 'wt',wt; 'x0',x0};
for i = 1:size(vals,1), w.assignin(vals{i,1}, vals{i,2}); end
so = sim(mdl, 'StopTime', num2str(T));
out = struct('t', so.tout, 'X', squeeze(so.Xsim).');
end

%% =====================================================================================================================
%% STRUCTURAL ANALYSIS (class method)
%% =====================================================================================================================
function info = report_structure(name, A, B, C)
% Kalman rank tests, Kalman decomposition, stabilizability / detectability.
n = size(A,1);  tol = 1e-9;
rc = rank(ctrb(A,B), tol);  ro = rank(obsv(A,C), tol);
fprintf('\n  %s\n', name);
fprintf('    n = %d, inputs = %d, outputs = %d (%s).  rank(ctrb) = %d, rank(obsv) = %d -> controllable: %s, observable: %s\n', n, size(B,2), ...
        size(C,1), mimo_label(size(C,1), size(B,2)), rc, ro, tf_(rc==n), tf_(ro==n));
lam = unique(round(eig(A)*1e6)/1e6);
for i = 1:numel(lam)
    pc = rank([lam(i)*eye(n) - A, B], tol);  po = rank([lam(i)*eye(n) - A; C], tol);
    fprintf('    At lambda = %+.3g: rank[lI-A, B] = %d (%s), rank[lI-A; C] = %d (%s)\n', real(lam(i)), pc, ...
            ifelse_(pc==n, 'controllable mode', 'UNCONTROLLABLE mode'), po, ifelse_(po==n, 'observable mode', 'UNOBSERVABLE mode'));
end
Kd_ = kalman_decomp(A, B, C);
lab = {'controllable & observable (co)', 'controllable & unobservable (c,no)', 'uncontrollable & observable (nc,o)', ...
       'uncontrollable & unobservable (nc,no)'};
fprintf('    Kalman decomposition z = T^-1 x, ordered [co; c-no; nc-o; nc-no]; structure check (forbidden blocks) = %.1e\n', Kd_.check);
ev_unc = [];  ev_uno = [];
for k = 1:4
    ix = Kd_.idx{k};
    if isempty(ix), continue; end
    ek = eig(Kd_.Ahat(ix,ix));
    fprintf('      %-40s dim %d, eig = %s\n', lab{k}, numel(ix), mat2str(round(ek.'*1e4)/1e4));
    if k >= 3, ev_unc = [ev_unc; ek]; end %#ok<AGROW>
    if k == 2 || k == 4, ev_uno = [ev_uno; ek]; end %#ok<AGROW>
end
stab = all(real(ev_unc) < -tol);  detc = all(real(ev_uno) < -tol);
fprintf('    stabilizable: %s%s   detectable: %s%s\n', tf_(stab), marg_note(ev_unc, tol), tf_(detc), marg_note(ev_uno, tol));
info = struct('rank_c',rc, 'rank_o',ro, 'dims',Kd_.dims, 'stabilizable',stab, 'detectable',detc, 'decomp',Kd_);
end

function K = kalman_decomp(A, B, C)
% Kalman canonical form from orthonormal bases: Vc = controllable subspace, Vno = unobservable subspace.
n = size(A,1);  tol = 1e-9;
Vc  = orth_(ctrb(A,B), tol);
Vno = null_(obsv(A,C), tol);
if isempty(Vc) || isempty(Vno)
    Xcno = zeros(n,0);
else
    N = null_([Vc, -Vno], tol);                                    % Vc*a = Vno*b  -> intersection
    if isempty(N), Xcno = zeros(n,0); else, Xcno = orth_(Vc*N(1:size(Vc,2),:), tol); end
end
Xco   = comp_(Vc,  Xcno, tol);                                     % controllable and observable
Xcbno = comp_(Vno, Xcno, tol);                                     % uncontrollable and unobservable
Q = [Xco Xcno Xcbno];
if isempty(Q), Xcbo = eye(n); else, Xcbo = null_(Q', tol); end     % uncontrollable and observable (complement)
T = [Xco Xcno Xcbo Xcbno];
dims = [size(Xco,2) size(Xcno,2) size(Xcbo,2) size(Xcbno,2)];
Ah = T\(A*T);  Bh = T\B;  Ch = C*T;
e = cumsum([0 dims]);  idx = {e(1)+1:e(2), e(2)+1:e(3), e(3)+1:e(4), e(4)+1:e(5)};
co = idx{1}; cno = idx{2}; nco = idx{3}; ncno = idx{4};
chk = max([0, max(abs(reshape(Ah([nco ncno], [co cno]),1,[]))), max(abs(reshape(Ah([co nco], [cno ncno]),1,[]))), ...
              max(abs(reshape(Bh([nco ncno],:),1,[]))), max(abs(reshape(Ch(:,[cno ncno]),1,[])))]);
K = struct('T',T, 'Ahat',Ah, 'Bhat',Bh, 'Chat',Ch, 'dims',dims, 'idx',{idx}, 'check',chk);
end

function X = orth_(M, tol)                                         % orthonormal basis of range(M), absolute tolerance
if isempty(M), X = zeros(size(M,1),0); return; end
s = svd(M);  [U,~,~] = svd(M);  X = U(:, 1:sum(s > tol));
end
function X = null_(M, tol)                                         % orthonormal basis of null(M), absolute tolerance
s = svd(M);  [~,~,V] = svd(M);  X = V(:, sum(s > tol)+1:end);
end
function X = comp_(V, W, tol)                                      % part of span(V) orthogonal to span(W)
if isempty(V), X = zeros(size(V,1),0); return; end
if isempty(W), X = orth_(V, tol); return; end
X = orth_(V - W*(W'*V), tol);
end
function s = tf_(b), yn = {'no','yes'}; s = yn{b+1}; end
function y = d2r_(x), y = x*pi/180; end
function y = r2d_(x), y = x*180/pi; end
function s = ifelse_(c, a, b), if c, s = a; else, s = b; end, end
function s = mimo_label(q, m), if q == 1 && m == 1, s = 'SISO'; elseif q == 1 || m == 1, s = 'SIMO/MISO'; else, s = sprintf('MIMO %dx%d', q, m); end, end
function s = marg_note(ev, tol), if any(abs(real(ev)) < tol), s = ' (marginal modes present)'; else, s = ''; end, end

%% =====================================================================================================================
%% COURSE-OBJECTIVE CHECKS: realizations, analytic trajectories, LTV transition matrix, MIMO placement
%% =====================================================================================================================
function realization_report(a)
% Non-uniqueness of the state model (objective 2): bearing channel w_f -> beta, G(s) = -(s + a)/s^2, in four realizations.
Ab = [0 a; 0 0];  Bb = [-1; -1];  Cb = [1 0];
Ti = canon_T(Ab, Bb);                                  % z = Ti*x: controller (phase-variable) canonical form
Ac = Ti*Ab/Ti;  Bc = Ti*Bb;  Cc = Cb/Ti;
To = canon_T(Ab', Cb');                                % duality: observer canonical form
Ao_ = (To*Ab'/To)';  Bo_ = (Bb'/To)';  Co_ = (To*Cb')';
Tr = [1 0.5; -0.3 2];                                  % arbitrary invertible T
Ar = Tr*Ab/Tr;  Br = Tr*Bb;  Cr = Cb/Tr;
Rz = {'original [beta; eps]', Ab, Bb, Cb; 'controller canonical', Ac, Bc, Cc; 'observer canonical', Ao_, Bo_, Co_; 'random similarity T', Ar, Br, Cr};
ws = [0.1 0.5 2 10];  G0 = tfresp_(Ab, Bb, Cb, ws);  Ga = -(1j*ws + a)./(1j*ws).^2;
r3 = @(M) mat2str(round(M*1e3)/1e3);
fprintf('  Bearing channel G(s) = -(s + a)/s^2, a = %.3g (numeric check %.1e). x -> T x gives (T A T^-1, T B, C T^-1): same G(s), different state model.\n', a, max(abs(G0 - Ga)));
for i = 1:size(Rz,1)
    A_ = Rz{i,2};  B_ = Rz{i,3};  C_ = Rz{i,4};
    fprintf('   %-22s A = %-26s B = %-14s C = %s\n', Rz{i,1}, r3(A_), r3(B_), r3(C_));
    fprintf('   %-22s rank(ctrb) %d, rank(obsv) %d, eig(A) = %s, max|G - G0| = %.1e\n', '', rank(ctrb(A_,B_)), rank(obsv(A_,C_)), ...
            mat2str(round(eig(A_).'*1e4)/1e4), max(abs(tfresp_(A_, B_, C_, ws) - G0)));
end
fprintf('  Invariants under T: G(s), eig(A), ranks of ctrb / obsv. Not invariant: A, B, C entries, ctrb / obsv themselves, Gramians.\n');
end
function Ti = canon_T(A, B)                            % Ti = [q; qA; ...; qA^(n-1)], q = last row of inv(ctrb(A,B))
n = size(A,1);  q = [zeros(1,n-1) 1]/ctrb(A,B);  Ti = zeros(n);
for i = 1:n, Ti(i,:) = q*A^(i-1); end
end
function g = tfresp_(A, B, C, ws)
g = zeros(size(ws));
for i = 1:numel(ws), g(i) = C*((1j*ws(i)*eye(size(A,1)) - A)\B); end
end

function analytic_checks(A, B, E, P)
% (objective 4) LTI with nonzero initial condition and constant forcing. A^3 = 0, so everything is a finite series:
%   x(t) = e^{At} x0 + (int_0^t e^{As} ds)(B u + E w),  e^{At} = I + At + A^2 t^2/2,  int_0^t e^{As} ds = I t + A t^2/2 + A^2 t^3/6
x0 = [0.3; d2r_(10); d2r_(5)];  u = [0.05; 0.02];  w = [0.04; 0.03];  tf = 8;  f = B*u + E*w;
xc = @(t) (eye(3) + A*t + A^2*t^2/2)*x0 + (eye(3)*t + A*t^2/2 + A^2*t^3/6)*f;
opt = odeset('RelTol',1e-10, 'AbsTol',1e-12);
[tt, xx] = ode45(@(t,x) A*x + f, [0 tf], x0, opt);
XC = zeros(3, numel(tt));  for i = 1:numel(tt), XC(:,i) = xc(tt(i)); end
fprintf('  LTI: |A^3| = %.1e -> e^{At} = I + At + A^2 t^2/2 exactly (|expm - series| = %.1e at t = 2).\n', norm(A^3), norm(expm(2*A) - (eye(3) + 2*A + 2*A^2)));
fprintf('       closed-form x(t) vs numerical integration (x0 = [0.3 m, 10 deg, 5 deg], u, w constant, t <= %g s): max error %.1e\n', tf, max(max(abs(xx.' - XC))));
% (objectives 3-5) LTV: speed ramp v(t) = v0 + vd*t gives a(t) = v(t)/d*. A(t1) A(t2) = 0 for all t1, t2, so the Peano-Baker series
% stops after its first term:  Phi(t,t0) = I + int_{t0}^t A(s) ds = I + alpha(t,t0) e2 e3',  alpha = (v0 (t-t0) + vd (t^2 - t0^2)/2)/d*
vd = 0.05;  t0 = 1;  t1 = 6;
al = @(t,s) (P.v0*(t - s) + vd*(t.^2 - s.^2)/2)/P.d;
Ph = @(t,s) eye(3) + al(t,s)*[0 0 0; 0 0 1; 0 0 0];
At = @(t) [0 0 0; 0 0 (P.v0 + vd*t)/P.d; 0 0 0];
[~, Yv] = ode45(@(t,y) reshape(At(t)*reshape(y,3,3), 9, 1), [t0 t1], reshape(eye(3), 9, 1), opt);
e1 = max(abs(Yv(end,:).' - reshape(Ph(t1,t0), [], 1)));
e2 = norm(Ph(t1,3)*Ph(3,t0) - Ph(t1,t0));
xv = Ph(t1,t0)*x0 + integral(@(s) Ph(t1,s)*f, t0, t1, 'ArrayValued', true);
[~, Xn] = ode45(@(t,x) At(t)*x + f, [t0 t1], x0, opt);
fprintf('  LTV (v(t) = v0 + %.2f t): |A(1)A(2)| = %.1e -> Phi(t,t0) = I + alpha(t,t0) e2 e3''. Checks on [%g, %g] s:\n', vd, norm(At(1)*At(2)), t0, t1);
fprintf('       |Phi analytic - Phi from the matrix ODE| = %.1e;  group property |Phi(t1,3)Phi(3,t0) - Phi(t1,t0)| = %.1e;\n', e1, e2);
fprintf('       forced trajectory (variation of constants) vs ode45: %.1e.  (If A(t1) and A(t2) did not commute, the full series would be needed.)\n', max(abs(Xn(end,:).' - xv)));
end

function mimo_placement_demo(A, B, C, P)
% (objective 11) (i) MIMO state feedback is not unique: per-channel placement vs full-MIMO place() with the same closed-loop poles.
%                (ii) partially controllable case (v0 = 0, a = 0): place the controllable poles in the Kalman coordinates; the rest stays.
[pr, pb] = pole_set(P, 0);  pd = [pr; pb];
Ks = assemble_K(place(0, -1, pr), place([0 A(2,3); 0 0], [-1; -1], pb), 0);
Km = place(A, B, pd);
fprintf('  (i) same poles %s, two different gains (rows dv_f, w_f; cols d_rho, beta, eps):\n', mat2str(round(pd.'*1e3)/1e3));
for k = 1:2
    if k == 1, Kk = Ks;  nm_ = 'per-channel (decoupled)'; else, Kk = Km;  nm_ = 'full-MIMO place()        '; end
    [V, ~] = eig(A - B*Kk);
    fprintf('      %s K = %s   |K|_F = %.2f, cond(V) = %.1f, eig = %s\n', nm_, mat2str(round(Kk*1e3)/1e3), norm(Kk,'fro'), cond(V), ...
            mat2str(round(sort(real(eig(A - B*Kk))).'*1e3)/1e3));
end
fprintf('      -> with m = 2 inputs K is not unique: the poles can be assigned to different channels and the eigenvectors are free. place() chose its own\n');
fprintf('         channel assignment and eigenvectors; the per-channel design fixes both (range gets %.3g, bearing gets %s) and keeps the loops readable.\n', pr, mat2str(round(pb.'*1e3)/1e3));
A0 = zeros(3);  kd = kalman_decomp(A0, B, C);
ic = [kd.idx{1}, kd.idx{2}];  nc = numel(ic);
pcl = -P.pp.wn_r*[1 1.6 2.2];  pcl = pcl(1:nc);
Khat = zeros(2,3);  Khat(:,ic) = place(kd.Ahat(ic,ic), kd.Bhat(ic,:), pcl);
K0 = Khat/kd.T;                                         % u = -Khat*z = -Khat*inv(T)*x
fprintf('  (ii) v0 = 0: controllable subspace has dimension %d of 3 (Kalman coordinates, dims [co c-no nc-o nc-no] = %s).\n', nc, mat2str(kd.dims));
fprintf('       Placed %s on it; closed-loop eig(A - B K) = %s -> the uncontrollable mode (eps, a = 0) stays at 0:\n', ...
        mat2str(pcl), mat2str(round(sort(real(eig(A0 - B*K0))).'*1e3)/1e3));
fprintf('       not stabilizable, which is why the controller needs v0 > 0.\n');
end

%% =====================================================================================================================
%% SYNTHESIS HELPERS
%% =====================================================================================================================
function [pr, pb] = pole_set(P, nz)
% Continuous-time poles for 'place': range channel (2 poles with integrator, 1 without), bearing channel (2 poles).
% Phase E: taken from Config/phase_e.json (range_poles, bearing_poles), the same as the Python runtime.
if nz, pr = P.pp.range_poles; else, pr = P.pp.range_poles(1); end
pb = P.pp.bearing_poles;
pr = pr(:);  pb = pb(:);
end
function K = assemble_K(Kr, Kb, nz)
% Block-diagonal gain, columns [d_rho beta eps (z)], rows [dv_f w_f].
if nz, K = [Kr(1) 0 0 Kr(2); 0 Kb(1) Kb(2) 0]; else, K = [Kr(1) 0 0; 0 Kb(1) Kb(2)]; end
end
function [Ad, Bd] = zoh_mat(A, B, dt)
n = size(A,1);  m = size(B,2);
M = expm([A B; zeros(m, n+m)]*dt);
Ad = M(1:n,1:n);  Bd = M(1:n,n+1:n+m);
end
function [num, den] = pade_tf(T, n)                                % n-th order Pade of exp(-sT), descending powers of s
c = zeros(1, n+1);
for k = 0:n, c(k+1) = factorial(2*n-k)*factorial(n)/(factorial(2*n)*factorial(k)*factorial(n-k)); end
num = zeros(1,n+1);  den = num;
for k = 0:n, num(n+1-k) = c(k+1)*(-T)^k;  den(n+1-k) = c(k+1)*T^k; end
end
function W = gram_fh(A, B, T)                                      % finite-horizon Gramian; does not require stable A
t = linspace(0, T, 300);  N = size(A,1);  Wk = zeros(N,N,numel(t));
for k = 1:numel(t), M = expm(A*t(k))*B;  Wk(:,:,k) = M*M'; end
W = zeros(N);
for i = 1:N, for j = 1:N, W(i,j) = trapz(t, squeeze(Wk(i,j,:))); end, end
end
function J = jac_fd(g, z0)
h = 1e-6;  n = numel(z0);  f0 = g(z0);  J = zeros(numel(f0), n);
for i = 1:n, dz = zeros(n,1);  dz(i) = h;  J(:,i) = (g(z0+dz) - g(z0-dz))/(2*h); end
end
function P = with_field_(P, name, value), P.(name) = value; end
function v = getopt(opt, name, default)
if isfield(opt, name), v = opt.(name); else, v = default; end
end
function hdr(s), fprintf('\n===== %s =====\n', s); end

%% =====================================================================================================================
%% PLANT, SENSOR, OBSERVER, CONTROLLER (used by the closed-loop simulation; same logic as the Raspberry Pi code)
%% =====================================================================================================================
function xdot = f_nl(x, vf, wf, vt, wt)                            % nonlinear relative kinematics, Eq. (nl)
rho = max(x(1), 1e-2);  beta = x(2);  al = x(3) - beta;
xdot = [vt*cos(al) - vf*cos(beta);  (vt*sin(al) + vf*sin(beta))/rho - wf;  wt - wf];
end
function x = rk4_step(x, u, w, dt)
f = @(s) f_nl(s, u(1), u(2), w(1), w(2));
k1 = f(x);  k2 = f(x + dt/2*k1);  k3 = f(x + dt/2*k2);  k4 = f(x + dt*k3);
x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

function [s, y, valid, cnt] = sensor_pair(s, x, xd, phic, P)
% Paired acoustic measurement: x = true state at the A+ capture, xd = true state at the A- capture (Tdw earlier).
% Multipath (Phase B): fast fringe = white ln-amplitude noise per capture (P.fringe, inside amp_); room bias = AR(1) bearing
% bias b with sd P.room_sd and correlation time P.room_tau = room_length/v0, applied as A+/- * exp(+-b/(2h)).
if P.room_sd > 0
    c = exp(-P.Tm/P.room_tau);
    if isnan(s.room), s.room = P.room_sd*randn; else, s.room = c*s.room + sqrt(1 - c^2)*P.room_sd*randn; end
else
    s.room = 0;
end
h = P.sig^2/(2*P.dlt);
Am = amp_(P.K*P.Kerr, xd(1), xd(2), phic - P.dlt, P)*exp(-s.room/(2*h));
Ap = amp_(P.K*P.Kerr, x(1),  x(2),  phic + P.dlt, P)*exp(s.room/(2*h));
b = phic + P.sig^2/(2*P.dlt)*log(Ap/Am);                                      % Eq. (beta)
r = P.K/sqrt(Ap*Am)*exp(-((phic - b)^2 + P.dlt^2)/(2*P.sig^2));               % Eq. (rho)
y = [r - P.d; b];  valid = double(min(Am, Ap) > P.A_thr);
s.n = s.n + 1;  cnt = s.n;
end
function A = amp_(Kt, rho, beta, phi, P)
a0 = Kt/max(rho,1e-2)*exp(-(phi - beta)^2/(2*P.sig^2));                       % Eq. (amp)
if P.fringe > 0, a0 = a0*exp(P.fringe*randn); end                              % fast multipath fringe (Phase B)
ec = complex(0,0);                                                            % optional reflection: an echo of relative amplitude refl_g, random phase
if P.refl_p > 0 && rand < P.refl_p, ec = P.refl_g*a0*exp(1j*2*pi*rand); end
A = abs(complex(a0 + P.sn*randn, P.sn*randn) + ec);                           % Rician magnitude (Goertzel output)
end

function [obs, xh] = observer_step(obs, uprev, ym, valid, cnt, xhd, D)
% Fixed-gain observer with delayed, intermittent measurements: predict every Ts; when a new valid sample arrives,
% x += Phi(Td)*L*(ym - Co*xhd), xhd = own estimate Td ago (exact replay for an LTI model, Td <= Tm).
% Innovation gate (soft): if nu'*inv(S)*nu > gate_chi2 the innovation is scaled back onto the gate boundary (Huber-type clipping), so a reflection
% has bounded influence but the sample is never thrown away (a hard reject starves the observer, see the report). Active only near lock (gate_zone).
% obs.age = control steps since the last used sample (feeds the watchdog).
nx = numel(obs.x);
x = D.Phi*obs.x + D.Gam*uprev;
obs.age = obs.age + 1;
if cnt ~= obs.cprev
    obs.cprev = cnt;
    if valid > 0.5
        if ~obs.init
            x = D.PhiTd*[ym; zeros(nx-2,1)];  obs.init = true;  obs.age = 0;          % first valid sample initialises [d_rho; beta]
        else
            nu = ym - D.Co*xhd;
            d2 = nu'*(D.S\nu);
            if abs(x(1)) < D.gate_zone(1) && abs(x(2)) < D.gate_zone(2) && d2 > D.gate_chi2     % only near lock
                nu = nu*sqrt(D.gate_chi2/d2);  obs.ntot = obs.ntot + 1;             % Huber-type clipping: bounded influence, data never discarded
            end
            x = x + D.PhiTd*(D.L*nu);  obs.age = 0;
        end
    end
end
obs.x = x;  xh = x;
end

function [ctl, vcmd, wcmd, ua, phic, stopped] = controller_step(ctl, xh, D, P, age)
% State feedback + integral action, command limits (O4), integrator active only if unsaturated and |d_rho_hat| < zband.
% Hard stop: v_f = 0, w_f = 0 if no accepted measurement for t_lost, or if the estimated range is below rho_stop.
stopped = (age*P.Ts > P.t_lost) || (xh(1) + P.d < P.rho_stop);
u = -D.Kd(:,1:3)*xh(1:3) - D.Kd(:,4)*ctl.z;
vcmd = min(max(P.v0 + u(1), 0), P.v_max);
if stopped, vcmd = 0; end
wlim = max(vcmd, 0.05)*tan(P.ds_max)/P.L;                                         % w = v tan(ds)/L
wcmd = min(max(u(2), -wlim), wlim);
if stopped, wcmd = 0; end
ua = [vcmd - P.v0; wcmd];
if D.integral && ~stopped && vcmd == P.v0 + u(1) && abs(xh(1)) < P.zband
    ctl.z = min(max(D.Ad4*xh(1:3) + ctl.z + D.Bd4*ua, -P.zmax), P.zmax);
end
phic = min(max(xh(2), -P.phi_max), P.phi_max);
end

function out = run_closed_loop(P, D, x0, wfun, T)
% Closed loop: nonlinear plant (RK4) + actuator lags + paired sensor + delay + observer + controller.
% Burst timing: a capture is attempted every Tm (+- Tm_jitter), is lost with probability p_drop, and never happens inside P.outage = [t0 t1].
Ts = P.Ts;  nm = round(P.Tm/Ts);  ndw = round(P.Tdw/Ts);  nd = round(P.Td/Ts);  ndo = round(P.Td_obs/Ts);  n = round(T/Ts);  nx = D.nx;
X = zeros(3,n+1);  XH = zeros(nx,n+1);  U = zeros(2,n);  X(:,1) = x0(:);  STP = false(1,n);
obs = struct('x',zeros(nx,1), 'cprev',0, 'init',false, 'ntot',0, 'age',0);  ctl = struct('z',0);  snsr = struct('n',0, 'room',NaN);
va = P.v0;  wa = 0;  uprev = [0; 0];  ym = [0; 0];  valid = 0;  cnt = 0;  knext = 0;
queue = zeros(4, n+nd+2);  qa = false(1, n+nd+2);  meas = zeros(3,0);
ev = exp(-Ts/P.tau_v);  ew = exp(-Ts/P.tau_w);
for k = 0:n-1
    if qa(k+1), ym = queue(1:2,k+1);  valid = queue(3,k+1);  cnt = queue(4,k+1); end
    if D.ideal
        xh = [X(1,k+1) - P.d; X(2:3,k+1); zeros(nx-3,1)];                           % cheat: true state
        obs.age = 0;
    else
        xhd = XH(:, max(k-ndo,0)+1);                                                % own estimate at the pair midpoint
        [obs, xh] = observer_step(obs, uprev, ym, valid, cnt, xhd, D);
    end
    XH(:,k+1) = xh;
    [ctl, vc, wc, ua, phic, sp] = controller_step(ctl, xh, D, P, obs.age);   U(:,k+1) = [vc; wc];  uprev = ua;  STP(k+1) = sp;
    if k == knext                                                                   % acoustic pair completes now
        knext = k + nm;
        if P.Tm_jitter > 0, knext = k + max(1, round(nm*(1 + P.Tm_jitter*(2*rand - 1)))); end
        lost = (P.p_drop > 0 && rand < P.p_drop) || (numel(P.outage) == 2 && k*Ts >= P.outage(1) && k*Ts < P.outage(2));
        if ~lost
            xd = X(:, max(k-ndw,0)+1);
            [snsr, y, vl, c] = sensor_pair(snsr, X(:,k+1), xd, phic, P);
            queue(:,k+nd+1) = [y; vl; c];  qa(k+nd+1) = true;
            if vl > 0.5, meas(:,end+1) = [k*Ts; y(1)+P.d; y(2)]; end %#ok<AGROW>
        end
    end
    va = ev*va + (1-ev)*vc;  wa = ew*wa + (1-ew)*wc;                                % exact first-order actuator lags
    X(:,k+2) = rk4_step(X(:,k+1), [va; wa], wfun(k*Ts), Ts);
end
out = struct('t',(0:n)*Ts, 'X',X, 'XH',XH, 'U',U, 'meas',meas, 'n_rej',obs.ntot, 'n_valid',size(meas,2), 't_stop',sum(STP)*Ts);
end

function m = loop_metrics(out, P)
% 2% settling of the true range error (band >= 3 cm), overshoot (only if |initial error| >= 0.25 m), RMS bearing.
t = out.t;  y = out.X(1,:) - P.d;
band = max(0.02*abs(y(1)), 0.03);
idx = find(abs(y) > band);
if isempty(idx), ts = 0; elseif idx(end) < numel(t), ts = t(idx(end)+1); else, ts = Inf; end
if abs(y(1)) < 0.25, ov = NaN; else, ov = max(0, max(-sign(y(1))*y))/abs(y(1))*100; end
tail = t > t(end) - 10;
m = struct('ts',ts, 'ov',ov, 'rms_beta_deg',r2d_(sqrt(mean(out.X(2,tail).^2))), 'rho_min',min(out.X(1,:)), 'yss',mean(y(tail)));
end

function plot_run(out, P)
figure;
subplot(3,1,1); plot(out.t, out.X(1,:), out.t, out.XH(1,:)+P.d, '--', out.meas(1,:), out.meas(2,:), '.'); ylabel('range [m]'); legend('true','estimate','raw');
subplot(3,1,2); plot(out.t, r2d_(out.X(2,:)), out.t, r2d_(out.XH(2,:)), '--'); ylabel('bearing [deg]');
subplot(3,1,3); plot(out.t(1:end-1), out.U(1,:), out.t(1:end-1), out.U(2,:)); ylabel('v_f [m/s], \omega_f [rad/s]'); xlabel('t [s]');
end
