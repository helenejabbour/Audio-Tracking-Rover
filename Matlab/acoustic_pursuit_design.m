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
%   2. Controllability / observability: Kalman rank tests, PBH tests, state transformation to the Kalman canonical form
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
if exist('OCTAVE_VERSION','builtin'), pkg load control; end
P = user_settings();
if isfield(opt,'P'), fn = fieldnames(opt.P); for i = 1:numel(fn), P.(fn{i}) = opt.P.(fn{i}); end, end
ctype = getopt(opt,'controller','lqr');   otype = getopt(opt,'observer','kalman');   dist = getopt(opt,'dist_states',true);
Ts = P.Ts;  Tm = P.Tm;  Td = P.Td;
yn = {'no','yes'};
if abs(Tm/Ts-round(Tm/Ts))>1e-9 || abs(Td/Ts-round(Td/Ts))>1e-9 || abs(P.Tdw/Ts-round(P.Tdw/Ts))>1e-9
    warning('Tm, Td and Tdw should be multiples of Ts (they are rounded in the simulation)');
end
if Td > Tm, warning('Td > Tm: the delay-compensated observer is no longer exact (several samples in flight)'); end

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
fprintf('  => MIMO (%d x %d). It is also sampled-data (control Ts = %.2f s, measurements every Tm = %.2f s) and delayed (Td = %.2f s).\n', q, m, Ts, Tm, Td);
fprintf('  Poles of A: '); fprintf('%g ', eig(A)); fprintf('  -> all at the origin: marginally stable, NOT asymptotically stable (A is nilpotent).\n');
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
hdr('2. Structural analysis: Kalman rank tests, PBH tests, Kalman canonical form');
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
Phim = expm(Ao*Tm);  PhiTd = expm(Ao*Td);
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

hdr('4c. Robustness O5: state-feedback loop at the plant input with actuator lags and the measured delay (Pade)');
% Assumes the delay-compensated observer delivers the current state, but still counts Td in the loop (conservative).
Gact = ss(diag([-1/P.tau_v -1/P.tau_w]), diag([1/P.tau_v 1/P.tau_w]), eye(2), zeros(2,2));
[nn, dd] = pade_tf(Td, P.pade_order);  Dl = ss(tf(nn, dd));
nl = size(Aloop,1);
Lsf = Kc*ss(Aloop, Bloop, eye(nl), zeros(nl,2))*Gact*append(Dl, Dl);
chname = {'range', 'bearing'};  verdict = {'FAIL', 'PASS'};
for ch = 1:2
    if norm(Kc(ch,:)) < eps
        MG.(chname{ch}) = [NaN NaN NaN];  fprintf('  %-8s no feedback on this channel (open loop)\n', chname{ch});  continue;
    end
    [Gm, Pm, ~, Wp] = margin(minreal(Lsf(ch,ch), 1e-6));          % minreal: drop cancelling integrator states (else margin() can return PM = 0)
    ok = (Pm >= P.PM_min) && (20*log10(Gm) >= P.GM_min_dB);
    MG.(chname{ch}) = [Pm, 20*log10(Gm), Wp];
    fprintf('  %-8s PM %5.1f deg @ %.2f rad/s (extra delay tolerated %.2f s), GM %5.1f dB -> %s (targets PM >= %g, GM >= %g dB)\n', ...
            chname{ch}, Pm, Wp, Pm*pi/180/Wp, 20*log10(Gm), verdict{ok+1}, P.PM_min, P.GM_min_dB);
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
fprintf('  open loop: eig(A) = 0 (nilpotent A) -> no P > 0 solves A''P + PA = -Q: marginally stable, not asymptotically; G(s) has poles at s = 0 -> not BIBO stable.\n');
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
    base = P.S1.base;  nS = round(P.S1.T/Ts);
    for s = P.S1.scales
        xn = base*s;  zn = 0;  xa = [base*s; 0];  XN = zeros(3,nS);  XL = XN;
        for k = 1:nS
            un = -Kd(:,1:3)*xn - Kd(:,4)*zn;
            zn = zn + Ad(4,1:3)*xn + Bd(4,:)*un;
            xn = rk4_step(x0n + xn, [P.v0 + un(1); un(2)], [P.v0; 0], Ts) - x0n;
            xa = (Ad - Bd*Kd)*xa;  XN(:,k) = xn;  XL(:,k) = xa(1:3);
        end
        nr = sqrt(mean((XN-XL).^2, 2))./sqrt(mean(XL.^2, 2));
        fprintf('    scale %.2f (d_rho0 %.2f m, beta0 %.0f deg, eps0 %.0f deg): d_rho %.3f  beta %.3f  eps %.3f\n', s, base(1)*s, ...
                r2d_(base(2)*s), r2d_(base(3)*s), nr);
    end
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
           'stress: reflections 15%, drops 20%, period jitter +-20%, speed offset (gate ON)', 'same stress, gate OFF'};
    names = {'2% settling [s]', 'overshoot [%] (|d_rho0|>=0.25 m)', 'RMS beta, last 10 s [deg]', 'steady-state range error [m]', 'min range [m]'};
    rng(P.seed);
    for sc = 1:4
        Ps_ = P;  Dg = D;
        if sc >= 3
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
    fid = fopen('gains.json','w');  fwrite(fid, jsonencode(G));  fclose(fid);
    fprintf('\nsaved gains.json (controller and observer matrices for the Raspberry Pi)\n');
end
end

%% =====================================================================================================================
%% USER SETTINGS  -- every hard-coded number of the design lives here
%% =====================================================================================================================
function P = user_settings()
% ---- (A) MEASURED / PLATFORM VALUES: replace the placeholders (*) with your calibration results -----------------------
P.v0 = 0.25;  P.d = 0.1;                      % nominal speed [m/s], desired separation d* [m]                (*)
P.L = 0.145;  P.ds_max = d2r_(30);         % wheelbase [m], steering limit [rad]                           (*)
P.v_max = 0.5;  P.tau_v = 0.20;  P.tau_w = 0.15;   % speed limit [m/s], speed / yaw actuator lags [s]          (*)
P.K = 1.0;  P.Kerr = 1.0;                     % K [counts*m] = A_geo_x_rho*exp(dlt^2/(2 sig^2)) (*); Kerr = K_true/K_model
P.sig = d2r_(25);  P.dlt = d2r_(25);    % horn Gaussian sigma (*) and firmware dither delta [rad]
P.sn = 0.01;  P.A_thr = 0.05;                 % amplitude noise std [counts] (speaker-off run), detection threshold [counts]   (*)
P.phi_max = d2r_(80);                      % servo limit [rad]
P.sr_floor = 0.03;  P.sb_floor = d2r_(2);  % model-error floors on rho_hat [m], beta_hat [rad]; raise for reflections  <== EXPERIMENT
P.Ts = 0.05;                                  % control period [s]
P.Tm = 0.40;                                  % measurement period [s] (logger T_m_median_s)                  (*)
P.Tdw = 0.10;                                 % delay between the A- and A+ captures [s] (logger dt_pair_ms)  (*)
P.Td = 0.30;                                  % latency capture -> controller [s]                             (*)
% ---- (B) CONTROLLER KNOBS --------------------------------------------------------------------------------------------
P.Qd = [50 5 3 5];                           % LQR state weights [d_rho beta eps int(d_rho)] ~ 1/tolerance^2   <== EXPERIMENT
P.Rd = [100 16];                              % LQR input weights [dv_f w_f]; larger => slower, more robust       <== EXPERIMENT
P.pp.wn_r = 0.47;  P.pp.zeta_r = 1.1;         % 'place': range-loop natural frequency [rad/s], damping (avoid exactly 1)  <== EXPERIMENT
P.pp.wn_b = 0.45;  P.pp.zeta_b = 1.25;        % 'place': bearing-loop natural frequency, damping (these reproduce the LQR poles) <== EXPERIMENT
P.zband = 0.10;                               % integrate only when |d_rho_hat| < zband [m] (avoids transient windup) <== EXPERIMENT
P.zmax = 1.0;                                 % integrator clamp [m*s]
% ---- (C) OBSERVER KNOBS ----------------------------------------------------------------------------------------------
P.Qw = 10*[1e-3 1e-3 1e-3 5e-4 2e-2];         % Kalman process-noise intensity [d_rho beta eps dv_t w_t]; larger => faster  <== EXPERIMENT
P.obs_poles = -[1.0 1.25 1.5 1.75 2.0];       % 'luenberger': continuous observer poles, distinct (first nx used)    <== EXPERIMENT
% ---- (D) SPECIFICATION AND ANALYSIS ----------------------------------------------------------------------------------
P.PM_min = 45;  P.GM_min_dB = 6;              % O5 targets
P.pade_order = 3;  P.n_freq = 150;            % Pade order of the delay, frequency grid for the sensitivity peak
P.S2.a_list = [0.5 1.0 1.5 2.0 2.5 3.0];  P.S2.T = 6;            % S2 sweep of a = v0/d*, Gramian horizon [s]
P.S2.tol_x = [0.1 0.17 0.26];  P.S2.tol_u = [0.1 0.5];  P.S2.tol_y = [0.03 0.035];   % scaling to tolerance units
P.S1.base = [0.3; d2r_(20); d2r_(20)];  P.S1.scales = [0.25 0.5 1 1.5 2];  P.S1.T = 15;   % S1 offsets, horizon [s]
% ---- (E) SIMULATION --------------------------------------------------------------------------------------------------
P.T = 30;  P.seed = 1;  P.N_mc = 20;          % run length [s], random seed, Monte Carlo runs
P.x0_demo = [P.d + 1.0; d2r_(0); d2r_(0)];  P.dvt_demo = 0.05;   % demo run start and target speed offset. before 0.4, 15, -10
P.d0_range = [-0.05 0.50];  P.beta0_range_deg = [-20 20];  P.eps0_range_deg = [-15 15];   % MC start drawn uniformly in [lo hi]: d_rho0 [m], beta0, eps0 [deg]
P.dvt_test = 0.05;                            % |target speed offset| in the disturbance scenario [m/s]
% ---- (F) ROBUSTNESS LAYER: gating, watchdog / hard stop, burst-timing and reflection stress -------------------------------
P.gate = true;  P.gate_chi2 = 9.21;  P.gate_infl = 0.5;                  % innovation clipping: chi2_2(0.99) limit on nu'*inv(S)*nu, S inflation   <== EXPERIMENT
P.gate_zone_rho = 0.15;  P.gate_zone_beta = d2r_(15);   % gate only near lock (|d_rho_hat|, |beta_hat| below these): far from lock the linear prediction is poor and the gate would reject good data
P.t_lost = 6*P.Tm;  P.rho_stop = 0.03;       % watchdog: v_f = 0 if no accepted sample for t_lost [s], or if rho_hat < rho_stop [m]   (t_lost is NOT updated if you override Tm via opt.P)
P.rho_safe = 0.05;                            % spec: the TRUE range must stay above this [m] (counted in S3)
P.Tm_jitter = 0;  P.p_drop = 0;  P.outage = [];   % burst timing: +- fractional jitter of Tm, per-capture loss probability, outage window [t0 t1] s (no captures)
P.refl_p = 0;  P.refl_g = 0.5;               % reflections: per-capture probability, echo amplitude / direct amplitude (random phase)
end

%% =====================================================================================================================
%% STRUCTURAL ANALYSIS (class method)
%% =====================================================================================================================
function info = report_structure(name, A, B, C)
% Kalman rank tests, PBH tests, Kalman decomposition, stabilizability / detectability.
n = size(A,1);  tol = 1e-9;
rc = rank(ctrb(A,B), tol);  ro = rank(obsv(A,C), tol);
fprintf('\n  %s\n', name);
fprintf('    n = %d, inputs = %d, outputs = %d (%s).  rank(ctrb) = %d, rank(obsv) = %d -> controllable: %s, observable: %s\n', n, size(B,2), ...
        size(C,1), mimo_label(size(C,1), size(B,2)), rc, ro, tf_(rc==n), tf_(ro==n));
lam = unique(round(eig(A)*1e6)/1e6);
for i = 1:numel(lam)
    pc = rank([lam(i)*eye(n) - A, B], tol);  po = rank([lam(i)*eye(n) - A; C], tol);
    fprintf('    PBH at lambda = %+.3g: rank[lI-A, B] = %d (%s), rank[lI-A; C] = %d (%s)\n', real(lam(i)), pc, ...
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
if nz, pr = roots([1 2*P.pp.zeta_r*P.pp.wn_r P.pp.wn_r^2]); else, pr = -P.pp.wn_r; end
pb = roots([1 2*P.pp.zeta_b*P.pp.wn_b P.pp.wn_b^2]);
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
function W = gram_fh(A, B, T)                                      % finite-horizon Gramian (A is only marginally stable)
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
Am = amp_(P.K*P.Kerr, xd(1), xd(2), phic - P.dlt, P);
Ap = amp_(P.K*P.Kerr, x(1),  x(2),  phic + P.dlt, P);
b = phic + P.sig^2/(2*P.dlt)*log(Ap/Am);                                      % Eq. (beta)
r = P.K/sqrt(Ap*Am)*exp(-((phic - b)^2 + P.dlt^2)/(2*P.sig^2));               % Eq. (rho)
y = [r - P.d; b];  valid = double(min(Am, Ap) > P.A_thr);
s.n = s.n + 1;  cnt = s.n;
end
function A = amp_(Kt, rho, beta, phi, P)
a0 = Kt/max(rho,1e-2)*exp(-(phi - beta)^2/(2*P.sig^2));                       % Eq. (amp)
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
Ts = P.Ts;  nm = round(P.Tm/Ts);  ndw = round(P.Tdw/Ts);  nd = round(P.Td/Ts);  n = round(T/Ts);  nx = D.nx;
X = zeros(3,n+1);  XH = zeros(nx,n+1);  U = zeros(2,n);  X(:,1) = x0(:);  STP = false(1,n);
obs = struct('x',zeros(nx,1), 'cprev',0, 'init',false, 'ntot',0, 'age',0);  ctl = struct('z',0);  snsr = struct('n',0);
va = P.v0;  wa = 0;  uprev = [0; 0];  ym = [0; 0];  valid = 0;  cnt = 0;  knext = 0;
queue = zeros(4, n+nd+2);  qa = false(1, n+nd+2);  meas = zeros(3,0);
ev = exp(-Ts/P.tau_v);  ew = exp(-Ts/P.tau_w);
for k = 0:n-1
    if qa(k+1), ym = queue(1:2,k+1);  valid = queue(3,k+1);  cnt = queue(4,k+1); end
    if D.ideal
        xh = [X(1,k+1) - P.d; X(2:3,k+1); zeros(nx-3,1)];                           % cheat: true state
        obs.age = 0;
    else
        xhd = XH(:, max(k-nd,0)+1);
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
