%% design_local_model.m  -- linear design, checks S1/S2, margins (O5), export for Simulink and the Pi.
% Needs Control System Toolbox. Run it first; it creates Po, Pc, Ps (parameter vectors used by the blocks).
clear; clc; close all
if exist('OCTAVE_VERSION','builtin'), pkg load control; end
p = params();  Ts = p.Ts;

%% 1. Local model (Eqs. lin, mats): x=[d_rho beta eps], u=[dv_f w_f], w=[dv_t w_t], y=[d_rho beta]
a = p.v0/p.d;
A = [0 0 0; 0 0 a; 0 0 0];  B = [-1 0; 0 -1; 0 -1];  E = [1 0; 0 0; 0 1];  C = [1 0 0; 0 1 0];

%% 2. Check the matrices against the nonlinear model (numeric Jacobian)
x0n = [p.d; 0; 0];  u0 = [p.v0; 0];  w0 = [p.v0; 0];
f  = @(x,u,w) blk_plant(x, u(1), u(2), w);
Aj = numjac(@(z) f(z,u0,w0), x0n);  Bj = numjac(@(z) f(x0n,z,w0), u0);  Ej = numjac(@(z) f(x0n,u0,z), w0);
fprintf('[S1] max |numeric Jacobian - Eq.(mats)| = %.2e\n', max([max(abs(Aj(:)-A(:))) max(abs(Bj(:)-B(:))) max(abs(Ej(:)-E(:)))]));

%% 3. S2: controllability / observability and Gramian conditioning vs a = v0/d*  (tolerance-scaled units)
Sx = diag([0.1 0.17 0.26]);  Su = diag([0.1 0.5]);  Sy = diag([0.03 0.035]);
fprintf('[S2]  a=v0/d* | rank ctrb  rank obsv | cond(Wc)  cond(Wo) | rank obsv with [dv_t,w_t]\n');
for aa = [0.02 0.05 0.1 0.25 0.5 1.0]
    A_ = [0 0 0; 0 0 aa; 0 0 0];
    As = Sx\(A_*Sx);  Bs = Sx\(B*Su);  Cs = Sy\(C*Sx);
    Ao_ = [A_ E; zeros(2,5)];  Co_ = [C zeros(2,2)];
    fprintf('      %5.2f   |   %d         %d      | %8.1e %8.1e |   %d\n', aa, rank(ctrb(A_,B)), rank(obsv(A_,C)), ...
            cond(gram_fh(As,Bs)), cond(gram_fh(As',Cs')), rank(obsv(Ao_,Co_)));
end

%% 4. Discrete LQR with integral action on range (O1)
Aa = [A zeros(3,1); C(1,:) 0];  Ba = [B; 0 0];
[Ad, Bd] = zoh_mat(Aa, Ba, Ts);
Kd = dlqr(Ad, Bd, diag(p.Qd), diag(p.Rd));
Kx = Kd(:,1:3);  Kz = Kd(:,4);
fprintf('\nLQR gain Kd (rows dv_f, w_f; cols d_rho, beta, eps, int d_rho):\n');  disp(Kd)
fprintf('closed-loop |eig| (discrete, augmented): ');  fprintf('%.3f ', sort(abs(eig(Ad-Bd*Kd))));  fprintf('\n');

%% 5. Observer on [x; dv_t; w_t]; measurement noise from App. B at the operating point
Apk = p.K/p.d*exp(-p.dlt^2/(2*p.sig^2));                    % A+/- with the arm centred on the target
s2  = 2*(p.sn/Apk)^2;                                       % var(ln A+) + var(ln A-)
sr  = sqrt(p.d^2*s2/4 + p.sr_floor^2);
sb  = sqrt((p.sig^2/(2*p.dlt))^2*s2 + p.sb_floor^2);
Rm  = diag([sr^2 sb^2]);                                    % per measurement
fprintf('measurement std at d*: rho_hat %.3f m (%.1f%%), beta_hat %.2f deg\n', sr, 100*sr/p.d, rad2deg(sb));
Ao = [A E; zeros(2,5)];  Bo = [B; zeros(2,2)];  Co = [C zeros(2,2)];
[Phi, Gam] = zoh_mat(Ao, Bo, Ts);
Phim = expm(Ao*p.Tm);  Qm = diag(p.Qw)*p.Tm;                % one measurement per Tm
Pp = dare(Phim', Co', Qm, Rm);   Lk = Pp*Co'/(Co*Pp*Co' + Rm);   % gain used by blk_observer
PhiTd = expm(Ao*p.Td);

%% 6. O5 margins: LQR loop broken at the plant input, with actuator lags and the measured delay (Pade)
% Assumes the delay-compensated observer delivers the current state, but still counts Td in the loop (conservative).
% Continuous-time LQR with the same weights is used only for this frequency-domain check.
Kc    = lqr(Aa, Ba, diag(p.Qd), diag(p.Rd));
Gact  = ss(diag([-1/p.tau_v -1/p.tau_w]), diag([1/p.tau_v 1/p.tau_w]), eye(2), zeros(2,2));
[nn, dd] = pade_tf(p.Td, 3);  Dl = ss(tf(nn, dd));
Lsf   = Kc*ss(Aa, Ba, eye(4), zeros(4,2))*Gact*append(Dl, Dl);     % K and plant are block-diagonal => channels decouple
fprintf('\n[O5] margins with delay Td = %.2f s (target PM >= 45 deg, GM >= 6 dB)\n', p.Td);
chname = {'range', 'bearing'};  verdict = {'FAIL', 'PASS'};
for ch = 1:2
    [Gm, Pm, ~, Wp] = margin(Lsf(ch,ch));
    ok = (Pm >= 45) && (20*log10(Gm) >= 6);
    fprintf('  %-8s PM %5.1f deg @ %.2f rad/s (extra delay tolerated %.2f s), GM %5.1f dB -> %s\n', chname{ch}, ...
            Pm, Wp, Pm*pi/180/Wp, 20*log10(Gm), verdict{ok+1});
end

%% 7. S1: nonlinear vs linear under the same discrete state feedback (kinematics only)
base = [0.3; deg2rad(20); deg2rad(20)];  Acl = Ad - Bd*Kd;  n = round(15/Ts);
fprintf('\n[S1] normalized RMS error nonlinear vs LTI, offsets scaled from d_rho0=0.3 m, beta0=eps0=20 deg\n');
for s = [0.25 0.5 1 1.5 2]
    xn = base*s;  zn = 0;  xa = [base*s; 0];  XN = zeros(3,n);  XL = XN;
    for k = 1:n
        un = -Kx*xn - Kz*zn;
        zn = zn + Ad(4,1:3)*xn + Bd(4,:)*un;
        xn = rk4_step(x0n + xn, [p.v0 + un(1); un(2)], [p.v0; 0], Ts) - x0n;
        xa = Acl*xa;  XN(:,k) = xn;  XL(:,k) = xa(1:3);
    end
    nr = sqrt(mean((XN-XL).^2, 2))./sqrt(mean(XL.^2, 2));
    fprintf('  scale %.2f: d_rho %.3f  beta %.3f  eps %.3f\n', s, nr);
end

%% 8. Export: parameter vectors for the block functions (Simulink + run_loop) and gains.json for the Pi
Po = [Phi(:); Gam(:); Co(:); Lk(:); PhiTd(:)];
Pc = [Kx(:); Kz(:); Ad(4,1:3)'; Bd(4,:)'; p.v0; p.v_max; p.ds_max; p.L; p.zmax; p.phi_max; p.zband];
Ps = [p.K; p.Kerr; p.sig; p.dlt; p.sn; p.A_thr; p.d];
save('sim_params.mat', 'Po', 'Pc', 'Ps', 'p');
G = struct('Ts',Ts, 'Tm',p.Tm, 'Td',p.Td, 'd',p.d, 'v0',p.v0, 'Kd',Kd, 'Phi',Phi, 'Gam',Gam, 'Co',Co, ...
           'L',Lk, 'PhiTd',PhiTd, 'Ad4',Ad(4,1:3), 'Bd4',Bd(4,:));
fid = fopen('gains.json','w');  fwrite(fid, jsonencode(G));  fclose(fid);
fprintf('\nsaved sim_params.mat (Po, Pc, Ps) and gains.json\n');
