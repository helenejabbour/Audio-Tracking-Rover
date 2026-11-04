%% run_validation.m -- S3: closed-loop runs with the block functions (run design_local_model first)
clear; clc; close all
if exist('OCTAVE_VERSION','builtin'), pkg load control; end
load('sim_params.mat', 'Po', 'Pc', 'Ps', 'p');
rng(1);

% representative run: start 0.4 m too far, 15 deg off, target 0.05 m/s faster than the follower's nominal speed
x0  = [p.d+0.4; deg2rad(15); deg2rad(-10)];
out = run_loop(p, Po, Pc, Ps, x0, @(t) [p.v0+0.05; 0], 30);
disp(loop_metrics(out, p))
try   % plots (skipped on machines without graphics)
    figure; subplot(3,1,1); plot(out.t, out.X(1,:), out.t, out.XH(1,:)+p.d, '--', out.meas(1,:), out.meas(2,:), '.'); ylabel('range [m]'); legend('true','estimate','raw');
    subplot(3,1,2); plot(out.t, rad2deg(out.X(2,:)), out.t, rad2deg(out.XH(2,:)), '--'); ylabel('bearing [deg]');
    subplot(3,1,3); plot(out.t(1:end-1), out.U(1,:), out.t(1:end-1), out.U(2,:)); ylabel('v_f [m/s], \omega_f [rad/s]'); xlabel('t [s]');
catch
    disp('(no graphics available: plot skipped)');
end

% Monte Carlo, two scenarios: transient to random offsets (O3) and constant target-speed offset (O1)
N = 20;
scnames = {'transient, no disturbance (O3)', 'target speed offset +-0.05 m/s (O1)'};
names = {'2% settling [s]', 'overshoot [%] (|d_rho0|>=0.25 m)', 'RMS beta, last 10 s [deg]', 'steady-state range error [m]', 'min range [m]'};
for scen = 1:2
    R = zeros(N,5);
    for i = 1:N
        x0 = [p.d + (-0.35+0.85*rand); deg2rad(-20+40*rand); deg2rad(-15+30*rand)];
        if scen == 1, dvt = 0; else, dvt = 0.05*(2*randi(2)-3); end        % scenario 2: +-0.05 m/s
        m = loop_metrics(run_loop(p, Po, Pc, Ps, x0, @(t) [p.v0+dvt; 0], 30), p);
        R(i,:) = [m.ts m.ov m.rms_beta_deg m.yss m.rho_min];
    end
    fprintf('\n[S3] %s, %d runs (mean +- std, worst)\n', scnames{scen}, N);
    for j = 1:5
        v = R(~isnan(R(:,j)), j);
        if j == 5, w = min(v); elseif j == 4, w = max(abs(v)); else, w = max(v); end
        fprintf('  %-34s %7.3f +- %6.3f   worst %7.3f\n', names{j}, mean(v), std(v), w);
    end
end
