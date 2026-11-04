function p = params()
% All model/design parameters in one place. Values marked (*) are PLACEHOLDERS: overwrite with measured ones.
% Acoustic amplitudes are in ADC counts (same units as the firmware): A = K/rho * exp(-(phi-beta)^2/(2 sig^2)).
%   K    = A_geo_x_rho * exp(dlt^2/(2 sig^2))   (A_geo_x_rho from esp_logger summary, arm centred on the target)
%   sn   = s_n_est from a speaker-off logger run (counts, per quadrature)
%   Tm   = T_m_median_s from the logger;  Td = capture -> controller latency (E4)
p.v0 = 0.25;  p.d = 1.0;                 % m/s, m   (*)
p.L = 0.145;  p.ds_max = deg2rad(30);    % wheelbase [m], steering limit [rad]   (*)
p.v_max = 0.5;  p.tau_v = 0.20;  p.tau_w = 0.15;   % speed limit [m/s], actuator lags [s]  (*)
p.rho_min = 0.30;                        % m (*)
p.K = 1.0;  p.Kerr = 1.0;                % K [counts*m] (*); Kerr = K_true/K_model (tests the O1 caveat)
p.sig = deg2rad(25);  p.dlt = deg2rad(25);   % horn sigma (*), dither delta [rad]
p.sn = 0.01;  p.A_thr = 0.05;            % noise std [counts] (*), detection threshold [counts]
p.phi_max = deg2rad(80);
p.sr_floor = 0.03;  p.sb_floor = deg2rad(2);   % model-error floors on rho_hat [m], beta_hat [rad] (reflections: raise)
p.Ts = 0.05;  p.Tm = 0.40;  p.Tdw = 0.10;  p.Td = 0.30;   % control, measurement, dwell, latency [s]  (*)
p.zmax = 1.0;                            % integrator clamp [m*s]
p.zband = 0.10;                          % integrate only when |d_rho_hat| < zband [m] (prevents transient windup)
p.Qd = [50 10 3 5];                      % LQR weights on [d_rho beta eps int(d_rho)] (tuned for O5 with Td = 0.3 s)
p.Rd = [100 16];                         % LQR weights on [dv_f w_f]
p.Qw = 10*[1e-3 1e-3 1e-3 5e-4 2e-2];    % observer process-noise intensity on [d_rho beta eps dv_t w_t] (larger = faster)
end
