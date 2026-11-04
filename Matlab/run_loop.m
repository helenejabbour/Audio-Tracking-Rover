function out = run_loop(p, Po, Pc, Ps, x0, wfun, T)
% Closed loop with the SAME block functions the Simulink model calls (no Simulink needed).
% wfun(t) -> [v_t; w_t].  Reference trace for checking the Simulink wiring.
clear blk_sensor blk_observer blk_controller
Ts = p.Ts;  nm = round(p.Tm/Ts);  ndw = round(p.Tdw/Ts);  nd = round(p.Td/Ts);  n = round(T/Ts);
X = zeros(3,n+1);  XH = zeros(5,n+1);  U = zeros(2,n);  X(:,1) = x0(:);
va = p.v0;  wa = 0;  uprev = [0; 0];
ym = [0; 0];  valid = 0;  cnt = 0;                          % held output of the Td delay block
queue = zeros(4, n+nd+2);  qa = false(1, n+nd+2);  meas = zeros(3,0);
ev = exp(-Ts/p.tau_v);  ew = exp(-Ts/p.tau_w);
for k = 0:n-1
    if qa(k+1), ym = queue(1:2,k+1);  valid = queue(3,k+1);  cnt = queue(4,k+1); end
    xhd = XH(:, max(k-nd,0)+1);
    xh  = blk_observer(uprev, ym, valid, cnt, xhd, Po);  XH(:,k+1) = xh;
    [vc, wc, ua, phic] = blk_controller(xh, Pc);          U(:,k+1) = [vc; wc];  uprev = ua;
    if mod(k, nm) == 0                                     % acoustic pair completes now
        xd = X(:, max(k-ndw,0)+1);
        [y, vl, c] = blk_sensor(X(:,k+1), xd, phic, Ps);
        queue(:,k+nd+1) = [y; vl; c];  qa(k+nd+1) = true;
        if vl > 0.5, meas(:,end+1) = [k*Ts; y(1)+p.d; y(2)]; end %#ok<AGROW>
    end
    va = ev*va + (1-ev)*vc;  wa = ew*wa + (1-ew)*wc;       % exact first-order actuator lags
    X(:,k+2) = rk4_step(X(:,k+1), [va; wa], wfun(k*Ts), Ts);
end
out = struct('t', (0:n)*Ts, 'X', X, 'XH', XH, 'U', U, 'meas', meas);
end
