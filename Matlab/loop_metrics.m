function m = loop_metrics(out, p)
% 2% settling of the true range error (band >= 3 cm), overshoot (only if |initial error| >= 0.25 m), RMS bearing.
t = out.t;  y = out.X(1,:) - p.d;
band = max(0.02*abs(y(1)), 0.03);
idx = find(abs(y) > band);
if isempty(idx), ts = 0; elseif idx(end) < numel(t), ts = t(idx(end)+1); else, ts = Inf; end
if abs(y(1)) < 0.25, ov = NaN; else, ov = max(0, max(-sign(y(1))*y))/abs(y(1))*100; end
tail = t > t(end) - 10;
m = struct('ts', ts, 'ov', ov, 'rms_beta_deg', rad2deg(sqrt(mean(out.X(2,tail).^2))), ...
           'rho_min', min(out.X(1,:)), 'yss', mean(y(tail)));
end
