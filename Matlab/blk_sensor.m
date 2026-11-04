function [y, valid, cnt] = blk_sensor(x, xd, phic, Ps)
% Paired acoustic measurement. x: true state at the A+ capture, xd: true state at the A- capture (Tdw earlier).
% Ps = [K Kerr sig dlt sn Athr d]. Output y = [rho_hat - d; beta_hat], valid flag, sample counter.
persistent n
if isempty(n), n = 0; end
K = Ps(1); Kerr = Ps(2); sig = Ps(3); dlt = Ps(4); sn = Ps(5); Athr = Ps(6); d = Ps(7);
Am = amp_(K*Kerr, xd(1), xd(2), phic - dlt, sig, sn);
Ap = amp_(K*Kerr, x(1),  x(2),  phic + dlt, sig, sn);
b = phic + sig^2/(2*dlt)*log(Ap/Am);                                   % Eq. (beta)
r = K/sqrt(Ap*Am)*exp(-((phic - b)^2 + dlt^2)/(2*sig^2));              % Eq. (rho)
y = [r - d; b];
valid = double(min(Am, Ap) > Athr);
n = n + 1;  cnt = n;
end

function A = amp_(Kt, rho, beta, phi, sig, sn)
a0 = Kt/max(rho, 1e-2)*exp(-(phi - beta)^2/(2*sig^2));                 % Eq. (amp)
A = abs(complex(a0 + sn*randn, sn*randn));                             % Rician magnitude, as the Goertzel output
end
