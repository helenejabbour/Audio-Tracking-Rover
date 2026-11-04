function [vcmd, wcmd, ua, phic] = blk_controller(xh, Pc)
% LQR with integral action on range + command limits (O4). The integrator runs only when the speed command is not
% saturated AND |d_rho_hat| < zband: integrating through a large initial offset winds it up and causes ~15-20% overshoot.
% Pc = [Kx(:)(6); Kz(2); Ad4(3); Bd4(2); v0; vmax; dsmax; Lw; zmax; phimax; zband].
% ua = [dv_f; w_f] actually commanded (fed to the observer); phic = arm centre for the next acoustic pair.
persistent z
if isempty(z), z = 0; end
Kx = reshape(Pc(1:6),2,3);  Kz = Pc(7:8);  Ad4 = Pc(9:11).';  Bd4 = Pc(12:13).';
v0 = Pc(14); vmax = Pc(15); dsmax = Pc(16); Lw = Pc(17); zmax = Pc(18); phimax = Pc(19); zband = Pc(20);
u = -Kx*xh(1:3) - Kz*z;
vcmd = min(max(v0 + u(1), 0), vmax);
wlim = max(vcmd, 0.05)*tan(dsmax)/Lw;                    % w = v tan(ds)/L
wcmd = min(max(u(2), -wlim), wlim);
ua = [vcmd - v0; wcmd];
if vcmd == v0 + u(1) && abs(xh(1)) < zband               % integrate only when unsaturated and near the setpoint
    z = min(max(Ad4*xh(1:3) + z + Bd4*ua, -zmax), zmax);
end
phic = min(max(xh(2), -phimax), phimax);
end
