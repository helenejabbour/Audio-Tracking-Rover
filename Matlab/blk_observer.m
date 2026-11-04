function xh = blk_observer(uprev, ym, valid, cnt, xhd, Po)
% Fixed-gain observer on [d_rho beta eps dv_t w_t] with delayed, intermittent measurements.
% Each Ts: predict with the previous applied input uprev = [dv_f; w_f].
% When a new sample arrives (cnt changed) and is valid:  x += Phi(Td) * L * (ym - Co*xhd),
% xhd = this estimate Td ago. For an LTI model this equals correcting the past estimate and replaying
% the stored inputs (exact for Td <= Tm). The first valid sample initialises [d_rho; beta].
persistent x cprev init
if isempty(x), x = zeros(5,1); cprev = 0; init = false; end
Phi = reshape(Po(1:25),5,5);   Gam = reshape(Po(26:35),5,2);  Co = reshape(Po(36:45),2,5);
L   = reshape(Po(46:55),5,2);  PhiTd = reshape(Po(56:80),5,5);
x = Phi*x + Gam*uprev;
if cnt ~= cprev
    cprev = cnt;
    if valid > 0.5
        if ~init
            x = PhiTd*[ym; 0; 0; 0];  init = true;
        else
            x = x + PhiTd*(L*(ym - Co*xhd));
        end
    end
end
xh = x;
end
