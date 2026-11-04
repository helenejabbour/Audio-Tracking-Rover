function xdot = blk_plant(x, vf, wf, w)
% Nonlinear relative kinematics, Eq. (nl). x = [rho; beta; eps], w = [v_t; w_t]. Plant block of the Simulink model.
rho = max(x(1), 1e-2);  beta = x(2);  al = x(3) - beta;
xdot = [ w(1)*cos(al) - vf*cos(beta);
        (w(1)*sin(al) + vf*sin(beta))/rho - wf;
         w(2) - wf ];
end
