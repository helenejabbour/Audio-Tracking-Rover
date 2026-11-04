function x = rk4_step(x, u, w, dt)
% One RK4 step of the nonlinear plant with constant u = [v_f; w_f], w = [v_t; w_t].
f = @(s) blk_plant(s, u(1), u(2), w);
k1 = f(x);  k2 = f(x + dt/2*k1);  k3 = f(x + dt/2*k2);  k4 = f(x + dt*k3);
x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
