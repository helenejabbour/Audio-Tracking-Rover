function [Ad, Bd] = zoh_mat(A, B, dt)
% Exact zero-order-hold discretisation.
n = size(A,1);  m = size(B,2);
M = expm([A B; zeros(m, n+m)]*dt);
Ad = M(1:n,1:n);  Bd = M(1:n,n+1:n+m);
end
