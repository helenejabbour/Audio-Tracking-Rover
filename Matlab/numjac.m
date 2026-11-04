function J = numjac(g, z0)
% Central-difference Jacobian of g at z0.
h = 1e-6;  n = numel(z0);  f0 = g(z0);  J = zeros(numel(f0), n);
for i = 1:n
    dz = zeros(n,1);  dz(i) = h;
    J(:,i) = (g(z0+dz) - g(z0-dz))/(2*h);
end
end
