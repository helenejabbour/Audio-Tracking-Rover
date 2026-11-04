function W = gram_fh(A, B, T, n)
% Finite-horizon Gramian  int_0^T e^{At} B B' e^{A't} dt  (A is only marginally stable, so no infinite horizon).
if nargin < 3, T = 6; end
if nargin < 4, n = 300; end
t = linspace(0, T, n);  N = size(A,1);  Wk = zeros(N, N, n);
for k = 1:n
    M = expm(A*t(k))*B;  Wk(:,:,k) = M*M';
end
W = zeros(N);
for i = 1:N
    for j = 1:N
        W(i,j) = trapz(t, squeeze(Wk(i,j,:)));
    end
end
end
