function [num, den] = pade_tf(T, n)
% n-th order Pade approximation of exp(-sT); coefficients in descending powers of s.
c = zeros(1, n+1);
for k = 0:n
    c(k+1) = factorial(2*n-k)*factorial(n)/(factorial(2*n)*factorial(k)*factorial(n-k));
end
num = zeros(1, n+1);  den = num;
for k = 0:n
    num(n+1-k) = c(k+1)*(-T)^k;
    den(n+1-k) = c(k+1)*(T)^k;
end
end
