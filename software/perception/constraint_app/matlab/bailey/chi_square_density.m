function f=chi_square_density(x,n)
%function f=chi_square_density(x,n)
% INPUTS: x - x-axis coordinates, n - degrees of freedom
% OUTPUT: f(x) - Chi-square probability density function
%
% Reference: Papoulis, "Probability, Random Variables and Stochastic Processes", 4th Ed., 2002, p89.
%
% Tim Bailey 2001.

k = n/2;
C = 2.^k .* gamma(k); % constant term
f = x.^(k-1) .* exp(-x/2) ./ C; 
