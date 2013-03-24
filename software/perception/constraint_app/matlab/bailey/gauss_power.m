function [P,w] = gauss_power(P,r)
%function [Pr,w] = gauss_power(P,r)
%
% INPUTS:
%   P - covariance of Gaussian
%   r - positive real power
%
% OUTPUTS:
%   Pr - resultant covariance
%   w - weight of resultant Gaussian
%
% Exponentiation of a Gaussian produces a weighted Gaussian, such that 
% Gauss(x,P)^r == w*Gauss(xr,Pr). Note, exponentiation does not alter the
% value of the mean, so xr == x.  
%
% Tim Bailey 2007.

d = size(P,1);
[V,D] = eig(P);
s = r-1;

w = 1 / sqrt((2*pi)^(d*s) * r^d * prod(diag(D).^s));
P = P/r;
