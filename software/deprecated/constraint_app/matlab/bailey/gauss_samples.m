function s = gauss_samples(x,P,n)
%function s = gauss_samples(x,P,N)
%
% INPUTS: 
%   x, P - mean vector and covariance matrix 
%   N - number of samples
%
% OUTPUT:
%  s - set of N samples
%
% Produce N random sample from multivariate Gaussian distribution.
% Renamed from multivariate_gauss.m.
%
% Tim Bailey 2005.

len= length(x);
S= chol(P)';
X = randn(len,n); 
s = S*X + x*ones(1,n);
