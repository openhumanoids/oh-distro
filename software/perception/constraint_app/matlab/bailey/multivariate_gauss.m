function s= multivariate_gauss(x,P,n)
%function s= multivariate_gauss(x,P,n)
%
% INPUTS: 
%   (x, P) mean vector and covariance matrix
%   obtain n samples
% OUTPUT:
%   sample set
%
% Random sample from multivariate Gaussian distribution.
% Adapted from MVNORMRND (c) 1998, Harvard University.
%
% Tim Bailey 2004.

len= length(x);
S= chol(P)';
X = randn(len,n); 
s = S*X + x*ones(1,n);
