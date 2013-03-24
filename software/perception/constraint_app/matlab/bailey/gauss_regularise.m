function s = gauss_regularise(x,P)
%function s = gauss_regularise(x,P)
%
% INPUTS: 
%   x - set of N column vectors
%   P - covariance matrix of Gaussian kernel
%
% OUTPUT:
%   s - set of N samples
%
% Jitter each vector in x according to a Gaussian kernel with covariance P.
%
% Tim Bailey 2005.

S = chol(P)';
X = randn(size(x)); 
s = S*X + x;
