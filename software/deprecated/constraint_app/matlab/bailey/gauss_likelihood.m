function w = gauss_likelihood(v,S,logflag)
%function w = gauss_likelihood(v,S,logflag)
%
% INPUTS:
%   v - a set of innovation vectors.
%   S - the covariance matrix for the innovations.
%   logflag - <optional> - if 1 computes the log-likelihood, otherwise computes 
%           the likelihood.
%
% OUTPUT:
%   w - set of Gaussian likelihoods or log-likelihoods for each v(:,i).
%
% This implementation uses the Cholesky factor of S to compute the likelihoods 
% and so is more numerically stable than a simple full covariance form. 
%
% Adapted from GAUSEVAL, in the ReBEL Toolkit, by Rudolph van der Merwe (2002).
%
% Tim Bailey 2005.

if nargin == 2
    logflag = 0;
end

D = size(v,1);
Sf = chol(S)';
M = Sf\v; % equivalent to writing inv(Sf)*v
% M is the normalised innovation of v, and M(:,i)'*M(:,i) gives the Mahalanobis 
% distance for each v(:,i). 

C = (2*pi)^(D/2) * abs(prod(diag(Sf))); % normalising term (makes Gaussian hyper-volume equal 1)
E = -0.5 * sum(M.*M, 1);                % exponential term
% Note: writing sum(x.*x, 1) is a fast way to compute sets of inner-products.

if logflag ~= 1
    w = exp(E) / C; % likelihood
else
    w = E - log(C); % log-likelihood
end
