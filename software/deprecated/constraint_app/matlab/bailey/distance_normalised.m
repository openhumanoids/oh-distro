function N = distance_normalised(v, S)
%function N = distance_normalised(v, S)
%
% INPUTS:
%   v - set of innovation vectors
%   S - innovation covariance matrix
%
% OUTPUTS:
%   N - normalised distance: v'*inv(S)*v + log(det(S))
%
% NOTES:
%   The normalised distance is derived from the Gaussian likelihood
% function, and is proportional to the negative log-likelihood. It
% is obtained by taking logs, multiplying by -2 and subtracting D*log(2*pi)
% where D is the dimension of v.
%
%   N = -2*gauss_likelihood(v,S,1) - size(v,1)*log(2*pi);
% 
% It is often used as a metric for "nearest neighbours" data association
% (eg, see S.S. Blackman and R. Popoli. "Design and Analysis of Modern Tracking 
% Systems". Artech House Radar Library, 1999).
%
% Tim Bailey 2005.

Sc = chol(S)';
nv = Sc\v; 
N = sum(nv.*nv, 1) + 2*sum(log(diag(Sc)));

% Alternatives:
%   N = distance_mahalanobis(v, S) + log(det(S));
%   N = -2*gauss_likelihood(v,S,1) - size(v,1)*log(2*pi);
