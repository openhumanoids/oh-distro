function M = distance_mahalanobis(v,S)
%function M = distance_mahalanobis(v,S)
%
% INPUTS:
%   v - a set of innovation vectors.
%   S - the covariance matrix for the innovations.
%
% OUTPUT:
%   M - set of Mahalanobis distances for each v(:,i).
%
% Tim Bailey 2005.

Sc = chol(S)';
nv = Sc\v; % "normalised innovation", equivalent to writing inv(Sc)*v
M = sum(nv.*nv, 1);
% Note: writing sum(x.*x, 1) is a fast way to compute sets of inner-products.
