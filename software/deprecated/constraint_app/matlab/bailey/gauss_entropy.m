function H = gauss_entropy(P)
%function H = gauss_entropy(P)
%
% INPUT:  P - covariance matrix
% OUTPUT: H - entropy
%
% The distribution entropy of a Gaussian is dependent only on the
% covariance, not the mean. For any fixed dimension, H is proportional to
% det(P).
%
% Tim Bailey 2007. 

e = exp(1);
d = size(P,1);
H = 0.5 * log((2*pi*e)^d * det(P));

% Alternatives:
%   log((2*pi*e)^d) = d*(1+log(2*pi))
%   so we could write:
%       H = 0.5 * (log((2*pi*e)^d) + log(det(P)))
%         = 0.5 * (d*(1+log(2*pi)) + log(det(P)))
%         = 0.5 * (d + d*log(2*pi) + log(det(P)))
