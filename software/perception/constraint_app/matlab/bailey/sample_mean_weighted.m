function [xm,P] = sample_mean_weighted(x,w)
%function [xm,P] = sample_mean_weighted(x,w)
%
% INPUTS:
%   x - set of N samples, where each sample is a column vector 
%   w - set of N weights
%
% OUTPUTS: 
%   xm - sample mean
%   P  - sample covariance matrix
%
% Compute the 1st and 2nd moments of a set of weighted samples. Note, for 
% these results to make much sense, it is advised to first normalise the 
% weights: w = w/sum(w).
%
% Tim Bailey 2005.

if abs(1-sum(w)) > 1e-12, warning('Weights should be normalised'), end
if sum(w~=0) <= size(x,1), warning('Samples form a hyperplane, covariance rank deficient'), end

D = size(x,1); % sample dimension
w = reprow(w,D);

xw = w .* x;
xm = sum(xw, 2);

if nargout == 2
    P  = xw*x' - xm*xm';
end

% Alternative formulation for P that may be better to give pos. def. result.
%xc = x - repcol(xm,size(x,2));
%P = w.*xc*xc'; % note, (w.*xc)*xc' is not w.*(xc*xc'). Latter is illegal.
