function w = chi_square_to_gauss(c, P, logflag)
%function w = chi_square_to_gauss(c, P, logflag)
%
% INPUTS:
%   c - chi^2 bound
%   P - Gaussian covariance matrix
%   logflag - if 0 return weight, if 1 return log-weight, (default 0) 
%
% OUTPUT:
%   w - Gaussian likelihood for c
%
% Given a chi^2 bound, c, compute the equivalent Gaussian cutoff weight, w,
% for a covariance P. 
%
% Tim Bailey 2006.

if nargin == 2, logflag = 0; end

D = size(P,1);

if logflag == 0
    w = exp(-0.5*c) ./ sqrt((2*pi)^D * det(P)); % likelihood

else
    Nd = c + log(det(P));          % normalised distance
    w = -0.5 * (Nd + D*log(2*pi)); % log-likelihood 
end
