function [xm,P]= sample_mean(x)
%function [xm,P]= sample_mean(x)
%
% INPUT: set of samples, where each sample is a column vector [x1;...;xi;...]
% OUTPUTS: sample mean, and sample covariance matrix
%
% Convert a set of samples to a mean vector and covariance matrix. Results of this 
% function are (should be) equivalent to calling xm = mean(x,2); and P = cov(x',1); 
% from the MatLab library. 
%   Note that P is the second moment of the data (ie, normalised by N) not the
% "unbiased" covariance estimate (ie, normalised by N-1). To convert to the latter, 
% simply write P = P*N/(N-1), where N = size(x,2).
%
%
% Tim Bailey 2005.

N = size(x,2); % number of samples
xm = sum(x,2) / N;

if nargout == 2,
    P = (x*x' / N) - xm*xm';
end

% Alternative formulations:
%
% xc = x - repmat(xm,1,N); 
% P = xc*xc' / N;               % 2nd moment version 1
% P = xc*xc' / (N-1);           % "unbiased covariance" version 1
% P = (x*x' / N) - xm*xm'       % 2nd moment version 2
% P = (x*x' - N*xm*xm') / (N-1) % "unbiased covariance" version 2
%
% Note, for the "unbiased" equations, need to first check for
% special case where N equals 1 (wherein P = zeros(length(x))).
