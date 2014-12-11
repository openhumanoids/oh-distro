function x = reprow(x, N)
%function xrep = reprow(x, N)
%
% Replicate a row vector x = [x1, x2, ...] so that
% 
%           x1 x2 x3 ...
% xrep =    x1 x2 x3 ...
%           ...
%
% This function is much faster than using the MatLab function REPMAT.
% ie, xr = repmat(x, N, 1);
%
% Tim Bailey 2005.

x = x(ones(1,N), :);
