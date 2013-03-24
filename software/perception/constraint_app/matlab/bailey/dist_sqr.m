function d2 = dist_sqr(x1, x2)
%function d2 = dist_sqr(x1, x2)
%
% INPUTS:
%   x1 - Matrix of N column vectors
%   x2 - Matrix of M column vectors
%
% OUTPUT:
%   d2 - M x N matrix of square distances
%
% Compute the square distances of each vector in x1 to each vector in x2.
% To compute the set of Euclidean distances, simply compute sqrt(d2).
% This equation is adapted from Netlab, dist2.m, by Ian T Nabney.
%
% Tim Bailey 2005.

[D1,N1] = size(x1);
[D2,N2] = size(x2);

if D1 ~= D2, error('Vectors must have the same dimension'), end
    
d2 = (ones(N1,1)*sum(x2.^2, 1))' + ... 
      ones(N2,1)*sum(x1.^2, 1) - ...
      2.*(x2'*x1); 

d2(d2<0) = 0; % ensure rounding errors do not give negative values
