function d2 = dist_sqr_v2(x1, x2)
%function d2 = dist_sqr_v2(x1, x2)
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
    
s1 = sum(x1.^2, 1);
s2 = sum(x2.^2, 1)';

if 0
d2 = s2(:,ones(1,N1)) + s1(ones(1,N2),:) - 2*x2'*x1;
else
d2 = s2(:,ones(1,N1)) - 2*x2'*x1 + s1(ones(1,N2),:);
end

d2(d2<0) = 0; % ensure rounding errors do not give negative values
