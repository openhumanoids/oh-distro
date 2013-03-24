function d2 = dist_sqr_(s, x)
%function d2 = dist_sqr_(s, x)
%
% INPUTS:
%   s - matrix of N column vectors
%   x - a single column vector OR 
%       a matrix of N column vectors
%
% OUTPUT:
%   d2 - an array of N square distances
%
% Compute the square distances from x to each vector in s. If x is a matrix of N column
% vectors, compute the square distance of x(:,i) to s(:,i) for each i. To compute the set 
% of Euclidean distances, simply write: d = sqrt(d2).
%
% Tim Bailey 2005.

[Ds,Ns] = size(s);
[Dx,Nx] = size(x);

if Ds ~= Dx, error('Vectors must have the same dimension'), end

switch Nx
case 1
    xdif = x(:, ones(1,Ns)) - s; 
case Ns
    xdif = x - s; 
otherwise 
    error('Second parameter must either have as many columns as the first parameter or be a single column vector.'), 
end

d2 = sum(xdif.*xdif, 1);
