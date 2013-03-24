function e = ellipse_mass(x, P, prob, N)
%function e = ellipse_mass(x, P, prob, N)
%
% INPUTS:
%   x, P - mean and covariance of a 2-D Gaussian
%   prob - probability mass bounded by ellipse
%   N - number of lines in polyline (default 60)
%
% OUTPUT:
%   e = points of ellipse polyline
%
% Tim Bailey 2006.

if nargin==4, inc = 2*pi/N; else inc = pi/30; end

r = sqrtm(P);
C = sqrt(chi_square_bound(prob, 2));

phi = [0:inc:2*pi 0];
a = C * r * [cos(phi); sin(phi)];

e(1,:) = a(1,:) + x(1);
e(2,:) = a(2,:) + x(2);
