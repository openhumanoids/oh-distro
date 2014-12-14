function e = sigma_ellipse(x, P, n, nseg)
%function e = sigma_ellipse(x, P, n)
% INPUTS: mean [x; y], covariance [xx xy; xy yy], number of sigmas
% OUTPUT: ellipse points [x; y]
% Plot n sigma ellipse
%
% Tim Bailey 2004, modified 2006.

if nargin==4, inc = 2*pi/nseg; else inc = pi/30; end

r=sqrtm(P);
phi=[0:inc:2*pi 0];
a=n*r*[cos(phi); sin(phi)];

e(1,:)= a(1,:)+x(1);
e(2,:)= a(2,:)+x(2);
