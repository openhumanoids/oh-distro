function d = angleDiff(phi1, phi2)

% compute the angular difference between two angles, paying careful attention to wraparound

phi2 = mod(phi2, 2*pi);
phi1 = mod(phi1, 2*pi);

d = phi2 - phi1;
d(d > pi) = -(2*pi - d(d > pi));
d(d < -pi) = (2*pi + d(d < -pi));
% if d > pi
%   d = -(2*pi - d);
% end
% if d < -pi
%   d = 2*pi + d;
% end
