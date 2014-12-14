function angle = pi_to_pi(angle)

%function angle = pi_to_pi(angle)
% Input: array of angles.
% Output: normalised angles.
% Tim Bailey 2000, modified 2005.

% Note: either rem or mod will work correctly
%angle = mod(angle, 2*pi); % mod() is very slow for some versions of MatLab (not a builtin function)
%angle = rem(angle, 2*pi); % rem() is typically builtin

twopi = 2*pi;
angle = angle - twopi*fix(angle/twopi); % this is a stripped-down version of rem(angle, 2*pi)

i = find(angle > pi);
angle(i) = angle(i) - twopi;

i = find(angle < -pi);
angle(i) = angle(i) + twopi;
