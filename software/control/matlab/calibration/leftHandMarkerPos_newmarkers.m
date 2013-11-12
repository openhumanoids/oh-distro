function [x, dx] = leftHandMarkerPos_newmarkers(params,nograd)
%HANDMARKERPOS [x, dx] = handMarkerPos(params)
% NOTEST
% five markers on the hand, 4 in xz-plane
% second point is out of plane
% [1 3 4 5] is in order
%
% experimenting here with different parameterizations

if nargin < 2
  nograd = false;
end

x = zeros(3,5);
x(:,2) = [.0487;.0948+.0715;.038 - .033];
% x(:,4) = [0;-.0948;.038];
x(:,[1 3:5]) = reshape(params,3,4);

if nograd
  dx = zeros(15,0);
else
dx = zeros(15,12);
dx(7:15,4:12) = eye(9);
dx(1:3,1:3) = eye(3);
end
% 
% left hand:
%   - short standoff dot on front of wrist 
%     - x: 48.7mm forward from center of force sensor
%     - y: 71.5mm outward from front wrist plate
%     - z: 32mm down from top edge of fron wrist plate


end

