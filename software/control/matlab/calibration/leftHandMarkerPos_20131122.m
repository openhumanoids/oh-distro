function [x, dx] = leftHandMarkerPos_20131024(params,nograd)
%HANDMARKERPOS [x, dx] = handMarkerPos(params)
% NOTEST
% experimenting here with different parameterizations
% left hand
% 5 dots
%  front, top of hand marker
%  x: 17mm from outside edge of wrist plate ridge
%  y: 15mm from front edge of wrist plate
%  z: 72mm from top of wrist plate

if nargin < 2
  nograd = false;
end
%2,5
x = zeros(3,5);
% x(:,5) = [.0487;-.0948-.0715;.019 - .033];
% x(:,2) = [-.065+.018;-.0948 + .015;.019 + .072];
x(:,2) = [.065-.017;.0948-.015;.038 + .072];
x(:,[1 3:5]) = reshape(params,3,4);

if nograd
  dx = zeros(15,0);
else
dx = zeros(15,12);
dx(1:3,1:3) = eye(3);
dx(7:15,4:end) = eye(9);
end
end