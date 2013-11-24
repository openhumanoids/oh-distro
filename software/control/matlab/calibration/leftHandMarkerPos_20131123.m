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

set_ind = 1;
other_ind = setdiff(1:5,set_ind);

% x(:,5) = [.0487;-.0948-.0715;.019 - .033];
% x(:,2) = [-.065+.018;-.0948 + .015;.019 + .072];
x(:,set_ind) = [.065 - .018;.0948 - .015;.038 + .04];
x(:,other_ind) = reshape(params,3,4);

if nograd
  dx = zeros(15,0);
else
  dx = zeros(15,12);
  for i=1:4,
    dx(3*(other_ind(i) - 1) + (1:3),(i-1)*3 + (1:3)) = eye(3);
  end
end
