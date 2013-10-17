function [x, dx] = handMarkerPos_newmarkers(params,nograd)
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


% x = repmat([.0485;-.0948-.0715;.038 - .033],1,5);

% x = zeros(3,5);
% x(:,2) = [.0485;-.0948-.0715;.038 - .033];
% x(:,[1 3:5]) = reshape(params,3,4);
% dx = zeros(15,12);
% dx(1:3,1:3) = eye(3);
% dx(7:end,4:end) = eye(9);

% x = zeros(3,5);
% x(:,1) = [.0485;-.0948-.0715;.038 - .033];
% x(:,[2 3:5]) = reshape(params,3,4);
% dx = zeros(15,12);
% dx(4:6,1:3) = eye(3);
% dx(7:end,4:end) = eye(9);

x = zeros(3,5);
x(:,5) = [.0487;-.0948-.0715;.038 - .033];
% x(:,4) = [0;-.0948;.038];
x(:,[1:4]) = reshape(params,3,4);

if nograd
  dx = zeros(15,0);
else
dx = zeros(15,12);
dx(1:9,1:9) = eye(9);
dx(10:12,10:end) = eye(3);
end
% right hand:
%   - short standoff dot on front of wrist 
%     - x: 48.7mm forward from center of force sensor
%     - y: 71.5mm outward from front wrist plate
%     - z: 33mm down from top edge of fron wrist plate

% 
% x = [-w12/2 w12/2 w35/2 -w12/2 -w35/2;
%      params(1) params(1) params(2) params(7) params(3);
%      params(4) params(5) (.00921-.86e-3) params(6) (.00921-.86e-3)];
%    
% if nograd
%   dx = zeros(15,0);
% else
%   dx = zeros(15,7);
%   dx(2,1) = 1;
%   dx(5,1) = 1;
%   dx(11,1) = 1;
%   dx(8,2) = 1;
%   dx(14,3) = 1;
%   dx(3,4) = 1;
%   dx(6,5) = 1;
%   dx(12,6) = 1;
%   dx(11,7) = 1;
% end
end

