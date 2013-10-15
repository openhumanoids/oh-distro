function [x, dx] = handMarkerPos_standoffs(params,nograd)
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

% h = .046; %height
w12 = .0979; %width
w35 = .0986;

% x = [-w/2 w/2 w/2 -w/2 -w/2;
%      params(1) params(1) params(2) params(1) params(3);
%      params(4) params(5) (.00921-.86e-3) params(6) (.00921-.86e-3)];
%    
% if nograd
%   dx = zeros(15,0);
% else
% dx = zeros(15,6);
% dx(2,1) = 1;
% dx(5,1) = 1;
% dx(11,1) = 1;
% dx(8,2) = 1;
% dx(14,3) = 1;
% dx(3,4) = 1;
% dx(6,5) = 1;
% dx(12,6) = 1;
% end

% x = [-w/2 w/2 w/2 -w/2 -w/2;
%      params(1) params(1) params(2) params(1) params(2)-.031;
%      params(4) params(4)-.031 (.00921-.86e-3) params(3) (.00921-.86e-3)];
%    
% if nograd
%   dx = zeros(15,0);
% else
% dx = zeros(15,4);
% dx(2,1) = 1;
% dx(5,1) = 1;
% dx(11,1) = 1;
% dx(8,2) = 1;
% dx(14,2) = 1;
% dx(3,4) = 1;
% dx(6,4) = 1;
% dx(12,3) = 1;
% end

% % up to date dimensions
% x = [-w12/2 w12/2 w35/2 -w12/2 -w35/2;
%      params(1) params(1) -.199 params(1) -.199-.0308;
%      params(2) params(2)-.0305 (.00921-.86e-3) params(3) (.00921-.86e-3)];
%    
% if nograd
%   dx = zeros(15,0);
% else
% dx = zeros(15,3);
% dx(2,1) = 1;
% dx(5,1) = 1;
% dx(11,1) = 1;
% dx(3,2) = 1;
% dx(6,2) = 1;
% dx(12,3) = 1;
% end

x = [-w12/2 w12/2 w35/2 -w12/2 -w35/2;
     params(1) params(1) params(2) params(7) params(3);
     params(4) params(5) (.00921-.86e-3) params(6) (.00921-.86e-3)];
   
if nograd
  dx = zeros(15,0);
else
  dx = zeros(15,7);
  dx(2,1) = 1;
  dx(5,1) = 1;
  dx(11,1) = 1;
  dx(8,2) = 1;
  dx(14,3) = 1;
  dx(3,4) = 1;
  dx(6,5) = 1;
  dx(12,6) = 1;
  dx(11,7) = 1;
end
end

