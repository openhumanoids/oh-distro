function [x, dx] = torsoMarkerPos_newmarkers(params, use_fixed)
%TORSOMARKERPOS [x, dx] = torsoMarkerPos(params)
%NOTEST
% 4 torso markers are arranged on the chest in the yz-plane
% experimenting with different parameterizations
% 11 parameters

if nargin < 2
  use_fixed = false;
end

if use_fixed
  params = [   -0.0035
    0.0925
   -0.0425
    0.1512
    0.0724
    0.2176];
  x = zeros(3,4);
  x(:,4) = [.2095 + .017;-.0765+.030;.2955];
  x(1,1:3) = repmat(x(1,4),1,3);
  x(2:3,1:3) = reshape(params,2,3);
  
  dx = zeros(12,0);
else

x = zeros(3,4);
x(:,4) = [.2095 + .017;-.0765+.030;.2955];
x(1,1:3) = repmat(x(1,4),1,3);
x(2:3,1:3) = reshape(params,2,3);

dx = zeros(12,6);
dx(2,1) = 1;
dx(3,2) = 1;
dx(5,3) = 1;
dx(6,4) = 1;
dx(8,5) = 1;
dx(9,6) = 1;
end

%   - top right dot
%     - x: 17mm forward center of top bar
%     - y: 30mm left from center of right vertical chest bar
%     - z: 0mm from center of top bar


% if use_fixed
% params = [    0.0533
%     0.2530
%     0.0129
%     0.2789
%     0.0149
%     0.1991
%    -0.0237
%     0.1813];
% end
% 
% x = zeros(3,5);
% x(1,:) = .225*ones(1,5); % was .225
% x(2:3,3) = [-.075+.026;.295-.036];
% x(2:3,1:2) = reshape(params(1:4),2,2);
% x(2:3,4:5) = reshape(params(5:8),2,2);
% 
% if use_fixed
%   dx = zeros(15,0);
% else
%   dx = zeros(15,8);
%   dx(1:6,1:4) = kron(eye(2),[0 0; 1 0; 0 1]);
%   dx(10:15,5:8) = kron(eye(2),[0 0; 1 0; 0 1]);
% end


end
