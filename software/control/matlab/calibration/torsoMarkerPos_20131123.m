function [x, dx] = torsoMarkerPos_newmarkers(params, use_fixed)
%TORSOMARKERPOS [x, dx] = torsoMarkerPos(params)
%NOTEST
% 4 torso markers are arranged on the chest in the yz-plane
% experimenting with different parameterizations
% 11 parameters
% markers positions as of 10/24/2013
% params(5) = .072;

if nargin < 2
  use_fixed = false;
end

if use_fixed
  params = [   -0.0400
    0.1517
   -0.0040
    0.0916
    .072
    0.2166];
  dx = zeros(12,0);
else
%   params = [params(1:4);.072;params(5:end)];

  dx = zeros(12,6);
  dx(5,1) = 1;
  dx(6,2) = 1;
  dx(8,3) = 1;
  dx(9,4) = 1;
  dx(11,5) = 1;
  dx(12,6) = 1;
%   dx = [dx(:,1:4) dx(:,6:end)];

end

x = zeros(3,4);
% x(:,3) = [.2095 + .017;-.0765+.030;.2955];
% x(:,3) = [.2095 + .018;-.0765 + .0326;.2955 + .0075];
x(:,1) = [.2095 + .018;-.0765 + .0303;.2955];
x(1,[2 3 4]) = repmat(x(1,1),1,3);
x(2:3,[2 3 4]) = reshape(params,2,3);

% x(2,4) = .072;

% params = [params(1:2); -.0765 + .0326;params(3:end)];
% 
% 
% if use_fixed
%   params = [   -0.0006
%     0.0927
%    -0.0365
%     0.1532
%     0.0787
%     0.2158];
%   dx = zeros(12,0);
% else
%   dx = zeros(12,6);
%   dx(2,1) = 1;
%   dx(3,2) = 1;
%   dx(5,3) = 1;
%   dx(6,4) = 1;
%   dx(11,5) = 1;
%   dx(12,6) = 1;
% end
% 
% x = zeros(3,4);
% % x(:,3) = [.2095 + .017;-.0765+.030;.2955];
% x(:,3) = [.2095 + .018;-.0765 + .0326;.2955 + .0075];
% x(1,[1 2 4]) = repmat(x(1,3),1,3);
% x(2:3,[1 2 4]) = reshape(params,2,3);
% 
% dx = [dx(:,1:2) dx(:,4:end)];


% top right dot
%   x: 11mm from FRONT of top bar
%   y: 22m right of rightmost middle bartop (vertical bar going up)
%   z: 0mm from center of top bar
% bottom two dots centered on bars

end
