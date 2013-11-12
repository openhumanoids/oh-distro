function [x, dx] = torsoMarkerPos(params, use_fixed)
%TORSOMARKERPOS [x, dx] = torsoMarkerPos(params)
%NOTEST
% 4 torso markers are arranged on the chest in the yz-plane
% experimenting with different parameterizations
% 11 parameters

if nargin < 2
  use_fixed = false;
end

% x = repmat([params(1);0;0],1,5) + [0 0 0 0 0; reshape(params(2:end),2,5)];
% dx = [repmat([1;0;0],5,1) kron(eye(5),[0 0; 1 0; 0 1])];

% x = repmat([.225;0;0],1,5) + [0 0 0 0 0; reshape(params,2,5)];
% dx = kron(eye(5),[0 0; 1 0; 0 1]);


% params = [    0.0534
%     0.2514
%     0.0134
%     0.2781
%     0.0141
%     0.1982
%    -0.0249
%     0.1811];

if use_fixed
%   params = [
%     0.0532
%     0.2523
%     0.0130
%     0.2785
%     0.0144
%     0.1987
%    -0.0244
%     0.1812];
params = [    0.0533
    0.2530
    0.0129
    0.2789
    0.0149
    0.1991
   -0.0237
    0.1813];
end

x = zeros(3,5);
x(1,:) = .225*ones(1,5); % was .225
x(2:3,3) = [-.075+.026;.295-.036];
x(2:3,1:2) = reshape(params(1:4),2,2);
x(2:3,4:5) = reshape(params(5:8),2,2);

if use_fixed
  dx = zeros(15,0);
else
  dx = zeros(15,8);
  dx(1:6,1:4) = kron(eye(2),[0 0; 1 0; 0 1]);
  dx(10:15,5:8) = kron(eye(2),[0 0; 1 0; 0 1]);
end

% Fix the position of the upper-right marker, as measured.
% Parameterize the x-position of the plane and the yz positions of the
% other four markers
% x = zeros(3,5);
% x(1,:) = params(9)*ones(1,5);
% x(2:3,3) = [-.075+.026;.295-.036];
% x(2:3,1:2) = reshape(params(1:4),2,2);
% x(2:3,4:5) = reshape(params(5:8),2,2);
% 
% dx = zeros(15,9);
% dx(1:6,1:4) = kron(eye(2),[0 0; 1 0; 0 1]);
% dx(10:15,5:8) = kron(eye(2),[0 0; 1 0; 0 1]);
% dx(:,9) = repmat([1;0;0],5,1);
end
