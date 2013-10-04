function [x, dx] = torsoMarkerPos(params)
%TORSOMARKERPOS [x, dx] = torsoMarkerPos(params)
%NOTEST
% 4 torso markers are arranged on the chest in the yz-plane
% experimenting with different parameterizations
% 11 parameters

% x = repmat([params(1);0;0],1,5) + [0 0 0 0 0; reshape(params(2:end),2,5)];
% dx = [repmat([1;0;0],5,1) kron(eye(5),[0 0; 1 0; 0 1])];

% x = repmat([.225;0;0],1,5) + [0 0 0 0 0; reshape(params,2,5)];
% dx = kron(eye(5),[0 0; 1 0; 0 1]);

% x = zeros(3,5);
% x(1,:) = .225*ones(1,5);
% x(2:3,3) = [-.075+.026;.295-.036];
% x(2:3,1:2) = reshape(params(1:4),2,2);
% x(2:3,4:5) = reshape(params(5:8),2,2);
% 
% dx = zeros(15,8);
% dx(1:6,1:4) = kron(eye(2),[0 0; 1 0; 0 1]);
% dx(10:15,5:8) = kron(eye(2),[0 0; 1 0; 0 1]);


% Fix the position of the upper-right marker, as measured.
% Parameterize the x-position of the plane and the yz positions of the
% other four markers
x = zeros(3,5);
x(1,:) = params(9)*ones(1,5);
x(2:3,3) = [-.075+.026;.295-.036];
x(2:3,1:2) = reshape(params(1:4),2,2);
x(2:3,4:5) = reshape(params(5:8),2,2);

dx = zeros(15,9);
dx(1:6,1:4) = kron(eye(2),[0 0; 1 0; 0 1]);
dx(10:15,5:8) = kron(eye(2),[0 0; 1 0; 0 1]);
dx(:,9) = repmat([1;0;0],5,1);
end
