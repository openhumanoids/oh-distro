function [x, dx] = handMarkerPos(params)
%HANDMARKERPOS [x, dx] = handMarkerPos(params)
%
% five markers on the hand, 4 in xz-plane
% second point is out of plane
% [1 3 4 5] is in order
%
% experimenting here with different parameterizations


h = .046; %height
w = .099; %width

% params(1:3) is center of box, (4:6) is position of marker 2
% x = [params(1:3) params(4:6) repmat(params(1:3),1,3)] + .5*[w 0 w -w -w ;zeros(1,5); -h 0 h h -h];
% dx = zeros(15,6);
% dx([1:3 7:end],1:3) = kron(ones(4,1),eye(3));
% dx(4:6,4:6) = eye(3);

% Fix x-position of box to 0.  Should be true.
% x = [[0; params(1:2)] params(3:5) repmat([0; params(1:2)],1,3)] + .5*[w 0 w -w -w ;zeros(1,5); -h 0 h h -h];
% dx = zeros(15,5);
% dx([1:3 7:end],1:2) = kron(ones(4,1),[0 0;eye(2)]);
% dx(4:6,3:5) = eye(3);

% Fix x=0 and z=.00921, from URDF wrist joint.
x = [[0; params(1); 0.00921] params(2:4) repmat([0; params(1); 0.00921],1,3)] + .5*[w 0 w -w -w ;zeros(1,5); -h 0 h h -h];
dx = zeros(15,4);
dx([1:3 7:end],1) = kron(ones(4,1),[0;1;0]);
dx(4:6,2:4) = eye(3);

end

