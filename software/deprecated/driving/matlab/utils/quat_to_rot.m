function r = quat_to_rot(q)
%
% R = QUAT_TO_ROT(Q) converts a quaternion to a direction cosine (rotation)
% matrix.
%   R = QUAT_TO_ROT(Q) returns the direction cosine (rotation) matrix, R in 
%   O(3), that corresponds to the quaternion given by Q. Per libbot
%   convention, Q is [w x y z]. Input quaternion Q does not need to be
%   normalized.
%
%   Implementation derived from
%   http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
%

error(nargchk(1, 1, nargin));
if ndims(q) > 2 || (isvector(q) && length(q) ~= 4) || (~isvector(q) && size(q,2) ~= 4)
    error('Invalid argument, Q must be a 4 element vector or Nx4 matrix.');
end
if isvector(q) && size(q,2) == 1
    q = q';
end

Nq = sum(q.^2, 2);
i = Nq < 1e-10;
s = zeros(size(q,1), 1);
if any(i)
    warning([num2str(sum(i)) ' near-zero or zero-length quaternion, assuming no rotation!']);
end
s(~i) = 2./Nq(~i);

X = q(:,2).*s; Y = q(:,3).*s; Z = q(:,4).*s;
wX = q(:,1).*X; wY = q(:,1).*Y; wZ = q(:,1).*Z;
xX = q(:,2).*X; xY = q(:,2).*Y; xZ = q(:,2).*Z;
yY = q(:,3).*Y; yZ = q(:,3).*Z; zZ = q(:,4).*Z;

r = zeros(3, 3, size(q,1));
r(1,1,:) = 1.0-(yY+zZ);
r(1,2,:) = xY-wZ;
r(1,3,:) = xZ+wY;
r(2,1,:) = xY+wZ;
r(2,2,:) = 1.0-(xX+zZ);
r(2,3,:) = yZ-wX;
r(3,1,:) = xZ-wY;
r(3,2,:) = yZ+wX;
r(3,3,:) = 1.0-(xX+yY);

end