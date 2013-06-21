function [C] = q2C(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

C = eye(3);

q = q/norm(q);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

x2 = x*x;
y2 = y*y;
z2 = z*z;
w2 = w*w;
xy = 2*x*y;
xz = 2*x*z;
yz = 2*y*z;
wx = 2*w*x;
wy = 2*w*y;
wz = 2*w*z;

C(1,1) = w2+x2-y2-z2;  C(1,2) = xy-wz;  C(1,3) = xz+wy;
C(2,1) = xy+wz;  C(2,2) = w2-x2+y2-z2;  C(2,3) = yz-wx;
C(3,1) = xz-wy;  C(3,2) = yz+wx;  C(3,3) = w2-x2-y2+z2;

end

