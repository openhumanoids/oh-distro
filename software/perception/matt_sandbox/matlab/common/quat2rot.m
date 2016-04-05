function R = quat2rot(q)

q = q/sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

x2 = x*x;  y2 = y*y;  z2 = z*z;  w2 = w*w;
xy = 2*x*y;  xz = 2*x*z;  yz = 2*y*z;
wx = 2*w*x;  wy = 2*w*y;  wz = 2*w*z;

R = [w2+x2-y2-z2, xy-wz, xz+wy;
     xy+wz, w2-x2+y2-z2, yz-wx;
     xz-wy, yz+wx, w2-x2-y2+z2];
