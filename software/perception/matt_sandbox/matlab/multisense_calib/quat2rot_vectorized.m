% q in rows
function R = quat2rot_vectorized(q)

qmag = sqrt(q(:,1).^2 + q(:,2).^2 + q(:,3).^2 + q(:,4).^2);
q = q./qmag(:,[1,1,1,1]);

w = q(:,1);
x = q(:,2);
y = q(:,3);
z = q(:,4);

x2 = x.*x;  y2 = y.*y;  z2 = z.*z;  w2 = w.*w;
xy = 2*x.*y;  xz = 2*x.*z;  yz = 2*y.*z;
wx = 2*w.*x;  wy = 2*w.*y;  wz = 2*w.*z;

R11 = w2+x2-y2-z2;
R12 = xy-wz;
R13 = xz+wy;
R21 = xy+wz;
R22 = w2-x2+y2-z2;
R23 = yz-wx;
R31 = xz-wy;
R32 = yz+wx;
R33 = w2-x2-y2+z2;

R = [R11,R12,R13, R21,R22,R23, R31,R32,R33];
