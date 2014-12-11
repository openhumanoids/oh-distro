function R = bot_quat_to_matrix(quat)
    %quat = quat/norm(quat);
    
    qw=quat(1);
    qx=quat(2);
    qy=quat(3);
    qz=quat(4);
    qx2 = qx^2;
    qy2 = qy^2;
    qz2 = qz^2;
    
    R = [1 - 2*qy2 - 2*qz2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw; ...
         2*qx*qy + 2*qz*qw, 1 - 2*qx2 - 2*qz2, 2*qy*qz - 2*qx*qw; ...
         2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx2 - 2*qy2 ];

% 
%   n = quat(1+0)*quat(1+0) + quat(1+1)*quat(1+1) + quat(1+2)*quat(1+2) + quat(1+3)*quat(1+3);
% 
%   norm = 1/n;
%   x = quat(1+1)*n;
%   y = quat(1+2)*n;
%   z = quat(1+3)*n;
%   w = quat(1+0)*n;
% 
%   x2 = x*x;
%   y2 = y*y;
%   z2 = z*z;
%   w2 = w*w;
%   xy = 2*x*y;
%   xz = 2*x*z;
%   yz = 2*y*z;
%   wx = 2*w*x;
%   wy = 2*w*y;
%   wz = 2*w*z;
% 
%   %R = zeros(3,3);
%   R(1,1) = w2+x2-y2-z2;  R(1,2) = xy-wz;        R(1,3) = xz+wy;
%   R(2,1) = xy+wz;        R(2,2) = w2-x2+y2-z2;  R(2,3) = yz-wx;
%   R(3,1) = xz-wy;        R(3,2) = yz+wx;        R(3,3) = w2-x2-y2+z2;