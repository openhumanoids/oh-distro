function T= HT(p,roll,pitch,yaw)
T = zeros(4);
M = rotz(yaw)*roty(pitch)*rotx(roll);

T(1:3,1:3) = M;
T(1:4,4) = [p(:); 1];
end