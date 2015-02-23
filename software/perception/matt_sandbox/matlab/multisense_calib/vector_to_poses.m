function [P_camera_to_pre_spindle,P_post_spindle_to_lidar] = vector_to_poses(x,do_full)

R = rpy2rot(x(1:3));
T = x(4:6);
P_camera_to_pre_spindle = [R,T(:);0,0,0,1];

if (do_full)
    R = rpy2rot(x(7:9));
    T = [x(10:11);0];
else
    R = rpy2rot([x(7);x(8);0]);
    T = [x(9);x(10);0];
    %R = rpy2rot([x(7);0;x(8)]);
    %T = [0;x(9);x(10)];
end
P_post_spindle_to_lidar = [R',-R'*T(:);0,0,0,1];
%P_post_spindle_to_lidar = [R,T(:);0,0,0,1];

