function [P_pre_spindle_to_camera,P_lidar_to_post_spindle] = vector_to_poses(x,do_full)

R = rpy2rot(x(1:3));
T = x(4:6);
P_pre_spindle_to_camera = [R,T(:);0,0,0,1];

if (do_full)
    R = rpy2rot(x(7:9));
    T = [x(10:11);0];
else
    R = rpy2rot([x(7);x(8);0]);
    T = [x(9);x(10);0];
    %R = rpy2rot([x(7);0;x(8)]);
    %T = [0;x(9);x(10)];
end
P_lidar_to_post_spindle = [R,T(:);0,0,0,1];
%P_lidar_to_post_spindle = inv([R,T(:);0,0,0,1]);
