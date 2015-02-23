function x = poses_to_vector(P_pre_spindle_to_camera,P_lidar_to_post_spindle,do_full)

x1 = [rot2rpy(P_pre_spindle_to_camera(1:3,1:3));P_pre_spindle_to_camera(1:3,4)];

%rpy = rot2rpy(P_post_spindle_to_lidar(1:3,1:3));
if (do_full)
    rpy = rot2rpy(P_lidar_to_post_spindle(1:3,1:3));
    x2 = [rpy(:); P_lidar_to_post_spindle(1:2,4)];
else
    % TODO: this sometimes gives nonzero yaw rpy = rot2rpy(P_lidar_to_post_spindle(1:3,1:3));
    rpy = rot2rpy_constrained(P_lidar_to_post_spindle(1:3,1:3));
    x2 = [rpy(1:2);P_lidar_to_post_spindle(1:2,4)];
    %P_post_spindle_to_lidar = inv(P_lidar_to_post_spindle);
    %x2 = [rpy([1,3]);P_post_spindle_to_lidar(2:3,4)];
end
x = [x1;x2];



function rpy = rot2rpy_constrained(R)
roll = -acos(R(2,2))*sign(R(2,3));
pitch = -acos(R(1,1))*sign(R(3,1));
rpy = [roll;pitch;0];
