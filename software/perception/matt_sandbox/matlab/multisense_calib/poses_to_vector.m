function x = poses_to_vector(P_camera_to_pre_spindle,P_post_spindle_to_lidar,do_full)

x1 = [rot2rpy(P_camera_to_pre_spindle(1:3,1:3));P_camera_to_pre_spindle(1:3,4)];

P_lidar_to_post_spindle = inv(P_post_spindle_to_lidar);
%rpy = rot2rpy(P_post_spindle_to_lidar(1:3,1:3));
if (do_full)
    rpy = rot2rpy(P_lidar_to_post_spindle(1:3,1:3));
    x2 = [rpy(:); P_lidar_to_post_spindle(1:3,4)];
else
    % TODO: this sometimes gives nonzero yaw rpy = rot2rpy(P_lidar_to_post_spindle(1:3,1:3));
    rpy = rot2rpy_constrained(P_lidar_to_post_spindle(1:3,1:3));
    x2 = [rpy(1:2);P_lidar_to_post_spindle(1:2,4)];
    %x2 = [rpy([1,3]);P_post_spindle_to_lidar(2:3,4)];
end
x = [x1;x2];



function rpy = rot2rpy_constrained(R)
roll = -acos(R(2,2))*sign(R(2,3));
pitch = -acos(R(1,1))*sign(R(3,1));
rpy = [roll;pitch;0];
