function result = optimize_poses_using_matches(lidar_data,cam_pts,poses_start,poses_end,...
    P_pre_spindle_to_camera,P_lidar_to_post_spindle,do_full)

if (~exist('do_full','var'))
    do_full = false;
end

if (~exist('P_pre_spindle_to_camera','var'))
    P_pre_spindle_to_camera = eye(4);
end
if (~exist('P_lidar_to_post_spindle','var'))
    P_lidar_to_post_spindle = eye(4);
end

lidar_data = sortrows(lidar_data,5);

prob.lidar_data = double(lidar_data);
prob.cam_pts = cam_pts;
prob.poses_start = poses_start;
prob.poses_end = poses_end;
prob.do_full = do_full;

x_init = poses_to_vector(P_pre_spindle_to_camera, P_lidar_to_post_spindle,do_full);
opts = optimset('display','iter','maxfunevals',1e6);

prob.draw = false;
x = lsqnonlin(@error_func,x_init,[],[],opts,prob);
[result.P_pre_spindle_to_camera,result.P_lidar_to_post_spindle] = vector_to_poses(x,do_full);

prob.draw = true;
result.errors = error_func(x,prob);


function e = error_func(x, prob)

[P_pre_spindle_to_camera, P_lidar_to_post_spindle] = vector_to_poses(x, prob.do_full);

all_pts = accum_lidar(prob.lidar_data,prob.poses_start,prob.poses_end,...
    P_pre_spindle_to_camera,P_lidar_to_post_spindle,true);
e = all_pts-prob.cam_pts;
e = e(:);

if (prob.draw)
    figure(22);
    myplot3(all_pts,'r.');
    hold on;
    myplot3(prob.cam_pts,'b.');
    hold off;
    axis equal;
    view3d on
end
