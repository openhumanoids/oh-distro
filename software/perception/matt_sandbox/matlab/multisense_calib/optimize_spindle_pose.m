function result = optimize_spindle_pose(data1,data2,poses_start1,poses_end1,...
    poses_start2, poses_end2,P_pre_spindle_to_camera,P_lidar_to_post_spindle,do_full)

% data row format: range, theta, x, y, scan id, point percent

if (~exist('do_full','var'))
    do_full = false;
end

x_full = poses_to_vector(P_pre_spindle_to_camera, P_lidar_to_post_spindle, do_full);

prob.data1 = data1;
prob.data2 = data2;
prob.poses_start1 = poses_start1;
prob.poses_end1 = poses_end1;
prob.poses_start2 = poses_start2;
prob.poses_end2 = poses_end2;
prob.P_pre_spindle_to_camera = P_pre_spindle_to_camera;
prob.P_lidar_to_post_spindle = P_lidar_to_post_spindle;
prob.x_cam = x_full(1:6);
prob.do_full = do_full;

x_init = x_full(7:end);
opts = optimset('display','iter','maxfunevals',1e6);

prob.draw = false;
result.errors_init = error_func(x_init,prob);
x = lsqnonlin(@error_func,x_init,[],[],opts,prob);

[result.P_pre_spindle_to_camera, result.P_lidar_to_post_spindle] = vector_to_poses([prob.x_cam(:);x(:)],do_full);

prob.draw = true;
result.errors = error_func(x,prob);



function e = error_func(x, prob)

x_full = [prob.x_cam(:); x(:)];
[P_pre_spindle_to_camera, P_lidar_to_post_spindle] = vector_to_poses(x_full,prob.do_full);

pts1 = accum_lidar(prob.data1, prob.poses_start1, prob.poses_end1,...
    P_pre_spindle_to_camera,P_lidar_to_post_spindle,false);
pts2 = accum_lidar(prob.data2, prob.poses_start2, prob.poses_end2,...
    P_pre_spindle_to_camera,P_lidar_to_post_spindle,false);

e = pts2-pts1;
e = e(:);

if (prob.draw)
    figure(21);
    clf
    hold on
    myplot3(pts1,'r.');
    myplot3(pts2,'b.');
    %mmm = [pts1,pts2];
    %plot3(mmm(:,[1,4])',mmm(:,[2,5])',mmm(:,[3,6])','g-');
    hold off
    axis equal
    view3d on
    drawnow
end
