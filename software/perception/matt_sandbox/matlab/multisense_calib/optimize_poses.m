function result = optimize_poses(data,poses_start,poses_end,planes,...
    P_pre_spindle_to_camera,P_lidar_to_post_spindle,do_full)

% data row format: range, theta, x, y, scan id, point percent, face

if (~exist('P_pre_spindle_to_camera','var'))
    P_pre_spindle_to_camera = eye(4);
end
if (~exist('P_lidar_to_post_spindle','var'))
    P_lidar_to_post_spindle = eye(4);
end
if (~exist('do_full','var'))
    do_full = false;
end

data = sortrows(data,5);

u = unique(data(:,7));
which_face = false(size(data,1),max(u));
for i = 1:numel(u)
    which_face(:,u(i)) = data(:,7)==u(i);
end

prob.faces = u;
prob.which_face = which_face;
prob.data = data;
prob.poses_start = poses_start;
prob.poses_end = poses_end;
prob.planes = planes;
prob.do_full = do_full;

x_init = poses_to_vector(P_pre_spindle_to_camera, P_lidar_to_post_spindle, do_full);
opts = optimset('display','iter','maxfunevals',1e6);

prob.draw = false;
[x,~,~,~,~,~,jacobian] = lsqnonlin(@error_func,x_init,[],[],opts,prob);
result.jacobian = jacobian;

[result.P_pre_spindle_to_camera, result.P_lidar_to_post_spindle] = vector_to_poses(x,do_full);

prob.draw = true;
result.errors = error_func(x,prob);


function e = error_func(x, prob)

[P_pre_spindle_to_camera, P_lidar_to_post_spindle] = vector_to_poses(x,prob.do_full);

if (prob.draw)
    figure(23);
    clf;
    hold on;
    colorset = [1,0,0;0,1,0;0,0,1;1,1,0;1,0,1;0,1,1];
end

all_pts = accum_lidar(prob.data, prob.poses_start, prob.poses_end,...
    P_pre_spindle_to_camera,P_lidar_to_post_spindle,true);
all_pts = [all_pts,ones(size(all_pts,1),1)];

e = zeros(size(prob.data,1),1);
for i = 1:numel(prob.faces)
    f = prob.faces(i);
    e(prob.which_face(:,f)) = all_pts(prob.which_face(:,f),:)*prob.planes(:,f);

    if (prob.draw)
        myplot3(all_pts(prob.which_face(:,f),1:3),'.','color',colorset(f,:));
    end
end

if (prob.draw)
    hold off;
    axis equal;
    view3d on;
end
