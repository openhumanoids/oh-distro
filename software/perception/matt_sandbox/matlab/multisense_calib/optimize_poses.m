function result = optimize_poses(data,poses,planes,...
    P_camera_to_pre_spindle,P_post_spindle_to_lidar)

if (~exist('P_camera_to_pre_spindle','var'))
    P_camera_to_pre_spindle = eye(4);
end
if (~exist('P_post_spindle_to_lidar','var'))
    P_post_spindle_to_lidar = eye(4);
end

data = sortrows(data,[4,5]);
d = diff(data(:,4));
starts = [1;find(d>0)+1];
ends = [starts(2:end)-1;numel(d)+1];

u = unique(data(:,5));
which_face = false(size(data,1),max(u));
for i = 1:numel(u)
    which_face(:,u(i)) = data(:,5)==u(i);
end

prob.starts = starts;
prob.ends = ends;
prob.faces = u;
prob.which_face = which_face;
prob.data = data;
prob.poses = poses;
prob.planes = planes;

rpy2 = rot2rpy(P_post_spindle_to_lidar(1:3,1:3));
x_init = [rot2rpy(P_camera_to_pre_spindle(1:3,1:3));P_camera_to_pre_spindle(1:3,4);
    rpy2([1,3]);P_post_spindle_to_lidar(2:3,4)];
opts = optimset('display','iter','maxfunevals',1e6);

prob.draw = false;
[x,~,~,~,~,~,jacobian] = lsqnonlin(@error_func,x_init,[],[],opts,prob);
result.jacobian = jacobian;

R = rpy2rot(x(1:3));
T = x(4:6);
result.P_camera_to_pre_spindle = [R,T(:);0,0,0,1];

R = rpy2rot([x(7);0;x(8)]);
T = [0;x(9);x(10)];
result.P_post_spindle_to_lidar = [R,T(:);0,0,0,1];

prob.draw = true;
result.errors = error_func(x,prob);


function e = error_func(x, prob)

R = rpy2rot(x(1:3));
T = x(4:6);
P_camera_to_pre_spindle = [R,T(:);0,0,0,1];

R = rpy2rot([x(7);0;x(8)]);
T = [0;x(9);x(10)];
P_post_spindle_to_lidar = [R,T(:);0,0,0,1];

if (prob.draw)
    figure(23);
    clf;
    hold on;
    colorset = [1,0,0;0,1,0;0,0,1;1,1,0;1,0,1;0,1,1];
end

counter = 1;
all_pts = zeros(size(prob.data,1),4);
for i = 1:numel(prob.starts)
    data_sub = prob.data(prob.starts(i):prob.ends(i),:);
    scan_ind = data_sub(1,4);
    P_pre_spindle_to_post_spindle = [prob.poses(scan_ind).R,prob.poses(scan_ind).T(:);0,0,0,1];
    P = P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*P_camera_to_pre_spindle;
    P(1:3,1:3) = P(1:3,1:3)';
    P(1:3,4) = -P(1:3,1:3)*P(1:3,4);
    sz = size(data_sub,1);
    all_pts(counter:counter+sz-1,:) = [data_sub(:,1:3),ones(size(data_sub,1),1)]*P';
    counter = counter+sz;
end

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