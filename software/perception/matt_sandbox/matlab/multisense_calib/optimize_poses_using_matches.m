function result = optimize_poses_using_matches(data,poses,...
    P_camera_to_pre_spindle,P_post_spindle_to_lidar,do_full)

if (~exist('do_full','var'))
    do_full = false;
end

if (~exist('P_camera_to_pre_spindle','var'))
    P_camera_to_pre_spindle = eye(4);
end
if (~exist('P_post_spindle_to_lidar','var'))
    P_post_spindle_to_lidar = eye(4);
end

data = sortrows(data,7);
d = diff(data(:,7));
starts = [1;find(d>0)+1];
ends = [starts(2:end)-1;numel(d)+1];

prob.starts = starts;
prob.ends = ends;
prob.data = double(data);
prob.poses = poses;
prob.do_full = do_full;

rpy2 = rot2rpy(P_post_spindle_to_lidar(1:3,1:3));
if (do_full)
    x_init = [rot2rpy(P_camera_to_pre_spindle(1:3,1:3));P_camera_to_pre_spindle(1:3,4);
        rpy2;P_post_spindle_to_lidar(1:3,4)];
else
    x_init = [rot2rpy(P_camera_to_pre_spindle(1:3,1:3));P_camera_to_pre_spindle(1:3,4);
        rpy2([1,3]);P_post_spindle_to_lidar(2:3,4)];
end
opts = optimset('display','iter','maxfunevals',1e6);

prob.draw = false;
x = lsqnonlin(@error_func,x_init,[],[],opts,prob);

R = rpy2rot(x(1:3));
T = x(4:6);
result.P_camera_to_pre_spindle = [R,T(:);0,0,0,1];

if (do_full)
    R = rpy2rot(x(7:9));
    T = x(10:12);
else
    R = rpy2rot([x(7);0;x(8)]);
    T = [0;x(9);x(10)];
end
result.P_post_spindle_to_lidar = [R,T(:);0,0,0,1];

prob.draw = true;
result.errors = error_func(x,prob);


function e = error_func(x, prob)

R = rpy2rot(x(1:3));
T = x(4:6);
P_camera_to_pre_spindle = [R,T(:);0,0,0,1];

if (prob.do_full)
    R = rpy2rot(x(7:9));
    T = x(10:12);
else
    R = rpy2rot([x(7);0;x(8)]);
    T = [0;x(9);x(10)];
end
P_post_spindle_to_lidar = [R,T(:);0,0,0,1];

counter = 1;
all_pts = zeros(size(prob.data,1),3);
for i = 1:numel(prob.starts)
    data_sub = prob.data(prob.starts(i):prob.ends(i),:);
    scan_ind = data_sub(1,7);
    P_pre_spindle_to_post_spindle = [prob.poses(scan_ind).R,prob.poses(scan_ind).T(:);0,0,0,1];
    P = P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*P_camera_to_pre_spindle;
    P(1:3,1:3) = P(1:3,1:3)';
    P(1:3,4) = -P(1:3,1:3)*P(1:3,4);
    sz = size(data_sub,1);
    all_pts(counter:counter+sz-1,:) = [data_sub(:,1:3),ones(size(data_sub,1),1)]*P(1:3,:)';
    counter = counter+sz;
end

if (prob.draw)
    figure(22);
    myplot3(all_pts,'r.');
    hold on;
    myplot3(prob.data(:,4:6),'b.');
    hold off;
    axis equal;
    view3d on
end

e = all_pts-prob.data(:,4:6);
e = e(:);
