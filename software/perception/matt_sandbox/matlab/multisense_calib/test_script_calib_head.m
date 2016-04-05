%% setup
%logfile1 = '/home/antone/data/multisense_05_calib/lcmlog-2013-09-04.00';
%logfile1 = '/home/antone/data/multisense_05_calib/lcmlog-2013-09-04.01';
%logfile1 = '/home/antone/data/2013-11-15-multisense-02-calib/lcmlog-2013-11-15-15-39-robot';
%logfile1 = '/home/antone/data/2013-11-15-multisense-02-calib/lcmlog-2013-11-15-15-32-robot';
%logfile1 = '/home/antone/data/2014-03-28_multisense-02-calib/lcmlog-2014-03-28.03';
%logfile1 = '/home/antone/data/2014-07-09_multisense-05-calib/lcmlog-2014-07-09.02';
%logfile1 = '/home/antone/data/2015-01-29_multisense-47-calib/lcmlog-2015-01-29.01';
%logfile1 = '/mnt/hgfs/host-data/Projects/DRC/data/2015-02-11_multisense-02-calib/lcmlog-2015-02-11.00.stripped';
logfile1 = '/home/mfallon/logs/multisense-calibration-mit-atlas/lcmlog-2015-02-11.00.stripped';
sensor_id = 2;
addpath('../common');
setup_lcm;


%% sensor info

% lidar params
scan_frequency = 40;  % Hz
scan_fov = 270; % degrees

spin_time = 1/scan_frequency;  % sec
scan_time = scan_fov/360*spin_time*1e6;  %usec


if (sensor_id == 2)
    fx = 591.909423828125;
    fy = fx;
    cx = 512;
    cy = 512;
    baseline = 0.0700931567882511;
    K = [fx,0,cx;0,fy,cy;0,0,1];
    
    R = eye(3);
    T = [0;0;0];
    P_camera_to_pre_spindle = [R,T(:);0,0,0,1];
    shift = 0;
    R = axisangle2rot([0;0;1], pi + shift*2*pi/3);
    P_camera_to_pre_spindle(1:3,1:3) = R*P_camera_to_pre_spindle(1:3,1:3);
    
    R = [[0;0;1],[-1;0;0],[0;-1;0]];
    T = [0;0;0];
    P_lidar_to_post_spindle = [R,T(:);0,0,0,1];

elseif (sensor_id == 5)
    fx = 555.223083496093750;
    fy = fx;
    cx = 512;
    cy = 512;
    baseline = 0.0694900734051353;
    K = [fx,0,cx;0,fy,cy;0,0,1];
    
    R = eye(3);
    T = [0;0;0];
    P_camera_to_pre_spindle = [R,T(:);0,0,0,1];
    shift = 0;
    R = axisangle2rot([0;0;1],pi/2 + shift*2*pi/3);
    P_camera_to_pre_spindle(1:3,1:3) = R*P_camera_to_pre_spindle(1:3,1:3);
    
    R = [[0;0;1],[-1;0;0],[0;-1;0]];
    T = [0;0;0];
    P_lidar_to_post_spindle = [R,T(:);0,0,0,1];

elseif (sensor_id == 47)
    fx = 590.715;
    fy = fx;
    cx = 512;
    cy = 512;
    baseline = 0.0704776;
    K = [fx,0,cx;0,fy,cy;0,0,1];
    
    R = eye(3);
    T = [0;0;0];
    P_camera_to_pre_spindle = [R,T(:);0,0,0,1];
    shift = 0;
    R = axisangle2rot([0;0;1],pi/2 + shift*2*pi/3);
    P_camera_to_pre_spindle(1:3,1:3) = R*P_camera_to_pre_spindle(1:3,1:3);
    
    R = [[0;0;1],[-1;0;0],[0;-1;0]];
    T = [0;0;0];
    P_lidar_to_post_spindle = [R,T(:);0,0,0,1];
else
    error('invalid sensor id');
end

P_pre_spindle_to_camera = inv(P_camera_to_pre_spindle);
clear P_camera_to_pre_spindle P_post_spindle_to_lidar

%% read data
log_data = read_log_data(logfile1);
fprintf('done reading logs.\n');

%% synchronize spindle pose with lidar scans (and interpolate)
scans = tag_scans_with_poses(log_data.scans, log_data.poses, scan_time);

%% get point cloud from depth maps
camera_cloud = point_cloud_from_disparities(log_data.disparities, K, baseline, [0.5, 3], false);

%% fit planes to points
dist_thresh = 0.02;
camera_pts_decimated = decimate_points(camera_cloud,0.01);
camera_planes = fit_box_planes(camera_pts_decimated, dist_thresh);

for i = 1:numel(camera_planes)
    if (camera_planes(i).plane(4) < 0)
        camera_planes(i).plane = -camera_planes(i).plane;
    end
end
all_planes = cat(2,camera_planes.plane);
for i = 1:numel(camera_planes)
    p = camera_planes(i).pts;
    dots = [p,ones(size(p,1),1)]*all_planes;
    camera_planes(i).pts(any(dots<-dist_thresh,2),:) = [];
end
camera_pts = cat(1,camera_planes.pts);

figure;
hold on;
myplot3(camera_planes(1).pts,'r.');
myplot3(camera_planes(2).pts,'g.');
myplot3(camera_planes(3).pts,'b.');
hold off;
axis equal;
view3d on
xlabel('x'); ylabel('y'); zlabel('z');

%% segment out the box from the lidar
segs = cell(numel(scans),1);
max_angle = 45;
for i = 1:numel(scans)
    s = split_scan_into_runs(scans(i),[0.5,3],[-max_angle*pi/180,max_angle*pi/180], 20, 0.05);
    [~,min_theta_ind] = min(abs(scans(i).thetas));
    good_ind = s(:,1)<=min_theta_ind & s(:,2)>=min_theta_ind;
    s = s(good_ind,:);

    if (numel(s)>0)
        xy = scans(i).xy(s(1):s(2),:);
        s_lines = subdivide_contour(xy,0.05);
        s_lines = s_lines+s(1)-1;
        s = s_lines;
    end

    segs{i} = s;
    
    if (false)
        figure(22);
        clf
        hold on;
        for j = 1:size(s,1)
            myplot(scans(i).xy(s(j,1):s(j,2),:),'.','color',rand(1,3));
        end
        hold off;
        axis equal;
        grid on;
        drawnow;
    end
end


%% assign segments to faces using approximate reprojection

% form data matrix
lidar_data = {};
cur_seg_id = 1;
for i = 1:numel(segs)
    s = segs{i};
    n = numel(scans(i).ranges);
    pct = linspace(0,1,n);
    cur_data = [scans(i).ranges(:),scans(i).thetas(:),scans(i).xy,repmat(i,[n,1]),pct(:)];
    for j = 1:size(s,1)
        inds = s(j,1):s(j,2);
        lidar_data{end+1,1} = [cur_data(inds,:),repmat(cur_seg_id,[numel(inds),1])];
        cur_seg_id = cur_seg_id+1;
    end
end
lidar_data = cell2mat(lidar_data);

% accumulate 3d points using current approximate transform
poses_start = cat(1,scans.pose_start);
poses_end = cat(1,scans.pose_end);
lidar_pts = accum_lidar(lidar_data,poses_start,poses_end,P_pre_spindle_to_camera,P_lidar_to_post_spindle);

% fit planes using ransac
pts_cur = lidar_pts;
data_cur = lidar_data;
clear planes;
for i = 1:3
    result = ransac_plane(pts_cur(:,1:3),0.05);
    planes(i).pts = pts_cur(result.inliers,:);
    planes(i).plane = result.plane;
    planes(i).data = data_cur(result.inliers,:);
    pts_cur = pts_cur(~result.inliers,:);
    data_cur = data_cur(~result.inliers,:);
end

% assign planes to correct orientations
lidar_planes = cat(2,planes.plane)';
flip_ind = lidar_planes(:,4)<0;
lidar_planes(flip_ind,:) = -lidar_planes(flip_ind,:);
[~,z_ind] = min(abs(lidar_planes(:,1)));
[~,x_ind] = max(lidar_planes(:,1));
[~,y_ind] = min(lidar_planes(:,1));
planes = planes([x_ind,y_ind,z_ind]);
lidar_planes = cat(2,planes.plane)';

% now assign each segment to a plane
[lidar_data,sort_ind] = sortrows(lidar_data,7);
lidar_pts = lidar_pts(sort_ind,:);
lidar_data = [lidar_data,zeros(size(lidar_data,1),1)];
lidar_data(:,7:8) = lidar_data(:,[8,7]);
starts = [1;find(diff(lidar_data(:,8))>0)+1];
ends = [starts(2:end)-1;size(lidar_data,1)];
for i = 1:numel(starts)
    inds = starts(i):ends(i);
    data_sub = lidar_data(inds,:);
    p = lidar_pts(inds,:);
    d = [p(:,1:3),ones(size(p,1),1)]*lidar_planes';
    d2 = sum(d.^2,1);
    [~,minind] = min(d2);
    lidar_data(inds,7) = minind;
end

p1 = lidar_pts(lidar_data(:,7)==1,:);
p2 = lidar_pts(lidar_data(:,7)==2,:);
p3 = lidar_pts(lidar_data(:,7)==3,:);
figure;
hold on;
myplot3(p1,'r.');
myplot3(p2,'g.');
myplot3(p3,'b.');
hold off;
axis equal;
view3d
xlabel('x');
ylabel('y');
zlabel('z');


%% optimize
result = optimize_poses(lidar_data,poses_start,poses_end,cat(2,camera_planes.plane),P_pre_spindle_to_camera,P_lidar_to_post_spindle,false);

%% re-assign points
lidar_pts = accum_lidar(lidar_data,poses_start,poses_end,...
    result.P_pre_spindle_to_camera,result.P_lidar_to_post_spindle);
d = [lidar_pts,ones(size(lidar_pts,1),1)]*cat(2,camera_planes.plane);
[minval,minidx] = min(d.^2,[],2);
lidar_data2 = lidar_data;
lidar_data2(:,7) = minidx;
bad_ind = minval>0.05^2;
lidar_data2(bad_ind,:) = [];
lidar_pts(bad_ind,:) = [];

p1 = lidar_pts(lidar_data2(:,7)==1,:);
p2 = lidar_pts(lidar_data2(:,7)==2,:);
p3 = lidar_pts(lidar_data2(:,7)==3,:);
figure;
hold on;
myplot3(p1,'r.');
myplot3(p2,'g.');
myplot3(p3,'b.');
hold off;
axis equal;
view3d
xlabel('x');
ylabel('y');
zlabel('z');

%% re-optimize
result2 = optimize_poses(lidar_data2,poses_start,poses_end,cat(2,camera_planes.plane),result.P_pre_spindle_to_camera,result.P_lidar_to_post_spindle,false);

%% iterative refinement loop using direct matches
res = result2;
max_iters = 5;
for iter = 1:max_iters
    fprintf('starting iter %d...\n', iter);
    matches = match_points(camera_pts_decimated,lidar_data2,poses_start,poses_end,...
        res.P_pre_spindle_to_camera,res.P_lidar_to_post_spindle,0.03);
    lidar_match_data = lidar_data2(matches(:,1),1:3);
    camera_match_data = camera_pts_decimated(matches(:,2),1:3);
    data = [lidar_match_data,camera_match_data,lidar_data2(matches(:,1),4)];
    res = optimize_poses_using_matches(lidar_data2(matches(:,1),:),camera_pts_decimated(matches(:,2),1:3),...
        poses_start,poses_end,res.P_pre_spindle_to_camera,res.P_lidar_to_post_spindle,false);
end
result3 = res;

%return

%% get pixel data
%res = result3;
all_pts = accum_scans(scans,res.P_pre_spindle_to_camera,res.P_lidar_to_post_spindle,...
    [0.25,3],[-45,45], 30);
img = log_data.imgs(1).img;
pix = all_pts(:,1:3)*K';
pix = pix(:,1:2)./pix(:,[3,3]);
figure, myplot3(all_pts(:,1:3),'r.'); axis equal; view3d

%% output as pcl cloud
rgb = impixel(img,pix(:,1),pix(:,2));
xyzrgb = [all_pts(:,1:3),rgb/255];
%savepcd('/tmp/xyzrgb_orig.pcd',xyzrgb');

%% output camera + lidar as cloud
rgb = impixel(img,pix(:,1),pix(:,2));
xyzrgb = [all_pts(:,1:3),rgb/255];
all_pts2 = accum_scans(scans,res.P_pre_spindle_to_camera,res.P_lidar_to_post_spindle,[1,10],[-1000,1000],30);
xyzrgb = [all_pts2(:,1:3), ones(size(all_pts2,1),3);xyzrgb];
%savepcd('/tmp/xyzrgb_all.pcd',xyzrgb');

%% show all lidar points superimposed on image
plot_points_on_image(all_pts(:,1:3),K,img);

%% show all camera points superimposed on image
plot_points_on_image(camera_cloud,K,img);


%% export
q = rot2quat(res.P_pre_spindle_to_camera(1:3,1:3));
fprintf('\npre spindle to camera:\n');
fprintf('translation = [ %.15f, %.15f, %.15f ];\n', res.P_pre_spindle_to_camera(1:3,4));
fprintf('quat = [ %.15f, %.15f, %.15f, %.15f ];\n', q);

q = rot2quat(res.P_lidar_to_post_spindle(1:3,1:3));
fprintf('lidar to post spindle:\n');
fprintf('translation = [ %.15f, %.15f, %.15f ];\n', res.P_lidar_to_post_spindle(1:3,4));
fprintf('quat = [ %.15f, %.15f, %.15f, %.15f ];\n', q);
