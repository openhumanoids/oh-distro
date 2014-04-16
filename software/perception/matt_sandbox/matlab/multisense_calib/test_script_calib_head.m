%% setup
%logfile1 = '/home/antone/data/multisense_05_calib/lcmlog-2013-09-04.00';
%logfile1 = '/home/antone/data/multisense_05_calib/lcmlog-2013-09-04.01';
%logfile1 = '/home/antone/data/2013-11-15-multisense-02-calib/lcmlog-2013-11-15-15-39-robot';
%logfile1 = '/home/antone/data/2013-11-15-multisense-02-calib/lcmlog-2013-11-15-15-32-robot';
logfile1 = '/home/antone/data/2014-03-28_multisense-02-calib/lcmlog-2014-03-28.03';
addpath('/home/antone/matlab/common');
setup_lcm;

%% stereo calib data and initial transforms (this is for sensor 05)
fx = 588.7705688476562;
fy = fx;
cx = 512;
cy = 272;
baseline = 0.070069553;
K = [fx,0,cx;0,fy,cy;0,0,1];

R = [0,1,0;-1,0,0;0,0,1];
T = [0;0;0];
P_camera_to_pre_spindle = [R,T(:);0,0,0,1];
R = [0,0,1;1,0,0;0,1,0];
T = [0;0;0];
P_post_spindle_to_lidar = [R,T(:);0,0,0,1];

%% stereo calib data and initial transforms (this is for sensor 02)
fx = 591.909423828125;
fy = fx;
cx = 512;
cy = 512;
baseline = 0.0700931567882511;
K = [fx,0,cx;0,fy,cy;0,0,1];

R = [0,1,0;-1,0,0;0,0,1]*[0,-1,0;1,0,0;0,0,1];
T = [0;0;0];
P_camera_to_pre_spindle = [R,T(:);0,0,0,1];
R = [0,0,1;1,0,0;0,1,0];
T = [0;0;0];
P_post_spindle_to_lidar = [R,T(:);0,0,0,1];


%% read data
log_data1 = read_log_data(logfile1);
%log_data2 = read_log_data(logfile2);
fprintf('done reading logs.\n');

%% synchronize spindle pose with lidar scans (and interpolate)
poses = synchronize_poses(log_data1.poses, cat(1,log_data1.scans.timestamp));

%% get point cloud from depth maps
camera_cloud = point_cloud_from_disparities(log_data1.disparities, K, baseline, [0.5, 3], false);

%% fit planes to points
dist_thresh = 0.02;
camera_pts_decimated = decimate_points(camera_cloud,0.01);
%camera_pts_decimated = chop_points(camera_pts_decimated,[-1,-1,1],[0.4,0.6,2.1]); % TODO: specific to one data set (02)
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
scans = log_data1.scans;
segs = cell(numel(scans),1);
max_angle = 45;
for i = 1:numel(scans)
    s = split_scan_into_runs(scans(i),[0.5,3],[-max_angle*pi/180,max_angle*pi/180], 20, 0.05);
    thetas = scans(i).theta_min + scans(i).theta_step*(0:numel(scans(i).ranges)-1);
    [~,min_theta_ind] = min(abs(thetas));
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
lidar_pts = {};
for i = 1:numel(segs)
    P_pre_spindle_to_post_spindle = [poses(i).R,poses(i).T(:);0,0,0,1];
    P = inv(P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*P_camera_to_pre_spindle);
    for j = 1:size(segs{i},1)
        p = scans(i).xy(segs{i}(j,1):segs{i}(j,2),:);
        p = [p,zeros(size(p,1),1),ones(size(p,1),1)]*P(1:3,:)';
        lidar_pts{end+1,1} = [p,i*ones(size(p,1),1)];
    end
end
lidar_pts = cell2mat(lidar_pts);

pts_cur = lidar_pts;
clear planes;
for i = 1:3
    result = ransac_plane(pts_cur(:,1:3),0.05);
    planes(i).pts = pts_cur(result.inliers,:);
    planes(i).plane = result.sol;
    pts_cur = pts_cur(~result.inliers,:);
end

lidar_planes = cat(2,planes.plane)';
flip_ind = lidar_planes(:,4)<0;
lidar_planes(flip_ind,:) = -lidar_planes(flip_ind,:);
[~,z_ind] = min(abs(lidar_planes(:,1)));
[~,x_ind] = max(lidar_planes(:,1));
[~,y_ind] = min(lidar_planes(:,1));
planes = planes([x_ind,y_ind,z_ind]);


% now assign segment at a time
lidar_planes = cat(2,planes.plane)';
lidar_pts = {};
lidar_data = {};
for i = 1:numel(segs)
    P_pre_spindle_to_post_spindle = [poses(i).R,poses(i).T(:);0,0,0,1];
    P = inv(P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*P_camera_to_pre_spindle);
    for j = 1:size(segs{i},1)
        xy = scans(i).xy(segs{i}(j,1):segs{i}(j,2),:);
        xy = [xy,zeros(size(xy,1),1)];
        p = [xy,ones(size(xy,1),1)]*P(1:3,:)';
        p = [p,i*ones(size(p,1),1)];
        d = [p(:,1:3),ones(size(p,1),1)]*lidar_planes';
        d2 = sum(d.^2,1);
        [~,minind] = min(d2);
        lidar_pts{end+1,1} = [p,minind*ones(size(xy,1),1)];
        lidar_data{end+1,1} = [xy,i*ones(size(xy,1),1),minind*ones(size(xy,1),1)];
    end
end
lidar_pts = cell2mat(lidar_pts);
lidar_data = cell2mat(lidar_data);

p1 = lidar_pts(lidar_pts(:,5)==1,:);
p2 = lidar_pts(lidar_pts(:,5)==2,:);
p3 = lidar_pts(lidar_pts(:,5)==3,:);
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
result = optimize_poses(lidar_data,poses,cat(2,camera_planes.plane),P_camera_to_pre_spindle,P_post_spindle_to_lidar);

%% re-assign points
lidar_pts = {};
for i = 1:numel(segs)
    P_pre_spindle_to_post_spindle = [poses(i).R,poses(i).T(:);0,0,0,1];
    P = inv(result.P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*result.P_camera_to_pre_spindle);
    for j = 1:size(segs{i},1)
        xy = scans(i).xy(segs{i}(j,1):segs{i}(j,2),:);
        p = [xy,zeros(size(xy,1),1),ones(size(xy,1),1)]*P';
        lidar_pts{end+1,1} = p;
    end
end
lidar_pts = cell2mat(lidar_pts);

d = lidar_pts*cat(2,camera_planes.plane);
[minval,minidx] = min(d.^2,[],2);
lidar_data2 = lidar_data;
lidar_data2(:,5) = minidx;
lidar_data2(minval>0.05^2,:) = [];

%% re-optimize
result2 = optimize_poses(lidar_data2,poses,cat(2,camera_planes.plane),result.P_camera_to_pre_spindle,result.P_post_spindle_to_lidar);

%% iterative refinement loop using direct matches
res = result2;
max_iters = 5;
for iter = 1:max_iters
    fprintf('starting iter %d...\n', iter);
    matches = match_points(camera_pts_decimated,lidar_data2,poses,res.P_camera_to_pre_spindle,res.P_post_spindle_to_lidar,0.03);
    data = [lidar_data2(matches(:,1),1:3),camera_pts_decimated(matches(:,2),1:3),lidar_data2(matches(:,1),4)];
    res = optimize_poses_using_matches(data,poses,res.P_camera_to_pre_spindle,res.P_post_spindle_to_lidar,false);
end
result3 = res;


%% animate
%res = result3;
scans = log_data.scans;
imgs = log_data.imgs;
poses2 = synchronize_poses(log_data.poses,cat(1,log_data.scans.timestamp));
figure;
all_pts = {};
for i = 1:numel(scans)
    P_pre_spindle_to_post_spindle = [poses2(i).R,poses2(i).T(:);0,0,0,1];
    P = inv(res.P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*res.P_camera_to_pre_spindle);
    p = [scans(i).xy,zeros(size(scans(i).xy,1),1)];
    p = [p,ones(size(p,1),1)]*P(1:3,:)';
    bad_ind = scans(i).ranges<0.25 | scans(i).ranges>3 | scans(i).thetas<-45*pi/180 | scans(i).thetas>45*pi/180;
    p(bad_ind,:) = [];
    intensities = scans(i).intensities;
    intensities(bad_ind,:) = [];
    all_pts{i,1} = [p,intensities(:)];
    pix = p*K';
    pix = pix(:,1:2)./pix(:,[3,3]);

    if (true)
        imshow(imgs(1).img);
        hold on;
        myplot(pix,'r.');
        hold off;
        drawnow;
    end
    %pause(0.1);
end

all_pts = cell2mat(all_pts);
pix = all_pts(:,1:3)*K';
pix = pix(:,1:2)./pix(:,[3,3]);
figure, plot3k(all_pts(:,1:3),'Marker',{'.',1}); axis equal; view3d

%% output as pcl cloud
rgb = impixel(imgs(1).img,pix(:,1),pix(:,2));
xyzrgb = [all_pts(:,1:3),rgb/255];
savepcd('/home/antone/xyzrgb_orig.pcd',xyzrgb');

%% show all lidar points superimposed on image
plot_points_on_image(all_pts(:,1:3),K,imgs(1).img);

%% show all camera points superimposed on image
plot_points_on_image(camera_cloud,K,imgs(1).img);

%% set original values to compare
translation = [ -0.000000, 0.018196, 0.000000 ];
quat = [ 0.494188, -0.496592, -0.505789, -0.503341 ];
P = [quat2rot(quat),translation(:);0,0,0,1];
result_orig.P_post_spindle_to_lidar = inv(P);
translation = [ 0.035, -0.0907, -0.0083];
rpy = [0.2998, -1.3998, 88.5263];
P = [rpy2rot(rpy*pi/180),translation(:);0,0,0,1];
result_orig.P_camera_to_pre_spindle = inv(P);

%%
depth_range = [0.5,3];
poses2 = synchronize_poses(log_data2.poses, cat(1, log_data2.scans.timestamp));
dd = {};
for i = 1:numel(log_data2.scans)
    s = log_data2.scans(i);
    xy = s.xy;
    xy(s.ranges>depth_range(2) | s.ranges<depth_range(1),:) = [];
    dd{i,1} = [xy,zeros(size(xy,1),1),i*ones(size(xy,1),1)];
end
dd = cell2mat(dd);
cur_pts = point_cloud_from_disparities(log_data2.disparities, K, baseline, depth_range);
ref_pts = accum_lidar(dd, poses2, eye(4), res.P_post_spindle_to_lidar);

%%
rez = 0.01;
pose_ident.R = eye(3);
pose_ident.T = [0;0;0];
P = res.P_camera_to_pre_spindle;
pose_init.R = P(1:3,1:3);
pose_init.T = P(1:3,4);
[~,world_to_vol,dist_inds] = create_dist_xform(ref_pts, rez, 10);

pts = [cur_pts,ones(size(cur_pts,1),1)]*P(1:3,:)';
pts = decimate_points(pts,rez);
pts = [pts,ones(size(pts,1),1)]*world_to_vol';
pts = pts(:,1:3);
result = optimize_pose_with_directions(dist_inds,pts,pose_ident);

P = [result.pose.R,result.pose.T(:);0,0,0,1];
P = world_to_vol\P*world_to_vol*[pose_init.R,pose_init.T(:);0,0,0,1];
pose.R = P(1:3,1:3);
pose.T = P(1:3,4);
P_camera_to_pre_spindle = inv([pose.R,pose.T(:);0,0,0,1]);


%% export
P_lidar_to_post_spindle = inv(res.P_post_spindle_to_lidar);
q = rot2quat(P_lidar_to_post_spindle(1:3,1:3));
fprintf('lidar to post spindle:\n');
fprintf('translation = [ %.15f, %.15f, %.15f ];\n', P_lidar_to_post_spindle(1:3,4));
fprintf('quat = [ %.15f, %.15f, %.15f, %.15f ];\n', q);

P_pre_spindle_to_camera = inv(res.P_camera_to_pre_spindle);
q = rot2quat(P_pre_spindle_to_camera(1:3,1:3));
fprintf('\npre spindle to camera:\n');
fprintf('translation = [ %.15f, %.15f, %.15f ];\n', P_pre_spindle_to_camera(1:3,4));
fprintf('quat = [ %.15f, %.15f, %.15f, %.15f ];\n', q);
