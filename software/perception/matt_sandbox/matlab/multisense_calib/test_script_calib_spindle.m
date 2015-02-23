%%
res = result3;

%% separate two halves of sweep
half_idx = round(numel(scans)/2);
scans1 = scans(1:half_idx);
scans2 = scans(half_idx+1:end);

%% run icp
result4 = icp_half_sweeps(scans1,scans2,result3,5,0.1);

%% draw result
range_range = [0,10];
theta_range = [-360,360];
range_filter_thresh = 20;
p1 = accum_scans(scans1,result4.P_camera_to_pre_spindle, result4.P_post_spindle_to_lidar,...
    range_range, theta_range, range_filter_thresh, false);
p2 = accum_scans(scans2,result4.P_camera_to_pre_spindle, result4.P_post_spindle_to_lidar,...
    range_range, theta_range, range_filter_thresh, false);

p1_prev = accum_scans(scans1,result3.P_camera_to_pre_spindle, result3.P_post_spindle_to_lidar,...
    range_range, theta_range, range_filter_thresh, false);
p2_prev = accum_scans(scans2,result3.P_camera_to_pre_spindle, result3.P_post_spindle_to_lidar,...
    range_range, theta_range, range_filter_thresh, false);


figure
hold on
myplot3(p1,'r.','markersize',1);
myplot3(p2,'b.','markersize',1);
hold off;
axis equal
view3d on

figure
hold on
myplot3(p1_prev,'r.','markersize',1);
myplot3(p2_prev,'b.','markersize',1);
hold off;
axis equal
view3d on

figure
hold on
myplot3(p1,'r.','markersize',1);
myplot3(p2,'b.','markersize',1);
myplot3(p1_prev,'g.','markersize',1);
myplot3(p2_prev,'k.','markersize',1);
hold off;
axis equal
view3d on

%%
p1 = accum_scans(scans1,result4.P_camera_to_pre_spindle, result4.P_post_spindle_to_lidar,...
    range_range, theta_range, range_filter_thresh, true);
p2 = accum_scans(scans1,result4.P_camera_to_pre_spindle, result4.P_post_spindle_to_lidar,...
    range_range, theta_range, range_filter_thresh, false);

figure
hold on
myplot3(p1,'r.','markersize',1);
myplot3(p2,'b.','markersize',1);
hold off;
axis equal
view3d on
