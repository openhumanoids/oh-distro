function all_pts = accum_scans(scans,P_camera_to_pre_spindle,P_post_spindle_to_lidar,range_range, theta_range, range_filter_thresh)

if (~exist('range_filter_thresh','var'))
    range_filter_thresh = 0;
end

theta_range = theta_range*pi/180;
range_filter_thresh = range_filter_thresh*pi/180;

% data row format: range, theta, x, y, scan id, point percent, extra....
data_blocks = cell(numel(scans),1);
for i = 1:numel(scans)
    r = scans(i).ranges;
    t = scans(i).thetas;
    xy = scans(i).xy;
    scan_id = repmat(i,[numel(r),1]);
    pct = linspace(0,1,numel(r));
    data_block = [r(:),t(:),xy,scan_id,pct(:),scans(i).intensities(:)];
    bad_ind = scans(i).ranges<range_range(1) | scans(i).ranges>range_range(2) | scans(i).thetas<theta_range(1) | scans(i).thetas>theta_range(2);
    if (range_filter_thresh>0)
        p1 = scans(i).xy(1:end-2,:);
        p2 = scans(i).xy(2:end-1,:);
        p3 = scans(i).xy(3:end,:);
        delta1 = (p1-p2);
        delta1 = delta1./repmat(sqrt(sum(delta1.^2,2)),[1,2]);
        delta2 = (p3-p2);
        delta2 = delta2./repmat(sqrt(sum(delta2.^2,2)),[1,2]);
        ray = p2./repmat(sqrt(sum(p2.^2,2)),[1,2]);
        angle1 = acos(sum(ray.*delta1,2));
        angle1(angle1>pi/2) = pi-angle1(angle1>pi/2);
        angle2 = acos(sum(ray.*delta2,2));
        angle2(angle2>pi/2) = pi-angle2(angle2>pi/2);
        bad_ind(2:end-1) = bad_ind(2:end-1) | ((angle1<range_filter_thresh & angle2<range_filter_thresh));
    end
    data_blocks{i} = data_block(~bad_ind,:);
end
data_blocks = cell2mat(data_blocks);

poses_start = cat(1,scans.pose_start);
poses_end = cat(1,scans.pose_end);

all_pts = accum_lidar(data_blocks,poses_start,poses_end,P_camera_to_pre_spindle,P_post_spindle_to_lidar);
all_pts = [all_pts,data_blocks(:,[7,5])];
