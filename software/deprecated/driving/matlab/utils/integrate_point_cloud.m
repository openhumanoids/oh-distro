function [pts,scan_ids,pt_ids,orig] = integrate_point_cloud(scans, poses, sensor_R, sensor_T, range_range)

total_pts = 0;
for i = 1:numel(scans)
    total_pts = total_pts + numel(scans(i).ranges);
end

orig = zeros(total_pts,2);
scan_ids = zeros(total_pts,1);
pose_stack = zeros(3,4,numel(scans));
pt_ids = zeros(total_pts,1);
cur_pt = 1;
for i = 1:numel(scans)
    r = scans(i).ranges(:);
    t = scans(i).thetas(:);
    x = r.*cos(t);
    y = r.*sin(t);
    orig(cur_pt:cur_pt+numel(x)-1,:) = [x(:),y(:)];
    scan_ids(cur_pt:cur_pt+numel(x)-1) = i;
    pt_ids(cur_pt:cur_pt+numel(x)-1) = 1:numel(x);

    pose_stack(:,:,i) = [quat2rot(poses(i).orientation),poses(i).position(:)];

    cur_pt = cur_pt + numel(x);
end

pose_stack = reshape(shiftdim(pose_stack,2),[size(pose_stack,3),3,4]);
pts = transform_sensor_to_local([orig,scan_ids],pose_stack,sensor_R,sensor_T);

all_ranges = horzcat(scans.ranges);
bad_ind = all_ranges(:)<range_range(1) | all_ranges(:)>range_range(2);
pts(bad_ind,:) = [];
scan_ids(bad_ind) = [];
pt_ids(bad_ind) = [];
orig(bad_ind,:) = [];
