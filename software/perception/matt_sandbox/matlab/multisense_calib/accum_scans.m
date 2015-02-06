function all_pts = accum_scans(scans,poses,P_post_spindle_to_lidar,P_camera_to_pre_spindle,range_range, theta_range)

all_pts = cell(numel(scans),1);
for i = 1:numel(scans)
    P_pre_spindle_to_post_spindle = [poses(i).R,poses(i).T(:);0,0,0,1];
    P = inv(P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*P_camera_to_pre_spindle);
    p = [scans(i).xy,zeros(size(scans(i).xy,1),1)];
    p = [p,ones(size(p,1),1)]*P(1:3,:)';
    bad_ind = scans(i).ranges<range_range(1) | scans(i).ranges>range_range(2) | scans(i).thetas<theta_range(1) | scans(i).thetas>theta_range(2);
    p(bad_ind,:) = [];
    intensities = scans(i).intensities;
    intensities(bad_ind,:) = [];
    all_pts{i,1} = [p,intensities(:)];
end

all_pts = cell2mat(all_pts);
