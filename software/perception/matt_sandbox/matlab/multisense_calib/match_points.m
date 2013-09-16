function matches = match_points(camera_pts,lidar_data,poses,P_camera_to_pre_spindle, P_post_spindle_to_lidar,tol)

lidar_pts = accum_lidar(lidar_data,poses,P_camera_to_pre_spindle, P_post_spindle_to_lidar);
lidar_pts = [lidar_pts,(1:size(lidar_pts,1))'];
camera_pts = camera_pts(:,1:3);

% find closest points
camera_index = build_spatial_index(camera_pts,tol,1);
lidar_bins = round([lidar_pts(:,1:3),ones(size(lidar_pts,1),1)]*camera_index.xform(1:3,:)');
sz = size(camera_index.vox);
good = all(lidar_bins>=1,2) & all(lidar_bins<=repmat(sz([2,1,3]),[size(lidar_bins,1),1]),2);
lidar_pts = lidar_pts(good,:);
lidar_inds = sub2ind(sz,lidar_bins(good,2),lidar_bins(good,1),lidar_bins(good,3));

cam_pt_inds = camera_index.vox(lidar_inds);
bad_cells = cellfun(@isempty,cam_pt_inds);
cam_pt_inds(bad_cells) = [];
lidar_pts(bad_cells,:) = [];

matches = {};
tol2 = tol^2;
for i = 1:size(lidar_pts,1)
    p_camera = camera_pts(cam_pt_inds{i},:);
    p_lidar = lidar_pts(i,:);
    d = (p_camera(:,1)-p_lidar(1)).^2 + (p_camera(:,2)-p_lidar(2)).^2 + (p_camera(:,3)-p_lidar(3)).^2;
    [minval,minind] = min(d);
    if (minval > tol2)
        continue;
    end
    matches{end+1,1} = [p_lidar(4),cam_pt_inds{i}(minind)];
end
matches = cell2mat(matches);
