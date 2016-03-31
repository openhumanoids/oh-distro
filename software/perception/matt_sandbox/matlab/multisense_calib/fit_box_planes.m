function planes = fit_box_planes(pts,max_dist)

pts_cur = pts;
planes = repmat(struct,[3,1]);
for i = 1:3
    res = ransac_plane(pts_cur,max_dist);
    planes(i).pts = pts_cur(res.inliers,:);
    planes(i).plane = res.plane;
    pts_cur = pts_cur(~res.inliers,:);
end

% reordering: left, right, bottom
all_planes = cat(2,planes.plane)';
flip_ind = all_planes(:,4)<0;
all_planes(flip_ind,:) = -all_planes(flip_ind,:);
[~,z_ind] = min(abs(all_planes(:,1)));
[~,x_ind] = max(all_planes(:,1));
[~,y_ind] = min(all_planes(:,1));
planes = planes([x_ind,y_ind,z_ind],:);
