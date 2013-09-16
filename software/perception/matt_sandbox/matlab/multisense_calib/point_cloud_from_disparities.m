function pts = point_cloud_from_disparities(disparities, K, baseline, depth_range)

sum_img = zeros(size(disparities(1).img));
count_img = sum_img;
for i = 1:numel(disparities)
    d = disparities(i).img;
    count_img = count_img + (d>0);
    sum_img = sum_img + d;
end
disparity_avg = sum_img./count_img;
good_ind = count_img>0;

% unproject points
depths = baseline*K(1,1)./disparity_avg;
good_ind = good_ind & depths>=depth_range(1) & depths<=depth_range(2);
[x,y] = meshgrid(0:size(depths,2)-1,0:size(depths,1)-1);
rays = [x(good_ind),y(good_ind),ones(sum(good_ind(:)),1)]*inv(K)';
pts = rays.*repmat(depths(good_ind)./rays(:,3),[1,3]);
