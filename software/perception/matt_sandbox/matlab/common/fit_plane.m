function p = fit_plane(pts)

avg = mean(pts,1);
pts = [pts(:,1)-avg(1),pts(:,2)-avg(2),pts(:,3)-avg(3)];
[~,~,v] = svd(pts,0);
p = [v(:,3);-sum(v(:,3).*avg(:))];
