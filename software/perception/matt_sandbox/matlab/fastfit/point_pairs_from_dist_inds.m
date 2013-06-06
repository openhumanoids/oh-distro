function [p0,p1] = point_pairs_from_dist_inds(dist_inds,pts,R,T)

sz = size(dist_inds);
to_model = [R,T(:)];
pts_cur = [pts,ones(size(pts,1),1)]*to_model';
pts_int = round(pts_cur);
good = all(pts_int>=1,2) & (pts_int(:,1)<=sz(2)) & (pts_int(:,2)<=sz(1)) & (pts_int(:,3)<=sz(3));
inds = sub2ind(size(dist_inds),pts_int(good,2),pts_int(good,1),pts_int(good,3));
[y,x,z] = ind2sub(size(dist_inds),dist_inds(inds));
p0 = double([x,y,z]);
p1 = pts_cur(good,:);
