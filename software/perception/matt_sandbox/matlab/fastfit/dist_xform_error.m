function e = dist_xform_error(dist_xform, pts, R,T,ext_val)

to_model = [R,T(:)];
pts = [pts,ones(size(pts,1),1)]*to_model';
d = interp3(dist_xform, pts(:,1),pts(:,2),pts(:,3),'linear',ext_val);
e = d(:);
