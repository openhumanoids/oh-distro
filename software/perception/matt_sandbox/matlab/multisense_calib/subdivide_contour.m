function segs = subdivide_contour(xy,max_dist)

segs = subdivide_recurse(xy,1,size(xy,1),max_dist);
segs = cell2mat(segs);


function segs = subdivide_recurse(xy,p1,p2,max_dist)

if (p2-p1<1)
    segs = {zeros(0,2)};
    return;
end

L = [xy(p1,2)-xy(p2,2), xy(p2,1)-xy(p1,1),...
    xy(p1,1)*xy(p2,2) - xy(p2,1)*xy(p1,2)];
L = L/sqrt(L(1)^2+L(2)^2);
d = xy(p1:p2,1)*L(1) + xy(p1:p2,2)*L(2) + L(3);
d2 = d.^2;
[maxval,maxind] = max(d2);
maxind = maxind+p1-1;
if (maxval > max_dist^2)
    segs1 = subdivide_recurse(xy,p1,maxind,max_dist);
    segs2 = subdivide_recurse(xy,maxind,p2,max_dist);
    segs = cat(1,segs1,segs2);
else
    segs = {[p1,p2]};
end
