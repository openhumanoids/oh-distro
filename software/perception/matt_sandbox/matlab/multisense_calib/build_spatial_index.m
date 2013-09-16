function index = build_spatial_index(pts,res,r)

minpt = min(pts,[],1);
p = [pts(:,1)-minpt(1),pts(:,2)-minpt(2),pts(:,3)-minpt(3)]/res + 1;
p = round(p);
sz = max(p,[],1);
index.vox = cell(sz(2),sz(1),sz(3));

inds = sub2ind(sz([2,1,3]),p(:,2),p(:,1),p(:,3));
[sorted_inds,sort_idx] = sort(inds);
d = diff(sorted_inds);
starts = [1;find(d>0)+1];
ends = [starts(2:end)-1;numel(d)+1];
for i = 1:numel(starts)
    inds_sub = sort_idx(starts(i):ends(i));
    central_bin = sorted_inds(starts(i));
    [y,x,z] = ind2sub(size(index.vox),central_bin);
    [xx,yy,zz] = meshgrid(x+(-r:r),y+(-r:r),z+(-r:r));
    good = xx>=1 & xx<=size(index.vox,2) & ...
        yy>=1 & yy<=size(index.vox,1) & ...
        zz>=1 & zz<=size(index.vox,3);
    bins = sub2ind(size(index.vox),yy(good),xx(good),zz(good));
    for j = 1:numel(bins)
        index.vox{bins(j)} = [index.vox{bins(j)};inds_sub];
    end
end

xform_to_vox = make_translation([1,1,1])*diag(1/res*[1,1,1,res])*make_translation(-minpt);
index.xform = xform_to_vox;


function T = make_translation(p)

T = eye(4);
T(1:3,4) = p(:);
