function index = build_spatial_index(pts,res,r)

minpt = min(pts,[],1);
p = [pts(:,1)-minpt(1),pts(:,2)-minpt(2),pts(:,3)-minpt(3)]/res + 1;
p = round(p);
sz = max(p,[],1);
index.vox = cell(sz(2),sz(1),sz(3));

[xx,yy,zz] = meshgrid(-r:r,-r:r,-r:r);

all_x = repmat(p(:,1),[1,numel(xx)]) + repmat(xx(:)',[size(p,1),1]);
all_y = repmat(p(:,2),[1,numel(yy)]) + repmat(yy(:)',[size(p,1),1]);
all_z = repmat(p(:,3),[1,numel(zz)]) + repmat(zz(:)',[size(p,1),1]);
good = all_x>=1 & all_y>=1 & all_z>=1 & ...
    all_x<=sz(1) & all_y<=sz(2) & all_z<=sz(3);

pt_inds = (p(:,3)-1)*sz(1)*sz(2) + (p(:,1)-1)*sz(2) + p(:,2);
pt_inds = 1:size(p,1);
all_dest_inds = (all_z-1)*sz(1)*sz(2) + (all_x-1)*sz(2) + all_y;
clear all_x all_y all_z
all_pt_inds = repmat(pt_inds,[1,size(all_dest_inds,2)]);
all_pairs = [all_dest_inds(:), all_pt_inds(:)];
clear all_pt_inds
all_pairs = all_pairs(good,:);
clear good
all_pairs = sortrows(all_pairs,1);
d = diff(all_pairs(:,1));
starts = [1;find(d>0)+1];
ends = [starts(2:end)-1;numel(d)+1];

for i = 1:numel(starts)
    dest = all_pairs(starts(i),1);
    indices = all_pairs(starts(i):ends(i),2);
    index.vox{dest} = indices;
end

xform_to_vox = make_translation([1,1,1])*diag(1/res*[1,1,1,res])*make_translation(-minpt);
index.xform = xform_to_vox;


function T = make_translation(p)

T = eye(4);
T(1:3,4) = p(:);
