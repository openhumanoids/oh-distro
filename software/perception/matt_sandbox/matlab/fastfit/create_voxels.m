function [vol,world_to_vol] = create_voxels(pts,res,padding)

if (~exist('padding','var'))
    padding = 0;
end


pt_min = min(pts);
pts_int = round((pts - repmat(pt_min,[size(pts,1),1]))/res) + 1 + padding;
world_to_vol = eye(4);
world_to_vol = [eye(3),[-1;-1;-1];0,0,0,1]*world_to_vol;
world_to_vol = [eye(3),[-padding;-padding;-padding];0,0,0,1]*world_to_vol;
world_to_vol = diag([res,res,res,1])*world_to_vol;
world_to_vol = [eye(3),pt_min(:);0,0,0,1]*world_to_vol;
world_to_vol = inv(world_to_vol);
sz = ceil(max(pts_int)) + padding;
vol = false(sz(2),sz(1),sz(3));
inds = sub2ind(size(vol),pts_int(:,2),pts_int(:,1),pts_int(:,3));
vol(inds) = true;
