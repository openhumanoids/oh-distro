function [dist_xform, world_to_vol, dist_inds] = create_dist_xform(pts,res,padding)

[vol,world_to_vol] = create_voxels(pts,res,padding);
[dist_xform,dist_inds] = bwdist(vol);
