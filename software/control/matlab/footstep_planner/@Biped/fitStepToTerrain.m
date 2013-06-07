function ground_pos = fitStepToTerrain(biped, pos, contact_grp)
% Take the pose of the center of the foot as pos and return an adjusted pose which matches the height and normal of the terrain under that point

if nargin < 3
  contact_grp = 'center';
end
needs_xform = ~strcmp(contact_grp, 'center');

if needs_xform
  pos = biped.footOrig2Contact(biped.footContact2Orig(pos, contact_grp, 1), 'center', 1);
end

sizecheck(pos, [6,1]);

closest_terrain_pos = pos(1:3);
[closest_terrain_pos(3), normal] = biped.getTerrainHeight(pos(1:3));
if normal(3) < 0
  normal = -normal;
end


ground_pos = pos;
if ~any(isnan(closest_terrain_pos))
  ground_pos(1:3) = closest_terrain_pos;
  ground_pos = fitPoseToNormal(ground_pos, normal);
end

if needs_xform
  ground_pos = biped.footOrig2Contact(biped.footContact2Orig(ground_pos, 'center', 1), contact_grp, 1);
end