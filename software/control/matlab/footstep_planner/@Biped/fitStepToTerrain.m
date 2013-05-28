function ground_pos = fitStepToTerrain(biped, pos)
% Take the pose of the center of the foot as pos and return an adjusted pose which matches the height and normal of the terrain under that point

sizecheck(pos, [6,1]);

closest_terrain_pos = pos(1:3);
[closest_terrain_pos(3), normal] = biped.getTerrainHeight(pos(1:3));
if normal(3) < 0
  normal = -normal;
end


ground_pos = pos;
if ~any(isnan(closest_terrain_pos))
  ground_pos(1:3) = closest_terrain_pos;
  ground_pos = fitPosToNormal(ground_pos, normal);
end
