function step = fitStepToTerrain(biped, step)
% Take the pose of the center of the foot as pos and return an adjusted pose which matches the height and normal of the terrain under that point

pos = step.pos.inFrame(step.frames.center).double();

closest_terrain_pos = pos(1:3,:);
[closest_terrain_pos(3,:), normal] = biped.getTerrainHeight(pos(1:3,:));
normal(3,normal(3,:) < 0) = -normal(3,normal(3,:) < 0);

ground_pos = pos;

assert(~any(isnan(closest_terrain_pos)));

ground_pos(1:3,:) = closest_terrain_pos(1:3,:);
ground_pos(:,:) = fitPoseToNormal(ground_pos, normal);

step.pos = Point(step.frames.center, ground_pos);
