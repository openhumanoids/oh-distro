function ground_pos = fitStepToTerrain(biped, pos)
% Take the pose of the center of the foot as pos and return an adjusted pose which matches the height and normal of the terrain under that point

sizecheck(pos, [6,1]);

closest_terrain_pos = pos(1:3);
[closest_terrain_pos(3), normal] = biped.getTerrainHeight(pos(1:3));
if normal(3) < 0
  normal = -normal;
end

M0 = rpy2rotmat(pos(4:6));
x0 = M0 * [1;0;0];
z0 = M0 * [0;0;1];
ax = cross(z0, normal);
costheta = dot(normal, z0) / norm(normal);
ground_pos = pos;

if ~any(isnan(closest_terrain_pos))
  ground_pos(1:3) = closest_terrain_pos;
  theta = real(acos(costheta));
  q = axis2quat([ax;theta]);
  Mf = quat2rotmat(q);
  xf = Mf * x0;
  zf = normal;
  yf = cross(zf, xf);
  Mx = xf / norm(xf);
  Mz = cross(xf, yf) / norm(cross(xf, yf));
  My = cross(Mz, xf) / norm(cross(Mz, xf));
  M = [Mx, My, Mz];
  new_rpy = rotmat2rpy(M);

  if ~any(isnan(new_rpy))
    ground_pos(4:5) = new_rpy(1:2);
  end
end
