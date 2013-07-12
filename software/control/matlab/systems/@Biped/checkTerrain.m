function [ground_pos, got_data, terrain_ok, ground_pts] = checkTerrain(biped, pos, is_right_foot)
  if nargin < 3
    is_right_foot = -1;
  end
  orig = biped.footContact2Orig(pos, 'center', is_right_foot);

  if is_right_foot ~= -1
    sizecheck(pos, [6, 1]);
    if is_right_foot
      gc = biped.foot_bodies.right.contact_pts;
    else
      gc = biped.foot_bodies.left.contact_pts;
    end
    for j = 1:length(gc(1,:))
      M = makehgtform('xrotate', orig(4), 'yrotate', orig(5), 'zrotate', orig(6));
      offs = gc(:,j) * 1.25;
      d = M * [offs; 1];
      gc(:,j) = orig(1:3) + d(1:3);
    end
    ground_pts = [gc, pos(1:3,:)];
    [ground_pts(3,:), normals] = biped.getTerrainHeight(ground_pts);
    plot_lcm_points(ground_pts', zeros(length(gc(1,:))+1,3), 100, 'contact pts', 1, 1);
    max_z_dist = max(ground_pts(3,:)) - min(ground_pts(3,:));
    if any(any(isnan(ground_pts)))
      terrain_ok = false;
      got_data = false;
    else
      if max_z_dist < 0.005
        terrain_ok = true;
      else
        % fit a plane to the ground points, and make sure all the contacts will be nearly coplanar
        coeff = princomp(ground_pts');
        plane_normal = coeff(:,3)';
        cosines = [];
        for j = 1:length(normals)
          cosines(end+1) = dot(normals(:,j), plane_normal) / (norm(normals(:,j)) * norm(plane_normal));
        end
        err = abs((ground_pts' - repmat(mean(ground_pts,2)',size(ground_pts, 2),1))*plane_normal');
        terrain_ok = all(abs(cosines) > cos(10*pi/180)) && all(err < 0.01);
      end
    end
  else
    ground_pts = [0];
    terrain_ok = 0;
  end
  
  closest_terrain_pos = pos(1:3,:);
  [closest_terrain_pos(3,:), normal] = biped.getTerrainHeight(pos(1:3,:));
  if normal(3) < 0
    normal = -normal;
  end
  
  M0 = rpy2rotmat(pos(4:6));
  x0 = M0 * [1;0;0];
  z0 = M0 * [0;0;1];
  ax = cross(z0, normal);
  costheta = dot(normal, z0) / norm(normal);
  terrain_ok = terrain_ok && abs(costheta) > cos(30*pi/180);
  
  % h = zeros(1, length(pos(1,:)));
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

%           new_rpy = quat2rpy(q);
    if ~any(isnan(new_rpy))
      ground_pos(4:5) = new_rpy(1:2);
    end
  end
  got_data = ~any(isnan(closest_terrain_pos)) && ~any(any(isnan(ground_pts)));
  % ground_pos(3,:) = h;
end