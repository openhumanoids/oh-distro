function step_traj = planSwing(biped, last_pos, next_pos)
% Compute a collision-free swing trajectory for a single foot. Uses the biped's RigidBodyTerrain to compute a slice of terrain between the two poses.

debug = false;
planar_clearance = 0.05;
z_clearance = 0.02;
foot_speed = 0.15 / (1.3 * .75); % m/s
lift_time = 0.1 * 1.3;

% Let lambda be a variable which indicates cartesian distance along the line from last_pos to next_pos. 
step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));
lambdas = linspace(0, step_dist_xy);
lambda2xy = PPTrajectory(foh([0, step_dist_xy], [last_pos(1:2), next_pos(1:2)]));
terrain_pts = [lambdas; biped.getTerrainHeight(lambda2xy.eval(lambdas))];


% We'll expand all of our obstacles in the plane by this distance, which is the maximum allowed distance from the center of the foot to the edge of an obstacle
contact_radius = sqrt(sum((biped.foot_contact_offsets.right.toe - biped.foot_contact_offsets.right.center).^2)) + planar_clearance;

expanded_terrain_pts = zeros(2, length(terrain_pts(1,:)) * 2);
for j = 1:length(terrain_pts(1,:))
  expanded_terrain_pts(:, j * 2) = terrain_pts(:, j) + [contact_radius; z_clearance];
  expanded_terrain_pts(:, j * 2 - 1) = terrain_pts(:, j) + [-contact_radius; z_clearance];
end

% Create a single default apex pose to ensure that we at least rise by this much on flat ground
apex_pos = [step_dist_xy / 2; max([last_pos(3), next_pos(3)]) + biped.nom_step_clearance];
expanded_terrain_pts = [expanded_terrain_pts, apex_pos];

expanded_terrain_pts(1,:) = bsxfun(@max, bsxfun(@min, expanded_terrain_pts(1,:), step_dist_xy), 0);
expanded_terrain_pts = expanded_terrain_pts(:, convhull(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'simplify', true));
expanded_terrain_pts = expanded_terrain_pts(:, end:-1:1); % convert counterclockwise to clockwise convex hull

traj_pts = expanded_terrain_pts(:, 1:find(expanded_terrain_pts(1,:) >= step_dist_xy, 1, 'first'));
traj_pts = [traj_pts, [step_dist_xy; next_pos(3) + 0.005]];
traj_ts = [0, cumsum(sqrt(sum(diff(traj_pts, 1, 2).^2, 1)))] ./ foot_speed;

% Add start and end points
traj_pts = [[0; last_pos(3)], traj_pts, repmat([step_dist_xy; next_pos(3)], 1, 2)];
traj_ts = [0, traj_ts + lift_time, traj_ts(end) + lift_time, traj_ts(end) + 1.5 * lift_time];
traj_pts_xyz = [lambda2xy.eval(traj_pts(1,:)); traj_pts(2,:)];

rpy_pts = [last_pos(4:6), next_pos(4:6)];
rpy_traj = PPTrajectory(foh(traj_ts([1, end]), rpy_pts));


step_traj = PPTrajectory(foh(traj_ts, [traj_pts_xyz; rpy_traj.eval(traj_ts)]));

if debug
  figure(1)
  clf
  hold on
  plot(terrain_pts(1,:), terrain_pts(2,:), 'g.')
  plot(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'ro')

  t = linspace(traj_ts(1), traj_ts(end));
  xyz = step_traj.eval(t);
  plot(xyz(1,:) - xyz(1,1), xyz(3,:),'k')
  axis equal
end
