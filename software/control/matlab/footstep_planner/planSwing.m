function [step_traj, takeoff_time, landing_time] = planSwing(biped, last_pos, next_pos, apex_pos)
% Compute a collision-free swing trajectory for a single foot. Uses the biped's RigidBodyTerrain to compute a slice of terrain between the two poses.

debug = false;
planar_clearance = 0.05;
nom_z_clearance = 0.02;
foot_speed = 0.5 / (1.3 * .75); % m/s
hold_time = 0.1 * 1.3;
ramp_distance = 0.03; % m

% Let lambda be a variable which indicates cartesian distance along the line from last_pos to next_pos. 
step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));
lambdas = linspace(0, step_dist_xy);
lambda2xy = PPTrajectory(foh([0, step_dist_xy], [last_pos(1:2), next_pos(1:2)]));
terrain_pts = [lambdas; biped.getTerrainHeight(lambda2xy.eval(lambdas))];


% We'll expand all of our obstacles in the plane by this distance, which is the maximum allowed distance from the center of the foot to the edge of an obstacle
contact_radius = sqrt(sum((biped.foot_contact_offsets.right.toe - biped.foot_contact_offsets.right.center).^2)) + planar_clearance;

% expanded_terrain_pts = zeros(2, length(terrain_pts(1,:)) * 2);
expanded_terrain_pts = [terrain_pts(:,1)];
for j = 1:length(terrain_pts(1,:))
  % if terrain_pts(1, j) < ramp_distance
  %   z_clearance = nom_z_clearance * (terrain_pts(1, j) / ramp_distance);
  % elseif terrain_pts(1, j) > (step_dist_xy - ramp_distance)
  %   z_clearance = nom_z_clearance * (step_dist_xy - terrain_pts(1,j)) / ramp_distance;
  % else
  %   z_clearance = nom_z_clearance;
  % end
  z_clearance = nom_z_clearance;
  if terrain_pts(2, j) > (min([last_pos(3), next_pos(3)]) + 0.02)
    expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [-contact_radius; z_clearance];
    expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [contact_radius; z_clearance];
  end
end
expanded_terrain_pts = [expanded_terrain_pts, terrain_pts(:,end)];

% Create a single default apex pose to ensure that we at least rise by this much on flat ground
if nargin < 3
  apex_pos = [step_dist_xy / 2; max([last_pos(3), next_pos(3)]) + biped.nom_step_clearance];
else
  apex_lambda = dot(apex_pos(1:2) - last_pos(1:2), next_pos(1:2) - last_pos(1:2)) / step_dist_xy;
  apex_pos = [apex_lambda; apex_pos(3)];
end
expanded_terrain_pts = [expanded_terrain_pts, apex_pos];

expanded_terrain_pts(1,:) = bsxfun(@max, bsxfun(@min, expanded_terrain_pts(1,:), step_dist_xy), 0);
expanded_terrain_pts = expanded_terrain_pts(:, convhull(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'simplify', true));
expanded_terrain_pts = expanded_terrain_pts(:, end:-1:1); % convert counterclockwise to clockwise convex hull

traj_pts = expanded_terrain_pts(:, 1:find(expanded_terrain_pts(1,:) >= step_dist_xy, 1, 'first'));
traj_pts = [[0; last_pos(3)], traj_pts, [step_dist_xy; next_pos(3) + 0.005]];
traj_ts = [0, cumsum(sqrt(sum(diff(traj_pts, 1, 2).^2, 1)))] ./ foot_speed;

% Add start and end points
traj_pts = [[0; last_pos(3)], traj_pts, [step_dist_xy; next_pos(3)]];
traj_ts = [0, traj_ts + hold_time, traj_ts(end) + 2.5*hold_time];
landing_time = traj_ts(end-1);
takeoff_time = traj_ts(2);
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
