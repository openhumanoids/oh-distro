function [swing_ts, swing_poses, takeoff_time, landing_time] = planSwing(biped, last_pos, next_pos, options)
% Compute a collision-free swing trajectory for a single foot. Uses the biped's RigidBodyTerrain to compute a slice of terrain between the two poses.

if ~isfield(options, 'foot_speed')
  options.foot_speed = 0.5; % m/s
end
if ~isfield(options, 'step_height')
  options.step_height = biped.nom_step_clearance; %m
end

apex_pos = biped.findApexPos(last_pos, next_pos, options.step_height);

debug = false;

planar_clearance = 0.05;
nom_z_clearance = 0.02;
hold_frac = 0.2; % fraction of leg swing time spent shifting weight to stance leg
% ramp_distance = 0.03; % m
pre_contact_height = 0.005; % height above the ground to aim for when foot is landing
foot_yaw_rate = 1; % rad/s

unwrapped = unwrap([last_pos(6), next_pos(6)]);
next_pos(6) = unwrapped(2);

swing_angle = atan2(next_pos(2) - last_pos(2), next_pos(1) - last_pos(1));
phi.last = last_pos(6) - swing_angle;
phi.next = next_pos(6) - swing_angle;
contact_pts.last = quat2rotmat(axis2quat([0;0;1;phi.last])) * biped.foot_bodies.right.contact_pts;
contact_pts.next = quat2rotmat(axis2quat([0;0;1;phi.next])) * biped.foot_bodies.right.contact_pts; 
effective_width = max([max(contact_pts.last(2,:)) - min(contact_pts.last(2,:)),...
                       max(contact_pts.next(2,:)) - min(contact_pts.next(2,:))]);
effective_length = max([max(contact_pts.last(1,:)) - min(contact_pts.last(1,:)),...
                        max(contact_pts.next(1,:)) - min(contact_pts.next(1,:))]);
effective_height = (max([effective_length, effective_width])/2) / sqrt(2) + nom_z_clearance;

contact_length = effective_length / 2 + planar_clearance;
contact_width = effective_width / 2 + planar_clearance;

step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));

if step_dist_xy > 0.01
  % Create a single default apex pose to ensure that we at least rise by this much on flat ground
  apex_pos_l = [step_dist_xy / 2; apex_pos(3)];
  
  % % We'll expand all of our obstacles in the plane by this distance, which is the maximum allowed distance from the center of the foot to the edge of an obstacle
  % contact_radius = sqrt(sum((biped.foot_contact_offsets.right.toe - biped.foot_contact_offsets.right.center).^2)) + planar_clearance;
  
  % Let lambda be a variable which indicates cartesian distance along the line from last_pos to next_pos in the xy plane.
  lambdas = linspace(0, step_dist_xy);
  lambda2xy = PPTrajectory(foh([0, step_dist_xy], [last_pos(1:2), next_pos(1:2)]));
  terrain_pts = terrainSample(biped, last_pos, next_pos, contact_width, 25, 10);
  terrain_pts(2,1) = max([terrain_pts(2,1), last_pos(3)]);
  terrain_pts(2,end) = max([terrain_pts(2,end), next_pos(3)]);
  
  expanded_terrain_pts = [terrain_pts(:,1)];
  for j = 1:length(terrain_pts(1,:))
    if terrain_pts(2, j) > (min([last_pos(3), next_pos(3)]) + options.step_height / 2)
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [-contact_length; effective_height];
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [contact_length; effective_height];
    end
  end
  expanded_terrain_pts = [expanded_terrain_pts, terrain_pts(:,end), apex_pos_l];

  expanded_terrain_pts(1,:) = bsxfun(@max, bsxfun(@min, expanded_terrain_pts(1,:), step_dist_xy), 0);
  expanded_terrain_pts = expanded_terrain_pts(:, convhull(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'simplify', true));
  expanded_terrain_pts = expanded_terrain_pts(:, end:-1:1); % convert counterclockwise to clockwise convex hull

  traj_pts = expanded_terrain_pts(:, 1:find(expanded_terrain_pts(1,:) >= step_dist_xy, 1, 'first'));

  traj_pts = [[0; last_pos(3)], traj_pts, [step_dist_xy; next_pos(3) + pre_contact_height]];

  traj_pts_xyz = [lambda2xy.eval(traj_pts(1,:)); traj_pts(2,:)];
else
  traj_pts = [[0; last_pos(3)], [0.5; apex_pos(3)], [1; next_pos(3)]];
  traj_pts_xyz = [last_pos(1:3), apex_pos(1:3), next_pos(1:3)];
end

traj_dts = max([sqrt(sum(diff(traj_pts_xyz, 1, 2).^2, 1)) / options.foot_speed;
                diff(traj_pts(1,:)) * abs(next_pos(6) - last_pos(6)) / foot_yaw_rate],[],1);
traj_ts = [0, cumsum(traj_dts)] ;
traj_pts_xyz = [last_pos(1:3), traj_pts_xyz, next_pos(1:3)];

hold_time = traj_ts(end) * hold_frac;
traj_ts = [0, traj_ts + hold_time, traj_ts(end) + 2.5*hold_time]; % add time for weight shift
landing_time = traj_ts(end-1);
takeoff_time = traj_ts(2);

% rpy_pts = [last_pos(4:6), next_pos(4:6)];
rpy_pts = [last_pos(4:6), interp1(traj_ts([2,end-1]), [last_pos(4:6), next_pos(4:6)]', traj_ts(2:end-1))', next_pos(4:6)];
% rpy_pts(:,3:end-2) = nan;
% rpy_traj = PPTrajectory(foh(traj_ts, rpy_pts));


% step_traj = PPTrajectory(foh(traj_ts, [traj_pts_xyz; rpy_traj.eval(traj_ts)]));
% step_traj = PPTrajectory(foh(traj_ts, [traj_pts_xyz; rpy_pts]));
swing_poses = [traj_pts_xyz; rpy_pts];
swing_ts = traj_ts;


if debug
  figure(1)
  clf
  hold on
  plot(terrain_pts(1,:), terrain_pts(2,:), 'g.')
  plot(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'ro')

  t = linspace(traj_ts(1), traj_ts(end));
  xyz = step_traj.eval(t);
  plot(sqrt(sum(bsxfun(@minus, xyz(1:2,:), xyz(1:2,1)).^2)), xyz(3,:),'k')
  axis equal
end

end

function terrain_pts = terrainSample(biped, last_pos, next_pos, contact_width, nlambda, nrho)
  step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));
  lambda_hat = (next_pos(1:2) - last_pos(1:2)) / step_dist_xy;

  rho_hat = [0, -1; 1, 0] * lambda_hat;

  terrain_pts = zeros(2, nlambda);
  lambdas = linspace(0, step_dist_xy, nlambda);
  rhos = linspace(-contact_width, contact_width, nrho);
  % figure(1)
  % clf
  % hold on
  for j = 1:nlambda
    terrain_pts(1,j) = lambdas(j);
    xy = bsxfun(@plus, bsxfun(@times, rhos, rho_hat),lambdas(j) * lambda_hat + last_pos(1:2));
  %   plot(xy(1,:), xy(2,:), 'bo')
    terrain_pts(2,j) = max(medfilt1(biped.getTerrainHeight(xy)));
%     plot_lcm_points([xy;biped.getTerrainHeight(xy)]', repmat([1 0 1], size(xy, 2), 1), 101, 'Swing terrain pts', 1, 1);
  end
end
