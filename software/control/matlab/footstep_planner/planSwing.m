function [swing_ts, swing_poses, takeoff_time, landing_time] = planSwing(biped, last_pos, next_pos, options)
% Compute a collision-free swing trajectory for a single foot. Uses the biped's RigidBodyTerrain to compute a slice of terrain between the two poses.

if ~isfield(options, 'foot_speed')
  options.foot_speed = 0.5; % m/s
end
if ~isfield(options, 'step_height')
  options.step_height = biped.nom_step_clearance; %m
end
if ~isfield(options, 'ignore_terrain')
  options.ignore_terrain = false;
end

apex_pos = biped.findApexPos(last_pos, next_pos, options.step_height);

debug = false;

planar_clearance = 0.05;
ignore_height = 0.5; % m, height above which we'll assume that our heightmap is giving us bad data (e.g. returns from an object the robot is carrying)
nom_z_clearance = options.step_height;
hold_frac = 0.2; % fraction of leg swing time spent shifting weight to stance leg
min_hold_time = 0.1; % s
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
effective_height = (max([effective_length, effective_width])/2) / sqrt(2); % assumes the foot never rotates past 45 degrees in the world frame, which we enforce with the toe_height constraint later
apex_pos(3) = apex_pos(3) + effective_height;

% % We'll expand all of our obstacles in the plane by this distance, which is the maximum allowed distance from the center of the foot to the edge of an obstacle
contact_length = effective_length / 2 + planar_clearance;
contact_width = effective_width / 2 + planar_clearance;
contact_height = effective_height + nom_z_clearance;

step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));

if (step_dist_xy > 0.01 && ~options.ignore_terrain)
  % Create a single default apex pose to ensure that we at least rise by this much on flat ground
  apex_pos_l = [step_dist_xy / 2; apex_pos(3)];
  
 
  % Let lambda be a variable which indicates cartesian distance along the line from last_pos to next_pos in the xy plane.
%   lambda2xy = PPTrajectory(foh([0, step_dist_xy], [last_pos(1:2), next_pos(1:2)]));

  %% Grab the max height of the terrain across the width of the foot from last_pos to next_pos
  terrain_pts = terrainSample(biped, last_pos, next_pos, contact_width, max([ceil(step_dist_xy / 0.02),3]), 10);
  terrain_pts(2,1) = max([terrain_pts(2,1), last_pos(3)]);
  terrain_pts(2,end) = max([terrain_pts(2,end), next_pos(3)]);
  if any(terrain_pts(2,:) > (max([last_pos(3), next_pos(3)]) + ignore_height))
    % If we're getting extremely high terrain heights, then assume it's bad lidar data
    terrain_pts(2,:) = max([last_pos(3), next_pos(3)]);
  end
  
  %% Expand terrain convex hull by the size of the foot
  % expanded_terrain_pts = [terrain_pts(:,1)];
  % expanded_terrain_pts = [[0;last_pos(3) + takeoff_height], apex_pos_l, [step_dist_xy; next_pos(3) + takeoff_height]];
  expanded_terrain_pts = [[0;last_pos(3)], apex_pos_l, [step_dist_xy; next_pos(3) + pre_contact_height]];
  for j = 1:length(terrain_pts(1,:))
    if terrain_pts(2, j) > (j / length(terrain_pts(1,:))) * (next_pos(3) - last_pos(3)) + last_pos(3) + (options.step_height / 2)
    % if terrain_pts(2, j) > (min([last_pos(3), next_pos(3)]) + options.step_height / 2)
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [-contact_length; contact_height];
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [contact_length; contact_height];
    end
  end
  % expanded_terrain_pts = [expanded_terrain_pts, terrain_pts(:,end), apex_pos_l];
  % expanded_terrain_pts = [expanded_terrain_pts, apex_pos_l];
  expanded_terrain_pts(1,:) = bsxfun(@max, bsxfun(@min, expanded_terrain_pts(1,:), step_dist_xy), 0);
  expanded_terrain_pts = expanded_terrain_pts(:, convhull(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'simplify', true));
  expanded_terrain_pts = expanded_terrain_pts(:, end:-1:1); % convert counterclockwise to clockwise convex hull

  %% Draw our swing trajectory around the convex hull of the expanded terrain
  traj_pts = expanded_terrain_pts(:, 1:find(expanded_terrain_pts(1,:) >= step_dist_xy, 1, 'first'));

  % add points just after takeoff and before landing
  % traj_pts = [[0; last_pos(3) + takeoff_height], traj_pts, [step_dist_xy; next_pos(3) + takeoff_height]];
  % add start and end points
  traj_pts = [[0; last_pos(3)], traj_pts, [step_dist_xy; next_pos(3) + pre_contact_height]];
else
  %% Just ignore the terrain and provide an apex pose
  step_dist_xy = 1;
  traj_pts = [[0; last_pos(3)], [0.5;apex_pos(3)], [1;next_pos(3) + pre_contact_height]];
  % traj_pts_xyz = [last_pos(1:3), apex_pos(1:3), next_pos(1:3) + [0;0;pre_contact_height]];
end
traj_pts_xyz = [last_pos(1) + (next_pos(1) - last_pos(1)) * traj_pts(1,:) / step_dist_xy;
                last_pos(2) + (next_pos(2) - last_pos(2)) * traj_pts(1,:) / step_dist_xy;
                traj_pts(2,:)];


%% Compute time required for swing from cartesian distance of poses as well as yaw distance
d_dist = sqrt(sum(diff(traj_pts_xyz, 1, 2).^2, 1));
total_dist_traveled = sum(d_dist);
traj_dts = max([d_dist / options.foot_speed;
                (d_dist / total_dist_traveled) .* (abs(angleDiff(next_pos(6), last_pos(6))) / foot_yaw_rate)],[],1);
traj_ts = [0, cumsum(traj_dts)] ;

%% Add time to shift weight
hold_time = traj_ts(end) * hold_frac;
hold_time = max([hold_time, min_hold_time])
traj_ts = [0, traj_ts + hold_time, traj_ts(end) + 2.5*hold_time]; % add time for weight shift
traj_pts = [traj_pts(:,1), traj_pts, traj_pts(:,end)];
traj_pts_xyz = [last_pos(1:3), traj_pts_xyz, next_pos(1:3)];
landing_time = traj_ts(end-1);
takeoff_time = traj_ts(2);

%% Interpolate in rpy to constrain the foot orientation. We may set these values to NaN later to free up the foot orientation
rpy_pts = [last_pos(4:6), interp1(traj_ts([2,end-1]), [last_pos(4:6), next_pos(4:6)]', traj_ts(2:end-1))', next_pos(4:6)];

swing_poses.center = [traj_pts_xyz; rpy_pts];

toe_start = biped.footOrig2Contact(biped.footContact2Orig(last_pos, 'center', 1), 'toe', 1);
toe_end = biped.footOrig2Contact(biped.footContact2Orig(next_pos, 'center', 1), 'toe', 1);
toe.min = max([traj_pts(2,:) - effective_height; 
               (traj_pts(1,:) / step_dist_xy) * (toe_end(3) - toe_start(3)) + toe_start(3)]);
toe.min = [nan*ones(2, length(toe.min)); toe.min];
toe.max = nan * toe.min;

heel_start = biped.footOrig2Contact(biped.footContact2Orig(last_pos, 'center', 1), 'heel', 1);
heel_end = biped.footOrig2Contact(biped.footContact2Orig(next_pos, 'center', 1), 'heel', 1);
heel.min = max([traj_pts(2,:) - effective_height;
                (traj_pts(1,:) / step_dist_xy) * (heel_end(3) - heel_start(3)) + heel_start(3)]);
heel.min = [nan*ones(2,length(heel.min)); heel.min];
heel.max = nan * heel.min;

% % Minimum height that we'll allow the toe (or any contact point on the bottom of the foot) to reach. This assumes that the right and left foot have the same contact points
% contact_zO_offs = min(biped.foot_bodies.right.contact_pts(3,:));
% toe_heights = max([traj_pts_xyz(3,:) - effective_height; 
%                    getTerrainHeight(biped, traj_pts_xyz(1:2,:))]);
% % make sure we don't ask the contact points to be above their lowest possible point
% toe_heights = min([toe_heights; traj_pts_xyz(3,:)]);


swing_poses.toe = toe;
swing_poses.heel = heel;
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
  [R, L] = meshgrid(rhos, lambdas);
  xy = bsxfun(@plus, last_pos(1:2), bsxfun(@times, reshape(R, 1, []), rho_hat) + bsxfun(@times, reshape(L, 1, []), lambda_hat));
  z = medfilt2(reshape(biped.getTerrainHeight(xy), size(R)), 'symmetric');
%   plot_lcm_points([xy; reshape(z, 1, [])]', repmat([1 0 1], size(xy, 2), 1), 101, 'Swing terrain pts', 1, 1);
  terrain_pts(2, :) = max(z, [], 2);
  terrain_pts(1,:) = lambdas;
end
