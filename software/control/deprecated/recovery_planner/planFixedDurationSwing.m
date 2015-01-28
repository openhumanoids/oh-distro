function [swing_ts, swing_poses, takeoff_time, landing_time] = planFixedDurationSwing(~, sw0, sw1, options)

if nargin < 4
  options = struct();
end
if ~isfield(options, 'step_speed'); options.step_speed = -0.5; end
if ~isfield(options, 'swing_height'); options.swing_height = 0.05; end

assert(options.step_speed < 0)
dt = -options.step_speed;
hold_time = min([0.1, 0.4 * dt]);

step_dist_xy = norm(sw1(1:2) - sw0(1:2));
h1 = options.swing_height + sw0(3);
goal = [step_dist_xy; sw1(3)];

well_strength = 1e-2;
field_strength = 1e3;

function xd = dynamics(t,x)
  xd(1,:) = ones(size(x(1,:))) .* sign(goal(1) - x(1,:));
  xd(1,:) = min([1.0*ones(size(xd(1,:)));xd(1,:)]);
  xd(1,:) = max([-1.0*ones(size(xd(1,:)));xd(1,:)]);
  xd(1,:) = xd(1,:) * step_dist_xy / (dt - hold_time);
  line_diff = interp1([0, goal(1)], [sw0(3), sw1(3)], x(1,:)) + h1 - x(2,:);
  % line_pull = field_strength * (h1 - x(2,:)) .* abs(h1 - x(2,:));
  line_pull = field_strength * line_diff .* abs(line_diff);
  line_pull = min([1.0*ones(size(line_pull));line_pull]);
  line_pull = max([-1.0*ones(size(line_pull));line_pull]);
  well_pull = - well_strength * 1 ./ abs(x(1,:) - goal(1)).^(1);
  well_pull = min([1.5*ones(size(well_pull)); well_pull]);
  well_pull = max([-1.5*ones(size(well_pull)); well_pull]);
  xd(2,:) = line_pull + well_pull;
end


x = linspace(-.1, 1.1,50);
y = linspace(-0.05, 0.2,50);
[X, Y] = meshgrid(x,y);
Z = dynamics([],[reshape(X,1,[]); reshape(Y,1,[])]);
u = reshape(Z(1,:), size(X));
v = reshape(Z(2,:), size(X));
max(max(v))

quiver(X,Y,u,v);
hold on

start = [0 0; 0 0.05; 0.2 0.1; 0.8 0.01]';
for j = 1:size(start,2)
  [t, q] = ode45(@dynamics, [0, abs(1-start(1,j))], start(:,j));
  q = q';
  plot(q(1,:), q(2,:), 'r-','LineWidth',3);
end

[t,q] = ode45(@dynamics, [0, dt-hold_time], [0, sw0(3)]);
q = q'; t = t';
plot(q(1,:), q(2,:), 'g-','LineWidth',3);

swing_ts = [t, dt];
traj_pts = [q, [step_dist_xy; sw1(3)]];
traj_pts_xyz = [sw0(1) + (sw1(1) - sw0(1)) * traj_pts(1,:) / step_dist_xy;
                sw0(2) + (sw1(2) - sw0(2)) * traj_pts(1,:) / step_dist_xy;
                traj_pts(2,:)];
%% Interpolate in rpy to constrain the foot orientation. We may set these values to NaN later to free up the foot orientation
rpy_pts = [sw0(4:6), interp1(swing_ts([2,end-1]), [sw0(4:6), sw1(4:6)]', swing_ts(2:end-1))', sw1(4:6)];


swing_poses.center = [traj_pts_xyz; rpy_pts];

takeoff_time = hold_time;
landing_time = dt - hold_time;

xlim([-.1, 1.1])
ylim([-0.05, 0.2])

end