function [Xright, Xleft, XY] = cubicSplineFootsteps(start_pos, goal_pos, step_length, step_width)
	
traj = cubicSplineTraj(start_pos, goal_pos);
lambda = traj.lambda;
total_dist = lambda(end) - lambda(1);
num_steps = int32(total_dist / (step_length / 2));
lambdai = linspace(lambda(1), lambda(end), num_steps);
lambdai = lambdai ./ lambda(end);

[xyi, dydxi] = traj.eval(lambdai);

unit_v_along_length = dydxi;

ndx_r = int32([1, 2, 4:2:(num_steps-1), num_steps]);
ndx_l = int32([1:2:(num_steps-1), num_steps]);

v_to_left = [-unit_v_along_length(:,2), unit_v_along_length(:,1)] .* (step_width / 2);
v_to_right = [unit_v_along_length(:,2), -unit_v_along_length(:,1)] .* (step_width / 2);

% figure(22)
% clf
% hold on
% quiver(xyi(:,1), xyi(:,2), v_to_left(:,1), v_to_left(:,2), .2)
% quiver(xyi(:,1), xyi(:,2), v_to_right(:,1), v_to_right(:,2), .2)
% axis equal

Xright = [xyi(ndx_r, :) + v_to_right(ndx_r, :)];
yaw_r = atan2(unit_v_along_length(ndx_r, 2), unit_v_along_length(ndx_r, 1));
Xright = [Xright'; repmat(0, 3, length(Xright(:,1))); yaw_r'];
Xleft = [xyi(ndx_l, :) + v_to_left(ndx_l, :)];
yaw_l = atan2(unit_v_along_length(ndx_l, 2), unit_v_along_length(ndx_l, 1));
Xleft = [Xleft'; repmat(0, 3, length(Xleft(:,1))); yaw_l'];
XY = traj.X;

% figure(23)
% quiver(Xright(1,:)', Xright(2,:)', unit_v_along_length(ndx_r,1), unit_v_along_length(ndx_r,2))
% axis equal