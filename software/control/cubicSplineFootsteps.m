function [Xright, Xleft, XY] = cubicSplineFootsteps(start_pos, goal_pos, step_length, step_width)

p0 = start_pos(1:2);
pf = goal_pos(1:2) - p0;
initial_angle = start_pos(6)
R = [cos(-initial_angle), -sin(-initial_angle); ...
  sin(-initial_angle), cos(-initial_angle)];
pf = R * pf;
final_angle = goal_pos(6) - initial_angle

dydx0 = 0;
dydxf = tan(final_angle);

pp = pchipDeriv([0, pf(1)], [0, pf(2)], [dydx0, dydxf]);

x = linspace(0, pf(1), 100);
y = ppval(pp, x);
dydx = diff(y) ./ diff(x);


lambda = cumsum(sqrt([0, diff(x)].^2 + [0, diff(y)].^2));
num_steps = int32((lambda(end) - lambda(1)) / (step_length / 2));
lambdai = linspace(lambda(1), lambda(end), num_steps);

ndx_r = int32([1, 2, 4:2:(num_steps-1), num_steps]);
ndx_l = int32([1:2:(num_steps-1), num_steps]);

% lambdai_r = [lambdai(1:2), lambdai(4:2:(end-1)), lambdai(end)];
% lambdai_l = [lambdai(1:2:(end-1)), lambdai(end)];

xyi = interp1(lambda, [x', y'], lambdai);
dydxi = interp1(lambda, [0;dydx'], lambdai)';
unit_v_along_length = [1 ./ sqrt(1 + dydxi.^2), dydxi ./ sqrt(1 + dydxi.^2)];
v_to_left = [-unit_v_along_length(:,2), unit_v_along_length(:,1)] .* (step_width / 2);
v_to_right = [unit_v_along_length(:,2), -unit_v_along_length(:,1)] .* (step_width / 2);

Xright = [xyi(ndx_r, :) + v_to_right(ndx_r, :)] + repmat(p0', length(ndx_r), 1);
Xright = Xright * inv(R)';
yaw_r = atan2(unit_v_along_length(ndx_r, 2), unit_v_along_length(ndx_r, 1));
Xright = [Xright'; repmat(0, 3, length(Xright(:,1))); yaw_r'];
Xleft = [xyi(ndx_l, :) + v_to_left(ndx_l, :)] + repmat(p0', length(ndx_l), 1);
Xleft = Xleft * inv(R)';
yaw_l = atan2(unit_v_along_length(ndx_l, 2), unit_v_along_length(ndx_l, 1));
Xleft = [Xleft'; repmat(0, 3, length(Xleft(:,1))); yaw_l'];
XY = ([x',y'] + repmat(p0', length(x), 1)) * inv(R)';