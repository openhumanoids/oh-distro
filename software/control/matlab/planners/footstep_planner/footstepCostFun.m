function [c, dc] = footstepCostFun(steps, steps_rel, goal_pos, right_foot_lead, dgoal, nominal_dxy)
w_goal = [1;1;0;0;0;10];
w_rot = 1;
w_dx_final = 10;
w_rel = [10; 10];
% dx_power = 2;
% dy_power = 2;
nominal_y_var = 0.05;

nsteps = size(steps, 2);
if ~right_foot_lead
  r_ndx = 1:2:nsteps;
  l_ndx = 2:2:nsteps;
%   dgoal = norm(steps(1:2,1) - goal_pos.right(1:2));
else
  r_ndx = 2:2:nsteps;
  l_ndx = 1:2:nsteps;
%   dgoal = norm(steps(1:2,1) - goal_pos.left(1:2));
end

% Normalize the goal weight so that the plans don't stretch out as the goal
% gets farther away
extra_distance = max(dgoal - (nsteps - 1) * nominal_dxy(1), 0.01);
%   w_goal(1:2) = w_rel(1) / ((4 / dx_power) * nominal_dxy(1) * extra_distance * (nsteps - j + 1));


c = 0;
dc = zeros(12,nsteps);
for j = 2:nsteps
  if mod(right_foot_lead + j, 2)
    c = c + 0.5 * sum(w_goal .* (steps(:,j) - goal_pos.right).^2);
    dc(1:6,j) = w_goal .* (steps(:,j) - goal_pos.right);
  else
    c = c + 0.5 * sum(w_goal .* (steps(:,j) - goal_pos.left).^2);
    dc(1:6,j) = w_goal .* (steps(:,j) - goal_pos.left);
  end
end

nominal_dxy = repmat(nominal_dxy, 1, nsteps);
nominal_dxy(2,r_ndx) = -nominal_dxy(2,r_ndx);

% c = c + (-1) * w_rot * sum(steps_rel(6,r_ndx)) + w_rot * sum(steps_rel(6,l_ndx));
% dc(12,r_ndx) = -w_rot;
% dc(12,l_ndx) = w_rot;


c_dex = 1;
c_dnom = 0;
assert(w_goal(1) == w_goal(2), 'Currently I assume that world x and y coordinate goal weights are the same');
for j = nsteps:-1:2
  w_rel(1) = w_goal(1) / nominal_dxy(1) * (c_dex * extra_distance + c_dnom * nominal_dxy(1));
  c = c + 0.5 * w_rel(1) * steps_rel(1,j).^2;
  dc(7,j) = w_rel(1) * steps_rel(1,j);
  
  w_rel(2) = w_goal(1) / nominal_dxy(1) * (c_dex * extra_distance + c_dnom * nominal_dxy(1));
  c = c + 0.5 * w_rel(2) * (steps_rel(2,j) - nominal_dxy(2,j)).^2;
  dc(8,j) = w_rel(2) * (steps_rel(2,j) - nominal_dxy(2,j));
  
  w_rot = w_goal(6) /  nominal_dxy(1) * (c_dex * extra_distance + c_dnom * nominal_dxy(1));
  c = c + 0.5 * w_rot * (steps_rel(6,j)).^2;
  dc(12,j) = w_rot * steps_rel(6,j);
  
  c_dnom = c_dnom + c_dex;
  c_dex = c_dex + 1;
end

% c = c + sum(w_rel(2) * ((steps_rel(2,2:end) - nominal_dxy(2,2:end))/nominal_y_var).^dy_power);
% dc(8,2:end) = w_rel(2) * dy_power*((steps_rel(2,2:end) - nominal_dxy(2,2:end))/nominal_y_var).^(dy_power-1) .* (1 / nominal_y_var);
% c = c + sum(w_rel(1) * (steps_rel(1,2:end) ./ nominal_dxy(1,2:end)).^dx_power);
% dc(7,2:end) = w_rel(1) * dx_power*(steps_rel(1,2:end) ./ nominal_dxy(1,2:end)).^(dx_power-1) .* (1 ./ nominal_dxy(1,2:end));

c = c + w_dx_final * steps_rel(1,end)^2;
dc(7,end) = dc(7,end) + w_dx_final * 2 * steps_rel(1,end);

dc = reshape(dc, [], 1);

end