function [c, dc] = footstepCostFun(steps, steps_rel, goal_pos, right_foot_lead, d_extra, nominal_dxy)
w_goal = [10;10;0;0;0;10];
w_rot = .1;
w_rot_final = 100;
w_rel = [1; 10];
w_rel_final = [100; 100];
% nominal_y_var = 0.05;

nsteps = size(steps, 2);
if ~right_foot_lead
  r_ndx = 1:2:nsteps;
  l_ndx = 2:2:nsteps;
else
  r_ndx = 2:2:nsteps;
  l_ndx = 1:2:nsteps;
end

% Normalize the goal weight so that the plans don't stretch out as the goal
% gets farther away
% extra_distance = max(dgoal - (nsteps - 1) * nominal_dxy(1), 0.01);
% w_goal(1:2) = w_rel(1) * nominal_dxy(1) / d_extra;

c = 0;
dc = zeros(12,nsteps);
for j = nsteps:nsteps
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
rot_dir = ones(1, nsteps);
rot_dir(r_ndx) = -1;

assert(w_goal(1) == w_goal(2), 'Currently I assume that world x and y coordinate goal weights are the same');
for j = 2:nsteps
  if j == nsteps
    w_rot = w_rot_final;
    w_rel = w_rel_final;
  end
  c = c + 0.5 * w_rel(1) * steps_rel(1,j).^2;
  dc(7,j) = w_rel(1) * steps_rel(1,j);

  c = c + 0.5 * w_rel(2) * (steps_rel(2,j) - nominal_dxy(2,j)).^2;
  dc(8,j) = w_rel(2) * (steps_rel(2,j) - nominal_dxy(2,j));

  c = c + w_rot * (rot_dir(j) * steps_rel(6,j));
  dc(12,j) = w_rot * rot_dir(j);

end

dc = reshape(dc, [], 1);

end