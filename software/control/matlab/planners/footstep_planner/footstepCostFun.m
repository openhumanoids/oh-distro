function [c, dc] = footstepCostFun(steps, steps_rel, goal_pos, right_foot_lead, dgoal, nominal_dxy)
w_goal = [10;10;1;1;1;100];
w_rot = 10;
w_dx_final = 100;
w_rel = [1; 1];
dx_power = 8;
dy_power = 2;
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
extra_distance = max(dgoal - (nsteps - 2) * nominal_dxy(1), 0.01);
w_goal(1:2) = w_rel(1) / ((4 / dx_power) * nominal_dxy(1) * extra_distance);

% c = c + sqrt(sum((w_goal .* (steps(:,r_ndx(end)) - goal_pos.right)).^2));
% dc(1:6,r_ndx(end)) = 0.5 * 1 / sqrt(sum((w_goal .* (steps(:,r_ndx(end)) - goal_pos.right)).^2)) ...
%       * 2 * (w_goal.^2 .* (steps(:,r_ndx(end)) - goal_pos.right));
% c = c + sqrt(sum((w_goal .* (steps(:,l_ndx(end)) - goal_pos.left)).^2));
% dc(1:6,l_ndx(end)) = 0.5 * 1 / sqrt(sum((w_goal .* (steps(:,l_ndx(end)) - goal_pos.left)).^2)) ...
%       * 2 * (w_goal.^2 .* (steps(:,l_ndx(end)) - goal_pos.left));
% c = sum(w_goal .* sum(bsxfun(@minus, steps(:,r_ndx(end)), goal_pos.right).^2,2)) ...
%            + sum(w_goal .* sum(bsxfun(@minus, steps(:,l_ndx(end)), goal_pos.left).^2,2));
c = sum(w_goal .* (steps(:,r_ndx(end)) - goal_pos.right).^2) + ...
    sum(w_goal .* (steps(:,l_ndx(end)) - goal_pos.left).^2);
dc = zeros(12,nsteps);
for j = nsteps-1:nsteps
  if mod(right_foot_lead + j, 2)
    dc(1:6,j) = w_goal .* (2*(steps(:,j) - goal_pos.right));
  else
    dc(1:6,j) = w_goal .* (2*(steps(:,j) - goal_pos.left));
  end
end

nominal_dxy = repmat(nominal_dxy, 1, nsteps);
nominal_dxy(2,r_ndx) = -nominal_dxy(2,r_ndx);

c = c + (-1) * w_rot * sum(steps_rel(6,r_ndx)) + w_rot * sum(steps_rel(6,l_ndx));
dc(12,r_ndx) = -w_rot;
dc(12,l_ndx) = w_rot;

c = c + sum(w_rel(2) * ((steps_rel(2,2:end) - nominal_dxy(2,2:end))/nominal_y_var).^dy_power);
dc(8,2:end) = w_rel(2) * dy_power*((steps_rel(2,2:end) - nominal_dxy(2,2:end))/nominal_y_var).^(dy_power-1) .* (1 / nominal_y_var);
c = c + sum(w_rel(1) * (steps_rel(1,2:end) ./ nominal_dxy(1,2:end)).^dx_power);
dc(7,2:end) = w_rel(1) * dx_power*(steps_rel(1,2:end) ./ nominal_dxy(1,2:end)).^(dx_power-1) .* (1 ./ nominal_dxy(1,2:end));

c = c + w_dx_final * steps_rel(1,end)^2;
dc(7,end) = dc(7,end) + w_dx_final * 2 * steps_rel(1,end);

dc = reshape(dc, [], 1);

% w_goal = [1;1;1;1;1;100];
% w_rot = 10;
% 
% nsteps = size(steps, 2);
% if ~right_foot_lead
%   r_ndx = 1:2:nsteps;
%   l_ndx = 2:2:nsteps;
% else
%   r_ndx = 2:2:nsteps;
%   l_ndx = 1:2:nsteps;
% end
% c = sum(w_goal .* sum(bsxfun(@minus, steps(:,r_ndx(end)), goal_pos.right).^2,2)) ...
%            + sum(w_goal .* sum(bsxfun(@minus, steps(:,l_ndx(end)), goal_pos.left).^2,2));
% dc = zeros(12,nsteps);
% for j = nsteps-1:nsteps
%   if mod(right_foot_lead + j, 2)
%     dc(1:6,j) = w_goal .* (2*(steps(:,j) - goal_pos.right));
%   else
%     dc(1:6,j) = w_goal .* (2*(steps(:,j) - goal_pos.left));
%   end
% end
% 
% c = c + (-1) * w_rot * sum(steps_rel(6,r_ndx)) + w_rot * sum(steps_rel(6,l_ndx));
% dc(12,r_ndx) = -w_rot;
% dc(12,l_ndx) = w_rot;
% 
% dc = reshape(dc, [], 1);
end