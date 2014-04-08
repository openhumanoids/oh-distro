function [c, dc] = footstepCostFun(steps, steps_rel, goal_pos, right_foot_lead)
w_goal = [1;1;1;1;1;100];
w_rot = 10;

nsteps = size(steps, 2);
if ~right_foot_lead
  r_ndx = 1:2:nsteps;
  l_ndx = 2:2:nsteps;
else
  r_ndx = 2:2:nsteps;
  l_ndx = 1:2:nsteps;
end
c = sum(w_goal .* sum(bsxfun(@minus, steps(:,r_ndx(end)), goal_pos.right).^2,2)) ...
           + sum(w_goal .* sum(bsxfun(@minus, steps(:,l_ndx(end)), goal_pos.left).^2,2));
dc = zeros(12,nsteps);
for j = nsteps-1:nsteps
  if mod(right_foot_lead + j, 2)
    dc(1:6,j) = w_goal .* (2*(steps(:,j) - goal_pos.right));
  else
    dc(1:6,j) = w_goal .* (2*(steps(:,j) - goal_pos.left));
  end
end

c = c + (-1) * w_rot * sum(steps_rel(6,r_ndx)) + w_rot * sum(steps_rel(6,l_ndx));
dc(12,r_ndx) = -w_rot;
dc(12,l_ndx) = w_rot;

dc = reshape(dc, [], 1);
end