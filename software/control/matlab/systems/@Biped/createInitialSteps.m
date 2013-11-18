function [X, foot_goals] = createInitialSteps(biped, x0, goal_pos, params)

if ~isfield(params, 'check_feasibility'); params.check_feasibility = false; end
if ~isfield(params, 'allow_optimization'); params.allow_optimization = false; end
if ~isstruct(goal_pos)
  goal_pos = struct('center', goal_pos);
elseif ~isfield(goal_pos, 'center')
  goal_pos.center = zeros(6,1);
  goal_pos.center(1:3) = mean([goal_pos.right(1:3), goal_pos.left(1:3)], 2);
  goal_pos.center(4:6) = goal_pos.right(4:6) + 0.5 * angleDiff(goal_pos.right(4:6), goal_pos.left(4:6));
end

if params.allow_optimization
  [X, foot_goals] = footstepCollocation(biped, x0, goal_pos, params);
else
  [X, foot_goals] = footstepLineSearch(biped, x0, goal_pos.center, params);
end

end
