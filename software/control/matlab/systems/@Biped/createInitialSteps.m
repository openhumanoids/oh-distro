function [X, foot_goals] = createInitialSteps(biped, x0, goal_pos, params)

if ~isfield(params, 'check_feasibility'); params.check_feasibility = false; end
if ~isfield(params, 'allow_optimization'); params.allow_optimization = false; end
if ~isstruct(goal_pos)
  goal_pos = struct('center', goal_pos);
end

if params.allow_optimization
  [X, foot_goals] = footstepCollocation(biped, x0, goal_pos, params);
else
  [X, foot_goals] = footstepLineSearch(biped, x0, goal_pos.center, params);
end

end
