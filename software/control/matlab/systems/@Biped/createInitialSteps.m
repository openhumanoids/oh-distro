function X = createInitialSteps(biped, x0, goal_pos, params)

if ~isfield(params, 'check_feasibility'); params.check_feasibility = false; end
if ~isfield(params, 'allow_optimization'); params.allow_optimization = false; end
if ~isstruct(goal_pos)
  goal_pos = struct('center', goal_pos);
end
if ~isfield(goal_pos, 'right')
  goal_pos.right = biped.stepCenter2FootCenter(goal_pos.center, true, params.nom_step_width);
  goal_pos.left = biped.stepCenter2FootCenter(goal_pos.center, false, params.nom_step_width);
elseif ~isfield(goal_pos, 'center')
  goal_pos.center = zeros(6,1);
  goal_pos.center(1:3) = mean([goal_pos.right(1:3), goal_pos.left(1:3)], 2);
  goal_pos.center(4:6) = goal_pos.right(4:6) + 0.5 * angleDiff(goal_pos.right(4:6), goal_pos.left(4:6));
end

q0 = x0(1:end/2);
foot_orig = biped.feetPosition(q0);

if params.velocity_based_steps
  X = createOriginSteps(biped, foot_orig, true);
  X(3).pos = goal_pos.right;
  X(3).is_right_foot = true;
  X(4).pos = goal_pos.left;
  X(4).is_right_foot = false;
else
  params_set = {params};
  if params.right_foot_lead == -1
    new_params_set = {};
    for p = params_set
      rparams = p{1};
      lparams = p{1};
      rparams.right_foot_lead = 1;
      lparams.right_foot_lead = 0;
      new_params_set{end+1} = rparams;
      new_params_set{end+1} = lparams;
    end
    params_set = new_params_set;
  end
  best_steps = [];
  best_length = inf;
  best_cost = inf;
  for p = params_set
    params = p{1};
    if params.allow_optimization
      X = footstepCollocation(biped, foot_orig, goal_pos, params);
    else
      X = footstepLineSearch(biped, foot_orig, goal_pos.center, params);
    end
    step_vect = encodeCollocationSteps([X(2:end).pos]);
    [steps, steps_rel] = decodeCollocationSteps(step_vect);
    l = length(X);
    c = footstepCostFun(steps, steps_rel, goal_pos, logical(params.right_foot_lead));
    if l < best_length || c < best_cost % always prefer fewer steps
      best_steps = X;
      best_length = l;
      best_cost = c;
    end
  end
end

X = best_steps;

for j = 1:length(X)
  X(j).id = j;
  for var = {'step_speed', 'step_height', 'bdi_step_duration', 'bdi_sway_duration', 'bdi_lift_height', 'bdi_toe_off', 'bdi_knee_nominal', 'bdi_max_body_accel', 'bdi_max_foot_vel', 'bdi_sway_end_dist', 'bdi_step_end_dist'}
    v = var{1};
    X(j).(v) = params.(v);
  end
  if j <= 2
    X(j).pos_fixed = ones(6,1);
  else
    X(j).pos_fixed = zeros(6,1);
  end
  X(j).is_in_contact = true;
  X(j).walking_params =params;
end

if params.ignore_terrain
  p0 = biped.footCenter2StepCenter(X(2).pos, X(2).is_right_foot, params.nom_step_width);
  normal = rpy2rotmat(p0(4:6)) * [0;0;1];
  for j = 3:length(X)
    X(j).pos(3) = p0(3) - (1 / normal(3)) * (normal(1) * (X(j).pos(1) - p0(1)) + normal(2) * (X(j).pos(2) - p0(2)));
    X(j).pos = fitPoseToNormal(X(j).pos, normal);
  end
else
  for j = 3:length(X)
    X(j).pos = fitStepToTerrain(biped, X(j).pos);
  end
end


end
