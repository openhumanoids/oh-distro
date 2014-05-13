function output_footsteps = searchNumSteps(biped, foot_orig, goal_pos, existing_steps, goal_steps, params, safe_regions)
% profile on
tic
foot_orig.right(4:5) = 0;
foot_orig.left(4:5) = 0;

GOAL_THRESHOLD = [0.02;0.02;0;0;0;0.1];
w_final = [5;5;0;0;0;1];
BEAM_WIDTH = 4;

plan_set = struct('steps', {}, 'cost', {}, 'regions', {}, 'goal_reached', {}, 'last_foot_right', {});
if length(existing_steps) > 0
  error('I don''t currently know how to handle existing steps in this mode. User-adjusted footsteps from the viewer should trigger StatelessFootstepPlanner.check_footstep_plan() instead');
end

if params.leading_foot == drc.footstep_plan_params_t.LEAD_AUTO
  plan_set(end+1).steps = createOriginSteps(biped, foot_orig, true);
  plan_set(end+1).steps = createOriginSteps(biped, foot_orig, false);
  allow_even_num_steps = true;
  allow_odd_num_steps = true;
elseif params.leading_foot == drc.footstep_plan_params_t.LEAD_RIGHT
  plan_set(end+1).steps = createOriginSteps(biped, foot_orig, true);
  if length(goal_steps) == 1
    allow_odd_num_steps = goal_steps(1).is_right_foot;
    allow_even_num_steps = ~allow_odd_num_steps;
  elseif length(goal_steps) > 1
    allow_odd_num_steps = goal_steps(2).is_right_foot;
    allow_even_num_steps = ~allow_odd_num_steps;
  else
    allow_even_num_steps = true;
    allow_odd_num_steps = true;
  end
elseif params.leading_foot == drc.footstep_plan_params_t.LEAD_LEFT
  plan_set(end+1).steps = createOriginSteps(biped, foot_orig, false);
  if length(goal_steps) == 1
    allow_odd_num_steps = ~goal_steps(1).is_right_foot;
    allow_even_num_steps = ~allow_odd_num_steps;
  elseif length(goal_steps) > 1
    allow_odd_num_steps = ~goal_steps(2).is_right_foot;
    allow_even_num_steps = ~allow_odd_num_steps;
  else
    allow_even_num_steps = true;
    allow_odd_num_steps = true;
  end
end
for j = 1:length(plan_set)
  plan_set(j).cost = inf;
  plan_set(j).regions = [];
  plan_set(j).goal_reached = false;
  plan_set(j).last_foot_right = plan_set(j).steps(end).is_right_foot;
end


min_steps = max([params.min_num_steps+2,3]);
max_steps = params.max_num_steps+2;
if ((mod(max_steps, 2) == 1) && ~allow_odd_num_steps) || ...
   ((mod(max_steps, 2) == 0) && ~allow_even_num_steps)
  max_steps = max_steps + 1;
end
% max_steps = 12;

best_costs = [];
completed_mask = [];

while true
  new_plan_set = struct('steps', {}, 'cost', {}, 'regions', {}, 'goal_reached', {});
  fprintf(1, 'plan length: %d\n', length(plan_set(1).steps));
  fprintf(1, 'plan set size: %d\n', length(plan_set));
  for j = 1:length(plan_set)
    seed_steps = plan_set(j).steps;
    seed_steps(end+1) = seed_steps(end-1);
    nsteps = length(seed_steps) - 1;
    new_region_idx = 1:length(safe_regions);
    for k = 1:length(new_region_idx)
      region_idx = [plan_set(j).regions, new_region_idx(k)];
      [footsteps, exitflag, cost] = footstepCollocation(biped, seed_steps, goal_pos,...
        params, safe_regions(region_idx));
      exitflag
      if exitflag < 10
        total_diff = error_from_goal(footsteps, goal_pos, GOAL_THRESHOLD, w_final);
        if total_diff <= 1e-3
          goal_reached = true;
        else
          goal_reached = false;
        end

%       if total_diff < min([plan_set.cost])
        if length(best_costs) < 2 || total_diff < 0.95 * best_costs(end-1)
          % Check improvement against the best plan we were able to produce
          % with n-2 steps
          new_plan_set(end+1).steps = footsteps;
          new_plan_set(end).cost = total_diff;
          new_plan_set(end).regions = region_idx;
          new_plan_set(end).goal_reached = goal_reached;
          new_plan_set(end).last_foot_right = footsteps(end).is_right_foot;
        end
      end
    end
  end


  if isempty(new_plan_set)
    break
  end

  plan_set = new_plan_set;
  best_costs(end+1) = min([plan_set.cost]);

  for j = 1:length(plan_set)
    plan_set(j).nsteps = length(plan_set(j).steps);
  end
  completed_mask = ([plan_set.nsteps] >= min_steps) & ([plan_set.goal_reached]);
  if length(goal_steps) > 0
    % We're only done if the handedness of the plan matches the goal steps
    completed_mask = completed_mask & ([plan_set.last_foot_right] == goal_steps(min(2, length(goal_steps))).is_right_foot);
  end

  if any(completed_mask)
    break
  end

  if plan_set(1).nsteps == max_steps
    break
  end

  [~, sort_idx] = sort([plan_set.cost]);
  plan_set = plan_set(sort_idx(1:min(BEAM_WIDTH, length(sort_idx))));
end

disp('done');
if isempty(completed_mask) || ~any(completed_mask)
  completed_mask = true(1, length(plan_set));
  if length(goal_steps) > 0
    % We're only done if the handedness of the plan matches the goal steps
    completed_mask = completed_mask & ([plan_set.last_foot_right] == goal_steps(min(2, length(goal_steps))).is_right_foot);
    if ~any(completed_mask)
      error('Discrete search could not find any footstep plans that were compatible with the goal steps given');
    end
  end
end

complete_plans = plan_set(completed_mask);
[~, sort_idx] = sort([complete_plans.cost]);
output_footsteps = complete_plans(sort_idx(1)).steps;
step_vect = encodeCollocationSteps([output_footsteps(2:end).pos]);
[steps, steps_rel] = decodeCollocationSteps(step_vect);
steps_rel
toc
% profile viewer
end

function total_diff = error_from_goal(footsteps, goal_pos, goal_threshold, w_final)
  if footsteps(end).is_right_foot
    diff_r = footsteps(end).pos - goal_pos.right;
    diff_l = footsteps(end-1).pos - goal_pos.left;
  else
    diff_l = footsteps(end).pos - goal_pos.left;
    diff_r = footsteps(end-1).pos - goal_pos.right;
  end
  diff_r = diff_r .* w_final; % don't count z, roll, and pitch
  diff_l = diff_l .* w_final;
  diff_r = max(0, abs(diff_r) - goal_threshold .* w_final);
  diff_l = max(0, abs(diff_l) - goal_threshold .* w_final);
  total_diff = sum(diff_r + diff_l);
end
