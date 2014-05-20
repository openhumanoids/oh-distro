function plan = searchNumSteps(biped, foot_orig, goal_pos, existing_steps, goal_steps, params, safe_regions)
% profile on
tic
foot_orig.right(4:5) = 0;
foot_orig.left(4:5) = 0;

weights = struct('relative', [10;50;10;0;0;.5],...
                 'relative_final', [1000;100;100;0;0;100],...
                 'goal', [100;100;0;0;0;10]);
                

start_steps = createOriginSteps(biped, foot_orig, true);
plan = FootstepPlan.blank_plan(22, [biped.foot_bodies_idx.right, biped.foot_bodies_idx.left], params, safe_regions);
plan.footsteps(1).pos = start_steps(1).pos;
plan.footsteps(2).pos = start_steps(2).pos;
min_steps = max([params.min_num_steps+2,3]);
max_steps = params.max_num_steps+2;


figure(1)
clf
for j = 1:2
  plan = footstepMIQP(biped, plan, weights, goal_pos, min_steps, max_steps);
  clf
  plot_plan(plan);
  plan = footstepCollocation(biped, plan, weights, goal_pos);
  clf
  plot_plan(plan);
end

step_vect = encodeCollocationSteps([plan.footsteps(2:end).pos]);
[steps, steps_rel] = decodeCollocationSteps(step_vect);
steps
steps_rel
return;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
  plan_set(j).seed_steps = plan_set(j).steps;
  plan_set(j).seed_steps(2).pos(6) = plan_set(j).seed_steps(1).pos(6);
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

best_costs = [];
completed_mask = [];

while true
  new_plan_set = struct('steps', {}, 'seed_steps', {}, 'cost', {}, 'goal_dist', {}, 'regions', {}, 'goal_reached', {});
  fprintf(1, 'plan length: %d\n', length(plan_set(1).steps));
  fprintf(1, 'plan set size: %d\n', length(plan_set));
  for j = 1:length(plan_set)
    seed_steps = plan_set(j).steps;
    seed_steps(end+1).pos = seed_steps(end-1).pos;
    seed_steps(end).is_right_foot = ~seed_steps(end-1).is_right_foot;

    nsteps = length(seed_steps)-1;
    region_assignments = zeros(length(safe_regions), nsteps-1);
    for k = 1:length(plan_set(j).regions)
      region_assignments(plan_set(j).regions(k), k) = 1;
    end
    if length(plan_set(j).regions) >= 2
      region_assignments(:,end) = region_assignments(:,end-1);
    else
      region_assignments(1,end) = 1;
    end
    miqp_seed = struct('steps', [seed_steps(2:end).pos], 'region_assignments', region_assignments);
    right_foot_lead = seed_steps(1).is_right_foot;
    [miqp_steps, region_assignments] = footstepMIQP(biped, foot_orig, goal_pos, nsteps, right_foot_lead, params, safe_regions, miqp_seed);
    [region_idx, ~] = find(abs(region_assignments - 1) < 1e-2);
    assert(length(region_idx) == size(region_assignments, 2));
    for k = 2:length(seed_steps)
      seed_steps(k).pos = miqp_steps(:,k-1);
    end

    [footsteps, exitflag, cost] = footstepCollocation(biped, seed_steps, goal_pos,...
        params, safe_regions(region_idx));
    if exitflag < 10
      total_diff = error_from_goal(footsteps, goal_pos, GOAL_THRESHOLD, w_final)
      if total_diff <= 1e-3
        goal_reached = true;
      else
        goal_reached = false;
      end
      if goal_reached || length(best_costs) < 2 || total_diff < 0.999 * best_costs(end-1)
        new_plan_set(end+1).steps = footsteps;
        new_plan_set(end).seed_steps = seed_steps;
        new_plan_set(end).cost = total_diff;
        new_plan_set(end).regions = region_idx;
        new_plan_set(end).goal_reached = goal_reached;
        new_plan_set(end).last_foot_right = footsteps(end).is_right_foot;
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
%
%   [~, sort_idx] = sort([plan_set.cost]);
%   plan_set = plan_set(sort_idx(1:min(BEAM_WIDTH, length(sort_idx))));
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
    total_diff = footsteps(end).pos - goal_pos.right;
  else
    total_diff = footsteps(end).pos - goal_pos.left;
  end
  total_diff = max(0, (abs(total_diff) - goal_threshold)' * w_final);


%   if footsteps(end).is_right_foot
%     diff_r = footsteps(end).pos - goal_pos.right;
%     diff_l = footsteps(end-1).pos - goal_pos.left;
%   else
%     diff_l = footsteps(end).pos - goal_pos.left;
%     diff_r = footsteps(end-1).pos - goal_pos.right;
%   end
%   diff_r = diff_r .* w_final; % don't count z, roll, and pitch
%   diff_l = diff_l .* w_final;
%   diff_r = max(0, abs(diff_r) - goal_threshold .* w_final);
%   diff_l = max(0, abs(diff_l) - goal_threshold .* w_final);
%   total_diff = sum(diff_r + diff_l);
end

function plot_plan(plan)
  hold on
  steps = [plan.footsteps.pos];
  plot(steps(1,:), steps(2,:), 'k:')
  quiver(steps(1,:), steps(2,:), cos(steps(6,:)), sin(steps(6,:)));
  step_vect = encodeCollocationSteps([plan.footsteps(2:end).pos]);
  [steps, steps_rel] = decodeCollocationSteps(step_vect);
  steps
  steps_rel
end

