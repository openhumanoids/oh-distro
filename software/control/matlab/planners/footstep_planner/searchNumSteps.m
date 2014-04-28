function output_footsteps = searchNumSteps(biped, foot_orig, goal_pos, goal_steps, terrain, corridor_pts, params, safe_regions)

foot_orig.right(4:5) = 0;
foot_orig.left(4:5) = 0;

GOAL_THRESHOLD = [0.02;0.02;0;0;0;0.1];

plan_set = struct('steps', {}, 'cost', {}, 'regions', {}, 'goal_reached', {});
if (params.leading_foot == drc.footstep_plan_params_t.LEAD_RIGHT) ...
   || (params.leading_foot == drc.footstep_plan_params_t.LEAD_AUTO)
  plan_set(end+1).steps = createOriginSteps(biped, foot_orig, true);
  plan_set(end).cost = inf;
  plan_set(end).regions = [];
  plan_set(end).goal_reached = false;
end
if (params.leading_foot == drc.footstep_plan_params_t.LEAD_LEFT) ...
   || (params.leading_foot == drc.footstep_plan_params_t.LEAD_AUTO)
  plan_set(end+1).steps = createOriginSteps(biped, foot_orig, false);
  plan_set(end).cost = inf;
  plan_set(end).regions = [];
  plan_set(end).goal_reached = false;
end

if isempty(safe_regions)
  safe_regions = {struct('A', [], 'b', [])};
end

min_steps = max([params.min_num_steps+2,3]);
% TODO: restore this
% max_steps = params.max_num_steps+2;
max_steps = 7;

while true
  new_plan_set = struct('steps', {}, 'cost', {}, 'regions', {}, 'goal_reached', {});
  disp(length(plan_set(1).steps));
  for j = 1:length(plan_set)
    seed_steps = plan_set(j).steps;
    seed_steps(end+1) = seed_steps(end-1);
    nsteps = length(seed_steps) - 1;
    new_region_idx = 1:length(safe_regions);
    for k = 1:length(new_region_idx)
      region_idx = [plan_set(j).regions, new_region_idx(k)];
      [footsteps, exitflag, cost] = footstepCollocation(biped, seed_steps, goal_pos,...
        terrain, corridor_pts, params, safe_regions(region_idx));
      if exitflag < 10
        if footsteps(end).is_right_foot
          diff_r = footsteps(end).pos - goal_pos.right;
          diff_l = footsteps(end-1).pos - goal_pos.left;
        else
          diff_l = footsteps(end).pos - goal_pos.right;
          diff_r = footsteps(end-1).pos - goal_pos.left;
        end
        diff_r = diff_r .* [1;1;0;0;0;1]; % don't count z, roll, and pitch
        diff_l = diff_l .* [1;1;0;0;0;1];
        diff_r = max(0, abs(diff_r) - GOAL_THRESHOLD);
        diff_l = max(0, abs(diff_l) - GOAL_THRESHOLD);
        total_diff = sum(diff_r + diff_l);
        if total_diff <= 0
          goal_reached = true;
        else
          goal_reached = false;
        end
        if total_diff < min([plan_set.cost])
          new_plan_set(end+1).steps = footsteps;
          new_plan_set(end).cost = total_diff;
          new_plan_set(end).regions = region_idx;
          new_plan_set(end).goal_reached = goal_reached;
        end
      end
    end
  end

  if isempty(new_plan_set)
    break
  end

  plan_set = new_plan_set;
  for j = 1:length(plan_set)
    plan_set(j).regions
  end

  for j = 1:length(plan_set)
    plan_set(j).nsteps = length(plan_set(j).steps);
  end
  completed_idx = find(([plan_set.nsteps] >= min_steps) & ([plan_set.goal_reached]));
  if ~isempty(completed_idx)
    break
  end

  if plan_set(1).nsteps == max_steps
    break
  end

end

disp('done');
if isempty(completed_idx)
  completed_idx = true(1, length(plan_set));
end
complete_plans = plan_set(completed_idx);
[~, sort_idx] = sort([complete_plans.cost]);
output_footsteps = complete_plans(sort_idx(1)).steps;

end