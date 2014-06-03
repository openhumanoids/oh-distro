function plan = searchNumSteps(biped, feet_centers, goal_pos, params, safe_regions)
% profile on
tic
feet_centers.right(4:5) = 0;
feet_centers.left(4:5) = 0;

weights = struct('relative', [10;10;10;0;0;10],...
                 'relative_final', [1000;100;100;0;0;100],...
                 'goal', [100;100;0;0;0;10]);

if (params.leading_foot == drc.walking_goal_t.LEAD_RIGHT) ...
  || (params.leading_foot == drc.walking_goal_t.LEAD_AUTO)
  % for now, "auto" mode just also leads with the right foot
  plan = FootstepPlan.blank_plan(biped, params.max_num_steps + 2, [biped.foot_bodies_idx.right, biped.foot_bodies_idx.left], params, safe_regions);
  plan.footsteps(1).pos = Point(plan.footsteps(1).frames.center, feet_centers.right);
  plan.footsteps(2).pos = Point(plan.footsteps(2).frames.center, feet_centers.left);
else
  plan = FootstepPlan.blank_plan(biped, params.max_num_steps + 2, [biped.foot_bodies_idx.left, biped.foot_bodies_idx.right], params, safe_regions);
  plan.footsteps(1).pos = Point(plan.footsteps(1).frames.center, feet_centers.left);
  plan.footsteps(2).pos = Point(plan.footsteps(2).frames.center, feet_centers.right);
end

min_steps = max([params.min_num_steps+2,3]);
max_steps = params.max_num_steps+2;

num_outer_iterations = 2;
for j = 1:num_outer_iterations
  miqp_plan = footstepMIQP(biped, plan, weights, goal_pos, min_steps, max_steps);
  if length(miqp_plan.footsteps) <= 2
    % No feasible solution was found
    plan = miqp_plan;
    break
  end
%   figure(1)
%   clf
%   plot_plan(miqp_plan);
  plan = footstepCollocation(biped, miqp_plan, weights, goal_pos);
%   figure(1)
%   clf
%   plot_plan(plan);
  miqp_steps = miqp_plan.step_matrix();
  nlp_steps = plan.step_matrix();
  if all(abs(miqp_steps(6,:) - nlp_steps(6,:)) <= pi/32)
    break
  end
end

steps = plan.step_matrix();
step_vect = encodeCollocationSteps(steps(:,2:end));
[steps, steps_rel] = decodeCollocationSteps(step_vect);
steps
steps_rel
return;
end


function plot_plan(plan)
  hold on
  steps = plan.step_matrix();
  plot(steps(1,:), steps(2,:), 'k:')
  quiver(steps(1,:), steps(2,:), cos(steps(6,:)), sin(steps(6,:)));
  step_vect = encodeCollocationSteps(steps(:,2:end));
  [steps, steps_rel] = decodeCollocationSteps(step_vect);
  steps
  steps_rel
end

