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

num_outer_iterations = 1;
for j = 1:num_outer_iterations
  plan = footstepMIQP(biped, plan, weights, goal_pos, min_steps, max_steps);
  % figure(1)
  % clf
  % plot_plan(plan);
  plan = footstepCollocation(biped, plan, weights, goal_pos);
  % figure(1)
  % clf
  % plot_plan(plan);
end

step_vect = encodeCollocationSteps([plan.footsteps(2:end).pos]);
[steps, steps_rel] = decodeCollocationSteps(step_vect);
steps
steps_rel
return;
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

