function recordStepMacros()

options.floating = true;
options.dt = 0.001;
biped = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
biped = removeCollisionGroupsExcept(biped,{'heel','toe'});
biped = compile(biped);

d = load('../data/atlas_fp.mat');
xstar = d.xstar;
xstar(1:6) = 0;

biped = biped.setInitialState(xstar);
nq = getNumDOF(biped);
qstar = xstar(1:nq);
qstar(1:6) = 0;

p = FootstepPlanner(biped);


directives(1) = struct('goal_pos_rel', [biped.max_forward_step * (4); 0; 0],...
                       'num_steps', 5,...
                       'name', 'step_forward_five');

                     
function [ground_pos, got_data, terrain_ok] = heightfun(pos, is_right_foot)
  ground_pos = pos;
  got_data = 0;
  terrain_ok = 1;
end
                     
for j = 1:length(directives)
  goal_pos_rel = directives(j).goal_pos_rel;
  num_steps = directives(j).num_steps;
  name = directives(j).name;
  M = makehgtform('translate', [xstar(1), xstar(2), xstar(3)], 'xrotate', 0, 'yrotate', 0, 'zrotate', xstar(6));
  goal_pos_abs = M * [goal_pos_rel; 1];
  goal_pos.translation = struct('x', goal_pos_abs(1), 'y', goal_pos_abs(2), 'z', goal_pos_abs(3));
  goal_rot = angle2quat(xstar(4), xstar(5), xstar(6), 'XYZ');
  goal_pos.rotation = struct('w', goal_rot(1), 'x', goal_rot(2), 'y', goal_rot(3), 'z', goal_rot(4));

  data = struct('x0', xstar, ...
                'goal', struct('allow_optimization', 1,...
                               'goal_pos', goal_pos,...
                               'max_num_steps', num_steps,...
                               'min_num_steps', num_steps,...
                               'right_foot_lead', 1,...
                               'is_new_goal', 1,...
                               'timeout', 0,...
                               'time_per_step', 0,...
                               'yaw_fixed', 0),...
                'utime', 0);
  changed = true;
  changelist = struct('plan_con', false, 'goal', true, 'plan_commit', false, 'plan_reject', false);


  footsteps = p.updatePlan([], data, changed, changelist, @heightfun);
  for j = 1:length(footsteps)
    footsteps(j).pos = biped.footContact2Orig(footsteps(j).pos, 'center', footsteps(j).is_right_foot);
  end

  [xtraj, qtraj, htraj, supptraj, comtraj, lfoottraj,rfoottraj, V, ts] = walkingPlanFromSteps(biped, xstar, qstar, footsteps);
  xtraj = PPTrajectory(spline(ts, xtraj));

  canned_plans.(name) = struct('htraj', htraj, 'supptraj', supptraj, 'comtraj', comtraj, 'lfoottraj', lfoottraj, 'rfoottraj', rfoottraj, 'zmptraj', zmptraj, 'ts', ts, 'footsteps', footsteps);
end

save('canned_plans.mat', 'canned_plans')

new_pos = d.xstar(1:6);
new_pos(6) = pi/4;



R = quat2rotmat(rpy2quat(new_pos(4:6)));
A = sparse([[R, zeros(3, length(xstar) - 3)];
     [zeros(length(xstar) - 3, 3), eye(length(xstar) - 3)]]);
addTransform(xtraj.getOutputFrame(), AffineTransform(xtraj.getOutputFrame(), biped.getStateFrame(), A,[new_pos; zeros(62, 1)]));

% v = biped.constructVisualizer();
% v.playback(xtraj, struct('slider', true));

end