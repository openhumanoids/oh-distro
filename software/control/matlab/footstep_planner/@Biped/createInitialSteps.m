function [X, foot_goals] = createInitialSteps(biped, x0, poses)

  q0 = x0(1:end/2);
  foot_orig = biped.feetPosition(q0);

  poses(6, poses(6,:) < -pi) = poses(6, poses(6,:) < -pi) + 2 * pi;
  poses(6, poses(6,:) > pi) = poses(6, poses(6,:) > pi) - 2 * pi;
  foot_goals = struct('right', biped.stepCenter2FootCenter(poses(1:6,end), 1), 'left', biped.stepCenter2FootCenter(poses(1:6,end), 0));


  X(1) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 1);
  X(2) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 0);

  X(3) = struct('pos', biped.footOrig2Contact(foot_goals.right, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 1);
  X(4) = struct('pos', biped.footOrig2Contact(foot_goals.left, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 0);
  
  t = num2cell(biped.getStepTimes([X.pos]));
  [X.time] = t{:};
end