function [X, foot_goals] = createInitialSteps(biped, x0, poses, options)

  if options.yaw_fixed 
    traj = turnGoTraj([x0, poses]);
  else
    traj = BezierTraj([x0, poses]);
  end

  q0 = x0(1:end/2);
  foot_orig = biped.feetPosition(q0);

  poses(6, poses(6,:) < -pi) = poses(6, poses(6,:) < -pi) + 2 * pi;
  poses(6, poses(6,:) > pi) = poses(6, poses(6,:) > pi) - 2 * pi;
  foot_goals = struct('right', biped.stepCenter2FootCenter(poses(1:6,end), 1), 'left', biped.stepCenter2FootCenter(poses(1:6,end), 0));


  X(1) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 1);
  X(2) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 0);

  lambda = 0;
  while (1)
    lambda_n = 1;
    while (1)
      x = traj.eval(lambda_n);
      if options.yaw_fixed
        x(6) = x0(6);
      end
      pos_n = biped.stepCenter2FootCenter(x, ~X(end).is_right_foot);
      if biped.checkStepFeasibility(X(end), pos_n, X(end).is_right_foot)
        break
      else
        lambda_n = lambda + (lambda_n - lambda) * .9;
      end
    end
    X(end+1) = struct('pos', pos_n, 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', zeros(6, 1), 'is_right_foot', ~X(end).is_right_foot);
    if X(end).is_right_foot
      goal = foot_goals.right;
    else
      goal = foot_goals.left;
    end
    if all(X(end).pos - goal < 0.05) or length(X) >= options.max_num_steps - 1
      break
    end
  end

  final_center = biped.footCenter2StepCenter(X(end).pos, X(end).is_right_foot);
  X(end+1) = struct('pos', biped.stepCenter2FootCenter(final_center, ~X(end).is_right_foot), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', zeros(6, 1), 'is_right_foot', ~X(end).is_right_foot);


  % X(3) = struct('pos', biped.footOrig2Contact(foot_goals.right, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 1);
  % X(4) = struct('pos', biped.footOrig2Contact(foot_goals.left, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 0);
  
  t = num2cell(biped.getStepTimes([X.pos]));
  [X.time] = t{:};
end