function [X, foot_goals] = createInitialSteps(biped, x0, poses, options)


  q0 = x0(1:end/2);
  foot_orig = biped.feetPosition(q0);

  poses(6, poses(6,:) < -pi) = poses(6, poses(6,:) < -pi) + 2 * pi;
  poses(6, poses(6,:) > pi) = poses(6, poses(6,:) > pi) - 2 * pi;
  foot_goals = struct('right', biped.stepCenter2FootCenter(poses(1:6,end), 1), 'left', biped.stepCenter2FootCenter(poses(1:6,end), 0));


  X(1) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', options.right_foot_lead);
  X(2) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', ~X(1).is_right_foot);

  p0 = mean([X(1).pos, X(2).pos], 2);
  if options.yaw_fixed 
    traj = turnGoTraj([p0, poses]);
  else
    traj = BezierTraj([p0, poses]);
  end
  t = linspace(0, 1);
  xy = traj.eval(t);
  plot_lcm_points([xy(1,:)', xy(2,:)', ones(length(t), 1)], repmat([0, 0, 1], length(t), 1), 50, 'Foostep Spline', 2, 1);

  lambda = 0;
  while (1)
    lambda_n = 1;
    while (1)
      x = traj.eval(lambda_n);
      if options.yaw_fixed
        x(6) = x0(6);
      end
      pos_n = biped.stepCenter2FootCenter(x, ~X(end).is_right_foot);
      c = biped.checkStepFeasibility(X(end).pos, pos_n, X(end).is_right_foot);
      if all(c <= 0)
        break
      else
        lambda_n = lambda + (lambda_n - lambda) * .9;
      end
    end
    lambda = lambda_n
    X(end+1) = struct('pos', pos_n, 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', zeros(6, 1), 'is_right_foot', ~X(end).is_right_foot);
    if X(end).is_right_foot
      goal = foot_goals.right;
    else
      goal = foot_goals.left;
    end
    if (all(abs(X(end).pos - goal) < 0.05) || length(X) - 2 >= options.max_num_steps - 1) && (length(X) - 2 >= options.min_num_steps)
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