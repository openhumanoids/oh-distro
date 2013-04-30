function [X, foot_goals] = createInitialSteps(biped, x0, poses, options, heightfun)

  debug = true;

  q0 = x0(1:end/2);
  foot_orig = biped.feetPosition(q0);


  poses(6, poses(6,:) < -pi) = poses(6, poses(6,:) < -pi) + 2 * pi;
  poses(6, poses(6,:) > pi) = poses(6, poses(6,:) > pi) - 2 * pi;

  if options.right_foot_lead
    X(1) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', true, 'is_in_contact', true);
    X(2) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', false, 'is_in_contact', true);
  else
    X(1) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', false, 'is_in_contact', true);
    X(2) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', true, 'is_in_contact', true);
  end
  
  last_good_z = X(1).pos(3);
  using_heightmap = false;

  p0 = [mean([X(1).pos(1:3), X(2).pos(1:3)], 2); X(1).pos(4:6)];
  poses(3,:) = p0(3);
  poses = heightfun(poses);
  foot_goals = struct('right', biped.stepCenter2FootCenter(poses(1:6,end), 1), 'left', biped.stepCenter2FootCenter(poses(1:6,end), 0));

  if options.yaw_fixed || all(sum(diff([p0(1:3), poses(1:3,:)], 1, 2).^2, 1) <= 1)
    traj = turnGoTraj([p0, poses]);
  else
    traj = BezierTraj([p0, poses]);
  end
  t = linspace(0, 1);
  xy = traj.eval(t);
  if debug
    plot_lcm_points([xy(1,:)', xy(2,:)', xy(3,:)'], repmat([0, 0, 1], length(t), 1), 50, 'Foostep Spline', 2, 1);
  end
  
  last_lambda = struct('right', 0, 'left', 0);
  
%   function [c, ceq] = nonlcon(lambda_n)
%     x = traj.eval(lambda_n);
%     if options.yaw_fixed
%       x(6) = x0(6);
%     end
%     [pos_n, got_data, terrain_ok] = heightfun(biped.stepCenter2FootCenter(x, is_right_foot), is_right_foot);
%     c = biped.checkStepFeasibility(X(end).pos, pos_n, ~is_right_foot, max_forward_step);
%     ceq = double(~terrain_ok);
%   end
%   opts = optimset('Algorithm', 'interior-point');
  
  while (1)
    is_right_foot = ~X(end).is_right_foot;
    if is_right_foot
      lambda = last_lambda.right;
    else
      lambda = last_lambda.left;
    end
    lambda_n = 1;
    nom_forward_step = biped.nom_forward_step;
    while (1)
      
%       [lambda_n, ~, exitflag] = fmincon(@(x) -x, lambda_n, [], [], [], [], [0], [1], @nonlcon, opts);
%       break
      x = traj.eval(lambda_n);
      if options.yaw_fixed
        x(6) = x0(6);
      end
      [pos_n, got_data, terrain_ok] = heightfun(biped.stepCenter2FootCenter(x, is_right_foot), is_right_foot);
      c = biped.checkStepFeasibility(X(end).pos, pos_n, ~is_right_foot, nom_forward_step);
      if (all(c <= 0)  && ~((got_data || using_heightmap) && ~terrain_ok)) 
        break
      elseif (lambda_n - lambda < 1e-2 && nom_forward_step + 0.05 > biped.max_forward_step)
        break
      elseif (lambda_n - lambda < 1e-2)
        nom_forward_step = nom_forward_step + 0.05;
%         lambda = lambda - 0.05;
        lambda_n = 1;
      else
        lambda_n = lambda + (lambda_n - lambda) * .9;
%         lambda_n = lambda_n - 0.03;
%         lambda_n = lambda - 0.05 + (lambda_n - lambda + 0.05) * .9;
      end
    end
    if got_data
      using_heightmap = true;
    end
    if ~using_heightmap
      pos_n(3) = last_good_z;
    end
    if is_right_foot
      last_lambda.right = lambda_n;
    else
      last_lambda.left = lambda_n;
    end
    if length(X) == 2
      last_pos = X(end-1).pos;
    else
      last_pos = X(end-2).pos;
    end
    apex_pos = get_apex_pos(last_pos, pos_n);
    X(end+1) = struct('pos', apex_pos, 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', zeros(6, 1), 'is_right_foot', is_right_foot, 'is_in_contact', false);
    X(end+1) = struct('pos', pos_n, 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', zeros(6, 1), 'is_right_foot', is_right_foot, 'is_in_contact', true);

    %%%%
    Xout = X;
    % Convert from foot center to foot origin
    for j = 1:length(X)
      Xout(j).pos = biped.footContact2Orig(X(j).pos, 'center', X(j).is_right_foot);
    end
    biped.publish_footstep_plan(Xout);
    %%%%%

    if X(end).is_right_foot
      goal = foot_goals.right;
    else
      goal = foot_goals.left;
    end
    if (all(abs(X(end).pos - goal) < 0.05) || (length(X) - 2)/2 >= options.max_num_steps - 1) && ((length(X) - 2)/2 >= options.min_num_steps - 1)
      break
    end
  end

  final_center = biped.footCenter2StepCenter(X(end).pos, X(end).is_right_foot);
  final_pos = biped.stepCenter2FootCenter(final_center, ~X(end).is_right_foot);
  is_right_foot = ~X(end).is_right_foot;
  apex_pos = get_apex_pos(X(end-2).pos, final_pos);
  X(end+1) = struct('pos', apex_pos, 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', zeros(6, 1), 'is_right_foot', is_right_foot, 'is_in_contact', false);

  X(end+1) = struct('pos', final_pos, 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', zeros(6, 1), 'is_right_foot', is_right_foot, 'is_in_contact', true);


  % X(3) = struct('pos', biped.footOrig2Contact(foot_goals.right, 'center', 1), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 1);
  % X(4) = struct('pos', biped.footOrig2Contact(foot_goals.left, 'center', 0), 'time', 0, 'id', biped.getNextStepID(), 'pos_fixed', ones(6,1), 'is_right_foot', 0);
  
  t = num2cell(biped.getStepTimes([X.pos], options.time_per_step));
  [X.time] = t{:};
  
  function apex_pos = get_apex_pos(last_pos, next_pos)
    apex_pos = mean([last_pos, next_pos], 2);
    if last_pos(3) - next_pos(3) > (0.8 * biped.nom_step_clearance)
      apex_pos(1:2) = next_pos(1:2);
    elseif next_pos(3) - last_pos(3) > (0.8 * biped.nom_step_clearance)
      apex_pos(1:2) = last_pos(1:2);
    end
    apex_pos(3) = max([last_pos(3), next_pos(3)]) + biped.nom_step_clearance;
  end
end
