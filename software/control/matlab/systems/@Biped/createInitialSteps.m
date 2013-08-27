function [X, foot_goals] = createInitialSteps(biped, x0, goal_pos, options)

  debug = false;

  q0 = x0(1:end/2);
  foot_orig = biped.feetPosition(q0);
  sizecheck(goal_pos, [6,1]);

  if options.right_foot_lead
    X(1) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 1), 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', ones(6,1), 'is_right_foot', true, 'is_in_contact', true);
    X(2) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 0), 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', ones(6,1), 'is_right_foot', false, 'is_in_contact', true);
  else
    X(1) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 1), 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', ones(6,1), 'is_right_foot', false, 'is_in_contact', true);
    X(2) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 0), 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', ones(6,1), 'is_right_foot', true, 'is_in_contact', true);
  end
 
  p0 = footCenter2StepCenter(biped, X(2).pos, X(2).is_right_foot, options.nom_step_width);
  goal_pos(6) = p0(6) + angleDiff(p0(6), goal_pos(6));
  goal_pos(3,:) = p0(3);
  if ~options.ignore_terrain
    goal_pos = fitStepToTerrain(biped, goal_pos);
  end

  if options.follow_spline
    traj = BezierTraj([p0, goal_pos]);
  else
    traj = DirectTraj([p0, goal_pos]);
  end

  [~, feasibility, foot_centers] = scanWalkingTerrain(biped, traj, p0, options.nom_step_width);
  if options.ignore_terrain
    % use position of the right foot to set the height and orientation of the steps
    normal = rpy2rotmat(p0(4:6)) * [0;0;1];
    foot_centers.right(3,:) = p0(3) - (1 / normal(3)) * (normal(1) * (foot_centers.right(1,:) - p0(1)) + normal(2) * (foot_centers.right(2,:) - p0(2)));
    foot_centers.left(3,:) = p0(3) - (1 / normal(3)) * (normal(1) * (foot_centers.left(1,:) - p0(1)) + normal(2) * (foot_centers.left(2,:) - p0(2)));
    foot_centers.right = fitPoseToNormal(foot_centers.right, repmat(normal, 1, length(foot_centers.right(1,:))));
    foot_centers.left = fitPoseToNormal(foot_centers.left, repmat(normal, 1, length(foot_centers.right(1,:))));
    feasibility.right = ones(size(feasibility.right));
    feasibility.left = ones(size(feasibility.left));
  else
    for f = {'right', 'left'}
      foot = f{1};
      foot_centers.(foot) = fitStepToTerrain(biped, foot_centers.(foot));
    end
  end

  last_safe_idx = find(feasibility.right & feasibility.left, 1, 'last');
  foot_goals = struct('right', foot_centers.right(:, last_safe_idx), 'left', foot_centers.left(:, last_safe_idx));

  if debug
    ls = linspace(0, 1);
    xy = traj.eval(ls);
    plot_lcm_points([xy(1,:)', xy(2,:)', xy(3,:)'], repmat([0, 0, 1], length(ls), 1), 50, 'Foostep Spline', 2, 1);
  end
  
  last_ndx = struct('right', 1, 'left', 1);
  
stall = struct('right', 0, 'left', 0);
aborted = false;
min_progress = [0.05;0.05;1;0.4;0.4;0.4];
% n = 0;
  
  while (1)
    is_right_foot = ~X(end).is_right_foot;
    if is_right_foot
      m_foot = 'right';
      s_foot = 'left';
    else
      m_foot = 'left';
      s_foot = 'right';
    end
    goal = foot_goals.(m_foot);

    potential_poses = foot_centers.(m_foot)(:, last_ndx.(m_foot):end);
    feas_opts = struct('forward_step', options.nom_forward_step,...
                       'nom_step_width', options.nom_step_width);
    reach = biped.checkStepReach(X(end).pos, potential_poses, ~is_right_foot, feas_opts);
    valid_pose_ndx = find(max(reach, [], 1) <= 0 & feasibility.(m_foot)(last_ndx.(m_foot):end)) + (last_ndx.(m_foot) - 1);
    if isempty(valid_pose_ndx)
      novalid = true;
    else
      novalid = false;
      if (...
          all(abs(X(end).pos - foot_goals.(s_foot)) < 0.05) || ...
          (...
           ((length(X) - 2) >= options.max_num_steps - 1) && ...
           (options.max_num_steps > 1)...
          )...
         )
        [~, j] = min(abs(valid_pose_ndx - last_ndx.(s_foot)));
        next_ndx = valid_pose_ndx(j);
      else
        next_ndx = valid_pose_ndx(end);
      end
      noprogress = all(abs(foot_centers.(m_foot)(:, next_ndx) - foot_centers.(m_foot)(:,last_ndx.(m_foot))) < min_progress);
    end

    if (novalid || noprogress)
      % Try again with a longer maximum step
      feas_opts.forward_step = options.max_forward_step;
      reach = biped.checkStepReach(X(end).pos, potential_poses, ~is_right_foot, feas_opts);
      valid_pose_ndx = find(max(reach, [], 1) <= 0 & feasibility.(m_foot)(last_ndx.(m_foot):end)) + last_ndx.(m_foot) - 1;
      if isempty(valid_pose_ndx)
        novalid = true;
      else
        if novalid
          next_ndx = valid_pose_ndx(1);
        else
          n = find(valid_pose_ndx > next_ndx, 1, 'first');
          if ~isempty(n)
            next_ndx = valid_pose_ndx(n);
          else
            next_ndx = valid_pose_ndx(end);
          end
        end
        novalid = false;
        noprogress = all(abs(foot_centers.(m_foot)(:, next_ndx) - foot_centers.(m_foot)(:,last_ndx.(m_foot))) < min_progress);
      end      
    end
    
    if (novalid || noprogress)
      stall.(m_foot) = stall.(m_foot) + 1;
      if (novalid || stall.(m_foot) >= 2 || (stall.(m_foot) > 0 && stall.(s_foot) > 0)) && (length(X) - 3 >= options.min_num_steps)
        aborted = true;
        break
      end
    else
      stall.(m_foot) = 0;
    end

    pos_n = foot_centers.(m_foot)(:, next_ndx);
    last_ndx.(m_foot) = next_ndx;

    X(end+1) = struct('pos', pos_n, 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', zeros(6, 1), 'is_right_foot', is_right_foot, 'is_in_contact', true);

    if ((all(abs(X(end).pos - goal) < 0.05) && all(abs(X(end-1).pos - foot_goals.(s_foot)) < 0.05)) || (length(X) - 2) >= options.max_num_steps) && ((length(X) - 2) >= options.min_num_steps)
      break
    end
  end
  
  if aborted && length(X) > 3
    % If we had to give up, then lose the last (unproductive) step
    X = X(1:end-1);
  end
 
  biped.getNextStepID(true); % reset the counter
  for j = 1:length(X)
    X(j).id = biped.getNextStepID();
    X(j).step_speed = options.step_speed;
    X(j).step_height = options.step_height;
  end
  
end
