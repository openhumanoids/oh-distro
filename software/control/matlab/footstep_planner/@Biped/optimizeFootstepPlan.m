function [Xright, Xleft] = optimizeFootstepPlan(biped, x0, poses, outputfun, updatefun, data)
q0 = x0(1:end/2);
[start_pos, step_width] = biped.feetPosition(q0);
poses = [start_pos, poses];

  if nargin < 5
    interactive = false;
  else
    interactive = true;
  end

  if nargin < 3
    outputfun = @(X, hfun) X;
  end

  % Structure of X:
  % x
  % y
  % z
  % roll
  % pitch
  % yaw
  % time
  % ID (unique, between 0 and 1)
  % x fixed flag
  % y fixed flag
  % z fixed flag
  % roll fixed flag
  % pitch fixed flag
  % yaw fixed flag
  X = interp1([1:length(poses(1,:))]', poses', [1:0.5:length(poses(1,:))]')';
  X(7, :) = (0:length(X(1,:))-1) * biped.step_time / 2;
  X(8, :) = int32((0:length(X(1,:))-1) .* 1000000000 ./ (length(X(1,:)) - 1));
  X(9:14,:) = zeros(6, length(X(1,:)));

  total_steps = length(X(1,:));

  ndx = biped.getStepNdx(total_steps);
  % ndx.right = int32([1, 2, 4:2:(total_steps-1), total_steps]);
  % ndx.left = int32([1:2:(total_steps-1), total_steps]);
  for p = poses
    [~,j] = min(sum((X(1:6,:) - repmat(p, 1, length(X(1,:)))).^2));
    X(9:14, j) = 1;
  end

  done = false;


  while 1
    X_old = X;
    while 1
      if interactive
        [data, changed, changelist] = updatefun(data);
        if changelist.plan_commit || changelist.plan_reject
          done = true;
          break
        end

        % new_X = listener.getNextMessage(0);
        if changelist.plan_con
          new_X = FootstepPlanListener.decodeFootstepPlan(data.plan_con);
          new_X = new_X(:,1);
          new_X(1:6) = biped.stepCenters(new_X(1:6), new_X(15));
          j = find(X(8,:) == new_X(8));
          X(:, j) = new_X(1:14);
          X(7, :) = (0:length(X(1,:))-1) * biped.step_time / 2;
        else
          break
        end
      end
    end
    if done
      break
    end

    [X, outputflag] = updateFootstepPlan(biped, X, @heightfun);
    
    if isequal(size(X_old), size(X)) && all(all(abs(X_old - X) < 0.01))
      modified = false;
    else
      modified = true;
    end
    % if modifed
      % biped.publish_footstep_plan(X);
      outputfun(X, @heightfun);
    % end
    if (~interactive && ~modified) || (done)
      break
    end
  end

  total_steps = length(X(1,:));
  ndx = biped.getStepNdx(total_steps);
  [Xright, Xleft] = biped.stepGoals(X, ndx.right, ndx.left);
  Xright(3,:) = heightfun(Xright(1:2,:));
  Xleft(3,:) = heightfun(Xleft(1:2,:));
end

function h = heightfun(xy)
  h = zeros(1, length(xy(1,:)));
%     h(xy(1,:) > 0.5 & xy(1,:) < 1 & xy(2,:) > -0.25 & xy(2,:) < 0.25) = -0.6;
%     h(xy(1,:) > 0.7 & xy(1,:) < .8 & xy(2,:) > -0.05 & xy(2,:) < 0.05) = 0;
end
