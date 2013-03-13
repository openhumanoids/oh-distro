function [Xright, Xleft] = optimizeFootstepPlan(biped, poses, interactive)

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
X(8, :) = int32((0:length(X(1,:))-1) / (length(X(1,:)) - 1)*1000000);
X(9:14,:) = zeros(6, length(X(1,:)));

total_steps = length(X(1,:));

ndx = biped.getStepNdx(total_steps);
% ndx.right = int32([1, 2, 4:2:(total_steps-1), total_steps]);
% ndx.left = int32([1:2:(total_steps-1), total_steps]);
for p = poses
  [~,j] = min(sum((X(1:6,:) - repmat(p, 1, length(X(1,:)))).^2));
  X(9:14, j) = 1;
end
[X, outputflag] = updateFootstepPlan(biped, X, ndx.right, ndx.left, @heightfun);


done = false;
function set_done()
  done = true;
end

listener = FootstepPlanListener('FOOTSTEP_PLAN_CONSTRAINT')
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('COMMITTED_FOOTSTEP_PLAN', aggregator);
lc.subscribe('REJECTED_FOOTSTEP_PLAN', aggregator);


while 1
  X_old = X;
  [Xright, Xleft] = biped.stepLocations(X);
  while 1
    con_msg = aggregator.getNextMessage(0);
    if ~isempty(con_msg)
      [Xright, Xleft] = biped.stepGoals(X, ndx.right, ndx.left);
      done = true;
      break
    end

    new_X = listener.getNextMessage(0);
    if ~isempty(new_X)
      new_X(1:6) = biped.stepCenters(new_X(1:6), new_X(15));
      j = find(X(8,:) == new_X(8));
      X(:, j) = new_X(1:14);
    else
      break
    end
  end
  if done
    break
  end

  ndx_fixed = find(any(X(9:14,:)));

  [d_r, r_r] = biped.stepDistance(Xright(1:6,1:(end-1)), Xright(1:6,2:end), 0);
  [d_l, r_l] = biped.stepDistance(Xleft(1:6,1:(end-1)), Xleft(1:6,2:end), 0);
  for n = 1:(length(ndx_fixed)-1)
    num_steps = ndx_fixed(n+1) - ndx_fixed(n);
    dist = max(sum(d_r(ndx_fixed(n):(ndx_fixed(n+1)-1))),...
               sum(d_l(ndx_fixed(n):(ndx_fixed(n+1)-1))));
    rot = max(sum(r_r(ndx_fixed(n):(ndx_fixed(n+1)-1))),...
              sum(r_l(ndx_fixed(n):(ndx_fixed(n+1)-1))));
    if  ((dist > num_steps * biped.max_step_length * .4 ...
          || rot > num_steps * biped.max_step_rot * .4) ...
        && (num_steps > 1 || n == length(ndx_fixed)-1 || n == 1))
      j = ndx_fixed(n);
      X(:,j+3:end+2) = X(:,j+1:end);
      X(:,[j+1,j+2]) = interp1([0,1], X(:,[j,j+1])', [1/3, 2/3])';
      X(8,:) = int32(X(8,:));
      X(7, :) = (0:length(X(1,:))-1) * biped.step_time / 2;
      X(9:14,[j+1,j+2]) = 0;
      break
    elseif (dist < num_steps * biped.max_step_length * 0.15 ...
            && rot < num_steps * biped.max_step_rot * 0.15) ...
        && (num_steps > 2)
      j = ndx_fixed(n);
      X(:,j+1:end-2) = X(:,j+3:end);
      X(:,end-1:end) = [];
      break
    end
  end
  total_steps = length(X(1,:));
  ndx = biped.getStepNdx(total_steps);
  
  [X, outputflag] = updateFootstepPlan(biped, X, ndx.right, ndx.left, @heightfun);
  
  [Xright, Xleft] = biped.stepGoals(X, ndx.right, ndx.left);
  if isequal(size(X_old), size(X)) && all(all(abs(X_old - X) < 0.01))
    modified = false;
  else
    modified = true;
  end
  % if modifed
    biped.publish_footstep_plan(X);
  % end
  if (~interactive && ~modified) || (done)
    break
  end
end

end

  function h = heightfun(xy)
    h = zeros(1, length(xy(1,:)));
%     h(xy(1,:) > 0.5 & xy(1,:) < 1 & xy(2,:) > -0.25 & xy(2,:) < 0.25) = -0.6;
%     h(xy(1,:) > 0.7 & xy(1,:) < .8 & xy(2,:) > -0.05 & xy(2,:) < 0.05) = 0;
  end
