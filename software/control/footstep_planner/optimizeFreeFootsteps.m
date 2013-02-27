function [Xright, Xleft] = optimizeFreeFootsteps(traj, lambda, poses, biped, ndx_r, ndx_l, interactive)

X = traj.eval(lambda(1:end));
fixed_steps = repmat({[]}, length(lambda), 2);

for p = poses
  [~,j] = min(sum((X - repmat(p, 1, length(X(1,:)))).^2));
  if find(ndx_r == j)
    [fixed_steps{j,1},~] = biped.footPositions(X(:,j));
  end
  if find(ndx_l == j)
    [~, fixed_steps{j,2}] = biped.footPositions(X(:,j));
  end
end
% 
% [fixed_steps{1,1}, fixed_steps{1,2}] = biped.footPositions(X(:,1));
% [fixed_steps{end,1}, fixed_steps{end,2}] = biped.footPositions(X(:,end));
X = updateFreeFootsteps(X, biped, fixed_steps);

done = false;
function set_done()
  done = true;
end

if interactive
  h = figure(22);
  set(h, 'WindowButtonDownFcn', @(s, e) mouse_down_handler(h));
  set(h, 'WindowButtonUpFcn', @(s, e) mouse_up_handler(h));
  uicontrol('style', 'pushbutton', 'String', 'Done', 'Callback', @(s, e) set_done());
end

while ~done
  X = updateFreeFootsteps(X, biped, fixed_steps);
  [Xright, Xleft] = biped.footPositions(X);
%   Xright = Xright(:, ndx_r);
%   Xleft = Xleft(:, ndx_l);
  [d_r, r_r] = stepDistance(Xright(:,1:(end-1)), Xright(:,2:end), 0);
  [d_l, r_l] = stepDistance(Xleft(:,1:(end-1)), Xleft(:,2:end), 0);
  for j = 1:length(d_r)
    if (d_r(j) > biped.max_step_length) || ...
       (r_r(j) > biped.max_step_rot) || ...
       (d_l(j) > biped.max_step_length) || ...
       (r_l(j) > biped.max_step_length)
      fixed_steps(j+3:end+2,:) = fixed_steps(j+1:end,:);
      fixed_steps([j+1,j+2],:) = repmat({[]}, 2, 2);
      X(:,j+3:end+2) = X(:,j+1:end);
      X(:,[j+1,j+2]) = interp1([0,1], X(:,[j,j+1])', [1/3, 2/3])';
      num_steps = length(X(1,:));
      ndx_r = int32([1, 2, 4:2:(num_steps-1), num_steps]);
      ndx_l = int32([1:2:(num_steps-1), num_steps]);
      break
    end
  end
  X
  [Xright, Xleft] = biped.footPositions(X, ndx_r, ndx_l);
  figure(22)
  plotFootstepPlan(traj, Xright, Xleft);
  drawnow
end


function mouse_down_handler(hFig)
  ax = gca;
  mouse_pt = get(ax, 'CurrentPoint');
  mouse_x = mouse_pt(1,1);
  mouse_y = mouse_pt(1,2);
  dist_r = sum((Xright(1:2,:) - repmat([mouse_x; mouse_y], 1, length(Xright(1,:)))).^2, 1);
  dist_l = sum((Xleft(1:2,:) - repmat([mouse_x; mouse_y], 1, length(Xleft(1,:)))).^2, 1);
  [min_r, step_r] = min(dist_r);
  [min_l, step_l] = min(dist_l);
  if min_r < min_l
    current_foot = 1;
    ndx = ndx_r(step_r);
    closest_point = Xright(:,step_r);
  else
    current_foot = 2;
    ndx = ndx_l(step_l);
    closest_point = Xleft(:,step_l);
  end
  set(hFig, 'WindowButtonMotionFcn', @(s, e) mouse_drag_handler(hFig, ndx, current_foot, closest_point));
end

function mouse_drag_handler(hFig, ndx, current_foot, closest_point)
  ax = gca;
  mouse_pt = get(ax, 'CurrentPoint')
  mouse_x = mouse_pt(1,1);
  mouse_y = mouse_pt(1,2);
  fixed_steps{ndx, current_foot} = [[mouse_x; mouse_y]; closest_point(3:end)];
end

function mouse_up_handler(hFig)
  set(hFig, 'WindowButtonMotionFcn', '');
end

end
