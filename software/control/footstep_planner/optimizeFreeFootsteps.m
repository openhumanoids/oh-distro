function [Xright, Xleft] = optimizeFreeFootsteps(traj, lambda, biped, ndx_r, ndx_l)

fixed_steps = repmat({[]}, length(lambda), 2);
% fixed_steps{2,1} = [.3;-.1;0;0;0;0];

h = figure(22);
set(h, 'WindowButtonDownFcn', @(s, e) mouse_down_handler(h));
set(h, 'WindowButtonUpFcn', @(s, e) mouse_up_handler(h));done = false;
hButton = uicontrol('style', 'pushbutton', 'String', 'Done', 'Callback', @(s, e) set_done());
function set_done()
  done = true;
end

X = traj.eval(lambda(1:end));
[Xright, Xleft] = biped.footPositions(X, ndx_r, ndx_l);

while ~done
  [X, Xright, Xleft] = optimizeFreeFootsteps(X, biped, ndx_r, ndx_l, fixed_steps);
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
