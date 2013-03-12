function [Xright, Xleft] = optimizeFreeFootsteps(biped, poses, interactive)

X = interp1([1:length(poses(1,:))]', poses', [1:0.5:length(poses(1,:))]')';
total_steps = length(X(1,:));

fixed_steps = repmat({[]}, total_steps, 2);

ndx_r = int32([1, 2, 4:2:(total_steps-1), total_steps]);
ndx_l = int32([1:2:(total_steps-1), total_steps]);
for p = poses
  [~,j] = min(sum((X - repmat(p, 1, length(X(1,:)))).^2));
  if find(ndx_r == j)
    [fixed_steps{j,1},~] = biped.stepLocations(X(:,j));
  end
  if find(ndx_l == j)
    [~, fixed_steps{j,2}] = biped.stepLocations(X(:,j));
  end
end
[X, outputflag] = updateFastFootsteps(biped, X, fixed_steps, ndx_r, ndx_l, @heightfun);


done = false;
function set_done()
  done = true;
end

h = figure(22);
set(h, 'WindowButtonDownFcn', @(s, e) mouse_down_handler(h));
set(h, 'WindowButtonUpFcn', @(s, e) mouse_up_handler(h));
uicontrol('style', 'pushbutton', 'String', 'Done', 'Callback', @(s, e) set_done());

drag_ndx = 1;

plan_publisher = FootstepPlanPublisher('atlas', 'r_foot','l_foot', 'CANDIDATE_FOOTSTEP_PLAN');
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('TRAJ_OPT_CONSTRAINT', aggregator);


while 1
  X_old = X;
  [Xright, Xleft] = biped.stampedStepLocations(X);
  con_msg = aggregator.getNextMessage(0);
  if ~isempty(con_msg)
    con_data = drc.traj_opt_constraint_t(con_msg.data);
    origin_pos = con_data.link_origin_position(1);
    [r p y] = quat2angle([origin_pos.rotation.x,...
                             origin_pos.rotation.y,...
                             origin_pos.rotation.z,...
                             origin_pos.rotation.w]);
    pos = [origin_pos.translation.x;...untitled.fig
           origin_pos.translation.y;...
           origin_pos.translation.z;...
           r;p;y];
    con_data.link_name(1)
    if strcmp(con_data.link_name(1), biped.r_foot_name)
      current_foot = 1;
      dist = sum((Xright(1:3,ndx_r) - repmat(pos(1:3), 1, length(ndx_r))).^2,1);
      [~, step_ndx] = min(dist);
      fixed_steps{ndx_r(step_ndx), current_foot} = pos;
    else
      current_foot = 2;
      dist = sum((Xleft(1:3,ndx_l) - repmat(pos(1:3), 1, length(ndx_l))).^2,1);
      [~, step_ndx] = min(dist);
      fixed_steps{ndx_l(step_ndx), current_foot} = pos;
    end
    fixed_steps
  end
  
  % costs = zeros(1, length(X(1,:))-1);
  % for j = 1:length(X(1,:))-1
  %   costs(j) = biped.stepCost(X(:,j:j+1));
  % end
  % [max_cost, j_max] = max(costs);
  % [min_cost, j_min] = min(costs);
  %   if max_cost > 2 * biped.max_step_length^2 && (outputflag == 1 || outputflag == -2 || outputflag == 2)
  %     j = j_max;
  %     fixed_steps(j+3:end+2,:) = fixed_steps(j+1:end,:);
  %     fixed_steps([j+1,j+2],:) = repmat({[]}, 2, 2);
  %     X(:,j+3:end+2) = X(:,j+1:end);
  %     X(:,[j+1,j+2]) = interp1([0,1], X(:,[j,j+1])', [1/3, 2/3])';
  %     if drag_ndx > j
  %       drag_ndx = drag_ndx + 2;
  %     end
  %     % break
  %   elseif (min_cost < .2 * biped.max_step_length^2) && j_min < length(X(1,:)) - 2 && all(all(cellfun(@isempty, fixed_steps(j_min+1:j_min+2,:))))
  %     j = j_min;
  %     fixed_steps(j+1:end-2,:) = fixed_steps(j+3:end,:);
  %     fixed_steps(end-1:end,:) = [];
  %     X(:,j+1:end-2) = X(:,j+3:end);
  %     X(:,end-1:end) = [];
  %     if drag_ndx > j+1
  %       drag_ndx = drag_ndx - 2;
  %     end
  %     % break
  %   end
  % % end


  ndx_fixed = find(any(cellfun(@(x) ~isempty(x),fixed_steps),2));

  [d_r, r_r] = biped.stepDistance(Xright(1:6,1:(end-1)), Xright(1:6,2:end), 0);
  [d_l, r_l] = biped.stepDistance(Xleft(1:6,1:(end-1)), Xleft(1:6,2:end), 0);
  for n = 1:(length(ndx_fixed)-1)
    num_steps = ndx_fixed(n+1) - ndx_fixed(n);
    dist = max(sum(d_r(ndx_fixed(n):(ndx_fixed(n+1)-1))),...
               sum(d_l(ndx_fixed(n):(ndx_fixed(n+1)-1))));
    rot = max(sum(r_r(ndx_fixed(n):(ndx_fixed(n+1)-1))),...
              sum(r_l(ndx_fixed(n):(ndx_fixed(n+1)-1))));
    if  ((dist > num_steps * biped.max_step_length * .4 ...
          || rot > num_steps * biped.max_step_rot * .4))
      j = ndx_fixed(n);
      fixed_steps(j+3:end+2,:) = fixed_steps(j+1:end,:);
      fixed_steps([j+1,j+2],:) = repmat({[]}, 2, 2);
      X(:,j+3:end+2) = X(:,j+1:end);
      X(:,[j+1,j+2]) = interp1([0,1], X(:,[j,j+1])', [1/3, 2/3])';
      if drag_ndx > j
        drag_ndx = drag_ndx + 2;
      end
      break
    elseif (dist < num_steps * biped.max_step_length * 0.15 ...
            && rot < num_steps * biped.max_step_rot * 0.15) ...
        && (num_steps > 2)
      j = ndx_fixed(n);
      fixed_steps(j+1:end-2,:) = fixed_steps(j+3:end,:);
      fixed_steps(end-1:end,:) = [];
      X(:,j+1:end-2) = X(:,j+3:end);
      X(:,end-1:end) = [];
      if drag_ndx > j+1
        drag_ndx = drag_ndx - 2;
      end
      break
    end
  end
  total_steps = length(X(1,:));
  ndx_r = int32([1, 2, 4:2:(total_steps-1), total_steps]);
  ndx_l = int32([1:2:(total_steps-1), total_steps]);
  
  [X, outputflag] = updateFastFootsteps(biped, X, fixed_steps, ndx_r, ndx_l, @heightfun);
  
  [Xright, Xleft] = biped.stampedStepLocations(X, ndx_r, ndx_l);
  if isequal(size(X_old), size(X)) && all(all(abs(X_old - X) < 0.01))
    modified = false;
  else
    modified = true;
  end
  if modified
    figure(22)
    hold off
    plotFootstepPlan(X, Xright, Xleft);
    hold on
    plan_publisher.publish(Xleft(7,:), Xleft(1:6,:), Xright(7,:), Xright(1:6,:));
    for j = 1:length(ndx_r)
      if ~isempty(fixed_steps{ndx_r(j), 1})
        plot(Xright(1,j), Xright(2,j), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g')
      end
    end
    for j = 1:length(ndx_l)
      if~isempty(fixed_steps{ndx_l(j), 2})
        plot(Xleft(1,j), Xleft(2,j), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r')
      end
    end
  end
  drawnow
  if (~interactive && ~modified) || (done)
    break
  end
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
    drag_ndx = ndx_r(step_r);
    closest_point = Xright(:,step_r);
  else
    current_foot = 2;
    drag_ndx = ndx_l(step_l);
    closest_point = Xleft(:,step_l);
  end
  
  set(hFig, 'WindowButtonMotionFcn', @(s, e) mouse_drag_handler(hFig, current_foot, closest_point));
end

function mouse_drag_handler(hFig, current_foot, closest_point)
  ax = gca;
  mouse_pt = get(ax, 'CurrentPoint');
  mouse_x = mouse_pt(1,1);
  mouse_y = mouse_pt(1,2);
  fixed_steps{drag_ndx, current_foot} = [[mouse_x; mouse_y]; closest_point(3:6)];
end

function mouse_up_handler(hFig)
  set(hFig, 'WindowButtonMotionFcn', '');
end

  function h = heightfun(xy)
    h = zeros(1, length(xy(1,:)));
%     h(xy(1,:) > 0.5 & xy(1,:) < 1 & xy(2,:) > -0.25 & xy(2,:) < 0.25) = -0.6;
%     h(xy(1,:) > 0.7 & xy(1,:) < .8 & xy(2,:) > -0.05 & xy(2,:) < 0.05) = 0;
  end

end
