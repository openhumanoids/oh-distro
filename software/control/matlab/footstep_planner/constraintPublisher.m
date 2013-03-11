classdef constraintPublisher
  properties
    hFig
    constraints
    active_ndx
    plan_listener
    lc
    plan
  end
  
  methods
    function obj = constraintPublisher()
      obj.constraints = {};
      obj.hFig = figure(23);
      set(obj.hFig, 'WindowButtonUpFcn', @(s, e) obj.mouse_up_handler());
      obj.plan_listener = FootstepPlanListener('atlas', 'CANDIDATE_FOOTSTEP_PLAN');
      obj.lc = lcm.lcm.LCM.getSingleton();
      disp('initialized')
    end
    
    function run(obj)
      disp('running')
      while 1
        new_plan = obj.plan_listener.getNextMessage(1);
        if ~isempty(new_plan)
          obj.plan = new_plan;
          set(obj.hFig, 'WindowButtonDownFcn', @(s, e) obj.mouse_down_handler());
          Xright = new_plan(3:8,find(new_plan(1,:)));
          Xleft = new_plan(3:8,find(~new_plan(1,:)));
%           obj.plan.right = Xright;
%           obj.plan.left = Xleft;
          figure(obj.hFig)
          plotFootstepPlan([], Xright, Xleft);
          drawnow
        end
      end
      
    end
    
    function mouse_down_handler(obj)
      ax = gca;
      mouse_pt = get(ax, 'CurrentPoint');
      mouse_x = mouse_pt(1,1);
      mouse_y = mouse_pt(1,2);
      dist = sum((obj.plan(3:4,:) - repmat([mouse_x; mouse_y], 1, length(obj.plan(1,:)))).^2,1);
      [~, step] = min(dist)
      closest_point = obj.plan(:, step)
      set(obj.hFig, 'WindowButtonMotionFcn', @(s, e) obj.mouse_drag_handler(step));
      
%       dist_r = sum((obj.plan.right(1:2,:) - repmat([mouse_x; mouse_y], 1, length(obj.plan.right(1,:)))).^2, 1);
%       dist_l = sum((obj.plan.left(1:2,:) - repmat([mouse_x; mouse_y], 1, length(obj.plan.left(1,:)))).^2, 1);
%       [min_r, step_r] = min(dist_r);
%       [min_l, step_l] = min(dist_l);
%       if min_r < min_l
%         current_foot = 1;
%         drag_ndx = ndx_r(step_r);
%         closest_point = obj.plan.right(:,step_r);
%       else
%         current_foot = 2;
%         drag_ndx = ndx_l(step_l);
%         closest_point = obj.plan.left(:,step_l);
%       end

%       set(hFig, 'WindowButtonMotionFcn', @(s, e) mouse_drag_handler(hFig, current_foot, closest_point));
    end

    function mouse_drag_handler(obj, step_ndx)
      ax = gca;
      mouse_pt = get(ax, 'CurrentPoint');
      mouse_x = mouse_pt(1,1);
      mouse_y = mouse_pt(1,2);
      obj.plan(3:4, step_ndx) = [mouse_x; mouse_y];
      obj.publish_constraint(obj.plan(:,step_ndx));
%       fixed_steps{drag_ndx, current_foot} = [[mouse_x; mouse_y]; closest_point(3:6)];
    end

    function mouse_up_handler(obj)
      set(obj.hFig, 'WindowButtonMotionFcn', '');
    end
    
    function publish_constraint(obj, X);
      msg = drc.traj_opt_constraint_t();
      msg.utime = 0;
      msg.robot_name = 'atlas';
      msg.num_links = 1;
      X
      if X(1)
        msg.link_name = {'r_foot'};
        disp('r_foot')
      else
        msg.link_name = {'l_foot'};
        disp('l_foot')
      end
      pos = drc.position_3d_t();
      trans = drc.vector_3d_t();
      trans.x = X(3);
      trans.y = X(4);
      trans.z = X(5);
      rot = drc.quaternion_t();
      q = angle2quat(X(6), X(7), X(8));
      rot.x = q(1);
      rot.y = q(2);
      rot.z = q(3);
      rot.w = q(4);
      pos.translation = trans;
      pos.rotation = rot;

      % Ugly hack, because we can't make a vector of length 1
      msg.link_origin_position = [pos, pos]; 
      msg.link_origin_position(2) = [];
      % end hack

      msg.link_timestamps = [0.1];

      obj.lc.publish('TRAJ_OPT_CONSTRAINT', msg);
    end
  end
end

