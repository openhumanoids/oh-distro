classdef FootstepViewer
  properties
    hFig
    constraints
    active_ndx
    listener
    publisher
    lc
  end
  
% Plan: [x; y; z; roll; pitch; yaw; time; id; fixed_x;
%        fixed_y; fixed_z; fixed_roll; fixed_pitch; fixed_yaw; is_right_foot]

  methods
    function obj = FootstepViewer()
      obj.constraints = {};
      obj.hFig = figure(23);
      drawnow
      obj.listener = FootstepPlanListener('CANDIDATE_FOOTSTEP_PLAN');
      obj.publisher = FootstepPlanPublisher('FOOTSTEP_PLAN_CONSTRAINT');
      obj.lc = lcm.lcm.LCM.getSingleton();
      disp('initialized')
    end
    
    function run(obj)
      set(obj.hFig, 'WindowButtonUpFcn', @(s, e) mouse_up_handler());
      set(obj.hFig, 'WindowButtonDownFcn', @(s, e) mouse_down_handler());
      uicontrol('style', 'pushbutton', 'String', 'Done', 'Callback', @(s, e) confirm_plan());
      disp('running')
      selected_id = 0;
      while 1
        new_plan = obj.listener.getNextMessage(0);
        if ~isempty(new_plan)
          plan = new_plan;
          Xright = plan(:, plan(15,:)==1);
          Xleft = plan(:, plan(15,:)==0);
          sfigure(obj.hFig);
          plotFootstepPlan([], Xright, Xleft);
          hold on
          for j = 1:size(Xright, 2)
            if any(Xright(9:14,j))
              plot(Xright(1,j), Xright(2,j), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g')
            end
          end
          for j = 1:size(Xleft, 2)
            if any(Xleft(9:14,j))
              plot(Xleft(1,j), Xleft(2,j), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r')
            end
          end
          hold off
        end
        drawnow
      end
      function mouse_down_handler()
        ax = gca;
        mouse_pt = get(ax, 'CurrentPoint');
        mouse_x = mouse_pt(1,1);
        mouse_y = mouse_pt(1,2);
        dist = sum((plan(1:2,:) - repmat([mouse_x; mouse_y], 1, length(plan(1,:)))).^2,1);
        [~, step_ndx] = min(dist);
        selected_id = plan(8, step_ndx);
        set(obj.hFig, 'WindowButtonMotionFcn', @(s, e) mouse_drag_handler(selected_id));
      end
        

      function mouse_drag_handler(selected_id)
        ax = gca;
        mouse_pt = get(ax, 'CurrentPoint');
        mouse_x = mouse_pt(1,1);
        mouse_y = mouse_pt(1,2);
        step_ndx = find(plan(8,:) == selected_id);
        step = plan(:, step_ndx);
        step(1:2) = [mouse_x; mouse_y];
        step(9:10) = 1;
        obj.publisher.publish(step);
  %       fixed_steps{drag_ndx, current_foot} = [[mouse_x; mouse_y]; closest_point(3:6)];
      end

      function mouse_up_handler()
        set(obj.hFig, 'WindowButtonMotionFcn', '');
      end

      function confirm_plan()
        obj.lc.publish('COMMITTED_FOOTSTEP_PLAN', FootstepPlanPublisher.encodeFootstepPlan(plan));
      end

      
    end
  end
end

