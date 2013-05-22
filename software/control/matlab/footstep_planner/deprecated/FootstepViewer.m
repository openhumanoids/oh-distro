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
      % nav_goal_coder = NavGoalCoder('atlas');
      % goal_msg = nav_goal_coder.encode([1;1;0;0;0;0]);
      % obj.lc.publish('NAV_GOAL_TIMED', goal_msg);

      % r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);
      % r = removeCollisionGroupsExcept(r,{'heel','toe'});
      % r = compile(r);      
      % d = load('../data/atlas_fp.mat');
      % xstar = d.xstar;
      % r = r.setInitialState(xstar);
      % state_coder = 

      set(obj.hFig, 'WindowButtonUpFcn', @(s, e) mouse_up_handler());
      set(obj.hFig, 'WindowButtonDownFcn', @(s, e) mouse_down_handler());
      uicontrol('style', 'pushbutton', 'String', 'Done', 'Callback', @(s, e) confirm_plan());
      disp('running')
      selected_id = 0;
      while 1
        new_plan = obj.listener.getNextMessage(50);
        if ~isempty(new_plan)
          plan = new_plan;
          Xpos = [plan.pos]
          Xright = Xpos(:, [plan.is_right_foot] == 1);
          Xleft = Xpos(:, [plan.is_right_foot] == 0);
          sfigure(obj.hFig);
          plotFootstepPlan([], Xright, Xleft);
          hold on
          for j = 1:length(plan)
            if any(plan(j).pos_fixed)
              if plan(j).is_right_foot
                plot(plan(j).pos(1), plan(j).pos(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g')
              else
                plot(plan(j).pos(1), plan(j).pos(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r')
              end
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
        dist = sum((Xpos(1:2,:) - repmat([mouse_x; mouse_y], 1, length(plan))).^2,1);
        [~, step_ndx] = min(dist);
        selected_id = plan(step_ndx).id;
        set(obj.hFig, 'WindowButtonMotionFcn', @(s, e) mouse_drag_handler(selected_id));
      end
        

      function mouse_drag_handler(selected_id)
        ax = gca;
        mouse_pt = get(ax, 'CurrentPoint');
        mouse_x = mouse_pt(1,1);
        mouse_y = mouse_pt(1,2);
        step_ndx = find([plan.id] == selected_id);
        step = plan(step_ndx);
        step.pos(1:2) = [mouse_x; mouse_y];
        step.pos_fixed(1:3) = 1;
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

